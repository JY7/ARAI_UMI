#!/usr/bin/env python3
"""UMI data collection. See README.

  python3 collect.py [--hands right] [--no-viz]
  python3 collect.py --list-vive | --calib-dxl | --calib-mount | --test-button
"""
import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from contextlib import suppress

from umi.align import calibrate_origin_live, tcp_in_origin
from umi.button import Buttons, test_buttons
from umi.camera import CameraSource
from umi.dxl import DxlSource, calibrate, fraction
from umi.episode import Recorder, data_path, load_current_config, origin_age_s, save_origin
from umi.gripper import run_mount
from umi.steamvr_log import FrameWatch
from umi.vive import ViveSource

ALARM_AFTER_S = 0.5    # s, problem duration before a voice alarm while recording
ALARM_REPEAT_S = 3.0   # s


SPEECH_MAX_PENDING = 3


class Speaker(threading.Thread):
    """espeak one message at a time so alerts don't overlap."""

    def __init__(self):
        super().__init__(daemon=True)
        self.pending = []
        self.cv = threading.Condition()
        self.start()

    def say(self, msg, important=True):
        with self.cv:
            if any(m == msg for m, _ in self.pending):
                return
            if important:
                i = next((k for k, (_, imp) in enumerate(self.pending) if not imp), len(self.pending))
                self.pending.insert(i, (msg, True))
            else:
                self.pending.append((msg, False))
                status = [k for k, (_, imp) in enumerate(self.pending) if not imp]
                for k in reversed(status[:-SPEECH_MAX_PENDING]):
                    del self.pending[k]
            self.cv.notify()

    def run(self):
        while True:
            with self.cv:
                while not self.pending:
                    self.cv.wait()
                msg = self.pending.pop(0)[0]
            with suppress(OSError, subprocess.SubprocessError):
                subprocess.run(["espeak", msg], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)


_speaker = None


def beep(msg, important=True):
    global _speaker
    if _speaker is None:
        _speaker = Speaker()
    _speaker.say(msg, important)


def apply_hands(cfg, hands=None):
    hands = list(hands or cfg.get("hands") or ["left", "right"])
    bad = [h for h in hands if h not in ("left", "right")]
    if bad:
        sys.exit("hands must be left/right: %s" % bad)
    hands = [h for h in ("left", "right") if h in hands]   # fixed order: state column order, dataset compatibility
    cfg["hands"] = hands
    for d, k in ((cfg["vive"], "serials"), (cfg["dynamixel"], "hands"), (cfg["button"]["bluetooth"], "devices"), (cfg, "cameras")):
        d[k] = {h: v for h, v in d[k].items() if h in hands}
    return cfg


USE_COLOR = sys.stdout.isatty() and not os.environ.get("NO_COLOR")
RED, GREEN, YELLOW, CYAN, BOLD = "31", "32", "33", "36", "1"
ANSI_RE = re.compile(r"(\x1b\[[0-9;]*m)")


def color(text, *codes):
    return "\x1b[%sm%s\x1b[0m" % (";".join(codes), text) if USE_COLOR and codes else text


TAG_COLORS = {"saved": (GREEN,), "stopped": (YELLOW,), "OK": (GREEN,), "recording": (RED, BOLD), "ERR": (RED,), "warn": (YELLOW,),
              "discarded": (YELLOW,), "discarded current": (YELLOW,), "origin": (CYAN,)}


def say(msg=""):
    m = re.match(r"\s*\[([^\]]+)\]", msg)
    if m:
        tag = m.group(1)
        codes = ((RED,) if ("fail" in msg or "abort" in msg) else (YELLOW,)) if tag == "mp4" else TAG_COLORS.get(tag, ())
        lead = msg[:len(msg) - len(msg.lstrip("\n"))]
        msg = lead + color(msg[len(lead):], *codes)
    print(msg)


def stopped_line(s, vive, dxl):
    """[stopped] if cameras (encoder prints [saved] after mp4), else [saved]."""
    c = s["counts"]
    issues = ["cam %s dropped %d" % (k, v) for k, v in s["dropped_frames"].items() if v]
    issues += ["cam %s corrupt %d" % (k, v) for k, v in s["corrupt_frames"].items() if v]
    if vive.is_alive() and not c.get("tracker_rows"):
        issues.append("no tracker data")
    if dxl.is_alive() and not c.get("gripper_rows"):
        issues.append("no gripper data")
    tag = "stopped" if any(k.startswith("cam_") for k in c) else "saved"
    line = "[%s] %s  %.1fs" % (tag, s["name"], s["duration_s"])
    return line + ("  ⚠ " + ", ".join(issues) if issues else "")


def fit(line, width):
    """Cut/pad to terminal width so the \\r redraw works; ASCII text, ANSI codes take 0 columns."""
    out, w = [], 0
    for k, part in enumerate(ANSI_RE.split(line)):   # odd parts = ANSI codes
        out.append(part if k % 2 else part[:width - w])
        w += 0 if k % 2 else len(out[-1])
    return "".join(out) + ("\x1b[0m" if USE_COLOR else "") + " " * (width - w)


class StatusLine:
    """Live status line: TCP pos in cm (origin frame), gripper 0 open .. 100% closed."""

    def __init__(self, cfg, rec, vive, dxl, cams, buttons):
        self.cfg, self.rec, self.vive, self.dxl, self.cams, self.buttons = cfg, rec, vive, dxl, cams, buttons
        self.hands = list(cfg["hands"])
        self.dxl_hands = cfg["dynamixel"]["hands"]
        self.vive_hz = float(cfg["vive"].get("rate_hz", 200))
        self.dxl_hz = float(cfg["dynamixel"].get("rate_hz", 100))

    def _hand(self, h):
        L = h[0].upper()
        vive, dxl = self.vive, self.dxl
        if vive.is_alive() and vive.valid.get(h) and h in vive.latest_T:
            p = tcp_in_origin(self.cfg, self.rec.origin, h, vive.latest_T[h])[:3, 3] * 100
            pos = " x%+6.1f y%+6.1f z%+6.1f" % tuple(p)
        else:
            pos = " " + color("%-23s" % "tracker lost", RED)
        if dxl.is_alive() and h in dxl.latest:
            grip = " grip %3.0f%%" % (100 * fraction(self.dxl_hands.get(h, {}), "gripper", dxl.latest[h]["pos"]))
        else:
            grip = " grip " + color(" --", RED)
        return color(L, BOLD) + pos + grip

    def __call__(self):
        rec = self.rec
        parts = [color("REC %5.1fs" % ((time.monotonic_ns() - rec.episode.t_start_ns) * 1e-9), RED, BOLD)
                 if rec.recording else "idle      "]
        parts += [self._hand(h) for h in self.hands]
        if rec.origin:
            age = origin_age_s(rec.origin)
            parts.append("origin " + color("%dm" % (age // 60) if age < 3600 else "%.1fh" % (age / 3600), GREEN))
        else:
            parts.append(color("no origin", YELLOW))
        bad = []
        if not self.vive.is_alive():
            bad.append("tracker OFF")
        elif self.vive.rate < 0.8 * self.vive_hz:
            bad.append("tracker %.0fHz" % self.vive.rate)
        if not self.dxl.is_alive():
            bad.append("gripper OFF")
        elif self.dxl.error:
            bad.append("gripper ERR")
        elif self.dxl.rate < 0.8 * self.dxl_hz:
            bad.append("gripper %.0fHz" % self.dxl.rate)
        for c in self.cams:
            if not c.is_alive() or c.error:
                bad.append("cam %s lost" % c.name)
            elif c.rate < 0.9 * c.fps:
                bad.append("cam %s %.0ffps" % (c.name, c.rate))
            if c.corrupt:
                bad.append("cam %s corrupt %d" % (c.name, c.corrupt))
        bad += ["button %s lost" % b.hand for b in self.buttons.bt if not b.connected]
        if bad:
            parts.append(color(" ".join(bad), RED))
        if rec.encoder.pending:
            parts.append(color("mp4 encoding %d" % rec.encoder.pending, YELLOW))
        return " | ".join(parts)


class Notifier:
    def __init__(self):
        self.last = {}

    def changes(self, items):
        out = []
        for k, msg in items.items():
            prev = self.last.get(k)
            if msg and msg != prev:
                out.append("[ERR] %s: %s" % (k, msg))
            elif not msg and prev:
                out.append("[OK] %s recovered" % k)
            self.last[k] = msg
        return out


def sensor_problems(vive, dxl, cams, hands):
    p = {}
    if not vive.is_alive():
        p["tracker stopped"] = vive.error or "vive stopped"
    else:
        if vive.error:
            p["tracker error"] = vive.error
        for h in hands:
            if not vive.valid.get(h, False):
                p["%s tracker lost" % h] = "tracker %s lost" % h
    if not dxl.is_alive():
        p["gripper stopped"] = dxl.error or "dxl stopped"
    elif dxl.error:
        p["gripper error"] = dxl.error
    for c in cams:
        if not c.is_alive():
            p["%s camera stopped" % c.name] = "%s: %s" % (c.label, c.error or "stopped")
        elif c.error:
            p["%s camera error" % c.name] = "%s: %s" % (c.label, c.error)
        elif c.latest is None:
            p["%s camera not ready" % c.name] = "%s: no frame yet" % c.label
    return p


class Alarm:
    """Voice alarm while recording (operator isn't watching the screen)."""

    def __init__(self):
        self.since, self.last = {}, {}

    def reset(self):
        self.since.clear()
        self.last.clear()

    def due(self, problems, now):
        for k in [k for k in self.since if k not in problems]:
            del self.since[k]
            self.last.pop(k, None)
        out = []
        for k, msg in problems.items():
            t0 = self.since.setdefault(k, now)
            if now - t0 >= ALARM_AFTER_S and now - self.last.get(k, -1e9) >= ALARM_REPEAT_S:
                first = k not in self.last
                self.last[k] = now
                out.append((k, msg, first))
        return out


class Reconnect:
    def __init__(self):
        self.since = {}
        self.all_ok = False

    def due(self, problems, now):
        back = [k for k, t0 in self.since.items() if k not in problems and now - t0 >= ALARM_AFTER_S]
        for k in [k for k in self.since if k not in problems]:
            del self.since[k]
        for k in problems:
            self.since.setdefault(k, now)
        if not problems and (back or not self.all_ok):
            self.all_ok = True
            return "all connected"
        if back:
            return ", ".join(k.rsplit(" ", 1)[0] for k in back) + " connected"
        return None


def drop_stale_origin(rec, t_change):
    """t_change: epoch s. True if the origin was dropped."""
    if not rec.origin or t_change <= time.time() - origin_age_s(rec.origin):
        return False
    rec.origin = None
    return True


def _interrupt(signum, frame):
    raise KeyboardInterrupt   # so terminal close/kill still saves the episode and releases gripper torque


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list-vive", action="store_true")
    ap.add_argument("--calib-dxl", action="store_true")
    ap.add_argument("--test-button", action="store_true")
    ap.add_argument("--viz", action=argparse.BooleanOptionalAction, default=True,
                    help="live viewer (Rerun): 3D gripper + cameras + gripper plot (default on, --no-viz to turn off)")
    ap.add_argument("--calib-mount", action="store_true",
                    help="calibrate tracker mount orientation (two poses) -> saved to config.yaml")
    ap.add_argument("--data", default=None,
                    help="subfolder under data/ for episodes, e.g. --data cup -> data/cup (absolute path used as is). "
                         "Numbering restarts at ep000 in a new folder")
    ap.add_argument("--hands", nargs="+", choices=["left", "right"],
                    help="hands to use (overrides config hands), e.g. --hands right")
    args = ap.parse_args()

    cfg = apply_hands(load_current_config(), args.hands)

    if args.list_vive:
        for i, cls, serial in ViveSource.list_devices():
            print("%2d %-12s %s" % (i, cls, serial))
        return
    if args.calib_mount:
        vive = ViveSource(cfg["vive"])
        vive.start()
        time.sleep(1.0)
        if vive.error:
            print("vive: %s" % vive.error)
        try:
            run_mount(cfg, vive, cfg["hands"], beep)
        finally:
            vive.stop()
        return
    if args.test_button:
        test_buttons()
        return
    if args.calib_dxl:
        calibrate(cfg["dynamixel"], beep)
        return

    unmeasured = [h for h, d in cfg["dynamixel"]["hands"].items()
                  if None in [d.get(k) for k in ("trigger_open", "trigger_closed", "gripper_open", "gripper_closed")]]
    if unmeasured:   # alignment uses recording-time values, so filling them in later won't fix recorded episodes
        sys.exit("dynamixel.hands.%s: open/closed raw values not measured (null). Run `python3 collect.py --calib-dxl --hands %s`, "
                 "or collect with --hands %s" % (unmeasured, " ".join(unmeasured), " ".join(h for h in cfg["hands"] if h not in unmeasured)))
    rec = Recorder(cfg, data_path(args.data or "", must_exist=False))
    vive = ViveSource(cfg["vive"], rec)
    frames = FrameWatch()   # SteamVR frame change -> origin invalid
    dxl = DxlSource(cfg["dynamixel"], rec)
    cams = [CameraSource(n, c, rec) for n, c in cfg["cameras"].items()]
    sources = [vive, dxl] + cams
    # fallible setup before starting sensors (a failure after would leave gripper torque on)
    live = None
    if args.viz:
        from umi.viz import LiveViz
        live = LiveViz(cfg, vive, dxl, cams, rec)
    buttons = Buttons(cfg["button"], lambda: rec.recording)
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _interrupt)

    stopped = set()
    alarm = Alarm()
    reconnect = Reconnect()
    notifier = Notifier()
    status = StatusLine(cfg, rec, vive, dxl, cams, buttons)
    try:
        for s in sources:
            s.start()
        if live is not None:
            live.start()
        print("data: %s | task: %s" % (rec.root, rec.task))
        print("keys: space start/stop | d discard last | t change task | o set origin | q quit")
        if frames.last and drop_stale_origin(rec, frames.last[0]):
            say("[origin] SteamVR coordinates changed after the origin was set (%s) -> not used. Set the origin again."
                % time.strftime("%H:%M:%S", time.localtime(frames.last[0])))
            beep("tracking changed, set origin again")
        print("origin: %s" % (rec.origin["time"] if rec.origin else "none - stand gripper(s) at the table edge, then 'o' (two hands: symmetric about the edge midpoint, one hand: at the midpoint)"))
        mount = (cfg.get("gripper_model") or {}).get("tracker_frame_rpy_deg") or {}
        uncal = [h for h in cfg["hands"] if mount.get(h) is None]
        if uncal:
            say("[warn] tracker mount not calibrated: %s -> run `python3 collect.py --calib-mount` (can be done later, applied by check_episode)" % uncal)
        while True:
            cmd, src = buttons.get(0.1)

            if cmd == "toggle":
                if rec.recording:
                    s = rec.stop(keep=True)
                    say("\n" + stopped_line(s, vive, dxl))
                    if live is not None:
                        live.add_episode("%s  %.1fs" % (s["name"], s["duration_s"]))
                    beep("stop")
                else:
                    name = rec.start()
                    say("\n[recording] %s  (%s)" % (name, src))
                    alarm.reset()
                    speech = ["start"]
                    if rec.origin is None:
                        say("[warn] no origin -> this episode will be excluded by to_gr00t (set with 'o' or button double-click)")
                        speech.append("no origin")
                    beep(", ".join(speech))
            elif cmd == "discard":
                if rec.recording:
                    rec.stop(keep=False)
                    say("\n[discarded current]")
                    beep("discard")
                else:
                    d = rec.discard_last()
                    say("\n[discarded] %s" % d if d else "\n[nothing to discard]")
                    if d and live is not None:
                        live.add_episode("%s  DISCARDED" % os.path.basename(d))
                    if d:
                        beep("discard")
            elif cmd == "task" and not rec.recording:
                t = buttons.read_line("\nnew task (current: %s): " % rec.task)
                if t:
                    rec.task = t.strip()
            elif cmd == "origin" and rec.recording:
                say("\n[origin] cannot set origin while recording")
                beep("recording")
            elif cmd == "origin":
                try:
                    say("\n[origin] measuring for 1s (hold gripper still)...")
                    T_o, raw, msg = calibrate_origin_live(vive, cfg, hands=cfg["hands"], level=True)
                    if msg:
                        say("[origin] " + msg)
                    rec.origin = save_origin(rec.origin_path, T_o, raw)
                    say("[origin] saved: %s  position (z-up world) %s" % (rec.origin_path, T_o[:3, 3].round(3)))
                    beep("origin set")
                except Exception as e:
                    say("[origin] failed: %s" % e)
                    beep("origin failed")
            elif cmd == "quit" or (live and live.viz.viewer.poll() is not None):   # viewer window closed -> quit
                break

            for t, line in frames.poll():
                if drop_stale_origin(rec, t):
                    say("\n[origin] SteamVR coordinates changed (%s) -> origin no longer valid, set it again%s"
                        % (line, "; this episode is affected, discard it" if rec.recording else ""))
                    beep("tracking changed, discard, set origin again" if rec.recording else "tracking changed, set origin again")
            for sensor in sources:
                if sensor.label not in stopped and not sensor.is_alive():
                    stopped.add(sensor.label)
                    say("\n[ERR] %s" % (sensor.error or "%s stopped" % sensor.label))
                    beep("%s stopped" % sensor.label.split()[0])
            problems = sensor_problems(vive, dxl, cams, cfg["hands"])
            if rec.recording:
                for speech, msg, first in alarm.due(problems, time.monotonic()):
                    if first:
                        say("\n[warn] while recording: %s" % msg)
                    beep(speech, important=False)
            problems.update({"%s button lost" % b.hand: b.error for b in buttons.bt if b.error})
            speech = reconnect.due(problems, time.monotonic())
            if speech:
                beep(speech, important=False)

            items = {s.label: s.error for s in sources if s.is_alive()}
            items.update({"%s camera settings" % c.name: c.warning for c in cams})
            items["dynamixel"] = dxl.warning
            items.update({"%s button" % b.hand: b.error for b in buttons.bt})
            for msg in notifier.changes(items):
                say("\n" + msg)
            while not rec.encoder.log.empty():
                say("\n" + rec.encoder.log.get())
            width = shutil.get_terminal_size((120, 20)).columns - 1
            try:
                print("\r" + fit(status(), width), end="", flush=True)
            except OSError:   # terminal closed (may be seen before SIGHUP)
                break
    except KeyboardInterrupt:
        pass
    finally:
        # order: save -> stop sensors (gripper torque off) -> restore terminal -> print (EIO after SIGHUP, ignored)
        saved = rec.stop(keep=True) if rec.recording else None
        if live is not None:
            live.stop()
        for s in sources:
            s.stop()
        for s in sources:
            if s.ident is not None:
                s.join(timeout=2.0)
        buttons.close()
        try:
            if saved:
                say("\n[stopped] %s" % saved["name"])
            if rec.encoder.pending:
                say("\n[mp4] finishing %d encode(s)... (if interrupted, JPGs are kept and check_episode will encode them)" % rec.encoder.pending)
        except OSError:
            pass
        rec.encoder.close()
        try:
            while not rec.encoder.log.empty():
                say(rec.encoder.log.get())
            say()
        except OSError:
            pass


if __name__ == "__main__":
    main()
