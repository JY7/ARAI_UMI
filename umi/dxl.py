"""Trigger (XC330-T181-T, 12V) -> gripper (XM430-W350-T) follower control and logging. Both X-series, Protocol 2.0, TTL"""
import time
from contextlib import suppress

import numpy as np

from .episode import set_config_value
from .source import SourceThread

ADDR_OPERATING_MODE = 11
ADDR_HARDWARE_ERROR = 70     # nonzero = motor disabled its torque (overload, overheat, voltage)
ADDR_TORQUE_ENABLE = 64
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_P_GAIN = 84
ADDR_GOAL_CURRENT = 102
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
READ_START, READ_LEN = 126, 10
MODE_CURRENT_BASED_POSITION = 5
HW_CHECK_S = 1.0


def _s16(v):
    return v - 0x10000 if v & 0x8000 else v


def _s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


def trigger_to_gripper(trig, h):
    t0, t1 = h["trigger_open"], h["trigger_closed"]
    g0, g1 = h["gripper_open"], h["gripper_closed"]
    a = (trig - t0) / float(t1 - t0) if t1 != t0 else 0.0
    a = min(1.0, max(0.0, a))
    return int(round(g0 + a * (g1 - g0)))


def fraction(hc, part, raw):
    """Raw -> 0 (<part>_open) .. 1 (<part>_closed); part: 'gripper' | 'trigger'"""
    lo, hi = float(hc.get(part + "_open", 0)), float(hc.get(part + "_closed", 4095))
    return np.clip((raw - lo) / ((hi - lo) or 1.0), 0, 1)


def open_bus(cfg, hands):
    return _Bus(cfg["port"], int(cfg.get("baudrate", 1000000)), [i for h in hands.values() for i in (h["trigger_id"], h["gripper_id"])])


class _Bus:
    def __init__(self, port, baud, ids):
        import dynamixel_sdk as sdk
        self.sdk = sdk
        self.port = sdk.PortHandler(port)
        self.pk = sdk.PacketHandler(2.0)
        if not self.port.openPort():
            raise RuntimeError("cannot open %s" % port)
        if not self.port.setBaudRate(baud):
            self.port.closePort()
            raise RuntimeError("cannot set baud %d on %s" % (baud, port))
        self.ids = list(ids)
        self.reader = sdk.GroupSyncRead(self.port, self.pk, READ_START, READ_LEN)
        for i in self.ids:
            self.reader.addParam(i)
        self.writer = sdk.GroupSyncWrite(self.port, self.pk, ADDR_GOAL_POSITION, 4)

    def write(self, dxl_id, addr, val, size=1):
        fn = {1: self.pk.write1ByteTxRx, 2: self.pk.write2ByteTxRx, 4: self.pk.write4ByteTxRx}[size]
        res, err = fn(self.port, dxl_id, addr, val)
        if res != self.sdk.COMM_SUCCESS or err:
            raise RuntimeError("id %d addr %d write failed (%s / %s)" % (
                dxl_id, addr, self.pk.getTxRxResult(res), self.pk.getRxPacketError(err)))

    def read1(self, dxl_id, addr):
        val, res, err = self.pk.read1ByteTxRx(self.port, dxl_id, addr)
        if res != self.sdk.COMM_SUCCESS:
            raise RuntimeError("id %d addr %d read failed (%s)" % (dxl_id, addr, self.pk.getTxRxResult(res)))
        return val

    def read_all(self):
        """{id: (cur, vel, pos)}; falls back to per-id reads if sync read fails (one motor not responding)"""
        res = self.reader.txRxPacket()
        out = {}
        if res != self.sdk.COMM_SUCCESS:
            for i in self.ids:
                data, r, err = self.pk.readTxRx(self.port, i, READ_START, READ_LEN)
                if r == self.sdk.COMM_SUCCESS and len(data) == READ_LEN:
                    b = bytes(data)
                    out[i] = tuple(int.from_bytes(b[a:a + n], "little", signed=True) for a, n in ((0, 2), (2, 4), (6, 4)))
            return out
        for i in self.ids:
            if self.reader.isAvailable(i, READ_START, READ_LEN):
                cur = _s16(self.reader.getData(i, ADDR_PRESENT_CURRENT, 2))
                vel = _s32(self.reader.getData(i, ADDR_PRESENT_VELOCITY, 4))
                pos = _s32(self.reader.getData(i, ADDR_PRESENT_POSITION, 4))
                out[i] = (cur, vel, pos)
        return out

    def write_goals(self, goals):
        sdk = self.sdk
        self.writer.clearParam()
        for i, g in goals.items():
            self.writer.addParam(i, [sdk.DXL_LOBYTE(sdk.DXL_LOWORD(g)), sdk.DXL_HIBYTE(sdk.DXL_LOWORD(g)),
                                     sdk.DXL_LOBYTE(sdk.DXL_HIWORD(g)), sdk.DXL_HIBYTE(sdk.DXL_HIWORD(g))])
        self.writer.txPacket()

    def close(self):
        self.port.closePort()


RECONNECT_S = 1.0
LOST_S = 1.0        # all reads failing this long = disconnected -> reconnect (12V off resets torque/mode, so full setup needed)
REPROBE_S = 5.0     # re-ping hands skipped at startup


class DxlSource(SourceThread):
    def __init__(self, cfg, recorder):
        super().__init__(1.0 / float(cfg.get("rate_hz", 100)), "dxl")
        self.cfg = cfg
        self.rec = recorder
        self.hands = dict(cfg.get("hands") or {})
        self.latest = {}
        self.warning = None
        self.bus = None
        self._hw_error = None
        self._hw_next = 0.0
        self.error = "dynamixel connecting"   # not-ok until the first successful connect

    def _setup(self):
        self.hands, self.warning = dict(self.cfg.get("hands") or {}), None
        self.bus = open_bus(self.cfg, self.hands)
        states = self.bus.read_all()
        missing = [hand for hand, h in self.hands.items()
                   if h["trigger_id"] not in states and h["gripper_id"] not in states]
        if missing:
            if len(missing) == len(self.hands):
                raise RuntimeError("no motor responded (check 12V power, cable, ids/baudrate)")
            self.warning = "no response from %s motors, running without them" % ", ".join(
                "%s (ids %d,%d)" % (hand, self.hands[hand]["trigger_id"], self.hands[hand]["gripper_id"]) for hand in missing)
            self.hands = {k: v for k, v in self.hands.items() if k not in missing}
            self.bus.close()   # drop missing ids from sync read, else every cycle falls back to slow per-id reads
            self.bus = open_bus(self.cfg, self.hands)
        b = self.bus
        for h in self.hands.values():
            # trigger return spring: P gain = stiffness, Goal Current = max force; goal set before torque on to avoid a jump
            b.write(h["trigger_id"], ADDR_TORQUE_ENABLE, 0)
            b.write(h["trigger_id"], ADDR_OPERATING_MODE, MODE_CURRENT_BASED_POSITION)
            b.write(h["trigger_id"], ADDR_GOAL_CURRENT, int(h["trigger_spring_current"]), 2)
            b.write(h["trigger_id"], ADDR_POSITION_P_GAIN, int(h["trigger_spring_p_gain"]), 2)
            # default D gain (1400) > P reacts to velocity noise -> notchy pull
            b.write(h["trigger_id"], ADDR_POSITION_D_GAIN, int(h["trigger_spring_d_gain"]), 2)
            b.write(h["trigger_id"], ADDR_GOAL_POSITION, int(h["trigger_open"]), 4)
            b.write(h["trigger_id"], ADDR_TORQUE_ENABLE, 1)
            # firmware overwrites Goal Position with present position on torque enable -> write again
            b.write(h["trigger_id"], ADDR_GOAL_POSITION, int(h["trigger_open"]), 4)
            b.write(h["gripper_id"], ADDR_TORQUE_ENABLE, 0)
            b.write(h["gripper_id"], ADDR_OPERATING_MODE, MODE_CURRENT_BASED_POSITION)
            b.write(h["gripper_id"], ADDR_GOAL_CURRENT, int(h.get("gripper_goal_current", 300)), 2)
            # goal before torque on = current trigger target (not a stale goal left in the register)
            trig, grip = states.get(h["trigger_id"]), states.get(h["gripper_id"])
            if trig is not None or grip is not None:
                goal = trigger_to_gripper(trig[2], h) if trig is not None else grip[2]
                b.write(h["gripper_id"], ADDR_GOAL_POSITION, goal, 4)
            b.write(h["gripper_id"], ADDR_TORQUE_ENABLE, 1)

    def _shutdown(self):
        if self.bus is None:
            return
        for h in self.hands.values():
            for i in (h["gripper_id"], h["trigger_id"]):
                with suppress(Exception):
                    self.bus.write(i, ADDR_TORQUE_ENABLE, 0)
        with suppress(Exception):
            self.bus.close()
        self.bus = None

    def _run(self):
        """Connect and run; reconnects every RECONNECT_S if the U2D2 is absent or unplugged, or 12V goes off"""
        while not self._halt.is_set():
            try:
                self._setup()
            except Exception as e:
                self.error = "dynamixel not connected (%s), retrying" % e
                self.rate = 0.0
                self._shutdown()
                self._halt.wait(RECONNECT_S)
                continue
            self.error = None
            self._reset_clock()
            try:
                self._loop()
            except Exception as e:
                self.error = "dynamixel connection lost (%s), reconnecting" % e
            finally:
                self._shutdown()
                self.latest, self.rate = {}, 0.0
            self._halt.wait(RECONNECT_S)

    def _loop(self):
        lost_since, next_probe = None, time.monotonic() + REPROBE_S
        while not self._halt.is_set():
            any_ok = self._step()
            now = time.monotonic()
            if any_ok:
                lost_since = None
            elif lost_since is None:
                lost_since = now
            elif now - lost_since > LOST_S:
                raise RuntimeError("no motor response for %.0fs (12V power?)" % LOST_S)
            if self.warning and now >= next_probe:
                next_probe = now + REPROBE_S
                if self._missing_respond():
                    self.error = "dynamixel: missing hand came online, reconnecting"
                    return
            self.tick()

    def _missing_respond(self):
        b = self.bus
        return any(b.pk.ping(b.port, i)[1] == b.sdk.COMM_SUCCESS for hand, h in (self.cfg.get("hands") or {}).items()
                   if hand not in self.hands for i in (h["trigger_id"], h["gripper_id"]))

    def _step(self):
        """One read/follow cycle; returns whether any hand was read"""
        t = time.monotonic_ns()
        states = self.bus.read_all()
        goals = {}
        failed = []
        for hand, h in self.hands.items():
            trig = states.get(h["trigger_id"])
            grip = states.get(h["gripper_id"])
            if trig is None or grip is None:
                failed.append(hand)
                continue
            goal = trigger_to_gripper(trig[2], h)
            goals[h["gripper_id"]] = goal
            row = [t, hand, trig[2], goal, grip[2], grip[0], grip[1]]
            self.rec.log_gripper(row)
            self.latest[hand] = {"trigger": trig[2], "pos": grip[2]}
        if goals:
            self.bus.write_goals(goals)
        if time.monotonic() >= self._hw_next:
            self._hw_next = time.monotonic() + HW_CHECK_S
            self._hw_error = self._check_hw()
        errs = (["read failed: %s" % ",".join(failed)] if failed else []) + ([self._hw_error] if self._hw_error else [])
        self.error = "; ".join(errs) or None
        return len(failed) < len(self.hands)

    def _check_hw(self):
        """Hardware Error Status of grippers (stop following when set) and triggers (spring stops returning)"""
        bad = []
        for hand, h in self.hands.items():
            for part, i in (("gripper", h["gripper_id"]), ("trigger", h["trigger_id"])):
                try:
                    v = self.bus.read1(i, ADDR_HARDWARE_ERROR)
                except Exception:
                    continue
                if v:
                    bad.append("%s %s 0x%02x" % (hand, part, v))
        return ("dynamixel hardware error %s (power cycle needed)" % ", ".join(bad)) if bad else None


MIN_CALIB_RANGE = 100   # min open/closed raw difference [ticks]
CALIB_CURRENT = 150     # gripper current limit during auto-calib (Goal Current raw, ~0.4A: safe to stall at the stops)
CALIB_VELOCITY = 60     # Profile Velocity raw during auto-calib (~14 rpm)
CALIB_STALL_TICKS = 3   # moved less than this within CALIB_STALL_S = stalled at a stop
CALIB_STALL_S = 0.4
CALIB_TIMEOUT_S = 10.0
CALIB_REPEATS = 3       # round trips, median saved
CALIB_MAX_SPREAD = 20
ADDR_PROFILE_VELOCITY = 112


def _run_to_stop(bus, dxl_id, target):
    """Drive toward a far target until stalled at a mechanical stop; returns the stall position (raw)"""
    bus.write(dxl_id, ADDR_GOAL_POSITION, int(target) & 0xFFFFFFFF, 4)
    hist, t0 = [], time.monotonic()
    while time.monotonic() - t0 < CALIB_TIMEOUT_S:
        st = bus.read_all().get(dxl_id)
        if st is not None:
            now = time.monotonic()
            hist = [(t, p) for t, p in hist if now - t <= CALIB_STALL_S] + [(now, st[2])]
            if now - t0 > CALIB_STALL_S + 0.3 and hist[-1][0] - hist[0][0] > CALIB_STALL_S * 0.8 \
                    and max(p for _, p in hist) - min(p for _, p in hist) < CALIB_STALL_TICKS:
                if abs(st[2] - target) < 200:
                    raise RuntimeError("id %d reached the target without hitting a mechanical stop" % dxl_id)
                return st[2]
        time.sleep(0.02)
    raise RuntimeError("id %d did not stop within %.0fs (no mechanical stop?)" % (dxl_id, CALIB_TIMEOUT_S))


def _show_triggers(bus, hands):
    import select
    import sys
    print("\nTrigger check (fixed values, not measured): pull each trigger and watch 0% (released) -> 100% (pulled). Enter to finish")
    print("\n" * len(hands), end="")
    while not select.select([sys.stdin], [], [], 0.05)[0]:
        st = bus.read_all()
        lines = []
        for hand, h in hands.items():
            raw = (st.get(h["trigger_id"]) or (0, 0, None))[2]
            if raw is None or h.get("trigger_open") is None or h.get("trigger_closed") is None:
                lines.append("%-5s trigger raw %s" % (hand, raw))
                continue
            lo, hi = h["trigger_open"], h["trigger_closed"]
            out = "  OUT OF RANGE" if not min(lo, hi) - 50 <= raw <= max(lo, hi) + 50 else ""
            lines.append("%-5s trigger raw %4d = %3.0f%%  (open %d, closed %d)%s" % (hand, raw, 100 * fraction(h, "trigger", raw) + 0.0, lo, hi, out))
        print("\x1b[%dF" % len(lines) + "".join("\x1b[K%s\n" % l for l in lines), end="", flush=True)
    sys.stdin.readline()
    print()


def calibrate(cfg, beep):
    """--calib-dxl: measure gripper open/closed stops by driving slowly at low current; trigger values stay fixed in config.
    raw increases when closing (both grippers assembled the same way)"""
    hands = cfg["hands"]
    bus = open_bus(cfg, hands)
    try:
        for h in hands.values():
            for i in (h["trigger_id"], h["gripper_id"]):
                bus.write(i, ADDR_TORQUE_ENABLE, 0)
        input("The gripper(s) %s will open and close by themselves (slow, low force). Keep fingers and objects clear -> Enter"
              % list(hands))
        for hand, h in hands.items():
            g = h["gripper_id"]
            print("\n=== %s gripper (id %d) ===" % (hand, g))
            vel0, _, _ = bus.pk.read4ByteTxRx(bus.port, g, ADDR_PROFILE_VELOCITY)
            try:
                pos0 = bus.read_all()[g][2]
                bus.write(g, ADDR_OPERATING_MODE, MODE_CURRENT_BASED_POSITION)
                bus.write(g, ADDR_GOAL_CURRENT, CALIB_CURRENT, 2)
                bus.write(g, ADDR_PROFILE_VELOCITY, CALIB_VELOCITY, 4)
                bus.write(g, ADDR_GOAL_POSITION, pos0 & 0xFFFFFFFF, 4)
                bus.write(g, ADDR_TORQUE_ENABLE, 1)
                runs = {"gripper_open": [], "gripper_closed": []}
                for k in range(CALIB_REPEATS):
                    runs["gripper_open"].append(_run_to_stop(bus, g, (runs["gripper_closed"] or [pos0])[-1] - 4096))
                    runs["gripper_closed"].append(_run_to_stop(bus, g, runs["gripper_open"][-1] + 4096))
                    print("  run %d/%d: open stop %d, closed stop %d" % (k + 1, CALIB_REPEATS, runs["gripper_open"][-1], runs["gripper_closed"][-1]))
                vals = {key: int(round(np.median(v))) for key, v in runs.items()}
                spread = {key: max(v) - min(v) for key, v in runs.items()}
                print("  median: open %d, closed %d | spread over runs: open %d, closed %d ticks%s"
                      % (vals["gripper_open"], vals["gripper_closed"], spread["gripper_open"], spread["gripper_closed"],
                         "  [warn] > %d, something may be slipping or blocking" % CALIB_MAX_SPREAD if max(spread.values()) > CALIB_MAX_SPREAD else ""))
                bus.write(g, ADDR_GOAL_POSITION, vals["gripper_open"] & 0xFFFFFFFF, 4)
                time.sleep(1.5)
            except Exception as e:
                print("  [warn] %s. Not saved." % e)
                continue
            finally:
                for addr, val, size in ((ADDR_TORQUE_ENABLE, 0, 1), (ADDR_PROFILE_VELOCITY, vel0, 4)):
                    with suppress(Exception):
                        bus.write(g, addr, val, size)
            rng = vals["gripper_closed"] - vals["gripper_open"]
            print("  range %+d ticks (open -> closed) | before: open %s, closed %s | trigger fixed: open %s, closed %s"
                  % (rng, h.get("gripper_open"), h.get("gripper_closed"), h.get("trigger_open"), h.get("trigger_closed")))
            if abs(rng) < MIN_CALIB_RANGE:
                print("  [warn] range < %d ticks. Check the gripper moves freely (not saved)." % MIN_CALIB_RANGE)
                continue
            beep("ok")
            for key in vals:
                set_config_value(["dynamixel", "hands", hand, key], "%d" % vals[key])
            h.update(vals)
            print("  saved to config.yaml (dynamixel.hands.%s)" % hand)
        _show_triggers(bus, hands)
    except KeyboardInterrupt:
        print()
    finally:
        for h in hands.values():
            for i in (h["trigger_id"], h["gripper_id"]):
                with suppress(Exception):
                    bus.write(i, ADDR_TORQUE_ENABLE, 0)
        bus.close()
