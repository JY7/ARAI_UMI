"""Episode writing/reading and the origin file. All t_ns = time.monotonic_ns()"""
import csv
import datetime
import fcntl
import json
import os
import glob
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
from contextlib import suppress

import cv2
import numpy as np
import yaml

TRACKER_HEADER = (["t_ns", "hand", "valid", "result"]
                  + ["m%d%d" % (r, c) for r in range(3) for c in range(4)]
                  + ["vx", "vy", "vz", "wx", "wy", "wz"])
GRIPPER_HEADER = ["t_ns", "hand", "trigger_pos", "gripper_goal", "gripper_pos", "gripper_cur", "gripper_vel"]
TIME_FMT = "%Y-%m-%dT%H:%M:%S%z"
VIDEO_NAME = "video.mp4"
VIDEO_CRF = 23              # libx264 quality (lower = better/larger); 23: ~1/13 of JPG size, PSNR ~39 dB at 256 px training res
CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config.yaml")
DATA_ROOT = os.path.join(os.path.dirname(CONFIG_PATH), "data")

_STOP = object()


class CsvWriter(threading.Thread):
    def __init__(self, path, header):
        super().__init__(daemon=True)
        self.q = queue.Queue()
        self.path = path
        self.header = header
        self.count = 0
        self.start()

    def put(self, row):
        self.q.put(row)

    def run(self):
        with open(self.path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(self.header)
            for row in iter(self.q.get, _STOP):
                w.writerow(row)
                self.count += 1

    def close(self):
        self.q.put(_STOP)
        self.join()


class ImageWriter(threading.Thread):
    def __init__(self, dirpath, maxsize=90):
        super().__init__(daemon=True)
        os.makedirs(dirpath, exist_ok=True)
        self.dir = dirpath
        self.q = queue.Queue(maxsize=maxsize)
        self.count = 0
        self.dropped = 0
        self.corrupt = 0
        self.start()

    def put(self, t_ns, frame, dev_ts_ms=-1.0):
        try:
            self.q.put_nowait((t_ns, frame, dev_ts_ms))
        except queue.Full:
            self.dropped += 1

    def run(self):
        params = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        with open(os.path.join(self.dir, "timestamps.csv"), "w", newline="", encoding="utf-8") as ts:
            tsw = csv.writer(ts)
            tsw.writerow(["frame_idx", "t_ns", "dev_ts_ms"])
            for idx, (t_ns, frame, dev_ts) in enumerate(iter(self.q.get, _STOP)):
                path = os.path.join(self.dir, "%06d.jpg" % idx)
                if isinstance(frame, (bytes, bytearray)):
                    with open(path, "wb") as f:
                        f.write(frame)
                else:
                    cv2.imwrite(path, frame, params)
                tsw.writerow([idx, t_ns, "%.3f" % dev_ts])
                self.count += 1

    def close(self):
        self.q.put(_STOP)
        self.join()


class Episode:
    def __init__(self, root, index, cfg, task, origin=None):
        self.name = "%s_ep%03d" % (time.strftime("%Y%m%d_%H%M%S"), index)
        self.dir = os.path.join(root, self.name)
        os.makedirs(self.dir, exist_ok=False)
        self.cfg, self.task, self.origin = cfg, task, origin
        self.t_start_ns = time.monotonic_ns()
        self.wall_start = time.strftime(TIME_FMT)
        self.closed = False
        self._lock = threading.Lock()
        self.tracker = CsvWriter(os.path.join(self.dir, "tracker.csv"), TRACKER_HEADER)
        self.gripper = CsvWriter(os.path.join(self.dir, "gripper.csv"), GRIPPER_HEADER)
        self.images = {name: ImageWriter(os.path.join(self.dir, "cam_" + name)) for name in (cfg.get("cameras") or {})}

    def close(self, keep=True):
        with self._lock:
            if self.closed:
                return None
            t_end = time.monotonic_ns()
            self.closed = True
        for w in [self.tracker, self.gripper] + list(self.images.values()):
            w.close()
        summary = {
            "name": self.name,
            "task": self.task,
            "wall_start": self.wall_start,
            "t_start_ns": self.t_start_ns,
            "t_end_ns": t_end,
            "duration_s": round((t_end - self.t_start_ns) * 1e-9, 3),
            "counts": {"tracker_rows": self.tracker.count, "gripper_rows": self.gripper.count,
                       **{"cam_" + k: v.count for k, v in self.images.items()}},
            "dropped_frames": {k: v.dropped for k, v in self.images.items()},
            "corrupt_frames": {k: v.corrupt for k, v in self.images.items()},
            "origin": self.origin,
            "config": self.cfg,
        }
        with open(os.path.join(self.dir, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        if not keep:
            shutil.rmtree(self.dir, ignore_errors=True)
        return summary


def encode_camera(cam_dir, fps):
    """cam_dir/%06d.jpg -> cam_dir/video.mp4; JPGs deleted only if the frame count matches timestamps.csv.
    Returns "done" | "skip" (no JPGs) | failure reason (JPGs kept)"""
    if not os.path.isdir(cam_dir):
        return "skip"
    with open(os.path.join(cam_dir, ".encode.lock"), "w") as lock:   # collect's background encoder and check_episode may race
        fcntl.flock(lock, fcntl.LOCK_EX)
        return _encode_camera(cam_dir, fps)


def _encode_camera(cam_dir, fps):
    jpgs = sorted(glob.glob(os.path.join(cam_dir, "[0-9]" * 6 + ".jpg")))
    if not jpgs:
        return "skip"
    n_ts = len(_read_rows(os.path.join(cam_dir, "timestamps.csv")))
    tmp = os.path.join(cam_dir, VIDEO_NAME + ".tmp.mp4")
    final = os.path.join(cam_dir, VIDEO_NAME)
    if os.path.exists(final) and _frame_count(final) == n_ts:   # killed while deleting JPGs -> just remove the rest
        for f in jpgs:
            os.remove(f)
        _remove(tmp)
        return "done"
    if len(jpgs) != n_ts or os.path.basename(jpgs[-1]) != "%06d.jpg" % (n_ts - 1):
        return "%d JPGs != %d timestamp rows" % (len(jpgs), n_ts)
    cmd = ["nice", "-n", "10",   # below the capture threads of an ongoing recording
           "ffmpeg", "-loglevel", "error", "-y", "-framerate", str(fps), "-i", os.path.join(cam_dir, "%06d.jpg"),
           "-c:v", "libx264", "-preset", "veryfast", "-crf", str(VIDEO_CRF), "-pix_fmt", "yuv420p",
           "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",   # yuv420p needs even dimensions
           "-g", str(max(1, int(round(fps)))),       # keyframe every 1 s for fast seeking
           tmp]   # fixed-fps image input -> frames map 1:1 (no -fps_mode: needs ffmpeg 5.1+, Ubuntu 20.04 has 4.2)
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        _remove(tmp)
        return "ffmpeg failed: %s" % (r.stderr.strip().splitlines() or ["?"])[-1]
    n_mp4 = _frame_count(tmp)
    if n_mp4 != n_ts:
        _remove(tmp)
        return "mp4 has %d frames != %d" % (n_mp4, n_ts)
    os.replace(tmp, final)
    for f in jpgs:
        os.remove(f)
    return "done"


def encode_episode(ep_dir):
    if not os.path.exists(os.path.join(ep_dir, "meta.json")):
        return {}
    cams = (load_meta(ep_dir).get("config") or {}).get("cameras") or {}
    return {c: encode_camera(os.path.join(ep_dir, "cam_" + c), float((v or {}).get("fps", 30))) for c, v in cams.items()}


def _frame_count(path):
    cap = cv2.VideoCapture(path)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if cap.isOpened() else -1
    cap.release()
    return n


def _remove(path):
    with suppress(OSError):
        os.remove(path)


class VideoEncoder(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.q = queue.Queue()
        self.pending = 0
        self.log = queue.Queue()
        self.start()

    def put(self, ep_dir):
        self.pending += 1
        self.q.put(ep_dir)

    def run(self):
        for ep_dir in iter(self.q.get, _STOP):
            try:
                t0 = time.monotonic()
                res = encode_episode(ep_dir)
                bad = {c: r for c, r in res.items() if r not in ("done", "skip")}
                if not os.path.isdir(ep_dir):
                    pass
                elif bad:
                    self.log.put("[mp4] %s encode failed, JPGs kept: %s" % (os.path.basename(ep_dir), bad))
                elif res:
                    mb = sum(os.path.getsize(os.path.join(ep_dir, "cam_" + c, VIDEO_NAME)) for c in res
                             if os.path.exists(os.path.join(ep_dir, "cam_" + c, VIDEO_NAME))) / 1e6
                    self.log.put("[saved] %s  (mp4 %.1f MB, encoded in %.1fs)" % (os.path.basename(ep_dir), mb, time.monotonic() - t0))
            except Exception as e:
                if os.path.isdir(ep_dir):
                    self.log.put("[mp4] %s encode aborted, JPGs kept: %s" % (os.path.basename(ep_dir), e))
            self.pending -= 1

    def close(self):
        self.q.put(_STOP)
        self.join()


class FrameReader:
    """Frames by index from cam_<name>/video.mp4, else JPGs; fast for sequential reads"""

    def __init__(self, ep_dir, cam):
        self.ep_dir, self.cam = ep_dir, cam
        self.path = os.path.join(ep_dir, "cam_" + cam, VIDEO_NAME)
        self.cap = cv2.VideoCapture(self.path) if os.path.exists(self.path) else None
        self.next = 0

    def get(self, idx):
        """BGR image, None on failure"""
        idx = int(idx)
        if self.cap is None:
            return cv2.imread(frame_path(self.ep_dir, self.cam, idx))
        if idx < self.next or idx - self.next > 60:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            self.next = idx
        while self.next < idx:
            self.cap.grab()
            self.next += 1
        ok, img = self.cap.read()
        self.next += 1
        return img if ok else None

    def describe(self, idx):
        return "%s #%d" % (self.path, idx) if self.cap is not None else frame_path(self.ep_dir, self.cam, idx)

    def close(self):
        if self.cap is not None:
            self.cap.release()


class Recorder:
    """Sources always run; data is written only while recording"""

    def __init__(self, cfg, root):
        self.cfg, self.root = cfg, root
        os.makedirs(self.root, exist_ok=True)
        self.task = cfg.get("task", "")
        self.origin_path = os.path.join(self.root, "origin.json")
        self.origin = load_origin(self.origin_path)
        max_age_h = float(cfg.get("origin_max_age_h", 12))
        if self.origin and origin_age_s(self.origin) > max_age_h * 3600:
            print("[origin] %s: origin is %.1fh old, not used (origin_max_age_h=%g). Set the origin again."
                  % (self.origin_path, origin_age_s(self.origin) / 3600, max_age_h))
            self.origin = None
        self.episode = None
        self.last_dir = None
        self._lock = threading.Lock()
        self.encoder = VideoEncoder()
        self._index = max([int(d.split("_ep")[1][:3]) + 1 for d in os.listdir(self.root)
                           if "_ep" in d and d.split("_ep")[1][:3].isdigit()] or [0])

    def _reuse_index(self, ep_dir):
        n = os.path.basename(ep_dir).split("_ep")[-1][:3]
        with self._lock:
            if self.episode is None and n.isdigit() and int(n) == self._index - 1:
                self._index = int(n)

    @property
    def recording(self):
        return self.episode is not None

    def start(self):
        with self._lock:
            if self.episode is None:
                self.episode = Episode(self.root, self._index, self.cfg, self.task, self.origin)
                self._index += 1
                return self.episode.name
        return None

    def stop(self, keep=True):
        with self._lock:
            ep, self.episode = self.episode, None
        if ep is None:
            return None
        summary = ep.close(keep)
        self.last_dir = ep.dir if keep else None
        if not keep:
            self._reuse_index(ep.dir)
        if keep and ep.images:
            self.encoder.put(ep.dir)
        return summary

    def discard_last(self):
        if self.last_dir and os.path.isdir(self.last_dir):
            shutil.rmtree(self.last_dir, ignore_errors=True)
            d, self.last_dir = self.last_dir, None
            self._reuse_index(d)
            return d
        return None

    # called from source threads
    def log_tracker(self, row):
        ep = self.episode
        if ep is not None and not ep.closed:
            ep.tracker.put(row)

    def log_gripper(self, row):
        ep = self.episode
        if ep is not None and not ep.closed:
            ep.gripper.put(row)

    def log_image(self, cam, t_ns, frame, dev_ts_ms=-1.0):
        ep = self.episode
        if ep is not None and not ep.closed and cam in ep.images:
            ep.images[cam].put(t_ns, frame, dev_ts_ms)

    def log_corrupt(self, cam):
        ep = self.episode
        if ep is not None and not ep.closed and cam in ep.images:
            ep.images[cam].corrupt += 1


def save_origin(path, T_world_origin, raw_means):
    d = {
        "T_world_origin": np.asarray(T_world_origin).tolist(),   # z-up world frame
        "raw_tracker_mean_steamvr": {h: np.asarray(T).tolist() for h, T in raw_means.items()},
        "time": time.strftime(TIME_FMT),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(d, f, indent=2)
    return d


def load_origin(path):
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def origin_age_s(o):
    t = datetime.datetime.strptime(o["time"], TIME_FMT)
    return (datetime.datetime.now(t.tzinfo) - t).total_seconds()


def load_meta(ep_dir):
    with open(os.path.join(ep_dir, "meta.json"), encoding="utf-8") as f:
        return json.load(f)


def _read_rows(path):
    with open(path, encoding="utf-8") as f:
        r = csv.reader(f)
        header = next(r, None)
        if header is None:
            print("  [warn] %s: empty file (no header)" % path)
            return []
        n = len(header)
        rows = list(r)
    good = [x for x in rows if len(x) == n]
    if len(good) != len(rows):
        print("  [warn] %s: ignored %d malformed rows" % (path, len(rows) - len(good)))
    return good


def _rows_by_hand(path):
    rows = {}
    for row in _read_rows(path):
        rows.setdefault(row[1], []).append(row)
    return rows


def load_tracker(ep_dir):
    """{hand: {"t": (N,) int64, "valid": (N,) bool, "result": (N,) int, "T": (N,4,4)}}"""
    out = {}
    for hand, rr in _rows_by_hand(os.path.join(ep_dir, "tracker.csv")).items():
        a = np.array([x[4:16] for x in rr], dtype=float)
        T = np.tile(np.eye(4), (len(rr), 1, 1))
        T[:, :3, :] = a.reshape(-1, 3, 4)
        out[hand] = {"t": np.array([x[0] for x in rr], dtype=np.int64),
                     "valid": np.array([x[2] == "1" for x in rr]),
                     "result": np.array([int(x[3]) for x in rr]), "T": T}
    return out


def load_gripper(ep_dir):
    """{hand: {"t", "goal", "pos"}} (goal = trigger command, pos = measured position)"""
    out = {}
    for hand, rr in _rows_by_hand(os.path.join(ep_dir, "gripper.csv")).items():
        out[hand] = {"t": np.array([x[0] for x in rr], dtype=np.int64),
                     "goal": np.array([float(x[3]) for x in rr]), "pos": np.array([float(x[4]) for x in rr])}
    return out


def load_camera(ep_dir, cam):
    """{"idx": (N,), "t": (N,) int64}. t = V4L2 buffer time (dev_ts, CLOCK_MONOTONIC), free of the ~1 frame queue delay
    and thread jitter of host time; host time if dev_ts is missing or >1 s off (different clock)"""
    rr = _read_rows(os.path.join(ep_dir, "cam_" + cam, "timestamps.csv"))
    t = np.array([int(x[1]) for x in rr], np.int64)
    dev = np.array([float(x[2]) * 1e6 for x in rr]).astype(np.int64)
    if len(t) and (dev > 0).all() and abs(np.median(t - dev)) < 1e9:
        t = dev
    return {"idx": np.array([int(x[0]) for x in rr], np.int64), "t": t}


def frame_path(ep_dir, cam, idx):
    return os.path.join(ep_dir, "cam_" + cam, "%06d.jpg" % idx)


def data_path(p="", must_exist=True):
    """Script path arg: '' -> data/, 'cup' -> data/cup; absolute and ~/ paths unchanged"""
    path = os.path.normpath(os.path.join(DATA_ROOT, os.path.expanduser(p)))
    if must_exist and not os.path.isdir(path):
        sys.exit("not found: %s (check the folder name with `ls data`)" % path)
    return path


def episodes_in(path):
    return [path] if os.path.exists(os.path.join(path, "meta.json")) else list_episodes(path)


def list_episodes(root):
    return [os.path.join(root, d) for d in sorted(os.listdir(root))
            if "_ep" in d and not d.endswith("_DISCARDED") and os.path.exists(os.path.join(root, d, "meta.json"))]


def list_incomplete(root):
    return [os.path.join(root, d) for d in sorted(os.listdir(root))
            if "_ep" in d and not d.endswith("_DISCARDED") and os.path.isdir(os.path.join(root, d))
            and not os.path.exists(os.path.join(root, d, "meta.json"))]


CALIB_KEYS = ("align", "gripper_model")
MOUNT_KEYS = ("tracker_frame_rpy_deg",)   # per-hand tracker mounting in gripper_model


def effective_config(meta_cfg, current_cfg=None, current_mount=False):
    """Episode config + current calibration (align, gripper_model, cameras.*.latency_ms). MOUNT_KEYS keep the
    recorded values so a re-mounted tracker's calib doesn't hit old episodes, unless null or current_mount=True"""
    cfg = dict(meta_cfg)
    if current_cfg:
        cfg.update({k: current_cfg[k] for k in CALIB_KEYS if k in current_cfg})
        if not current_mount and cfg.get("gripper_model"):
            gm = dict(cfg["gripper_model"])
            rec = meta_cfg.get("gripper_model") or {}
            for k in MOUNT_KEYS:
                vals = dict(gm.get(k) or {})
                vals.update({h: v for h, v in (rec.get(k) or {}).items() if v is not None})
                gm[k] = vals
            cfg["gripper_model"] = gm
        cur_cams = current_cfg.get("cameras") or {}
        cfg["cameras"] = {c: dict(v, **{k: x for k, x in (cur_cams.get(c) or {}).items() if k == "latency_ms"})
                          for c, v in (cfg.get("cameras") or {}).items()}
    return cfg


def load_current_config():
    with open(CONFIG_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f)


def set_config_value(keys, value):
    """config.yaml keys[0].keys[1]... = value (str); keeps comments (same column) and line endings"""
    with open(CONFIG_PATH, encoding="utf-8", newline="") as f:
        text = f.read()
    eol = "\r\n" if "\r\n" in text else "\n"
    lines = text.split(eol)
    depth, indent = 0, -1
    for i, l in enumerate(lines):
        if not l.strip() or l.lstrip().startswith("#"):
            continue
        ind = len(l) - len(l.lstrip())
        if depth and ind <= indent:
            break
        if ind > indent and re.match(r"\s*%s\s*:" % re.escape(keys[depth]), l):
            if depth == len(keys) - 1:
                c = l.find("#")
                head = l[:l.index(":") + 1] + " " + value
                lines[i] = head + (" " * max(1, c - len(head)) + l[c:] if c >= 0 else "")
                with open(CONFIG_PATH, "w", encoding="utf-8", newline="") as f:
                    f.write(eol.join(lines))
                return
            depth, indent = depth + 1, ind
    raise RuntimeError("config.yaml has no %s" % ".".join(keys))
