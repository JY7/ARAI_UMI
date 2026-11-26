"""Coordinate frames, origin, time alignment.

world: SteamVR (x right, y up, z back) -> z-up (x forward, y left, z up).
origin: two hands = TCP midpoint, y = right -> left hand; one hand = TCP, x = gripper forward; + origin_offset_xyz.
align: per align.fps tick, pose/gripper are sampled at the nearest master-camera frame's timestamp (zero video-pose offset).
invalid if tracker/gripper gap > max_pose_gap_ms or camera gap > 2.5x median (1 dropped frame is repeated, 2 -> invalid).
"""
import copy
import hashlib
import json

import numpy as np
from scipy.spatial.transform import Rotation

from . import episode as io
from .dxl import fraction
from .gripper import BODY_UP_B, load_model, save_mount

W_ZUP = np.eye(4)
W_ZUP[:3, :3] = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]

MAX_FRAME_GAP = 2.5           # x median frame interval
DEFAULT_ACTION_HORIZON = 16
ALIGNED_VERSION = 3           # bump on aligned.npz format change: to_gr00t then demands re-alignment
GRIPPER_ALIGN_KEYS = ("urdf", "tracker_link", "tcp_link") + io.MOUNT_KEYS   # gripper_model keys that affect alignment (rest are viewer-only)


def tcp_offsets(cfg, hands=None):
    model = load_model(cfg)
    return {h: model.tracker_to_tcp(h) for h in hands or cfg.get("hands") or ["left", "right"]}


def raw_to_world(T_raw, T_tracker_tcp):
    return W_ZUP @ T_raw @ T_tracker_tcp


def tcp_in_origin(cfg, origin, hand, T_raw):
    """raw SteamVR tracker pose -> TCP pose in the origin frame (origin.json dict; None = z-up world)"""
    T = raw_to_world(T_raw, tcp_offsets(cfg, [hand])[hand])
    return np.linalg.inv(np.array(origin["T_world_origin"])) @ T if origin else T


def mean_pose(Ts):
    Ts = np.asarray(Ts)
    T = np.eye(4)
    T[:3, 3] = Ts[:, :3, 3].mean(0)
    T[:3, :3] = Rotation.from_matrix(Ts[:, :3, :3]).mean().as_matrix()
    return T


MAX_ORIGIN_ROLL_DEG = 3.0   # SteamVR up error is ~1 deg; more means the hands are not at the same height
MAX_LEVEL_TILT_DEG = 5.0    # max tilt still treated as standing flat on the table


def _frame(x, y, z, p):
    T = np.eye(4)
    T[:3, :3] = np.stack([x, y, z], axis=1)
    T[:3, 3] = p
    return T


def origin_from_hands(T_tcp_by_hand, hands, offset=None):
    z = np.array([0.0, 0.0, 1.0])
    if "left" in hands and "right" in hands:
        p_l, p_r = T_tcp_by_hand["left"][:3, 3], T_tcp_by_hand["right"][:3, 3]
        y = p_l - p_r   # keeps height diff: both hands assumed on the table, corrects left/right tilt of SteamVR up
        n = np.linalg.norm(y[:2])
        if n < 0.05:
            raise ValueError("hands too close (%.3fm). Spread the hands apart and set the origin again." % n)
        y /= np.linalg.norm(y)
        roll = np.degrees(np.arcsin(y[2]))
        if abs(roll) > MAX_ORIGIN_ROLL_DEG:
            raise ValueError("hands differ in height by %.0fmm (%.1f deg). Stand both grippers on the table at the same height and set the origin again."
                             % ((p_l - p_r)[2] * 1e3, roll))
        z = z - (z @ y) * y
        z /= np.linalg.norm(z)
        T = _frame(np.cross(y, z), y, z, 0.5 * (p_l + p_r))
    else:   # one hand standing on handle bottom: gripper up axis = table normal, x = finger (forward) direction
        T_tcp = T_tcp_by_hand[hands[0]]
        up = -T_tcp[:3, 2]   # TCP z points to the handle (down)
        tilt = np.degrees(np.arccos(np.clip(up @ z, -1.0, 1.0)))
        if tilt > MAX_LEVEL_TILT_DEG:
            raise ValueError("gripper tilted %.1f deg from vertical. Stand it flat on the table (handle bottom down) and set the origin again." % tilt)
        z = up
        x = T_tcp[:3, 0] - (T_tcp[:3, 0] @ z) * z
        x /= np.linalg.norm(x)
        T = _frame(x, np.cross(z, x), z, T_tcp[:3, 3])
    if offset is not None:   # TCP midpoint -> table front-edge midpoint
        T[:3, 3] += T[:3, :3] @ np.asarray(offset, float)
    return T


def origin_from_raw(raw_by_hand, cfg):
    hands = list(raw_by_hand)
    tcp = tcp_offsets(cfg, hands)
    T_tcp = {h: raw_to_world(raw_by_hand[h], tcp[h]) for h in hands}
    return origin_from_hands(T_tcp, hands, (cfg.get("align") or {}).get("origin_offset_xyz"))


def _rot_between(a, b):
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v = np.cross(a, b)
    s = np.linalg.norm(v)
    if s < 1e-12:
        return np.eye(3)
    return Rotation.from_rotvec(v / s * np.arctan2(s, a @ b)).as_matrix()


def level_mount(raw_by_hand, cfg, iters=3):
    """both grippers standing flat -> {hand: (tracker_frame_rpy_deg, initial tilt deg)}; fixes only the 2 tilt axes"""
    cfg = copy.deepcopy(cfg)
    rpys = cfg["gripper_model"].get("tracker_frame_rpy_deg") or {}
    cfg["gripper_model"]["tracker_frame_rpy_deg"] = rpys
    tilt0 = {}
    for it in range(iters):   # mount change moves TCP heights -> origin tilt, so iterate
        n = origin_from_raw(raw_by_hand, cfg)[:3, 2]
        link = load_model(cfg).T_body_tracker_link[:3, :3]
        for h, raw in raw_by_hand.items():
            R_rpy = Rotation.from_euler("xyz", rpys.get(h) or [0, 0, 0], degrees=True).as_matrix()
            v = R_rpy.T @ link.T @ BODY_UP_B                 # gripper up axis under current mount (tracker frame)
            t = (W_ZUP @ np.asarray(raw))[:3, :3].T @ n      # table normal (tracker frame)
            if it == 0:
                tilt0[h] = float(np.degrees(np.arccos(np.clip(v @ t, -1.0, 1.0))))
            rpys[h] = [round(float(a), 2) for a in Rotation.from_matrix(R_rpy @ _rot_between(v, t).T).as_euler("xyz", degrees=True)]
    return {h: (rpys[h], tilt0[h]) for h in raw_by_hand}


def calibrate_origin_live(vive, cfg, duration=1.0, hands=("left", "right"), level=False):
    """-> (T_world_origin, raw_means, msg); with two hands + level also levels and saves the mount (updates cfg)"""
    raw = {h: mean_pose(Ts) for h, Ts in vive.sample_hands(hands, duration, min_n=10).items()}
    msg = None
    if level and len(raw) == 2:
        lv = level_mount(raw, cfg)
        worst = max(t for _, t in lv.values())
        if worst > MAX_LEVEL_TILT_DEG:
            msg = "mount not leveled: gripper tilted %.1f deg (> %.0f). Stand both grippers flat on the table." % (worst, MAX_LEVEL_TILT_DEG)
        else:
            for h, (rpy, _) in lv.items():
                save_mount(cfg, h, rpy)
            msg = "mount leveled to table (tilt %s)" % ", ".join("%s %.1f deg" % (h, t) for h, (_, t) in lv.items())
    return origin_from_raw(raw, cfg), raw, msg


def calib_hash(cfg):
    lat = {c: float((v or {}).get("latency_ms", 0.0)) for c, v in (cfg.get("cameras") or {}).items()}
    gm = cfg.get("gripper_model") or {}
    d = {"align": cfg.get("align") or {}, "gripper_model": {k: gm.get(k) for k in GRIPPER_ALIGN_KEYS}, "latency_ms": lat}
    return hashlib.sha1(json.dumps(d, sort_keys=True, default=str).encode()).hexdigest()[:12]


def gap_ok(t_src, t_q, max_gap_ns):
    """per t_q: inside t_src range and the surrounding samples are <= max_gap_ns apart"""
    if len(t_src) < 2:
        return np.zeros(len(t_q), bool)
    i1 = np.clip(np.searchsorted(t_src, t_q, side="left"), 1, len(t_src) - 1)
    return (t_q >= t_src[0]) & (t_q <= t_src[-1]) & (t_src[i1] - t_src[i1 - 1] <= max_gap_ns)


def interp_pose(t_src, T_src, valid_src, t_q, max_gap_ns):
    tv, Tv = t_src[valid_src], T_src[valid_src]
    n = len(t_q)
    out = np.tile(np.eye(4), (n, 1, 1))
    if len(tv) < 2:
        return out, np.zeros(n, bool)
    i1 = np.clip(np.searchsorted(tv, t_q, side="left"), 1, len(tv) - 1)
    i0 = i1 - 1
    gap = tv[i1] - tv[i0]
    ok = gap_ok(tv, t_q, max_gap_ns)
    a = np.where(gap > 0, (t_q - tv[i0]) / np.maximum(gap, 1), 0.0).clip(0, 1)
    out[:, :3, 3] = Tv[i0, :3, 3] + a[:, None] * (Tv[i1, :3, 3] - Tv[i0, :3, 3])
    r0 = Rotation.from_matrix(Tv[i0, :3, :3])
    rel = (r0.inv() * Rotation.from_matrix(Tv[i1, :3, :3])).as_rotvec()
    out[:, :3, :3] = (r0 * Rotation.from_rotvec(rel * a[:, None])).as_matrix()
    return out, ok


def nearest(t_src, t_q):
    j = np.searchsorted(t_src, t_q).clip(1, len(t_src) - 1)
    return np.where(np.abs(t_src[j - 1] - t_q) <= np.abs(t_src[j] - t_q), j - 1, j)


def rot6d(R):
    """6D = first two rows of R, flattened. same convention as GR00T pose.py _matrix_to_rot6d"""
    return R[..., :2, :].reshape(R.shape[:-2] + (6,))


def state_names(hands):
    return ["%s_%s" % (h, k) for h in hands for k in ["x", "y", "z", "r00", "r01", "r02", "r10", "r11", "r12", "grip"]]


def align_episode(ep_dir, cfg, trk=None, grp=None, cams=None, meta=None):
    acfg = cfg.get("align", {})
    cams_cfg = cfg.get("cameras") or {}
    if not cams_cfg:
        raise ValueError("episode recorded without cameras (no time base for alignment)")
    master = acfg.get("master_camera")
    if master not in cams_cfg:
        master = list(cams_cfg)[0]
    fps = float(acfg.get("fps", 30))
    max_gap = int(float(acfg.get("max_pose_gap_ms", 50)) * 1e6)
    hands = cfg.get("hands", ["left", "right"])
    dxl_hands = (cfg.get("dynamixel") or {}).get("hands") or {}

    cams = cams if cams is not None else {c: io.load_camera(ep_dir, c) for c in cams_cfg}
    for c in cams:
        if len(cams[c]["t"]) < 2:
            raise ValueError("camera %s has %d frames (video recording failed)" % (c, len(cams[c]["t"])))
        cams[c]["tc"] = cams[c]["t"] - int(float(cams_cfg[c].get("latency_ms", 0.0)) * 1e6)
    trk = trk if trk is not None else io.load_tracker(ep_dir)
    grp = grp if grp is not None else io.load_gripper(ep_dir)

    # t_s = real timestamp of the master frame picked per tick; pose and gripper are sampled there
    tm = cams[master]["tc"]
    step = int(round(1e9 / fps))
    t_q = np.arange(tm[0], tm[-1] + 1, step, dtype=np.int64)
    t_s = tm[nearest(tm, t_q)]
    n = len(t_q)
    valid = np.ones(n, bool)

    out = {"t_ns": t_q, "fps": fps}
    cam_hz = []
    for c in cams:
        tc = cams[c]["tc"]
        cam_hz.append((len(tc) - 1) / max((tc[-1] - tc[0]) * 1e-9, 1e-6) if len(tc) > 1 else 0.0)
        # judge drops by gap vs median period: nearest-frame offset can't tell 1 from 2 drops (depends on phase)
        period = max(float(np.median(np.diff(tc))) if len(tc) > 1 else step, step)
        out["frame_" + c] = cams[c]["idx"][nearest(tc, t_s)]
        ok = gap_ok(tc, t_q, MAX_FRAME_GAP * period)
        out["cam_ok_" + c] = ok
        valid &= ok
    out["cam_hz"] = np.array(cam_hz)
    out["cam_names"] = np.array(list(cams))

    tcp = tcp_offsets(cfg)
    for h in hands:
        if h not in trk:
            valid[:] = False
            out["T_" + h] = np.tile(np.eye(4), (n, 1, 1))
            continue
        T, ok = interp_pose(trk[h]["t"], trk[h]["T"], trk[h]["valid"], t_s, max_gap)
        out["T_" + h] = raw_to_world(T, tcp[h])
        out["pose_ok_" + h] = ok
        valid &= ok

    # grip = measured position (state), grip_cmd = trigger command (action); 0 open .. 1 closed
    for h in hands:
        if h not in grp:
            out["grip_" + h] = out["grip_cmd_" + h] = np.zeros(n)
            valid[:] = False
            continue
        g, hc = grp[h], dxl_hands.get(h, {})
        ok = gap_ok(g["t"], t_s, max_gap)   # gaps from read failures are invalid, not interpolated
        out["grip_" + h] = fraction(hc, "gripper", np.interp(t_s, g["t"], g["pos"]))
        out["grip_cmd_" + h] = fraction(hc, "gripper", np.interp(t_s, g["t"], g["goal"]))
        out["grip_ok_" + h] = ok
        valid &= ok

    o = (meta or io.load_meta(ep_dir)).get("origin")
    T_o = None
    if o is None:
        print("  [warn] %s: no saved origin -> using SteamVR origin, excluded from to_gr00t (set with BT button double-click or 'o' while collecting)" % ep_dir)
    else:
        try:
            T_o = origin_from_raw({h: np.array(T) for h, T in o["raw_tracker_mean_steamvr"].items()}, cfg)
        except ValueError as e:
            print("  [warn] %s: origin recompute failed (%s) -> using saved origin" % (ep_dir, e))
            T_o = np.array(o["T_world_origin"])
    if T_o is not None:
        inv = np.linalg.inv(T_o)
        for h in hands:
            out["T_" + h] = inv @ out["T_" + h]
    out["origin_mode"] = "calib" if T_o is not None else "none"

    out["valid"] = valid
    out["format_version"] = ALIGNED_VERSION
    out["calib_hash"] = calib_hash(cfg)
    return out


def trim_valid(valid):
    """[s, e) with leading/trailing invalid trimmed, and the invalid count inside"""
    idx = np.flatnonzero(valid)
    if len(idx) == 0:
        return 0, 0, int(len(valid))
    s, e = idx[0], idx[-1] + 1
    return int(s), int(e), int((~valid[s:e]).sum())
