#!/usr/bin/env python3
"""Align + quality check -> <ep>/aligned.npz, check.png. Exit code 1 on any failure.

  python3 check_episode.py              # all episodes under data/
  python3 check_episode.py cup          # data/cup (only that one if it's an episode folder). absolute/~ paths as is
"""
import argparse
import os
import sys

import numpy as np

from umi import episode as io
from umi.align import DEFAULT_ACTION_HORIZON, align_episode, trim_valid


def rate_stats(t):
    """(mean Hz, max gap ms) from ns timestamps."""
    if len(t) < 2 or t[-1] <= t[0]:
        return 0.0, 0.0
    return (len(t) - 1) / ((t[-1] - t[0]) * 1e-9), np.diff(t).max() * 1e-6


def frame_gaps(t):
    """Dropped frames; align tolerates single drops, 2+ in a row -> invalid."""
    if len(t) < 3:
        return 0
    d = np.diff(t)
    return int((d > 1.5 * np.median(d)).sum())


def check(ep_dir, plot=True, current_cfg=None, current_mount=False):
    meta = io.load_meta(ep_dir)
    # delete first so a failure can't leave stale results for conversion
    for f in ("aligned.npz", "check.png"):
        p = os.path.join(ep_dir, f)
        if os.path.exists(p):
            os.remove(p)
    cfg = io.effective_config(meta["config"], current_cfg, current_mount)
    print("== %s  (%.1fs, task=%r)" % (meta["name"], meta["duration_s"], meta["task"]))
    if current_cfg and not current_mount:
        rec_gm = meta["config"].get("gripper_model") or {}
        cur_gm = current_cfg.get("gripper_model") or {}
        diff = sorted({"%s.%s" % (k, h) for k in io.MOUNT_KEYS for h, v in (rec_gm.get(k) or {}).items()
                       if v is not None and v != (cur_gm.get(k) or {}).get(h)})
        if diff:
            print("  [info] using tracker mount values from recording time (differ from current config: %s). If you re-calibrated with the same mount, use --use-current-mount"
                  % ", ".join(diff))
    trk = io.load_tracker(ep_dir)
    for h, d in sorted(trk.items()):
        hz, gap = rate_stats(d["t"])
        # valid but result != Running_OK (200): possibly IMU-extrapolated pose (reported only, not filtered)
        not_ok = 100 * (d["valid"] & (d["result"] != 200)).mean()
        print("  tracker %-5s %6.1f Hz  max gap %6.1f ms  valid %5.1f%%  valid&result≠200 %4.1f%%"
              % (h, hz, gap, 100 * d["valid"].mean(), not_ok))
    grp = io.load_gripper(ep_dir)
    for h, d in sorted(grp.items()):
        hz, gap = rate_stats(d["t"])
        print("  gripper %-5s %6.1f Hz  max gap %6.1f ms  pos [%d, %d]" % (h, hz, gap, d["pos"].min(), d["pos"].max()))
    cams = {}
    for c in (cfg.get("cameras") or {}):
        cams[c] = io.load_camera(ep_dir, c)
        hz, gap = rate_stats(cams[c]["t"])
        print("  cam     %-5s %6.1f Hz  max gap %6.1f ms  frames %d  gaps %d  dropped %s  corrupt %s" % (
            c, hz, gap, len(cams[c]["t"]), frame_gaps(cams[c]["t"]),
            meta["dropped_frames"][c], meta["corrupt_frames"][c]))

    if not cams:
        print("  aligned: skipped (recorded without cameras - component test)")
        return None
    try:
        al = align_episode(ep_dir, cfg, trk, grp, cams, meta)
    except Exception as ex:
        print("  aligned: failed (%s)" % ex)
        return False
    al["current_mount"] = bool(current_mount)   # lets to_gr00t check calib_hash against the same mount values
    for c, hz in zip(al["cam_names"], al["cam_hz"]):
        want = float((cfg.get("cameras") or {}).get(str(c), {}).get("fps", al["fps"]))
        if hz < 0.9 * min(want, al["fps"]):
            print("  [warn] camera %s actual %.1fHz (set %.0f, align %.0f fps) - likely low-light auto exposure"
                  " (add light / set cameras.%s.exposure). Frames and poses repeated" % (c, hz, want, al["fps"], c))
    s, e, holes = trim_valid(al["valid"])
    print("  aligned: %d samples @ %.0f fps, usable [%d:%d], invalid inside %d" % (
        len(al["t_ns"]), al["fps"], s, e, holes))
    if e - s == 0:
        print("  [warn] no valid frames - check missing streams (tracker/gripper/camera). Will be excluded from conversion")
    elif holes:
        why = []
        for c in al["cam_names"]:
            k = int((~al["cam_ok_" + str(c)][s:e]).sum())
            if k:
                why.append("camera %s frame gaps %d" % (c, k))
        for h in cfg.get("hands", []):
            for key, what in (("pose_ok_", "tracker"), ("grip_ok_", "gripper")):
                if key + h in al:
                    k = int((~al[key + h][s:e]).sum())
                    if k:
                        why.append("%s %s lost %d" % (what, h, k))
        print("  [warn] %d invalid inside (%s) -> will be excluded by to_gr00t"
              % (holes, ", ".join(why) or "unknown cause"))
    if 0 < e - s < DEFAULT_ACTION_HORIZON:
        print("  [warn] usable length %d frames < %d (action horizon) -> will be excluded by to_gr00t" % (e - s, DEFAULT_ACTION_HORIZON))
    np.savez_compressed(os.path.join(ep_dir, "aligned.npz"), **al)

    if plot:
        try:
            save_plot(ep_dir, meta, cfg, al, cams, trk)
        except Exception as ex:
            print("  [warn] failed to create check.png (%s: %s)" % (type(ex).__name__, ex))
    return al


def save_plot(ep_dir, meta, cfg, al, cams, trk):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t0 = al["t_ns"][0]
    tt = (al["t_ns"] - t0) * 1e-9
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    for h in cfg.get("hands", []):
        if "T_" + h not in al:
            continue
        for k, lab in enumerate("xyz"):
            ax[0].plot(tt, al["T_" + h][:, k, 3], label="%s %s" % (h, lab))
        ax[1].plot(tt, al["grip_" + h], label=h)
    for c, d in cams.items():
        ax[2].plot((d["t"][1:] - t0) * 1e-9, np.diff(d["t"]) * 1e-6, ".", ms=3, label=c)
    for h, d in trk.items():
        ax[3].plot((d["t"] - t0) * 1e-9, d["valid"].astype(int), label="tracker " + h)
    ax[3].plot(tt, al["valid"].astype(int) * 0.9, label="aligned valid")
    ax[0].set_ylabel("TCP pos [m] (z-up)")
    ax[1].set_ylabel("gripper (0 open, 1 closed)")
    ax[2].set_ylabel("frame interval [ms]")
    ax[3].set_ylabel("valid")
    ax[3].set_xlabel("time [s]")
    for a in ax:
        a.legend(loc="upper right", fontsize=7)
        a.grid(alpha=0.3)
    fig.suptitle(meta["name"])
    fig.tight_layout()
    fig.savefig(os.path.join(ep_dir, "check.png"), dpi=90)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="",
                    help="episode or folder under data/ (e.g. cup, cup/<episode>); absolute or ~ paths as is. default: data/")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--use-current-mount", action="store_true",
                    help="use current config tracker mount values (tracker_frame_rpy_deg) instead of recording-time values (after re-calibrating the same mount)")
    args = ap.parse_args()
    cur = io.load_current_config()
    path = io.data_path(args.path)
    eps = io.episodes_in(path)
    failed = []
    for ep in eps:
        for c, r in io.encode_episode(ep).items():
            if r != "skip":
                print("  [mp4] %s cam_%s: %s" % (os.path.basename(ep), c, "JPG -> mp4 encoded" if r == "done" else "encode failed, JPGs kept (%s)" % r))
        try:
            ok = check(ep, plot=not args.no_plot, current_cfg=cur, current_mount=args.use_current_mount)
        except Exception as ex:
            print("  failed: %s (%s: %s)" % (ep, type(ex).__name__, ex))
            ok = False
        if ok is False:
            failed.append(ep)
    incomplete = io.list_incomplete(path)
    if incomplete:
        print("\n%d incomplete recordings (no meta.json: power loss / killed while recording -> automatically excluded):" % len(incomplete))
        for ep in incomplete:
            print("  " + ep)
    if failed:
        print("\n%d failed to align (no aligned.npz -> to_gr00t will stop):" % len(failed))
        for ep in failed:
            print("  " + ep)
        print("-> fix the cause, or append _DISCARDED to the folder name of episodes you will not use")
        sys.exit(1)


if __name__ == "__main__":
    main()
