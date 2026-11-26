#!/usr/bin/env python3
"""Replay episodes in one Rerun window (3D + wrist video + gripper), aligned as in check_episode.

  python3 replay.py                     # all episodes under data/ -> pick from the Rerun list on the left
  python3 replay.py cup                 # data/cup (same folder as collect --data cup)
  python3 replay.py cup/<episode> ...   # selected ones. absolute/~ paths as is

Play, speed, loop and frame stepping: Rerun timeline at the bottom.
"""
import argparse
import os
import sys
import time

import rerun as rr

from umi import episode as io
from umi.align import align_episode
from umi.viz import GripperViz, home


def replay(ep, args):
    meta = io.load_meta(ep)
    cfg = io.effective_config(meta["config"], io.load_current_config(), args.use_current_mount)
    hands = list(cfg.get("hands", ["left", "right"]))
    cams = list(cfg.get("cameras") or {})
    if not cams:
        print("%s: recorded without cameras (component test) -> skipped" % meta["name"])
        return
    al = align_episode(ep, cfg)
    t_s = (al["t_ns"] - al["t_ns"][0]) * 1e-9
    hands = [h for h in hands if "pose_ok_" + h in al or "grip_ok_" + h in al]

    viz = GripperViz("umi_replay", cfg, hands, cams, name=meta["name"])
    viz.show_table(al["origin_mode"] != "none")
    for h in hands:
        viz.trail(h, al["T_" + h][al["valid"], :3, 3])
    pts = {}
    for c in cams:
        mp4 = os.path.join(ep, "cam_" + c, io.VIDEO_NAME)
        if os.path.exists(mp4):
            v = rr.AssetVideo(path=mp4)
            rr.log("cam/" + c, v, static=True)
            pts[c] = v.read_frame_timestamps_nanos()
    for i, t in enumerate(t_s):
        rr.set_time("time", duration=t)
        for h in hands:
            viz.set(h, al["T_" + h][i] if "pose_ok_" + h in al else home(h, hands), al["grip_" + h][i], al["grip_cmd_" + h][i],   # trigger = command
                    bool(al.get("pose_ok_" + h, al["valid"])[i]))
        for c in cams:
            k = int(al["frame_" + c][i])
            rr.log("cam/" + c, rr.VideoFrameReference(nanoseconds=int(pts[c][k])) if c in pts
                   else rr.EncodedImage(path=io.frame_path(ep, c, k)))
    print("%s  %d frames, %.1fs, origin=%s, task=%r -> Rerun" % (meta["name"], len(t_s), t_s[-1], al.get("origin_mode"), meta["task"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="*", default=[""],
                    help="episode or folder under data/ (e.g. cup, cup/<episode>); absolute or ~ paths as is. default: data/")
    ap.add_argument("--use-current-mount", action="store_true", help="use current config tracker mount values (same as check_episode)")
    args = ap.parse_args()
    eps = [e for p in args.paths for e in io.episodes_in(io.data_path(p))]
    if not eps:
        sys.exit("no episodes: %s" % args.paths)
    for ep in eps:
        try:
            replay(ep, args)
        except Exception as e:
            print("%s: skipped (%s: %s)" % (os.path.basename(ep), type(e).__name__, e))
    # set all recordings playing from 0; only the visible one advances, so switching back resumes
    time.sleep(1)   # let the viewer receive the last data
    vc = rr.experimental.ViewerClient()
    for r in vc.viewer_state().recordings:
        if r.store_id.startswith("Recording:umi_replay:"):
            vc.set_time("time", duration=0, play=True, recording=r.store_id)


if __name__ == "__main__":
    main()
