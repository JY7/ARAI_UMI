"""Rerun viewer: 3D grippers (origin frame, z-up, hand pose = TCP) + wrist cameras + gripper plots"""
import os
import subprocess
import threading
import time

import cv2
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import rerun_cli

from .align import tcp_in_origin
from .dxl import fraction
from .episode import load_meta
from .gripper import load_model

CAD_COLORS = {"left": 0x93C5FDFF, "right": 0xFCA5A5FF}
TRAIL_COLORS = {"left": 0x3B82F6FF, "right": 0xEF4444FF}
LIVE_CAM_FPS = 15   # collect --viz display rate; higher makes the viewer decoder fall behind
GRAY = 0x9CA3AFFF   # tracking lost
AXIS_COLORS = {"x": 0xEF4444FF, "y": 0x22C55EFF, "z": 0x3B82F6FF}
TABLE_COLOR, FLOOR_COLOR = 0xD6B98C80, 0x9CA3AF33


def _tf(T):
    return rr.Transform3D(translation=T[:3, 3], mat3x3=T[:3, :3])


class GripperViz:
    def __init__(self, app, cfg, hands, cams, live=False, name=None):
        """live (collect): viewer is a child process closed with us; else (replay) the window stays open"""
        self.m, self.hands, self._valid = load_model(cfg), list(hands), {}
        self.table = cfg.get("table") or {}
        self.z_floor = 0.0   # added to plotted z (table height while the table is shown)
        rr.init(app, recording_id=name)
        cols = [rrb.Spatial3DView(origin="world", eye_controls=rrb.EyeControls3D(position=[-0.5, 0.25, 0.35], look_target=[0, 0, 0], eye_up=[0, 0, 1])),
                rrb.Vertical(*[rrb.Spatial2DView(origin="cam/" + c) for c in cams])]
        if live:
            cols.insert(0, rrb.TextLogView(origin="episodes", name="episodes",
                                           columns=rrb.TextLogColumns(timeline_columns=[rrb.encodings.TimelineColumn("time", visible=False)], text_log_columns=["body"])))
        graphs = [rrb.TimeSeriesView(origin="grip")] + [rrb.TimeSeriesView(origin="pose/" + h, name="%s position [m] (z: floor)" % h) for h in hands]
        bp = rrb.Blueprint(rrb.Vertical(rrb.Horizontal(*cols, column_shares=[1, 3, 2][-len(cols):]),
                                        rrb.Horizontal(*graphs), row_shares=[3, 1]),
            rrb.BlueprintPanel(state="collapsed" if live else "expanded"),
            rrb.SelectionPanel(state="collapsed"), rrb.TimePanel(state="collapsed"))
        self.viewer = None
        if live:   # run the viewer binary directly: the .venv/bin/rerun wrapper doesn't forward signals
            self.viewer = subprocess.Popen([os.path.join(os.path.dirname(rerun_cli.__file__), "rerun"), "--memory-limit", "1GB"],   # cameras add ~150 MB/min; live only needs the last minutes
                                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            rr.connect_grpc()
        else:
            rr.spawn(memory_limit="4GB")
        if name:
            rr.send_recording_name(name)
        rr.send_blueprint(bp)   # default_blueprint is ignored if the viewer has a saved layout
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.log("world/origin", rr.TransformAxes3D(0.15), static=True)
        m = self.m
        g_open = m.fk(1.0)
        # only moving parts are re-logged per frame
        self.moving = [i for i, (l, _, _) in enumerate(m.visuals)
                       if not np.allclose(m.fk(0.0)[l], g_open[l], atol=1e-6)
                       or not np.allclose(m.fk(0.5, 0.0)[l], m.fk(0.5, 1.0)[l], atol=1e-6)]
        for h in self.hands:
            rr.log("world/%s" % h, rr.TransformAxes3D(0.05), static=True)
            rr.log("grip/%s/gripper" % h, rr.SeriesLines(colors=TRAIL_COLORS[h], names="%s gripper" % h), static=True)
            rr.log("grip/%s/trigger" % h, rr.SeriesLines(colors=CAD_COLORS[h], names="%s trigger" % h), static=True)
            for a, c in AXIS_COLORS.items():
                rr.log("pose/%s/%s" % (h, a), rr.SeriesLines(colors=c, names=a), static=True)
            rr.log("world/%s/model" % h, _tf(np.linalg.inv(m.T_body_tcp)), static=True)
            rr.log("world/%s/model/tracker" % h, _tf(m.T_body_tracker(h)), rr.TransformAxes3D(0.04), static=True)
            for i, (l, path, To) in enumerate(m.visuals):
                rr.log("world/%s/model/v%02d" % (h, i), rr.Asset3D(path=path), static=True)
                if i not in self.moving:
                    rr.log("world/%s/model/v%02d" % (h, i), _tf(g_open[l] @ To), static=True)

    def set(self, h, T, grip=None, trigger=None, valid=True):
        """T: TCP pose (4x4), grip: 0 open .. 1 closed, trigger: 0 .. 1 (both None = no gripper data)"""
        rr.log("world/%s" % h, _tf(T))
        if valid:   # leave plot gaps while tracking is lost (don't plot the home pose)
            for a, v in zip(AXIS_COLORS, T[:3, 3]):
                rr.log("pose/%s/%s" % (h, a), rr.Scalars(v + (self.z_floor if a == "z" else 0.0)))
        if self._valid.get(h) != valid:
            self._valid[h] = valid
            for i in range(len(self.m.visuals)):
                rr.log("world/%s/model/v%02d" % (h, i), rr.Asset3D.from_fields(albedo_factor=CAD_COLORS[h] if valid else GRAY))
        if grip is None:
            return
        rr.log("grip/%s/gripper" % h, rr.Scalars(grip))
        rr.log("grip/%s/trigger" % h, rr.Scalars(trigger))
        fk = self.m.fk(1.0 - grip, trigger)
        for i in self.moving:
            l, _, To = self.m.visuals[i]
            rr.log("world/%s/model/v%02d" % (h, i), _tf(fk[l] @ To))

    def show_table(self, on):
        """Show/clear the table (top z = 0, origin = front edge midpoint, x inward); only valid with an origin"""
        if not on:
            self.z_floor = 0.0
            rr.log("world/table", rr.Clear(recursive=True), static=True)
            return
        h = float(self.table.get("height", 0.73))
        d, w = (float(v) for v in self.table.get("size_xy", [0.8, 1.6]))
        self.z_floor = h
        rr.log("world/table/top", rr.Boxes3D(centers=[[d / 2, 0, -0.01]], half_sizes=[[d / 2, w / 2, 0.01]],
                                             colors=TABLE_COLOR, fill_mode="solid"), static=True)
        legs = [[[x, y, -0.02], [x, y, -h]] for x in (0.05, d - 0.05) for y in (w / 2 - 0.05, -w / 2 + 0.05)]
        rr.log("world/table/legs", rr.LineStrips3D(legs, colors=0x8B6B4AFF, radii=0.015), static=True)
        rr.log("world/table/floor", rr.Boxes3D(centers=[[d / 2, 0, -h]], half_sizes=[[1.5, 1.5, 0.001]],
                                               colors=FLOOR_COLOR, fill_mode="solid"), static=True)

    def close(self):
        if self.viewer:
            rr.disconnect()
            self.viewer.terminate()

    def trail(self, h, xyz):
        rr.log("world/trail/%s" % h, rr.LineStrips3D([xyz], colors=TRAIL_COLORS[h]), static=True)


def home(h, hands):
    """Pose without tracker data: upright (180° about x), hands spread along y (y = left)"""
    T = np.diag([1.0, -1.0, -1.0, 1.0])
    T[1, 3] = {"left": 0.15, "right": -0.15}[h] if len(hands) > 1 else 0.0
    return T


def log_image(cam, f):
    """f: JPEG bytes or BGR array -> half-size JPEG (640x360: ~1/4 viewer load; recording stays full res)"""
    img = (cv2.imdecode(np.frombuffer(f, np.uint8), cv2.IMREAD_REDUCED_COLOR_2) if isinstance(f, bytes)   # decodes at 1/2: fast
           else cv2.resize(f, (f.shape[1] // 2, f.shape[0] // 2), interpolation=cv2.INTER_AREA))
    ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80]) if img is not None else (False, None)
    if ok:
        rr.log("cam/" + cam, rr.EncodedImage(contents=enc.tobytes(), media_type="image/jpeg"))


class LiveViz(threading.Thread):
    def __init__(self, cfg, vive, dxl, cams, rec):
        super().__init__(daemon=True)
        self.vive, self.dxl, self.cams, self.rec, self.cfg = vive, dxl, cams, rec, cfg
        self.dxl_hands = (cfg.get("dynamixel") or {}).get("hands") or {}
        self.viz = GripperViz("umi_collect", cfg, cfg["hands"], [c.name for c in cams], live=True)
        self._halt = threading.Event()
        self._has_origin = None
        for d in sorted(os.listdir(rec.root)):
            p = os.path.join(rec.root, d)
            if "_ep" in d and os.path.exists(os.path.join(p, "meta.json")):
                self.add_episode(d.replace("_DISCARDED", "  DISCARDED") if d.endswith("_DISCARDED")
                                 else "%s  %.1fs" % (d, load_meta(p)["duration_s"]))

    @staticmethod
    def add_episode(text):
        rr.set_time("time", timestamp=time.time())   # set_time is per thread
        rr.log("episodes", rr.TextLog(text))

    def stop(self):
        self._halt.set()
        self.viz.close()

    def run(self):
        shown, shown_t = {}, {}
        while not self._halt.wait(1 / 30):
            rr.set_time("time", timestamp=time.time())
            o = self.rec.origin
            if self._has_origin != (o is not None):
                self._has_origin = o is not None
                self.viz.show_table(self._has_origin)
            for h in self.viz.hands:
                T = self.vive.latest_T.get(h) if self.vive else None
                d = self.dxl.latest.get(h) if self.dxl else None
                hc = self.dxl_hands.get(h, {})
                self.viz.set(h, home(h, self.viz.hands) if T is None else tcp_in_origin(self.cfg, o, h, T),
                             None if d is None else fraction(hc, "gripper", d["pos"]),
                             None if d is None else fraction(hc, "trigger", d["trigger"]),
                             self.vive.valid.get(h, False) if self.vive else True)
            for c in self.cams:
                f = c.latest
                now = time.monotonic()
                if (f is not None and shown.get(c.name) is not f
                        and now - shown_t.get(c.name, 0.0) >= 1 / LIVE_CAM_FPS - 0.005):
                    shown[c.name], shown_t[c.name] = f, now
                    log_image(c.name, f)
