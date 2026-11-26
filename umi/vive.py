"""Vive Tracker pose capture (openvr): one pose query per tick, raw 3x4 matrices logged."""
import subprocess
import time

import numpy as np

from .source import SourceThread

CLASS_NAMES = {1: "HMD", 2: "Controller", 3: "Tracker", 4: "BaseStation"}   # openvr.TrackedDeviceClass_*


def _vrserver_running():
    return subprocess.run(["pgrep", "-x", "vrserver"], stdout=subprocess.DEVNULL).returncode == 0


def _openvr_init(ov, timeout=90.0):
    """start SteamVR via Steam first: a vrserver spawned by openvr runs outside the Steam runtime (fails on 20.04, no libpipewire-0.3)"""
    if not _vrserver_running():
        print("SteamVR not running -> starting it via Steam (up to %ds)" % timeout)
        subprocess.Popen(["steam", "steam://rungameid/250820"], stdin=subprocess.DEVNULL,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        t_end = time.monotonic() + timeout
        while not _vrserver_running():
            if time.monotonic() > t_end:
                raise RuntimeError("SteamVR did not start in %ds (logged in to Steam?)" % timeout)
            time.sleep(0.5)
        time.sleep(3.0)   # wait for IPC after process start (else openvr spawns its own vrserver)
    return ov.init(ov.VRApplication_Other)


def _devices(ov, vr):
    for i in range(ov.k_unMaxTrackedDeviceCount):
        cls = vr.getTrackedDeviceClass(i)
        if cls != ov.TrackedDeviceClass_Invalid:
            yield i, cls, vr.getStringTrackedDeviceProperty(i, ov.Prop_SerialNumber_String)


class ViveSource(SourceThread):
    def __init__(self, cfg, recorder=None):
        super().__init__(1.0 / float(cfg.get("rate_hz", 200)), "vive")
        self.rec = recorder
        self.serials = dict(cfg.get("serials") or {})  # hand -> serial
        self.valid = {}
        self.latest_T = {}     # hand -> raw SteamVR 4x4, for calibration
        self._index = {}
        self.vr = self.openvr = None

    @staticmethod
    def list_devices():
        import openvr
        vr = _openvr_init(openvr)
        out = [(i, CLASS_NAMES.get(c, str(c)), s) for i, c, s in _devices(openvr, vr)]
        openvr.shutdown()
        return out

    def _refresh_index(self):
        by_serial = {s: i for i, _, s in _devices(self.openvr, self.vr)}
        self._index = {h: by_serial[s] for h, s in self.serials.items() if s in by_serial}
        missing = [h for h in self.serials if h not in self._index]
        self.error = ("tracker not found: %s" % missing) if missing else None

    def sample(self, hand, duration, min_n=10, period=0.01):
        return self.sample_hands([hand], duration, min_n, period)[hand]

    def sample_hands(self, hands, duration, min_n=10, period=0.01):
        """valid raw SteamVR 4x4 poses per hand, sampled together over duration seconds"""
        Ts, t_end = {h: [] for h in hands}, time.monotonic() + duration
        while time.monotonic() < t_end:
            for h in hands:
                T = self.latest_T.get(h)
                if T is not None and self.valid.get(h):
                    Ts[h].append(T.copy())
            time.sleep(period)
        for h in hands:
            if len(Ts[h]) < min_n:
                raise RuntimeError("tracker %s: too few valid samples (%d). Check SteamVR/tracker" % (h, len(Ts[h])))
        return Ts

    def _run(self):
        try:
            import openvr
            self.openvr = openvr
            self.vr = _openvr_init(openvr)
            self._refresh_index()
        except Exception as e:
            self.error = "openvr init failed: %s" % e
            return
        self._reset_clock()
        last_refresh = time.monotonic()
        try:
            while not self._halt.is_set():
                t = time.monotonic_ns()
                for row in self._read(t):
                    if self.rec is not None:
                        self.rec.log_tracker(row)
                    self.valid[row[1]] = bool(row[2])
                    T = np.eye(4)
                    T[:3, :] = np.array(row[4:16]).reshape(3, 4)
                    self.latest_T[row[1]] = T
                if self.error and time.monotonic() - last_refresh > 2.0:
                    self._refresh_index()  # pick up trackers switched on late
                    last_refresh = time.monotonic()
                self.tick()
        finally:
            self.valid = dict.fromkeys(self.valid, False)   # status line shows tracker lost after stop
            if self.vr is not None:
                self.openvr.shutdown()

    def _read(self, t):
        ov = self.openvr
        poses = self.vr.getDeviceToAbsoluteTrackingPose(ov.TrackingUniverseStanding, 0, ov.k_unMaxTrackedDeviceCount)
        rows = []
        for hand, i in self._index.items():
            p = poses[i]
            m, v, w = p.mDeviceToAbsoluteTracking.m, p.vVelocity.v, p.vAngularVelocity.v
            rows.append([t, hand, int(bool(p.bPoseIsValid)), int(p.eTrackingResult)]
                        + [float(m[r][c]) for r in range(3) for c in range(4)]
                        + [float(x) for x in (v[0], v[1], v[2], w[0], w[1], w[2])])
        return rows
