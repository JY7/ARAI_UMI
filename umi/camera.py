"""USB UVC cameras (V4L2, MJPG): raw MJPG saved without re-encoding, decoded only if unavailable.
t_ns = host time right after read(), dev_ts_ms = V4L2 buffer time (used for alignment)"""
import time

import cv2
import numpy as np

from .source import SourceThread

NON_JPEG_STREAK = 3     # this many non-JPEG frames in a row -> switch to decode-and-save mode
CONTROLS = (            # (config key, auto prop, (auto, manual) values, value prop); values are V4L2 raw, null = camera auto
    ("exposure", cv2.CAP_PROP_AUTO_EXPOSURE, (3, 1), cv2.CAP_PROP_EXPOSURE),
    ("wb_temp", cv2.CAP_PROP_AUTO_WB, (1, 0), cv2.CAP_PROP_WB_TEMPERATURE),
    ("gain", None, None, cv2.CAP_PROP_GAIN),
    ("gamma", None, None, cv2.CAP_PROP_GAMMA),
)


def open_capture(cfg, raw_mjpeg=False):
    """(VideoCapture, warning or None) from a config.yaml cameras.<name> entry; also used on the robot"""
    dev = cfg["device"]
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError("cannot open %s" % dev)
    size = (int(cfg.get("width", 1280)), int(cfg.get("height", 720)))
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    cap.set(cv2.CAP_PROP_FPS, float(cfg.get("fps", 30)))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)   # 1 makes V4L2 skip every other frame (30 -> 15 fps)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0 if raw_mjpeg else 1)   # 0: raw MJPG
    warning = None
    for key, auto, modes, prop in CONTROLS:
        v = cfg.get(key)
        if auto is not None:
            cap.set(auto, modes[v is not None])
        if v is not None and not cap.set(prop, float(v)):
            warning = "cannot set %s=%s" % (key, v)
    got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if got != size:
        warning = "requested %dx%d, got %dx%d" % (*size, *got)
    return cap, warning


class CameraSource(SourceThread):
    def __init__(self, name, cfg, recorder):
        super().__init__(label="%s camera" % name)
        self.name = name
        self.cfg = cfg
        self.rec = recorder
        self.fps = float(cfg.get("fps", 30))
        self.latest = None
        self.warning = None
        self.raw_mjpeg = True
        self.corrupt = 0
        self._non_jpeg = 0
        self.cap = None

    def _open(self):
        self.cap, warning = open_capture(self.cfg, self.raw_mjpeg)
        self.warning = warning or self.warning

    def _run(self):
        try:
            self._open()
        except Exception as e:
            self.error = str(e)
            return
        self._reset_clock()
        try:
            while not self._halt.is_set():
                ok, frame = self.cap.read()
                t = time.monotonic_ns()
                if not ok:
                    self.error, self.rate = "read failed", 0.0
                    time.sleep(0.01)
                    continue
                self.error = None
                dev_ts = self.cap.get(cv2.CAP_PROP_POS_MSEC)
                if self.raw_mjpeg:
                    frame = self._as_jpeg(frame)
                    if frame is None:
                        continue
                elif frame.ndim != 3:   # driver still not decoding after reopen -> don't save broken images
                    self.error, self.rate = "frame not decoded (shape %s, check camera format)" % (frame.shape,), 0.0
                    time.sleep(0.01)
                    continue
                self.latest = frame
                self.rec.log_image(self.name, t, frame, dev_ts)
                self.tick()
        finally:
            if self.cap is not None:
                self.cap.release()

    def _as_jpeg(self, frame):
        """Complete JPEG -> bytes, else None (frame dropped)"""
        if frame.ndim <= 2 and frame.dtype == np.uint8 and (frame.ndim == 1 or frame.shape[0] == 1):
            b = frame.tobytes()
            if b[:2] == b"\xff\xd8":
                self._non_jpeg = 0
                b = b.rstrip(b"\x00")   # some cameras pad zeros after FFD9
                if b[-2:] == b"\xff\xd9":
                    return b
                self.corrupt += 1
                self.rec.log_corrupt(self.name)
                return None
        self._non_jpeg += 1
        if self._non_jpeg < NON_JPEG_STREAK:
            return None
        self.raw_mjpeg = False
        self.warning = "raw MJPG not available, saving re-encoded JPEG"
        # CONVERT_RGB may not apply mid-stream on V4L2 -> reopen (retry: device can be busy right after close)
        self.cap.release()
        self.cap = None
        for k in range(3):
            try:
                self._open()
                break
            except RuntimeError:
                if k == 2:
                    raise
                time.sleep(0.3)
        return None
