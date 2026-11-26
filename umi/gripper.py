"""CAD gripper model (URDF) + tracker mount calibration.

body frame: +z fingers, +x finger opening, +y handle.
TCP: tcp_link when closed (fingertip center), axes x, y, z = body +z, +x, +y.
"""
import copy
import json
import os
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .episode import set_config_value

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # umi_collect/

# columns = TCP x, y, z axes in body frame
R_BODY_TCP = np.array([[0, 1, 0],
                       [0, 0, 1],
                       [1, 0, 0]], float)


def _T(origin_el):
    if origin_el is None:
        return np.eye(4)
    return rpy_T(np.degrees([float(v) for v in origin_el.get("rpy", "0 0 0").split()]),
                 [float(v) for v in origin_el.get("xyz", "0 0 0").split()])


def rpy_T(rpy_deg, xyz=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    T[:3, 3] = xyz
    return T


class GripperModel:
    def __init__(self, gcfg):
        self.cfg = gcfg
        urdf = gcfg.get("urdf", "assets/urdf/umi_arai.urdf")
        self.urdf_path = urdf if os.path.isabs(urdf) else os.path.join(HERE, urdf)
        if not os.path.exists(self.urdf_path):
            raise FileNotFoundError("URDF not found: %s (check config gripper_model.urdf and assets/)" % self.urdf_path)
        self.open_angle = float(gcfg.get("open_angle_rad", np.deg2rad(60.0)))
        self.trigger_angle = float(gcfg.get("trigger_angle_rad", 1.13446))
        self.finger_joints = {k: float(v) for k, v in (gcfg.get("finger_joint_coef") or {}).items()}
        self.trigger_joints = {k: float(v) for k, v in (gcfg.get("trigger_joint_coef") or {}).items()}
        self.trigger_offset = np.deg2rad(float(gcfg.get("trigger_offset_deg") or 0.0))
        self._parse()
        g0 = self.fk(0.0, 0.0)
        links = {k: gcfg.get(k, d) for k, d in (("tcp_link", "tcp"), ("tracker_link", "tracker"))}
        for k, l in links.items():
            if l not in g0:
                raise KeyError("link %s not in URDF (check config gripper_model.%s)" % (l, k))
        self.T_body_tcp = np.eye(4)
        self.T_body_tcp[:3, :3] = R_BODY_TCP
        self.T_body_tcp[:3, 3] = g0[links["tcp_link"]][:3, 3]
        self.T_body_tracker_link = g0[links["tracker_link"]]

    def _parse(self):
        root = ET.parse(self.urdf_path).getroot()
        self.joints = {}   # child -> joint
        for j in root.findall("joint"):
            ax = j.find("axis")
            axis = np.array([float(v) for v in (ax.get("xyz") if ax is not None else "0 0 1").split()])
            self.joints[j.find("child").get("link")] = dict(
                name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                T=_T(j.find("origin")), axis=axis / (np.linalg.norm(axis) or 1.0))
        self.visuals = []   # (link, mesh_path, T_link_visual)
        for l in root.findall("link"):
            for v in l.findall("visual"):
                m = v.find("geometry/mesh")
                if m is not None:
                    self.visuals.append((l.get("name"), self._mesh_path(m.get("filename")), _T(v.find("origin"))))
        self.links = [l.get("name") for l in root.findall("link")]

    def _mesh_path(self, fn):
        p = os.path.normpath(os.path.join(os.path.dirname(self.urdf_path), fn))
        if not os.path.exists(p):
            raise FileNotFoundError("mesh not found: %s" % p)
        return p

    def fk(self, opening, trigger=None):
        """body-frame 4x4 per link. opening: 0 closed .. 1 open, trigger: 0..1 (None -> 1 - opening)"""
        q = float(np.clip(opening, 0, 1)) * self.open_angle
        t = float(np.clip(1.0 - opening if trigger is None else trigger, 0, 1)) * self.trigger_angle
        vals = {n: s * q for n, s in self.finger_joints.items()}
        vals.update({n: s * t + self.trigger_offset for n, s in self.trigger_joints.items()})
        out = {}

        def get(link):
            if link not in out:
                j = self.joints.get(link)
                if j is None:
                    out[link] = np.eye(4)
                else:
                    M = get(j["parent"]) @ j["T"]
                    if j["type"] in ("revolute", "continuous"):
                        Rj = np.eye(4)
                        Rj[:3, :3] = Rotation.from_rotvec(j["axis"] * vals.get(j["name"], 0.0)).as_matrix()
                        M = M @ Rj
                    out[link] = M
            return out[link]

        for l in self.links:
            get(l)
        return out

    def tracker_to_tcp(self, hand=None):
        """tracker -> TCP: CAD + tracker_frame_rpy_deg"""
        rpy = (self.cfg.get("tracker_frame_rpy_deg") or {}).get(hand) or [0, 0, 0]
        return np.linalg.inv(self.T_body_tracker_link @ rpy_T(rpy)) @ self.T_body_tcp

    def T_body_tracker(self, hand=None):
        return self.T_body_tcp @ np.linalg.inv(self.tracker_to_tcp(hand))


_MODELS = {}


def load_model(cfg):
    """cached per config. must raise on failure, never fall back (data would get TCP = tracker position)"""
    g = cfg["gripper_model"]
    key = json.dumps(g, sort_keys=True)
    if key not in _MODELS:
        _MODELS[key] = GripperModel(copy.deepcopy(g))
    return _MODELS[key]


UP_STEAMVR = np.array([0.0, 1.0, 0.0])          # SteamVR standing universe: +y up
BODY_UP_A = np.array([0.0, 0.0, -1.0])          # body axis pointing up in mount pose A (fingers down)
BODY_UP_B = np.array([0.0, -1.0, 0.0])          # body axis pointing up in mount pose B (handle down)


def rotation_spread_deg(Rs):
    R = Rotation.from_matrix(np.asarray(Rs))
    return R.mean(), float(np.degrees(np.max(np.linalg.norm((R.mean().inv() * R).as_rotvec(), axis=1))))


def solve_mount(R_wt_A, R_wt_B, R_body_link=np.eye(3)):
    """tracker rotations (SteamVR world <- tracker) in poses A, B -> (tracker_frame_rpy_deg, angle between poses)"""
    uA, uB = R_wt_A.T @ UP_STEAMVR, R_wt_B.T @ UP_STEAMVR
    ang = float(np.degrees(np.arccos(np.clip(np.dot(uA, uB), -1, 1))))
    R_tb = Rotation.align_vectors(np.stack([uA, uB]), np.stack([BODY_UP_A, BODY_UP_B]))[0]  # body -> tracker
    R_link_tracker = R_body_link.T @ R_tb.as_matrix().T
    return Rotation.from_matrix(R_link_tracker).as_euler("xyz", degrees=True), ang


def save_mount(cfg, hand, rpy):
    """gripper_model.tracker_frame_rpy_deg.<hand> -> config.yaml and cfg"""
    rpy = [round(float(v), 2) for v in rpy]
    set_config_value(["gripper_model", "tracker_frame_rpy_deg", hand], "[%s]" % ", ".join("%.2f" % v for v in rpy))
    cfg["gripper_model"].setdefault("tracker_frame_rpy_deg", {})[hand] = rpy


def run_mount(cfg, vive, hands, beep):
    model = GripperModel(cfg.get("gripper_model") or {})
    for h in hands:
        print("\n=== %s gripper ===" % h)
        poses = {}
        for key, desc in (("A", "hold vertical with fingertips pointing down"),
                          ("B", "hold vertical with the handle pointing down (fingers level, pointing forward)")):
            input("[pose %s] %s -> hold still and press Enter (keep still for 1.5s)" % (key, desc))
            R, spread = rotation_spread_deg([T[:3, :3] for T in vive.sample(h, 1.5, min_n=20)])
            print("  measured (shake %.1f deg)%s" % (spread, "  [warn] too much shake, re-measure." if spread > 5 else ""))
            poses[key] = R.as_matrix()
            beep("ok")
        rpy, ang = solve_mount(poses["A"], poses["B"], model.T_body_tracker_link[:3, :3])
        print("  angle between poses %.1f deg (closer to 90 is better), tracker_frame_rpy_deg = %s" % (ang, np.round(rpy, 2).tolist()))
        if abs(ang - 90) > 10:
            print("  [warn] poses are not perpendicular. Re-measure following the pose instructions. (not saved)")
            continue
        save_mount(cfg, h, rpy)
        print("  saved to config.yaml  (tracker->TCP distance %.3fm)" % np.linalg.norm(model.tracker_to_tcp(h)[:3, 3]))

