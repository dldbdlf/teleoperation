"""Fuses camera-based wrist POSITION (AprilTag T3 on the dorsal hand, seen by the single D415 --
this project's designated online teleop camera per share/2026-11-12_roadmap.md) with glove-based
wrist ORIENTATION (the Rokoko Smartglove's `hub` sensor, see share/rokoko_glove/README.md) into a
live share/sim/baselines.py-compatible Frame.

Why fuse instead of using one or the other alone: the glove gives orientation with no position;
AprilTag gives a 6D pose but its own orientation estimate at a few cm tag size is comparatively
noisy and the whole point of adding the glove was to get a cleaner orientation signal. So: position
always comes from the tag, orientation comes from the glove once calibrated against the tag's own
orientation at one reference moment.

CALIBRATION (manual, click-triggered -- never automatic, per this project's established
convention): the glove's quaternion is expressed in the glove's own IMU reference frame (presumably
aligned to local gravity + magnetic north), not the camera's frame. Clicking CALIBRATE while the
tag is visible records R_offset = R_cam_tag @ inv(R_glove_hub) at that instant; afterwards every
frame's fused orientation is R_offset @ R_glove_hub(now). This is a one-shot alignment, not a
proper hand-eye calibration -- it is only as good as that single snapshot (both camera-tag noise
and glove noise contribute directly, uncancelled), and it silently assumes the tag and the glove's
hub stay rigidly aligned to the same hand frame for the rest of the session (re-click CALIBRATE if
the glove shifts on the hand, or after taking it off and putting it back on).

NOT YET DONE (see share/rokoko_glove/README.md and this project's own Frame fields): b_psi/b_kappa
(elbow-swivel belief) have no live source here -- this script always reports them as
(nan, 0.0), the same "unknown" convention canonical.py's pose-relaxed fallback already handles.
Wiring in T1/T2 (upper-arm/forearm) AprilTag tracking for psi is future work, not attempted here.

Usage:
    python3 camera_glove_fusion.py
Requires the D415 ROS2 camera node running as /cam_1/D415 (see share/apriltag_gt/README.md for the
launch command) and the glove connected via USB.
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

sys.path.insert(0, str(Path(__file__).resolve().parent))
from glove_reader import GloveReader  # noqa: E402

TAG_ID = 3            # T3, dorsal hand -- see share/apriltag_gt/tag_measurements.yaml
TAG_LEN_M = 0.04
HUB_STALE_S = 1.0      # treat the glove orientation as unavailable if no sample this recent


class FusionNode(Node):
    def __init__(self):
        super().__init__("camera_glove_fusion")
        self.bridge = CvBridge()
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11))
        self.frame = None
        self.info = None
        self.create_subscription(CameraInfo, "/cam_1/D415/color/camera_info",
                                  lambda m: setattr(self, "info", m), 1)
        self.create_subscription(Image, "/cam_1/D415/color/image_raw", self._on_image, 1)
        self.tag_pose = None  # (R, t) of the tag in camera frame, this instant, or None

    def _on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.tag_pose = None
        if self.info is None:
            return
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.det.detectMarkers(gray)
        if ids is None or TAG_ID not in ids.flatten():
            return
        idx = list(ids.flatten()).index(TAG_ID)
        s = TAG_LEN_M
        obj = np.array([[-s/2, s/2, 0], [s/2, s/2, 0], [s/2, -s/2, 0], [-s/2, -s/2, 0]], dtype=np.float32)
        K = np.array(self.info.k).reshape(3, 3)
        D = np.array(self.info.d)
        ok, rvec, tvec = cv2.solvePnP(obj, corners[idx], K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            R, _ = cv2.Rodrigues(rvec)
            self.tag_pose = (R, tvec.flatten())


def quat_wxyz_to_R(q):
    w, x, y, z = q
    n = (w*w + x*x + y*y + z*z) ** 0.5
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),     2*(x*z+y*w)],
        [2*(x*y+z*w),     1-2*(x*x+z*z),   2*(y*z-x*w)],
        [2*(x*z-y*w),     2*(y*z+x*w),     1-2*(x*x+y*y)],
    ])


# ---- GUI state shared between the mouse callback and the render loop (no auto-capture) ----
R_offset = None
status_lines = ["Not calibrated. Show tag 3 to the camera, wear the glove, then click CALIBRATE."]
BTN = {"calibrate": (20, 20, 220, 60), "clear": (240, 20, 380, 60)}


def draw_button(img, rect, label, color):
    x1, y1, x2, y2 = rect
    cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
    cv2.rectangle(img, (x1, y1), (x2, y2), (200, 200, 200), 1)
    cv2.putText(img, label, (x1+8, y1+27), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)


def on_mouse(event, x, y, flags, ctx):
    global R_offset
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    node, reader = ctx
    for name, (x1, y1, x2, y2) in BTN.items():
        if not (x1 <= x <= x2 and y1 <= y <= y2):
            continue
        if name == "clear":
            R_offset = None
            status_lines.append("Calibration cleared.")
        elif name == "calibrate":
            hub = reader.sensors["hub"]
            hub_fresh = hub.t_host and (time.time() - hub.t_host) < HUB_STALE_S
            if node.tag_pose is None:
                status_lines.append("CALIBRATE failed: tag 3 not visible right now.")
            elif not hub_fresh:
                status_lines.append("CALIBRATE failed: glove hub orientation is stale/unavailable.")
            else:
                R_cam_tag, _ = node.tag_pose
                R_glove = quat_wxyz_to_R(hub.quat_wxyz)
                R_offset = R_cam_tag @ R_glove.T
                status_lines.append("Calibrated: R_offset set from this instant's tag+glove pose.")
        del status_lines[:-6]


def main():
    rclpy.init()
    node = FusionNode()
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()

    reader = GloveReader()
    print(f"Glove on {reader.port}. Configuring stream...")
    reader.start()

    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera", 640, 480)
    cv2.moveWindow("Camera", 40, 60)
    cv2.namedWindow("Fusion", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Fusion", 760, 320)
    cv2.moveWindow("Fusion", 720, 60)
    cv2.setMouseCallback("Fusion", on_mouse, (node, reader))

    try:
        while True:
            if node.frame is not None:
                f = node.frame.copy()
                gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = node.det.detectMarkers(gray)
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(f, corners, ids)
                cv2.imshow("Camera", f)

            panel = np.full((320, 760, 3), 30, dtype=np.uint8)
            draw_button(panel, BTN["calibrate"], "CALIBRATE", (0, 110, 0))
            draw_button(panel, BTN["clear"], "CLEAR", (110, 0, 0))

            hub = reader.sensors["hub"]
            hub_age = time.time() - hub.t_host if hub.t_host else float("inf")
            tag_txt = "VISIBLE" if node.tag_pose is not None else "not visible"
            cv2.putText(panel, f"Tag 3: {tag_txt}   |   Glove hub age: {hub_age:.2f}s",
                        (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1, cv2.LINE_AA)

            if R_offset is not None and node.tag_pose is not None and hub_age < HUB_STALE_S:
                R_cam_tag, t_cam_tag = node.tag_pose
                R_glove = quat_wxyz_to_R(hub.quat_wxyz)
                b_R = R_offset @ R_glove
                b_W = t_cam_tag
                cv2.putText(panel, "FUSED Frame (live):", (20, 130),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(panel, f"b_W (m, camera frame) = {np.round(b_W, 4)}", (20, 160),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, cv2.LINE_AA)
                for i, row in enumerate(b_R):
                    cv2.putText(panel, f"b_R row{i} = {np.round(row, 3)}", (20, 190 + i*24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, cv2.LINE_AA)
                cv2.putText(panel, "b_psi = nan, b_kappa = 0.0  (no elbow source wired in yet)",
                            (20, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 255), 1, cv2.LINE_AA)
            else:
                cv2.putText(panel, "Not producing a fused Frame right now (see status below).",
                            (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150, 150, 150), 1, cv2.LINE_AA)

            for i, line in enumerate(status_lines[-3:]):
                cv2.putText(panel, line[:100], (20, 285 + i*16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)
            cv2.imshow("Fusion", panel)

            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
