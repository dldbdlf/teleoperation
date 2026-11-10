"""D4 prep: AprilTag pose -> joint center (S/E/W) estimation, using D435i (GT camera) only.

Convention (IMPORTANT, must match how tags were physically mounted):
  Each tag's local frame: origin at tag center, +x = tag's right (as printed),
  +y = tag's "up" (as printed, i.e. the generated PNG's top edge), +z = out of the
  tag surface toward the camera (same convention as extrinsic_calib_apriltag.py's
  solvePnP object points).

  Tags must be mounted with +y pointing toward the PROXIMAL joint (the joint closer
  to the torso):
    T0 (torso): +y -> shoulder (S)
    T1 (upper arm): +y -> shoulder (S), -y -> elbow (E)
    T2 (forearm):   +y -> elbow (E),    -y -> wrist (W)
    T3 (hand):      +y -> wrist (W)

  Joint center = tag_center + along_bone * (+-y_axis) - depth * z_axis
  (the "- depth * z_axis" term pushes the point from the skin surface inward,
  toward the limb's core/bone, since +z points OUT of the surface toward the camera)

Usage:
    python3 compute_joint_centers.py --rate 5
Requires /cam_2/D435i/color/image_raw + camera_info running, and
tag_measurements.yaml + tag_joint_offsets.yaml in this directory.
"""
import argparse
import time
import yaml
import numpy as np
import cv2
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def pose_to_matrix(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T


def load_config():
    with open('/home/inha/apriltag_gt/tag_measurements.yaml') as f:
        sizes = yaml.safe_load(f)
    with open('/home/inha/apriltag_gt/tag_joint_offsets.yaml') as f:
        offsets = yaml.safe_load(f)
    return sizes, offsets


class JointCenterEstimator(Node):
    def __init__(self, sizes, offsets, rate_hz):
        super().__init__('compute_joint_centers')
        self.sizes = sizes
        self.offsets = offsets
        self.bridge = CvBridge()
        self.detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36H11))
        self.min_period = 1.0 / rate_hz
        self.last_print = 0.0

        # tag_id -> (name_in_measurements, name_in_offsets)
        self.tag_map = {
            0: ('T0_torso', 'T0_torso'),
            1: ('T1_upperarm', 'T1_upperarm'),
            2: ('T2_forearm', 'T2_forearm'),
            3: ('T3_hand', 'T3_hand'),
        }

        self.img_sub = self.create_subscription(
            Image, '/cam_2/D435i/color/image_raw', self.on_image, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/cam_2/D435i/color/camera_info', self.on_info, 10)
        self.overlay_pub = self.create_publisher(Image, '/gt_overlay/image', 10)
        self.K = None
        self.D = None
        self.get_logger().info('Waiting for D435i camera_info + images...')

    def on_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)

    def tag_pose(self, frame, corners_i, tag_id):
        size_cm = self.sizes[self.tag_map[tag_id][0]]['size_cm']
        L = size_cm / 100.0
        obj_points = np.array([
            [-L / 2,  L / 2, 0],
            [ L / 2,  L / 2, 0],
            [ L / 2, -L / 2, 0],
            [-L / 2, -L / 2, 0],
        ], dtype=np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_points, corners_i, self.K, self.D)
        if not ok:
            return None
        return pose_to_matrix(rvec, tvec)

    def joint_point(self, T_cam_tag, along_bone_cm, depth_cm, sign):
        p_local = np.array([0.0, sign * along_bone_cm / 100.0, -depth_cm / 100.0, 1.0])
        p_cam = T_cam_tag @ p_local
        return p_cam[:3]

    def project(self, p_cam):
        p = self.K @ p_cam
        if p[2] <= 0:
            return None
        return (int(p[0] / p[2]), int(p[1] / p[2]))

    def on_image(self, msg):
        if self.K is None:
            return
        now = time.time()
        if now - self.last_print < self.min_period:
            return
        self.last_print = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(frame)
        joints = {}  # name -> list of (source_tag, xyz)

        if ids is not None:
            ids = ids.flatten()
            for i, tag_id in enumerate(ids):
                if tag_id not in self.tag_map:
                    continue
                try:
                    self.process_tag(frame, corners[i], tag_id, joints)
                except Exception as e:
                    self.get_logger().warn(f'tag {tag_id}: skipping ({e})', throttle_duration_sec=5)

        self.publish_overlay(frame, msg, joints)

    def process_tag(self, frame, corners_i, tag_id, joints):
        off_name = self.tag_map[tag_id][1]
        off = self.offsets[off_name]
        if tag_id == 0:
            depth_cm = off['to_shoulder_S'].get('depth_cm')
        else:
            depth_cm = off.get('depth_cm')
        T_cam_tag = self.tag_pose(frame, corners_i, tag_id)
        if T_cam_tag is None or depth_cm is None:
            return

        if tag_id == 0:  # T0 torso -> S
            d = off['to_shoulder_S']['along_bone_cm']
            if d is not None:
                joints.setdefault('S', []).append(('T0', self.joint_point(T_cam_tag, d, depth_cm, +1)))
        elif tag_id == 1:  # T1 upper arm -> S, E
            d = off['to_shoulder_S']['along_bone_cm']
            if d is not None:
                joints.setdefault('S', []).append(('T1', self.joint_point(T_cam_tag, d, depth_cm, +1)))
            d = off['to_elbow_E']['along_bone_cm']
            if d is not None:
                joints.setdefault('E', []).append(('T1', self.joint_point(T_cam_tag, d, depth_cm, -1)))
        elif tag_id == 2:  # T2 forearm -> E, W (mount_sign flips if tag is physically mounted rotated 180deg)
            mount_sign = self.offsets.get('T2_forearm', {}).get('mount_sign', 1)
            d = off['to_elbow_E']['along_bone_cm']
            if d is not None:
                joints.setdefault('E', []).append(('T2', self.joint_point(T_cam_tag, d, depth_cm, +1 * mount_sign)))
            d = off['to_wrist_W']['along_bone_cm']
            if d is not None:
                joints.setdefault('W', []).append(('T2', self.joint_point(T_cam_tag, d, depth_cm, -1 * mount_sign)))
        elif tag_id == 3:  # T3 hand -> W
            d = off['to_wrist_W']['along_bone_cm']
            if d is not None:
                joints.setdefault('W', []).append(('T3', self.joint_point(T_cam_tag, d, depth_cm, +1)))

    def publish_overlay(self, frame, msg, joints):
        overlay = frame.copy()
        colors = {'S': (0, 0, 255), 'E': (0, 255, 0), 'W': (255, 0, 0)}
        log_parts = []
        for name, entries in joints.items():
            for src, xyz in entries:
                pt = self.project(xyz)
                if pt is not None:
                    cv2.circle(overlay, pt, 8, colors[name], -1)
                    cv2.putText(overlay, f'{name}({src})', (pt[0] + 10, pt[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[name], 2)
                log_parts.append(f'{name}({src})=[{xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f}]')
            if name == 'E' and len(entries) == 2:
                diff = np.linalg.norm(entries[0][1] - entries[1][1])
                log_parts.append(f'E cross-check |T1-T2|={diff*1000:.1f}mm')

        if log_parts:
            self.get_logger().info(' | '.join(log_parts))

        out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        out_msg.header = msg.header
        self.overlay_pub.publish(out_msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=5.0, help='processing rate (Hz)')
    args = ap.parse_args()

    sizes, offsets = load_config()
    rclpy.init()
    node = JointCenterEstimator(sizes, offsets, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()