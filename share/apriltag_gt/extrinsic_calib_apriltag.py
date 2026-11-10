"""D415 <-> D435i extrinsic re-calibration using AprilTag 36h11 (improved from the 2026-11-03
ArUco version, which measured z std = 44.7mm and was flagged as insufficient for GT use).

Usage:
    python3 extrinsic_calib_apriltag.py --tag-id 0 --tag-length-m 0.09 --n-samples 60

--tag-length-m MUST be the measured physical side length of the printed black square
(quiet-zone margin excluded), in meters. Using the nominal/requested size instead of a ruler
measurement is exactly the kind of error that produced the 44.7mm std dev last time.
"""
import argparse
import time

import numpy as np
import cv2
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from cv_bridge import CvBridge


def pose_to_matrix(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T


class ExtrinsicCalibAprilTag(Node):
    def __init__(self, tag_id, tag_length_m, n_samples):
        super().__init__('extrinsic_calib_apriltag')
        self.tag_id = tag_id
        self.n_samples = n_samples
        self.bridge = CvBridge()
        self.detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36H11))
        self.obj_points = np.array([
            [-tag_length_m / 2,  tag_length_m / 2, 0],
            [ tag_length_m / 2,  tag_length_m / 2, 0],
            [ tag_length_m / 2, -tag_length_m / 2, 0],
            [-tag_length_m / 2, -tag_length_m / 2, 0],
        ], dtype=np.float32)
        self.samples = []

        img1 = message_filters.Subscriber(self, Image, '/cam_1/D415/color/image_raw')
        info1 = message_filters.Subscriber(self, CameraInfo, '/cam_1/D415/color/camera_info')
        img2 = message_filters.Subscriber(self, Image, '/cam_2/D435i/color/image_raw')
        info2 = message_filters.Subscriber(self, CameraInfo, '/cam_2/D435i/color/camera_info')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [img1, info1, img2, info2], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.callback)
        self.get_logger().info(
            f'Waiting for AprilTag id={tag_id} (36h11, {tag_length_m*100:.1f}cm) '
            f'visible in BOTH cameras... move it through varied positions/angles while visible '
            f'to both.')

    def detect_pose(self, img_msg, info_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        K = np.array(info_msg.k).reshape(3, 3)
        D = np.array(info_msg.d)
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return None
        ids = ids.flatten()
        if self.tag_id not in ids:
            return None
        idx = list(ids).index(self.tag_id)
        ok, rvec, tvec = cv2.solvePnP(self.obj_points, corners[idx], K, D)
        if not ok:
            return None
        return pose_to_matrix(rvec, tvec)

    def callback(self, img1, info1, img2, info2):
        if len(self.samples) >= self.n_samples:
            return
        T_cam1_tag = self.detect_pose(img1, info1)
        T_cam2_tag = self.detect_pose(img2, info2)
        if T_cam1_tag is None or T_cam2_tag is None:
            return
        T_cam1_cam2 = T_cam1_tag @ np.linalg.inv(T_cam2_tag)
        self.samples.append(T_cam1_cam2)
        if len(self.samples) % 10 == 0 or len(self.samples) == self.n_samples:
            self.get_logger().info(f'sample {len(self.samples)}/{self.n_samples} captured')
        if len(self.samples) == self.n_samples:
            self.report()

    def report(self):
        translations = np.array([T[:3, 3] for T in self.samples])
        t_mean = translations.mean(axis=0)
        t_std = translations.std(axis=0)

        R_sum = np.zeros((3, 3))
        for T in self.samples:
            R_sum += T[:3, :3]
        U, _, Vt = np.linalg.svd(R_sum)
        R_mean = U @ Vt
        if np.linalg.det(R_mean) < 0:
            U[:, -1] *= -1
            R_mean = U @ Vt

        T_final = np.eye(4)
        T_final[:3, :3] = R_mean
        T_final[:3, 3] = t_mean

        from scipy.spatial.transform import Rotation
        q = Rotation.from_matrix(R_mean).as_quat()

        print('\n===== D415 <- D435i extrinsic (cam1 <- cam2), AprilTag-based =====')
        print(f'n_samples: {len(self.samples)}')
        print('Translation (m), mean:', t_mean, ' std:', t_std)
        print(f'std magnitude: {np.linalg.norm(t_std)*1000:.2f} mm '
              f'(2026-11-03 ArUco result: z std alone was 44.7mm)')
        print('4x4 transform matrix:')
        print(T_final)
        print('\nstatic_transform_publisher command (x y z qx qy qz qw parent child):')
        print(f"ros2 run tf2_ros static_transform_publisher "
              f"{t_mean[0]:.5f} {t_mean[1]:.5f} {t_mean[2]:.5f} "
              f"{q[0]:.5f} {q[1]:.5f} {q[2]:.5f} {q[3]:.5f} "
              f"D415_color_optical_frame D435i_color_optical_frame")
        print('====================================================\n')
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag-id', type=int, required=True)
    ap.add_argument('--tag-length-m', type=float, required=True,
                     help='measured black-square side length in meters (not nominal)')
    ap.add_argument('--n-samples', type=int, default=60)
    args = ap.parse_args()

    rclpy.init()
    node = ExtrinsicCalibAprilTag(args.tag_id, args.tag_length_m, args.n_samples)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()