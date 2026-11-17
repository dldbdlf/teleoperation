"""Validate each camera's own metric scale (not the inter-camera extrinsic) by averaging a tag's
PnP position over N samples at one physical stop, independently in each camera's own frame.
Run once per stop (e.g. "before", "after" a known-distance move) with --save <name>; a third
mode --diff <name1> <name2> prints the displacement each camera independently measured, to
compare against the known ground-truth move distance.
"""
import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

OUT_DIR = __file__.rsplit('/', 1)[0] + '/calib_results'


class Capture(Node):
    def __init__(self, tag_id, tag_length_m, n_samples):
        super().__init__('check_metric_scale')
        self.tag_id = tag_id
        self.n_samples = n_samples
        self.bridge = CvBridge()
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11))
        s = tag_length_m
        self.obj = np.array([[-s/2, s/2, 0], [s/2, s/2, 0], [s/2, -s/2, 0], [-s/2, -s/2, 0]], dtype=np.float32)
        self.samples = {'A': [], 'B': []}
        self.info = {'A': None, 'B': None}
        self.done = False

        self.create_subscription(CameraInfo, '/cam_1/D415/color/camera_info',
                                  lambda m: self.info.__setitem__('A', m), 1)
        self.create_subscription(CameraInfo, '/cam_2/D435i/color/camera_info',
                                  lambda m: self.info.__setitem__('B', m), 1)
        self.create_subscription(Image, '/cam_1/D415/color/image_raw',
                                  lambda m: self.on_image('A', m), 1)
        self.create_subscription(Image, '/cam_2/D435i/color/image_raw',
                                  lambda m: self.on_image('B', m), 1)
        self.get_logger().info(f'Capturing tag {tag_id}, need {n_samples} samples from EACH camera '
                                f'(independent, no cross-camera sync needed)...')

    def on_image(self, key, img_msg):
        self.n_calls = getattr(self, 'n_calls', 0) + 1
        if self.done or len(self.samples[key]) >= self.n_samples or self.info[key] is None:
            return
        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        K = np.array(self.info[key].k).reshape(3, 3)
        D = np.array(self.info[key].d)
        corners, ids, _ = self.det.detectMarkers(frame)
        if self.n_calls % 30 == 1:
            # Periodic visibility status -- if tag_id never shows up here, it simply isn't in
            # view of this camera right now; this is not a bug (cost real debugging time to learn).
            seen = None if ids is None else ids.flatten().tolist()
            print(f'[{key}] frame#{self.n_calls}: tags visible = {seen}  (need id={self.tag_id})', flush=True)
        if ids is None or self.tag_id not in ids.flatten():
            return
        idx = list(ids.flatten()).index(self.tag_id)
        ok, rvec, tvec = cv2.solvePnP(self.obj, corners[idx], K, D)
        if not ok:
            return
        self.samples[key].append(tvec.flatten())
        if len(self.samples['A']) >= self.n_samples and len(self.samples['B']) >= self.n_samples and not self.done:
            self.done = True
            self.report()

    def report(self):
        a_mean = np.mean(self.samples['A'], axis=0)
        b_mean = np.mean(self.samples['B'], axis=0)
        a_std = np.std(self.samples['A'], axis=0)
        b_std = np.std(self.samples['B'], axis=0)
        print(f'A (D415) mean position (m): {a_mean}  std: {a_std}')
        print(f'B (D435i) mean position (m): {b_mean}  std: {b_std}')
        np.savez(f'{OUT_DIR}/{SAVE_NAME}.npz', a=a_mean, b=b_mean, a_std=a_std, b_std=b_std)
        print(f'saved {OUT_DIR}/{SAVE_NAME}.npz')
        rclpy.shutdown()


def main():
    global SAVE_NAME
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag-id', type=int, default=0)
    ap.add_argument('--tag-length-m', type=float, default=0.09)
    ap.add_argument('--n-samples', type=int, default=30)
    ap.add_argument('--save', type=str, help='stage name, e.g. before / after')
    ap.add_argument('--diff', nargs=2, help='two stage names to diff, e.g. before after')
    ap.add_argument('--gt-m', type=float, default=None, help='known ground-truth move distance, meters')
    args = ap.parse_args()

    if args.diff:
        n1, n2 = args.diff
        d1 = np.load(f'{OUT_DIR}/{n1}.npz')
        d2 = np.load(f'{OUT_DIR}/{n2}.npz')
        for cam in ('a', 'b'):
            disp = d2[cam] - d1[cam]
            mag = np.linalg.norm(disp)
            line = f'camera {cam.upper()}: displacement (m) = {disp}, magnitude = {mag*1000:.2f} mm'
            if args.gt_m:
                err = (mag - args.gt_m) * 1000
                pct = (mag - args.gt_m) / args.gt_m * 100
                line += f'  |  vs GT {args.gt_m*100:.1f}cm: error = {err:+.2f} mm ({pct:+.2f}%)'
            print(line)
        return

    SAVE_NAME = args.save
    rclpy.init()
    node = Capture(args.tag_id, args.tag_length_m, args.n_samples)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
