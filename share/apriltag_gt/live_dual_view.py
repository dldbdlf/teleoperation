"""Live dual-camera preview via ROS2 topics (D415/cam_1, D435i/cam_2), with AprilTag overlay.
Not an Artifact page -- renders two OpenCV windows directly on this machine's display, for
positioning the tag rig before running extrinsic_calib_apriltag.py.

Usage: python3 live_dual_view.py
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

TAG_SIZE_M = {0: 0.09, 1: 0.06, 2: 0.06, 3: 0.04}


class LiveDualView(Node):
    def __init__(self):
        super().__init__('live_dual_view')
        self.bridge = CvBridge()
        self.det = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11))
        self.frames = {}
        self.create_subscription(Image, '/cam_1/D415/color/image_raw',
                                  lambda m: self.on_image('Camera 1 - D415', m), 1)
        self.create_subscription(Image, '/cam_2/D435i/color/image_raw',
                                  lambda m: self.on_image('Camera 2 - D435i', m), 1)
        for name in ('Camera 1 - D415', 'Camera 2 - D435i'):
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(name, 640, 480)
        cv2.moveWindow('Camera 1 - D415', 40, 60)
        cv2.moveWindow('Camera 2 - D435i', 720, 60)
        self.create_timer(1 / 30.0, self.draw)

    def on_image(self, name, msg):
        self.frames[name] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def draw(self):
        for name, frame in self.frames.items():
            f = frame.copy()
            gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.det.detectMarkers(gray)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(f, corners, ids)
                for c, i in zip(corners, ids.flatten()):
                    cx, cy = c[0].mean(axis=0)
                    label = f"id={i} ({TAG_SIZE_M.get(int(i), '?')}m)"
                    cv2.putText(f, label, (int(cx) - 40, int(cy) - 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(name, f)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main():
    rclpy.init()
    node = LiveDualView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
