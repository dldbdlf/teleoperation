"""Live dual-camera view (OpenCV windows on this machine's display, NOT a web artifact) with a
manual CAPTURE button per camera: you decide the exact moment, click the button, it grabs one
tag-0 pose sample from that camera right then. Used for the "move tag exactly 30cm, compare each
camera's own measured displacement to the known 30cm" metric-scale accuracy check.

Controls (click the button rectangles, or use the keys shown):
  CAPTURE A / CAPTURE B  - grab one sample from that camera into the current stage's buffer
  SAVE BEFORE  (key 1)   - average the buffers, save as the "before" stage, clear buffers
  SAVE AFTER   (key 2)   - average the buffers, save as the "after" stage, compute + show the
                           displacement each camera independently measured vs the known move
  RESET        (key r)   - clear buffers without saving
  QUIT         (key q)
"""
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

OUT_DIR = __file__.rsplit('/', 1)[0] + '/calib_results'
TAG_ID = 0
TAG_LEN_M = 0.09
GT_MOVE_M = 0.30  # the known, intended tag displacement for this check


class Cams(Node):
    def __init__(self):
        super().__init__('manual_capture_gui')
        self.bridge = CvBridge()
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11))
        self.frame = {'A': None, 'B': None}
        self.info = {'A': None, 'B': None}
        self.pose_now = {'A': None, 'B': None}  # current tag-0 pose this instant, or None
        self.create_subscription(CameraInfo, '/cam_1/D415/color/camera_info',
                                  lambda m: self.info.__setitem__('A', m), 1)
        self.create_subscription(CameraInfo, '/cam_2/D435i/color/camera_info',
                                  lambda m: self.info.__setitem__('B', m), 1)
        self.create_subscription(Image, '/cam_1/D415/color/image_raw', lambda m: self.on_image('A', m), 1)
        self.create_subscription(Image, '/cam_2/D435i/color/image_raw', lambda m: self.on_image('B', m), 1)

    def on_image(self, key, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame[key] = frame
        self.pose_now[key] = None
        if self.info[key] is None:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.det.detectMarkers(gray)
        if ids is None or TAG_ID not in ids.flatten():
            return
        idx = list(ids.flatten()).index(TAG_ID)
        s = TAG_LEN_M
        obj = np.array([[-s/2, s/2, 0], [s/2, s/2, 0], [s/2, -s/2, 0], [-s/2, -s/2, 0]], dtype=np.float32)
        K = np.array(self.info[key].k).reshape(3, 3)
        D = np.array(self.info[key].d)
        ok, rvec, tvec = cv2.solvePnP(obj, corners[idx], K, D)
        if ok:
            self.pose_now[key] = tvec.flatten()


buffers = {'A': [], 'B': []}
saved = {}  # stage name -> {'A': mean, 'B': mean}
status_lines = ['Ready. Position tag 0, click CAPTURE A / CAPTURE B several times per stage.']

BTN = {
    'cap_a':  (20, 20, 160, 60),
    'cap_b':  (180, 20, 320, 60),
    'before': (340, 20, 460, 60),
    'after':  (480, 20, 600, 60),
    'reset':  (620, 20, 740, 60),
}


def draw_button(img, rect, label, color=(70, 70, 70)):
    x1, y1, x2, y2 = rect
    cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
    cv2.rectangle(img, (x1, y1), (x2, y2), (200, 200, 200), 1)
    cv2.putText(img, label, (x1 + 8, y1 + 27), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)


def do_capture(cam_key, cams):
    p = cams.pose_now[cam_key]
    if p is None:
        status_lines.append(f'capture {cam_key}: tag {TAG_ID} not visible right now -- nothing captured')
    else:
        buffers[cam_key].append(p.copy())
        status_lines.append(f'capture {cam_key}: sample #{len(buffers[cam_key])} = {p.round(4)}')
    del status_lines[:-6]


def do_save(stage):
    if not buffers['A'] or not buffers['B']:
        status_lines.append(f'save {stage}: need at least one CAPTURE A and one CAPTURE B first')
        del status_lines[:-6]
        return
    a_mean = np.mean(buffers['A'], axis=0)
    b_mean = np.mean(buffers['B'], axis=0)
    saved[stage] = {'A': a_mean, 'B': b_mean, 'nA': len(buffers['A']), 'nB': len(buffers['B'])}
    np.savez(f'{OUT_DIR}/manual_{stage}.npz', a=a_mean, b=b_mean)
    status_lines.append(f"saved '{stage}': A n={len(buffers['A'])} mean={a_mean.round(4)}, "
                         f"B n={len(buffers['B'])} mean={b_mean.round(4)}")
    buffers['A'].clear()
    buffers['B'].clear()
    if 'before' in saved and 'after' in saved:
        for cam in ('A', 'B'):
            disp = saved['after'][cam] - saved['before'][cam]
            mag = np.linalg.norm(disp) * 1000
            err = mag - GT_MOVE_M * 1000
            pct = err / (GT_MOVE_M * 1000) * 100
            status_lines.append(f'cam {cam}: measured move = {mag:.1f}mm vs GT {GT_MOVE_M*1000:.0f}mm '
                                 f'-> error {err:+.1f}mm ({pct:+.1f}%)')
    del status_lines[:-8]


def on_mouse(event, x, y, flags, cams):
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    for name, (x1, y1, x2, y2) in BTN.items():
        if x1 <= x <= x2 and y1 <= y <= y2:
            if name == 'cap_a':
                do_capture('A', cams)
            elif name == 'cap_b':
                do_capture('B', cams)
            elif name == 'before':
                do_save('before')
            elif name == 'after':
                do_save('after')
            elif name == 'reset':
                buffers['A'].clear()
                buffers['B'].clear()
                status_lines.append('buffers reset')


def main():
    rclpy.init()
    cams = Cams()
    t = threading.Thread(target=rclpy.spin, args=(cams,), daemon=True)
    t.start()

    for name in ('Camera A - D415', 'Camera B - D435i'):
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, 640, 480)
    cv2.moveWindow('Camera A - D415', 40, 100)
    cv2.moveWindow('Camera B - D435i', 720, 100)
    cv2.namedWindow('Control', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Control', 780, 260)
    cv2.moveWindow('Control', 40, 620)
    cv2.setMouseCallback('Control', on_mouse, cams)

    while True:
        for key, win in (('A', 'Camera A - D415'), ('B', 'Camera B - D435i')):
            f = cams.frame[key]
            if f is None:
                continue
            f = f.copy()
            tag_ok = cams.pose_now[key] is not None
            label = f'tag {TAG_ID}: {"VISIBLE" if tag_ok else "not visible"}'
            cv2.putText(f, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if tag_ok else (0, 0, 255), 2)
            cv2.putText(f, f'buffered: {len(buffers[key])}', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.imshow(win, f)

        ctrl = np.full((260, 780, 3), 30, dtype=np.uint8)
        draw_button(ctrl, BTN['cap_a'], 'CAPTURE A', (0, 110, 0))
        draw_button(ctrl, BTN['cap_b'], 'CAPTURE B', (0, 110, 0))
        draw_button(ctrl, BTN['before'], 'SAVE BEFORE', (110, 70, 0))
        draw_button(ctrl, BTN['after'], 'SAVE AFTER', (110, 70, 0))
        draw_button(ctrl, BTN['reset'], 'RESET', (110, 0, 0))
        for i, line in enumerate(status_lines[-7:]):
            cv2.putText(ctrl, line[:95], (20, 95 + i * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (230, 230, 230), 1, cv2.LINE_AA)
        cv2.imshow('Control', ctrl)

        k = cv2.waitKey(30) & 0xFF
        if k == ord('q'):
            break
        elif k == ord('1'):
            do_save('before')
        elif k == ord('2'):
            do_save('after')
        elif k == ord('r'):
            buffers['A'].clear()
            buffers['B'].clear()

    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
