"""Stream a Windows webcam to WSL as length-prefixed JPEG frames over TCP.

WSL's usbipd passthrough cannot carry webcam (isochronous) video, so the camera
stays on Windows and agx_arm_net_camera receives the frames inside WSL.
This script connects to agx_arm_net_camera at the WSL VM address rather than
listening, because the Windows firewall blocks connections from WSL.

Run on Windows (needs opencv-python), after starting agx_arm_net_camera:
    py webcam_stream_windows.py --camera 0
"""
import argparse
import socket
import struct
import subprocess
import threading
import time

import cv2


class LatestFrame:
    """Grabs frames in the background and keeps only the newest one, so a slow
    connection drops frames instead of building up latency."""

    def __init__(self, cap):
        self.cap = cap
        self.frame = None
        self.seq = 0
        self.cond = threading.Condition()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            with self.cond:
                self.frame = frame
                self.seq += 1
                self.cond.notify_all()

    def wait_newer(self, seq):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != seq, timeout=1.0)
            return self.seq, self.frame


def wsl_ip():
    """WSL2 VM address. Windows can reach it directly; localhost forwarding
    keeps half-open connections when the receiver restarts."""
    out = subprocess.run(["wsl.exe", "hostname", "-I"], capture_output=True, text=True, check=True)
    return out.stdout.split()[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--camera", type=int, default=0, help="DirectShow camera index")
    parser.add_argument(
        "--host", default="", help="agx_arm_net_camera address (default: IP of the default WSL distro)"
    )
    parser.add_argument("--port", type=int, default=5600)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--quality", type=int, default=85, help="JPEG quality 1-100")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open camera {args.camera}")
    frames = LatestFrame(cap)

    if not args.host:
        args.host = wsl_ip()
    print(f"Camera {args.camera} {args.width}x{args.height}, sending to {args.host}:{args.port}")
    encode = [cv2.IMWRITE_JPEG_QUALITY, args.quality]
    while True:
        try:
            conn = socket.create_connection((args.host, args.port), timeout=3.0)
        except OSError:
            print("  waiting for agx_arm_net_camera ...")
            time.sleep(2.0)
            continue
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # A small send buffer makes sendall block while the receiver is busy, so
        # frames are dropped here instead of queuing up as latency.
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 64 * 1024)
        print("Connected")
        seq, sent, t0 = 0, 0, time.time()
        try:
            while True:
                seq, frame = frames.wait_newer(seq)
                if frame is None:
                    continue
                ok, jpg = cv2.imencode(".jpg", frame, encode)
                if not ok:
                    continue
                data = jpg.tobytes()
                conn.sendall(struct.pack("!I", len(data)) + data)
                sent += 1
                if time.time() - t0 >= 5.0:
                    print(f"  {sent / (time.time() - t0):.1f} fps")
                    sent, t0 = 0, time.time()
        except (ConnectionError, OSError):
            print("Disconnected")
        finally:
            conn.close()


if __name__ == "__main__":
    main()
