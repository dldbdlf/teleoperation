"""ROS2 bridge: publishes the Rokoko Smartglove's live hand/wrist orientation (and, when the
stream happens to deliver them, finger orientations) as standard geometry_msgs, so other nodes
(e.g. a Frame-building bridge into share/sim/baselines.py's solver) can subscribe without caring
about the glove's USB-serial debug-shell protocol at all.

Published topics (geometry_msgs/QuaternionStamped, frame_id = "<sensor>_imu"):
    /rokoko_glove/hub/orientation      -- hand/wrist orientation (the one this project needs)
    /rokoko_glove/thumb/orientation
    /rokoko_glove/index/orientation
    /rokoko_glove/middle/orientation
    /rokoko_glove/ring/orientation
    /rokoko_glove/little/orientation
    /rokoko_glove/arm/orientation

Only published once at least one sample has arrived for that sensor -- per glove_reader.py's own
CAVEATS, "little" and "arm" may never publish at all in a given session; a consumer that needs the
glove's forearm orientation should not block waiting for /rokoko_glove/arm/orientation and should
have a fallback (this project's existing AprilTag/RealSense pipeline is the known-working one).

Usage:
    python3 glove_ros_node.py
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped

sys.path.insert(0, str(Path(__file__).resolve().parent))
from glove_reader import GloveReader, SENSOR_NAMES  # noqa: E402


class GloveRosNode(Node):
    def __init__(self, port: str | None = None):
        super().__init__("rokoko_glove_node")
        self.reader = GloveReader(port=port)
        self.get_logger().info(f"Connected to glove on {self.reader.port}, configuring stream...")
        self.reader.start()
        self.pubs = {
            name: self.create_publisher(QuaternionStamped, f"/rokoko_glove/{name}/orientation", 10)
            for name in SENSOR_NAMES
        }
        self._last_seen = {name: 0 for name in SENSOR_NAMES}
        self.create_timer(1.0 / 60.0, self._tick)  # polls reader state; actual update rate is
        # whatever the glove's stream delivers (measured ~9-12Hz for hub), not this timer's rate.
        self.create_timer(5.0, self._report_status)

    def _tick(self):
        for name in SENSOR_NAMES:
            st = self.reader.sensors[name]
            if st.n_updates == self._last_seen[name]:
                continue  # no new sample since last tick -- don't republish a stale value
            self._last_seen[name] = st.n_updates
            msg = QuaternionStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"{name}_imu"
            w, x, y, z = st.quat_wxyz
            msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z = w, x, y, z
            self.pubs[name].publish(msg)

    def _report_status(self):
        parts = []
        for name in SENSOR_NAMES:
            st = self.reader.sensors[name]
            age = time.time() - st.t_host if st.t_host else float("inf")
            parts.append(f"{name}={'OK' if age < 2.0 else 'STALE/NONE'}({st.n_updates})")
        self.get_logger().info("glove sensor status: " + " ".join(parts))

    def destroy_node(self):
        self.get_logger().info("Stopping glove reader and rebooting glove to a clean state...")
        self.reader.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = GloveRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
