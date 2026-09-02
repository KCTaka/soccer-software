"""Synthetic camera (sim aid).

Renders a trivial RoboCup-like scene — green pitch, a white field line, and an
orange ball — and publishes it on ``camera/image_raw``. This stands in for the
Isaac Sim camera sensor so the *entire* perception → localization → strategy →
control loop can run on a laptop without a GPU.

To close the loop visually it reads ``joint_states``: the ball's horizontal
position in the image is the *world ball bearing minus the neck_pan angle*, so
when Strategy tracks the ball the motor pans until the ball is centred — exactly
the behaviour the real robot exhibits.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge

    _HAVE_CV = True
except Exception:  # pragma: no cover
    _HAVE_CV = False


class SimCamera(Node):
    def __init__(self) -> None:
        super().__init__("sim_camera")
        self.declare_parameter("ball_bearing", 0.3)   # world bearing of the ball (rad)
        self.declare_parameter("hfov", 1.05)          # ~60 deg
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self._ball_bearing = float(self.get_parameter("ball_bearing").value)
        self._hfov = float(self.get_parameter("hfov").value)
        self._w = int(self.get_parameter("width").value)
        self._h = int(self.get_parameter("height").value)
        self._pan = 0.0
        self._bridge = CvBridge() if _HAVE_CV else None

        self.create_subscription(JointState, "joint_states", self._on_js, 10)
        # Match the real camera contract: images are best-effort SensorData QoS.
        self._pub = self.create_publisher(Image, "camera/image_raw", qos_profile_sensor_data)
        # ...and the contract includes camera_info. Consumers MUST take intrinsics
        # from this topic rather than hardcoding them: doing the latter caused
        # 2.5-8.5 m ground-projection errors on the real robot, because the ZED is
        # 1280x720 with fx=732.9, not 640x480 with fx=550.
        # See docs/architecture/perception_gpu_migration.md §2.
        # Published next to the image (camera/camera_info), which is where the ZED
        # wrapper puts it and therefore what the contract is.
        self._info_pub = self.create_publisher(
            CameraInfo, "camera/camera_info", qos_profile_sensor_data)
        # Intrinsics implied by the synthetic pinhole model actually rendered:
        # a horizontal FOV of `hfov` over `width` pixels.
        fx = (self._w / 2.0) / math.tan(self._hfov / 2.0)
        self._info = CameraInfo()
        self._info.width, self._info.height = self._w, self._h
        self._info.distortion_model = "plumb_bob"
        self._info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._info.k = [fx, 0.0, self._w / 2.0,
                        0.0, fx, self._h / 2.0,
                        0.0, 0.0, 1.0]
        self._info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self._info.p = [fx, 0.0, self._w / 2.0, 0.0,
                        0.0, fx, self._h / 2.0, 0.0,
                        0.0, 0.0, 1.0, 0.0]
        self.create_timer(1.0 / 30.0, self._render)  # 30 Hz
        self.get_logger().info(
            f"sim_camera up (30 Hz synthetic scene, fx={fx:.1f}).")

    def _on_js(self, msg: JointState) -> None:
        if "neck_pan" in msg.name:
            self._pan = msg.position[msg.name.index("neck_pan")]

    def _render(self) -> None:
        if not _HAVE_CV:
            return
        img = np.zeros((self._h, self._w, 3), dtype=np.uint8)
        img[:] = (60, 140, 60)  # green pitch (BGR)
        # A white field line across the lower third.
        cv2.line(img, (0, int(self._h * 0.7)), (self._w, int(self._h * 0.7)),
                 (240, 240, 240), 4)
        # Ball position from (world bearing - current pan).
        rel = self._ball_bearing - self._pan
        if abs(rel) <= self._hfov / 2.0:
            u = int(self._w / 2 - (rel / (self._hfov / 2.0)) * (self._w / 2))
            cv2.circle(img, (u, int(self._h * 0.62)), 16, (0, 140, 255), -1)  # orange
        msg = self._bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        self._pub.publish(msg)
        # Same stamp and frame as the image so consumers can pair them.
        self._info.header = msg.header
        self._info_pub.publish(self._info)


def main() -> None:
    rclpy.init()
    node = SimCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
