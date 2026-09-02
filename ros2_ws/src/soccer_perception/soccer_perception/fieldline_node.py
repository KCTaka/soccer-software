"""Field-line segmentation node (L3, localization report §2 claim 6, §3.2).

Field-line extraction is a *semantic segmentation* problem (each pixel = line /
field / background), NOT object detection — which is why it is a separate node
from the detector. The production target is a compact seg net (PIDNet/BiSeNet)
exported to TensorRT; this implementation ships the classical HSV grass-mask +
white-line extractor (mirroring the legacy ``detector_fieldline.py``) so the MCL
has a real point cloud to localize against.

Output: ``FieldFeatureArray`` of line points in the robot **base frame**, the
"point cloud" the Tier-2 MCL weights against the likelihood-field map.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from soccer_msgs.msg import FieldFeature, FieldFeatureArray
from std_msgs.msg import Header

from soccer_perception.camera_model import PinholeCamera, project_flat_ground
from sensor_msgs.msg import CameraInfo

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    _HAVE_CV = True
except Exception:  # pragma: no cover
    _HAVE_CV = False
    from sensor_msgs.msg import Image


class FieldlineNode(Node):
    """Extracts white field-line pixels and projects them to the base frame."""

    def __init__(self) -> None:
        super().__init__("fieldline_node")
        self.declare_parameter("image_topic", "camera/image_raw")
        self.declare_parameter("max_points", 200)  # subsample to keep MCL cheap
        image_topic = self.get_parameter("image_topic").value
        self._max_points = int(self.get_parameter("max_points").value)

        # Intrinsics come from camera_info; nothing is projected until they do.
        # These were hardcoded to 640x480/fx=550, which is right for sim_camera and
        # badly wrong for the ZED (1280x720/fx=732.9): 2.5-8.5 m ground-projection
        # errors, plus above-horizon pixels fabricated into confident ground
        # points. See docs/architecture/perception_gpu_migration.md §2.
        # On real hardware this node is replaced by the C++ TensorRT component;
        # it still runs for camera:=sim, and must be correct there too.
        self._cam = PinholeCamera(
            fx=0.0, fy=0.0, cx=0.0, cy=0.0, width=0, height=0,
            mount_height=0.30, tilt=0.35,
        )
        self._have_intrinsics = False
        self._bridge = CvBridge() if _HAVE_CV else None

        self.pub = self.create_publisher(FieldFeatureArray, "field_features", 10)
        # Camera images are a high-rate sensor stream -> best-effort SensorData QoS.
        self.sub = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )
        # Published next to the image by both the ZED wrapper and sim_camera.
        self.caminfo_sub = self.create_subscription(
            CameraInfo, "camera/camera_info", self._on_camera_info, qos_profile_sensor_data
        )
        self.get_logger().info("fieldline_node up (waiting for camera_info).")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        k = msg.k
        if not (k[0] > 0.0 and k[4] > 0.0):
            return
        self._cam.fx, self._cam.fy = float(k[0]), float(k[4])
        self._cam.cx, self._cam.cy = float(k[2]), float(k[5])
        if msg.width and msg.height:
            self._cam.width, self._cam.height = int(msg.width), int(msg.height)
        if not self._have_intrinsics:
            self._have_intrinsics = True
            self.get_logger().info(
                f"intrinsics from camera_info: {self._cam.width}x{self._cam.height} "
                f"fx={self._cam.fx:.1f} cx={self._cam.cx:.1f} cy={self._cam.cy:.1f}"
            )

    def _on_image(self, msg) -> None:
        if not self._have_intrinsics:
            # Wrong geometry is worse than no geometry: the MCL cannot tell the
            # difference, so publish nothing rather than guess.
            self.get_logger().warn(
                "no camera_info yet -- not publishing field features.",
                throttle_duration_sec=10.0,
            )
            return
        out = FieldFeatureArray()
        out.header = Header(stamp=msg.header.stamp, frame_id="base_link")
        if _HAVE_CV:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            out.features = self._extract(frame)
        self.pub.publish(out)

    def _extract(self, frame) -> list:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Grass mask (green) then white lines that sit *inside* the field.
        grass = cv2.inRange(hsv, np.array([35, 40, 40]), np.array([85, 255, 255]))
        white = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 60, 255]))
        field = cv2.dilate(grass, np.ones((25, 25), np.uint8))
        lines = cv2.bitwise_and(white, field)

        ys, xs = np.where(lines > 0)
        feats = []
        if len(xs) == 0:
            return feats
        step = max(1, len(xs) // self._max_points)
        for u, v in zip(xs[::step], ys[::step]):
            p = project_flat_ground(self._cam, float(u), float(v))
            if p is None or p[0] > 6.0:  # ignore unrealistic/horizon points
                continue
            f = FieldFeature()
            f.type = FieldFeature.TYPE_LINE_POINT
            f.position.x, f.position.y, f.position.z = float(p[0]), float(p[1]), 0.0
            f.confidence = 0.8
            feats.append(f)
        return feats


def main() -> None:
    rclpy.init()
    node = FieldlineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
