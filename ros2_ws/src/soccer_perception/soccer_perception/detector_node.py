"""Object detector node (L3, blueprint §5, localization report §3.2-3.3).

A *swappable* detector. The production target is **RF-DETR** (DINOv2 backbone,
Apache-2.0) exported to TensorRT, detecting ``ball``, ``goalpost`` and ``robot``
— chosen for its accuracy-latency Pareto and small-data domain adaptation. Until
that engine is benchmarked on the Jetson (the public numbers are T4!), this node
ships a dependency-light classical HSV ball detector so the rest of the pipeline
is exercisable end-to-end. Swapping in RF-DETR is a node-internal change; the
published ``BoundingBoxes`` contract is unchanged (report §6).
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from soccer_msgs.msg import BoundingBox, BoundingBoxes

try:  # heavy deps are optional so the node imports cleanly anywhere
    import cv2
    import numpy as np
    from cv_bridge import CvBridge

    _HAVE_CV = True
except Exception:  # pragma: no cover - exercised only without OpenCV
    _HAVE_CV = False


class DetectorNode(Node):
    """Detects ball/goalpost/robot in the monocular camera feed."""

    def __init__(self) -> None:
        super().__init__("detector_node")
        self.declare_parameter("image_topic", "camera/image_raw")
        self.declare_parameter("engine_path", "")  # RF-DETR TensorRT engine (.engine)
        image_topic = self.get_parameter("image_topic").value
        self._engine_path = self.get_parameter("engine_path").value

        self._bridge = CvBridge() if _HAVE_CV else None
        self._detector = self._load_detector()

        self.pub = self.create_publisher(BoundingBoxes, "detections", 10)
        # Camera images are a high-rate sensor stream -> best-effort SensorData QoS
        # (matches the ZED publisher; avoids RELIABLE retransmit stalls on big frames).
        self.sub = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"detector_node up ({'RF-DETR' if self._engine_path else 'HSV fallback'})."
        )

    def _load_detector(self):
        # TODO(perception/ml): if self._engine_path: load RF-DETR TensorRT engine,
        # warm it up, and return a callable image -> detections. Benchmark distant-
        # ball recall + latency on the ACTUAL Orin/Thor before committing (report §3.2).
        return None

    def _on_image(self, msg: Image) -> None:
        out = BoundingBoxes()
        out.header = msg.header
        if _HAVE_CV:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            out.bounding_boxes = self._detect_ball_hsv(frame)
        self.pub.publish(out)

    def _detect_ball_hsv(self, frame) -> list:
        """Classical orange-ball detector (stand-in for RF-DETR)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([5, 120, 120]), np.array([25, 255, 255]))
        mask = cv2.medianBlur(mask, 5)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 25:  # reject specks; tune for competition ball-at-range recall
                continue
            x, y, w, h = cv2.boundingRect(c)
            b = BoundingBox()
            b.class_id = "ball"
            b.probability = float(min(1.0, area / 2000.0))
            b.xmin, b.ymin, b.xmax, b.ymax = int(x), int(y), int(x + w), int(y + h)
            # Ground-contact point used by projection_node (bottom-centre of bbox).
            b.xbase, b.ybase = int(x + w / 2), int(y + h)
            b.id = -1
            boxes.append(b)
        return boxes


def main() -> None:
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
