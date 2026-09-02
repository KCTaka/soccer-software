"""3D projection node (L3, blueprint §5, localization report §6).

Turns 2D image detections into 3D positions in the robot **base frame**:

* the **ball** ground-contact point -> ``geometry_msgs/PointStamped`` for Strategy
  and team comms;
* **goalposts** -> ``FieldFeature`` landmarks for the MCL (goalposts add the
  asymmetry that disambiguates the symmetric field — report §3.1).

Uses ZED stereo **depth** when a depth image is available (accurate for objects
above the ground), else falls back to the flat-ground homography. Depth is the
single biggest accuracy win over the legacy monocular pipeline (report §3.2).
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from soccer_msgs.msg import BoundingBoxes, FieldFeature, FieldFeatureArray
from std_msgs.msg import Header

from soccer_perception.camera_model import (
    PinholeCamera,
    project_flat_ground,
    project_with_depth,
)

try:
    import numpy as np
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    _HAVE_CV = True
except Exception:  # pragma: no cover
    _HAVE_CV = False
    from sensor_msgs.msg import Image


class ProjectionNode(Node):
    """Projects detector boxes into 3D base-frame coordinates."""

    def __init__(self) -> None:
        super().__init__("projection_node")
        self.declare_parameter("use_depth", True)
        self._use_depth = bool(self.get_parameter("use_depth").value) and _HAVE_CV

        # Intrinsics are NOT set here. They arrive on camera_info and nothing is
        # projected until they do -- see _on_camera_info. Hardcoding them caused
        # 2.5-8.5 m errors on the real robot (the ZED is 1280x720 / fx=732.9, not
        # 640x480 / fx=550) AND, worse, this node was subscribing to a topic
        # nobody published, so the placeholders were never overwritten.
        # See docs/architecture/perception_gpu_migration.md §2.
        self._cam = PinholeCamera(
            fx=0.0, fy=0.0, cx=0.0, cy=0.0, width=0, height=0,
            mount_height=0.30, tilt=0.35,
        )
        self._have_intrinsics = False
        self._depth = None
        self._bridge = CvBridge() if _HAVE_CV else None

        self.ball_pub = self.create_publisher(PointStamped, "ball/point", 10)
        self.feat_pub = self.create_publisher(FieldFeatureArray, "object_features", 10)
        # detections is an internal, low-rate topic -> keep the default reliable QoS.
        self.det_sub = self.create_subscription(
            BoundingBoxes, "detections", self._on_detections, 10
        )
        # camera_info + depth are camera sensor streams -> best-effort SensorData QoS
        # (matches the ZED publishers).
        #
        # The topic is "camera/camera_info", NOT "camera_info". Both the ZED
        # wrapper and sim_camera publish camera_info NEXT TO the image, per ROS
        # convention. This node previously subscribed to "camera_info", which had
        # zero publishers, so it silently ran on placeholder intrinsics forever.
        self.caminfo_sub = self.create_subscription(
            CameraInfo, "camera/camera_info", self._on_camera_info, qos_profile_sensor_data
        )
        if self._use_depth:
            self.depth_sub = self.create_subscription(
                Image, "camera/depth", self._on_depth, qos_profile_sensor_data
            )
        self.get_logger().info(
            f"projection_node up (depth={'on' if self._use_depth else 'off'})."
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        k = msg.k  # row-major 3x3 intrinsics
        if not (k[0] > 0.0 and k[4] > 0.0):
            return  # degenerate calibration -- keep waiting for a good one
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

    def _on_depth(self, msg) -> None:
        self._depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def _project(self, u: int, v: int):
        if self._use_depth and self._depth is not None:
            vv = min(v, self._depth.shape[0] - 1)
            uu = min(u, self._depth.shape[1] - 1)
            d = float(self._depth[vv, uu])
            if d > 0.05 and not np.isnan(d):
                p = project_with_depth(self._cam, u, v, d)
                return [float(p[2]), float(-p[0]), 0.0]  # optical -> base (x fwd,y left)
        return project_flat_ground(self._cam, float(u), float(v))

    def _on_detections(self, msg: BoundingBoxes) -> None:
        if not self._have_intrinsics:
            # Publishing here would mean inventing geometry. Stay quiet and loud.
            self.get_logger().warn(
                "no camera_info yet -- not projecting. Expecting camera/camera_info.",
                throttle_duration_sec=10.0,
            )
            return
        feats = FieldFeatureArray()
        feats.header = Header(stamp=msg.header.stamp, frame_id="base_link")
        for b in msg.bounding_boxes:
            p = self._project(int(b.xbase), int(b.ybase))
            if p is None:
                continue
            if b.class_id == "ball":
                ps = PointStamped()
                ps.header = feats.header
                ps.point = Point(x=float(p[0]), y=float(p[1]), z=0.0)
                self.ball_pub.publish(ps)
            elif b.class_id == "goalpost":
                f = FieldFeature()
                f.type = FieldFeature.TYPE_GOALPOST
                f.position = Point(x=float(p[0]), y=float(p[1]), z=0.0)
                f.confidence = float(b.probability)
                feats.features.append(f)
        if feats.features:
            self.feat_pub.publish(feats)


def main() -> None:
    rclpy.init()
    node = ProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
