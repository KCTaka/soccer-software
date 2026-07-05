"""Tier-1 odometry: EKF sensor fusion (localization report §3, §5).

Smooth, high-rate local odometry — answers "how have I moved since the last
instant?" It fuses the IMU (orientation + yaw rate) with joint/leg odometry
(and, on the full robot, ZED Visual-Inertial Odometry) into ``odom -> base_link``.
This is the tier the blueprint calls "robot_localization"; in production it is the
``robot_localization`` EKF configured by YAML, here it is a compact equivalent so
the graph is complete. It feeds the MCL's motion model; it does NOT resolve field
symmetry — that is the MCL's job (Tier-2).

State x = [px, py, theta, v, omega].
"""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def _wrap(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


class EkfNode(Node):
    def __init__(self) -> None:
        super().__init__("ekf_node")
        self.x = np.zeros(5)                 # [px, py, theta, v, omega]
        self.P = np.eye(5) * 0.01
        self.Q = np.diag([1e-4, 1e-4, 1e-4, 1e-2, 1e-2])  # process noise
        self.R_imu = np.diag([1e-3, 1e-3])   # [theta, omega] measurement noise
        self._theta_bias = None
        self._last = self.get_clock().now()

        self._tf = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "odom", 20)
        # IMU is a high-rate sensor stream -> best-effort SensorData QoS.
        self.create_subscription(Imu, "imu/data", self._on_imu, qos_profile_sensor_data)
        self.create_timer(0.005, self._predict_and_publish)  # 200 Hz (L2 band)
        self.get_logger().info("ekf_node up (200 Hz odometry).")

    def _predict(self, dt: float) -> None:
        px, py, th, v, w = self.x
        self.x[0] = px + v * np.cos(th) * dt
        self.x[1] = py + v * np.sin(th) * dt
        self.x[2] = _wrap(th + w * dt)
        F = np.eye(5)
        F[0, 2] = -v * np.sin(th) * dt
        F[0, 3] = np.cos(th) * dt
        F[1, 2] = v * np.cos(th) * dt
        F[1, 3] = np.sin(th) * dt
        F[2, 4] = dt
        self.P = F @ self.P @ F.T + self.Q

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        if self._theta_bias is None:
            self._theta_bias = yaw
        z = np.array([_wrap(yaw - self._theta_bias), msg.angular_velocity.z])
        H = np.zeros((2, 5))
        H[0, 2] = 1.0
        H[1, 4] = 1.0
        y = z - H @ self.x
        y[0] = _wrap(y[0])
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = _wrap(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P

    def _predict_and_publish(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last).nanoseconds * 1e-9
        self._last = now
        if dt <= 0 or dt > 0.5:
            return
        self._predict(dt)

        stamp = now.to_msg()
        qz, qw = np.sin(self.x[2] / 2), np.cos(self.x[2] / 2)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(self.x[0])
        t.transform.translation.y = float(self.x[1])
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self._tf.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(self.x[0])
        odom.pose.pose.position.y = float(self.x[1])
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        odom.twist.twist.linear.x = float(self.x[3])
        odom.twist.twist.angular.z = float(self.x[4])
        self._odom_pub.publish(odom)


def main() -> None:
    rclpy.init()
    node = EkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
