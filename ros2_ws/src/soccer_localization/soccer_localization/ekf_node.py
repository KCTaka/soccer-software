"""Tier-1 odometry: EKF sensor fusion (localization report §3, §5).

Smooth, high-rate local odometry — answers "how have I moved since the last
instant?" It fuses the IMU (orientation + yaw rate) with joint/leg odometry
(and, on the full robot, ZED Visual-Inertial Odometry) into ``odom -> base_link``.
This is the tier the blueprint calls "robot_localization"; in production it is the
``robot_localization`` EKF configured by YAML, here it is a compact equivalent so
the graph is complete. It feeds the MCL's motion model; it does NOT resolve field
symmetry — that is the MCL's job (Tier-2).

State x = [px, py, theta, v, omega].

.. warning::

   **Translation is not observed.** Nothing in this node measures ``v``: there is
   no wheel encoder, no leg odometry and no VIO input wired up, so ``v`` stays at
   its initial zero and ``px``/``py`` never move. Measured on the running robot,
   1200 consecutive ``/odom`` samples reported position exactly (0, 0) and linear
   velocity exactly 0. This node currently publishes **heading-only odometry**.

   Rather than emit a confident-looking pose that is silently meaningless — the
   failure mode that hid the camera intrinsics bug for so long — the position and
   linear-velocity entries of the published covariance are set to
   :data:`UNOBSERVED_VARIANCE` and the node says so on startup. Wire a
   translation source (leg odometry or ZED ``pose``) before trusting ``/odom``
   for anything but heading.

Rate
----
The predict step is nearly free (0.02 ms) but *waking up* to run it is not: a
bare rclpy node with a 200 Hz timer burns 18.2 % of a core before doing any work
at all, and 29.7 % once it also publishes TF and Odometry. Measured cost against
timer rate, in percent of one core::

    rate    spin only   + 5x5 EKF   + TF & odom
    200 Hz     18.2 %      19.0 %        29.7 %
    100 Hz     10.3 %      11.3 %        16.8 %
     50 Hz      5.5 %       6.0 %         9.3 %
     25 Hz      2.7 %       3.0 %         4.7 %

So the prediction is now driven by IMU arrivals, which the node has to wake for
anyway, and the timer only publishes. ``publish_rate_hz`` defaults to 50 Hz;
tf2 interpolates between transforms, and the only consumer of ``/odom`` is the
MCL, whose measurement update runs at ~22 Hz.
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

#: Variance published for states this node does not actually estimate.
UNOBSERVED_VARIANCE = 1e6


def _wrap(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


class EkfNode(Node):
    def __init__(self) -> None:
        super().__init__("ekf_node")
        self.declare_parameter("publish_rate_hz", 50.0)
        # Process noise for [px, py, theta, v, omega]. Position and velocity are
        # unobserved, so their entries only control how fast the (meaningless)
        # covariance grows.
        self.declare_parameter("process_noise",
                               [1e-4, 1e-4, 1e-4, 1e-2, 1e-2])
        # The ZED reports genuine measurement covariances (yaw 2.7e-10 rad^2,
        # yaw rate 5.2e-6 rad^2/s^2), which is far tighter than the 1e-3 this
        # node used to hardcode. Use what the sensor reports, but never claim
        # better than these floors: the reported figures describe short-term
        # noise, not the unbounded yaw drift of a gyro with no magnetometer.
        self.declare_parameter("yaw_variance_floor", 1e-6)
        self.declare_parameter("yaw_rate_variance_floor", 1e-6)

        self.x = np.zeros(5)                 # [px, py, theta, v, omega]
        self.P = np.eye(5) * 0.01
        self.Q = np.diag(
            np.asarray(self.get_parameter("process_noise").value, dtype=float))
        self._yaw_var_floor = float(self.get_parameter("yaw_variance_floor").value)
        self._rate_var_floor = float(
            self.get_parameter("yaw_rate_variance_floor").value)
        self._theta_bias = None
        self._last = self.get_clock().now()
        self._imu_seen = False

        rate = float(self.get_parameter("publish_rate_hz").value)
        self._tf = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "odom", 20)
        # IMU is a high-rate sensor stream -> best-effort SensorData QoS. The
        # prediction rides on these callbacks so the node needs no fast timer.
        self.create_subscription(Imu, "imu/data", self._on_imu, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().warn(
            "ekf_node up (%.0f Hz odometry, heading only). No translation source "
            "is wired up, so pose x/y stay at 0 and are published with variance "
            "%.0e. Do not use /odom position until leg odometry or ZED VIO feeds "
            "this node." % (rate, UNOBSERVED_VARIANCE))

    # ── Filter ──
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

    def _advance(self) -> None:
        """Bring the state up to the current clock. Safe to call from any path."""
        now = self.get_clock().now()
        dt = (now - self._last).nanoseconds * 1e-9
        self._last = now
        if 0.0 < dt <= 0.5:
            self._predict(dt)

    def _measurement_noise(self, msg: Imu) -> np.ndarray:
        """Per-message R, taken from the driver where it is available."""
        yaw_var = float(msg.orientation_covariance[8])
        rate_var = float(msg.angular_velocity_covariance[8])
        # ROS convention: a leading -1 means the sensor provides no estimate.
        if msg.orientation_covariance[0] < 0.0 or not np.isfinite(yaw_var):
            yaw_var = 0.0
        if msg.angular_velocity_covariance[0] < 0.0 or not np.isfinite(rate_var):
            rate_var = 0.0
        return np.diag([max(yaw_var, self._yaw_var_floor),
                        max(rate_var, self._rate_var_floor)])

    def _on_imu(self, msg: Imu) -> None:
        self._advance()
        q = msg.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        if self._theta_bias is None:
            self._theta_bias = yaw
            self._imu_seen = True
        z = np.array([_wrap(yaw - self._theta_bias), msg.angular_velocity.z])
        H = np.zeros((2, 5))
        H[0, 2] = 1.0
        H[1, 4] = 1.0
        y = z - H @ self.x
        y[0] = _wrap(y[0])
        S = H @ self.P @ H.T + self._measurement_noise(msg)
        # solve() rather than inv(): same result, better conditioned, and it does
        # not build an explicit inverse of a matrix that can become near-singular
        # once the measurement covariance is as small as the ZED reports.
        K = np.linalg.solve(S, (self.P @ H.T).T).T
        self.x = self.x + K @ y
        self.x[2] = _wrap(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P
        # Keep P symmetric. Over a match this runs ~10^5 times and the
        # (I - KH) P form slowly loses symmetry to floating-point error.
        self.P = 0.5 * (self.P + self.P.T)

    # ── Output ──
    def _publish(self) -> None:
        self._advance()
        if not self._imu_seen:
            self.get_logger().warn("no imu/data yet -- publishing identity odometry.",
                                   throttle_duration_sec=10.0)

        stamp = self.get_clock().now().to_msg()
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
        # Say out loud which of these numbers mean anything. Position and linear
        # velocity have no measurement behind them; heading and yaw rate do.
        odom.pose.covariance[0] = UNOBSERVED_VARIANCE     # x
        odom.pose.covariance[7] = UNOBSERVED_VARIANCE     # y
        odom.pose.covariance[35] = float(self.P[2, 2])    # yaw
        odom.twist.covariance[0] = UNOBSERVED_VARIANCE    # vx
        odom.twist.covariance[35] = float(self.P[4, 4])   # yaw rate
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
