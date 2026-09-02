"""Tier-2 global localization: Monte-Carlo Localization (localization report §3.1).

A multi-hypothesis particle filter — the correct tool for a *symmetric* field
where the belief is inherently multi-modal and a single-hypothesis EKF/UKF can
lock onto the wrong half. Each step:

  1. **Drift**    — apply the odometry delta from Tier-1.
  2. **Diffuse**  — add process noise scaled by that delta, not by the tick rate.
  3. **Measure**  — weight every particle at once against the precomputed
                    distance field (vectorised O(1) Chamfer matching), using a
                    per-observation sigma that grows with the point's range.
  4. **Resample** — importance resampling + a fraction of **explorer particles**
                    re-seeded uniformly for kidnapped-robot / penalty-return recovery.
  5. **Estimate** — weighted mean + covariance -> the ``map -> odom`` correction.

The :class:`ParticleFilter` is ROS-free so it can be unit-tested directly.
"""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from soccer_msgs.msg import FieldFeature, FieldFeatureArray
from tf2_ros import TransformBroadcaster

from soccer_localization.field_model import SoccerbotField
from soccer_localization.sensor_model import GroundProjectionNoise, OdometryNoise


def _wrap(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


class ParticleFilter:
    """Field-feature MCL over poses [x, y, theta]."""

    def __init__(self, field: SoccerbotField, num_particles: int = 300,
                 explorer_frac: float = 0.05, seed: int = 0,
                 obs_noise: GroundProjectionNoise | None = None,
                 odom_noise: OdometryNoise | None = None,
                 max_observations: int = 60,
                 max_range_m: float = 4.0,
                 z_rand: float = 0.05,
                 independent_observations: float = 10.0) -> None:
        self.field = field
        self.n = num_particles
        self.explorer_frac = explorer_frac
        self.rng = np.random.default_rng(seed)
        self.obs_noise = obs_noise or GroundProjectionNoise()
        self.odom_noise = odom_noise or OdometryNoise()
        self.max_observations = max_observations
        self.max_range_m = max_range_m
        self.z_rand = z_rand
        self.independent_observations = independent_observations
        self.particles = self.field.random_poses(self.rng, self.n)
        self.weights = np.full(self.n, 1.0 / self.n)

    # 1-2. Motion model: drift by odometry delta + diffuse proportionally to it.
    def predict(self, d: np.ndarray, dt: float = 0.0, noise=None) -> None:
        c, s = np.cos(self.particles[:, 2]), np.sin(self.particles[:, 2])
        self.particles[:, 0] += c * d[0] - s * d[1]
        self.particles[:, 1] += s * d[0] + c * d[1]
        self.particles[:, 2] = _wrap(self.particles[:, 2] + d[2])
        sigma = self.odom_noise.sigma(d, dt) if noise is None else np.asarray(noise)
        if np.any(sigma > 0.0):
            self.particles += self.rng.normal(0.0, sigma, self.particles.shape)
        self.particles[:, 2] = _wrap(self.particles[:, 2])

    # 3a. Reject observations the projection geometry cannot support.
    def gate(self, pts_base: np.ndarray) -> np.ndarray:
        """Range-gate and subsample observations before they reach the filter.

        Points beyond ``max_range_m`` carry more than a metre of ground-projection
        uncertainty (see :mod:`soccer_localization.sensor_model`), and hundreds of
        points along the same line are not hundreds of independent measurements.
        """
        if pts_base.shape[0] == 0:
            return pts_base.reshape(0, 2)
        r = np.hypot(pts_base[:, 0], pts_base[:, 1])
        keep = pts_base[r <= self.max_range_m]
        if keep.shape[0] > self.max_observations:
            idx = self.rng.choice(keep.shape[0], self.max_observations, replace=False)
            keep = keep[idx]
        return keep

    # 3b. Measurement model: one vectorised distance-field lookup for every
    #     particle-observation pair, with a per-observation sigma.
    def update(self, pts_base: np.ndarray) -> None:
        pts = self.gate(np.asarray(pts_base, dtype=np.float64))
        if pts.shape[0] == 0:
            return
        cos_th = np.cos(self.particles[:, 2])[:, None]
        sin_th = np.sin(self.particles[:, 2])[:, None]
        px, py = pts[None, :, 0], pts[None, :, 1]
        world = np.empty((self.n, pts.shape[0], 2))
        world[:, :, 0] = self.particles[:, 0][:, None] + cos_th * px - sin_th * py
        world[:, :, 1] = self.particles[:, 1][:, None] + sin_th * px + cos_th * py

        sigma = self.obs_noise.sigma(np.hypot(pts[:, 0], pts[:, 1]))[None, :]
        dist = self.field.line_distance(world)
        # Gaussian hit + uniform outlier floor: one bad point can no longer veto a
        # particle, which is what drove the effective sample size to 1.0.
        lik = (1.0 - self.z_rand) * np.exp(-0.5 * (dist / sigma) ** 2) + self.z_rand

        # Line points are strongly correlated along each segment, so treating all
        # of them as independent evidence makes the filter absurdly overconfident.
        # Normalise to a fixed budget of independent observations, which also makes
        # the weights invariant to how many points perception happens to emit.
        scale = min(1.0, self.independent_observations / pts.shape[0])
        log_w = scale * np.log(lik).sum(axis=1)

        log_w -= log_w.max()
        self.weights *= np.exp(log_w)
        total = self.weights.sum()
        self.weights = (np.full(self.n, 1.0 / self.n)
                        if total < 1e-12 else self.weights / total)

    @property
    def effective_sample_size(self) -> float:
        """Number of particles actually carrying the belief. Healthy is > n/2."""
        return float(1.0 / np.sum(self.weights ** 2))

    # 4. Systematic resampling + explorer-particle injection.
    def resample(self) -> None:
        if self.effective_sample_size > self.n / 2.0:
            return  # effective sample size healthy -> skip
        positions = (np.arange(self.n) + self.rng.random()) / self.n
        cumsum = np.cumsum(self.weights)
        idx = np.searchsorted(cumsum, positions)
        idx = np.clip(idx, 0, self.n - 1)
        self.particles = self.particles[idx]
        n_expl = int(self.explorer_frac * self.n)
        if n_expl:  # kidnapped-robot recovery
            self.particles[:n_expl] = self.field.random_poses(self.rng, n_expl)
        self.weights = np.full(self.n, 1.0 / self.n)

    # 5. Weighted estimate (circular mean for theta).
    def estimate(self) -> np.ndarray:
        x = np.average(self.particles[:, 0], weights=self.weights)
        y = np.average(self.particles[:, 1], weights=self.weights)
        th = np.arctan2(
            np.average(np.sin(self.particles[:, 2]), weights=self.weights),
            np.average(np.cos(self.particles[:, 2]), weights=self.weights),
        )
        return np.array([x, y, th])

    def covariance(self) -> np.ndarray:
        """Weighted 3x3 covariance of the particle cloud, [x, y, theta].

        Published so consumers can tell a converged belief from a cloud spread
        across both halves of a symmetric field. Previously always zero.
        """
        est = self.estimate()
        d = np.empty_like(self.particles)
        d[:, 0] = self.particles[:, 0] - est[0]
        d[:, 1] = self.particles[:, 1] - est[1]
        d[:, 2] = _wrap(self.particles[:, 2] - est[2])
        return (self.weights[:, None] * d).T @ d / max(1e-12, self.weights.sum())


class MclNode(Node):
    """ROS wrapper: consumes field features + odom, publishes map->odom."""

    def __init__(self) -> None:
        super().__init__("mcl_node")
        self.declare_parameter("num_particles", 300)
        self.declare_parameter("explorer_frac", 0.05)
        # Observation gating. max_range_m is derived, not guessed: past ~4 m the
        # ground projection carries over half a metre of uncertainty.
        self.declare_parameter("max_observations", 60)
        self.declare_parameter("max_range_m", 4.0)
        self.declare_parameter("z_rand", 0.05)
        self.declare_parameter("independent_observations", 10.0)
        # Camera extrinsics feeding the range-dependent observation noise. These
        # must match the fieldline node's mount_height_m / tilt_rad.
        self.declare_parameter("mount_height_m", 0.30)
        self.declare_parameter("focal_px", 732.9)
        self.declare_parameter("pixel_sigma_px", 2.0)
        self.declare_parameter("tilt_sigma_rad", 0.0087)

        obs_noise = GroundProjectionNoise(
            focal_px=float(self.get_parameter("focal_px").value),
            mount_height_m=float(self.get_parameter("mount_height_m").value),
            pixel_sigma_px=float(self.get_parameter("pixel_sigma_px").value),
            tilt_sigma_rad=float(self.get_parameter("tilt_sigma_rad").value),
        )
        self._field = SoccerbotField()
        self._pf = ParticleFilter(
            self._field,
            int(self.get_parameter("num_particles").value),
            float(self.get_parameter("explorer_frac").value),
            obs_noise=obs_noise,
            max_observations=int(self.get_parameter("max_observations").value),
            max_range_m=float(self.get_parameter("max_range_m").value),
            z_rand=float(self.get_parameter("z_rand").value),
            independent_observations=float(
                self.get_parameter("independent_observations").value),
        )
        self._last_odom = None
        self._last_odom_t = None

        self._tf = TransformBroadcaster(self)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, "mcl_pose", 10)
        self.create_subscription(FieldFeatureArray, "field_features", self._on_feats, 5)
        self.create_subscription(Odometry, "odom", self._on_odom, 20)
        self.create_timer(0.1, self._publish)  # 10 Hz map->odom
        self.get_logger().info(
            "mcl_node up (%d particles, <=%d observations gated to %.1f m; "
            "projection sigma %.2f m at that range)."
            % (self._pf.n, self._pf.max_observations, self._pf.max_range_m,
               float(obs_noise.sigma(self._pf.max_range_m))))

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        cur = np.array([p.x, p.y, yaw])
        now = self.get_clock().now()
        if self._last_odom is not None:
            d = cur - self._last_odom
            d[2] = _wrap(np.array([d[2]]))[0]
            dt = (now - self._last_odom_t).nanoseconds * 1e-9
            self._pf.predict(d, dt=max(0.0, min(dt, 0.5)))
        self._last_odom = cur
        self._last_odom_t = now

    def _on_feats(self, msg: FieldFeatureArray) -> None:
        pts = np.array(
            [[f.position.x, f.position.y] for f in msg.features
             if f.type == FieldFeature.TYPE_LINE_POINT]
        )
        self._pf.update(pts if pts.ndim == 2 else pts.reshape(0, 2))
        self._pf.resample()

    def _publish(self) -> None:
        est = self._pf.estimate()
        # In a full system the map->odom transform is est composed with the inverse
        # of the latest odom; for soccerbot we publish the estimate as map->base_link.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = float(est[0])
        t.transform.translation.y = float(est[1])
        t.transform.rotation.z = float(np.sin(est[2] / 2))
        t.transform.rotation.w = float(np.cos(est[2] / 2))
        self._tf.sendTransform(t)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = t.header.stamp
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(est[0])
        msg.pose.pose.position.y = float(est[1])
        msg.pose.pose.orientation.z = t.transform.rotation.z
        msg.pose.pose.orientation.w = t.transform.rotation.w
        # Report the actual spread of the belief. On a symmetric field a cloud
        # split across both halves shows up here as a huge x variance, which is
        # the only way a consumer can tell "converged" from "two hypotheses".
        cov = self._pf.covariance()
        msg.pose.covariance[0] = float(cov[0, 0])   # xx
        msg.pose.covariance[1] = float(cov[0, 1])   # xy
        msg.pose.covariance[6] = float(cov[1, 0])   # yx
        msg.pose.covariance[7] = float(cov[1, 1])   # yy
        msg.pose.covariance[35] = float(cov[2, 2])  # yaw-yaw
        self._pose_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MclNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
