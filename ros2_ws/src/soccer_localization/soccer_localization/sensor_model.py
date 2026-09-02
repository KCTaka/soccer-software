"""Observation and motion noise models derived from the *corrected* camera geometry.

Until 2026-09-02 the perception stack projected image points to the ground with
hardcoded VGA intrinsics (``fx=fy=550, cx=320, cy=240``) while the ZED Mini was
actually producing 1280x720 with ``fx=fy=732.9``. Ground positions were wrong by
2.5-8.5 m and, above the horizon, entirely fabricated. Any noise value tuned by
observing filter behaviour before that date was tuned against garbage, so this
module derives the noise from first principles instead.

Nothing here is fitted. Every number falls out of the projection geometry and a
handful of physically measurable calibration tolerances.

Ground-projection error
-----------------------
A camera at height ``h`` looking down at a depression angle ``alpha`` sees a
ground point at range ``r = h / tan(alpha)``. Differentiating::

    dr/dalpha = -(h + r^2 / h)

The ``r^2 / h`` term is the whole story: projection error grows with the *square*
of range and inversely with mount height. A robot camera 0.30 m off the ground
amplifies a given angular error 30x more at 3 m than at 1 m. Three independent
error sources feed through it:

===================  =========================================================
Source               Contribution to the range standard deviation
===================  =========================================================
Pixel noise          ``(h + r^2/h) * sigma_v / fy``
Tilt calibration     ``(h + r^2/h) * sigma_tilt``
Mount-height error   ``(r / h) * sigma_h``
===================  =========================================================

For the shipped configuration (``h = 0.30 m``, ``fy = 732.9``, ``sigma_v = 2 px``,
``sigma_tilt = 0.5 deg``, ``sigma_h = 10 mm``) that gives:

======  =========  =========  =========  ===========
r (m)   pixel (m)  tilt (m)   height (m) total (m)
======  =========  =========  =========  ===========
1.0     0.010      0.032      0.033      0.047
2.0     0.037      0.119      0.067      0.142
3.0     0.083      0.264      0.100      0.294
4.0     0.146      0.467      0.133      0.507
6.0     0.328      1.047      0.200      1.114
======  =========  =========  =========  ===========

Two conclusions follow, and both contradict the shipped parameters:

1. The fixed ``line_sigma = 0.15 m`` is only honest out to about 2 m. Beyond 3 m
   the true uncertainty is 2-7x larger, so the filter was treating far-range
   observations as far more trustworthy than they are. That is a direct cause of
   the particle depletion measured before this change (effective sample size 1.0
   out of 300 on *every* update).
2. Tilt calibration dominates past ~2 m, not pixel noise. Improving the
   segmentation buys almost nothing; measuring the camera tilt buys a lot.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class GroundProjectionNoise:
    """Range-dependent standard deviation of a ground-projected image point.

    ``focal_px`` is the one camera intrinsic in this repository that is not read
    from ``camera_info``, which deserves an explanation given that hardcoding
    intrinsics is exactly what caused the bug this module exists to answer.

    It is tolerable here, and only here, because of where it enters. In the
    projection itself the focal length scales the *position* of every point, so
    a wrong value moved points by metres. Here it scales one term of an
    uncertainty, and not the dominant one: at 4 m the pixel term contributes
    0.146 m of a 0.509 m total, so a 10 % error in the focal length moves the
    reported sigma by about 1.5 %. Subscribing to ``camera_info`` purely to
    obtain it would cost roughly 3 % of a CPU core in executor wakeups for a
    value that never changes.

    It is still a ROS parameter, and ``tools/check_repo_invariants.py`` checks
    that the mounting geometry agrees across the field-line node and this one.
    """

    focal_px: float = 732.9          # ZED Mini HD720 fy
    mount_height_m: float = 0.30     # optical centre above the ground
    pixel_sigma_px: float = 2.0      # line-centroid noise, full-resolution pixels
    tilt_sigma_rad: float = 0.0087   # 0.5 deg of mount/IMU tilt calibration error
    height_sigma_m: float = 0.01     # 10 mm of mount-height error
    floor_m: float = 0.05            # never claim better than one grid cell

    def sigma(self, ranges: np.ndarray) -> np.ndarray:
        """Standard deviation in metres for ground ranges (any shape)."""
        r = np.asarray(ranges, dtype=np.float64)
        h = self.mount_height_m
        d_range_d_angle = h + r * r / h
        s_pixel = d_range_d_angle * self.pixel_sigma_px / self.focal_px
        s_tilt = d_range_d_angle * self.tilt_sigma_rad
        s_height = r / h * self.height_sigma_m
        return np.sqrt(self.floor_m ** 2 + s_pixel ** 2
                       + s_tilt ** 2 + s_height ** 2)

    def usable_range(self, max_sigma_m: float) -> float:
        """Largest range whose projection uncertainty stays within `max_sigma_m`.

        Used to justify the observation range gate instead of guessing it: the
        default 6 m gate admits points with over a metre of uncertainty.
        """
        r = np.linspace(0.0, 20.0, 4001)
        ok = r[self.sigma(r) <= max_sigma_m]
        return float(ok[-1]) if ok.size else 0.0


@dataclass
class OdometryNoise:
    """Motion-model noise, proportional to the motion rather than to the tick rate.

    The original filter added a fixed ``0.02 m`` of diffusion on *every* odometry
    callback. Because odometry ran at 200 Hz that is a 0.02 * sqrt(200) = 0.28 m/s
    random walk while standing perfectly still, and it silently changes meaning
    whenever the publish rate changes. Scaling by the reported motion (Thrun,
    *Probabilistic Robotics* §5.4) plus a small sqrt(dt) drift term makes the
    filter behave identically at any odometry rate.
    """

    alpha_trans_trans: float = 0.10   # m of translation error per m translated
    alpha_trans_rot: float = 0.05     # m of translation error per rad rotated
    alpha_rot_rot: float = 0.15       # rad of heading error per rad rotated
    alpha_rot_trans: float = 0.05     # rad of heading error per m translated
    drift_trans_m_per_sqrt_s: float = 0.01
    drift_rot_rad_per_sqrt_s: float = 0.01

    def sigma(self, delta: np.ndarray, dt: float) -> np.ndarray:
        """``[sigma_x, sigma_y, sigma_theta]`` for one odometry step."""
        trans = float(np.hypot(delta[0], delta[1]))
        rot = abs(float(delta[2]))
        root_dt = np.sqrt(max(dt, 0.0))
        s_trans = np.hypot(
            self.alpha_trans_trans * trans + self.alpha_trans_rot * rot,
            self.drift_trans_m_per_sqrt_s * root_dt,
        )
        s_rot = np.hypot(
            self.alpha_rot_rot * rot + self.alpha_rot_trans * trans,
            self.drift_rot_rad_per_sqrt_s * root_dt,
        )
        return np.array([s_trans, s_trans, s_rot])
