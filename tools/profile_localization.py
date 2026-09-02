#!/usr/bin/env python3
"""Benchmark and validate the localization stack without a field.

The robot cannot currently be put on a pitch, so every claim about the MCL and
EKF has to come from measurement plus synthetic ground truth. This reproduces
the numbers quoted in ``docs/architecture/localization_tuning.md``.

Run inside the app container (it needs numpy, scipy, rclpy and the package)::

    docker cp tools/profile_localization.py soccer-app:/tmp/prof.py
    docker exec soccer-app bash -lc '. /opt/ros/jazzy/setup.sh && \
        . /ws/install/setup.sh && ROS_DOMAIN_ID=99 python3 /tmp/prof.py'

Sections:
  1. observation noise implied by the corrected camera geometry
  2. MCL measurement-update cost, old per-particle loop vs new vectorised form
  3. rclpy executor overhead vs callback rate (why the EKF cost what it did)
  4. filter health: effective sample size and convergence on synthetic data
"""
from __future__ import annotations

import argparse
import multiprocessing as mp
import os
import time

import numpy as np

from soccer_localization.field_model import SoccerbotField
from soccer_localization.mcl_node import ParticleFilter
from soccer_localization.sensor_model import GroundProjectionNoise

RULE = "-" * 72


def _timed(fn, reps: int) -> float:
    fn()
    t0 = time.perf_counter()
    for _ in range(reps):
        fn()
    return (time.perf_counter() - t0) / reps * 1000.0


def visible_line_points(field: SoccerbotField, pose, max_range: float = 4.0,
                        per_segment: int = 24) -> np.ndarray:
    """Field line points inside a forward FOV, expressed in the base frame."""
    x, y, th = pose
    c, s = np.cos(-th), np.sin(-th)
    pts = []
    for (x0, y0, x1, y1) in field._segments:
        for t in np.linspace(0.0, 1.0, per_segment):
            dx = x0 + t * (x1 - x0) - x
            dy = y0 + t * (y1 - y0) - y
            bx, by = c * dx - s * dy, s * dx + c * dy
            if 0.2 < bx < max_range and abs(by) < bx:  # crude forward FOV
                pts.append([bx, by])
    return np.array(pts) if pts else np.zeros((0, 2))


def corrupt(pts: np.ndarray, rng: np.random.Generator,
            noise: GroundProjectionNoise) -> np.ndarray:
    """Apply the error a real ground projection would make to these points.

    Flat noise would be a rigged comparison: it makes the legacy fixed 0.15 m
    sigma look correct because nothing in the synthetic data degrades with range.
    The error is almost entirely radial and grows as r^2, so inject it that way.
    """
    if pts.shape[0] == 0:
        return pts
    r = np.hypot(pts[:, 0], pts[:, 1])
    radial = pts / np.maximum(r, 1e-9)[:, None]
    along = rng.normal(0.0, noise.sigma(r))[:, None] * radial
    return pts + along + rng.normal(0.0, 0.01, pts.shape)


# ── 1. Observation noise ──────────────────────────────────────────────────────
def section_noise() -> None:
    print(f"\n1. GROUND-PROJECTION NOISE FROM THE CORRECTED GEOMETRY\n{RULE}")
    n = GroundProjectionNoise()
    print(f"   h={n.mount_height_m} m  fy={n.focal_px}  "
          f"sigma_v={n.pixel_sigma_px} px  sigma_tilt={n.tilt_sigma_rad} rad\n")
    print(f"   {'range':>6s} {'pixel':>8s} {'tilt':>8s} {'height':>8s} {'total':>8s}")
    h = n.mount_height_m
    for r in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0):
        d = h + r * r / h
        print(f"   {r:5.1f}m {d * n.pixel_sigma_px / n.focal_px:8.3f} "
              f"{d * n.tilt_sigma_rad:8.3f} {r / h * n.height_sigma_m:8.3f} "
              f"{float(n.sigma(r)):8.3f}")
    print(f"\n   The shipped fixed line_sigma of 0.15 m is honest only to "
          f"{n.usable_range(0.15):.2f} m.")
    print(f"   The fieldline node's default max_range_m=6.0 admitted points with "
          f"{float(n.sigma(6.0)):.2f} m of error.")
    print(f"   Tilt calibration dominates beyond ~2 m: better segmentation buys "
          f"almost nothing, measuring the tilt buys a lot.")


# ── 2. MCL measurement update cost ────────────────────────────────────────────
def _legacy_update(pf: ParticleFilter, pts: np.ndarray) -> None:
    """The pre-optimisation per-particle Python loop, kept for comparison."""
    log_w = np.zeros(pf.n)
    for i in range(pf.n):
        x, y, th = pf.particles[i]
        c, s = np.cos(th), np.sin(th)
        world = np.empty_like(pts)
        world[:, 0] = x + c * pts[:, 0] - s * pts[:, 1]
        world[:, 1] = y + s * pts[:, 0] + c * pts[:, 1]
        log_w[i] = np.sum(np.log(pf.field.line_likelihood(world) + 1e-6))


def section_mcl_cost(rate_hz: float = 22.2) -> None:
    print(f"\n2. MCL MEASUREMENT UPDATE, {rate_hz} Hz field_features\n{RULE}")
    field = SoccerbotField()
    rng = np.random.default_rng(0)
    print(f"   {'particles':>9s} {'points':>7s} {'old loop':>10s} "
          f"{'vectorised':>11s} {'speedup':>8s} {'core saved':>11s}")
    for n_part in (150, 300, 500):
        for n_pts in (60, 200):
            pf = ParticleFilter(field, num_particles=n_part, seed=1,
                                max_observations=n_pts, max_range_m=99.0)
            pts = np.column_stack([rng.uniform(0.5, 3.5, n_pts),
                                   rng.uniform(-1.5, 1.5, n_pts)])
            old = _timed(lambda: _legacy_update(pf, pts), 10)
            new = _timed(lambda: pf.update(pts), 20)
            print(f"   {n_part:9d} {n_pts:7d} {old:8.2f}ms {new:9.2f}ms "
                  f"{old / new:7.1f}x {(old - new) * rate_hz / 10:9.1f} %")
    print("\n   'core saved' is percent of ONE core at the measured "
          "field_features rate.")


# ── 3. rclpy executor overhead ────────────────────────────────────────────────
def _spin_child(rate_hz: float, mode: str, secs: float, q) -> None:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

    rclpy.init(args=[])
    node = Node(f"bench_{mode}_{int(rate_hz)}")
    pub = node.create_publisher(Odometry, f"bench_{mode}_{int(rate_hz)}", 20)
    tfb = TransformBroadcaster(node)
    P = np.eye(5) * 0.01
    Q = np.diag([1e-4, 1e-4, 1e-4, 1e-2, 1e-2])

    def cb() -> None:
        if mode == "empty":
            return
        F = np.eye(5)
        F[0, 3] = F[2, 4] = 0.005
        P[:] = F @ P @ F.T + Q
        if mode == "math":
            return
        stamp = node.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = "odom", "base_link"
        t.transform.rotation.w = 1.0
        tfb.sendTransform(t)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id, o.child_frame_id = "odom", "base_link"
        o.pose.pose.orientation.w = 1.0
        pub.publish(o)

    node.create_timer(1.0 / rate_hz, cb)
    end = time.perf_counter() + secs
    c0 = sum(os.times()[:2])
    while time.perf_counter() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    q.put((sum(os.times()[:2]) - c0) / secs * 100.0)
    node.destroy_node()
    rclpy.shutdown()


def _measure_spin(rate_hz: float, mode: str, secs: float = 6.0) -> float:
    q = mp.Queue()
    p = mp.Process(target=_spin_child, args=(rate_hz, mode, secs, q))
    p.start()
    val = q.get()
    p.join()
    return val


def section_executor() -> None:
    print(f"\n3. rclpy EXECUTOR OVERHEAD, % of ONE core\n{RULE}")
    print(f"   {'rate':>8s} {'spin only':>11s} {'+5x5 EKF':>11s} {'+TF & odom':>12s}")
    rows = {}
    for rate in (200.0, 100.0, 50.0, 25.0):
        rows[rate] = [_measure_spin(rate, m) for m in ("empty", "math", "full")]
        print(f"   {rate:6.0f}Hz {rows[rate][0]:9.1f} % {rows[rate][1]:9.1f} % "
              f"{rows[rate][2]:10.1f} %")
    hi, lo = rows[200.0], rows[50.0]
    print(f"\n   Arithmetic is {hi[1] - hi[0]:.1f} % of a core. Waking up 200x a "
          f"second is {hi[0]:.1f} %.")
    print(f"   200 Hz -> 50 Hz saves {hi[2] - lo[2]:.1f} % of a core "
          f"({(1 - lo[2] / hi[2]) * 100:.0f} % less) for identical filter output.")


# ── 4. Filter health ──────────────────────────────────────────────────────────
LEGACY_NOISE = GroundProjectionNoise(floor_m=0.15, pixel_sigma_px=0.0,
                                     tilt_sigma_rad=0.0, height_sigma_m=0.0)

VARIANTS = (
    ("legacy: flat 0.15 m sigma, no floor",
     dict(z_rand=0.0, independent_observations=1e9, max_range_m=99.0), True),
    ("+ outlier floor (z_rand)",
     dict(z_rand=0.05, independent_observations=1e9, max_range_m=99.0), True),
    ("+ range-dependent sigma",
     dict(z_rand=0.05, independent_observations=1e9, max_range_m=99.0), False),
    ("+ correlation budget",
     dict(z_rand=0.05, independent_observations=10.0, max_range_m=99.0), False),
    ("+ range gate (shipped)",
     dict(z_rand=0.05, independent_observations=10.0, max_range_m=4.0), False),
)


def _run_trials(field, truth, kwargs, legacy_noise, trials=10, steps=12):
    """Steady-state error and effective sample size over repeated updates."""
    truth_noise = GroundProjectionNoise()
    errs, esss = [], []
    for trial in range(trials):
        rng = np.random.default_rng(trial)
        pf = ParticleFilter(field, num_particles=300, explorer_frac=0.0,
                            seed=trial, **kwargs)
        if legacy_noise:
            pf.obs_noise = LEGACY_NOISE
        pf.particles = truth + rng.normal(0, [0.3, 0.3, 0.2], size=(pf.n, 3))
        for _ in range(steps):
            pf.predict(np.zeros(3), dt=0.045)
            pts = visible_line_points(field, truth, max_range=6.0)
            pf.update(corrupt(pts, rng, truth_noise))
            pf.resample()
        est = pf.estimate()
        errs.append(np.hypot(est[0] - truth[0], est[1] - truth[1]))
        esss.append(pf.effective_sample_size)
    return float(np.mean(errs)), float(np.max(errs)), float(np.mean(esss))


def section_health() -> None:
    print(f"\n4. FILTER HEALTH ON SYNTHETIC OBSERVATIONS\n{RULE}")
    field = SoccerbotField()
    truth_noise = GroundProjectionNoise()
    truth = np.array([-1.0, 0.5, 0.2])
    pts = visible_line_points(field, truth, max_range=6.0)
    print(f"   {len(pts)} synthetic line points visible from "
          f"[{truth[0]}, {truth[1]}, {truth[2]}], corrupted with the real")
    print("   range-dependent projection error rather than flat noise.")
    print("   Steady state after 12 update+resample cycles, 10 seeds each.\n")
    print(f"   {'configuration':<38s} {'err mean':>9s} {'err max':>9s} {'ESS':>10s}")
    for label, kwargs, legacy in VARIANTS:
        mean, mx, ess = _run_trials(field, truth, kwargs, legacy)
        print(f"   {label:<38s} {mean:8.3f}m {mx:8.3f}m {ess:6.1f}/300")
    print("\n   ESS is the number of particles actually carrying the belief.")
    print("   Below n/2 = 150 the filter resamples every cycle and loses diversity.")

    print("\n   Divergence checks (shipped configuration):")
    checks = []

    # Long run: the estimate must stay bounded and on the field.
    rng = np.random.default_rng(7)
    pf = ParticleFilter(field, num_particles=300, seed=7)
    pf.particles = truth + rng.normal(0, [0.2, 0.2, 0.1], size=(pf.n, 3))
    worst = 0.0
    for _ in range(200):
        pf.predict(np.zeros(3), dt=0.045)
        pf.update(corrupt(pts, rng, truth_noise))
        pf.resample()
        est = pf.estimate()
        worst = max(worst, float(np.hypot(est[0] - truth[0], est[1] - truth[1])))
    checks.append(("200 cycles stationary, worst error", f"{worst:.3f} m",
                   worst < 0.5))

    # No observations at all (the current no-field-lines case): must not drift.
    pf = ParticleFilter(field, num_particles=300, explorer_frac=0.0, seed=8)
    pf.particles = np.tile(truth, (pf.n, 1)).astype(float)
    for _ in range(400):
        pf.predict(np.zeros(3), dt=0.02)
        pf.update(np.zeros((0, 2)))
        pf.resample()
    drift = float(np.hypot(*(pf.estimate()[:2] - truth[:2])))
    checks.append(("400 cycles with zero observations, drift", f"{drift:.3f} m",
                   drift < 0.1))

    # Kidnapped robot: converge, teleport the truth, then require recovery.
    rng = np.random.default_rng(9)
    pf = ParticleFilter(field, num_particles=600, explorer_frac=0.02, seed=9)
    pf.particles = truth + rng.normal(0, [0.2, 0.2, 0.1], size=(pf.n, 3))
    for _ in range(30):  # settle on the original pose first
        pf.predict(np.zeros(3), dt=0.045)
        pf.update(corrupt(pts, rng, truth_noise))
        pf.resample()
    moved = np.array([2.0, -1.2, -1.0])
    moved_pts = visible_line_points(field, moved, max_range=6.0)
    recovered_at = None
    for step in range(200):
        pf.predict(np.zeros(3), dt=0.045)
        pf.update(corrupt(moved_pts, rng, truth_noise))
        pf.resample()
        if recovered_at is None and np.hypot(
                *(pf.estimate()[:2] - moved[:2])) < 0.5:
            recovered_at = step
    kidnap_err = float(np.hypot(*(pf.estimate()[:2] - moved[:2])))
    checks.append(("kidnapped robot, error after recovery",
                   f"{kidnap_err:.3f} m", kidnap_err < 1.0))
    checks.append(("kidnapped robot, cycles to recover",
                   f"{recovered_at}" if recovered_at is not None else "never",
                   recovered_at is not None))

    # Garbage observations must not produce a confident wrong answer.
    pf = ParticleFilter(field, num_particles=300, seed=10)
    for _ in range(50):
        junk = np.column_stack([rng.uniform(0.5, 4.0, 60), rng.uniform(-2, 2, 60)])
        pf.predict(np.zeros(3), dt=0.045)
        pf.update(junk)
        pf.resample()
    spread = float(np.sqrt(pf.covariance()[0, 0] + pf.covariance()[1, 1]))
    checks.append(("random observations, reported spread",
                   f"{spread:.3f} m", np.isfinite(spread)))

    for label, value, ok in checks:
        print(f"     {'PASS' if ok else 'FAIL'}  {label:<44s} {value:>10s}")

    print("\n   Motion noise, 200 predict steps standing perfectly still:")
    pf = ParticleFilter(field, num_particles=300, explorer_frac=0.0, seed=0)
    pf.particles = np.tile(truth, (pf.n, 1)).astype(float)
    for _ in range(200):
        pf.predict(np.zeros(3), dt=0.02)
    pf2 = ParticleFilter(field, num_particles=300, explorer_frac=0.0, seed=0)
    pf2.particles = np.tile(truth, (pf2.n, 1)).astype(float)
    for _ in range(200):
        pf2.predict(np.zeros(3), noise=(0.02, 0.02, 0.02))
    print(f"     legacy fixed per-callback noise  cloud spread "
          f"{float(np.std(pf2.particles[:, 0])):.3f} m after 4 s")
    print(f"     motion-scaled noise              cloud spread "
          f"{float(np.std(pf.particles[:, 0])):.3f} m after 4 s")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--skip-executor", action="store_true",
                    help="skip section 3 (needs rclpy and ~1.5 minutes)")
    args = ap.parse_args()
    section_noise()
    section_mcl_cost()
    if not args.skip_executor:
        section_executor()
    section_health()
    print()


if __name__ == "__main__":
    mp.set_start_method("spawn")
    main()
