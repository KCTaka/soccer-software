"""Unit tests for the field model + MCL particle filter (no ROS required).

The robot cannot currently be placed on a pitch, so these are the only validation
the localization stack gets. They are deliberately geometric and synthetic: field
truth is generated from :class:`SoccerbotField` itself, corrupted with the error a
*real* ground projection would make, and fed back in.
"""
import numpy as np

from soccer_localization.field_model import SoccerbotField
from soccer_localization.mcl_node import ParticleFilter
from soccer_localization.sensor_model import GroundProjectionNoise, OdometryNoise


def test_likelihood_higher_on_lines():
    f = SoccerbotField()
    on_line = np.array([[0.0, 1.5]])      # on the halfway line (y in [-2,2])
    off_line = np.array([[1.3, 0.4]])     # empty grass
    assert f.line_likelihood(on_line)[0] > f.line_likelihood(off_line)[0]


def test_distance_field_is_metric():
    f = SoccerbotField()
    # The halfway line runs along x = 0, so distance tracks |x| nearby.
    assert f.line_distance(np.array([[0.0, 1.5]]))[0] < f.resolution
    assert abs(f.line_distance(np.array([[0.3, 1.5]]))[0] - 0.3) < f.resolution
    # Points off the baked map report the sentinel, not a silent zero.
    assert f.line_distance(np.array([[50.0, 50.0]]))[0] == f.off_grid_distance


def test_line_likelihood_accepts_a_stacked_particle_set():
    f = SoccerbotField()
    pts = np.zeros((7, 5, 2))            # 7 particles x 5 observations
    assert f.line_distance(pts).shape == (7, 5)
    # A per-observation sigma must broadcast against the observation axis.
    sigma = np.linspace(0.1, 0.5, 5)[None, :]
    assert f.line_likelihood(pts, sigma).shape == (7, 5)


# ── Observation noise model ───────────────────────────────────────────────────
def test_projection_noise_grows_with_the_square_of_range():
    n = GroundProjectionNoise()
    s2, s4 = float(n.sigma(2.0)), float(n.sigma(4.0))
    assert float(n.sigma(1.0)) < s2 < s4
    # The r^2 / h term dominates, so doubling range roughly quadruples the error.
    assert 3.0 < s4 / s2 < 5.0


def test_fixed_line_sigma_is_only_honest_at_short_range():
    """Guards the recalibration finding: 0.15 m encodes a 2 m assumption.

    The stack shipped a flat ``line_sigma`` of 0.15 m while the fieldline node
    emitted points out to 6 m, so far observations were weighted as if they were
    seven times more accurate than they are.
    """
    n = GroundProjectionNoise()
    assert 1.8 < n.usable_range(0.15) < 2.2
    assert float(n.sigma(6.0)) > 1.0     # what the old 6 m gate was admitting


def test_tilt_calibration_dominates_beyond_two_metres():
    """Justifies spending effort on extrinsics rather than on segmentation."""
    n = GroundProjectionNoise()
    d = n.mount_height_m + 3.0 ** 2 / n.mount_height_m
    from_pixels = d * n.pixel_sigma_px / n.focal_px
    from_tilt = d * n.tilt_sigma_rad
    assert from_tilt > 2.0 * from_pixels


# ── Motion noise model ────────────────────────────────────────────────────────
def test_motion_noise_is_independent_of_the_odometry_rate():
    """A standing robot must not random-walk faster just because odom is faster.

    The original filter added a fixed 0.02 m on *every* callback, so raising the
    odometry rate from 50 Hz to 200 Hz quadrupled the diffusion variance of a
    stationary belief. One second of standing still must now cost the same
    regardless of rate.
    """
    field = SoccerbotField()
    spreads = []
    for rate in (50, 200):
        pf = ParticleFilter(field, num_particles=500, explorer_frac=0.0, seed=0)
        pf.particles = np.zeros((pf.n, 3))
        for _ in range(rate):            # one second of standing still
            pf.predict(np.zeros(3), dt=1.0 / rate)
        spreads.append(float(np.std(pf.particles[:, 0])))
    assert abs(spreads[0] - spreads[1]) < 0.3 * max(spreads)


def test_motion_noise_scales_with_motion():
    noise = OdometryNoise()
    still = noise.sigma(np.zeros(3), dt=0.02)
    moving = noise.sigma(np.array([0.5, 0.0, 0.0]), dt=0.02)
    turning = noise.sigma(np.array([0.0, 0.0, 1.0]), dt=0.02)
    assert moving[0] > 10.0 * still[0]
    assert turning[2] > still[2]


# ── Measurement update ────────────────────────────────────────────────────────
def _reference_update(pf, pts):
    """The pre-optimisation per-particle loop, used to pin the vectorised form."""
    log_w = np.zeros(pf.n)
    sigma = pf.obs_noise.sigma(np.hypot(pts[:, 0], pts[:, 1]))
    for i in range(pf.n):
        x, y, th = pf.particles[i]
        c, s = np.cos(th), np.sin(th)
        world = np.empty_like(pts)
        world[:, 0] = x + c * pts[:, 0] - s * pts[:, 1]
        world[:, 1] = y + s * pts[:, 0] + c * pts[:, 1]
        lik = ((1.0 - pf.z_rand)
               * np.exp(-0.5 * (pf.field.line_distance(world) / sigma) ** 2)
               + pf.z_rand)
        log_w[i] = np.sum(np.log(lik))
    return log_w * min(1.0, pf.independent_observations / pts.shape[0])


def test_vectorised_update_matches_the_reference_loop():
    field = SoccerbotField()
    rng = np.random.default_rng(0)
    pf = ParticleFilter(field, num_particles=50, seed=4, max_observations=40,
                        max_range_m=99.0)
    pts = np.column_stack([rng.uniform(0.5, 3.5, 40), rng.uniform(-1.5, 1.5, 40)])

    expected = _reference_update(pf, pts)
    expected = np.exp(expected - expected.max())
    expected /= expected.sum()

    pf.update(pts)
    assert np.allclose(pf.weights, expected, atol=1e-9)


def test_observations_beyond_the_range_gate_are_dropped():
    field = SoccerbotField()
    pf = ParticleFilter(field, num_particles=10, seed=0, max_range_m=4.0,
                        max_observations=1000)
    pts = np.array([[1.0, 0.0], [3.0, 0.0], [5.0, 0.0], [9.0, 0.0]])
    assert pf.gate(pts).shape[0] == 2


def test_observations_are_subsampled_to_the_budget():
    field = SoccerbotField()
    pf = ParticleFilter(field, num_particles=10, seed=0, max_observations=25,
                        max_range_m=99.0)
    pts = np.column_stack([np.full(400, 1.0), np.linspace(-1, 1, 400)])
    assert pf.gate(pts).shape[0] == 25


def test_empty_observations_leave_the_belief_untouched():
    field = SoccerbotField()
    pf = ParticleFilter(field, num_particles=20, seed=0)
    before = pf.weights.copy()
    pf.update(np.zeros((0, 2)))
    assert np.array_equal(pf.weights, before)


# ── Filter behaviour ──────────────────────────────────────────────────────────
def _observations(field, pose, max_range=6.0, per_segment=24):
    """World line points visible from `pose`, expressed in the base frame."""
    x, y, th = pose
    c, s = np.cos(-th), np.sin(-th)
    pts = []
    for (x0, y0, x1, y1) in field._segments:
        for t in np.linspace(0, 1, per_segment):
            dx, dy = x0 + t * (x1 - x0) - x, y0 + t * (y1 - y0) - y
            bx, by = c * dx - s * dy, s * dx + c * dy
            if 0.2 < bx < max_range and abs(by) < bx:   # crude forward FOV
                pts.append([bx, by])
    return np.array(pts)


def _corrupt(pts, rng, noise):
    """Apply the error a real projection makes: radial and range-dependent."""
    r = np.hypot(pts[:, 0], pts[:, 1])
    radial = pts / np.maximum(r, 1e-9)[:, None]
    return (pts + rng.normal(0.0, noise.sigma(r))[:, None] * radial
            + rng.normal(0.0, 0.01, pts.shape))


def test_mcl_converges_when_seeded_near_truth():
    field = SoccerbotField()
    rng = np.random.default_rng(1)
    noise = GroundProjectionNoise()
    true_pose = np.array([-1.0, 0.5, 0.2])

    pf = ParticleFilter(field, num_particles=400, explorer_frac=0.0, seed=2)
    # Seed particles around the truth (a global init would be symmetric-ambiguous;
    # explorer particles handle that recovery, tested separately below).
    pf.particles = true_pose + rng.normal(0, [0.3, 0.3, 0.2], size=(pf.n, 3))

    for _ in range(8):
        pf.predict(np.zeros(3), dt=0.045)
        pf.update(_corrupt(_observations(field, true_pose), rng, noise))
        pf.resample()

    est = pf.estimate()
    assert np.hypot(est[0] - true_pose[0], est[1] - true_pose[1]) < 0.4


def test_mcl_keeps_a_healthy_effective_sample_size():
    """Over-confidence is as dangerous as divergence: it kills the alternatives."""
    field = SoccerbotField()
    rng = np.random.default_rng(5)
    noise = GroundProjectionNoise()
    truth = np.array([-1.0, 0.5, 0.2])
    pf = ParticleFilter(field, num_particles=300, explorer_frac=0.0, seed=5)
    pf.particles = truth + rng.normal(0, [0.2, 0.2, 0.1], size=(pf.n, 3))
    for _ in range(12):
        pf.predict(np.zeros(3), dt=0.045)
        pf.update(_corrupt(_observations(field, truth), rng, noise))
        pf.resample()
    assert pf.effective_sample_size > pf.n / 2.0


def test_mcl_does_not_drift_without_observations():
    """The current situation: a camera in a room sees no field lines at all."""
    field = SoccerbotField()
    truth = np.array([-1.0, 0.5, 0.2])
    pf = ParticleFilter(field, num_particles=300, explorer_frac=0.0, seed=6)
    pf.particles = np.tile(truth, (pf.n, 1)).astype(float)
    for _ in range(400):
        pf.predict(np.zeros(3), dt=0.02)
        pf.update(np.zeros((0, 2)))
        pf.resample()
    assert np.hypot(*(pf.estimate()[:2] - truth[:2])) < 0.1


def _symmetry_group(pose):
    """Poses that produce an identical set of line observations.

    Every line in the model - touchlines, goal lines, the halfway line and the
    centre circle - is symmetric under a mirror in x, a mirror in y, and a 180
    degree rotation. The goalposts at (+-3, +-1) share those symmetries, so no
    geometric feature in this field model can break them. Field symmetry must be
    resolved by the *initial* pose and preserved by odometry.
    """
    x, y, th = pose
    return [
        np.array([x, y, th]),
        np.array([-x, y, np.pi - th]),
        np.array([x, -y, -th]),
        np.array([-x, -y, th + np.pi]),
    ]


def test_field_lines_cannot_resolve_the_mirror_ambiguity():
    """Documents a limit of the stack, so nobody debugs it as a filter fault.

    Two mirrored poses generate the same observations, so the MCL landing on the
    wrong one is correct behaviour, not divergence. ``projection_node`` publishes
    ``TYPE_GOALPOST`` features to disambiguate, but (a) ``mcl_node`` filters them
    out and (b) the detector's class list is ["ball", "robot"] so goalposts are
    never produced. Even if both were fixed, the goalposts are symmetric too.
    """
    field = SoccerbotField()
    truth = np.array([2.0, -1.2, -1.0])
    mirrored = _symmetry_group(truth)[2]
    a = _observations(field, truth)
    b = _observations(field, mirrored)
    assert len(a) == len(b)
    # Same points, mirrored in the base frame's y axis.
    assert np.allclose(np.sort(a[:, 0]), np.sort(b[:, 0]), atol=1e-9)
    assert np.allclose(np.sort(a[:, 1]), np.sort(-b[:, 1]), atol=1e-9)


def test_kidnap_recovery_is_not_reliable_and_that_is_expected():
    """Pins a known limitation so it is not mistaken for a regression.

    Uniform explorer re-seeding is the only recovery path, and it is weak: a
    single lucky explorer rarely outvotes a large cluster already sitting on a
    wrong-but-plausible pose. Measured over 25 seeds at 400 particles and 250
    cycles (~11 s at 22 Hz): 0/25 recoveries with explorer_frac=0, 6/25 at 0.02,
    9/25 at 0.05. This asserts only that recovery is *possible* - that some
    particle mass reaches the new pose - not that it always wins.
    """
    field = SoccerbotField()
    rng = np.random.default_rng(9)
    noise = GroundProjectionNoise()
    start = np.array([-1.0, 0.5, 0.2])
    moved = np.array([2.0, -1.2, -1.0])
    pf = ParticleFilter(field, num_particles=400, explorer_frac=0.05, seed=9)
    pf.particles = start + rng.normal(0, [0.2, 0.2, 0.1], size=(pf.n, 3))
    for _ in range(30):                      # settle on the original pose
        pf.predict(np.zeros(3), dt=0.045)
        pf.update(_corrupt(_observations(field, start), rng, noise))
        pf.resample()
    assert np.hypot(*(pf.estimate()[:2] - start[:2])) < 0.4

    closest = np.inf
    for _ in range(250):                     # teleport, then watch the cloud
        pf.predict(np.zeros(3), dt=0.045)
        pf.update(_corrupt(_observations(field, moved), rng, noise))
        pf.resample()
        for candidate in _symmetry_group(moved):
            d = np.hypot(pf.particles[:, 0] - candidate[0],
                         pf.particles[:, 1] - candidate[1]).min()
            closest = min(closest, float(d))
    assert closest < 0.5, "no particle ever reached the new pose"


def test_covariance_reports_the_spread_of_the_belief():
    field = SoccerbotField()
    rng = np.random.default_rng(12)
    tight = ParticleFilter(field, num_particles=300, seed=12)
    tight.particles = rng.normal(0, [0.05, 0.05, 0.02], size=(300, 3))
    loose = ParticleFilter(field, num_particles=300, seed=13)
    loose.particles = field.random_poses(rng, 300)
    assert tight.covariance()[0, 0] < loose.covariance()[0, 0]
    assert np.all(np.isfinite(loose.covariance()))


def test_estimate_stays_finite_under_garbage_observations():
    field = SoccerbotField()
    rng = np.random.default_rng(14)
    pf = ParticleFilter(field, num_particles=200, seed=14)
    for _ in range(50):
        junk = np.column_stack([rng.uniform(0.5, 4.0, 60), rng.uniform(-2, 2, 60)])
        pf.predict(np.zeros(3), dt=0.045)
        pf.update(junk)
        pf.resample()
    assert np.all(np.isfinite(pf.estimate()))
    assert np.all(np.isfinite(pf.covariance()))
    assert abs(pf.estimate()[0]) < field.length     # still on the map


def test_explorer_particles_reseed_on_resample():
    field = SoccerbotField()
    pf = ParticleFilter(field, num_particles=100, explorer_frac=0.1, seed=3)
    # Force a degenerate weight distribution so resampling triggers.
    pf.weights = np.zeros(pf.n)
    pf.weights[0] = 1.0
    before = pf.particles.copy()
    pf.resample()
    # At least the explorer fraction should have been re-seeded (changed).
    changed = np.any(~np.isclose(pf.particles, before), axis=1).sum()
    assert changed >= int(0.1 * pf.n)
