"""Geometric and numerical tests for the tier-1 EKF.

The robot is on a desk in a room with no field lines, so none of this can be
validated against ground truth on a pitch. Everything here is therefore either a
property that must hold for any correct implementation (rate independence,
covariance symmetry, wrap-around) or a statement about what the filter does
*not* know (translation).
"""
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import Imu

from soccer_localization.ekf_node import UNOBSERVED_VARIANCE, EkfNode, _wrap


@pytest.fixture(scope="module", autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def ekf():
    node = EkfNode()
    yield node
    node.destroy_node()


def _imu(yaw=0.0, rate=0.0, yaw_var=2.7e-10, rate_var=5.2e-6):
    """An IMU message shaped like the ZED's, whose covariances are real."""
    m = Imu()
    m.orientation.z = float(np.sin(yaw / 2))
    m.orientation.w = float(np.cos(yaw / 2))
    m.angular_velocity.z = float(rate)
    cov = [0.0] * 9
    cov[0] = cov[4] = cov[8] = yaw_var
    m.orientation_covariance = cov
    rcov = [0.0] * 9
    rcov[0] = rcov[4] = rcov[8] = rate_var
    m.angular_velocity_covariance = rcov
    return m


# ── Prediction ──
def test_heading_prediction_is_independent_of_the_step_size(ekf):
    """The whole point of moving predict onto the IMU callback is that the
    filter must not care how often it is stepped."""
    ekf.x[:] = [0.0, 0.0, 0.0, 0.0, 0.5]
    for _ in range(200):
        ekf._predict(1.0 / 200.0)
    fast = ekf.x[2]

    ekf.x[:] = [0.0, 0.0, 0.0, 0.0, 0.5]
    for _ in range(25):
        ekf._predict(1.0 / 25.0)
    slow = ekf.x[2]

    assert fast == pytest.approx(slow, abs=1e-12)
    assert fast == pytest.approx(0.5, abs=1e-12)


def test_prediction_grows_uncertainty(ekf):
    before = ekf.P[2, 2]
    ekf._predict(0.02)
    assert ekf.P[2, 2] > before


def test_heading_wraps_at_pi(ekf):
    ekf.x[:] = [0.0, 0.0, 3.0, 0.0, 2.0]
    ekf._predict(0.5)
    assert -np.pi <= ekf.x[2] <= np.pi
    assert ekf.x[2] == pytest.approx(_wrap(4.0))


# ── Measurement ──
def test_the_imus_reported_covariance_is_used(ekf, monkeypatch):
    """The node used to hardcode R = 1e-3. The ZED reports 2.7e-10 for yaw,
    six orders of magnitude tighter, and the correction must reflect that."""
    monkeypatch.setattr(ekf, "_advance", lambda: None)
    ekf._on_imu(_imu(yaw=0.0))          # latch the bias

    tight = EkfNode()
    loose = EkfNode()
    try:
        for node in (tight, loose):
            monkeypatch.setattr(node, "_advance", lambda: None)
            node._on_imu(_imu(yaw=0.0))
            node.P[2, 2] = 0.01
        tight._on_imu(_imu(yaw=0.4, yaw_var=1e-9))
        loose._on_imu(_imu(yaw=0.4, yaw_var=1e-1))
        assert tight.x[2] > loose.x[2]
        assert tight.x[2] == pytest.approx(0.4, abs=1e-3)
        assert loose.x[2] < 0.05
    finally:
        tight.destroy_node()
        loose.destroy_node()


def test_variance_floor_stops_the_filter_trusting_the_sensor_absolutely(ekf):
    """2.7e-10 rad^2 is 16 microradians. That describes the ZED's short-term
    noise, not the unbounded drift of a gyro with no magnetometer."""
    R = ekf._measurement_noise(_imu())
    assert R[0, 0] == ekf._yaw_var_floor
    assert R[1, 1] == pytest.approx(5.2e-6)


def test_a_sensor_reporting_no_estimate_falls_back_to_the_floor(ekf):
    msg = _imu()
    msg.orientation_covariance = [-1.0] + [0.0] * 8
    msg.angular_velocity_covariance = [-1.0] + [0.0] * 8
    R = ekf._measurement_noise(msg)
    assert R[0, 0] == ekf._yaw_var_floor
    assert R[1, 1] == ekf._rate_var_floor


def test_the_first_imu_message_defines_zero_heading(ekf, monkeypatch):
    monkeypatch.setattr(ekf, "_advance", lambda: None)
    ekf._on_imu(_imu(yaw=2.1))
    assert ekf.x[2] == pytest.approx(0.0, abs=1e-9)


def test_heading_correction_takes_the_short_way_round(ekf, monkeypatch):
    """A measurement just below -pi against a state just above +pi is a small
    correction, not a near-2*pi one."""
    monkeypatch.setattr(ekf, "_advance", lambda: None)
    ekf._on_imu(_imu(yaw=0.0))
    ekf.x[2] = np.pi - 0.05
    ekf.P[2, 2] = 0.01
    ekf._on_imu(_imu(yaw=-np.pi + 0.05))
    assert abs(_wrap(ekf.x[2] - np.pi)) < 0.06


# ── Numerical health ──
def test_covariance_stays_symmetric_and_positive_definite(ekf, monkeypatch):
    monkeypatch.setattr(ekf, "_advance", lambda: None)
    rng = np.random.default_rng(0)
    for _ in range(2000):
        ekf._predict(1.0 / 99.0)
        ekf._on_imu(_imu(yaw=float(rng.normal(0.0, 0.02)),
                         rate=float(rng.normal(0.0, 0.05))))
    assert np.allclose(ekf.P, ekf.P.T, atol=1e-15)
    assert np.all(np.linalg.eigvalsh(ekf.P) > 0.0)
    assert np.all(np.isfinite(ekf.x))


def test_the_filter_tracks_a_constant_turn(ekf, monkeypatch):
    monkeypatch.setattr(ekf, "_advance", lambda: None)
    ekf._on_imu(_imu(yaw=0.0))
    yaw = 0.0
    for _ in range(300):
        yaw += 0.5 / 99.0
        ekf._predict(1.0 / 99.0)
        ekf._on_imu(_imu(yaw=yaw, rate=0.5))
    assert ekf.x[2] == pytest.approx(yaw, abs=0.02)
    assert ekf.x[4] == pytest.approx(0.5, abs=0.02)


# ── What the filter does not know ──
def test_translation_is_never_observed(ekf, monkeypatch):
    """1200 consecutive samples on the running robot reported position (0, 0).
    This pins that behaviour so it is a documented property, not a surprise."""
    monkeypatch.setattr(ekf, "_advance", lambda: None)
    for i in range(500):
        ekf._predict(1.0 / 99.0)
        ekf._on_imu(_imu(yaw=0.01 * np.sin(i / 20.0), rate=0.1))
    assert ekf.x[0] == pytest.approx(0.0, abs=1e-12)
    assert ekf.x[1] == pytest.approx(0.0, abs=1e-12)
    assert ekf.x[3] == pytest.approx(0.0, abs=1e-9)


def test_published_covariance_admits_position_is_unknown(ekf):
    published = []
    ekf._odom_pub.publish = published.append
    ekf._tf.sendTransform = lambda _t: None
    ekf._publish()
    odom = published[0]
    assert odom.pose.covariance[0] == UNOBSERVED_VARIANCE
    assert odom.pose.covariance[7] == UNOBSERVED_VARIANCE
    assert odom.twist.covariance[0] == UNOBSERVED_VARIANCE
    assert 0.0 < odom.pose.covariance[35] < 1.0
    assert 0.0 < odom.twist.covariance[35] < 1.0


def test_a_stale_clock_jump_does_not_blow_up_the_filter(ekf):
    """_advance() clamps dt. A missed wakeup or a clock step must not integrate
    a huge step into the state."""
    before = ekf.P.copy()
    ekf._predict(0.0)
    assert np.all(np.isfinite(ekf.P))
    ekf.P = before
    ekf._last = ekf.get_clock().now() - rclpy.duration.Duration(seconds=30)
    ekf._advance()
    assert np.allclose(ekf.P, before)
