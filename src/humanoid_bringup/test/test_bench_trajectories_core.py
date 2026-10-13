"""Unit tests for the pure bench trajectory math (no ROS)."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PACKAGE, 'nodes'))

import bench_trajectories_core as core  # noqa: E402


def _numeric_velocity(traj, t, dt=1e-6):
    return (traj.point(t + dt)[0] - traj.point(t - dt)[0]) / (2.0 * dt)


# --- Sine ---------------------------------------------------------------------

def test_sine_starts_and_ends_at_q0_with_zero_velocity():
    s = core.Sine(amplitude_rad=0.3, frequency_hz=0.4, duration_s=10.0, fade_s=2.0)
    assert s.point(0.0) == (0.0, 0.0)
    assert s.point(s.duration()) == (0.0, 0.0)
    # Just inside the ends: still tiny (fade envelope -> 0).
    for t in (1e-3, s.duration() - 1e-3):
        pos, vel = s.point(t)
        assert abs(pos) < 1e-3 and abs(vel) < 1e-2


def test_sine_velocity_is_the_derivative_of_position():
    s = core.Sine(amplitude_rad=0.25, frequency_hz=0.3, duration_s=12.0, fade_s=3.0)
    # Avoid the two fade junctions (t=fade, t=T-fade) where acceleration kinks.
    for t in np.linspace(0.05, s.duration() - 0.05, 400):
        if abs(t - s.fade) < 1e-2 or abs(t - (s.duration() - s.fade)) < 1e-2:
            continue
        assert abs(s.point(t)[1] - _numeric_velocity(s, t)) < 1e-3


def test_sine_reaches_its_amplitude_in_the_middle():
    s = core.Sine(amplitude_rad=0.2, frequency_hz=0.5, duration_s=20.0, fade_s=2.0)
    peak = max(abs(s.point(t)[0]) for t in np.linspace(0, s.duration(), 2000))
    assert abs(peak - 0.2) < 1e-3
    assert s.extent() == (-0.2, 0.2)


# --- Trap ---------------------------------------------------------------------

def test_trap_starts_and_ends_at_q0_with_zero_velocity():
    tr = core.Trap(step_rad=0.1, ramp_s=2.0, hold_s=1.0, cycles=2)
    assert tr.point(0.0) == (0.0, 0.0)
    assert tr.point(tr.duration()) == (0.0, 0.0)


def test_trap_velocity_is_the_derivative_of_position():
    tr = core.Trap(step_rad=0.15, ramp_s=1.5, hold_s=1.0, cycles=2)
    for t in np.linspace(0.02, tr.duration() - 0.02, 800):
        assert abs(tr.point(t)[1] - _numeric_velocity(tr, t)) < 1e-3


def test_trap_is_continuous_in_position_and_velocity_at_segment_boundaries():
    tr = core.Trap(step_rad=0.1, ramp_s=2.0, hold_s=1.5, cycles=2)
    # Boundaries within one cycle: 2,3,5,6.5,8.5,9.5,11.5 ... just check each ramp/hold join.
    cycle = 4 * 2.0 + 3 * 1.5
    offs = [2.0, 3.5, 5.5, 7.0, 9.0, 10.5]  # ends of segs 1..6 within a cycle
    for c in range(tr.cycles):
        for b in offs:
            t = c * cycle + b
            before, after = tr.point(t - 1e-4), tr.point(t + 1e-4)
            assert abs(before[0] - after[0]) < 1e-3   # position continuous
            assert abs(before[1] - after[1]) < 1e-2   # velocity continuous (0 at boundaries)


def test_trap_reaches_plus_and_minus_step():
    tr = core.Trap(step_rad=0.1, ramp_s=1.0, hold_s=0.5, cycles=1)
    deltas = [tr.point(t)[0] for t in np.linspace(0, tr.duration(), 2000)]
    assert abs(max(deltas) - 0.1) < 1e-3
    assert abs(min(deltas) + 0.1) < 1e-3


# --- limits + factory ---------------------------------------------------------

def test_limit_violation_reports_the_overshoot():
    # q0=1.4, +-0.2 reaches [1.2, 1.6]; upper 1.5 -> over by 0.1; lower 1.0 -> ok.
    assert abs(core.limit_violation(1.4, 1.0, 1.5, (-0.2, 0.2)) - 0.1) < 1e-9
    assert core.limit_violation(0.0, -1.5, 1.5, (-0.2, 0.2)) == 0.0
    assert abs(core.limit_violation(-1.4, -1.5, 1.5, (-0.2, 0.2)) - 0.1) < 1e-9


def test_make_trajectory_selects_the_mode():
    assert isinstance(core.make_trajectory('sine'), core.Sine)
    assert isinstance(core.make_trajectory('trap'), core.Trap)
    try:
        core.make_trajectory('nope')
        assert False, "expected ValueError"
    except ValueError:
        pass
