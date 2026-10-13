"""Reference-trajectory math for the bench, with no ROS in it.

Each trajectory yields a position DELTA relative to a joint's start pose q0 and the matching
velocity feed-forward (the analytic time-derivative). Both are continuous, and both are zero at the
start and end of the motion, so a reference stream begins and ends exactly at q0 with no jump. The
ROS node (bench_trajectories) adds q0 per joint and publishes; this module is unit-tested on its own.
"""
import math


def cosine_ramp(tau, a, b, ramp_s):
    """Smooth transition a->b over [0, ramp_s] with a raised-cosine shape.

    Returns (value, d value/dt). The velocity is zero at tau=0 and tau=ramp_s, so ramps join holds
    (and each other) with continuous velocity.
    """
    if ramp_s <= 0.0 or tau >= ramp_s:
        return (b, 0.0)
    if tau <= 0.0:
        return (a, 0.0)
    x = math.pi * tau / ramp_s
    value = a + (b - a) * 0.5 * (1.0 - math.cos(x))
    velocity = (b - a) * 0.5 * (math.pi / ramp_s) * math.sin(x)
    return (value, velocity)


class Sine:
    """q0 + A·e(t)·sin(ωt): a sine with a smooth cosine fade-in/out envelope e(t) so position and
    velocity start and end at q0 / 0 regardless of the end phase."""

    def __init__(self, amplitude_rad=0.2, frequency_hz=0.25, duration_s=20.0, fade_s=2.0):
        self.amplitude = float(amplitude_rad)
        self.omega = 2.0 * math.pi * float(frequency_hz)
        self._duration = float(duration_s)
        self.fade = min(float(fade_s), self._duration / 2.0)

    def duration(self):
        return self._duration

    def _envelope(self, t):
        """Envelope e(t) in [0,1] and its derivative; 0 (with 0 slope) at the two ends."""
        if t <= 0.0 or t >= self._duration or self.fade <= 0.0:
            return (1.0, 0.0) if 0.0 < t < self._duration else (0.0, 0.0)
        if t < self.fade:
            x = math.pi * t / self.fade
            return (0.5 * (1.0 - math.cos(x)), 0.5 * (math.pi / self.fade) * math.sin(x))
        if t > self._duration - self.fade:
            x = math.pi * (self._duration - t) / self.fade
            return (0.5 * (1.0 - math.cos(x)), -0.5 * (math.pi / self.fade) * math.sin(x))
        return (1.0, 0.0)

    def point(self, t):
        if t <= 0.0 or t >= self._duration:
            return (0.0, 0.0)
        e, de = self._envelope(t)
        s = math.sin(self.omega * t)
        c = math.cos(self.omega * t)
        pos = self.amplitude * e * s
        vel = self.amplitude * (de * s + e * self.omega * c)
        return (pos, vel)

    def extent(self):
        return (-abs(self.amplitude), abs(self.amplitude))


class Trap:
    """Per cycle: q0 -> +step -> hold -> q0 -> hold -> -step -> hold -> q0, each move a raised-cosine
    ramp (continuous velocity, zero at the ends of every segment)."""

    def __init__(self, step_rad=0.1, ramp_s=2.0, hold_s=2.0, cycles=3):
        self.step = float(step_rad)
        self.ramp = float(ramp_s)
        self.hold = float(hold_s)
        self.cycles = int(cycles)
        d = self.step
        # (kind, from, to); ramps last ramp_s, holds last hold_s.
        self._segments = [
            ('ramp', 0.0, d), ('hold', d, d),
            ('ramp', d, 0.0), ('hold', 0.0, 0.0),
            ('ramp', 0.0, -d), ('hold', -d, -d),
            ('ramp', -d, 0.0),
        ]
        self._cycle_duration = 4.0 * self.ramp + 3.0 * self.hold

    def duration(self):
        return self.cycles * self._cycle_duration

    def point(self, t):
        if t <= 0.0 or t >= self.duration() or self._cycle_duration <= 0.0:
            return (0.0, 0.0)
        tc = math.fmod(t, self._cycle_duration)
        for kind, a, b in self._segments:
            seg_len = self.ramp if kind == 'ramp' else self.hold
            if seg_len <= 0.0:
                continue
            if tc <= seg_len:
                return cosine_ramp(tc, a, b, self.ramp) if kind == 'ramp' else (a, 0.0)
            tc -= seg_len
        return (0.0, 0.0)

    def extent(self):
        return (-abs(self.step), abs(self.step))


def make_trajectory(mode, **params):
    if mode == 'sine':
        return Sine(
            amplitude_rad=params.get('amplitude_rad', 0.2),
            frequency_hz=params.get('frequency_hz', 0.25),
            duration_s=params.get('duration_s', 20.0),
            fade_s=params.get('fade_s', 2.0))
    if mode == 'trap':
        return Trap(
            step_rad=params.get('step_rad', 0.1),
            ramp_s=params.get('ramp_s', 2.0),
            hold_s=params.get('hold_s', 2.0),
            cycles=params.get('cycles', 3))
    raise ValueError(f"unknown mode {mode!r} (want 'sine' or 'trap')")


def limit_violation(q0, lower, upper, extent):
    """How far (rad) the trajectory centred on q0 would exceed [lower, upper], else 0.0."""
    under = lower - (q0 + extent[0])
    over = (q0 + extent[1]) - upper
    return max(0.0, under, over)
