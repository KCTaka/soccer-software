"""The arithmetic of the G1 walking policy, with no ROS in it.

Observation layout, scales and the action mapping follow unitree_rl_gym's g1_env.py
`compute_observations` and its deploy script; see policies/unitree_g1_walk/README.md.
"""
import math

import numpy as np

OBSERVATION_SIZE = 47


def projected_gravity(q_xyzw):
    """Gravity direction (unit, pointing down) in the body frame of attitude `q_xyzw`.

    `q_xyzw` is the body's orientation in the world, as sensor_msgs/Imu reports it.
    """
    x, y, z, w = q_xyzw
    return np.array([
        2.0 * (-z * x + w * y),
        -2.0 * (z * y + w * x),
        1.0 - 2.0 * (w * w + z * z),
    ])


def gait_phase(step, rate_hz, period_s):
    """Phase in [0, 1) after `step` policy steps, as the training environment's episode clock."""
    return ((step / rate_hz) % period_s) / period_s


def build_observation(gyro, gravity, command, q, dq, last_action, phase, cfg):
    """The 47-value observation, shape (1, 47), float32.

    `q`, `dq` and `last_action` hold the policy's 12 joints; `cfg` is a mapping with
    `default_angles`, `ang_vel_scale`, `command_scale`, `dof_pos_scale` and `dof_vel_scale`.
    """
    obs = np.concatenate([
        np.asarray(gyro) * cfg['ang_vel_scale'],
        gravity,
        np.asarray(command) * np.asarray(cfg['command_scale']),
        (np.asarray(q) - np.asarray(cfg['default_angles'])) * cfg['dof_pos_scale'],
        np.asarray(dq) * cfg['dof_vel_scale'],
        last_action,
        [math.sin(2.0 * math.pi * phase), math.cos(2.0 * math.pi * phase)],
    ])
    assert obs.size == OBSERVATION_SIZE
    return obs.astype(np.float32).reshape(1, OBSERVATION_SIZE)


def action_to_targets(action, default_angles, action_scale):
    """Joint position targets for the policy's 12 joints."""
    return np.asarray(default_angles) + action_scale * np.asarray(action)


def clip_to_limits(targets, lower, upper):
    """Targets limited to [lower, upper]. A learned policy does not know the joint stops: it asks
    for targets past them on about a fifth of its steps, which the safety kernel would treat as a
    fault."""
    return np.clip(np.asarray(targets), np.asarray(lower), np.asarray(upper))


def clip_command(command, limit):
    """Each command component limited to its trained range."""
    return np.clip(np.asarray(command, dtype=float), -np.asarray(limit), np.asarray(limit))


def ramp_targets(start, goal, elapsed_s, duration_s):
    """Linear move from `start` to `goal`, reaching it at `duration_s` and holding after."""
    alpha = 1.0 if duration_s <= 0.0 else min(1.0, max(0.0, elapsed_s / duration_s))
    return (1.0 - alpha) * np.asarray(start) + alpha * np.asarray(goal)


class Policy:
    """The ONNX policy with its LSTM state carried between steps."""

    def __init__(self, path):
        import onnxruntime  # imported here so the arithmetic above needs only numpy
        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = 1
        self._session = onnxruntime.InferenceSession(
            path, options, providers=['CPUExecutionProvider'])
        self.reset()

    def reset(self):
        self._h = np.zeros((1, 1, 64), np.float32)
        self._c = np.zeros((1, 1, 64), np.float32)

    def step(self, observation):
        action, self._h, self._c = self._session.run(
            None, {'obs': observation, 'h': self._h, 'c': self._c})
        return action[0]
