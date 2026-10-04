import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PACKAGE, 'nodes'))

import walk_policy_core as core  # noqa: E402

POLICY_DIR = os.path.join(PACKAGE, 'policies', 'unitree_g1_walk')


def _policy_yaml():
    with open(os.path.join(POLICY_DIR, 'policy.yaml')) as f:
        return yaml.safe_load(f)


def _rotation(q_xyzw):
    """Rotation matrix of a unit quaternion, written out independently of the code under test."""
    x, y, z, w = q_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def test_gravity_of_an_upright_body_points_down_its_z_axis():
    assert np.allclose(core.projected_gravity([0, 0, 0, 1]), [0, 0, -1])


def test_gravity_matches_the_rotation_matrix_for_random_attitudes():
    rng = np.random.default_rng(1)
    for _ in range(100):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        expected = _rotation(q).T @ np.array([0.0, 0.0, -1.0])
        assert np.allclose(core.projected_gravity(q), expected)


def test_body_pitched_forward_sees_gravity_ahead_of_it():
    # Nose-down by 90 degrees about +y: the body's +x axis points at the ground.
    half = math.radians(90.0) / 2.0
    g = core.projected_gravity([0.0, math.sin(half), 0.0, math.cos(half)])
    assert np.allclose(g, [1.0, 0.0, 0.0], atol=1e-12)


def test_observation_layout_and_scales():
    cfg = _policy_yaml()['walk_policy']['ros__parameters']
    obs = core.build_observation(
        gyro=[1.0, 2.0, 3.0], gravity=np.array([0.1, 0.2, 0.3]), command=[1.0, 1.0, 1.0],
        q=np.array(cfg['default_angles']) + 0.5, dq=np.full(12, 2.0), last_action=np.full(12, 7.0),
        phase=0.25, cfg=cfg)
    assert obs.shape == (1, 47) and obs.dtype == np.float32
    o = obs[0]
    assert np.allclose(o[0:3], np.array([1.0, 2.0, 3.0]) * cfg['ang_vel_scale'])
    assert np.allclose(o[3:6], [0.1, 0.2, 0.3])
    assert np.allclose(o[6:9], cfg['command_scale'])
    assert np.allclose(o[9:21], 0.5 * cfg['dof_pos_scale'])
    assert np.allclose(o[21:33], 2.0 * cfg['dof_vel_scale'])
    assert np.allclose(o[33:45], 7.0)
    assert np.allclose(o[45:47], [1.0, 0.0], atol=1e-6)  # sin and cos of a quarter turn


def test_gait_phase_follows_the_episode_clock():
    assert core.gait_phase(0, 50.0, 0.8) == 0.0
    assert math.isclose(core.gait_phase(10, 50.0, 0.8), 0.25)
    assert math.isclose(core.gait_phase(40, 50.0, 0.8), 0.0, abs_tol=1e-12)


def test_action_maps_to_offsets_from_the_default_stance():
    assert np.allclose(core.action_to_targets([2.0, -2.0], [0.1, 0.3], 0.25), [0.6, -0.2])


def test_command_is_limited_to_the_trained_range():
    assert np.allclose(core.clip_command([3.0, -3.0, 0.5], [1.0, 1.0, 1.0]), [1.0, -1.0, 0.5])


def test_targets_are_limited_to_the_joint_stops():
    assert np.allclose(core.clip_to_limits([-1.0, 0.0, 2.0], [-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]),
                       [-0.5, 0.0, 0.5])


def test_ramp_reaches_its_goal_and_holds():
    start, goal = np.array([0.0, 1.0]), np.array([1.0, 0.0])
    assert np.allclose(core.ramp_targets(start, goal, 0.0, 2.0), start)
    assert np.allclose(core.ramp_targets(start, goal, 1.0, 2.0), [0.5, 0.5])
    assert np.allclose(core.ramp_targets(start, goal, 5.0, 2.0), goal)


def test_policy_gives_a_finite_action_that_depends_on_its_memory():
    policy = core.Policy(os.path.join(POLICY_DIR, 'policy.onnx'))
    obs = np.zeros((1, 47), np.float32)
    obs[0, 5] = -1.0  # upright
    first = policy.step(obs)
    second = policy.step(obs)
    assert first.shape == (12,) and np.all(np.isfinite(first))
    assert not np.allclose(first, second)  # the LSTM state carried over
    policy.reset()
    assert np.allclose(policy.step(obs), first)  # and reset restores the initial state


def test_policy_file_is_consistent():
    cfg = _policy_yaml()['walk_policy']['ros__parameters']
    gains = _policy_yaml()['humanoid_mit_controller']['ros__parameters']
    controlled = cfg['controlled_joints']
    assert cfg['policy_joints'] == controlled[:12]
    assert len(cfg['default_angles']) == 12
    assert len(gains['kp']) == len(gains['kd']) == len(controlled) == 29


def test_controlled_joints_are_the_controllers_joints_in_order():
    with open(os.path.join(PACKAGE, 'config', 'hardware_sil.yaml')) as f:
        controller = yaml.safe_load(f)['humanoid_mit_controller']['ros__parameters']
    assert _policy_yaml()['walk_policy']['ros__parameters']['controlled_joints'] == \
        controller['joints']


def test_sil_starts_in_the_policys_default_stance():
    params = _policy_yaml()
    walk = params['walk_policy']['ros__parameters']
    sim = params['mujoco_sim']['ros__parameters']
    assert sim['initial_pose.joints'] == walk['policy_joints']
    assert sim['initial_pose.positions'] == walk['default_angles']
