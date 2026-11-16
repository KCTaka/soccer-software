"""Each SIL robot's hand-written stand configs against each other and its generated URDF: the
controller's joints and gains, and the keyframes."""
import pytest

from bringup_files import CONFIG, commanded_joints, load_yaml, sil_robots, urdf

ROBOTS = sil_robots()


def _controller(robot):
    return load_yaml(CONFIG, robot, 'sil_stand.yaml')['humanoid_mit_controller']['ros__parameters']


def _keyframes(robot):
    return load_yaml(CONFIG, robot, 'stand_keyframes.yaml')


def test_the_default_robot_is_among_the_sil_robots():
    assert 'unitree_g1' in ROBOTS


@pytest.mark.parametrize('robot', ROBOTS)
def test_stand_gains_give_one_value_per_joint(robot):
    # The controller refuses to configure otherwise, which only shows when SIL is launched.
    controller = _controller(robot)
    assert len(controller['kp']) == len(controller['kd']) == len(controller['joints'])


@pytest.mark.parametrize('robot', ROBOTS)
def test_keyframes_follow_the_controllers_joint_order(robot):
    # The controller reads a reference by index, not by joint name.
    keyframes = _keyframes(robot)
    assert keyframes['joint_names'] == _controller(robot)['joints']
    for i, frame in enumerate(keyframes['keyframes']):
        assert len(frame['positions']) == len(keyframes['joint_names']), f'keyframe {i}'


@pytest.mark.parametrize('robot', ROBOTS)
def test_controller_joints_are_joints_the_description_commands(robot):
    commanded = commanded_joints(urdf(robot))
    for joint in _controller(robot)['joints']:
        assert joint in commanded, f'{joint} is not commanded by the {robot} URDF'


@pytest.mark.parametrize('robot', ROBOTS)
def test_keyframe_positions_are_inside_the_joint_limits(robot):
    limits = {j.get('name'): j.find('limit') for j in urdf(robot).findall('joint')}
    keyframes = _keyframes(robot)
    for i, frame in enumerate(keyframes['keyframes']):
        for joint, position in zip(keyframes['joint_names'], frame['positions']):
            limit = limits.get(joint)
            assert limit is not None, f'{joint} has no <limit> in the {robot} URDF'
            lower, upper = float(limit.get('lower')), float(limit.get('upper'))
            assert lower <= position <= upper, \
                f'keyframe {i}: {joint} = {position} is outside [{lower}, {upper}]'
