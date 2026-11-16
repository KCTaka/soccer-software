"""The SIL stand's hand-written configs against each other: the controller and the keyframes."""
from bringup_files import CONFIG, load_yaml


def _controller():
    return load_yaml(CONFIG, 'hardware_sil.yaml')['humanoid_mit_controller']['ros__parameters']


def test_stand_gains_give_one_value_per_joint():
    # The controller refuses to configure otherwise, which only shows when SIL is launched.
    controller = _controller()
    assert len(controller['kp']) == len(controller['kd']) == len(controller['joints'])


def test_keyframes_follow_the_controllers_joint_order():
    # The controller reads a reference by index, not by joint name.
    keyframes = load_yaml(CONFIG, 'stand_keyframes.yaml')
    assert keyframes['joint_names'] == _controller()['joints']
    for i, frame in enumerate(keyframes['keyframes']):
        assert len(frame['positions']) == len(keyframes['joint_names']), f'keyframe {i}'
