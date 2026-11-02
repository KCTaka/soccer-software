import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)


def _config(name):
    with open(os.path.join(PACKAGE, 'config', name)) as f:
        return yaml.safe_load(f)


def _controller():
    return _config('hardware_sil.yaml')['humanoid_mit_controller']['ros__parameters']


def test_stand_gains_give_one_value_per_joint():
    # The controller refuses to configure otherwise, which only shows when SIL is launched.
    controller = _controller()
    assert len(controller['kp']) == len(controller['kd']) == len(controller['joints'])


def test_keyframes_follow_the_controllers_joint_order():
    # The controller reads a reference by index, not by joint name.
    keyframes = _config('stand_keyframes.yaml')
    assert keyframes['joint_names'] == _controller()['joints']
    for frame in keyframes['keyframes']:
        assert len(frame['positions']) == len(keyframes['joint_names'])
