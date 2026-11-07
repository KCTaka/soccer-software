"""The leg prototype's hand-written configs against its generated description and manifest.

Mistakes here only show when the leg is launched: a controller joint the description does not
command, or ankle gains that the linkage turns into motor gains the RS02 silently clamps.
"""
import importlib.util
import os
import xml.etree.ElementTree as ET

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
GENERATED = os.path.join(PACKAGE, '..', '..', 'model', 'robots', 'leg_prototype', 'generated')


def _yaml(*path):
    with open(os.path.join(*path)) as f:
        return yaml.safe_load(f)


def _controller():
    return _yaml(PACKAGE, 'config', 'leg_prototype', 'controllers.yaml')[
        'humanoid_mit_controller']['ros__parameters']


def _urdf():
    return ET.parse(os.path.join(GENERATED, 'robot.urdf')).getroot()


def test_the_controller_commands_the_descriptions_command_joints():
    # The controller reads references by index, so its order must be the description's too.
    commanded = [j.get('name') for j in _urdf().find('ros2_control').findall('joint')
                 if j.find('command_interface') is not None]
    assert _controller()['joints'] == commanded


def test_ankle_gains_stay_inside_the_motors_caps_through_the_linkage():
    # The hardware component multiplies the ankle's gains by ratio^2. The worst ratio is the
    # manifest's motor torque bound over the description's ankle torque bound, both generated.
    manifest = _yaml(GENERATED, 'safety_manifest.yaml')['envelopes']
    (motor, envelope), = manifest.items()
    transmission = _urdf().find('ros2_control').find('transmission')
    assert transmission.find('actuator').get('name') == motor
    output = next(j.get('name') for j in transmission.findall('joint')
                  if j.get('role') == 'output')
    ankle = next(j for j in _urdf().findall('joint') if j.get('name') == output)
    worst_ratio = envelope['torque_peak_nm'] / float(ankle.find('limit').get('effort'))
    assert 1.0 < worst_ratio < 2.0
    controller = _controller()
    assert controller['default_kp'] * worst_ratio ** 2 <= envelope['stiffness_max_nm_rad']
    assert controller['default_kd'] * worst_ratio ** 2 <= envelope['damping_max_nm_s_rad']


def test_each_mode_selects_its_transport():
    for mode, plugin in (('sil', 'humanoid_transport_mujoco/MujocoActuatorTransport'),
                         ('hil', 'humanoid_transport_stm32/Stm32SerialTransport')):
        params = _yaml(PACKAGE, 'config', 'leg_prototype', f'{mode}.yaml')[
            'humanoidactuatorsystem']['ros__parameters']
        assert params['transport_plugin'] == plugin
        assert params['accepted_degradation'] == 'none'


def test_the_sliders_offer_only_the_commanded_joints():
    # The slider GUI publishes every movable joint of the description it is given, and the
    # controller reads references by index: a linkage's input or coupler there would shift them.
    spec = importlib.util.spec_from_file_location(
        'leg_launch', os.path.join(PACKAGE, 'launch', 'leg.launch.py'))
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    with open(os.path.join(GENERATED, 'robot.urdf')) as f:
        sliders = ET.fromstring(launch.commanded_only(f.read()))
    movable = [j.get('name') for j in sliders.findall('joint') if j.get('type') != 'fixed']
    assert movable == _controller()['joints']
