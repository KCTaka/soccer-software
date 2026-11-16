"""Each robot's hand-written configs against its generated description and safety manifest.

Mistakes here only show when a robot is launched: a controller joint the description does not
command, or linkage gains that the hardware component turns into motor gains the drive silently
clamps. Each check runs for every robot with a config/<robot>/ directory.
"""
import os
import xml.etree.ElementTree as ET

import pytest

from bringup_files import (CONFIG, commanded_joints, configured_robots, controller, description_dir,
                           load_launch, load_yaml, transmissions, urdf, urdf_text)

ROBOTS = configured_robots()
LINKAGE_ROBOTS = [robot for robot in ROBOTS if transmissions(urdf(robot))]
TRANSPORTS = {'sil': 'humanoid_transport_mujoco/MujocoActuatorTransport',
              'hil': 'humanoid_transport_stm32/Stm32SerialTransport'}
MODES = [(robot, mode) for robot in ROBOTS for mode in TRANSPORTS
         if os.path.isfile(os.path.join(CONFIG, robot, f'{mode}.yaml'))]


def test_every_configured_robot_has_a_generated_description():
    for robot in ROBOTS:
        assert os.path.isdir(description_dir(robot)), f'{robot} has no generated description'


@pytest.mark.parametrize('robot', ROBOTS)
def test_the_controller_commands_the_descriptions_command_joints(robot):
    # The controller reads references by index, so its order must be the description's too.
    assert controller(robot)['joints'] == commanded_joints(urdf(robot))


@pytest.mark.parametrize('robot', ROBOTS)
def test_the_sliders_offer_only_the_commanded_joints(robot):
    # The slider GUI publishes every movable joint of the description it is given, and the
    # controller reads references by index: a linkage's input or coupler there would shift them.
    sliders = ET.fromstring(load_launch('leg').commanded_only(urdf_text(robot)))
    movable = [j.get('name') for j in sliders.findall('joint') if j.get('type') != 'fixed']
    assert movable == commanded_joints(urdf(robot))


@pytest.mark.parametrize('robot', LINKAGE_ROBOTS)
def test_linkage_gains_stay_inside_the_motors_caps(robot):
    # The hardware component multiplies the output joint's gains by ratio^2. The worst ratio is
    # the manifest's motor torque bound over the description's output torque bound, both
    # generated.
    root = urdf(robot)
    envelopes = load_yaml(description_dir(robot), 'config', 'safety_manifest.yaml')['envelopes']
    gains = controller(robot)
    for transmission in transmissions(root):
        envelope = envelopes[transmission.find('actuator').get('name')]
        output = next(j.get('name') for j in transmission.findall('joint')
                      if j.get('role') == 'output')
        joint = next(j for j in root.findall('joint') if j.get('name') == output)
        worst_ratio = envelope['torque_peak_nm'] / float(joint.find('limit').get('effort'))
        assert worst_ratio > 0.0, transmission.get('name')
        assert gains['default_kp'] * worst_ratio ** 2 <= envelope['stiffness_max_nm_rad']
        assert gains['default_kd'] * worst_ratio ** 2 <= envelope['damping_max_nm_s_rad']


@pytest.mark.parametrize(('robot', 'mode'), MODES)
def test_each_mode_selects_its_transport(robot, mode):
    params = load_yaml(CONFIG, robot, f'{mode}.yaml')['humanoidactuatorsystem']['ros__parameters']
    assert params['transport_plugin'] == TRANSPORTS[mode]
