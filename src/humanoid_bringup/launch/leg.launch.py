"""Leg prototype bringup: the shin clamped to a stand, the ankle driven through its four-bar.

Arguments:
  mode    sil (default): MuJoCo software-in-the-loop. MujocoActuatorTransport simulates the
          generated scene, the closed loop included; nothing touches hardware.
          monitor: the real leg, read-only. The same stack as hil, but Stm32SerialTransport never
          arms: every cycle asks the RS02 for IDLE, and /joint_states shows the ankle through the
          linkage. Use it to check the zero offset and direction sign before arming.
          hil: the real leg. Stm32SerialTransport talks to the master STM32 and the RS02 behind it.
          The motor is armed and will move.
          monitor and hil need the calibrated zero offset: the generated stm32_wiring.yaml leaves it
          out until robot_model.yaml has one, and the transport then refuses to configure
          (docs/leg-prototype.md).
  rviz    true (default): RViz2. Needs an X display.
  sliders auto (default): true in sil, false otherwise. true opens a slider window next to RViz
          (joint_state_publisher_gui) that commands the ankle: it publishes each commanded joint's
          slider value, within its URDF limits, on /humanoid_mit_controller/joint_references.
          The sliders start at 0 and each drag is a step the controller does not interpolate, so on
          the real leg (sliders:=true with mode:=hil) start with the foot near 0. RViz shows the
          linkage from the measured, or simulated, states. Do not run bench_trajectories with it.
  foxglove  false (default): foxglove_bridge on ws://<host>:8765, for a host without a display.
  serial_device  monitor and hil. The master's USB CDC device, /host-dev/robosoccer-master in the
          dev container; on a bare host pass /dev/robosoccer-master.

Every mode runs the same stack: controller_manager with HumanoidActuatorSystem, which maps the
ankle's MIT tuple through the linkage to the motor, joint_state_broadcaster, and
humanoid_mit_controller, which holds the measured pose until a reference arrives on
/humanoid_mit_controller/joint_references. Drive it with bench_trajectories, e.g.
  ros2 run humanoid_bringup bench_trajectories --ros-args \\
    -p joints:=[left_ankle_pitch_joint] -p amplitude_rad:=0.2 -p frequency_hz:=0.25
"""
import os
import tempfile
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MODES = ('sil', 'monitor', 'hil')


def _truthy(value):
    return str(value).lower() in ('true', '1', 'yes')


def commanded_only(urdf):
    """`urdf` with every joint its <ros2_control> block does not command made fixed. Given it, the
    slider GUI offers and publishes exactly the joints the controller reads, in their order, and
    never a linkage's input or coupler, which the hardware component derives."""
    robot = ET.fromstring(urdf)
    commanded = {j.get('name') for c in robot.findall('ros2_control') for j in c.findall('joint')
                 if j.find('command_interface') is not None}
    for joint in robot.findall('joint'):
        if joint.get('name') not in commanded:
            joint.set('type', 'fixed')
    return ET.tostring(robot, encoding='unicode')


def _setup(context, *_args, **_kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODES:
        raise RuntimeError(f"mode must be one of {MODES}, got {mode!r}")
    rviz = _truthy(LaunchConfiguration('rviz').perform(context))
    sliders = LaunchConfiguration('sliders').perform(context)
    sliders = mode == 'sil' if sliders == 'auto' else _truthy(sliders)
    foxglove = _truthy(LaunchConfiguration('foxglove').perform(context))
    serial_device = LaunchConfiguration('serial_device').perform(context)

    share = FindPackageShare('humanoid_bringup').perform(context)
    config = os.path.join(share, 'config', 'leg_prototype')
    with open(os.path.join(share, 'config', 'leg_prototype_description.urdf')) as f:
        robot_description = {'robot_description': f.read()}

    parameters = [robot_description, os.path.join(config, 'controllers.yaml'),
                  os.path.join(config, 'sil.yaml' if mode == 'sil' else 'hil.yaml')]
    if mode == 'sil':
        parameters.append({
            'mjcf_path': os.path.join(
                share, 'model', 'robots', 'leg_prototype', 'generated', 'scene.xml'),
            # The controllers take seconds to start; held until then, the foot does not drop.
            'hold_until_commanded': True,
        })
    else:
        parameters += [os.path.join(config, 'stm32_wiring.yaml'),
                       {'serial_device': serial_device, 'monitor_only': mode == 'monitor'}]

    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=parameters, output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'humanoid_mit_controller'], output='screen'),
    ]
    if rviz:
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', os.path.join(config, 'leg.rviz')], output='screen'))
    if sliders:
        # The GUI reads its description from a file, and in its own namespace no one publishes on
        # its robot_description topic, so the full description never replaces the filtered one.
        fd, slider_urdf = tempfile.mkstemp(prefix='leg_sliders_', suffix='.urdf')
        with os.fdopen(fd, 'w') as f:
            f.write(commanded_only(robot_description['robot_description']))
        nodes.append(Node(
            package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
            namespace='ankle_slider', arguments=[slider_urdf], parameters=[{'rate': 50}],
            remappings=[('joint_states', '/humanoid_mit_controller/joint_references')],
            output='screen'))
    if foxglove:
        # Viewer-only: no client publishing, services or parameter access.
        nodes.append(Node(package='foxglove_bridge', executable='foxglove_bridge',
                          parameters=[{'capabilities': ['connectionGraph', 'assets']}],
                          output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sil',
            description='sil: MuJoCo, no hardware. monitor: the real leg, never armed. '
                        'hil: the real leg, armed'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument(
            'sliders', default_value='auto',
            description='Slider window commanding the ankle. auto: on in sil, off otherwise'),
        DeclareLaunchArgument('foxglove', default_value='false',
                              description='Start foxglove_bridge on ws://<host>:8765'),
        DeclareLaunchArgument(
            'serial_device', default_value='/host-dev/robosoccer-master',
            description="The master STM32's USB CDC device (mode:=monitor or hil)"),
        OpaqueFunction(function=_setup),
    ])
