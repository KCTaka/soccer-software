"""Leg prototype bringup: the shin clamped to a stand, the ankle driven through its four-bar.

Arguments:
  mode    sil (default): MuJoCo software-in-the-loop. MujocoActuatorTransport simulates the
          generated scene, the closed loop included; nothing touches hardware.
          hil: the real leg. Stm32SerialTransport talks to the master STM32 and the RS02 behind it.
          The motor is armed and will move. Needs the calibrated zero offset: the generated
          stm32_wiring.yaml leaves it out until robot_model.yaml has one, and the transport then
          refuses to configure (docs/leg-prototype.md).
  rviz    true (default): RViz2. Needs an X display.
  foxglove  false (default): foxglove_bridge on ws://<host>:8765, for a host without a display.
  serial_device  hil only. The master's USB CDC device, /host-dev/robosoccer-master in the dev
          container; on a bare host pass /dev/robosoccer-master.

Both modes run the same stack: controller_manager with HumanoidActuatorSystem, which maps the
ankle's MIT tuple through the linkage to the motor, joint_state_broadcaster, and
humanoid_mit_controller, which holds the measured pose until a reference arrives on
/humanoid_mit_controller/joint_references. Drive it with bench_trajectories, e.g.
  ros2 run humanoid_bringup bench_trajectories --ros-args \\
    -p joints:=[left_ankle_pitch_joint] -p amplitude_rad:=0.2 -p frequency_hz:=0.25
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MODES = ('sil', 'hil')


def _truthy(value):
    return str(value).lower() in ('true', '1', 'yes')


def _setup(context, *_args, **_kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODES:
        raise RuntimeError(f"mode must be one of {MODES}, got {mode!r}")
    rviz = _truthy(LaunchConfiguration('rviz').perform(context))
    foxglove = _truthy(LaunchConfiguration('foxglove').perform(context))
    serial_device = LaunchConfiguration('serial_device').perform(context)

    share = FindPackageShare('humanoid_bringup').perform(context)
    config = os.path.join(share, 'config', 'leg_prototype')
    with open(os.path.join(share, 'config', 'leg_prototype_description.urdf')) as f:
        robot_description = {'robot_description': f.read()}

    parameters = [robot_description, os.path.join(config, 'controllers.yaml'),
                  os.path.join(config, f'{mode}.yaml')]
    if mode == 'sil':
        parameters.append({
            'mjcf_path': os.path.join(
                share, 'model', 'robots', 'leg_prototype', 'generated', 'scene.xml'),
            # The controllers take seconds to start; held until then, the foot does not drop.
            'hold_until_commanded': True,
        })
    else:
        parameters += [os.path.join(config, 'stm32_wiring.yaml'),
                       {'serial_device': serial_device}]

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
    if foxglove:
        # Viewer-only: no client publishing, services or parameter access.
        nodes.append(Node(package='foxglove_bridge', executable='foxglove_bridge',
                          parameters=[{'capabilities': ['connectionGraph', 'assets']}],
                          output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sil',
                              description='sil: MuJoCo, no hardware. hil: the real leg, armed'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument('foxglove', default_value='false',
                              description='Start foxglove_bridge on ws://<host>:8765'),
        DeclareLaunchArgument(
            'serial_device', default_value='/host-dev/robosoccer-master',
            description="The master STM32's USB CDC device (mode:=hil only)"),
        OpaqueFunction(function=_setup),
    ])
