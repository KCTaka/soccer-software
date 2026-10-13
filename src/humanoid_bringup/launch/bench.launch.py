"""Bench bringup for a dial board — one launch for both benches, two modes.

Arguments:
  robot   bench_1s_5m (default, 5 motors / 1 slave) or bench_2s_10m (10 motors / 2 slaves).
  sim     true (default): NOT connected to a bench. Model-only slider view --
          joint_state_publisher_gui drives /joint_states into RViz so you can pose the board by
          hand. No controller_manager, no transport, no hardware.
          false: the real STM32 bench. controller_manager runs HumanoidActuatorSystem +
          Stm32SerialTransport with mit_controller active; the motors are armed and the controller
          subscribes to /humanoid_mit_controller/joint_references. Drive it with bench_trajectories.

Everything is per-robot and generated: config/<robot>_description.urdf, config/<robot>/controllers.yaml
and config/<robot>/stm32_wiring.yaml. serial_device defaults to /host-dev/robosoccer-master inside
the devcontainer (on a bare host pass serial_device:=/dev/robosoccer-master); it is used only when
sim:=false. gui:=false publishes a zero pose instead of the slider GUI (sim only).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ROBOTS = ('bench_1s_5m', 'bench_2s_10m')


def _truthy(value):
    return str(value).lower() in ('true', '1', 'yes')


def _setup(context, *_args, **_kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ROBOTS:
        raise RuntimeError(f"robot must be one of {ROBOTS}, got {robot!r}")
    sim = _truthy(LaunchConfiguration('sim').perform(context))
    gui = _truthy(LaunchConfiguration('gui').perform(context))
    rviz = _truthy(LaunchConfiguration('rviz').perform(context))
    serial_device = LaunchConfiguration('serial_device').perform(context)

    share = FindPackageShare('humanoid_bringup').perform(context)
    urdf_path = os.path.join(share, 'config', f'{robot}_description.urdf')
    rviz_cfg = os.path.join(share, 'config', 'bench_dial.rviz')
    with open(urdf_path) as f:
        robot_description = {'robot_description': f.read()}

    nodes = [Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description], output='screen')]

    if sim:
        # Slider view: sliders (or a zero pose) -> /joint_states -> RViz. No hardware.
        jsp = 'joint_state_publisher_gui' if gui else 'joint_state_publisher'
        nodes.append(Node(package=jsp, executable=jsp,
                          parameters=[robot_description], output='screen'))
    else:
        # Real bench: controller_manager + transport + mit_controller (armed).
        controllers = os.path.join(share, 'config', robot, 'controllers.yaml')
        wiring = os.path.join(share, 'config', robot, 'stm32_wiring.yaml')
        nodes += [
            Node(
                package='controller_manager', executable='ros2_control_node',
                parameters=[robot_description, controllers, wiring,
                            {'serial_device': serial_device}],
                output='screen'),
            Node(
                package='controller_manager', executable='spawner',
                arguments=['joint_state_broadcaster'], output='screen'),
            Node(
                package='controller_manager', executable='spawner',
                arguments=['humanoid_mit_controller'], output='screen'),
        ]

    if rviz:
        nodes.append(Node(
            package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='bench_1s_5m',
                              description='bench_1s_5m | bench_2s_10m'),
        DeclareLaunchArgument('sim', default_value='true',
                              description='true: slider view, no hardware. false: real armed bench'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='sim only: slider GUI; false publishes a zero pose'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument(
            'serial_device', default_value='/host-dev/robosoccer-master',
            description="The master STM32's USB CDC device (sim:=false only)"),
        OpaqueFunction(function=_setup),
    ])
