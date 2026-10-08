"""Read-only monitor of the bench_2s_10m dial board: telemetry from the real STM32 master into
/joint_states and RViz, with the motors left IDLE.

controller_manager runs HumanoidActuatorSystem + Stm32SerialTransport in monitor_only mode (every
cycle sends IDLE; nothing ever arms) and a joint_state_broadcaster. No MIT controller, so no command
interface is claimed. The transport's joints map is the GENERATED stm32_wiring.yaml; the bench
description and safety manifest are generated too. Turn a dial by hand and the matching joint moves
in RViz, with no motor resistance.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'serial_device', default_value='/host-dev/robosoccer-master',
            description="The master STM32's USB CDC device"),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2 (needs an X display on this host)'),
    ]
    share = FindPackageShare('humanoid_bringup')
    bench = PathJoinSubstitution([share, 'config', 'bench_2s_10m'])
    robot_description = ParameterValue(
        Command(['cat ', PathJoinSubstitution([share, 'config', 'bench_description.urdf'])]),
        value_type=str)

    return LaunchDescription(launch_args + [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                PathJoinSubstitution([bench, 'monitor.yaml']),
                # The generated joints map (chain/motor/direction_sign/zero_offset_rad); merges into
                # the stm32_serial node alongside monitor.yaml's monitor_only.
                PathJoinSubstitution([bench, 'stm32_wiring.yaml']),
                # Consumed by the stm32_serial node the transport creates inside this process.
                {'serial_device': LaunchConfiguration('serial_device')},
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],  # no MIT controller: monitor is read-only
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([share, 'config', 'bench_dial.rviz'])],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
