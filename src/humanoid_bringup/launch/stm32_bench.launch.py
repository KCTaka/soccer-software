"""The full pipeline against free drives on the STM32 master.

A non-real-time reference node, MitImpedanceController, HumanoidActuatorSystem with its
SafetyKernel, Stm32SerialTransport, the master, the slaves, and the drives. See
docs/transport-stm32.md, "Bench".

The drives move. Run it only with nothing attached to them.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'serial_device', default_value='/dev/ttyACM0',
            description="The master STM32's USB CDC device"),
        DeclareLaunchArgument(
            'player_delay_s', default_value='4.0',
            description='Seconds before the reference node starts, so the controller is active'),
    ]
    config = PathJoinSubstitution([FindPackageShare('humanoid_bringup'), 'config', 'bench'])
    robot_description = ParameterValue(
        Command(['cat ', PathJoinSubstitution([config, 'stm32_bench.urdf'])]), value_type=str)
    parameters = PathJoinSubstitution([config, 'stm32_bench.yaml'])

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
                parameters,
                # Consumed by the stm32_serial node the transport creates inside this process.
                {'serial_device': LaunchConfiguration('serial_device')},
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', 'humanoid_mit_controller'],
            output='screen',
        ),
        TimerAction(
            period=LaunchConfiguration('player_delay_s'),
            actions=[Node(
                package='humanoid_bringup',
                executable='bench_player.py',
                parameters=[parameters],
                output='screen',
            )],
        ),
    ])
