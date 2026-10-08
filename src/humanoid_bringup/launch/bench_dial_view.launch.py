"""View the bench_2s_10m dial board in RViz and drive its joints by hand with
joint_state_publisher_gui. Model-only: no transport, no controllers, no hardware. Use it to
eyeball the generated URDF (link frames, joint axes, the dial-away zero pose)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2 (needs an X display on this host)'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start joint_state_publisher_gui sliders; false publishes a zero pose'),
    ]

    robot_description_path = PathJoinSubstitution([
        FindPackageShare('humanoid_bringup'), 'config', 'bench_description.urdf'
    ])
    robot_description = ParameterValue(
        Command(['cat ', robot_description_path]), value_type=str)

    return LaunchDescription(launch_args + [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        # Sliders publish /joint_states; robot_state_publisher turns them into TF.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{'robot_description': robot_description}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
        # Without the GUI, publish a constant zero pose so RViz still shows the board.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('gui')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('humanoid_bringup'), 'config', 'bench_dial.rviz'
            ])],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
