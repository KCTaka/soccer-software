import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    workspace_root = os.environ.get('HUMANOID_WORKSPACE')
    if workspace_root:
        os.environ.setdefault(
            'HUMANOID_MJCF_PATH',
            os.path.join(workspace_root, 'model', 'generated', 'robot.mjcf'))

    robot_description_path = PathJoinSubstitution([
        FindPackageShare('humanoid_bringup'), 'config', 'robot_description.urdf'
    ])
    robot_description = ParameterValue(
        Command(['cat ', robot_description_path]), value_type=str)

    return LaunchDescription([
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
                PathJoinSubstitution([
                    FindPackageShare('humanoid_bringup'),
                    'config', 'hardware_sil.yaml'
                ]),
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       'humanoid_mit_controller'],
            output='screen',
        ),
        Node(
            package='humanoid_bringup',
            executable='trajectory_player.py',
            arguments=[
                '--keyframes', PathJoinSubstitution([
                    FindPackageShare('humanoid_bringup'),
                    'config', 'stand_keyframes.yaml']),
                '--hold', '15.0',
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('humanoid_bringup'), 'config', 'sil_stand.rviz'
            ])],
            output='screen',
        ),
    ])
