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
        # SIL-only ground-truth view. MujocoActuatorTransport publishes
        # sim_world -> ground_truth/pelvis; this second publisher hangs the
        # link tree off it under the "ground_truth/" prefix. The production
        # tree (odom -> pelvis -> links, owned by the Tier 0 estimator and
        # the publisher above) is never touched by the simulator, so SIL and
        # hardware resolve the same frames through the same producers.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='ground_truth',
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': 'ground_truth/',
            }],
            # /tf is absolute in tf2_ros and unaffected by the namespace;
            # joint_states is relative and must be pulled back to the root.
            remappings=[('joint_states', '/joint_states')],
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
