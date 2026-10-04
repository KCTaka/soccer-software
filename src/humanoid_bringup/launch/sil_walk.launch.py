from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from typing import List


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'mjcf_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_bringup'),
                'model', 'generated', 'robot.mjcf']),
            description='MJCF model loaded by MujocoActuatorTransport'),
        DeclareLaunchArgument(
            'command', default_value='[0.0, 0.0, 0.0]',
            description='Velocity command [vx m/s, vy m/s, yaw rate rad/s] used when no cmd_vel '
                        'has been received recently; each is limited to the trained range'),
        DeclareLaunchArgument(
            'push_force_n', default_value='0.0',
            description='SIL push force in N; 0 disables the push'),
        DeclareLaunchArgument(
            'push_start_time_s', default_value='0.0',
            description='Simulation time at which the SIL push starts, s'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2 (needs an X display on this host)'),
        DeclareLaunchArgument(
            'foxglove', default_value='false',
            description='Start foxglove_bridge on ws://<host>:8765 for a remote Foxglove viewer'),
    ]
    sim_params = {
        'mjcf_path': LaunchConfiguration('mjcf_path'),
        # The controllers take seconds to start; without this the robot falls meanwhile.
        'hold_until_commanded': True,
        'disturbance.push.force_n': ParameterValue(
            LaunchConfiguration('push_force_n'), value_type=float),
        'disturbance.push.start_time_s': ParameterValue(
            LaunchConfiguration('push_start_time_s'), value_type=float),
    }

    share = FindPackageShare('humanoid_bringup')
    policy_dir = PathJoinSubstitution([share, 'policies', 'unitree_g1_walk'])
    # One file serves both nodes: walk_policy's section, and the controller's gains. The gains
    # belong to the policy, which was trained against exactly these torques.
    policy_params = PathJoinSubstitution([policy_dir, 'policy.yaml'])

    robot_description_path = PathJoinSubstitution([share, 'config', 'robot_description.urdf'])
    robot_description = ParameterValue(
        Command(['cat ', robot_description_path]), value_type=str)

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   'imu_sensor_broadcaster',
                   'humanoid_mit_controller'],
        output='screen',
    )
    # Started once every controller is active (the spawner exits then): the policy's first stance
    # reference is what releases the held simulation, so it must not begin earlier.
    walk_policy = Node(
        package='humanoid_bringup',
        executable='walk_policy.py',
        parameters=[
            policy_params,
            {
                'policy_path': PathJoinSubstitution([policy_dir, 'policy.onnx']),
                'joint_limits_file': PathJoinSubstitution(
                    [share, 'config', 'safety_manifest.yaml']),
                'fallback_command': ParameterValue(
                    LaunchConfiguration('command'), value_type=List[float]),
            },
        ],
        output='screen',
    )

    return LaunchDescription(launch_args + [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='ground_truth',
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': 'ground_truth/',
            }],
            remappings=[('joint_states', '/joint_states')],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                PathJoinSubstitution([share, 'config', 'hardware_sil.yaml']),
                policy_params,
                sim_params,
            ],
            output='screen',
        ),
        controller_spawner,
        RegisterEventHandler(OnProcessExit(
            target_action=controller_spawner, on_exit=[walk_policy])),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([share, 'config', 'sil_stand.rviz'])],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=[{'capabilities': ['connectionGraph', 'assets']}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),
    ])
