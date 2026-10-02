from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'mjcf_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_bringup'),
                'model', 'generated', 'robot.mjcf']),
            description='MJCF model loaded by MujocoActuatorTransport'),
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
            description='Start foxglove_bridge on ws://<host>:8765 for a remote Foxglove viewer; '
                        'needs no display or GPU on this host'),
    ]
    # Consumed by the mujoco_sim node that the transport creates inside
    # ros2_control_node. value_type=float keeps "50" from arriving as an int.
    sim_params = {
        'mjcf_path': LaunchConfiguration('mjcf_path'),
        'disturbance.push.force_n': ParameterValue(
            LaunchConfiguration('push_force_n'), value_type=float),
        'disturbance.push.start_time_s': ParameterValue(
            LaunchConfiguration('push_start_time_s'), value_type=float),
    }

    robot_description_path = PathJoinSubstitution([
        FindPackageShare('humanoid_bringup'), 'config', 'robot_description.urdf'
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
                sim_params,
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
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        # Viewer-only: no client publishing, services or parameter access, so anything that can
        # reach the port can watch the robot but not command it.
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=[{
                'capabilities': ['connectionGraph', 'assets'],
                # robot.urdf references its meshes as file:// URIs under model/source (the
                # bridge's default only serves package://). No dots in the directory part, so
                # ".." cannot escape.
                'asset_uri_allowlist': [
                    r'^file:///(?:[-\w]+/)*model/source/(?:[-\w]+/)*[-\w]+\.stl$',
                ],
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),
    ])
