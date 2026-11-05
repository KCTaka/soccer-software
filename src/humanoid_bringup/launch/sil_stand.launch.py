import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml

def generate_launch_description():
    keyframes_path = os.path.join(
        get_package_share_directory('humanoid_bringup'), 'config', 'stand_keyframes.yaml')
    with open(keyframes_path) as f:
        keyframes = yaml.safe_load(f)

    launch_args = [
        DeclareLaunchArgument(
            'mjcf_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_bringup'),
                'model', 'robots', 'unitree_g1', 'generated', 'scene.xml']),
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
        # The controllers take seconds to start; without this the robot falls meanwhile.
        'hold_until_commanded': True,
        # Start in the trajectory's first pose, so its first reference matches the robot.
        'initial_pose.joints': keyframes['joint_names'],
        'initial_pose.positions': [float(p) for p in keyframes['keyframes'][0]['positions']],
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

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   'imu_sensor_broadcaster',
                   'humanoid_mit_controller'],
        output='screen',
    )
    # Started once every controller is active (the spawner exits then). The player's clock starts
    # with the node, so started any earlier it plays part of the trajectory to nobody and the
    # controller joins it midway, at a point that varies from run to run.
    trajectory_player = Node(
        package='humanoid_bringup',
        executable='trajectory_player.py',
        arguments=['--keyframes', keyframes_path, '--hold', '15.0'],
        output='screen',
    )

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
        controller_spawner,
        RegisterEventHandler(OnProcessExit(
            target_action=controller_spawner, on_exit=[trajectory_player])),
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
            # The default asset allow-list already serves the package:// meshes that robot.urdf
            # references.
            parameters=[{'capabilities': ['connectionGraph', 'assets']}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),
    ])
