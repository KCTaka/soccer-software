"""Produce the repo's generic camera contract from a real ZED camera.

This is the real-hardware counterpart to the sim-only ``sim_camera_node``. Rather
than run a Python relay, it loads the Stereolabs ``ZedCamera`` **component**
directly and **remaps** its native topics onto the driver-agnostic contract the
perception / localization stack consumes (``docs/jetson_zed_workflow.md`` §3, §6):

===========================================  ==================
ZED SDK 5.x topic (private ``~/…``)          contract topic
===========================================  ==================
``~/rgb/color/rect/image``                   ``camera/image_raw``
``~/rgb/color/rect/camera_info``             ``camera_info``
``~/depth/depth_registered``                 ``camera/depth``
``~/imu/data``                               ``imu/data``
===========================================  ==================

Why a component + remaps and NOT a bridge node (``docs/zed_jetson_integration.md``
§5): a ``launch_ros`` ``ComposableNode`` accepts ``remappings`` directly (they are
sent to the container as ``remap_rules``), so the camera publishes the contract
topics *itself* — no extra process, no per-frame (de)serialization, and one fewer
inter-process hop for the full-res image + depth streams. The component is loaded
with ``use_intra_process_comms`` so a future C++ perception component co-loaded
into this container gets **zero-copy** frames (inter-process subscribers — the app
container — still receive normal DDS). QoS is best-effort SensorData (set in the
ZED params override), matching what the consumers now subscribe with.

The node sits in the ``robot_name`` namespace, so the RELATIVE remap targets
resolve to ``/<robot_name>/camera/…`` — exactly what the graph subscribes to.

Runs in the ``zed-driver-image`` (the only image carrying the ZED SDK + wrapper /
``zed_components``). It only references ``zed_wrapper`` + ``zed_components``, so it
runs there by file path without a soccer_ws build.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# ZED SDK 5.x native topics (verified live — docs/zed_jetson_integration.md §2)
# → generic contract. Targets are RELATIVE, so they inherit the node's
# ``robot_name`` namespace (→ /<robot_name>/camera/image_raw, …).
_REMAPPINGS = [
    ("~/rgb/color/rect/image", "camera/image_raw"),
    ("~/rgb/color/rect/camera_info", "camera_info"),
    ("~/depth/depth_registered", "camera/depth"),
    ("~/imu/data", "imu/data"),
]


def _launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    camera_model = LaunchConfiguration("camera_model").perform(context)
    camera_name = LaunchConfiguration("camera_name").perform(context)
    container_name = LaunchConfiguration("container_name").perform(context)
    override = LaunchConfiguration("ros_params_override_path").perform(context)

    # Load the wrapper's own tuned defaults (same files its stock launch uses),
    # then the optional site override (grab_resolution + QoS), then the two
    # launch-arg params the wrapper expects from the launcher.
    zed_share = get_package_share_directory("zed_wrapper")
    params: list = [
        os.path.join(zed_share, "config", "common_stereo.yaml"),
        os.path.join(zed_share, "config", f"{camera_model}.yaml"),
    ]
    if override:
        params.append(override)
    params.append({
        "general.camera_name": camera_name,
        "general.camera_model": camera_model,
    })

    zed_node = ComposableNode(
        package="zed_components",
        plugin="stereolabs::ZedCamera",
        name="zed_node",
        namespace=robot_name,
        parameters=params,
        remappings=_REMAPPINGS,
        # Zero-copy for any C++ component later co-loaded into this container;
        # the app container's (inter-process) subscribers still use normal DDS.
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name=container_name,
        namespace=robot_name,
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=[zed_node],
        arguments=["--use_multi_threaded_executor"],
        output="screen",
    )
    return [container]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name", default_value="robot_1",
            description="Namespace for the ZED node; the remapped contract topics "
                        "land on /<robot_name>/camera/... (must match robot.launch.py)."),
        DeclareLaunchArgument(
            "camera_model", default_value="zedm",
            description="ZED model (zedm = ZED Mini); selects the <model>.yaml config."),
        DeclareLaunchArgument(
            "camera_name", default_value="zed",
            description="ZED camera name (TF frame prefix); does not affect topic names."),
        DeclareLaunchArgument(
            "container_name", default_value="zed_container",
            description="Component container name; co-load a C++ perception "
                        "component here for zero-copy image delivery."),
        DeclareLaunchArgument(
            "ros_params_override_path", default_value="",
            description="Optional extra ZED params YAML (wins over the wrapper "
                        "defaults); e.g. grab_resolution + best-effort qos_overrides."),
        OpaqueFunction(function=_launch_setup),
    ])
