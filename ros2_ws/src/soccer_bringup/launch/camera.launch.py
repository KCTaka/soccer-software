"""Produce the repo's generic camera contract from a real ZED camera.

This is the real-hardware counterpart to the sim-only ``sim_camera_node``. Rather
than run a Python relay, it loads the Stereolabs ``ZedCamera`` **component**
directly and **remaps** its native topics onto the driver-agnostic contract the
perception / localization stack consumes (``docs/zed_jetson_integration.md``):

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
topics *itself* — no per-frame relay process, no per-frame (de)serialization, and
one fewer inter-process hop for the full-res image + depth streams. The component
is loaded with ``use_intra_process_comms`` so a future C++ perception component
co-loaded into this container gets **zero-copy** frames (inter-process subscribers
— the app container — still receive normal DDS). QoS is best-effort SensorData (set
in the ZED params override), matching what the consumers now subscribe with.

Alongside the component this launches a ``robot_state_publisher`` for the ZED's own
URDF. That is NOT optional: the ZED component blocks its grab loop until the camera
static TF chain exists, so without it the node starts, advertises every topic, and
then publishes nothing at all.

The node sits in the ``robot_name`` namespace, so the RELATIVE remap targets
resolve to ``/<robot_name>/camera/…`` — exactly what the graph subscribes to.

Runs in the ``zed-driver-image`` (the only image carrying the ZED SDK + wrapper /
``zed_components``, and the only container with a GPU). It also co-loads the
``soccer_perception_gpu`` components into the same container so they get frames by
pointer; if that package is absent the launch degrades to a plain camera driver
(``perception:=false`` forces that too).
"""
import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# ZED SDK 5.x native topics (verified live — docs/zed_jetson_integration.md §2)
# → generic contract. Targets are RELATIVE, so they inherit the node's
# ``robot_name`` namespace (→ /<robot_name>/camera/image_raw, …).
_REMAPPINGS = [
    ("~/rgb/color/rect/image", "camera/image_raw"),
    # camera_info rides ALONGSIDE the image, not at the namespace root. The ZED
    # wrapper publishes it as the image topic's companion, so remapping the image
    # to camera/image_raw already puts it on camera/camera_info; this entry only
    # covers the wrapper's own explicit publisher and must point at the SAME
    # place, or consumers end up subscribed to a topic nobody publishes.
    # That exact mismatch ("camera_info" vs "camera/camera_info") left
    # projection_node running on placeholder intrinsics for the whole bring-up.
    # See docs/architecture/perception_gpu_migration.md §2.
    ("~/rgb/color/rect/camera_info", "camera/camera_info"),
    ("~/depth/depth_registered", "camera/depth"),
    ("~/imu/data", "imu/data"),
]


def _perception_nodes(context, robot_name: str) -> list:
    """The GPU perception components, co-loaded for zero-copy frames.

    ``detector_node`` (RF-DETR / TensorRT) and ``fieldline_node`` live in the ZED
    container rather than the app container for one reason: composed into the same
    process with ``use_intra_process_comms``, they receive ``image_raw`` as a
    shared_ptr instead of a 3.69 MB DDS message. At 18 Hz that is ~66 MB/s of
    serialize → transport → deserialize work deleted twice over (once per
    subscriber). See docs/architecture/perception_gpu_migration.md §3.

    Returns [] when ``soccer_perception_gpu`` is not in the image, so this launch
    file still works standalone as a plain camera driver.
    """
    if LaunchConfiguration("perception").perform(context).lower() in ("false", "0", "no"):
        return []
    try:
        params = os.path.join(
            get_package_share_directory("soccer_perception_gpu"),
            "config", "perception_gpu.yaml",
        )
    except PackageNotFoundError:
        print("[camera.launch.py] soccer_perception_gpu not installed - "
              "running camera-only (no GPU perception).")
        return []

    common = dict(
        package="soccer_perception_gpu",
        namespace=robot_name,
        parameters=[params],
        # The whole point: shared_ptr hand-off from the ZED component above.
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return [
        ComposableNode(
            plugin="soccer_perception_gpu::DetectorComponent",
            name="detector_node", **common),
        ComposableNode(
            plugin="soccer_perception_gpu::FieldlineComponent",
            name="fieldline_node", **common),
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
        composable_node_descriptions=[zed_node] + _perception_nodes(context, robot_name),
        arguments=["--use_multi_threaded_executor"],
        output="screen",
    )

    # The ZED component blocks in "Waiting for valid static transformations..."
    # until the camera's own TF chain (<camera_name>_camera_link -> ..._left_camera
    # _frame) exists, so the grab loop never starts and NO images are published
    # without this. The stock zed_camera.launch.py ships the same node; it must be
    # carried over here. `robot_description` is remapped to <camera_name>_description
    # so it cannot collide with the robot's own description from robot.launch.py.
    zed_descr = os.path.join(
        get_package_share_directory("zed_description"), "urdf", "zed_descr.urdf.xacro"
    )
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="zed_state_publisher",
        namespace=robot_name,
        output="screen",
        parameters=[{
            "robot_description": Command(
                ["xacro ", zed_descr,
                 " camera_name:=", camera_name,
                 " camera_model:=", camera_model]
            ),
        }],
        remappings=[("robot_description", f"{camera_name}_description")],
    )

    return [rsp_node, container]


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
        DeclareLaunchArgument(
            "perception", default_value="true",
            description="Co-load the GPU perception components (detector_node, "
                        "fieldline_node) into this container for zero-copy frames. "
                        "Set false to run a bare camera driver."),
        OpaqueFunction(function=_launch_setup),
    ])
