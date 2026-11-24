"""robot_model.yaml -> robot.urdf, with its <ros2_control> block for HumanoidActuatorSystem."""
import os
import xml.etree.ElementTree as ET

from . import linkage
from .common import collision_meshes, fmt, mesh_file, quat_to_rpy

DESCRIPTION_PACKAGE = "humanoid_description"
FOUR_BAR_PLUGIN = "humanoid_actuator_system/FourBarLinkage"
CMD_INTERFACES = ("position", "velocity", "effort", "stiffness", "damping")
STATE_INTERFACES = ("position", "velocity", "effort")
# The interface names imu_sensor_broadcaster reads; kImuStateFields in batch_types.hpp spells the
# same list, and the hardware contract rejects a description that differs.
IMU_INTERFACES = (
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
)

def emit_urdf(model, out_path, meshes_dir, meshes_uri, safety_manifest_path):
    """Write robot.urdf, its meshes under `meshes_uri` and its manifest at `safety_manifest_path` in
    the description package. Returns the mesh files it names."""
    joints = model["joints"]
    drives = linkage.drives(model)
    drive_by_input = {d["actuated_joint"]: d for d in drives if d["linkage"] is not None}
    coupler_by_joint = {d["coupler"]["joint"]: d["coupler"] for d in drives
                        if d["coupler"] is not None}
    robot = ET.Element("robot", name=model["robot"]["name"])

    # Index actual files on disk (case-insensitive mapping). A primitive-only robot has no assets.
    disk_assets = os.listdir(meshes_dir) if meshes_dir and os.path.exists(meshes_dir) else []
    # package:// rather than file://: the URI must not depend on where this ran (/ws, a laptop),
    # and Foxglove only fetches package:// meshes over its bridge.
    mesh_files = set()

    for l in model["links"]:
        le = ET.SubElement(robot, "link", name=l["name"])
        ie = ET.SubElement(le, "inertial")
        inertia, com = l["inertia"], l["com_m"]
        ie.set("xyz", fmt(com))
        ie.set("rpy", fmt([0, 0, 0]))  # the tensor's axes are the link frame's
        ET.SubElement(ie, "mass", value=f"{l['mass_kg']:.10g}")
        ET.SubElement(
            ie,
            "inertia",
            ixx=f"{inertia['ixx_kg_m2']:.10g}",
            iyy=f"{inertia['iyy_kg_m2']:.10g}",
            izz=f"{inertia['izz_kg_m2']:.10g}",
            ixy=f"{inertia['ixy_kg_m2']:.10g}",
            ixz=f"{inertia['ixz_kg_m2']:.10g}",
            iyz=f"{inertia['iyz_kg_m2']:.10g}",
        )

        for tag, mesh in ([("visual", l.get("visual_mesh"))] +
                          [("collision", mesh) for mesh in collision_meshes(l)]):
            if mesh:
                ve = ET.SubElement(le, tag)
                ET.SubElement(ve, "origin", xyz="0 0 0", rpy="0 0 0")
                f = mesh_file(mesh, disk_assets)
                mesh_files.add(f)
                ET.SubElement(ve, "geometry").append(ET.Element(
                    "mesh", filename=f"{meshes_uri}/{f}"))

        def add_primitive(tag, prim, le=le):
            ve = ET.SubElement(le, tag)
            ET.SubElement(
                ve,
                "origin",
                xyz=fmt(prim["frame"]["xyz_m"]),
                rpy=fmt(prim["frame"]["rpy_rad"]),
            )
            geometry = ET.SubElement(ve, "geometry")
            shape = prim["shape"]
            dimensions = prim["dimensions_m"]
            if shape == "box":
                ET.SubElement(
                    geometry, "box", size=fmt([2.0 * x for x in dimensions])
                )
            elif shape == "sphere":
                ET.SubElement(
                    geometry, "sphere", radius=f"{dimensions[0]:.10g}"
                )
            elif shape == "cylinder":
                ET.SubElement(
                    geometry,
                    "cylinder",
                    radius=f"{dimensions[0]:.10g}",
                    length=f"{2.0 * dimensions[1]:.10g}",
                )
            elif shape == "capsule":
                ET.SubElement(
                    geometry,
                    "capsule",
                    radius=f"{dimensions[0]:.10g}",
                    length=f"{2.0 * dimensions[1]:.10g}",
                )

        # A mesh-less link (e.g. a primitive-only bench fixture) would otherwise have no visual and
        # be invisible in RViz, so mirror each collision primitive into a visual. Links that have a
        # mesh keep the mesh as their only visual.
        emit_visual = l.get("visual_mesh") is None
        for prim in l["collision"]["primitives"]:
            if emit_visual:
                add_primitive("visual", prim)
            add_primitive("collision", prim)

    for j in joints:
        je = ET.SubElement(
            robot,
            "joint",
            name=j["name"],
            type={
                "revolute": "revolute",
                "prismatic": "prismatic",
                "fixed": "fixed",
            }[j["type"]],
        )
        ET.SubElement(
            je,
            "origin",
            xyz=fmt(j["origin"]["xyz_m"]),
            rpy=fmt(j["origin"]["rpy_rad"]),
        )
        ET.SubElement(je, "parent", link=j["parent_link"])
        ET.SubElement(je, "child", link=j["child_link"])
        if j["type"] != "fixed":
            ET.SubElement(je, "axis", xyz=fmt(j["axis"]))
        lim = j["limits"]
        if lim is not None:
            bounds = (lim["software_lower_rad"], lim["software_upper_rad"], lim["effort_peak_nm"],
                      lim["velocity_rad_s"])
        elif j["name"] in drive_by_input:
            # A linkage's input: the image of its output's range, and the bounds that implies.
            dl = drive_by_input[j["name"]]["limits"]
            bounds = (dl["position_min_rad"], dl["position_max_rad"], dl["effort_peak_nm"],
                      dl["velocity_rad_s"])
        else:
            # A linkage's coupler: a free pin, which carries no torque about its axis.
            c = coupler_by_joint[j["name"]]
            bounds = (c["position_min_rad"], c["position_max_rad"], 0.0, c["velocity_rad_s"])
        ET.SubElement(je, "limit", lower=f"{bounds[0]:.10g}", upper=f"{bounds[1]:.10g}",
                      effort=f"{bounds[2]:.10g}", velocity=f"{bounds[3]:.10g}")
        tr = j["transmission"]
        ET.SubElement(
            je,
            "dynamics",
            damping=f"{tr['friction_viscous_nm_s_rad']:.10g}",
            friction=f"{tr['friction_coulomb_nm']:.10g}",
        )

    imu = model.get("imu")
    if imu is not None:
        # The frame the IMU's readings are expressed in, for TF and for the broadcaster's frame_id.
        ET.SubElement(robot, "link", name=f"{imu['name']}_link")
        ij = ET.SubElement(robot, "joint", name=f"{imu['name']}_joint", type="fixed")
        ET.SubElement(ij, "origin", xyz=fmt(imu["pos_m"]),
                      rpy=fmt(quat_to_rpy(imu["quat_wxyz"])))
        ET.SubElement(ij, "parent", link=imu["link"])
        ET.SubElement(ij, "child", link=f"{imu['name']}_link")

    rc = ET.SubElement(
        robot, "ros2_control", name="HumanoidActuatorSystem", type="system"
    )
    hw = ET.SubElement(rc, "hardware")
    ET.SubElement(hw, "plugin").text = (
        "humanoid_actuator_system/HumanoidActuatorSystem"
    )
    ET.SubElement(hw, "param", name="safety_manifest_path").text = safety_manifest_path
    ET.SubElement(hw, "param", name="safety_manifest_package").text = DESCRIPTION_PACKAGE
    # Controllers command the commanded joints. A linkage's input and coupler are state-only: the
    # hardware component derives them through the linkage, which <transmission> describes.
    for d in drives:
        je = ET.SubElement(rc, "joint", name=d["joint"])
        for c in CMD_INTERFACES:
            ET.SubElement(je, "command_interface", name=c)
        for s in STATE_INTERFACES:
            ET.SubElement(je, "state_interface", name=s)
    state_only = linkage.members(model)
    for j in joints:
        if j["name"] in state_only:
            je = ET.SubElement(rc, "joint", name=j["name"])
            for s in STATE_INTERFACES:
                ET.SubElement(je, "state_interface", name=s)
    for d in drives:
        if d["linkage"] is not None:
            emit_transmission(rc, d)
    if imu is not None:
        se = ET.SubElement(rc, "sensor", name=imu["name"])
        for s in IMU_INTERFACES:
            ET.SubElement(se, "state_interface", name=s)

    ET.indent(robot)
    ET.ElementTree(robot).write(
        out_path, xml_declaration=True, encoding="utf-8"
    )
    return mesh_files

def emit_transmission(rc, drive):
    """A linkage as a ros2_control <transmission>: the joint controllers command (role output),
    the joint the motor turns (the actuator, role input), the coupler, and the planar four-bar's
    parameters (linkage.planar_parameters). HumanoidActuatorSystem builds its FourBarLinkage from
    them; ros2_control's transmission_interface, whose transmissions are linear, never sees it."""
    lk = drive["linkage"]
    te = ET.SubElement(rc, "transmission", name=lk["name"])
    ET.SubElement(te, "plugin").text = FOUR_BAR_PLUGIN
    ET.SubElement(te, "actuator", name=lk["input_joint"], role="input")
    ET.SubElement(te, "joint", name=lk["output_joint"], role="output")
    ET.SubElement(te, "joint", name=lk["coupler_joint"], role="coupler")
    for key, value in drive["four_bar"].p.items():
        ET.SubElement(te, "param", name=key).text = (
            str(value) if isinstance(value, int) else f"{value:.10g}")
