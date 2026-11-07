"""robot_model.yaml (+ contact overlay) -> URDF, MJCF, safety_manifest.yaml.

Every actuated joint gets one MJCF <general> actuator that MujocoActuatorTransport turns into the
drive's MIT impedance law (ADR-001 tuple); see emit_mjcf_actuators. Compiler flags are the pinned
MJCF_COMPILER; see AMEND-1 and model-gate-0 §5.5.
"""
import hashlib
import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from . import linkage
from .common import quat_to_rpy, rpy_to_quat

TIMING_HPP = "src/humanoid_transport/include/humanoid_transport/timing.hpp"
FOUR_BAR_PLUGIN = "humanoid_actuator_system/FourBarLinkage"
# A linkage's closure in MJCF: a connect equality between two sites. MuJoCo's constraints are soft,
# so the pin stretches under load. solref's time constant is two 1 ms timesteps, the smallest that
# stays stable, and solimp's impedance the largest MuJoCo accepts. Measured on leg_prototype: under
# 1 um at its 3.05 N*m motor envelope and 5 um at the RS02's 17 N*m, held statically; 15 um
# transiently on a 0.2 rad step at kp 10 N*m/rad (0.5-0.75 mm at kp 200-330, the drive saturated).
# The input joint's armature is what keeps it small: see validator check L-06 and
# docs/leg-prototype.md.
LINKAGE_SOLREF = "0.002 1"
LINKAGE_SOLIMP = "0.9999 0.9999 0.001 0.5 2"

# Provisional envelope constants until G3 actuator characterisation.
PROV_KP_MAX, PROV_KD_MAX = 500.0, 10.0
PROV_SLEW_NM_S, PROV_POWER_W = 500.0, 400.0
CMD_INTERFACES = ("position", "velocity", "effort", "stiffness", "damping")
STATE_INTERFACES = ("position", "velocity", "effort")
# MJCF <compiler> attributes, all explicit: MuJoCo must never rewrite the model's inertial data, and
# never fall back to a default that differs between URDF and MJCF input.
MJCF_COMPILER = (
    ("angle", "radian"),           # the model's unit; the default differs between URDF and MJCF
    ("balanceinertia", "false"),   # true silently averages an invalid diagonal inertia
    ("boundmass", "0"),            # a positive bound silently raises a small mass
    ("boundinertia", "0"),         # a positive bound silently raises a small inertia
    ("settotalmass", "-1"),        # a positive value silently rescales every mass
    ("inertiafromgeom", "false"),  # inertia comes from the model, never from collision geometry
    ("fusestatic", "false"),       # fusing bodies loses the link identity URDF and firmware share
    ("strippath", "false"),        # the default differs between URDF and MJCF
    ("discardvisual", "false"),    # the default differs between URDF and MJCF
)
# The interface names imu_sensor_broadcaster reads; kImuStateFields in batch_types.hpp spells the
# same list, and the hardware contract rejects a description that differs.
IMU_INTERFACES = (
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
)

def load(src, overlay_path):
    with open(src) as f:
        model = yaml.safe_load(f)
    overlay = None
    if overlay_path and os.path.exists(overlay_path):
        with open(overlay_path) as f:
            overlay = yaml.safe_load(f)
    return model, overlay

def children_by_parent(joints):
    kids = {}
    for j in joints:
        kids.setdefault(j["parent_link"], []).append(j)
    return kids

def fmt(xs):
    return " ".join(f"{x:.10g}" for x in xs)

def mesh_file(mesh, disk_assets):
    """The file in the assets directory (listing `disk_assets`) holding mesh `mesh`.
    Deterministic: scan a sorted list, and prefer an exact match over a prefix one (os.listdir
    order must never decide "pelvis" -> pelvis.STL vs pelvis_contour_link.STL)."""
    names = sorted(disk_assets)
    found = next((f for f in names if f == mesh or os.path.splitext(f)[0] == mesh), None)
    if found is None:  # suffixes like torso_link_rev_1_0.STL
        found = next((f for f in names if os.path.splitext(f)[0].startswith(mesh)), None)
    if found is None:  # nothing on disk matched
        found = mesh if "." in mesh else f"{mesh}.STL"
    return found

def emit_urdf(model, out_path, assets_dir,
              safety_manifest_path="config/safety_manifest.yaml",
              safety_manifest_package="humanoid_bringup"):
    joints = model["joints"]
    drives = linkage.drives(model)
    drive_by_input = {d["actuated_joint"]: d for d in drives if d["linkage"] is not None}
    coupler_by_joint = {d["coupler"]["joint"]: d["coupler"] for d in drives
                        if d["coupler"] is not None}
    robot = ET.Element("robot", name=model["robot"]["name"])

    # Index actual files on disk (case-insensitive mapping). A primitive-only robot has no assets.
    disk_assets = os.listdir(assets_dir) if assets_dir and os.path.exists(assets_dir) else []
    # package:// rather than file://: the URI must not depend on where this ran (/ws, a laptop),
    # and Foxglove only fetches package:// meshes over its bridge. humanoid_bringup installs the
    # assets under the same repo-relative path (see its CMakeLists.txt).
    assets_uri = ("package://humanoid_bringup/" + os.path.relpath(assets_dir).replace("\\", "/")
                  if assets_dir else None)

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

        for tag, mesh in (("visual", l.get("visual_mesh")), ("collision", l["collision"]["mesh"])):
            if mesh:
                ve = ET.SubElement(le, tag)
                ET.SubElement(ve, "origin", xyz="0 0 0", rpy="0 0 0")
                ET.SubElement(ve, "geometry").append(ET.Element(
                    "mesh", filename=f"{assets_uri}/{mesh_file(mesh, disk_assets)}"))

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
    ET.SubElement(hw, "param", name="safety_manifest_path").text = (
        safety_manifest_path
    )
    ET.SubElement(hw, "param", name="safety_manifest_package").text = (
        safety_manifest_package
    )
    # Controllers command the commanded joints. A linkage's input and coupler are state-only: the
    # hardware component derives them through the linkage, which <transmission> describes.
    for d in drives:
        je = ET.SubElement(rc, "joint", name=d["joint"])
        for c in CMD_INTERFACES:
            ET.SubElement(je, "command_interface", name=c)
        for s in STATE_INTERFACES:
            ET.SubElement(je, "state_interface", name=s)
    for j in joints:
        if j["name"] in linkage.members(model):
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

def _mujoco():
    try:
        import mujoco  # only robots that emit MJCF need it
    except ImportError as e:
        raise RuntimeError("robot.xml derives its placement with MuJoCo: install the mujoco Python "
                           "package, or run in the dev container") from e
    return mujoco

def lowest_collision_z(m, d, root):
    """The lowest world z of the collidable geometry of the tree rooted at `root`, or None: the
    bounding-box rule of SIL's lowest_collision_z (src/humanoid_transport_mujoco/src/
    mujoco_placement.cpp), applied by MuJoCo to its own collision geometry."""
    mujoco = _mujoco()
    lowest = None
    for g in range(m.ngeom):
        b = m.geom_bodyid[g]
        collidable = m.geom_contype[g] != 0 or m.geom_conaffinity[g] != 0
        if (b == 0 or m.body_rootid[b] != root or not collidable or
                m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE):
            continue
        # geom_aabb is (centre, half-size) in the geom frame; the world-z row of the geom's rotation
        # maps it to a world centre height and half-height.
        centre, half, rz = m.geom_aabb[g][:3], m.geom_aabb[g][3:], d.geom_xmat[g][6:9]
        bottom = d.geom_xpos[g][2] + rz @ centre - np.abs(rz) @ half
        lowest = bottom if lowest is None else min(lowest, bottom)
    return lowest

def standing_qpos(mjcf_xml, root_link, joint_angles):
    """qpos with the joints at `joint_angles` (joint name -> rad; the rest at 0) and the root, at the
    origin in `mjcf_xml`, raised until the robot's lowest collision geom sits on z = 0, so the
    height is derived, never typed. Rounded up to the micrometre: the robot never starts inside the
    floor, and float noise cannot flip the output. The root's free joint is the first joint
    (qpos[0:7])."""
    mujoco = _mujoco()
    m = mujoco.MjModel.from_xml_string(mjcf_xml)
    root = m.body(root_link).id
    if m.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE or m.jnt_bodyid[0] != root:
        raise ValueError(f"{root_link}: the first joint must be the root's free joint")
    d = mujoco.MjData(m)
    for name, angle in joint_angles.items():
        d.qpos[m.joint(name).qposadr[0]] = angle
    mujoco.mj_kinematics(m, d)
    lowest = lowest_collision_z(m, d, root)
    if lowest is None:
        raise ValueError(f"{root_link}: no collidable geometry to stand on")
    qpos = [float(x) for x in d.qpos]
    qpos[2] = math.ceil(-lowest * 1e6) / 1e6
    return qpos

def mount_height(mjcf_xml, model, samples=41):
    """The root height of a fixed-base robot whose root, at the origin in `mjcf_xml`, is lifted just
    clear of the floor: with each commanded joint swept over its hard stops (the others at 0, every
    linkage closed), its collision geometry never goes below z = 0. Rounded up to the micrometre.
    A test stand's real height is the operator's choice; this is the lowest one that never collides
    with the floor, so a fixed-base scene stays contact-free."""
    mujoco = _mujoco()
    m = mujoco.MjModel.from_xml_string(mjcf_xml)
    d = mujoco.MjData(m)
    root = m.body(model["robot"]["root_link"]).id
    by_output = {dr["joint"]: dr for dr in linkage.drives(model) if dr["linkage"] is not None}
    lowest = None
    for j in linkage.commanded_joints(model):
        lim = j["limits"]
        for q in np.linspace(lim["hard_stop_lower_rad"], lim["hard_stop_upper_rad"], samples):
            mujoco.mj_resetData(m, d)
            d.qpos[m.joint(j["name"]).qposadr[0]] = q
            dr = by_output.get(j["name"])
            if dr is not None:
                q_in, q_cpl, _, _ = dr["four_bar"].from_output(float(q))
                d.qpos[m.joint(dr["linkage"]["input_joint"]).qposadr[0]] = q_in
                d.qpos[m.joint(dr["linkage"]["coupler_joint"]).qposadr[0]] = q_cpl
            mujoco.mj_kinematics(m, d)
            z = lowest_collision_z(m, d, root)
            if z is not None:
                lowest = z if lowest is None else min(lowest, z)
    if lowest is None:
        raise ValueError(f"{model['robot']['root_link']}: no collidable geometry to clear the "
                         "floor")
    return math.ceil(-lowest * 1e6) / 1e6

def emit_mjcf(model, overlay, out_path, assets_dir, assets_rel):
    """Write robot.xml. Returns the standing qpos of the model's `home` pose, or None without one.

    Geometry comes from the contact overlay when the robot has one (G1, imported from Menagerie);
    otherwise from the model: each link's visual mesh as a non-colliding geom (group 2) and its
    collision primitives (group 3). A linkage's closure pin is a connect equality between two sites
    that coincide at qpos0, and the linkage's bodies never collide with each other: their pins and
    clearances are finer than the primitives, and the output joint's range models the hard stops."""
    links = {l["name"]: l for l in model["links"]}
    joints = model["joints"]
    kids = children_by_parent(joints)
    joint_by_child = {j["child_link"]: j for j in joints}
    joints_by_name = {j["name"]: j for j in joints}
    imu = model.get("imu")
    floating = model["robot"]["base"] == "floating"
    linkages = model.get("linkages", [])

    mj = ET.Element("mujoco", model=model["robot"]["name"])
    comp = ET.SubElement(mj, "compiler")
    for key, value in MJCF_COMPILER:
        comp.set(key, value)
    comp.set("meshdir", assets_rel.replace("\\", "/"))

    asset = ET.SubElement(mj, "asset")
    if overlay:
        meshes = overlay["meshes"]
    else:
        disk_assets = os.listdir(assets_dir) if assets_dir and os.path.exists(assets_dir) else []
        meshes = {mesh: mesh_file(mesh, disk_assets) for l in model["links"]
                  for mesh in (l["visual_mesh"], l["collision"]["mesh"]) if mesh}
    for mname in sorted(k for k in meshes if k is not None):
        if meshes[mname]:
            ET.SubElement(asset, "mesh", name=mname, file=meshes[mname])

    # Each linkage's closure pin: a site in each of the two links it joins.
    sites = {}
    for lk in linkages:
        cl = lk["closure"]
        sites.setdefault(joints_by_name[lk["coupler_joint"]]["child_link"], []).append(
            (f"{lk['name']}_coupler_pin", cl["coupler_point_m"]))
        sites.setdefault(joints_by_name[lk["input_joint"]]["child_link"], []).append(
            (f"{lk['name']}_input_pin", cl["input_point_m"]))

    def add_geoms(be, link_name):
        if overlay:
            for g in overlay["bodies"].get(link_name, []):
                ge = ET.SubElement(be, "geom", type=g["type"])
                ge.set("pos", fmt(g["pos"])); ge.set("quat", fmt(g["quat"]))
                for a in ("size", "mesh", "friction", "condim", "solref", "solimp",
                          "contype", "conaffinity", "group", "density"):
                    if a in g and g[a] is not None:
                        val = g[a]
                        ge.set(a, val if isinstance(val, str) else fmt(val))
            return
        l = links[link_name]
        if l["visual_mesh"]:
            ET.SubElement(be, "geom", name=f"{link_name}_visual", type="mesh",
                          mesh=l["visual_mesh"], contype="0", conaffinity="0", group="2")
        if l["collision"]["mesh"]:
            ET.SubElement(be, "geom", name=f"{link_name}_collision_mesh", type="mesh",
                          mesh=l["collision"]["mesh"], group="3")
        for i, prim in enumerate(l["collision"]["primitives"]):
            ET.SubElement(be, "geom", name=f"{link_name}_collision_{i}", type=prim["shape"],
                          size=fmt(prim["dimensions_m"]), pos=fmt(prim["frame"]["xyz_m"]),
                          quat=fmt(rpy_to_quat(prim["frame"]["rpy_rad"])), group="3")

    def add_body(be_parent, link_name, is_root):
        l = links[link_name]
        be = ET.SubElement(be_parent, "body", name=link_name)
        if not is_root:
            j = joint_by_child[link_name]
            be.set("pos", fmt(j["origin"]["xyz_m"]))
            be.set("quat", fmt(rpy_to_quat(j["origin"]["rpy_rad"])))
        elif floating:
            ET.SubElement(be, "freejoint", name="root")
        inertia, com = l["inertia"], l["com_m"]
        ie = ET.SubElement(be, "inertial")
        ie.set("pos", fmt(com)); ie.set("mass", f"{l['mass_kg']:.10g}")
        ie.set("fullinertia", fmt([inertia["ixx_kg_m2"], inertia["iyy_kg_m2"],
                                   inertia["izz_kg_m2"], inertia["ixy_kg_m2"],
                                   inertia["ixz_kg_m2"], inertia["iyz_kg_m2"]]))
        add_geoms(be, link_name)
        if imu is not None and imu["link"] == link_name:
            ET.SubElement(be, "site", name=imu["name"], pos=fmt(imu["pos_m"]),
                          quat=fmt(imu["quat_wxyz"]), size="0.01")
        for name, point in sites.get(link_name, []):
            ET.SubElement(be, "site", name=name, pos=fmt(point), size="0.002")
        if not is_root:
            j = joint_by_child[link_name]
            if j["type"] != "fixed":
                tr = j["transmission"]
                je = ET.SubElement(be, "joint", name=j["name"],
                                   type="hinge" if j["type"] == "revolute" else "slide",
                                   axis=fmt(j["axis"]))
                # A linkage's input and coupler have no limits of their own: the loop bounds them.
                if j["limits"] is not None:
                    je.set("range", f"{j['limits']['hard_stop_lower_rad']:.10g} "
                                    f"{j['limits']['hard_stop_upper_rad']:.10g}")
                je.set("damping", f"{tr['friction_viscous_nm_s_rad']:.10g}")
                je.set("frictionloss", f"{tr['friction_coulomb_nm']:.10g}")
                je.set("armature", f"{tr['armature_kg_m2']:.10g}")
        for cj in kids.get(link_name, []):
            add_body(be, cj["child_link"], False)

    wb = ET.SubElement(mj, "worldbody")
    add_body(wb, model["robot"]["root_link"], True)
    root_body = wb.find("body")

    if linkages:
        contact = ET.SubElement(mj, "contact")
        equality = ET.SubElement(mj, "equality")
        for lk in linkages:
            j_in, j_out, j_cpl = (joints_by_name[lk[k]] for k in
                                  ("input_joint", "output_joint", "coupler_joint"))
            bodies = [j_out["parent_link"], j_out["child_link"], j_in["child_link"],
                      j_cpl["child_link"]]
            for i, b1 in enumerate(bodies):
                for b2 in bodies[i + 1:]:
                    ET.SubElement(contact, "exclude", body1=b1, body2=b2)
            ET.SubElement(equality, "connect", name=f"{lk['name']}_pin",
                          site1=f"{lk['name']}_coupler_pin", site2=f"{lk['name']}_input_pin",
                          solref=LINKAGE_SOLREF, solimp=LINKAGE_SOLIMP)

    emit_mjcf_actuators(mj, linkage.drives(model))

    if imu is not None:
        # Exactly one sensor of each type: MujocoActuatorTransport finds them by type, not name.
        sensors = ET.SubElement(mj, "sensor")
        ET.SubElement(sensors, "framequat", name=f"{imu['name']}_orientation", objtype="site",
                      objname=imu["name"])
        ET.SubElement(sensors, "gyro", name=f"{imu['name']}_angular_velocity", site=imu["name"])
        ET.SubElement(sensors, "accelerometer", name=f"{imu['name']}_linear_acceleration",
                      site=imu["name"])

    # Place the robot so qpos0 is physically valid for every consumer, not only SIL: a floating
    # robot stands on z = 0 at its reference pose, a fixed one is mounted clear of the floor.
    # Compiled with an absolute meshdir so this works from any directory.
    comp.set("meshdir", os.path.abspath(assets_dir).replace("\\", "/"))
    xml = ET.tostring(mj, encoding="unicode")
    root_link = model["robot"]["root_link"]
    if floating:
        root_z = standing_qpos(xml, root_link, {})[2]
        home = standing_qpos(xml, root_link, model["home"]) if "home" in model else None
    else:
        root_z = mount_height(xml, model)
        home = None
    comp.set("meshdir", assets_rel.replace("\\", "/"))
    root_body.set("pos", fmt([0.0, 0.0, root_z]))

    ET.indent(mj)
    ET.ElementTree(mj).write(out_path, xml_declaration=True, encoding="utf-8")
    return home

def emit_mjcf_actuators(mj, drives):
    """One actuator per drive, on the joint its motor turns, for MujocoActuatorTransport to drive
    with the MIT law

        tau = kp (q_d - q) + kd (qdot_d - qdot) + tau_ff.

    MuJoCo's affine actuator computes force = gain * ctrl + b0 + b1 q + b2 qdot. With gain 1 the
    transport sets ctrl = kp q_d + kd qdot_d + tau_ff and b = (0, -kp, -kd) every cycle, so MuJoCo
    evaluates the law at every physics step, as the drive does at its own loop rate, and the
    integrator treats the damping term implicitly. forcerange saturates at the actuated joint's
    peak torque, as the drive does. Through a linkage the motor turns the input crank, and MuJoCo's
    closed loop, not this model, carries its torque to the output."""
    act = ET.SubElement(mj, "actuator")
    for d in drives:
        if d["actuator"] is None:
            continue
        peak = d["limits"]["effort_peak_nm"]
        ET.SubElement(act, "general", name=d["actuator"], joint=d["actuated_joint"],
                      gaintype="fixed", gainprm="1", biastype="affine", biasprm="0 0 0",
                      ctrllimited="false", forcelimited="true",
                      forcerange=f"{-peak:.10g} {peak:.10g}")

def emit_scene(model, out_path, home_qpos):
    """The world every robot is simulated in: physics settings and a floor at z = 0, around an
    include of robot.xml. One definition for all robots, and robot.xml stays free of world
    assumptions, so another scene (a field with a ball) can include the same robot. The `home`
    keyframe lives here, not in robot.xml: a keyframe sets every qpos, and MuJoCo fills the qpos
    of anything a scene adds (the ball) with zeros rather than that body's own position."""
    mj = ET.Element("mujoco", model=model["robot"]["name"])
    ET.SubElement(mj, "include", file="robot.xml")
    # 1 ms physics: 5 substeps per 5 ms control period (ADR-007-03, asserted at configure).
    ET.SubElement(mj, "option", timestep="0.001", integrator="implicitfast")
    wb = ET.SubElement(mj, "worldbody")
    ET.SubElement(wb, "geom", name="floor", type="plane", size="0 0 0.1", pos="0 0 0",
                  friction="0.6", condim="3")
    if home_qpos is not None:
        ET.SubElement(ET.SubElement(mj, "keyframe"), "key", name="home", qpos=fmt(home_qpos))
    ET.indent(mj)
    ET.ElementTree(mj).write(out_path, xml_declaration=True, encoding="utf-8")

def emit_safety_manifest(model, motor_limits, out_path):
    """The SafetyKernel's envelopes, one per motor, keyed by the joint the motor turns: the kernel
    is the last gate before the transport, so it bounds what each motor is asked to do. Through a
    linkage those bounds are derived from the output joint's limits (linkage.drives).
    Per-joint gain caps come from the actuator's motor model in model/motors.yaml; a PROVISIONAL sim
    actuator has none and falls back to the global PROV_* envelope."""
    act_by_name = {a["name"]: a for a in model["actuators"]}
    envelopes = {}
    for d in linkage.drives(model):
        lim = d["limits"]
        drive = motor_limits.get(act_by_name[d["actuator"]]["motor_class"])
        envelopes[d["actuated_joint"]] = {
            "position_min_rad": lim["position_min_rad"],
            "position_max_rad": lim["position_max_rad"],
            "velocity_max_rad_s": lim["velocity_rad_s"],
            "torque_continuous_nm": lim["effort_continuous_nm"],
            "torque_peak_nm": lim["effort_peak_nm"],
            "torque_peak_duration_s": lim["effort_peak_duration_s"],
            "stiffness_max_nm_rad": drive["kp_max"] if drive else PROV_KP_MAX,
            "damping_max_nm_s_rad": drive["kd_max"] if drive else PROV_KD_MAX,
            "torque_slew_max_nm_s": PROV_SLEW_NM_S,
            "power_max_w": PROV_POWER_W,
        }
    manifest = {
        "schema_version": "0.1.0-provisional",
        "source_digest_note": "envelopes derived from robot_model.yaml limits; "
                              "stiffness/damping/slew/power are provisional constants",
        "joint_count": len(envelopes),
        "max_consecutive_bad_cycles": 3,   # ADR-002 command-loss rule
        "feedback_max_age_us": 15000,      # three 5 ms cycles
        "envelopes": envelopes,
    }
    with open(out_path, "w") as f:
        yaml.safe_dump(manifest, f, sort_keys=False, width=1000)

def control_rate_hz():
    """The control rate, read from its single source of truth (timing.hpp), never hand-typed."""
    with open(TIMING_HPP) as f:
        m = re.search(r"kControlRateHz\s*=\s*(\d+)", f.read())
    if not m:
        raise ValueError(f"kControlRateHz not found in {TIMING_HPP}")
    return int(m.group(1))

def emit_firmware_config(model, motor_limits, out_path, setup):
    """A config artifact the firmware repo consumes (it is committed there with its hash).
    Everything is derived from robot_model.yaml; the firmware never hand-edits it, and the hash
    lets its generator refuse a tampered copy."""
    drive_by_act = {d["actuator"]: d for d in linkage.drives(model)}

    # Per-model drive ranges and the shared Kp/Kd/position ranges, from the motor models used.
    models, shared = {}, None
    for a in model["actuators"]:
        d = motor_limits.get(a["motor_class"])
        if d is None or "default_kp" not in a or "default_kd" not in a:
            raise ValueError(f"actuator {a['name']} needs a model/motors.yaml motor_class and "
                             "default_kp/default_kd for firmware_config")
        v, t = d["velocity_max_rad_s"], d["torque_max_nm"]
        models.setdefault(a["motor_class"],
                          {"v_min": -v, "v_max": v, "t_min": -t, "t_max": t})
        p = d["position_max_rad"]
        sh = {"p_min": -p, "p_max": p, "kp_min": 0.0, "kp_max": d["kp_max"],
              "kd_min": 0.0, "kd_max": d["kd_max"]}
        shared = shared or sh

    # One group per bus segment, motors ordered by slot.
    slaves = []
    for seg in sorted({a["bus_segment"] for a in model["actuators"]}):
        motors = sorted((a for a in model["actuators"] if a["bus_segment"] == seg),
                        key=lambda a: a["slot"])
        rows = []
        for a in motors:
            # The motor's own joint and bounds: through a linkage, the input crank's.
            d = drive_by_act[a["name"]]
            lim = d["limits"]
            rows.append({
                "idx": a["slot"], "can_id": a["can_node_id"], "model": a["motor_class"],
                "joint_name": d["actuated_joint"],
                "soft_min": lim["position_min_rad"], "soft_max": lim["position_max_rad"],
                "max_vel": lim["velocity_rad_s"], "max_tau": lim["effort_peak_nm"],
                "default_kp": a["default_kp"], "default_kd": a["default_kd"],
            })
        slaves.append({"slave": seg, "motors": rows})

    rate = control_rate_hz()
    body = {
        "setup": setup,
        "rates": {"master_poll_hz": rate, "telemetry_hz": rate,
                  "slave_tick_hz": rate, "host_cmd_hz": rate},
        "shared_ranges": shared,
        "models": {k: models[k] for k in sorted(models)},
        "slaves": slaves,
    }
    digest = hashlib.sha256(
        yaml.safe_dump(body, sort_keys=True, default_flow_style=False).encode()).hexdigest()

    banner = (f"# AUTO-GENERATED by model/generators from {model['robot']['name']}. Do not edit.\n"
              f"# The firmware repo consumes this file; verify content_sha256 before use.\n")
    doc = {"setup": setup, "content_sha256": digest, **{k: v for k, v in body.items()
                                                        if k != "setup"}}
    with open(out_path, "w") as f:
        f.write(banner)
        yaml.safe_dump(doc, f, sort_keys=False, width=1000)
    with open(out_path + ".sha256", "w") as f:
        f.write(f"{digest}  {os.path.basename(out_path)}\n")

def emit_stm32_wiring(model, out_path):
    """The `joints:` block for the Stm32SerialTransport node (`stm32_serial`), generated from the
    model so the transport's identity map is derived, not hand-written (per the transport doc).
    One entry per motor, named by the joint it turns. chain = index of the actuator's bus_segment;
    motor = its slot; direction_sign and zero_offset_rad come from that joint. A zero offset not yet
    calibrated (null) is left out, so the transport refuses to configure rather than run on a guess.
    Serial device and timeouts are deployment parameters and stay in the launch, not here."""
    segments = sorted({a["bus_segment"] for a in model["actuators"]})
    chain_of = {seg: i for i, seg in enumerate(segments)}
    act_by_name = {a["name"]: a for a in model["actuators"]}
    joints_by_name = {j["name"]: j for j in model["joints"]}
    rows = []
    for d in linkage.drives(model):
        a, j = act_by_name[d["actuator"]], joints_by_name[d["actuated_joint"]]
        rows.append((chain_of[a["bus_segment"]], a["slot"], j["name"],
                     j["transmission"]["direction_sign"], j["calibration"]["zero_offset_rad"]))
    rows.sort()
    joints = {}
    for chain, motor, name, sign, zero in rows:
        joints[name] = {"chain": chain, "motor": motor, "direction_sign": sign}
        if zero is not None:
            joints[name]["zero_offset_rad"] = zero
    doc = {"stm32_serial": {"ros__parameters": {"joints": joints}}}
    banner = (f"# AUTO-GENERATED by model/generators from {model['robot']['name']}. Do not edit.\n"
              "# The Stm32SerialTransport joints map (chain/motor/direction_sign/zero_offset_rad).\n"
              "# zero_offset_rad and direction_sign are provisional until verified by hand.\n")
    uncalibrated = [name for _, _, name, _, zero in rows if zero is None]
    if uncalibrated:
        banner += (f"# Not calibrated, so the transport will refuse to start: "
                   f"{', '.join(uncalibrated)}.\n")
    with open(out_path, "w") as f:
        f.write(banner)
        yaml.safe_dump(doc, f, sort_keys=False, width=1000)

def generate(src, overlay_path, motor_limits, out_dir, generated_dir, assets_dir, urdf, mjcf,
             firmware_config=False, firmware_setup=None, stm32_wiring=False,
             safety_manifest_path=None, safety_manifest_package=None):
    """Write the robot's generated files to out_dir. generated_dir is where they live in the repo:
    the MJCF's relative meshdir is computed from it, so a regeneration elsewhere (check's temp
    dir) still matches the committed file."""
    model, overlay = load(src, overlay_path)
    os.makedirs(out_dir, exist_ok=True)
    written = []

    if urdf:
        # URDF gets assets_dir to build package:// URIs with .STL extensions. The safety-manifest
        # path/package override is per-robot; the defaults keep G1's ros2_control block unchanged.
        urdf_kwargs = {}
        if safety_manifest_path:
            urdf_kwargs["safety_manifest_path"] = safety_manifest_path
        if safety_manifest_package:
            urdf_kwargs["safety_manifest_package"] = safety_manifest_package
        emit_urdf(model, os.path.join(out_dir, "robot.urdf"), assets_dir, **urdf_kwargs)
        written.append("robot.urdf")

    if mjcf:
        # MJCF keeps relative path because of the <compiler meshdir="..." /> tag
        assets_rel = os.path.relpath(assets_dir, generated_dir).replace("\\", "/")
        home_qpos = emit_mjcf(model, overlay, os.path.join(out_dir, "robot.xml"), assets_dir,
                              assets_rel)
        emit_scene(model, os.path.join(out_dir, "scene.xml"), home_qpos)
        written += ["robot.xml", "scene.xml"]

    emit_safety_manifest(model, motor_limits, os.path.join(out_dir, "safety_manifest.yaml"))
    written.append("safety_manifest.yaml")

    if firmware_config:
        setup = firmware_setup or model["robot"]["name"]
        emit_firmware_config(model, motor_limits, os.path.join(out_dir, "firmware_config.yaml"),
                             setup)
        written += ["firmware_config.yaml", "firmware_config.yaml.sha256"]

    if stm32_wiring:
        emit_stm32_wiring(model, os.path.join(out_dir, "stm32_wiring.yaml"))
        written.append("stm32_wiring.yaml")

    print(f"generated {', '.join(written)} in {out_dir}")