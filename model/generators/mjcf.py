"""robot_model.yaml (+ contact overlay) -> robot.xml and scene.xml.

Every actuated joint gets one MJCF <general> actuator that MujocoActuatorTransport turns into the
drive's MIT impedance law (ADR-001 tuple); see emit_mjcf_actuators. Compiler flags are the pinned
MJCF_COMPILER; see AMEND-1 and model-gate-0 §5.5. The robot's placement (standing height, mount
height) and the links that touch at the zero pose are derived by MuJoCo from the emitted model.
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

from . import linkage
from .common import collision_meshes, fmt, mesh_file, rpy_to_quat

# MJCF has no package:// and resolves meshdir against the model file's directory, so the meshes sit
# beside mjcf/ in the same tree, installed and in the repository alike.
MJCF_MESHDIR = "../meshes"
# A linkage's closure in MJCF: a connect equality between two sites. MuJoCo's constraints are soft,
# so the pin stretches under load. solref's time constant is two 1 ms timesteps, the smallest that
# stays stable, and solimp's impedance the largest MuJoCo accepts. Measured on leg_prototype: under
# 1 um at its 3.05 N*m motor envelope and 5 um at the RS02's 17 N*m, held statically; 15 um
# transiently on a 0.2 rad step at kp 10 N*m/rad, kd 0.3 (39 um at kp 200-330 with the drive
# saturated, growing with kd to 0.18 mm at kd 3.35). The input joint's armature is what keeps it
# small: see validator check L-06 and docs/leg-prototype.md.
LINKAGE_SOLREF = "0.002 1"
LINKAGE_SOLIMP = "0.9999 0.9999 0.001 0.5 2"
# Closer than this at the zero pose, two links' collision geometry counts as touching (see
# touching_at_zero_pose). MuJoCo's contacts act only on penetration (margin 0), and a designed
# clearance this small between links that are not parent and child is a joint's nested parts.
ZERO_POSE_CLEARANCE_M = 0.001
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

def children_by_parent(joints):
    kids = {}
    for j in joints:
        kids.setdefault(j["parent_link"], []).append(j)
    return kids

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

def touching_at_zero_pose(mjcf_xml, model):
    """Pairs of links whose collision geometry comes within ZERO_POSE_CLEARANCE_M of each other at
    the zero pose (every joint at 0, each linkage closed), sorted; a parent and its child are left
    out, since MuJoCo never collides them. The CAD is clear at the zero pose, so a pair that touches
    there is an artefact of the collision geometry: a convex piece fills the clearance a joint's
    parts nest in, and left in, the pair would push apart from the first step. Excluding it costs
    that pair's contacts at every pose; the joint ranges, from the CAD meshes, keep it apart."""
    mujoco = _mujoco()
    m = mujoco.MjModel.from_xml_string(mjcf_xml)
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)
    links = {l["name"] for l in model["links"]}
    geoms = [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name in links and
             (m.geom_contype[g] or m.geom_conaffinity[g])]
    pairs = set()
    for i, g1 in enumerate(geoms):
        for g2 in geoms[i + 1:]:
            b1, b2 = m.geom_bodyid[g1], m.geom_bodyid[g2]
            if b1 == b2 or m.body_parentid[b1] == b2 or m.body_parentid[b2] == b1:
                continue
            names = tuple(sorted((m.body(b1).name, m.body(b2).name)))
            if names in pairs:
                continue
            fromto = np.zeros(6)
            if mujoco.mj_geomDistance(m, d, g1, g2, ZERO_POSE_CLEARANCE_M, fromto) < \
                    ZERO_POSE_CLEARANCE_M:
                pairs.add(names)
    return sorted(pairs)

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

def emit_mjcf(model, overlay, out_path, meshes_dir):
    """Write robot.xml. Returns the standing qpos of the model's `home` pose (None without one) and
    the mesh files it names.

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
    comp.set("meshdir", MJCF_MESHDIR)

    asset = ET.SubElement(mj, "asset")
    if overlay:
        meshes = overlay["meshes"]
    else:
        disk_assets = os.listdir(meshes_dir) if meshes_dir and os.path.exists(meshes_dir) else []
        meshes = {mesh: mesh_file(mesh, disk_assets) for l in model["links"]
                  for mesh in [l["visual_mesh"]] + collision_meshes(l) if mesh}
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
        pieces = collision_meshes(l)
        for i, mesh in enumerate(pieces):
            name = f"{link_name}_collision_mesh" + (f"_{i}" if len(pieces) > 1 else "")
            ET.SubElement(be, "geom", name=name, type="mesh", mesh=mesh, group="3")
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
            j_in, j_out, j_cpl = (joints_by_name[lk[k]] for k in linkage.ROLES)
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
    comp.set("meshdir", os.path.abspath(meshes_dir).replace("\\", "/"))
    xml = ET.tostring(mj, encoding="unicode")
    if not overlay:
        excluded = {frozenset((e.get("body1"), e.get("body2"))) for e in mj.iter("exclude")}
        touching = [p for p in touching_at_zero_pose(xml, model) if frozenset(p) not in excluded]
        if touching:
            contact = mj.find("contact")
            if contact is None:
                contact = ET.Element("contact")
                mj.insert(list(mj).index(mj.find("worldbody")) + 1, contact)
            for b1, b2 in touching:
                ET.SubElement(contact, "exclude", body1=b1, body2=b2)
            xml = ET.tostring(mj, encoding="unicode")
    root_link = model["robot"]["root_link"]
    if floating:
        root_z = standing_qpos(xml, root_link, {})[2]
        home = standing_qpos(xml, root_link, model["home"]) if "home" in model else None
    else:
        root_z = mount_height(xml, model)
        home = None
    comp.set("meshdir", MJCF_MESHDIR)
    root_body.set("pos", fmt([0.0, 0.0, root_z]))

    ET.indent(mj)
    ET.ElementTree(mj).write(out_path, xml_declaration=True, encoding="utf-8")
    return home, {f for f in meshes.values() if f}

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
