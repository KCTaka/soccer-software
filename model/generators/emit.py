"""robot_model.yaml (+ contact overlay) -> URDF, MJCF, safety_manifest.yaml.

MJCF is emitted with NO <actuator> section: MujocoActuatorTransport applies the MIT
tuple as joint torque via qfrc_applied (ADR-007 seam, ADR-001 tuple). Compiler flags
come from the pinned simulation block; see AMEND-1 and model-gate-0 §5.5.
"""
import math
import os
import xml.etree.ElementTree as ET

import yaml

from .common import num_or, rpy_to_quat

# Provisional envelope constants until G3 actuator characterisation.
PROV_KP_MAX, PROV_KD_MAX = 500.0, 10.0
PROV_SLEW_NM_S, PROV_POWER_W = 500.0, 400.0
CMD_INTERFACES = ("position", "velocity", "effort", "stiffness", "damping")
STATE_INTERFACES = ("position", "velocity", "effort")

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

def _inertial_xml(link):
    inertia, com = link["inertia"], link["com_m"]
    e = ET.Element("inertial")
    if inertia["reference_frame"] == "principal":
        e.set("origin", "")  # replaced below; kept explicit for clarity
        e.attrib.clear()
        rpy = inertia["principal_axes_rpy_rad"]
        e.set("xyz", fmt(com)); e.set("rpy", fmt(rpy))
        e2 = ET.SubElement(link_elem_placeholder := e, "mass") if False else None
    return e  # placeholder; real construction below

def emit_urdf(model, out_path, assets_rel):
    links = {l["name"]: l for l in model["links"]}
    joints = model["joints"]
    robot = ET.Element("robot", name=model["robot"]["name"])

    for l in model["links"]:
        le = ET.SubElement(robot, "link", name=l["name"])
        ie = ET.SubElement(le, "inertial")
        inertia, com = l["inertia"], l["com_m"]
        rpy = inertia["principal_axes_rpy_rad"] or [0, 0, 0]
        ie.set("xyz", fmt(com)); ie.set("rpy", fmt(rpy))
        ET.SubElement(ie, "mass", value=f"{l['mass']['cad_kg']:.10g}")
        ET.SubElement(ie, "inertia",
                      ixx=f"{inertia['ixx_kg_m2']:.10g}", iyy=f"{inertia['iyy_kg_m2']:.10g}",
                      izz=f"{inertia['izz_kg_m2']:.10g}", ixy="0", ixz="0", iyz="0")
        # NOTE: when reference_frame == "link", rotate the tensor by rpy=0 identity only if the
        # tensor is already link-aligned; Menagerie data is principal, which is what we emit.
        mesh = l["visual_mesh"]
        if mesh:
            for tag in ("visual", "collision"):
                ve = ET.SubElement(le, tag)
                oe = ET.SubElement(ve, "origin", xyz="0 0 0", rpy="0 0 0")
                ET.SubElement(ve, "geometry").append(
                    ET.Element("mesh", filename=f"{assets_rel}/{mesh}"))
        for prim in l["collision"]["primitives"]:
            ve = ET.SubElement(le, "collision")
            ET.SubElement(ve, "origin", xyz=fmt(prim["frame"]["xyz_m"]),
                          rpy=fmt(prim["frame"]["rpy_rad"]))
            geometry = ET.SubElement(ve, "geometry")
            shape = prim["shape"]
            dimensions = prim["dimensions_m"]
            if shape == "box":
                ET.SubElement(geometry, "box", size=fmt([2.0 * x for x in dimensions]))
            elif shape == "sphere":
                ET.SubElement(geometry, "sphere", radius=f"{dimensions[0]:.10g}")
            elif shape == "cylinder":
                ET.SubElement(geometry, "cylinder", radius=f"{dimensions[0]:.10g}",
                              length=f"{2.0 * dimensions[1]:.10g}")
            elif shape == "capsule":
                ET.SubElement(geometry, "capsule", radius=f"{dimensions[0]:.10g}",
                              length=f"{2.0 * dimensions[1]:.10g}")

    for j in joints:
        je = ET.SubElement(robot, "joint", name=j["name"],
                           type={"revolute": "revolute", "prismatic": "prismatic",
                                 "fixed": "fixed"}[j["type"]])
        ET.SubElement(je, "origin", xyz=fmt(j["origin"]["xyz_m"]),
                      rpy=fmt(j["origin"]["rpy_rad"]))
        ET.SubElement(je, "parent", link=j["parent_link"])
        ET.SubElement(je, "child", link=j["child_link"])
        if j["type"] != "fixed":
            ET.SubElement(je, "axis", xyz=fmt(j["axis"]))
        lim = j["limits"]
        ET.SubElement(je, "limit", lower=f"{lim['software_lower_rad']:.10g}",
                      upper=f"{lim['software_upper_rad']:.10g}",
                      effort=f"{lim['effort_peak_nm']:.10g}",
                      velocity=f"{lim['velocity_rad_s']:.10g}")
        tr = j["transmission"]
        ET.SubElement(je, "dynamics", damping=f"{num_or(tr['friction_viscous_nm_s_rad']):.10g}",
                      friction=f"{num_or(tr['friction_coulomb_nm']):.10g}")

    rc = ET.SubElement(robot, "ros2_control", name="HumanoidActuatorSystem", type="system")
    hw = ET.SubElement(rc, "hardware")
    ET.SubElement(hw, "plugin").text = "humanoid_actuator_system/HumanoidActuatorSystem"
    ET.SubElement(hw, "param", name="robot_model_path").text = "model/source/robot_model.yaml"
    for j in joints:
        if j["type"] == "fixed":
            continue
        je = ET.SubElement(rc, "joint", name=j["name"])
        for c in CMD_INTERFACES:
            ET.SubElement(je, "command_interface", name=c)
        for s in STATE_INTERFACES:
            ET.SubElement(je, "state_interface", name=s)

    ET.indent(robot)
    ET.ElementTree(robot).write(out_path, xml_declaration=True, encoding="utf-8")

def emit_mjcf(model, overlay, out_path, assets_rel):
    links = {l["name"]: l for l in model["links"]}
    joints = model["joints"]
    kids = children_by_parent(joints)
    joint_by_child = {j["child_link"]: j for j in joints}
    sim = model["simulation"]["mjcf_compiler"]

    mj = ET.Element("mujoco", model=model["robot"]["name"])
    comp = ET.SubElement(mj, "compiler")
    comp.set("angle", sim["angle"])
    comp.set("balanceinertia", "true" if sim["balanceinertia"] else "false")
    comp.set("boundmass", f"{sim['boundmass']}")
    comp.set("boundinertia", f"{sim['boundinertia']}")
    comp.set("settotalmass", f"{sim['settotalmass']}")
    comp.set("inertiafromgeom", sim["inertiafromgeom"])
    comp.set("fusestatic", sim["fusestatic"])
    comp.set("strippath", "true" if sim["strippath"] else "false")
    comp.set("discardvisual", "true" if sim["discardvisual"] else "false")
    comp.set("meshdir", assets_rel.replace("\\", "/"))
    # 1 ms physics: 5 substeps per 5 ms control period (ADR-007-03, asserted at configure).
    ET.SubElement(mj, "option", timestep="0.001", integrator="implicitfast")

    asset = ET.SubElement(mj, "asset")
    meshes = overlay["meshes"] if overlay else {}
    for mname in sorted(k for k in meshes if k is not None):
        if meshes[mname]:
            ET.SubElement(asset, "mesh", name=mname, file=meshes[mname])

    def add_geoms(be, link_name):
        geoms = overlay["bodies"].get(link_name, []) if overlay else []
        for g in geoms:
            ge = ET.SubElement(be, "geom", type=g["type"])
            ge.set("pos", fmt(g["pos"])); ge.set("quat", fmt(g["quat"]))
            for a in ("size", "mesh", "friction", "condim", "solref", "solimp",
                                "contype", "conaffinity", "group", "density"):
                if a in g and g[a] is not None:
                    val = g[a]
                    ge.set(a, val if isinstance(val, str) else fmt(val))

    def add_body(be_parent, link_name, is_root):
        l = links[link_name]
        be = ET.SubElement(be_parent, "body", name=link_name)
        if not is_root:
            j = joint_by_child[link_name]
            be.set("pos", fmt(j["origin"]["xyz_m"]))
            be.set("quat", fmt(rpy_to_quat(j["origin"]["rpy_rad"])))
        else:
            ET.SubElement(be, "freejoint", name="root")
        inertia, com = l["inertia"], l["com_m"]
        ie = ET.SubElement(be, "inertial")
        ie.set("pos", fmt(com)); ie.set("mass", f"{l['mass']['cad_kg']:.10g}")
        if inertia["reference_frame"] == "principal":
            ie.set("quat", fmt(rpy_to_quat(inertia["principal_axes_rpy_rad"])))
            ie.set("diaginertia", fmt([inertia["ixx_kg_m2"], inertia["iyy_kg_m2"],
                                       inertia["izz_kg_m2"]]))
        else:
            ie.set("fullinertia", fmt([inertia["ixx_kg_m2"], inertia["iyy_kg_m2"],
                                       inertia["izz_kg_m2"], inertia["ixy_kg_m2"],
                                       inertia["ixz_kg_m2"], inertia["iyz_kg_m2"]]))
        add_geoms(be, link_name)
        if not is_root:
            j = joint_by_child[link_name]
            if j["type"] != "fixed":
                tr = j["transmission"]
                je = ET.SubElement(be, "joint", name=j["name"],
                                   type="hinge" if j["type"] == "revolute" else "slide",
                                   axis=fmt(j["axis"]),
                                   range=f"{j['limits']['hard_stop_lower_rad']:.10g} "
                                         f"{j['limits']['hard_stop_upper_rad']:.10g}")
                je.set("damping", f"{num_or(tr['friction_viscous_nm_s_rad']):.10g}")
                je.set("frictionloss", f"{num_or(tr['friction_coulomb_nm']):.10g}")
                je.set("armature", f"{num_or(tr['armature_kg_m2']):.10g}")
        for cj in kids.get(link_name, []):
            add_body(be, cj["child_link"], False)

    wb = ET.SubElement(mj, "worldbody")
    add_body(wb, model["robot"]["root_link"], True)

    ET.indent(mj)
    ET.ElementTree(mj).write(out_path, xml_declaration=True, encoding="utf-8")

def emit_safety_manifest(model, out_path):
    envelopes = {}
    for j in model["joints"]:
        if j["type"] == "fixed":
            continue
        lim = j["limits"]
        envelopes[j["name"]] = {
            "position_min_rad": lim["software_lower_rad"],
            "position_max_rad": lim["software_upper_rad"],
            "velocity_max_rad_s": lim["velocity_rad_s"],
            "torque_continuous_nm": lim["effort_continuous_nm"],
            "torque_peak_nm": lim["effort_peak_nm"],
            "torque_peak_duration_s": lim["effort_peak_duration_s"],
            "stiffness_max_nm_rad": PROV_KP_MAX,
            "damping_max_nm_s_rad": PROV_KD_MAX,
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

def generate(src, overlay_path, out_dir, assets_dir):
    model, overlay = load(src, overlay_path)
    os.makedirs(out_dir, exist_ok=True)
    # Use canonical relative path from model/generated to assets_dir to maintain deterministic diffs
    assets_rel = os.path.relpath(assets_dir, "model/generated").replace("\\", "/")
    emit_urdf(model, os.path.join(out_dir, "robot.urdf"), assets_rel)
    emit_mjcf(model, overlay, os.path.join(out_dir, "robot.mjcf"), assets_rel)
    emit_safety_manifest(model, os.path.join(out_dir, "safety_manifest.yaml"))
    print(f"generated robot.urdf, robot.mjcf, safety_manifest.yaml in {out_dir}")