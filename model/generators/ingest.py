"""Ingest MuJoCo Menagerie MJCF -> model/source/robot_model.yaml (+ contact overlay).

Assumptions to verify on first run (parser is defensive, but confirm):
  - one movable joint per body; free joint only on the root body
  - explicit <inertial> on every body (inertiafromgeom data is inadmissible)
  - joint torque limits from actuatorfrcrange, else actuator forcerange
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

import yaml

from .common import quat_to_rpy, sha256_file, vec, geom_size

SCHEMA_VERSION = "1.0.0"
GEOM_EXTRA_ATTRS = ("size", "mesh", "friction", "condim", "solref", "solimp",
                    "contype", "conaffinity", "group", "density")
# Provisional placeholders; flagged in the limitations table, replaced at G3.
PROVISIONAL_VELOCITY_RAD_S = 20.0
PROVISIONAL_TORQUE_NM = 10.0
LIMIT_MARGIN_RAD = 0.02

class Ctx:
    def __init__(self):
        self.links, self.joints, self.actuators = [], [], []
        self.overlay, self.meshes, self.warnings = {}, {}, []
        self.actuator_by_joint = {}
        self.root_link, self.floating_root = None, False

def collect_defaults(root, tag):
    """class name (None = root default) -> merged <tag> attributes."""
    out = {}
    top = root.find("default")
    if top is None:
        return {None: {}}
    def visit(elem, inherited, cname):
        merged = dict(inherited)
        sub = elem.find(tag)
        if sub is not None:
            merged.update(sub.attrib)
        out[cname] = merged
        for child in elem.findall("default"):
            visit(child, merged, child.get("class"))
    visit(top, {}, top.get("class"))
    return out

def resolve_attrs(elem, eff_class, defaults):
    cls = elem.get("class") or eff_class
    base = dict(defaults.get(cls, defaults.get(None, {})))
    base.update(elem.attrib)
    return base

def parse_assets(root, ctx):
    asset = root.find("asset")
    if asset is not None:
        for m in asset.findall("mesh"):
            file_attr = m.get("file")
            # MuJoCo's rule: if name is missing, use filename without directory or extension
            derived_name = os.path.splitext(os.path.basename(file_attr))[0] if file_attr else None
            name = m.get("name") or derived_name
            if name and file_attr:
                ctx.meshes[name] = file_attr

def parse_actuators(root, ctx):
    act = root.find("actuator")
    if act is None:
        return
    for a in act:
        joint = a.get("joint")
        if not joint:
            continue
        fr = a.get("forcerange")
        ctx.actuator_by_joint[joint] = {
            "name": a.get("name") or joint,
            "forcerange": vec(fr, 2, [0, 0]) if fr else None,
            "type": a.tag,
        }

def torque_limit(joint_elem, joint_name, ctx):
    frc = joint_elem.get("actuatorfrcrange") if joint_elem is not None else None
    if frc:
        return max(abs(x) for x in vec(frc, 2, [0, 0]))
    act = ctx.actuator_by_joint.get(joint_name)
    if act and act["forcerange"]:
        return max(abs(x) for x in act["forcerange"])
    ctx.warnings.append(f"{joint_name}: no torque limit found; using {PROVISIONAL_TORQUE_NM} Nm")
    return PROVISIONAL_TORQUE_NM

def make_limits(lo, hi, torque_nm, ctx, fixed=False):
    margin = 0.0 if fixed else min(LIMIT_MARGIN_RAD, (hi - lo) / 4.0)
    return {
        "hard_stop_lower_rad": lo, "hard_stop_upper_rad": hi,
        "software_lower_rad": lo + margin, "software_upper_rad": hi - margin,
        "velocity_rad_s": 1.0 if fixed else PROVISIONAL_VELOCITY_RAD_S,
        "velocity_limit_origin": "drive",
        "effort_continuous_nm": torque_nm,  # optimistic: no thermal data (NR)
        "effort_peak_nm": torque_nm,
        "effort_peak_duration_s": 1.0,
    }

def make_transmission(attrs):
    return {
        "gear_ratio": 1.0,          # provisional: sim plant is the ground truth
        "direction_sign": 1,        # provisional: axis defines positive
        "backlash_rad": "NR",
        "stiffness_nm_rad": "NR",
        "friction_coulomb_nm": float(attrs.get("frictionloss", 0.0)),
        "friction_viscous_nm_s_rad": float(attrs.get("damping", 0.0)),
    }

def walk_body(body, parent_link, eff_class, angle, ctx, joint_defaults, geom_defaults):
    name = body.get("name") or f"anon_body_{len(ctx.links)}"
    if body.get("childclass"):
        eff_class = body.get("childclass")
    pos = vec(body.get("pos"), 3, [0, 0, 0])
    quat = vec(body.get("quat"), 4, [1, 0, 0, 0])

    joints = body.findall("joint")
    free = [j for j in joints if j.get("type") == "free"]
    movable = [j for j in joints if j.get("type", "hinge") in ("hinge", "slide")]
    if len(movable) > 1:
        raise ValueError(f"body {name}: more than one movable joint is unsupported")
    if free:
        if parent_link is not None:
            raise ValueError(f"body {name}: free joint on a non-root body")
        ctx.floating_root = True

    inertial = body.find("inertial")
    if inertial is None:
        raise ValueError(f"body {name}: no explicit <inertial>; "
                         "geometry-derived inertia is inadmissible (inertiafromgeom=false)")
    mass = float(inertial.get("mass"))
    com = vec(inertial.get("pos"), 3, [0, 0, 0])
    iquat = vec(inertial.get("quat"), 4, [1, 0, 0, 0])
    diag, full = inertial.get("diaginertia"), inertial.get("fullinertia")
    if diag:
        d = vec(diag, 3, None)
        inertia = {"reference_point": "com", "reference_frame": "principal",
                   "principal_axes_rpy_rad": quat_to_rpy(iquat),
                   "ixx_kg_m2": d[0], "iyy_kg_m2": d[1], "izz_kg_m2": d[2],
                   "ixy_kg_m2": 0.0, "ixz_kg_m2": 0.0, "iyz_kg_m2": 0.0}
    elif full:
        ixx, iyy, izz, ixy, ixz, iyz = vec(full, 6, None)
        inertia = {"reference_point": "com", "reference_frame": "link",
                   "principal_axes_rpy_rad": None,
                   "ixx_kg_m2": ixx, "iyy_kg_m2": iyy, "izz_kg_m2": izz,
                   "ixy_kg_m2": ixy, "ixz_kg_m2": ixz, "iyz_kg_m2": iyz}
    else:
        raise ValueError(f"body {name}: inertial has neither diaginertia nor fullinertia")

    # Geoms: schema-conformant subset into collision; full attributes into the overlay.
    prims, mesh_name, overlay_geoms = [], None, []
    for g in body.findall("geom"):
        attrs = resolve_attrs(g, eff_class, geom_defaults)
        gt = attrs.get("type", "sphere")
        gpos = vec(attrs.get("pos"), 3, [0, 0, 0])
        gquat = vec(attrs.get("quat"), 4, [1, 0, 0, 0])
        o = {"type": gt, "pos": gpos, "quat": gquat}
        for a in GEOM_EXTRA_ATTRS:
            if a in attrs and attrs[a] is not None:
                o[a] = attrs[a]
        overlay_geoms.append(o)
        if gt == "mesh":
            if mesh_name is None:
                mesh_name = g.get("mesh")
            else:
                ctx.warnings.append(f"{name}: multiple mesh geoms; only first kept in schema record")
        elif gt in ("box", "cylinder", "sphere", "capsule"):
            raw_sz = [float(x) for x in (g.get("size") or "").split()]
            # Pad size to 3 elements (sphere has 1, capsule/cylinder have 2, box has 3)
            dim = (raw_sz + [0.0] * 3)[:3]
        elif gt in ("box", "cylinder", "sphere", "capsule"):
            prims.append({"shape": gt,
                "frame": {"xyz_m": gpos, "rpy_rad": quat_to_rpy(gquat)},
                "dimensions_m": geom_size(attrs.get("size"))})
    ctx.overlay[name] = overlay_geoms

    # Joint (if any) attaches this body to its parent.
    joint_obj, joint_name = None, None
    if parent_link is not None:
        if movable:
            j = movable[0]
            joint_name = j.get("name") or f"{name}_joint"
            attrs = resolve_attrs(j, eff_class, joint_defaults)
            axis = vec(j.get("axis"), 3, [0, 0, 1])
            n = sum(a * a for a in axis) ** 0.5
            axis = [a / n for a in axis]
            rng = j.get("range")
            lo, hi = (vec(rng, 2, [0, 0])[0] * angle, vec(rng, 2, [0, 0])[1] * angle) if rng else (0.0, 0.0)
            torque = torque_limit(j, joint_name, ctx)
            joint_obj = {
                "name": joint_name,
                "type": "revolute" if j.get("type", "hinge") == "hinge" else "prismatic",
                "parent_link": parent_link, "child_link": name,
                "origin": {"xyz_m": pos, "rpy_rad": quat_to_rpy(quat)},
                "axis": axis,
                "positive_direction": {
                    "description": (f"Provisional: positive follows the MJCF right-hand rule about "
                                    f"axis {axis} in link {name}. Not physically verified."),
                    "photo": "PROVISIONAL", "verified_by": None},
                "limits": make_limits(lo, hi, torque, ctx),
                "transmission": make_transmission(attrs),
                "calibration": {"zero_offset_rad": None, "zero_pose_value_rad": 0.0,
                                "measured_at": None},
                "actuator": None,  # filled after actuator entries exist
            }
        else:
            joint_name = f"{name}_fixed"
            joint_obj = {
                "name": joint_name, "type": "fixed",
                "parent_link": parent_link, "child_link": name,
                "origin": {"xyz_m": pos, "rpy_rad": quat_to_rpy(quat)},
                "axis": [0.0, 0.0, 1.0],
                "positive_direction": {"description": "Fixed joint; no motion axis.",
                                       "photo": "PROVISIONAL", "verified_by": None},
                "limits": make_limits(0.0, 0.0, PROVISIONAL_TORQUE_NM, ctx, fixed=True),
                "transmission": make_transmission({}),
                "calibration": {"zero_offset_rad": None, "zero_pose_value_rad": 0.0,
                                "measured_at": None},
                "actuator": None,
            }
    if joint_obj is not None:
        ctx.joints.append(joint_obj)

    provenance = ctx.provenance
    ctx.links.append({
        "name": name,
        "parent_joint": joint_name,
        "mass": {"cad_kg": mass, "measured_kg": None, "provenance": provenance},
        "com_m": com,
        "inertia": dict(inertia, provenance=provenance),
        "visual_mesh": mesh_name,
        "collision": {"primitives": prims, "mesh": mesh_name},
    })
    if parent_link is None:
        ctx.root_link = name
    for child in body.findall("body"):
        walk_body(child, name, eff_class, angle, ctx, joint_defaults, geom_defaults)

def ingest(config_path, out_path, overlay_out_path):
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    mjcf_path = cfg["source"]["mjcf"]
    root = ET.parse(mjcf_path).getroot()

    compiler = root.find("compiler")
    angle = 3.141592653589793 / 180.0 if (compiler is not None and
            compiler.get("angle", "radian") == "degree") else 1.0

    ctx = Ctx()
    src_ref = f"{cfg['source']['repository']}@{cfg['source']['commit']}:{mjcf_path}"
    ctx.provenance = {"cad_system": "derived", "document_id": src_ref,
                      "version_or_configuration": cfg["source"]["commit"],
                      "exported_at": cfg["exported_at"], "exported_by": cfg["exported_by"]}

    parse_assets(root, ctx)
    parse_actuators(root, ctx)
    joint_defaults = collect_defaults(root, "joint")
    geom_defaults = collect_defaults(root, "geom")

    world = root.find("worldbody")
    bodies = world.findall("body")
    if len(bodies) != 1:
        raise ValueError("expected exactly one root body in worldbody")
    walk_body(bodies[0], None, None, angle, ctx, joint_defaults, geom_defaults)

    # Actuators: one PROVISIONAL entry per actuated joint (schema A-01).
    joint_names = [j["name"] for j in ctx.joints]
    for j in ctx.joints:
        if j["type"] == "fixed":
            continue
        act = ctx.actuator_by_joint.get(j["name"])
        act_name = (act["name"] if act else f"{j['name']}_act").lower().replace("-", "_")
        nr_enc = {"bits": "NR", "min": "NR", "max": "NR", "endianness": "NR"}
        ctx.actuators.append({
            "name": act_name, "motor_class": "RS00",
            "bus_segment": "sim0", "can_node_id": len(ctx.actuators),
            "mit_encoding": {
                "verified_by": "unverified",
                "position": dict(nr_enc), "velocity": dict(nr_enc), "effort": dict(nr_enc),
                "stiffness": dict(nr_enc), "damping": dict(nr_enc),
                "frames_per_cycle_command": "NR", "frames_per_cycle_feedback": "NR",
                "dlc_command_bytes": "NR", "dlc_feedback_bytes": "NR"},
            "thermal": {"derating_curve": "NR", "measured": False},
        })
        j["actuator"] = act_name

    digests = {mjcf_path: sha256_file(mjcf_path)}
    assets_dir = cfg["source"]["assets_dir"]
    for mesh_file in sorted(ctx.meshes.values()):
        p = os.path.join(assets_dir, mesh_file)
        if os.path.exists(p):
            digests[p] = sha256_file(p)

    model = {
        "schema_version": SCHEMA_VERSION,
        "robot": {"name": cfg["model_name"], "size_class": "kidsize",
                  "root_link": ctx.root_link, "total_mass_measured_kg": None,
                  "zero_pose_photo": "PROVISIONAL"},
        "provenance": {"generated_from": [src_ref],
                       "ingest_tool_version": cfg["ingest_tool_version"],
                       "source_digests": digests},
        "links": ctx.links,
        "joints": ctx.joints,
        "actuators": ctx.actuators,
        "closed_loops": [],
        "emission": {
            "urdf": {"enabled": True, "mode": "full_tree", "excluded_links": []},
            "mjcf": {"enabled": True},
            "sdf": {"enabled": False},
            "usd": {"enabled": False}},
        "simulation": {"mjcf_compiler": {
            "balanceinertia": False, "boundmass": 0, "boundinertia": 0,
            "settotalmass": -1, "inertiafromgeom": "false",
            "strippath": False, "angle": "radian",
            "fusestatic": "false", "discardvisual": False}},
    }
    with open(out_path, "w") as f:
        yaml.safe_dump(model, f, sort_keys=False, width=1000, allow_unicode=True)

    overlay = {"generated_from": [src_ref],
               "source_digest": digests[mjcf_path],
               "meshes": ctx.meshes,
               "bodies": ctx.overlay}
    with open(overlay_out_path, "w") as f:
        yaml.safe_dump(overlay, f, sort_keys=False, width=1000, allow_unicode=True)

    for w in ctx.warnings:
        print(f"WARN: {w}", file=sys.stderr)
    if not ctx.floating_root:
        print("WARN: source has no free joint on root; emitter will add one", file=sys.stderr)
    print(f"ingested {len(ctx.links)} links, {len(ctx.joints)} joints, "
          f"{len(ctx.actuators)} actuators -> {out_path}")