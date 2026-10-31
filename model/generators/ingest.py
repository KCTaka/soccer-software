"""Import a MuJoCo Menagerie MJCF into a robot_model.yaml (+ contact overlay).

  python3 -m model.generators.ingest CONFIG MODEL_OUT OVERLAY_OUT

A one-off conversion tool, not part of `check`: its output is a starting point, and from then on
the written robot_model.yaml is the hand-edited source of truth. Re-running it overwrites any hand
edits, so diff before committing.

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

from .common import quat_to_matrix, quat_to_rpy, vec, geom_size

SCHEMA_VERSION = "1.0.0"
GEOM_EXTRA_ATTRS = ("size", "mesh", "friction", "condim", "solref", "solimp",
                    "contype", "conaffinity", "group", "density")
# Provisional placeholders; flagged in the limitations table, replaced at G3.
PROVISIONAL_VELOCITY_RAD_S = 20.0
PROVISIONAL_TORQUE_NM = 10.0
LIMIT_MARGIN_RAD = 0.02
MIRRORED_TRANSMISSION_JOINTS = {
    "right_hip_roll_joint",
    "right_ankle_roll_joint",
    "right_shoulder_roll_joint",
}

class Ctx:
    def __init__(self):
        self.links, self.joints, self.actuators = [], [], []
        self.overlay, self.meshes, self.warnings = {}, {}, []
        self.actuator_by_joint = {}
        self.root_link, self.floating_root = None, False
        self.imu_site, self.imu = None, None

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
        "effort_continuous_nm": torque_nm,  # optimistic: no thermal data
        "effort_peak_nm": torque_nm,
        "effort_peak_duration_s": 1.0,
    }

def make_transmission(attrs, joint_name=None):
    return {
        "direction_sign": -1 if joint_name in MIRRORED_TRANSMISSION_JOINTS else 1,
        "friction_coulomb_nm": float(attrs.get("frictionloss", 0.0)),
        "friction_viscous_nm_s_rad": float(attrs.get("damping", 0.0)),
        "armature_kg_m2": float(attrs.get("armature", 0.0)),
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
        # Principal moments in a frame rotated by `quat`; the model keeps link-frame axes:
        # I = R diag(d) R^T.
        d = vec(diag, 3, None)
        r = quat_to_matrix(iquat)
        t = [[sum(r[a][k] * d[k] * r[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
        inertia = {"ixx_kg_m2": t[0][0], "iyy_kg_m2": t[1][1], "izz_kg_m2": t[2][2],
                   "ixy_kg_m2": t[0][1], "ixz_kg_m2": t[0][2], "iyz_kg_m2": t[1][2]}
    elif full:
        ixx, iyy, izz, ixy, ixz, iyz = vec(full, 6, None)
        inertia = {"ixx_kg_m2": ixx, "iyy_kg_m2": iyy, "izz_kg_m2": izz,
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
            prims.append({"shape": gt,
                "frame": {"xyz_m": gpos, "rpy_rad": quat_to_rpy(gquat)},
                "dimensions_m": geom_size(attrs.get("size"))})
    ctx.overlay[name] = overlay_geoms

    for site in body.findall("site"):
        if ctx.imu_site is not None and site.get("name") == ctx.imu_site:
            ctx.imu = {"name": "imu", "link": name,
                       "pos_m": vec(site.get("pos"), 3, [0, 0, 0]),
                       "quat_wxyz": vec(site.get("quat"), 4, [1, 0, 0, 0])}

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
                "limits": make_limits(lo, hi, torque, ctx),
                "transmission": make_transmission(attrs, joint_name),
                "calibration": {"zero_offset_rad": None},
                "actuator": None,  # filled after actuator entries exist
            }
        else:
            joint_name = f"{name}_fixed"
            joint_obj = {
                "name": joint_name, "type": "fixed",
                "parent_link": parent_link, "child_link": name,
                "origin": {"xyz_m": pos, "rpy_rad": quat_to_rpy(quat)},
                "axis": [0.0, 0.0, 1.0],
                "limits": make_limits(0.0, 0.0, PROVISIONAL_TORQUE_NM, ctx, fixed=True),
                "transmission": make_transmission({}),
                "calibration": {"zero_offset_rad": None},
                "actuator": None,
            }
    if joint_obj is not None:
        ctx.joints.append(joint_obj)

    ctx.links.append({
        "name": name,
        "parent_joint": joint_name,
        "mass_kg": mass,
        "com_m": com,
        "inertia": inertia,
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
    # Where the file came from, as a comment: nothing downstream reads it.
    header = (f"# Imported from {cfg['source']['repository']}@{cfg['source']['commit']}:"
              f"{mjcf_path}\n# by `python3 -m model.generators.ingest`. Edit this file directly; "
              "re-importing overwrites edits.\n")

    ctx.imu_site = cfg["source"].get("imu_site")
    parse_assets(root, ctx)
    parse_actuators(root, ctx)
    joint_defaults = collect_defaults(root, "joint")
    geom_defaults = collect_defaults(root, "geom")

    world = root.find("worldbody")
    bodies = world.findall("body")
    if len(bodies) != 1:
        raise ValueError("expected exactly one root body in worldbody")
    walk_body(bodies[0], None, None, angle, ctx, joint_defaults, geom_defaults)

    if ctx.imu_site is not None and ctx.imu is None:
        raise ValueError(f"imu_site '{ctx.imu_site}' not found in {mjcf_path}")

    # Actuators: one PROVISIONAL entry per actuated joint (schema A-01).
    joint_names = [j["name"] for j in ctx.joints]
    for j in ctx.joints:
        if j["type"] == "fixed":
            continue
        act = ctx.actuator_by_joint.get(j["name"])
        act_name = (act["name"] if act else f"{j['name']}_act").lower().replace("-", "_")
        ctx.actuators.append({
            "name": act_name, "motor_class": "PROVISIONAL",
            "bus_segment": "sim0", "can_node_id": len(ctx.actuators),
        })
        j["actuator"] = act_name

    model = {
        "schema_version": SCHEMA_VERSION,
        "robot": {"name": cfg["model_name"], "root_link": ctx.root_link},
        "links": ctx.links,
        "joints": ctx.joints,
        "actuators": ctx.actuators,
    }
    if ctx.imu is not None:
        model["imu"] = ctx.imu
    with open(out_path, "w") as f:
        f.write(header)
        yaml.safe_dump(model, f, sort_keys=False, width=1000, allow_unicode=True)

    overlay = {"meshes": ctx.meshes, "bodies": ctx.overlay}
    with open(overlay_out_path, "w") as f:
        f.write(header)
        yaml.safe_dump(overlay, f, sort_keys=False, width=1000, allow_unicode=True)

    for w in ctx.warnings:
        print(f"WARN: {w}", file=sys.stderr)
    if not ctx.floating_root:
        print("WARN: source has no free joint on root; emitter will add one", file=sys.stderr)
    print(f"ingested {len(ctx.links)} links, {len(ctx.joints)} joints, "
          f"{len(ctx.actuators)} actuators -> {out_path}")

def main():
    p = argparse.ArgumentParser(description="Import a Menagerie MJCF into a robot_model.yaml.")
    p.add_argument("config", help="import config, e.g. model/robots/unitree_g1/source/ingest.yaml")
    p.add_argument("model_out", help="robot_model.yaml to write")
    p.add_argument("overlay_out", help="contact overlay to write")
    args = p.parse_args()
    ingest(args.config, args.model_out, args.overlay_out)

if __name__ == "__main__":
    main()
