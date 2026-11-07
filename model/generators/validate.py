"""Schema validation + the structural checks JSON Schema cannot express
(model-gate-0 §5.9 check IDs)."""
import sys

import numpy as np

from . import linkage

def schema_validate(model, schema_path):
    try:
        import jsonschema
    except ImportError:
        raise RuntimeError("jsonschema is required; refusing to skip schema validation")
    import json
    with open(schema_path) as f:
        schema = json.load(f)
    v = jsonschema.Draft202012Validator(schema)
    return [f"schema: {e.json_path}: {e.message}" for e in v.iter_errors(model)]

def structural_checks(model, motor_limits):
    errs = []
    links = {l["name"]: l for l in model["links"]}
    joints = model["joints"]
    joints_by_name = {j["name"]: j for j in joints}

    roots = [l for l in links.values() if l["parent_joint"] is None]
    if len(roots) != 1:
        errs.append(f"K-01: expected exactly one root link, found {len(roots)}")

    # K-03 / K-04: references resolve, tree is acyclic and connected.
    parent_of = {}
    for j in joints:
        if j["parent_link"] not in links or j["child_link"] not in links:
            errs.append(f"K-03: joint {j['name']} references an undeclared link")
        parent_of[j["child_link"]] = j["parent_link"]
    for l in links.values():
        if l["parent_joint"] is not None and l["parent_joint"] not in joints_by_name:
            errs.append(f"K-03: link {l['name']} parent_joint not declared")
    for name in links:
        seen, cur = set(), name
        while cur in parent_of:
            if cur in seen:
                errs.append(f"K-04: cycle at link {cur}")
                break
            seen.add(cur)
            cur = parent_of[cur]

    # K-02: joint axes are unit vectors.
    for j in joints:
        n = np.linalg.norm(j["axis"])
        if abs(n - 1.0) > 1e-9:
            errs.append(f"K-02: joint {j['name']} axis norm {n}")

    # M-01 / M-02: a tensor a rigid body can have. S = tr(I)/2 E - I is the second moment of mass
    # (integral of r r^T dm), so I must be positive definite (M-01) and S positive semi-definite
    # (M-02). S's eigenvalues are tr(I)/2 - lambda_i, so M-02 is the triangle inequality on the
    # principal moments lambda_i: none exceeds the sum of the other two.
    for l in links.values():
        i = l["inertia"]
        moments = np.linalg.eigvalsh([[i["ixx_kg_m2"], i["ixy_kg_m2"], i["ixz_kg_m2"]],
                                      [i["ixy_kg_m2"], i["iyy_kg_m2"], i["iyz_kg_m2"]],
                                      [i["ixz_kg_m2"], i["iyz_kg_m2"], i["izz_kg_m2"]]])
        if moments.min() <= 0:
            errs.append(f"M-01: link {l['name']} inertia is not positive definite")
        # A thin plate sits on the boundary (S singular); allow rounding relative to the tensor.
        h = moments.sum() / 2
        if moments.max() - h > 1e-9 * h:
            errs.append(f"M-02: link {l['name']} inertia violates the triangle inequality")

    # M-03: masses positive.
    for l in links.values():
        if l["mass_kg"] <= 0:
            errs.append(f"M-03: link {l['name']} non-positive mass")

    # J-01 / J-02.
    for j in joints:
        lim = j["limits"]
        if lim is None:
            continue
        if not (lim["software_lower_rad"] >= lim["hard_stop_lower_rad"] and
                lim["software_upper_rad"] <= lim["hard_stop_upper_rad"]):
            errs.append(f"J-01: joint {j['name']} software limits outside hard stops")
        if lim["effort_continuous_nm"] > lim["effort_peak_nm"]:
            errs.append(f"J-02: joint {j['name']} continuous > peak")

    # A-01 / A-02. A joint has an actuator unless it is fixed or a linkage moves it (its output and
    # coupler; the linkage's input carries the motor).
    act_names = {a["name"] for a in model["actuators"]}
    linkages = model.get("linkages", [])
    passive = {lk[k] for lk in linkages for k in ("output_joint", "coupler_joint")}
    for j in joints:
        if (j["type"] != "fixed" and j["name"] not in passive) != (j["actuator"] is not None):
            errs.append(f"A-01: joint {j['name']} must have an actuator exactly when it is neither "
                        "fixed nor a linkage's output or coupler")
        if j["actuator"] is not None and j["actuator"] not in act_names:
            errs.append(f"A-01: joint {j['name']} references undeclared actuator")
    seen = set()
    for a in model["actuators"]:
        key = (a["bus_segment"], a["can_node_id"])
        if key in seen:
            errs.append(f"A-02: duplicate can_node_id {key}")
        seen.add(key)

    # L-01 to L-05: linkages.
    roles = {}
    for lk in linkages:
        for role in ("input_joint", "output_joint", "coupler_joint"):
            if lk[role] in roles:
                errs.append(f"L-01: joint {lk[role]} is a member of {roles[lk[role]]} and "
                            f"{lk['name']}")
            roles[lk[role]] = lk["name"]
        params, geometry_errs = linkage.planar_parameters(model, lk)
        errs += geometry_errs   # L-02 (a planar four-bar) and L-03 (closed at the zero pose)
        members_ok = all(lk[r] in joints_by_name for r in ("input_joint", "output_joint",
                                                           "coupler_joint"))
        if not members_ok:
            continue
        j_in, j_out, j_cpl = (joints_by_name[lk[r]] for r in ("input_joint", "output_joint",
                                                               "coupler_joint"))
        if j_in["limits"] is not None or j_cpl["limits"] is not None or j_out["limits"] is None:
            errs.append(f"L-04: {lk['name']}: the output joint has limits; the input and coupler "
                        "joints have none (the linkage derives them)")
        if params is not None and not geometry_errs and j_out["limits"] is not None:
            lim = j_out["limits"]
            if linkage.FourBar(params).sweep(lim["hard_stop_lower_rad"],
                                             lim["hard_stop_upper_rad"]) is None:
                errs.append(f"L-05: {lk['name']} does not close, or passes a toggle, within "
                            f"{j_out['name']}'s hard stops")
        # L-06: the motor's reflected rotor inertia. Every motor has one, and MuJoCo needs it: its
        # implicit integrator applies the MIT damping through the input's mass matrix but computes
        # the closure force without it, so on a light crank with no armature the pin stretches by
        # millimetres (docs/leg-prototype.md).
        if j_in["transmission"]["armature_kg_m2"] <= 0:
            errs.append(f"L-06: {lk['name']}: input joint {j_in['name']} needs its motor's "
                        "reflected rotor inertia as a positive armature")
    for j in joints:
        if j["limits"] is None and j["name"] not in {lk[k] for lk in linkages
                                                     for k in ("input_joint", "coupler_joint")}:
            errs.append(f"L-04: joint {j['name']} has no limits but is not a linkage's input or "
                        "coupler")

    # D-01: a motor's mounting sign is set for every joint a motor turns, and only for those.
    for j in joints:
        sign = j["transmission"]["direction_sign"]
        if j["actuator"] is not None and sign is None:
            errs.append(f"D-01: joint {j['name']} has an actuator but no direction_sign")
        if j["name"] in passive and sign is not None:
            errs.append(f"D-01: joint {j['name']} is moved by a linkage, so its direction_sign "
                        "must be null")

    # A-03 / A-04 / A-05 / A-06: physical actuators with an explicit slot and a motor model from
    # model/motors.yaml (e.g. the bench). PROVISIONAL sim actuators carry neither. A-05 compares
    # the actuated joint's bounds, which a linkage derives from its output's.
    try:
        drive_by_act = {d["actuator"]: d for d in linkage.drives(model)}
    except (ValueError, KeyError, TypeError):
        drive_by_act = {}   # reported above as L-0x
    slots_by_segment = {}
    for a in model["actuators"]:
        if "slot" in a:
            slots_by_segment.setdefault(a["bus_segment"], []).append(a["slot"])
    for seg, slots in slots_by_segment.items():
        if len(set(slots)) != len(slots):
            errs.append(f"A-03: segment {seg} has duplicate slots {sorted(slots)}")
        if sorted(slots) != list(range(len(slots))):
            errs.append(f"A-03: segment {seg} slots {sorted(slots)} are not contiguous 0..n-1")
    for a in model["actuators"]:
        d = motor_limits.get(a["motor_class"])
        if d is None:
            if a["motor_class"] != "PROVISIONAL":
                errs.append(f"A-06: actuator {a['name']} motor_class {a['motor_class']} is not in "
                            "model/motors.yaml")
            continue
        if a.get("default_kp", 0.0) > d["kp_max"]:
            errs.append(f"A-04: actuator {a['name']} default_kp {a['default_kp']} > kp_max "
                        f"{d['kp_max']}")
        if a.get("default_kd", 0.0) > d["kd_max"]:
            errs.append(f"A-04: actuator {a['name']} default_kd {a['default_kd']} > kd_max "
                        f"{d['kd_max']}")
        drive = drive_by_act.get(a["name"])
        if drive is not None:
            lim, name = drive["limits"], drive["actuated_joint"]
            if lim["velocity_rad_s"] > d["velocity_max_rad_s"]:
                errs.append(f"A-05: joint {name} velocity {lim['velocity_rad_s']} exceeds "
                            f"{a['motor_class']} velocity_max {d['velocity_max_rad_s']}")
            if lim["effort_peak_nm"] > d["torque_max_nm"]:
                errs.append(f"A-05: joint {name} effort_peak {lim['effort_peak_nm']} exceeds "
                            f"{a['motor_class']} torque_max {d['torque_max_nm']}")

    # H-01: the home pose names every commanded joint, each within its software limits, so the
    # SafetyKernel accepts the pose the robot starts in.
    home = model.get("home")
    if home is not None:
        movable = {j["name"]: j for j in linkage.commanded_joints(model)}
        for name in sorted(set(movable) - set(home)):
            errs.append(f"H-01: home pose lacks joint {name}")
        for name, angle in home.items():
            j = movable.get(name)
            if j is None:
                errs.append(f"H-01: home pose names {name}, which is not a commanded joint")
            elif not (j["limits"]["software_lower_rad"] <= angle <=
                      j["limits"]["software_upper_rad"]):
                errs.append(f"H-01: home angle {angle} for {name} is outside its software limits")

    # S-01 / S-02: the IMU sits on a declared link, with a unit orientation.
    imu = model.get("imu")
    if imu is not None:
        if imu["link"] not in links:
            errs.append(f"S-02: imu link {imu['link']} is not declared")
        n = np.linalg.norm(imu["quat_wxyz"])
        if abs(n - 1.0) > 1e-9:
            errs.append(f"S-01: imu quaternion norm {n}")

    return errs