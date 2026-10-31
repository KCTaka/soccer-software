"""Schema validation + the structural checks JSON Schema cannot express
(model-gate-0 §5.9 check IDs)."""
import math
import sys

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

def _principal_minors(m):
    """The principal minors of a symmetric 3x3: the three 1x1, the three 2x2, the determinant."""
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] ** 2)
           - m[0][1] * (m[0][1] * m[2][2] - m[1][2] * m[0][2])
           + m[0][2] * (m[0][1] * m[1][2] - m[1][1] * m[0][2]))
    return ([m[k][k] for k in range(3)],
            [m[a][a] * m[b][b] - m[a][b] ** 2 for a, b in ((0, 1), (0, 2), (1, 2))],
            det)

def structural_checks(model, motor_limits):
    errs, warns = [], []
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
        n = math.sqrt(sum(a * a for a in j["axis"]))
        if abs(n - 1.0) > 1e-9:
            errs.append(f"K-02: joint {j['name']} axis norm {n}")

    # M-01 / M-02: a tensor a rigid body can have. S = tr(I)/2 E - I is the second moment of mass
    # (integral of r r^T dm), so I must be positive definite (M-01) and S positive semi-definite
    # (M-02); M-02 is the triangle inequality on the principal moments, checked in any frame.
    for l in links.values():
        i = l["inertia"]
        t = [[i["ixx_kg_m2"], i["ixy_kg_m2"], i["ixz_kg_m2"]],
             [i["ixy_kg_m2"], i["iyy_kg_m2"], i["iyz_kg_m2"]],
             [i["ixz_kg_m2"], i["iyz_kg_m2"], i["izz_kg_m2"]]]
        h = (t[0][0] + t[1][1] + t[2][2]) / 2
        d1, d2, d3 = _principal_minors(t)
        if min(d1) <= 0 or min(d2) <= 0 or d3 <= 0:
            errs.append(f"M-01: link {l['name']} inertia is not positive definite")
        # A thin plate sits on the boundary (S singular); allow rounding relative to the tensor.
        d1, d2, d3 = _principal_minors([[(h if a == b else 0.0) - t[a][b] for b in range(3)]
                                        for a in range(3)])
        eps = 1e-9
        if min(d1) < -eps * h or min(d2) < -eps * h ** 2 or d3 < -eps * h ** 3:
            errs.append(f"M-02: link {l['name']} inertia violates the triangle inequality")

    # M-03: masses positive.
    for l in links.values():
        if l["mass_kg"] <= 0:
            errs.append(f"M-03: link {l['name']} non-positive mass")

    # J-01 / J-02.
    for j in joints:
        lim = j["limits"]
        if not (lim["software_lower_rad"] >= lim["hard_stop_lower_rad"] and
                lim["software_upper_rad"] <= lim["hard_stop_upper_rad"]):
            errs.append(f"J-01: joint {j['name']} software limits outside hard stops")
        if lim["effort_continuous_nm"] > lim["effort_peak_nm"]:
            errs.append(f"J-02: joint {j['name']} continuous > peak")

    # A-01 / A-02.
    act_names = {a["name"] for a in model["actuators"]}
    for j in joints:
        if (j["type"] == "fixed") != (j["actuator"] is None):
            errs.append(f"A-01: joint {j['name']} actuator/fixed mismatch")
        if j["actuator"] is not None and j["actuator"] not in act_names:
            errs.append(f"A-01: joint {j['name']} references undeclared actuator")
    seen = set()
    for a in model["actuators"]:
        key = (a["bus_segment"], a["can_node_id"])
        if key in seen:
            errs.append(f"A-02: duplicate can_node_id {key}")
        seen.add(key)

    # A-03 / A-04 / A-05 / A-06: physical actuators with an explicit slot and a motor model from
    # model/motors.yaml (e.g. the bench). PROVISIONAL sim actuators carry neither.
    joint_by_act = {j["actuator"]: j for j in joints if j["actuator"] is not None}
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
        j = joint_by_act.get(a["name"])
        if j is not None:
            lim = j["limits"]
            if lim["velocity_rad_s"] > d["velocity_max_rad_s"]:
                errs.append(f"A-05: joint {j['name']} velocity {lim['velocity_rad_s']} exceeds "
                            f"{a['motor_class']} velocity_max {d['velocity_max_rad_s']}")
            if lim["effort_peak_nm"] > d["torque_max_nm"]:
                errs.append(f"A-05: joint {j['name']} effort_peak {lim['effort_peak_nm']} exceeds "
                            f"{a['motor_class']} torque_max {d['torque_max_nm']}")

    # S-01 / S-02: the IMU sits on a declared link, with a unit orientation.
    imu = model.get("imu")
    if imu is not None:
        if imu["link"] not in links:
            errs.append(f"S-02: imu link {imu['link']} is not declared")
        n = math.sqrt(sum(a * a for a in imu["quat_wxyz"]))
        if abs(n - 1.0) > 1e-9:
            errs.append(f"S-01: imu quaternion norm {n}")

    warns.append("C-01: sole polygon check skipped (provisional model; no sole geometry)")
    return errs, warns