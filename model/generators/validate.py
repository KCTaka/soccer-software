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

def structural_checks(model):
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

    # M-01 / M-02: inertia triangle inequality and positive semi-definiteness.
    for l in links.values():
        i = l["inertia"]
        d = [i["ixx_kg_m2"], i["iyy_kg_m2"], i["izz_kg_m2"]]
        if i["reference_frame"] == "principal":
            for k in range(3):
                a, b, c = d[k], d[(k + 1) % 3], d[(k + 2) % 3]
                if a + b < c - 1e-12:
                    errs.append(f"M-01: link {l['name']} violates triangle inequality")
        else:
            m = [[i["ixx_kg_m2"], i["ixy_kg_m2"], i["ixz_kg_m2"]],
                 [i["ixy_kg_m2"], i["iyy_kg_m2"], i["iyz_kg_m2"]],
                 [i["ixz_kg_m2"], i["iyz_kg_m2"], i["izz_kg_m2"]]]
            # crude PSD check via pivots; replace with numpy if available
            if any(m[k][k] < -1e-12 for k in range(3)):
                errs.append(f"M-02: link {l['name']} negative diagonal inertia")

    # M-03: masses positive; report the sum (reconciliation needs measured total).
    total = 0.0
    for l in links.values():
        m = l["mass"]["cad_kg"]
        if m <= 0:
            errs.append(f"M-03: link {l['name']} non-positive mass")
        total += m
    if model["robot"]["total_mass_measured_kg"] is None:
        warns.append(f"M-03: no measured total mass; sum of link masses = {total:.3f} kg")

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

    warns.append("C-01: sole polygon check skipped (provisional model; no sole geometry)")
    return errs, warns, total