"""Import a Fusion 360 export (the acdc4robot add-in's MJCF and STL meshes) into a robot_model.yaml.

  python3 -m model.generators.ingest_fusion CONFIG

A tool for a CAD (re-)export, not part of `check`. It writes only what the CAD owns: link frames,
joint origins and axes, inertials, meshes, collision primitives and the linkages' closure pins.
Every other field (limits, transmission, calibration, actuator, the actuators, home, imu) is kept
from the robot_model.yaml it replaces, so hand edits survive a re-export; a joint the old file
lacks is written without them and fails the schema until they are filled in. Diff before
committing.

The export is a kinematic tree rooted wherever the CAD assembly was grounded, posed however the CAD
was left, with every closed loop cut open. The import, in order:

 1. Compiles the export with MuJoCo for every body's pose and inertial at the export pose.
 2. Closes each linkage: with the output joint at the export pose's angle, solves the input and
    coupler joints for the closure pin's two points to coincide. That closed configuration is the
    model's zero pose, so MJCF's equality constraint holds at qpos0 and every joint's zero means
    the same thing in the model, the URDF and the hardware.
 3. Re-roots the tree at `root`, the part that is bolted down, reversing the joints between it and
    the export's root.
 4. Gives every link a frame on its parent joint's axis with the robot's axes (REP-103: x forward,
    y left, z up), so every joint origin has rpy 0. The root's frame sits on `root_frame_at_joint`.
 5. Writes each mesh in metres in its link frame, and transforms inertials, collision primitives
    and closure points into the link frames. With `density_kg_m3` set, each part's inertial is
    computed from its mesh at that density, and a purchased part's placeholder shell takes the
    part's catalogue mass (see mass_properties); without it, the inertials are Fusion's.
"""
import argparse
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from .common import matrix_to_rpy

SCHEMA_VERSION = "1.0.0"
# Fields of an existing robot_model.yaml that the CAD does not own and a re-import keeps.
KEPT_JOINT_FIELDS = ("limits", "transmission", "calibration", "actuator")
KEPT_TOP_LEVEL = ("actuators", "home", "imu")
# Linear positions are rounded to 0.1 nm and unit vectors to 1e-12: far below any physical meaning,
# and it keeps CAD float noise (0.040000000000000036) out of the hand-edited file.
POSITION_DECIMALS = 10
DIRECTION_DECIMALS = 12

def read_stl(path):
    """Triangles (n, 3, 3) of a binary STL, in its file units."""
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype(
        [("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].astype(float)

def write_stl(path, tri):
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, length, out=np.zeros_like(normals), where=length > 0)
    rec = np.zeros(len(tri), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)),
                                             ("a", "<u2")]))
    rec["n"], rec["v"] = normals, tri
    with open(path, "wb") as f:
        f.write(b"robot_model mesh, metres, link frame".ljust(80, b" "))
        f.write(struct.pack("<I", len(tri)))
        f.write(rec.tobytes())

def shells(tri):
    """The connected shells of a triangle soup: a list of (vertices, faces)."""
    v, inv = np.unique(np.round(tri.reshape(-1, 3), 9), axis=0, return_inverse=True)
    f = inv.reshape(-1, 3)
    rows = np.concatenate([f[:, 0], f[:, 1], f[:, 2]])
    cols = np.concatenate([f[:, 1], f[:, 2], f[:, 0]])
    count, label = connected_components(
        coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(len(v), len(v))), directed=False)
    return [(v, f[label[f[:, 0]] == k]) for k in range(count)]

def volume_moments(v, f):
    """Volume, first moment and second moment (integral of r r^T dV) of a closed mesh, exactly,
    by signed tetrahedra from the origin."""
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    det = np.einsum("ij,ij->i", a, np.cross(b, c))
    second = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            second[i, j] = (det * (2 * (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j])
                                   + a[:, i] * b[:, j] + a[:, j] * b[:, i]
                                   + a[:, i] * c[:, j] + a[:, j] * c[:, i]
                                   + b[:, i] * c[:, j] + b[:, j] * c[:, i])).sum() / 120
    return det.sum() / 6, (det[:, None] * (a + b + c)).sum(0) / 24, second

def inertia_about_origin(second):
    return np.trace(second) * np.eye(3) - second

def mass_properties(tri, density, purchased):
    """Mass, COM and inertia about the COM of a body, in its frame, from its mesh `tri` (m): every
    shell solid at `density` (kg/m^3), except each purchased part's shell, the one whose bounding
    box holds its placeholder point, which takes the part's catalogue mass over its volume. Exact for
    the mesh: each closed shell's volume integrals, by signed tetrahedra."""
    body_shells = shells(tri)
    boxes = [(v[np.unique(f)].min(0), v[np.unique(f)].max(0)) for v, f in body_shells]
    part_of = {}
    for part in purchased:
        point = np.asarray(part["placeholder_point_m"], float)
        inside = [k for k, (lo, hi) in enumerate(boxes)
                  if np.all(lo <= point) and np.all(point <= hi)]
        if len(inside) != 1:
            raise ValueError(f"placeholder point {point.tolist()} lies in the bounding box of "
                             f"{len(inside)} shells; it must pick exactly one")
        part_of[inside[0]] = part
    mass, first_moment, inertia_o = 0.0, np.zeros(3), np.zeros((3, 3))
    for k, (v, f) in enumerate(body_shells):
        vol, first, second = volume_moments(v, f)
        if vol <= 0:
            raise ValueError(f"shell {k} has volume {vol}: not a closed, outward-facing mesh")
        rho = part_of[k]["mass_kg"] / vol if k in part_of else density
        mass += rho * vol
        first_moment += rho * first
        inertia_o += rho * inertia_about_origin(second)
    com = first_moment / mass
    return mass, com, inertia_o - mass * (com @ com * np.eye(3) - np.outer(com, com))

def primitive_in_link(shape, dims, r_lb):
    """(rpy, dimensions) of a primitive whose axes are its body's, in link axes `r_lb`. A box is
    symmetric under any signed permutation of its axes, and a cylinder under one that keeps its
    axis (z) on a link axis, so an axis-aligned part gets rpy 0 and permuted dimensions rather than
    an rpy of multiples of pi/2 with float noise."""
    snapped = r_lb.copy()
    for target in (0.0, 1.0, -1.0):
        snapped[np.abs(snapped - target) < 1e-12] = target
    if np.all(np.isin(np.abs(snapped), (0.0, 1.0))):
        body_axis = np.abs(snapped).argmax(axis=1)   # link axis i is body axis body_axis[i]
        if shape == "box":
            return [0.0, 0.0, 0.0], [dims[body_axis[i]] for i in range(3)]
        if shape in ("cylinder", "capsule") and body_axis[2] == 2:
            return [0.0, 0.0, 0.0], list(dims)
    return matrix_to_rpy(snapped), list(dims)

def rnd(xs, decimals=POSITION_DECIMALS):
    return [round(float(x), decimals) + 0.0 for x in xs]   # + 0.0 turns -0.0 into 0.0

def ingest(config_path):
    import mujoco  # the importer compiles the export with MuJoCo
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    export_dir = os.path.dirname(cfg["export"])
    m = mujoco.MjModel.from_xml_path(cfg["export"])
    d = mujoco.MjData(m)

    # --- 2. Close each linkage at the export pose's output angle. ---
    def body(name):
        return m.body(name).id
    def qadr(joint):
        return m.joint(joint).qposadr[0]
    for lk in cfg["linkages"]:
        cl = lk["closure"]
        def residual(x, lk=lk, cl=cl):
            d.qpos[qadr(lk["input_joint"])], d.qpos[qadr(lk["coupler_joint"])] = x
            mujoco.mj_kinematics(m, d)
            def world(body_name, point):
                b = body(body_name)
                return d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.asarray(point, dtype=float)
            return world(cl["coupler_body"], cl["coupler_point_m"]) - \
                world(cl["input_body"], cl["input_point_m"])
        solutions = []
        for start in np.linspace(-np.pi, np.pi, 13):
            r = least_squares(residual, [0.0, start], xtol=1e-15, ftol=1e-15, gtol=1e-15)
            if np.linalg.norm(residual(r.x)) < 1e-9:
                solutions.append(r.x)
        if not solutions:
            raise ValueError(f"{lk['name']}: the closure pin's points cannot be made to coincide")
        # Of the two assembly modes, the CAD's is the one with the input crank nearest its pose.
        best = min(solutions, key=lambda x: abs(x[0]))
        d.qpos[qadr(lk["input_joint"])], d.qpos[qadr(lk["coupler_joint"])] = best
        print(f"{lk['name']}: closed at {lk['input_joint']} = {best[0]:+.6f} rad, "
              f"{lk['coupler_joint']} = {best[1]:+.6f} rad from the export pose")
    mujoco.mj_kinematics(m, d)

    # --- 3. Re-root at `root`. ---
    edges = {}   # body pair -> export joint name
    for j in range(m.njnt):
        child = m.jnt_bodyid[j]
        edges[frozenset((m.body_parentid[child], child))] = m.joint(j).name
    links_cfg, joints_cfg = cfg["links"], cfg["joints"]
    root = body(cfg["root"])
    parent_of, order, frontier = {root: None}, [root], [root]
    while frontier:
        b = frontier.pop(0)
        for pair, jname in edges.items():
            if b in pair:
                other = next(iter(pair - {b}))
                if other not in parent_of and other != 0:
                    parent_of[other] = (b, jname)
                    order.append(other)
                    frontier.append(other)
    names = {body(n): links_cfg[n] for n in links_cfg}
    if set(names) != set(order):
        raise ValueError("config `links` must name every body of the export exactly once")

    # --- 4. Link frames: robot axes, origin on the parent joint's axis. ---
    forward, up = np.asarray(cfg["forward"], float), np.asarray(cfg["up"], float)
    r_rc = np.array([forward, np.cross(up, forward), up])   # rows: robot x, y, z in CAD axes
    if not np.allclose(r_rc @ r_rc.T, np.eye(3), atol=1e-12):
        raise ValueError("`forward` and `up` must be orthogonal unit vectors")
    def anchor(jname):
        return d.xanchor[m.joint(jname).id].copy()
    origin = {root: anchor(cfg["root_frame_at_joint"])}
    for b in order[1:]:
        origin[b] = anchor(parent_of[b][1])
    def to_link(b, world_point):
        return r_rc @ (np.asarray(world_point) - origin[b])

    # --- 5. Links: inertial, mesh, collision. ---
    mesh_files = {}
    scale_of = {}
    for mesh in ET.parse(cfg["export"]).getroot().iter("mesh"):
        mesh_files[mesh.get("name")] = os.path.join(export_dir, mesh.get("file"))
        scale_of[mesh.get("name")] = np.array([float(s) for s in
                                               mesh.get("scale", "1 1 1").split()])
    geom_mesh = {m.geom_bodyid[g]: m.mesh(m.geom_dataid[g]).name for g in range(m.ngeom)
                 if m.geom_dataid[g] >= 0}
    density = cfg.get("density_kg_m3")
    purchased = {}
    for part in cfg.get("purchased_parts", []):
        purchased.setdefault(part["body"], []).append(part)
    if purchased and density is None:
        raise ValueError("purchased_parts replace a placeholder's mass, which needs the other "
                         "shells' density: set density_kg_m3")
    os.makedirs(cfg["meshes_out"], exist_ok=True)
    links = []
    for b in order:
        bname, lname = m.body(b).name, names[b]
        r_wb, p_wb = d.xmat[b].reshape(3, 3), d.xpos[b]
        tri = read_stl(mesh_files[geom_mesh[b]]) * scale_of[geom_mesh[b]]   # body frame, m

        if density is None:
            # Fusion's inertial, compiled by MuJoCo, in the body frame.
            mass = float(m.body_mass[b])
            com_b = m.body_ipos[b].copy()
            r_ib = np.zeros(9)
            mujoco.mju_quat2Mat(r_ib, m.body_iquat[b])
            r_ib = r_ib.reshape(3, 3)
            inertia_b = r_ib @ np.diag(m.body_inertia[b]) @ r_ib.T
        else:
            mass, com_b, inertia_b = mass_properties(tri, density, purchased.get(bname, []))
        print(f"{lname}: {mass:.4f} kg")
        r_lb = r_rc @ r_wb                                   # body axes in link axes
        com = to_link(b, p_wb + r_wb @ com_b)
        inertia = r_lb @ inertia_b @ r_lb.T

        verts = tri.reshape(-1, 3) @ r_wb.T + p_wb
        write_stl(os.path.join(cfg["meshes_out"], f"{lname}.stl"),
                  ((verts - origin[b]) @ r_rc.T).reshape(-1, 3, 3))

        prims = []
        for prim in cfg["collision"].get(bname, []):
            centre = to_link(b, p_wb + r_wb @ np.asarray(prim["centre_m"], float))
            rpy, dims = primitive_in_link(prim["shape"], prim["dimensions_m"], r_lb)
            prims.append({"shape": prim["shape"],
                          "frame": {"xyz_m": rnd(centre), "rpy_rad": rnd(rpy, DIRECTION_DECIMALS)},
                          "dimensions_m": rnd(dims)})
        links.append({
            "name": lname,
            "parent_joint": None if parent_of[b] is None else joints_cfg[parent_of[b][1]]["name"],
            "mass_kg": float(mass),
            "com_m": rnd(com),
            "inertia": {"ixx_kg_m2": float(inertia[0, 0]), "iyy_kg_m2": float(inertia[1, 1]),
                        "izz_kg_m2": float(inertia[2, 2]), "ixy_kg_m2": float(inertia[0, 1]),
                        "ixz_kg_m2": float(inertia[0, 2]), "iyz_kg_m2": float(inertia[1, 2])},
            "visual_mesh": lname,
            "collision": {"primitives": prims, "mesh": None},
        })

    # --- Joints. ---
    joints = []
    for b in order[1:]:
        pb, jname = parent_of[b]
        jc = joints_cfg[jname]
        jid = m.joint(jname).id
        reversed_ = m.jnt_bodyid[jid] != b        # the export's child is now the parent
        cad_axis = r_rc @ d.xaxis[jid] * (-1.0 if reversed_ else 1.0)
        axis = np.asarray(jc["axis"], float)
        if np.linalg.norm(np.cross(axis, cad_axis)) > 1e-9 or abs(np.linalg.norm(axis) - 1) > 1e-12:
            raise ValueError(f"{jname}: configured axis {axis.tolist()} is not +-{cad_axis} "
                             "in robot axes")
        joints.append({"name": jc["name"], "type": "revolute",
                       "parent_link": names[pb], "child_link": names[b],
                       "origin": {"xyz_m": rnd(r_rc @ (origin[b] - origin[pb])),
                                  "rpy_rad": [0.0, 0.0, 0.0]},
                       "axis": rnd(axis, DIRECTION_DECIMALS)})

    linkages = []
    for lk in cfg["linkages"]:
        cl = lk["closure"]
        def closure_point(body_name, point):
            b = body(body_name)
            return rnd(to_link(b, d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.asarray(point, float)))
        linkages.append({
            "name": lk["name"], "type": "planar_four_bar",
            "input_joint": joints_cfg[lk["input_joint"]]["name"],
            "output_joint": joints_cfg[lk["output_joint"]]["name"],
            "coupler_joint": joints_cfg[lk["coupler_joint"]]["name"],
            "closure": {"coupler_point_m": closure_point(cl["coupler_body"], cl["coupler_point_m"]),
                        "input_point_m": closure_point(cl["input_body"], cl["input_point_m"])}})

    # --- Keep what the CAD does not own. ---
    old = {}
    if os.path.exists(cfg["model_out"]):
        with open(cfg["model_out"]) as f:
            old = yaml.safe_load(f) or {}
    old_joints = {j["name"]: j for j in old.get("joints", [])}
    for j in joints:
        for key in KEPT_JOINT_FIELDS:
            if key in old_joints.get(j["name"], {}):
                j[key] = old_joints[j["name"]][key]
    model = {"schema_version": SCHEMA_VERSION,
             "robot": {"name": cfg["model_name"], "root_link": names[root], "base": cfg["base"]},
             "links": links, "joints": joints}
    for key in KEPT_TOP_LEVEL:
        if key in old:
            model[key] = old[key]
    model.setdefault("actuators", [])
    model["linkages"] = linkages

    header = (f"# Imported from {cfg['export']} by `python3 -m model.generators.ingest_fusion "
              f"{config_path}`.\n"
              "# Re-importing rewrites frames, inertials, meshes, collision and closure pins, and "
              "keeps every other field.\n")
    for line in cfg.get("notes", []):
        header += f"# {line}\n"
    text = header + yaml.safe_dump(model, sort_keys=False, width=1000, allow_unicode=True)
    with open(cfg["model_out"] + ".tmp", "w") as f:
        f.write(text)
    os.replace(cfg["model_out"] + ".tmp", cfg["model_out"])
    print(f"wrote {cfg['model_out']} and {len(links)} meshes in {cfg['meshes_out']}")

def main():
    p = argparse.ArgumentParser(description="Import a Fusion 360 (acdc4robot) export into a "
                                            "robot_model.yaml.")
    p.add_argument("config", help="import config, e.g. model/robots/leg_prototype/source/"
                                  "ingest.yaml")
    ingest(p.parse_args().config)

if __name__ == "__main__":
    main()
