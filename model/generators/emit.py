"""robot_model.yaml (+ contact overlay) -> URDF, MJCF, safety_manifest.yaml.

MJCF is emitted with NO <actuator> section: MujocoActuatorTransport applies the MIT
tuple as joint torque via qfrc_applied (ADR-007 seam, ADR-001 tuple). Compiler flags
come from the pinned simulation block; see AMEND-1 and model-gate-0 §5.5.
"""
import hashlib
import math
import os
import re
import xml.etree.ElementTree as ET

import yaml

from .common import num_or, quat_to_rpy, rpy_to_quat

TIMING_HPP = "src/humanoid_transport/include/humanoid_transport/timing.hpp"

# Provisional envelope constants until G3 actuator characterisation.
PROV_KP_MAX, PROV_KD_MAX = 500.0, 10.0
PROV_SLEW_NM_S, PROV_POWER_W = 500.0, 400.0
CMD_INTERFACES = ("position", "velocity", "effort", "stiffness", "damping")
STATE_INTERFACES = ("position", "velocity", "effort")
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

def emit_urdf(model, out_path, assets_dir,
              safety_manifest_path="config/safety_manifest.yaml",
              safety_manifest_package="humanoid_bringup"):
    links = {l["name"]: l for l in model["links"]}
    joints = model["joints"]
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
        rpy = inertia["principal_axes_rpy_rad"] or [0, 0, 0]
        ie.set("xyz", fmt(com))
        ie.set("rpy", fmt(rpy))
        ET.SubElement(ie, "mass", value=f"{l['mass']['cad_kg']:.10g}")
        ET.SubElement(
            ie,
            "inertia",
            ixx=f"{inertia['ixx_kg_m2']:.10g}",
            iyy=f"{inertia['iyy_kg_m2']:.10g}",
            izz=f"{inertia['izz_kg_m2']:.10g}",
            ixy="0",
            ixz="0",
            iyz="0",
        )

        mesh = l.get("visual_mesh")
        if mesh:
            # Find the actual filename on disk matching this mesh name. Deterministic: scan a
            # sorted list, and prefer an exact match over a prefix one (os.listdir order must never
            # decide "pelvis" -> pelvis.STL vs pelvis_contour_link.STL).
            def _match(names, mesh=mesh):
                for fname in names:
                    stem = os.path.splitext(fname)[0]
                    if fname == mesh or stem == mesh:
                        return fname
                for fname in names:  # suffixes like torso_link_rev_1_0.STL
                    if os.path.splitext(fname)[0].startswith(mesh):
                        return fname
                return None

            mesh_file = _match(sorted(disk_assets))
            # Fallback to appending .STL if no file on disk matched
            if not mesh_file:
                mesh_file = mesh if "." in mesh else f"{mesh}.STL"

            mesh_uri = f"{assets_uri}/{mesh_file}"

            for tag in ("visual", "collision"):
                ve = ET.SubElement(le, tag)
                oe = ET.SubElement(ve, "origin", xyz="0 0 0", rpy="0 0 0")
                ET.SubElement(ve, "geometry").append(
                    ET.Element("mesh", filename=mesh_uri)
                )

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
        ET.SubElement(
            je,
            "limit",
            lower=f"{lim['software_lower_rad']:.10g}",
            upper=f"{lim['software_upper_rad']:.10g}",
            effort=f"{lim['effort_peak_nm']:.10g}",
            velocity=f"{lim['velocity_rad_s']:.10g}",
        )
        tr = j["transmission"]
        ET.SubElement(
            je,
            "dynamics",
            damping=f"{num_or(tr['friction_viscous_nm_s_rad']):.10g}",
            friction=f"{num_or(tr['friction_coulomb_nm']):.10g}",
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
    for j in joints:
        if j["type"] == "fixed":
            continue
        je = ET.SubElement(rc, "joint", name=j["name"])
        for c in CMD_INTERFACES:
            ET.SubElement(je, "command_interface", name=c)
        for s in STATE_INTERFACES:
            ET.SubElement(je, "state_interface", name=s)
    if imu is not None:
        se = ET.SubElement(rc, "sensor", name=imu["name"])
        for s in IMU_INTERFACES:
            ET.SubElement(se, "state_interface", name=s)

    ET.indent(robot)
    ET.ElementTree(robot).write(
        out_path, xml_declaration=True, encoding="utf-8"
    )

def emit_mjcf(model, overlay, out_path, assets_rel):
    links = {l["name"]: l for l in model["links"]}
    joints = model["joints"]
    kids = children_by_parent(joints)
    joint_by_child = {j["child_link"]: j for j in joints}
    sim = model["simulation"]["mjcf_compiler"]
    imu = model.get("imu")

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
        if imu is not None and imu["link"] == link_name:
            ET.SubElement(be, "site", name=imu["name"], pos=fmt(imu["pos_m"]),
                          quat=fmt(imu["quat_wxyz"]), size="0.01")
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
                je.set("armature", f"{num_or(tr.get('armature_kg_m2'), default=0.0):.10g}")
        for cj in kids.get(link_name, []):
            add_body(be, cj["child_link"], False)

    wb = ET.SubElement(mj, "worldbody")

    ground = ET.SubElement(wb, "geom")
    ground.set("type", "plane")
    ground.set("size", "0 0 0.1")
    ground.set("pos", "0 0 0")
    ground.set("friction", "0.6")
    ground.set("condim", "3")

    add_body(wb, model["robot"]["root_link"], True)

    if imu is not None:
        # Exactly one sensor of each type: MujocoActuatorTransport finds them by type, not name.
        sensors = ET.SubElement(mj, "sensor")
        ET.SubElement(sensors, "framequat", name=f"{imu['name']}_orientation", objtype="site",
                      objname=imu["name"])
        ET.SubElement(sensors, "gyro", name=f"{imu['name']}_angular_velocity", site=imu["name"])
        ET.SubElement(sensors, "accelerometer", name=f"{imu['name']}_linear_acceleration",
                      site=imu["name"])

    ET.indent(mj)
    ET.ElementTree(mj).write(out_path, xml_declaration=True, encoding="utf-8")

def emit_safety_manifest(model, out_path):
    # Per-joint gain caps come from the actuator's measured `drive` block when present; a
    # provisional/sim actuator without one falls back to the global PROV_* envelope.
    act_by_name = {a["name"]: a for a in model["actuators"]}
    envelopes = {}
    for j in model["joints"]:
        if j["type"] == "fixed":
            continue
        lim = j["limits"]
        drive = (act_by_name.get(j["actuator"]) or {}).get("drive")
        envelopes[j["name"]] = {
            "position_min_rad": lim["software_lower_rad"],
            "position_max_rad": lim["software_upper_rad"],
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

def emit_firmware_config(model, out_path, setup):
    """A config artifact the firmware repo consumes (it is committed there with its hash).
    Everything is derived from robot_model.yaml; the firmware never hand-edits it, and the hash
    lets its generator refuse a tampered copy."""
    act_by_name = {a["name"]: a for a in model["actuators"]}
    joint_by_act = {j["actuator"]: j for j in model["joints"] if j["actuator"] is not None}

    # Per-model drive ranges and the shared Kp/Kd/position ranges, reconstructed from the drives.
    models, shared = {}, None
    for a in model["actuators"]:
        d = a.get("drive")
        if d is None:
            raise ValueError(f"actuator {a['name']} has no drive block; firmware_config needs it")
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
            j, d = joint_by_act[a["name"]], a["drive"]
            lim = j["limits"]
            rows.append({
                "idx": a["slot"], "can_id": a["can_node_id"], "model": a["motor_class"],
                "joint_name": j["name"],
                "soft_min": lim["software_lower_rad"], "soft_max": lim["software_upper_rad"],
                "max_vel": lim["velocity_rad_s"], "max_tau": lim["effort_peak_nm"],
                "default_kp": d["default_kp"], "default_kd": d["default_kd"],
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
    chain = index of the actuator's bus_segment; motor = its slot; direction_sign and
    zero_offset_rad come from the joint. Serial device and timeouts are deployment parameters and
    stay in the launch, not here."""
    segments = sorted({a["bus_segment"] for a in model["actuators"]})
    chain_of = {seg: i for i, seg in enumerate(segments)}
    act_by_name = {a["name"]: a for a in model["actuators"]}
    rows = []
    for j in model["joints"]:
        if j["type"] == "fixed":
            continue
        a = act_by_name[j["actuator"]]
        rows.append((chain_of[a["bus_segment"]], a["slot"], j["name"],
                     j["transmission"]["direction_sign"], j["calibration"]["zero_offset_rad"]))
    rows.sort()
    joints = {name: {"chain": chain, "motor": motor,
                     "direction_sign": sign, "zero_offset_rad": zero}
              for chain, motor, name, sign, zero in rows}
    doc = {"stm32_serial": {"ros__parameters": {"joints": joints}}}
    banner = (f"# AUTO-GENERATED by model/generators from {model['robot']['name']}. Do not edit.\n"
              "# The Stm32SerialTransport joints map (chain/motor/direction_sign/zero_offset_rad).\n"
              "# zero_offset_rad and direction_sign are provisional until verified by hand.\n")
    with open(out_path, "w") as f:
        f.write(banner)
        yaml.safe_dump(doc, f, sort_keys=False, width=1000)

def generate(src, overlay_path, out_dir, assets_dir, firmware_config=False, firmware_setup=None,
             stm32_wiring=False, safety_manifest_path=None, safety_manifest_package=None):
    model, overlay = load(src, overlay_path)
    os.makedirs(out_dir, exist_ok=True)
    emission = model["emission"]
    written = []

    if emission["urdf"]["enabled"]:
        # URDF gets assets_dir to build package:// URIs with .STL extensions. The safety-manifest
        # path/package override is per-robot; the defaults keep G1's ros2_control block unchanged.
        urdf_kwargs = {}
        if safety_manifest_path:
            urdf_kwargs["safety_manifest_path"] = safety_manifest_path
        if safety_manifest_package:
            urdf_kwargs["safety_manifest_package"] = safety_manifest_package
        emit_urdf(model, os.path.join(out_dir, "robot.urdf"), assets_dir, **urdf_kwargs)
        written.append("robot.urdf")

    if emission["mjcf"]["enabled"]:
        # MJCF keeps relative path because of the <compiler meshdir="..." /> tag
        assets_rel = os.path.relpath(assets_dir, "model/generated").replace("\\", "/")
        emit_mjcf(model, overlay, os.path.join(out_dir, "robot.mjcf"), assets_rel)
        written.append("robot.mjcf")

    emit_safety_manifest(model, os.path.join(out_dir, "safety_manifest.yaml"))
    written.append("safety_manifest.yaml")

    if firmware_config:
        setup = firmware_setup or model["robot"]["name"]
        emit_firmware_config(model, os.path.join(out_dir, "firmware_config.yaml"), setup)
        written += ["firmware_config.yaml", "firmware_config.yaml.sha256"]

    if stm32_wiring:
        emit_stm32_wiring(model, os.path.join(out_dir, "stm32_wiring.yaml"))
        written.append("stm32_wiring.yaml")

    print(f"generated {', '.join(written)} in {out_dir}")