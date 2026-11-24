"""robot_model.yaml -> the YAML configs other code loads: the safety manifest, the
Stm32SerialTransport wiring, and the firmware config the firmware repository imports."""
import hashlib
import os
import re

import yaml

from . import linkage

TIMING_HPP = "src/humanoid_transport/include/humanoid_transport/timing.hpp"
# Provisional envelope constants until G3 actuator characterisation.
PROV_KP_MAX, PROV_KD_MAX = 500.0, 10.0
PROV_SLEW_NM_S, PROV_POWER_W = 500.0, 400.0

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
