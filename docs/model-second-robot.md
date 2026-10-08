# A second robot in the model pipeline: `bench_2s_10m`

This design adds a second robot — the 10-motor dial bench — to `model/`, and makes the model
pipeline the single source of truth for a **firmware config artifact** the firmware repo consumes.
It was reviewed before implementation; the decisions taken are recorded at the end.

The rules it must not break are in [AGENTS.md](../AGENTS.md) and
[docs/standards/CPP_GUIDELINES.md](standards/CPP_GUIDELINES.md): one source, derived or checked,
never copied; generated files are never hand-edited; `python3 -m model.generators.cli check`
rejects drift.

## 1. What the pipeline does today, and where it is single-robot

`model/generators` is a three-stage pipeline over one robot:

- **`ingest`** — `model/source/ingest.yaml` (a pinned MuJoCo-Menagerie MJCF) → `robot_model.yaml`
  + the contact overlay. Menagerie-specific.
- **`generate`** — `robot_model.yaml` (+ overlay) → `robot.urdf`, `robot.mjcf`,
  `safety_manifest.yaml`.
- **`check`** — JSON-Schema + structural checks (`validate.py`) + **regenerate-and-diff**
  (ADR-009-07): re-run ingest and generate into a temp dir, byte-compare.

Single-robot assumptions that have to give:

| Assumption | Location | Change |
|---|---|---|
| Hardcoded paths (`SRC`, `OUT`, `CONFIG`, …) | `cli.py:11-15` | a robots **registry**; `--robot <name>` |
| `ingest` needs a Menagerie MJCF | `ingest.py` | an **authored** source kind whose ingest is an identity normaliser |
| `generate` always emits URDF+MJCF, ignoring `emission.*.enabled` | `emit.py:360-376` | honour the per-robot `emission` toggles |
| MJCF geometry comes from the contact overlay | `emit_mjcf` `add_geoms` | bench disables MJCF; primitive-only MJCF is a later follow-up |
| Safety gains caps are global `PROV_*` (`PROV_KD_MAX=10` > drive 5) | `emit.py:16-17,343-346` | read per-joint caps from the actuator's `drive` block |
| `size_class: const kidsize` | schema | relax to an enum (`kidsize`, `benchtop`) |

What already generalises: `validate.structural_checks`, the schema `$defs`, optional `imu`,
and `actuators[].bus_segment/can_node_id` as the identity that crosses the transport seam.

## 2. Multi-robot structure

A registry, `model/robots.yaml`:

```yaml
robots:
  unitree_g1:                              # the existing robot, unmoved
    source_kind: menagerie
    ingest_config: model/source/ingest.yaml
    source:       model/source/robot_model.yaml
    overlay:      model/source/menagerie_contact_overlay.yaml
    generated:    model/generated
    assets_dir:   model/source/menagerie/unitree_g1/assets
    firmware_config: false
  bench_2s_10m:
    source_kind: authored
    source:     model/robots/bench_2s_10m/source/robot_model.yaml
    overlay:    null
    generated:  model/robots/bench_2s_10m/generated
    assets_dir: null                       # primitive geometry, no meshes
    firmware_config: true
```

`cli.py` takes `--robot <name>` (default `unitree_g1`, so existing invocations are unchanged);
`check` with no `--robot` iterates every registered robot. `source_kind`:

- **`menagerie`** — today's flow; regenerate-and-diff re-runs ingest and compares `robot_model.yaml`.
- **`authored`** — `robot_model.yaml` is the hand-written source of truth. `ingest` is an
  **identity normaliser**: it loads the source and re-emits it with the same
  `yaml.safe_dump` settings. Regenerate-and-diff then catches a hand-edit that is not in canonical
  form, exactly as it catches a stale Menagerie ingest for G1. The committed source is produced
  through that same dumper so it is already canonical.

G1 stays at its current paths so its generated files remain byte-identical; the registry simply
names them.

## 3. The `bench_2s_10m` model

A dial board, authored as a star-shaped kinematic tree (not a serial chain — each motor is
independently mounted):

- **Root link** `extrusion_base`: an aluminium extrusion, a box along +X.
- **Ten revolute joints**, all parented to `extrusion_base`. Five mount on the +Y face (slave 0),
  five on the −Y face (slave 1). Each child link `bench_j<k>_hand` is a thin "clock-hand" box.
- **Zero pose**: at joint angle 0 the hand points **radially away from the extrusion** (+Z),
  `calibration.zero_pose_value_rad: 0.0`.
- **Axis**: along each motor's shaft, pointing **out of the shaft** — `[0, 1, 0]` for the +Y face
  (slave 0), `[0, -1, 0]` for the −Y face (slave 1). `direction_sign: +1` for all ten; the sign is
  **verified by hand per joint** (turn the dial, confirm the reported angle increases the intended
  way) rather than assumed. A wrong sign runs a joint backwards, so it is a bench-verified value,
  not a guess.

Identity and limits, taken from the firmware's `configs/2s_10m` (`/firmware/configs/2s_10m`):

| joint | fw `joint_name` | bus_segment | slot | can_node_id | model | soft limits (rad) |
|---|---|---|---|---|---|---|
| `bench_j1` | motor0 | slave0 | 0 | 1 | RS02 | ±0.79 |
| `bench_j2` | motor1 | slave0 | 1 | 2 | RS02 | ±1.05 |
| `bench_j3` | motor2 | slave0 | 2 | 3 | RS00 | ±1.31 |
| `bench_j4` | motor3 | slave0 | 3 | 4 | RS00 | ±1.31 |
| `bench_j5` | motor4 | slave0 | 4 | 5 | RS00 | ±1.31 |
| `bench_j6` | motor5 | slave1 | 0 | 6 | RS02 | ±0.79 |
| `bench_j7` | motor6 | slave1 | 1 | 7 | RS02 | ±1.05 |
| `bench_j8` | motor7 | slave1 | 2 | 8 | RS00 | ±1.31 |
| `bench_j9` | motor8 | slave1 | 3 | 9 | RS00 | ±1.31 |
| `bench_j10` | motor9 | slave1 | 4 | 10 | RS00 | ±1.31 |

All ten: `velocity_rad_s: 10.0` (fw `max_vel`), `effort_continuous_nm = effort_peak_nm = 0.8`
(fw `max_tau`, the IDLE-trip torque), `effort_peak_duration_s: 1.0`,
`hard_stop = soft ∓ 0.02 rad` (the margin the G1 model uses, so J-01 holds),
`gear_ratio: 1.0`, `direction_sign: +1`.

### Two CAN buses, explicit slots, free-form CAN IDs

`bus_segment` is **one per physical CAN bus** (`slave0`, `slave1`) — not a single logical segment.
The **slot** is the explicit order within a bus (0-based); the chain/slot mapping is **stored, not
derived from the CAN id**. CAN ids are free-form (here 1–5 and 6–10, matching the firmware, but
nothing requires contiguity or a location encoding). `validate.py` enforces, across the whole robot:

- `(bus_segment, slot)` unique, and slots contiguous `0..n-1` within each segment;
- `(bus_segment, can_node_id)` unique (generalises the existing A-02).

The STM32 transport's `chain`/`motor` wiring is `chain = index of bus_segment`
(`slave0`→0, `slave1`→1), `motor = slot` — read straight from the model, no id arithmetic.

### Gains and drive caps on the actuator

The `actuator` `$def` gains two optional blocks (optional so G1 still validates):

- `slot: <int>`
- `drive: { default_kp, default_kd, kp_max, kd_max, velocity_max_rad_s, torque_max_nm,
  position_max_rad }`

For the bench: `default_kp: 15.0`, `default_kd: 1.0`; `kp_max: 500.0`, `kd_max: 5.0`
(RS00/RS02 `shared_ranges`); `velocity_max_rad_s`/`torque_max_nm` the per-model drive maxima
(RS00 33 / 14, RS02 44 / 17); `position_max_rad: 12.57` (±4π). `emit_safety_manifest` reads
`kp_max`/`kd_max` from `drive` when present, so the bench manifest caps stiffness at **500** and
damping at **5** — closing the gap the STM32 transport doc flagged (`PROV_KD_MAX=10` exceeded the
drives' Kd of 5). G1, with no `drive` block, keeps the `PROV_*` constants and byte-identical output.

## 4. The firmware config artifact

`generate` emits, for a robot whose registry entry sets `firmware_config: true`,
`<generated>/firmware_config.yaml` and a `<generated>/firmware_config.yaml.sha256` sidecar.
The ROS model **owns** this file; the firmware repo **consumes** it (committed there alongside its
hash). This reverses today's direction, where `configs/<setup>/slaveN.yaml` is the firmware's
source and host tools are generated from it — `scripts/gen_motor_config.py` would switch its input
to the imported `firmware_config.yaml`.

```yaml
# AUTO-GENERATED by model/generators from bench_2s_10m. Do not edit.
setup: 2s_10m
content_sha256: <hex over the body below, this field excluded>
rates:                          # from humanoid_transport/timing.hpp (200 Hz), not hand-typed
  master_poll_hz: 200
  telemetry_hz: 200
  slave_tick_hz: 200
  host_cmd_hz: 200
shared_ranges: {p_min: -12.57, p_max: 12.57, kp_min: 0.0, kp_max: 500.0, kd_min: 0.0, kd_max: 5.0}
models:
  RS00: {v_min: -33.0, v_max: 33.0, t_min: -14.0, t_max: 14.0}
  RS02: {v_min: -44.0, v_max: 44.0, t_min: -17.0, t_max: 17.0}
slaves:
  - slave: slave0
    motors:                     # ordered by slot
      - {idx: 0, can_id: 1, model: RS02, joint_name: bench_j1, soft_min: -0.79, soft_max: 0.79,
         max_vel: 10.0, max_tau: 0.8, default_kp: 15.0, default_kd: 1.0}
      # …
  - slave: slave1
    motors: [ … can_id 6..10 … ]
```

Everything is derived from `bench_2s_10m`'s `robot_model.yaml`: `slaves` groups actuators by
`bus_segment` and orders by `slot`; `models`/`shared_ranges` are reconstructed from the actuators'
`drive` blocks; `rates` come from `timing.hpp`. The hash is `sha256` over the canonical
(sorted-key, fixed-float) serialisation of the body with `content_sha256` removed. The firmware
generator records the hash in its banner and refuses to build if a recompute disagrees, so a
hand-edit in either repo is caught — "one source, never copied" across the repo boundary.

## 5. Validation additions (`validate.py`)

For a robot with actuator `drive`/`slot` blocks (the authored/physical kind):

- `(bus_segment, slot)` unique; slots contiguous `0..n-1` per segment.
- `(bus_segment, can_node_id)` unique (generalised A-02).
- `default_kp ≤ kp_max`, `default_kd ≤ kd_max`.
- `velocity_rad_s ≤ velocity_max_rad_s`, `effort_peak_nm ≤ torque_max_nm` for the joint's model.

These turn the transport doc's "nothing enforces gains/limits inside the drives' spans" into a
build-time check.

## 6. Files

| File | Change |
|---|---|
| `model/robots.yaml` | **new** — registry |
| `model/robots/bench_2s_10m/source/robot_model.yaml` | **new** — the authored dial model |
| `model/robots/bench_2s_10m/generated/` | **new** — `robot.urdf`, `safety_manifest.yaml`, `firmware_config.yaml` (+`.sha256`) |
| `model/schema/robot_model.schema.json` | `size_class` enum; actuator `slot` + `drive` (optional) |
| `model/generators/cli.py` | `--robot`; registry resolution; per-kind ingest/regen; iterate in `check` |
| `model/generators/ingest.py` | authored identity normaliser |
| `model/generators/emit.py` | honour `emission`; per-joint gain caps; `emit_firmware_config` |
| `model/generators/validate.py` | bench slot/can/gain/range checks |
| `humanoid_bringup` | `bench_dial.launch.py` + generated bench config + generated `stm32_serial` wiring |

## Decisions taken

1. Joint names `bench_j1 .. bench_j10` (j1–j5 = slave0/can 1–5, j6–j10 = slave1/can 6–10).
2. The ROS model **owns** the firmware config; the firmware consumes the exported
   `firmware_config.yaml`, committed in the firmware repo with its hash.
3. Each joint axis is along the motor shaft pointing out of the shaft; `direction_sign: +1` for
   all, verified by hand per joint.
4. `bus_segment` is one per physical CAN bus (`slave0`, `slave1`).
5. Slot order is explicit per bus segment; CAN IDs are free-form; `(bus_segment, slot)` and
   `(bus_segment, can_node_id)` uniqueness is validated across the robot.
