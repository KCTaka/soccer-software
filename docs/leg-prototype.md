# Leg prototype (`leg_prototype`)

One lower leg on a test stand: the shin is clamped, and the foot swings about the ankle. A RobStride
RS02 on the shin turns the ankle through a four-bar linkage. This page covers the model, how the
closed loop is handled, SIL, and the procedure for the first hardware session.

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd                                   # first, in its own terminal
ros2 launch humanoid_bringup leg.launch.py                          # SIL (MuJoCo) with sliders
ros2 launch humanoid_bringup leg.launch.py mode:=monitor            # real leg, never armed
ros2 launch humanoid_bringup leg.launch.py mode:=hil                # real leg, motor armed
ros2 run humanoid_bringup bench_trajectories --ros-args \
  -p joints:=[left_ankle_pitch_joint] -p amplitude_rad:=0.2 -p frequency_hz:=0.25
```

Add `rviz:=false foxglove:=true` on a host without a display (the Jetson), and
`serial_device:=/dev/robosoccer-master` outside the dev container.

## The mechanism

| Link | Pins | Length |
|---|---|---|
| Ground (shin) | ankle axis to motor axis | 120 mm |
| Input crank (horn, on the RS02) | motor axis to rod pin | 45 mm |
| Coupler (rod) | horn pin to foot pin | 120 mm |
| Output crank (foot) | ankle axis to rod pin | 40 mm |

All four pins are parallel to the robot's y axis. The lengths are hole centres fitted to the CAD
meshes. It is not a parallelogram, so the transmission ratio varies:

| Ankle (deg) | Motor (deg) | d(ankle)/d(motor) |
|---|---|---|
| −40 | −34.96 | 1.1935 |
| −20 | −17.72 | 1.1375 |
| 0 | 0.00 | 1.1251 |
| +20 | +17.67 | 1.1440 |
| +40 | +34.71 | 1.2193 |

The ankle turns faster than the motor and with less torque: ankle torque = motor torque / ratio. The
foot crank is the shortest link (Grashof crank-rocker), so the horn is the rocker. At its two toggles,
ankle +81.5° and −102°, the ratio is unbounded and the motor can no longer hold the foot. The joint
limits keep the foot well clear of both.

**Conventions.** Link frames follow REP-103 (x toward the toe, y left, z up), and every joint turns
about +y. Positive ankle angle is toes down, as for the G1. Zero is the foot square to the shin with
the loop closed; the horn then sits 0.13° off its CAD orientation, because a 45 mm horn and a 40 mm
crank cannot both be level when the rod closes the loop.

**Range.** A CAD interference sweep puts the rod against the shin between +44° and +45° toes down,
and toes up is clear to −70°. Those are the hard stops (`−1.2217`, `+0.7679` rad). The software
limits are ±40° (±0.6981 rad): the ratio stays within 1.125–1.22 and the transmission angle above
47°. With the motor limp, the foot hangs at about +12°, inside the range.

## How the closed loop is handled

The Fusion export was a tree, with the rod's top pin missing and the rod left 17.9° from closing.
The model now carries the loop explicitly, and each consumer gets the representation it can use:

- **Model.** `robot_model.yaml` has a `linkages` entry: input `left_ankle_motor_joint` (the horn,
  with the RS02 as its actuator), output `left_ankle_pitch_joint` (the ankle, which controllers
  command), coupler `left_ankle_rod_joint`, and the closure pin as a point in the rod and in the
  horn. The zero pose closes the loop, and validator checks L-01 to L-06 hold it to that.
- **MJCF (SIL).** The true closed chain. The motor drives the horn joint, and a `connect` equality
  between two sites pins the rod to the horn. Contacts between the four linkage bodies are excluded,
  and the ankle's range models the hard stops.
- **URDF and ros2_control.** The tree, with the horn and rod joints state-only. A `<transmission>`
  of plugin `humanoid_actuator_system/FourBarLinkage` carries the four-bar's parameters.
- **HumanoidActuatorSystem.** Controllers work in joint space. The component maps the ankle's MIT
  tuple to the motor's with the exact closed-form four-bar (`FourBarLinkage`) and maps the motor's
  feedback back to the ankle, horn and rod. The SafetyKernel runs after the map, in motor space, so
  it bounds what the motor is asked to do even if the map were wrong.

The command map at commanded ankle angle q, with r = d(ankle)/d(motor) there, is:

| Field | Motor |
|---|---|
| position | the horn angle that closes the loop at q |
| velocity | ankle velocity / r |
| feed-forward torque | ankle torque × r |
| stiffness, damping | ankle gain × r² |

That makes the impedance exact about the setpoint and first-order accurate away from it.

**Why the ankle, not the motor, is the commanded joint.** The URDF, TF and any learned policy need
the ankle angle on a serial tree, so the four-bar map has to run at runtime either way. Joint-space
gains and torques keep their physical meaning across the range, and a future 2-DOF parallel ankle
has coupled motor space that a motor-space interface cannot express. Running the map in front of
the SafetyKernel keeps every motor-side limit native and exact. SIL simulates the true linkage
rather than the same map, so it tests the map against independent physics. The map itself is exact,
with no lookup table or fitted polynomial, and it is cheap.

### Evidence

- **Kinematics.** The C++ linkage closes MuJoCo's own tree of the generated MJCF to under 1 nm
  over the whole range (`four_bar_mujoco_test`). The same test checks the static ratio against
  MuJoCo's constraint forces, which carry the motor's torque round the loop.
- **Statics.** In a sweep of the generated scene, the ratio matches τ_ankle = τ_motor / r to
  0.012% at −40°, 0° and +40°.
- **Pin stretch.** MuJoCo's constraints are soft, so the pin stretches under load. Held statically,
  it opens under 1 µm at the motor's 3.05 N·m envelope and 5 µm at the RS02's 17 N·m peak. On a
  0.2 rad step at the leg's gains it reaches 15 µm transiently, and 0.5–0.75 mm at kp 200–330
  N·m/rad with the drive saturated.
- **Why the armature matters.** MuJoCo's implicit integrator applies the MIT damping through the
  horn joint's mass matrix but computes the pin force without it. With armature 0 on a 35 g horn,
  the pin stretches by millimetres. Every real motor has rotor inertia, and with it the stretch
  stays at the numbers above. Check L-06 requires it.
- **SIL tracking.** A 0.3 rad, 0.5 Hz sine on the ankle tracks with gain 1.0001 and a 1.7 mrad
  residual after removing a 60 ms pipeline lag. A slider held at 0.3 rad puts the ankle at
  0.2999 rad, the motor at 0.2654 rad and the rod at −0.3018 rad, as the linkage predicts.

## Model provenance and what is provisional

`robot_model.yaml` was imported by `python3 -m model.generators.ingest_fusion
model/robots/leg_prototype/source/ingest.yaml`. Re-run it on a new CAD export (see below).

| Value | Status |
|---|---|
| Masses and inertias | **Provisional.** Every part is solid PLA (1.24 g/cm³, infill not modelled), integrated exactly over its CAD mesh: foot 95.5 g, horn 13.3 g, rod 9.6 g, shin 343 g. Fusion's own values used its default steel on most parts, so they are not used. Replace with weighed masses. |
| Shin motor mass | The 57 × 52 × 57 mm placeholder block in the shin takes the RS02's 380 g (shin total 723 g). The shin is clamped, so this matters only for the full leg. |
| Horn joint armature, 0.002 kg·m² | **An estimate.** The RS02's reflected rotor inertia, taken as a 59 g, Φ55/45 × 10 mm rotor ring × 7.75², uncertain by a factor of 2–3. RobStride publishes none. With PLA parts it is 87% of the ankle's effective inertia, so it decides the leg's dynamics in SIL: identify it on hardware (below). |
| Hard stops | From the CAD interference sweep. |
| `zero_offset_rad` | **Not calibrated.** Null, so the STM32 transport refuses to start. |
| `direction_sign` | **Provisional.** +1, to verify. |
| Collision | Boxes and a cylinder fitted to each part's solids. The sole is a 160 × 60 × 6 mm box. |

## SIL

`leg.launch.py` (mode `sil`) runs MujocoActuatorTransport on `generated/scene.xml`. The shin is
welded 88 mm above the floor, which is just clear of it over the ankle's whole range. The scene holds
still until the controller's first command. `humanoid_mit_controller` holds the measured pose with
kp 10 N·m/rad and kd 0.3 N·m·s/rad until a reference arrives. `/joint_states` reports the ankle, the
motor's joint and the rod.

**Sliders.** SIL opens a slider window (`joint_state_publisher_gui`) next to RViz that commands the
ankle (`sliders:=auto`, the default, turns it on in `sil` only).
- It publishes the slider's value on `/humanoid_mit_controller/joint_references`, within the
  ankle's ±40° URDF limits, and RViz shows the linkage from the simulated states.
- The window is given a copy of the description with every joint the `<ros2_control>` block does
  not command made fixed. It offers only the ankle, and never puts the horn or rod in a reference,
  which the controller reads by index.
- Each drag is a step that the controller does not interpolate. Don't run `bench_trajectories`
  while the window is open, because both publish references.
- On the real leg, `sliders:=true` with `mode:=hil` works the same way. The slider starts at 0,
  so start with the foot near 0.

## First hardware session

You need:

- the shin clamped so the foot can swing ±45° without touching anything;
- a way to cut motor power within reach;
- the master and one slave flashed with the generated `1s_1m` config,
  `model/robots/leg_prototype/generated/firmware_config.yaml` (the firmware repo checks its
  `content_sha256`);
- the RS02's CAN id set to 1, on slave0's bus.

The steps below come from the STM32 transport's code.

1. **Read-only link check.** Run
   `ros2 run humanoid_transport_stm32 stm32_link_probe /host-dev/robosoccer-master`. It sends
   nothing. Expect chain 0 with one motor, its drive `DISABLED` (powered, not armed) with an `age`
   of a few ms, and its `pos` in radians in the wire's home frame.
2. **Calibrate the zero offset.** With the drive powered but `DISABLED`, hold the foot square to the
   shin and read the motor's `pos` from the probe. That is `zero_offset_rad`: the transport maps
   joint = direction_sign × (wire − zero_offset_rad).
   - The transport refuses an envelope that leaves ±π in the home frame once the offset is applied.
     The motor's range is about ±0.61 rad, so |zero_offset_rad| must be under about 2.5 rad.
   - If it isn't, set the RS02's mechanical zero near the foot-square pose with RobStride's tool,
     then read the offset again.
3. **Find the direction sign.** Tilt the foot toes down by hand. The horn tip should rise and the
   joint angle increase. If the probe's `pos` falls instead, `direction_sign` is −1.
4. **Regenerate.** Set `calibration.zero_offset_rad` and `transmission.direction_sign` on
   `left_ankle_motor_joint` in `robot_model.yaml`. Then run
   `python3 -m model.generators.cli generate --robot leg_prototype` and
   `python3 -m model.generators.cli check`, and rebuild with
   `colcon build --packages-select humanoid_bringup`. The wiring now carries the offset.
5. **Monitor.** Launch with `mode:=monitor`. The transport never arms the motor.
   - With the foot square, `/joint_states` should read the ankle at 0 ± 0.01 rad.
   - Toes down should read positive.
   - A protractor at ±30° should agree with the reading. In RViz, the horn and rod should follow the
     real ones.
6. **Arm.** Support the foot inside ±40° and launch with `mode:=hil`. The controller holds the
   measured pose. Run `bench_trajectories` with `-p amplitude_rad:=0.1 -p frequency_hz:=0.25` first.
   The summary should report `direction=match` and a small tracking error. Then go to 0.3 rad.
   - The motor's torque bound is 3.05 N·m, the ankle's 2.5 N·m, and the firmware trips at
     `max_tau`. The SafetyKernel stops the run at the first command outside the motor's envelope.
7. **Identify the armature.** Unpin the rod from the horn and clamp a known mass to the horn tip
   (about 200 g at the 45 mm hole), so the horn is a pendulum on the motor. Launch `mode:=monitor`,
   lift the horn about 20° and release it, and record `/joint_states`
   (`ros2 bag record /joint_states`).
   - From the damped swing frequency ω, armature = m g d / ω² − I, with I the horn and mass about the
     motor axis and m g d their gravity moment (m the total mass, d its distance from the axis).
   - Use only `left_ankle_motor_joint`: with the rod off, the derived ankle is meaningless.
   - If friction stops the swing, use a heavier mass.
8. **Report back** the zero offset, the sign, the armature, and weighed masses for the foot, horn and
   rod. They go in `robot_model.yaml`; regenerate and commit.

## Re-importing a CAD export

Export from Fusion with acdc4robot into `source/Leg-Assembly---Export-v<N>/` and point `export` in
`source/ingest.yaml` at it. Run `python3 -m model.generators.ingest_fusion
model/robots/leg_prototype/source/ingest.yaml`, then `cli generate` and `cli check`, and diff before
committing.

The import rewrites only what the CAD owns: frames, inertials, meshes, collision boxes and the
closure pin. It keeps limits, calibration, actuators and transmission fields. The closure pin's
points and the collision boxes in `ingest.yaml` are in each export body's frame, so update them
if those parts change.
