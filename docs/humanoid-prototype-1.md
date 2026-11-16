# Humanoid prototype 1 (`humanoid_prototype_1`)

The first whole-robot prototype: a 20-joint, 13.9 kg, 0.72 m biped on RobStride motors, imported
from its Fusion 360 export. It is set up for simulation only. Its actuators are `PROVISIONAL`, so it
has no firmware export, STM32 wiring or hardware launch yet.

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd                                         # first, in its own terminal
ros2 launch humanoid_bringup sil_stand.launch.py robot:=humanoid_prototype_1
```

Add `rviz:=false foxglove:=true` on a host without a display.

## The model

| Link | Joint (motor) | Axis |
|---|---|---|
| `pelvis` (floating base): torso, both hip-pitch motors, both shoulder-pitch motors | | |
| `*_hip_pitch_link` | `*_hip_pitch_joint` (RS03) | +y |
| `*_hip_roll_link` | `*_hip_roll_joint` (RS06) | +x |
| `*_hip_yaw_link` (thigh, knee motor) | `*_hip_yaw_joint` (RS02) | +z |
| `*_knee_link` (shin, ankle motor) | `*_knee_joint` (RS03) | +y |
| `*_foot_link` | `*_ankle_pitch_joint`, through the four-bar | +y |
| `*_ankle_horn_link`, `*_ankle_rod_link` | `*_ankle_motor_joint` (RS00), `*_ankle_rod_joint` | +y |
| `*_shoulder_pitch_link` | `*_shoulder_pitch_joint` (RS02) | +y |
| `*_shoulder_roll_link` (upper arm) | `*_shoulder_roll_joint` (RS00) | +x |
| `*_elbow_link` (forearm) | `*_elbow_joint` (RS00) | ∓z at zero |

Axes follow the G1's: positive pitch swings a leg back, bends the knee and points the toes down.
The three hip axes meet at one point. At zero the legs are straight and the arms are in the CAD's
T-pose, so each elbow's axis is vertical. Each elbow axis is the CAD's own, so with the arm lowered
positive flexes the forearm forward.

Each ankle is the [leg prototype's](leg-prototype.md) four-bar, with the same 45/40/120/120 mm
geometry and the same rod part, so its ratio table and toggles apply. It is driven by an RS00 here.
The export cut each loop at the rod's foot pin and left the rod 5 mm off it. The importer closes the
loop there, which moves the horn 0.13° and the rod 2.25° from the export pose. It then moves the rod
onto the foot, so the rod's horn pin becomes the closure, as the model and `FourBarLinkage` expect.

## Where the numbers come from

[`ingest.yaml`](../model/robots/humanoid_prototype_1/ingest.yaml) records each source in its
`notes`. In short:

- **Inertials** are computed from each part's mesh as solid PLA at 1.24 g/cm³, with infill not
  modelled. Every part is printed PLA except the 16 motors, which keep the CAD's catalogue mass
  spread over their shells. The bearings are PLA here too, though real ones are steel. The CAD's
  own 500 kg/m³ for printed parts would make the robot 10.7 kg, not 13.9. All of these are
  provisional until the parts are weighed.
- **Torque and speed** use RobStride's rated/peak torques and no-load speeds. The ankle's torque is
  the RS00's divided by the four-bar's largest ratio, 1.22.
- **Armature** values are estimates. They scale the leg prototype's RS02 estimate by rotor
  inertia ~ mass^(5/3) and by gear ratio squared, because RobStride publishes no rotor inertia.
- **Joint ranges** are provisional, because the export has no stops. They start from typical
  kid-size ranges and are clipped where the CAD meshes meet as each joint sweeps from the zero pose.
  Combined poses are not swept.
- **Collision geometry** is one fitted box or cylinder per part that can meet another link. Around
  the hip and shoulder, boxes over the brackets met the torso's at angles where the meshes are still
  clear, so there only the motors carry one.
- **direction_sign** is +1 everywhere and unverified. The **IMU** is a placeholder at the pelvis
  origin.

### Design findings from the import

- **Hip adduction is 0.10 rad.** Rolling a leg inward, the lower half of the hip-yaw bracket swings
  toward the hip-pitch drive. The bracket and drive are 5 mm apart in the CAD, and they meet at
  5.7°. A walking gait usually wants more, so check the gap in the CAD.
- **The arms cannot hang straight down.** The shoulders are no wider than the hips, so a lowered
  forearm meets the hip at 1.32 rad from the T-pose. The home pose lowers the arms 1.2 rad.
- The knee flexes to 2.09 rad before the ankle rod reaches the hip-yaw motor. The ankle's toes-down
  stop is 0.74 rad, where the rod reaches the shin. The leg prototype's stop was 0.77 rad.

## SIL

`config/humanoid_prototype_1/` holds the stand config. The robot starts in a crouch (hip −0.4,
knee 0.8, ankle −0.4) and stands to the home pose (−0.2, 0.4, −0.2) over 2.5 s. PD alone holds it.
The inverted pendulum about the ankles needs m·g·h ≈ 48 N·m/rad, and the two ankles give 120.

When SIL starts in a pose that bends a linkage's output, `MujocoActuatorTransport` closes the loop
before the first step: it solves the MJCF's equality constraints with the named joints held. Before
this, the ankle's horn and rod started at zero, and the closure snapped them in on the first step
and knocked the robot over.
