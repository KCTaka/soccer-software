# Unitree G1 walking policy

A 12-joint, velocity-conditioned walking policy for the G1 legs, trained by Unitree and run here
without retraining. It observes only a base-mounted IMU (gyro and attitude), joint encoders, a
velocity command and its own previous action. It uses no foot contact, no base linear velocity,
no magnetometer and no camera, so it needs nothing the robot is not planned to carry.

| | |
|---|---|
| Source | [unitreerobotics/unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym), `deploy/pre_train/g1/motion.pt`, last changed in commit `fb7514ad38fd7d0bb7761beda6c244befa61f9d6` |
| License | BSD-3-Clause, see `LICENSE` |
| `motion.pt` SHA-256 | `cf668f75b90d1abf73d2b87612a6e76bccc61ff7e083b63582d3f6aaa3c1759d` |
| `policy.onnx` SHA-256 | `d46d42000e022a145aad60aea7dd4af8ad080ed3c42dc6a9eb59ba0a0fca07ca` |
| Conversion | `tools/export_unitree_g1_policy.py`; matches the TorchScript model to 6e-6 over a 200-step recurrent rollout |

## Interface

`policy.onnx` holds an LSTM (47 to 64), Linear (64 to 32), ELU and Linear (32 to 12). Its LSTM
state is explicit: inputs `obs [1,47]`, `h [1,1,64]`, `c [1,1,64]`, outputs `action [1,12]`,
`h_out`, `c_out`. The state starts at zero, as in training.

Observation, 47 values, in this order (from `g1_env.py` `compute_observations`):

| Index | Count | Value |
|---|---|---|
| 0 | 3 | base angular velocity in the body frame, times `ang_vel_scale` |
| 3 | 3 | gravity direction in the body frame (projected gravity) |
| 6 | 3 | command (vx, vy, yaw rate) times `command_scale` |
| 9 | 12 | (joint position minus `default_angles`) times `dof_pos_scale` |
| 21 | 12 | joint velocity times `dof_vel_scale` |
| 33 | 12 | previous action |
| 45 | 2 | sin and cos of 2 pi times the gait phase |

Action: `target = default_angles + action_scale * action`, sent as a position reference with the
gains in `policy.yaml`. The policy runs at 50 Hz; the controller applies the MIT law at 200 Hz.

## Running it in SIL

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd        # once, in another shell
ros2 launch humanoid_bringup sil_walk.launch.py rviz:=false foxglove:=true command:='[0.3, 0.0, 0.0]'
```

`command` is [vx m/s, vy m/s, yaw rate rad/s], used when no `cmd_vel` has arrived for 0.5 s. Publish
`geometry_msgs/Twist` on `/walk_policy/cmd_vel` or remap `cmd_vel` to steer at run time. Each
component is limited to the trained range of 1.0.

The launch starts the simulation in the stance and holds it still until the first stiff command
(`mujoco_sim` parameters `initial_pose.*` and `hold_until_commanded`), then starts the policy once
every controller is active. The stance is only dynamically stable: held by PD alone it tips over
within a second, so there is deliberately no ramp into it in SIL.

## What differs from the training setup

- Training used a 12-DOF G1 whose waist and arms were rigid. Here they are actuated and held at
  their starting pose with stiff gains chosen for this deployment, not by Unitree.
- The upper body must be held stiffly. With the waist at 100 and the arms at 30 N*m/rad, the policy
  stands for about eight seconds and then falls; at 500 and 200 it walks. `policy.yaml` uses the
  latter.
- The policy asks for targets past the joint stops on about a fifth of its steps (the ankle roll
  range is +-15 degrees), because training never clipped them. `walk_policy` clips every target to
  the safety manifest's position limits, which the safety kernel would otherwise treat as a fault.
- Training ran in Isaac Gym with friction, mass and push randomization.
- The gait phase starts at zero when the policy starts, as at the start of a training episode.

## Verified in SIL

Through the full ROS stack (`sil_walk.launch.py`), from the stance at zero command and then
commanded forward. Speeds are measured over the run; the model's ground-truth pose is the
reference.

| Command | Result |
|---|---|
| vx 0.3 m/s | walked 44 s at 0.31 m/s, pelvis height steady at 0.77 m, no safety stops |
| vx 0.5 m/s | walked 25 s at 0.48 m/s, no safety stops |

Not yet tested: pushes, uneven ground, yaw tracking (heading wanders at zero yaw command), and any
hardware.
