# Bench bringup (`bench.launch.py`)

One launch for both dial benches, with two modes. Everything is per-robot and generated.

```bash
# slider view, no hardware (default): pose the board by hand in RViz
ros2 launch humanoid_bringup bench.launch.py                         # bench_1s_5m
ros2 launch humanoid_bringup bench.launch.py robot:=bench_2s_10m

# the real bench, motors armed, mit_controller active
ros2 launch humanoid_bringup bench.launch.py sim:=false
ros2 launch humanoid_bringup bench.launch.py robot:=bench_2s_10m sim:=false
```

## Arguments

- **`robot`** — `bench_1s_5m` (default, 5 motors / 1 slave) or `bench_2s_10m` (10 motors / 2 slaves).
  Selects the generated description (`config/<robot>_description.urdf`), the controller/hardware
  config (`config/<robot>/controllers.yaml`), and the transport wiring (`config/<robot>/stm32_wiring.yaml`).
- **`sim`** — `true` (default): **not connected to a bench.** Model-only slider view —
  `joint_state_publisher_gui` drives `/joint_states` into RViz so you can pose the board by hand;
  no `controller_manager`, no transport, no hardware. `false`: the **real** STM32 bench —
  `controller_manager` runs `HumanoidActuatorSystem` + `Stm32SerialTransport` with
  `MitImpedanceController` active. **The motors are armed and will move.**
- **`gui`** — sim only: `false` publishes a zero pose instead of the slider GUI.
- **`rviz`** — `false` to run headless (no X display).
- **`serial_device`** — the master's USB CDC device, used only when `sim:=false`. Default
  `/host-dev/robosoccer-master` (the devcontainer passthrough); on a bare host pass
  `/dev/robosoccer-master`.

Motion always comes from outside the launch — the slider GUI (sim) or `bench_trajectories`
(`sim:=false`). `MitImpedanceController` takes a position/velocity reference and streams the full MIT
tuple with the configured `kp=15, kd=1`; it does **not** interpolate (it holds the latest reference),
so a reference stream must be published at the control rate for smooth motion. There is no
joint_trajectory_controller — JTC commands only position and cannot claim the MIT gain interfaces
this actuator owns whole (ADR-001). The SafetyKernel (in the actuator system, not the controller)
does the envelope/finiteness/slew checks.

## Prerequisites (`sim:=false`)

- A Zenoh router: `ros2 run rmw_zenoh_cpp rmw_zenohd`.
- The master flashed with the matching firmware config (`1s_5m` / `2s_10m`) and reading READY
  (firmware smoke: chain(s) ALIVE, motors DISABLED). In the dev container the USB CDC device is
  `/host-dev/robosoccer-master` (the default).

> A failed `configure()` (busy port, or the bench not READY) aborts `ros2_control_node` — it is set
> to activate the hardware at startup. Free the port and confirm the smoke before launching.

## Moving the motors: `bench_trajectories` (two terminals)

With `bench.launch.py sim:=false` running in one terminal, publish a reference trajectory from
another. `bench_trajectories` captures the measured pose `q0` from `/joint_states`, centres the
active joints' trajectory on their own `q0`, holds every other joint at its `q0`, and refuses to
start if a joint is missing from `/joint_states` or the trajectory would exceed the URDF software
limits. It ends (or, on Ctrl-C, ramps back) exactly at `q0`.

```bash
# sine on one joint
ros2 run humanoid_bringup bench_trajectories --ros-args \
  -p mode:=sine -p joints:=[bench_j1] -p amplitude_rad:=0.2 -p frequency_hz:=0.25

# trapezoid ladder on one joint
ros2 run humanoid_bringup bench_trajectories --ros-args \
  -p mode:=trap -p joints:=[bench_j1] -p step_rad:=0.1
```

Common params: `rate_hz` (200), `return_s` (2.0), `record_csv` (path). sine: `amplitude_rad`,
`frequency_hz`, `duration_s`, `fade_s`. trap: `step_rad`, `ramp_s`, `hold_s`, `cycles`. On exit it
prints per-joint RMS/max tracking error, peak |effort|, direction match, and non-active drift
(flagged above 0.01 rad). Start small on the real bench (amplitude ~0.1–0.3 rad): the 3 N·m torque
cap limits how hard it can pull.

> `bench_trajectories` assumes the five `bench_j1..bench_j5` joints; it does not yet follow the
> `bench_2s_10m` joints `bench_j6..bench_j10`.

A draggable alternative on the real bench: run the slider GUI with its output remapped onto the
reference topic (interactive, but each drag is a step the controller does not interpolate):
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui \
  --ros-args -r joint_states:=/humanoid_mit_controller/joint_references
```

> **One reference source at a time.** Do not run `bench_trajectories` and the slider GUI together —
> they both publish to `/humanoid_mit_controller/joint_references`.
