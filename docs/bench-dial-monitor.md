# Bench dial monitor (read-only)

`bench_dial_monitor.launch.py` streams telemetry from the 10-motor dial bench (`bench_2s_10m`) into
`/joint_states` and RViz, with the motors left **IDLE**. It is read-only: `Stm32SerialTransport`
runs in `monitor_only` mode, so `activate()` only connects and every control cycle sends IDLE for
all motors. Nothing ever arms, so it is safe against a powered bench.

It runs the stock `controller_manager` with `HumanoidActuatorSystem` + `Stm32SerialTransport`
(`monitor_only: true`), a `joint_state_broadcaster`, `robot_state_publisher` with the generated
bench URDF, and RViz with `config/bench_dial.rviz`. **No MIT controller**, so no command interface
is ever claimed. The transport's joints map is the generated `config/bench_2s_10m/stm32_wiring.yaml`
(chain = bus-segment index, motor = slot); the bench description, safety manifest, and the
firmware's own config are generated from the same model (`model/robots/bench_2s_10m`).

## Prerequisites

- The master STM32 is flashed with the artifact-generated config
  (`model/robots/bench_2s_10m/generated/firmware_config.yaml`, committed in the firmware repo).
- A Zenoh router is running: `ros2 run rmw_zenoh_cpp rmw_zenohd`.
- The master's USB CDC device is reachable. In the dev container it is passed through under
  `/host-dev/` (default `serial_device:=/host-dev/robosoccer-master`); on another host use
  `/dev/robosoccer-master` or the right `/dev/ttyACM<n>`.

## Run

```bash
ros2 launch humanoid_bringup bench_dial_monitor.launch.py              # RViz on a host with a display
ros2 launch humanoid_bringup bench_dial_monitor.launch.py rviz:=false  # headless (check /joint_states only)
ros2 launch humanoid_bringup bench_dial_monitor.launch.py serial_device:=/dev/ttyACM0
```

RViz opens on the bench board; turn each dial by hand and watch its joint move. Ctrl-C to stop
(the transport deactivates and the port is released).

If the first bring-up stops with "3 consecutive bad cycles" within ~25 ms of activation, that is the
known intermittent first-contact failure (`docs/transport-stm32.md`, "Bench": about 1 start in 23).
Re-run; it is not a monitor-mode fault.

## Checklist

- [ ] **Each dial turns the matching joint in RViz.** Turn one dial at a time and confirm the
      expected joint moves, in the mapping below:

  | dial (bus/CAN) | joint | | dial (bus/CAN) | joint |
  |---|---|---|---|---|
  | slave0 / can 1 | `bench_j1` | | slave1 / can 6 | `bench_j6` |
  | slave0 / can 2 | `bench_j2` | | slave1 / can 7 | `bench_j7` |
  | slave0 / can 3 | `bench_j3` | | slave1 / can 8 | `bench_j8` |
  | slave0 / can 4 | `bench_j4` | | slave1 / can 9 | `bench_j9` |
  | slave0 / can 5 | `bench_j5` | | slave1 / can 10 | `bench_j10` |

- [ ] **`/joint_states` publishes at ~200 Hz.** Check with `ros2 topic hz /joint_states`
      (expect ~200 Hz; all 10 joints present in `ros2 topic echo --once /joint_states`).
- [ ] **Motors stay IDLE — no resistance when turning by hand.** Monitor mode never arms; a dial
      should spin freely, with no holding torque. (Confirm with
      `ros2 run humanoid_transport_stm32 stm32_link_probe <device>` after Ctrl-C: every motor
      `IDLE`, cause `NONE`.)
- [ ] **Direction and zero are UNVERIFIED.** Every joint uses `direction_sign: +1` and
      `zero_offset_rad: 0.0` as provisional placeholders (`measured_at`/`verified_by` are null in
      the model). A dial may read backwards or be offset from the model's zero; note which, per
      joint, so the values can be corrected and re-generated before the bench is ever commanded.
