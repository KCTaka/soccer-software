# soccer-bot — RoboCup humanoid software stack

A **complete, layered** reference implementation of the RoboCup Humanoid software
architecture described in `docs/architecture/`. Every layer of the real system is
present and wired through the **same** package boundaries, `ros2_control`
abstraction, and DevOps flow used on the full robot.

> The robot model is currently a **minimal placeholder** — one `neck_pan` joint ·
> one monocular camera · one IMU — deliberately the smallest thing that still
> exercises **every layer**.

Growing the placeholder into the full humanoid is a matter of expanding the URDF
and policies, **not** re-architecting. Actuation is provided by **Robostride**
quasi-direct-drive actuators that close the impedance loop **onboard** (MIT mode):
the Jetson streams full MIT setpoints (`q*, qd*, kp, kd, τ_ff`) to an **STM32
Master** (safety + aggregation) over USB-CDC, which bridges to the actuators over
CAN. See [docs/architecture/jetson_master_protocol.md](docs/architecture/jetson_master_protocol.md).

## The layered architecture (frequency domains)

| Layer                  | Rate               | Package(s)                                                         | Runs on          |
| ---------------------- | ------------------ | ------------------------------------------------------------------ | ---------------- |
| L5 Mission             | ~2 Hz              | `game_controller_bridge`                                           | Jetson           |
| L4 Strategy            | 5–20 Hz            | `soccer_strategy`, `soccer_teamcomm`                               | Jetson           |
| L3 Perception          | 30–60 Hz           | `soccer_perception`                                                | Jetson           |
| L3/L2 Localization     | 30–60 / 100–400 Hz | `soccer_localization` (MCL + EKF)                                  | Jetson           |
| L1 Whole-body control  | 50–100 Hz          | `soccer_control` (MPC + residual RL)                               | Jetson           |
| L0 Real-time actuation | actuator onboard   | `soccer-firmware/` submodule (STM32 Master/Slave → Robostride CAN) | STM32 + actuator |

The **cardinal rule**: slow cognition (vision, strategy) must never block the fast balance
loop. Each layer degrades gracefully — a crash in perception can't stall the actuator's
onboard control loop.

## Repository layout

```text
soccer-bot/
├── .github/workflows/        # CI: build, lint, test, multi-arch image
├── docs/                     # architecture blueprints + IMPLEMENTATION.md
├── ros2_ws/src/              # ROS 2 workspace (deployed to robots)
│   ├── soccer_msgs/          # custom interfaces (IDL)
│   ├── soccer_description/   # URDF/xacro + ros2_control tags
│   ├── soccer_hardware/      # [C++] ros2_control HW interfaces (sim + real)
│   ├── soccer_control/       # [C++] MPC + residual-RL runner
│   ├── soccer_perception/    # [Py]  detector + field-line seg + 3D projection
│   ├── soccer_localization/  # [Py]  Tier-1 EKF + Tier-2 MCL particle filter
│   ├── soccer_strategy/      # [C++] BehaviorTree.CPP + role auction
│   ├── soccer_teamcomm/      # [Py]  decentralized world model + role bids
│   ├── game_controller_bridge/ # [Py] UDP 3838/3939 ↔ /gc/game_state
│   └── soccer_bringup/       # launch + params + per-robot namespacing
├── soccer-firmware/          # [submodule] STM32 Master/Slave → Robostride CAN actuators
├── sim/                      # Isaac Lab task + ONNX→TensorRT export
├── hardware/                 # CAD / PCB placeholders (Git LFS)
├── deploy/                   # docker + compose + ansible
└── tools/                    # dev scripts, calibration, dataset tooling
```

## Quick start

```bash
# 1. Build the ROS 2 workspace
make build

# 2. Bring up ONE robot in simulation
cd ros2_ws && source install/setup.bash
ros2 launch soccer_bringup robot.launch.py robot_name:=robot_1 sim:=true

# 3. Bring up a 2-robot scrimmage with the GameController bridge
ros2 launch soccer_bringup team.launch.py num_robots:=2

# Or use Docker for the whole multi-robot sim:
make sim
```

### Build commands

| Command | Description |
|---|---|
| `make build` | Full workspace build |
| `make build-pkg pkg=<name>` | Rebuild one package (e.g. `make build-pkg pkg=soccer_bringup`) |
| `make clean` | Remove build/install/log artifacts |
| `make sim` | Start 2-robot sim via Docker Compose |
| `make robot` | Start real robot stack via Docker Compose |

See [Makefile](Makefile) for the full list.

## Documentation

- [docs/IMPLEMENTATION.md](docs/IMPLEMENTATION.md) — full walkthrough of what was built and why (with diagrams).
- [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) — silent failure modes, measured performance profile, and the detector roadmap. **Read this before deploying to hardware.**
- [docs/zed_jetson_integration.md](docs/zed_jetson_integration.md) — as-built ZED Mini + Jetson Docker integration.
- [docs/architecture/new_architecture_blueprint.md](docs/architecture/new_architecture_blueprint.md) — the source blueprint.
- [docs/architecture/localization_strategy_report.md](docs/architecture/localization_strategy_report.md) — localization strategy.

## Target platform

- **ROS 2 Jazzy Jalisco** (LTS → 2029) · Ubuntu 24.04
- **Jetson Orin NX / Thor** onboard · **RTX** training workstation
- **STM32 Master/Slave** bridge → **Robostride CAN** actuators (onboard MIT impedance)

## License

BSD 3-Clause — see [LICENSE](LICENSE).
