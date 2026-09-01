# soccer-bot — RoboCup humanoid software stack

A **complete, layered** reference implementation of the RoboCup Humanoid software
architecture. Every layer of the real system is present and wired through the
**same** package boundaries, `ros2_control` abstraction, and DevOps flow used on the
full robot.

> **[→ Full documentation](docs/README.md)** — architecture, every design decision,
> interfaces, operations, and an honest status ledger.

> The robot model is currently a **minimal placeholder** — one `neck_pan` joint ·
> one monocular camera · one IMU — deliberately the smallest thing that still
> exercises **every layer**.

Growing the placeholder into the full humanoid is a matter of expanding the URDF
and policies, **not** re-architecting. Actuation is provided by **Robostride**
quasi-direct-drive actuators that close the impedance loop **onboard** (MIT mode):
the Jetson streams full MIT setpoints (`q*, qd*, kp, kd, τ_ff`) to an **STM32
Master** (safety + aggregation) over USB-CDC, which bridges to the actuators over
CAN. See [docs/09-firmware-and-actuators.md](docs/09-firmware-and-actuators.md).

## The layered architecture (frequency domains)

| Layer                  | Rate                          | Package(s)                                                         | Runs on          |
| ---------------------- | ----------------------------- | ------------------------------------------------------------------ | ---------------- |
| L5 Mission             | ~2 Hz                         | `game_controller_bridge`                                           | Jetson           |
| L4 Strategy            | 10 Hz tick / 4 Hz comms       | `soccer_strategy`, `soccer_teamcomm`                               | Jetson           |
| L3 Perception          | camera rate (~20 Hz)          | `soccer_perception`                                                | Jetson           |
| L3/L2 Localization     | 10 Hz MCL / 200 Hz EKF        | `soccer_localization`                                              | Jetson           |
| L1 Whole-body control  | 50 Hz MPC / 100 Hz controller | `soccer_control` (MPC + residual RL)                               | Jetson           |
| L0 Real-time actuation | actuator onboard              | `soccer-firmware/` submodule (STM32 Master/Slave → Robostride CAN) | STM32 + actuator |

Detail: [docs/01-system-overview.md](docs/01-system-overview.md).

The **cardinal rule**: slow cognition (vision, strategy) must never block the fast balance
loop. Each layer degrades gracefully — a crash in perception can't stall the actuator's
onboard control loop.

## Repository layout

```text
soccer-bot/
├── .github/workflows/        # CI: build, lint, test, multi-arch image
├── docs/                     # official documentation (start at docs/README.md)
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

| Command                     | Description                                                    |
| --------------------------- | -------------------------------------------------------------- |
| `make build`                | Full workspace build                                           |
| `make build-pkg pkg=<name>` | Rebuild one package (e.g. `make build-pkg pkg=soccer_bringup`) |
| `make clean`                | Remove build/install/log artifacts                             |
| `make sim`                  | Start 2-robot sim via Docker Compose                           |
| `make robot`                | Start real robot stack via Docker Compose                      |

See [Makefile](Makefile) for the full list.

## Documentation

**[docs/README.md](docs/README.md)** is the index. Most-used entry points:

| I want to…                             | Read                                                             |
| -------------------------------------- | ---------------------------------------------------------------- |
| Understand the system in 10 minutes    | [01 — System Overview](docs/01-system-overview.md)               |
| Know _why_ something is built this way | [02 — Design Decisions](docs/02-design-decisions.md) (D-01…D-44) |
| Publish to or subscribe from the graph | [03 — ROS 2 Interfaces](docs/03-ros-2-interfaces.md)             |
| Build, test or deploy                  | [12 — Build, Test & Deploy](docs/12-build-test-deploy.md)        |
| Bring up a robot or debug a failure    | [13 — Operations Runbook](docs/13-operations-runbook.md)         |
| Know what does **not** work yet        | [14 — Status & Roadmap](docs/14-status-and-roadmap.md)           |

The original working documents are preserved in [docs/archive/](docs/archive/README.md).

## Target platform

- **ROS 2 Jazzy Jalisco** (LTS → May 2029) · Ubuntu 24.04
- **Jetson Orin Nano Super 8 GB** validated (JetPack 7.2 / L4T R39.2 / CUDA 13.2) ·
  **Orin NX 16 GB** is the production target · **RTX** training workstation
- **ZED Mini** stereo camera (ZED SDK 5.4)
- **STM32 Master/Slave** bridge → **Robostride CAN** actuators (onboard MIT impedance)

Detail: [docs/11-compute-platform.md](docs/11-compute-platform.md).

## License

BSD 3-Clause — see [LICENSE](LICENSE).
