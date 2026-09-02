# soccerbot — Implementation Walkthrough

> **What this is.** A complete, working scaffold of the new RoboCup Humanoid
> software architecture (from `docs/architecture/new_architecture_blueprint.md`
> and `docs/architecture/localization_strategy_report.md`), realized for the
> smallest robot that still exercises **every layer**: **one motor, one camera,
> one IMU**. It is a 1:1 structural mirror of the full system — same packages,
> same `ros2_control` boundary, same DevOps flow — just scaled down so it is
> runnable on a laptop today and grows into the 20-DOF humanoid without an
> architectural rewrite.

---

## 1. The robot we built: placeholder soccerbot

| Real humanoid           | soccerbot placeholder                   | Still exercises                                         |
| ----------------------- | --------------------------------------- | ------------------------------------------------------- |
| 20+ actuators           | **1 motor** (`neck_pan` revolute joint) | `ros2_control`, MPC reference, residual RL, onboard MIT |
| Multi-camera + depth    | **1 monocular camera**                  | detector, field-line seg, 3D projection                 |
| Full IMU + foot sensors | **1 body IMU**                          | Tier-1 EKF odometry                                     |
| Walk / kick / get-up    | **scan / track the ball**               | Behavior Tree, role auction, MPC modes                  |

The placeholder's job: stand on a small field, **pan its camera to find and track the
ball**, while **localizing on the field** and obeying the **GameController** —
coordinating roles with teammates through a **decentralized auction**. That
single behavior touches every layer L0–L5.

```mermaid
flowchart LR
    classDef hw fill:#ffe2b3,stroke:#333,color:#000;
    M["neck_pan motor"]:::hw
    C["monocular camera"]:::hw
    I["body IMU"]:::hw
    M -.pans.-> C
    I -.fixed to.-> base["base_link"]
    base -.neck_pan.-> C
```

---

## 2. Layered architecture as built

The cardinal rule from the blueprint — **isolate loops by frequency so slow
cognition never blocks the fast balance loop** — is realized as concrete ROS 2
packages and the `soccer-firmware` submodule. Each box below is a real directory in this repo.

```mermaid
flowchart TB
    classDef ext fill:#f9d5e5,stroke:#333,color:#000;
    classDef slow fill:#cfe8ff,stroke:#333,color:#000;
    classDef mid fill:#d5f5d5,stroke:#333,color:#000;
    classDef fast fill:#ffe2b3,stroke:#333,color:#000;
    classDef rt fill:#ffc9c9,stroke:#b00,stroke-width:2px,color:#000;

    GC["RoboCup GameController<br/>(UDP referee)"]:::ext

    subgraph L5["L5 MISSION · ~2 Hz"]
        GCB["game_controller_bridge<br/>parse 3838 / reply 3939"]:::slow
    end
    subgraph L4["L4 STRATEGY · 5-20 Hz"]
        BT["soccer_strategy<br/>BehaviorTree.CPP + role auction"]:::mid
        TEAM["soccer_teamcomm<br/>shared world model"]:::mid
    end
    subgraph L3["L3 PERCEPTION · 30-60 Hz"]
        VIS["soccer_perception<br/>detector · fieldline · projection"]:::slow
        MCL["soccer_localization (MCL)<br/>Tier-2 global field pose"]:::slow
    end
    subgraph L2["L2 STATE ESTIMATION · 100-400 Hz"]
        EKF["soccer_localization (EKF)<br/>Tier-1 odometry"]:::fast
    end
    subgraph L1["L1 WHOLE-BODY CONTROL · 50-100 Hz (Jetson)"]
        MPC["soccer_control<br/>mpc_node (reference)"]:::fast
        RLP["soccer_control<br/>ResidualRLController (TensorRT)"]:::fast
    end
    subgraph L0["L0 REAL-TIME · onboard (Robostride actuator)"]
        PD["soccer-firmware Master/Slave<br/>safety + CAN-FD bridge"]:::rt
        MOT["neck_pan Robostride · encoder · IMU<br/>onboard MIT impedance"]:::rt
    end

    GC --> GCB --> BT
    TEAM <--> BT
    VIS --> MCL --> BT
    VIS --> MPC
    BT -->|"ControlGoal: scan/track"| MPC
    MPC --> RLP
    EKF --> MPC
    EKF --> MCL
    RLP -->|"MIT setpoint @ ~50 Hz"| PD
    PD -->|"MIT cmd (CAN-FD)"| MOT
    MOT -->|"q, qd, τ, IMU"| EKF
    MOT --> PD
```

A crash in `soccer_perception` (L3) cannot stall `soccer_control` (L1) or the
actuator's onboard loop (L0): they are separate processes/targets communicating
over topics, and the Master watchdog zeroes torque independently if the Jetson
goes silent.

---

## 3. Repository map (what lives where)

```text
soccer-bot/
├── .github/workflows/ci.yml      # build · lint · test · multi-arch image
├── docs/
│   ├── IMPLEMENTATION.md          # ← this file
│   └── architecture/              # the two source blueprints (verbatim)
├── ros2_ws/src/
│   ├── soccer_msgs/               # [IDL]  interfaces (8 messages)
│   ├── soccer_description/        # [xacro] URDF + ros2_control + controllers.yaml
│   ├── soccer_hardware/           # [C++]  sim + real ros2_control plugins
│   ├── soccer_control/            # [C++]  mpc_node + ResidualRLController plugin
│   ├── soccer_perception/         # [Py]   detector · fieldline · projection
│   ├── soccer_localization/       # [Py]   ekf_node (Tier-1) · mcl_node (Tier-2)
│   ├── soccer_strategy/           # [C++]  BehaviorTree + decentralized role auction
│   ├── soccer_teamcomm/           # [Py]   TeamData over DDS (no master)
│   ├── game_controller_bridge/    # [Py]   UDP 3838/3939 ↔ /gc/game_state
│   └── soccer_bringup/            # [launch] per-robot namespacing + sim camera
├── soccer-firmware/                # [submodule] STM32 Master/Slave → Robostride CAN
├── sim/                           # [Py]   Isaac Lab task + DR + ONNX export
├── hardware/                      # CAD/PCB placeholders (Git LFS)
├── deploy/                        # Docker + compose + Ansible + MCU toolchain
└── tools/                         # mock GameController · camera calibration
```

### Package dependency graph

```mermaid
flowchart TD
    classDef base fill:#eee,stroke:#333,color:#000;
    classDef core fill:#cfe8ff,stroke:#333,color:#000;

    MSGS["soccer_msgs"]:::base
    DESC["soccer_description"]:::base
    HW["soccer_hardware"]:::core
    CTRL["soccer_control"]:::core
    PERC["soccer_perception"]:::core
    LOC["soccer_localization"]:::core
    STRAT["soccer_strategy"]:::core
    TEAM["soccer_teamcomm"]:::core
    GCB["game_controller_bridge"]:::core
    BRINGUP["soccer_bringup"]:::core

    MSGS --> CTRL & PERC & LOC & STRAT & TEAM & GCB
    DESC --> HW & CTRL & BRINGUP
    HW --> BRINGUP
    LOC --> STRAT
    PERC --> LOC
    PERC --> STRAT
    GCB --> STRAT
    TEAM --> STRAT
    CTRL --> BRINGUP
    STRAT --> BRINGUP
    PERC --> BRINGUP
    GCB --> BRINGUP
```

---

## 4. The `ros2_control` boundary — the linchpin of sim-to-real

The single most important design choice (blueprint §10): **the exact same ROS
graph runs in sim and on the robot — only the hardware plugin swaps.** That swap
is one xacro argument.

```mermaid
flowchart LR
    classDef j fill:#cfe8ff,stroke:#333,color:#000;
    classDef sim fill:#d5f5d5,stroke:#333,color:#000;
    classDef real fill:#ffe2b3,stroke:#333,color:#000;

    subgraph SAME["Identical above the boundary"]
        MPC["mpc_node"]:::j
        RL["ResidualRLController"]:::j
        EKF["ekf_node"]:::j
    end
    BND{{"ros2_control<br/>command/state interfaces<br/>neck_pan + imu_sensor"}}
    SAME --> BND
    BND -->|"sim:=true"| SIMHW["SoccerbotSimHardware<br/>(integrates joint in SW)"]:::sim
    BND -->|"sim:=false"| REALHW["SoccerbotSerialHardware<br/>(COBS/CRC16 USB-CDC)"]:::real
    REALHW <-->|"USB-CDC"| MCU["soccer-firmware Master"]:::real
```

- Both plugins live in `soccer_hardware` and export the **same** interfaces:
  `neck_pan` (`position`/`velocity`/`kp`/`kd`/`effort` command — the full MIT
  tuple; `position`/`velocity`/`effort` state) and a 10-channel `imu_sensor`. See
  [soccerbot.ros2_control.xacro](../ros2_ws/src/soccer_description/urdf/soccerbot.ros2_control.xacro).
- The real plugin speaks the **same COBS/CRC16-framed protocol** as the
  `soccer-firmware` Master, so the Jetson↔Master contract is faithful.
- **`effort` is a first-class state interface** because the residual-RL
  observation needs measured torque (τ from current sense) — non-negotiable for
  sim-to-real parity (blueprint §9).

---

## 5. Control: hierarchical MPC + bounded residual RL (blueprint §4)

Responsibility is split across three frequency bands so the trajectory stays
debuggable while RL absorbs disturbances:

```mermaid
flowchart TB
    classDef plan fill:#cfe8ff,stroke:#333,color:#000;
    classDef rl fill:#d5f5d5,stroke:#333,color:#000;
    classDef rt fill:#ffc9c9,stroke:#b00,stroke-width:2px,color:#000;

    GOAL["ControlGoal from Strategy<br/>(IDLE / SCAN / TRACK bearing)"]:::plan
    subgraph MID["mpc_node · 50 Hz · C++"]
        MPC["clamped-accel reference<br/>→ q*, qd*"]:::plan
    end
    subgraph RLL["ResidualRLController · 100 Hz · C++ plugin"]
        POL["policy π(obs) → Δq"]:::rl
        SUM["q_target = q* + clamp(Δq, ±0.20)"]:::rl
    end
    subgraph LOW["actuator · onboard · Robostride"]
        PD["τ = kp(q*-q)+kd(qd*-qd)+τ_ff<br/>(onboard impedance)"]:::rt
    end
    GOAL --> MPC --> SUM
    POL --> SUM
    SUM -->|"MIT setpoint"| PD --> MOT["neck_pan"]
    MOT -->|"q, qd, τ"| POL
    MOT --> MPC
```

| Loop              | Frequency   | Where                   | File                                                                                       |
| ----------------- | ----------- | ----------------------- | ------------------------------------------------------------------------------------------ |
| MPC reference     | 50 Hz       | Jetson (C++)            | [mpc_node.cpp](../ros2_ws/src/soccer_control/src/mpc_node.cpp)                             |
| Residual policy   | 100 Hz      | Jetson (C++ plugin)     | [residual_rl_controller.cpp](../ros2_ws/src/soccer_control/src/residual_rl_controller.cpp) |
| **MIT impedance** | **onboard** | **Robostride actuator** | `soccer-firmware` Master bridge (CAN-FD)                                                   |

**Why bounded residual, not end-to-end:** the residual is **hard-clamped to
±0.20 rad** in the controller, so a misbehaving policy can never destabilize the
joint. With no policy loaded the controller emits a **zero residual → pure MPC**,
the safe default before training. This is the corrected frequency contract from
the blueprint (the policy is ~50–100 Hz; the MIT impedance loop runs onboard the
Robostride actuator).

---

## 6. Perception: two decoupled vision problems (localization report §3.2)

The report's key correction — **object detection and field-line extraction are
different problems** — is realized as separate nodes:

```mermaid
flowchart TB
    classDef in fill:#ffe2b3,stroke:#333,color:#000;
    classDef det fill:#cfe8ff,stroke:#333,color:#000;
    classDef seg fill:#d5f5d5,stroke:#333,color:#000;
    classDef out fill:#f9d5e5,stroke:#333,color:#000;

    IMG["camera/image_raw"]:::in
    subgraph DET["detector_node (swappable)"]
        D["RF-DETR target (TensorRT)<br/>HSV ball fallback today"]:::det
    end
    subgraph SEG["fieldline_node"]
        S["semantic seg target<br/>HSV grass+white today"]:::seg
    end
    PROJ["projection_node<br/>ZED depth (flat-ground fallback)"]:::det
    IMG --> DET --> PROJ
    IMG --> SEG
    D -->|BoundingBoxes| PROJ
    PROJ -->|"ball/point (3D)"| STRAT["→ Strategy"]:::out
    PROJ -->|"goalpost features"| MCLO["→ MCL"]:::out
    S -->|"line point cloud (base frame)"| MCLO
```

- **`detector_node`** is a _swappable_ node. The production target is **RF-DETR**
  (DINOv2, Apache-2.0) → TensorRT for `ball`/`goalpost`/`robot`; today it ships a
  dependency-light **HSV ball detector** so the loop runs without a GPU. The
  published `BoundingBoxes` contract is unchanged when RF-DETR drops in — exactly
  the "node-level swap, not a rewrite" the report calls for.
- **`fieldline_node`** does **semantic** extraction (grass mask → white lines →
  project to base frame), emitting the `FieldFeatureArray` line cloud the MCL
  weighs.
- **`projection_node`** prefers **ZED depth** (accurate 3D) and falls back to the
  monocular **flat-ground homography** — replacing the legacy pipeline's biggest
  error source. Math is unit-tested in
  [test_projection.py](../ros2_ws/src/soccer_perception/test/test_projection.py).

---

## 7. Localization: two tiers (localization report §3, §8)

The report's central insight — **separate smooth local odometry from global
field pose**, reconciling "EKF vs particle filter" by using _both_ at different
tiers — is the structure here.

```mermaid
flowchart TB
    classDef sense fill:#ffe2b3,stroke:#333,color:#000;
    classDef odom fill:#cfe8ff,stroke:#333,color:#000;
    classDef map fill:#d5f5d5,stroke:#333,color:#000;

    IMU["IMU (imu/data)"]:::sense
    subgraph T1["TIER 1 · ekf_node · 200 Hz"]
        EKF["EKF: [px,py,θ,v,ω]<br/>smooth, drifts slowly"]:::odom
    end
    subgraph T2["TIER 2 · mcl_node · 10 Hz"]
        MCL["particle filter (300)<br/>lines+goalposts+circle<br/>likelihood-field map<br/>explorer particles"]:::map
    end
    IMU --> EKF -->|"odom→base_link"| T2
    FEAT["field_features (line cloud)"]:::map --> MCL
    MCL -->|"map→odom"| OUT["global field pose → Strategy"]
```

| Tier         | Node       | Filter              | Frame output       | Resolves symmetry?                         |
| ------------ | ---------- | ------------------- | ------------------ | ------------------------------------------ |
| 1 (odometry) | `ekf_node` | EKF                 | `odom → base_link` | no (not its job)                           |
| 2 (global)   | `mcl_node` | **Particle filter** | `map → odom`       | **yes** (multi-modal + explorer particles) |

The MCL is the report's recommended design, verified against the leading
open-source team (Bit-Bots):

```mermaid
flowchart LR
    classDef step fill:#d5f5d5,stroke:#333,color:#000;
    P["particles (x,y,θ)"] --> D["1 drift<br/>(odom delta)"]:::step
    D --> F["2 diffuse<br/>(noise)"]:::step
    F --> M["3 weight<br/>likelihood-field lookup O(1)"]:::step
    M --> R["4 resample<br/>+ explorer particles"]:::step
    R --> E["5 estimate<br/>→ map→odom"]:::step
    E --> P
```

- The **likelihood field** is a distance-transform image of the field lines baked
  once in [field_model.py](../ros2_ws/src/soccer_localization/soccer_localization/field_model.py)
  — the efficient form of Chamfer matching. Weighting an observed point is an O(1)
  lookup.
- **Explorer particles** re-seed a fraction each resample → automatic
  kidnapped-robot / penalty-return recovery (the thing a single-hypothesis filter
  cannot do).
- The filter is ROS-free and **unit-tested**: convergence near truth + explorer
  re-seeding in [test_mcl.py](../ros2_ws/src/soccer_localization/test/test_mcl.py).

---

## 8. Strategy: Behavior Trees + decentralized role auction (blueprint §6)

```mermaid
flowchart TB
    classDef bt fill:#d5f5d5,stroke:#333,color:#000;
    classDef role fill:#cfe8ff,stroke:#333,color:#000;

    GS["/gc/game_state"] --> ROOT
    WM["ball/point + mcl_pose"] --> ROOT
    subgraph TREE["per-robot Behavior Tree (role-specific XML)"]
        ROOT["IsNotHalted gate"]:::bt
        ROOT --> TRK["IsBallDetected → TrackBall"]:::bt
        ROOT --> SCAN["else ScanForBall"]:::bt
        ROOT --> IDLE["halted → Idle"]:::bt
    end
    subgraph AUC["decentralized auction (/team_data)"]
        BID["each robot bids cost=dist-to-ball"]:::role
        ASSIGN["deterministic assignment<br/>(goalie static, closest=striker)<br/>re-bid on dropout"]:::role
        BID --> ASSIGN --> ROOT
    end
```

- Roles are **XML trees** (`striker.xml`, `supporter.xml`, `goalie.xml`) loaded at
  launch and **hot-swapped** when the auction changes a robot's role — swapping
  behavior is a config change, not code.
- The whole tree is gated by **`IsNotHalted`**: a non-PLAYING GameController state
  or a penalty freezes the robot — the referee is the **ultimate authority**.
- The **auction is deterministic** (`assign_role`): every robot runs the identical
  computation over all bids on the global `/team_data` topic and reaches the same
  result with **no master**. Dropouts trigger automatic re-assignment. This logic
  is **unit-tested** (gtest) in
  [test_role_auction.cpp](../ros2_ws/src/soccer_strategy/test/test_role_auction.cpp).

---

## 9. Mission: GameController bridge (blueprint §7)

```mermaid
sequenceDiagram
    participant GC as GameController
    participant BR as gc_bridge_node
    participant BT as strategy_node
    loop 2 Hz
        GC-->>BR: UDP broadcast :3838 (control data)
        BR->>BT: /gc/game_state (PLAYING / penalized)
    end
    loop ~1 Hz
        BR-->>GC: UDP unicast :3939 (alive)
    end
    Note over BT: HALT/PENALIZED overrides all AI
```

`game_controller_bridge` parses port 3838, publishes the clean `GameState`, and
sends the mandatory 3939 return so the GC does not flag the robot. The wire
format is shared with the **mock GameController** test tool, and the round-trip
(including per-player penalties) is **unit-tested** in
[test_gc_protocol.py](../ros2_ws/src/game_controller_bridge/test/test_gc_protocol.py).

---

## 10. Multi-robot: fully decentralized (blueprint §8)

```mermaid
flowchart TB
    classDef r fill:#d5f5d5,stroke:#333,color:#000;
    classDef net fill:#f9d5e5,stroke:#333,color:#000;
    WIFI(("/team_data over DDS<br/>(5 GHz, CycloneDDS)")):::net
    R1["/robot_1 — full stack"]:::r <--> WIFI
    R2["/robot_2 — full stack"]:::r <--> WIFI
    R3["/robot_3 — full stack"]:::r <--> WIFI
```

Every robot runs the **identical graph** from `robot.launch.py`; identity is just
a **ROS namespace** (`/robot_1` …) + `player_id`. `team.launch.py` replicates it N
times. They share only the lightweight `/team_data` (world model + bids), so
losing one robot degrades the team gracefully — no single point of failure.

---

## 11. The motor firmware (`soccer-firmware` submodule, blueprint §9)

The hard-real-time code lives in the **`soccer-firmware`** submodule (STM32
Master/Slave). On our side the Master **aggregates** all joints, enforces the
**watchdog**, and **bridges** to the actuators over CAN-FD. The impedance law
itself runs **onboard each Robostride actuator** — we only stream setpoints.

```mermaid
flowchart LR
    classDef rt fill:#ffc9c9,stroke:#b00,stroke-width:2px,color:#000;
    RX["recv MIT cmd frame<br/>(COBS + CRC16)"]:::rt --> WD["feed watchdog"]:::rt
    WD --> SILENT{"bus silent?"}:::rt
    SILENT -->|yes| ZERO["τ = 0 (safe halt)"]:::rt
    SILENT -->|no| FWD["forward MIT setpoints<br/>over CAN-FD"]:::rt
    FWD --> ACT["Robostride: onboard<br/>impedance + FOC"]:::rt
    ACT --> TX["return JointState + IMU<br/>(q, qd, τ, temp, fault)"]:::rt
```

- The Master watchdog zeroes torque on bus silence **independently of the
  Jetson** — the last line of safety. The host plugin (`SoccerbotSerialHardware`)
  adds a second, redundant watchdog.
- Firmware unit tests now live in the **`soccer-firmware`** submodule, not this
  repo. The wire contract is specified in
  [jetson_master_protocol.md](architecture/jetson_master_protocol.md).

---

## 12. Sim-to-real pipeline (blueprint §10)

```mermaid
flowchart LR
    classDef sim fill:#d5f5d5,stroke:#333,color:#000;
    classDef bridge fill:#cfe8ff,stroke:#333,color:#000;
    classDef real fill:#ffe2b3,stroke:#333,color:#000;

    URDF["URDF (soccer_description)<br/>single source of truth"]:::sim --> ENV
    ENV["sim/tasks · Isaac Lab task<br/>+ domain randomization"]:::sim --> TRAIN["train_residual.py"]:::sim
    TRAIN --> ONNX["export_onnx.py → .onnx"]:::bridge
    ONNX --> TRT["trtexec → .engine (Jetson)"]:::bridge
    TRT --> CTRL["ResidualRLController.policy_path"]:::real
    REAL["robot telemetry"]:::real -->|sysID| ENV
```

- The RL task observation is **exactly** what the controller feeds at runtime
  (`[q, qd, q_ref, qd_ref, gyro_z, command]`), so a trained policy drops straight
  in. The residual exists to **absorb model error** — demonstrated by the env's
  unmodeled offset the policy learns to cancel within its bound.
- **Domain randomization** (mass, friction, latency, sensor noise) is applied each
  episode so the policy is forced to be robust.
- The pipeline runs **without Isaac/GPU** via a numpy fallback env, so the whole
  train → export chain is exercisable on any laptop; the production path is the
  Isaac Lab task config on the workstation.
- Isaac Lab runs on the x86 workstation and exports ONNX — **decoupled from the
  robot's ROS distro** (the §3.2 nuance).

---

## 13. DevOps: build once, deploy everywhere (blueprint §12)

```mermaid
flowchart LR
    classDef dev fill:#cfe8ff,stroke:#333,color:#000;
    classDef ci fill:#d5f5d5,stroke:#333,color:#000;
    classDef reg fill:#ffe2b3,stroke:#333,color:#000;
    classDef bot fill:#ffc9c9,stroke:#b00,color:#000;

    DEV["git push"]:::dev --> CI["GitHub Actions<br/>build · lint · test · arm64 image"]:::ci
    CI --> REG["GHCR<br/>versioned runtime image"]:::reg
    REG --> ANS["ansible-playbook deploy.yml"]:::reg
    ANS -->|SSH| B1["robot_1: pull + restart"]:::bot
    ANS -->|SSH| B2["robot_2: pull + restart"]:::bot
```

- CI ([ci.yml](../.github/workflows/ci.yml)) builds + tests the colcon workspace,
  lints Python, runs the **firmware host tests**, and (on `main`) pushes a **lean
  arm64 runtime image** containing only the built `install/` space — no source, no
  CAD, no sim assets.
- **Ansible** ([deploy.yml](../deploy/ansible/deploy.yml)) pulls a pinned image
  tag onto every robot and restarts the container in one command. Secrets stay in
  CI/vault, never committed.
- **Git LFS** ([.gitattributes](../.gitattributes)) tracks USD/CAD/ONNX so the
  runtime clone stays lean.

---

## 14. How to run it

```bash
# Build the workspace (ROS 2 Jazzy)
cd ros2_ws && colcon build --symlink-install && source install/setup.bash

# One robot in simulation (sim hardware plugin + synthetic camera)
ros2 launch soccer_bringup robot.launch.py robot_name:=robot_1 sim:=true

# Two-robot decentralized scrimmage
ros2 launch soccer_bringup team.launch.py num_robots:=2
python tools/mock_gamecontroller.py --state playing   # kick off PLAYING

# Whole sim in Docker
cd deploy/compose && docker compose -f sim.compose.yaml up
```

What you observe: the synthetic camera shows an orange ball; `detector_node`
finds it; `projection_node` emits its 3D point; `strategy_node` (as striker)
commands **TRACK**; `mpc_node` + `ResidualRLController` pan `neck_pan` until the
ball centers — the camera **visibly follows the ball**, closing the full L5→L0
loop. Meanwhile `fieldline_node` + `mcl_node` localize on the field, and
`gc_bridge_node` gates everything on the GameController state.

### 14.1 Who can run what

The team is mixed, and only one machine has the camera. Map each developer to
what they can run so nobody is blocked on hardware.

| Environment | Can do | Cannot do |
| ----------- | ------ | --------- |
| **No NVIDIA GPU** (Windows 11 + WSL2, Apple-Silicon Mac) | Write code, full CPU sim, replay rosbags through perception / MCL / strategy, Foxglove viz, `colcon test` | Open a ZED, build TensorRT engines, Isaac Lab training |
| **NVIDIA GPU laptop or desktop** | Everything above, plus opening a ZED locally, replaying SVOs, Isaac Lab training and ONNX export, x86 TensorRT experiments | Build the **on-robot** engine — that must happen on the Jetson (§13) |
| **The shared Jetson** | Live ZED capture, arm64 TensorRT engines, MCU serial and motor hardware-in-the-loop | — |

Notes that matter in practice:

- On **Windows**, do all ROS work inside WSL2 Ubuntu 24.04 or the dev container.
  Native Windows is not a ROS 2 target.
- An **Apple-Silicon Mac** is arm64 — the same architecture as the Jetson — so
  the arm64 Jazzy dev container runs natively. No CUDA, so no ZED or TensorRT,
  but full sim, perception-on-rosbag, control and strategy all work.
- **Reserve the Jetson for what only it can do.** Push everything else off it.
  Two developers can share it simultaneously by exporting distinct
  `ROS_DOMAIN_ID` values (the default is 42; hand out 41, 43, …). Only live
  camera and motor time needs exclusive booking.

### 14.2 Testing at three scales

| Scale | What runs | Where | Command |
| ----- | --------- | ----- | ------- |
| **Unit** | One node or one function, no ROS graph needed | Anywhere, including a Mac | `colcon test`, or plain `pytest` for pure logic |
| **Subsystem** | A pipeline fed from a rosbag or SVO — no GPU, no camera | Anywhere | `ros2 bag play` into `detector_node` / `mcl_node` |
| **Full stack** | Everything, in sim or on hardware | Sim anywhere; hardware on the Jetson | `robot.launch.py sim:=true`, or `make robot` |

A useful middle option: run the ZED node on the Jetson and your node under test
on your laptop, in the same DDS graph. Set a matching `ROS_DOMAIN_ID` and
`RMW_IMPLEMENTATION` on both — see
[new_architecture_blueprint.md §8.1](architecture/new_architecture_blueprint.md).

Before any hardware run, `make robot` invokes `tools/preflight.sh`. See
[TROUBLESHOOTING.md §10](TROUBLESHOOTING.md) for what it checks and why.

---

## 15. What was verified

| Check                             | How                                 | Result              |
| --------------------------------- | ----------------------------------- | ------------------- |
| All Python compiles               | `py_compile` over the tree          | ✅                  |
| Camera projection math            | depth + flat-ground + horizon cases | ✅                  |
| Field likelihood map              | on-line ≫ off-line likelihood       | ✅ (1.000 vs 0.000) |
| GameController protocol           | pack/parse round-trip + penalties   | ✅                  |
| Sim train → save                  | numpy env + ES + weights file       | ✅                  |
| MCL convergence + explorer reseed | pytest (`test_mcl.py`)              | ✅ logic            |
| Role auction (4 cases)            | gtest (`test_role_auction.cpp`)     | ✅ logic            |
| Firmware control + watchdog       | in `soccer-firmware` submodule      | — moved out         |

> Note: `colcon build` and the C++/gtest suites require a ROS 2 Jazzy + Linux
> toolchain (the CI image); on this Windows host the ROS-free Python/numeric logic
> was executed directly and passes. The C++ and ROS nodes are written against the
> Jazzy APIs and wired through CI.

---

## 16. Mapping every blueprint/report concept to where it lives

| Concept (source)                                                 | Realized in                                                    |
| ---------------------------------------------------------------- | -------------------------------------------------------------- |
| Layered frequency domains (BP §2)                                | the L0–L5 package split                                        |
| `ros2_control` sim/real boundary (BP §10)                        | `soccer_hardware` two plugins + `soccerbot.ros2_control.xacro` |
| Hierarchical MPC + bounded residual RL (BP §4)                   | `soccer_control` (`mpc_node` + `ResidualRLController`)         |
| MIT impedance onboard + Master watchdog (BP §4, §9)              | Robostride actuator + `soccer-firmware`                        |
| Current-sense τ in state vector (BP §9)                          | `effort` state interface end-to-end                            |
| Proprio/extero split (BP §5)                                     | `ekf_node` (proprio) vs `soccer_perception` (extero)           |
| RF-DETR detector, swappable (report §3.2)                        | `detector_node` (HSV fallback, RF-DETR target)                 |
| Field-line semantic seg (report §3.2)                            | `fieldline_node`                                               |
| ZED depth projection (report §6)                                 | `projection_node`                                              |
| Tier-1 EKF odometry (report §3)                                  | `ekf_node`                                                     |
| Tier-2 MCL + likelihood field + explorer particles (report §3.1) | `mcl_node` + `field_model.py`                                  |
| Behavior Trees + Groot2 (BP §6)                                  | `soccer_strategy` + `trees/*.xml`                              |
| Decentralized role auction, no master (BP §6, §8)                | `role_auction.cpp` + `/team_data`                              |
| GameController UDP 3838/3939 (BP §7)                             | `game_controller_bridge`                                       |
| Per-robot namespacing (BP §8, §11)                               | `soccer_bringup` launch files                                  |
| Isaac Lab + DR + teacher/student + ONNX (BP §10)                 | `sim/`                                                         |
| Monorepo + Git LFS (BP §11)                                      | repo layout + `.gitattributes`                                 |
| Docker + GHCR + Ansible + CI (BP §12)                            | `deploy/` + `.github/workflows/ci.yml`                         |
| ROS 2 Jazzy target (BP §3.1)                                     | CI image, Dockerfiles, package manifests                       |

---

## 17. Growing the placeholder into the full humanoid

Because the architecture — not just the code — is what was scaffolded, scaling up
is **additive**:

1. **URDF/USD**: add the 20 leg/arm joints to `soccer_description`; the
   `ros2_control` block grows, nothing above it changes.
2. **Control**: replace `mpc_node`'s single-joint reference with a real
   ZMP/whole-body MPC; the `ResidualRLController` interface is unchanged.
3. **Firmware**: one MIT command/state pair per joint; the Master aggregates all
   joints over CAN-FD — the protocol and watchdog are already there.
4. **Perception**: drop RF-DETR + a seg net into the two existing nodes; contracts
   unchanged.
5. **Strategy**: richer trees (`approach → align → kick`); the auction and gating
   are unchanged.

Everything else — localization tiers, team comms, GameController, namespacing,
sim-to-real, DevOps — is already the full-system design.
