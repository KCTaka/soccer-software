# RoboCup Humanoid — Modern Architecture & Workflow Blueprint

> **Purpose:** A vendor-honest, research-backed conclusion for the _new_ team repository and engineering workflow. This document synthesizes the two prior proposals (`new_flow_proposal.md`, `new-architecture-conversation.md`), corrects their technical errors, and folds in your confirmed preferences plus fresh **June 2026** research.
>
> **Scope:** 1–3 nearly-identical autonomous humanoids · NVIDIA Isaac ecosystem · hierarchical MPC + residual RL control · RoboCup Humanoid GameController · ~$10,000 budget.

---

> **Update (2026-06-27) — L0 actuation revised.** The original plan put a custom
> **1 kHz PD loop on an STM32/Teensy MCU** (`firmware/motor_controller`). The robot
> now uses **Robostride** quasi-direct-drive actuators that **close the MIT
> impedance loop onboard**; an **STM32 Master/Slave** bridge (the `soccer-firmware`
> submodule, replacing `firmware/`) handles safety, the watchdog, telemetry
> aggregation, and the CAN-FD link. The Jetson streams the full MIT setpoint
> (`q*, qd*, kp, kd, τ_ff`) rather than a position target. The blueprint's core
> argument is **unchanged** — the hard-real-time loop stays **off** the Linux box —
> only its _location_ moved from a custom MCU to the actuator. Wherever this doc
> says "1 kHz PD on the MCU," read "onboard MIT impedance on the Robostride; Master
> = safety + aggregation." Details:
> [jetson_master_protocol.md](jetson_master_protocol.md) ·
> [soccer_hardware_rewrite.md](soccer_hardware_rewrite.md).

---

## 0. Confirmed Decisions & Recommendations (Read This First)

| Area                      | Your Choice                     | My Recommendation (with research)                                                                 | Status             |
| ------------------------- | ------------------------------- | ------------------------------------------------------------------------------------------------- | ------------------ |
| **Simulation / Learning** | NVIDIA Isaac Lab                | ✅ Isaac Lab (+ Isaac Sim). Use teacher→student distillation + domain randomization.              | Locked             |
| **Locomotion control**    | Hierarchical: MPC + residual RL | ✅ Excellent and modern. MPC = reference, RL = disturbance absorber, PD = 1 kHz on MCU.           | Locked             |
| **Training compute**      | Buy GPU workstation             | ✅ RTX 4090/5090 workstation. Owning beats cloud for continuous RL.                               | Locked             |
| **Onboard compute**       | Jetson Orin NX / AGX            | ⚠️ See **§3.2** — the _latest_ Isaac ROS moved to **Jetson Thor / JetPack 7.1**. Decision needed. | Needs validation   |
| **Fleet**                 | 1–3, nearly identical           | ✅ Fully decentralized, namespaced, no master robot.                                              | Locked             |
| **ROS 2 distro**          | Humble now, open to upgrade     | 🔶 **Recommend Jazzy Jalisco (LTS → 2029).** Humble dies **May 2027**.                            | Recommended change |
| **Languages**             | C++ realtime, Python ML         | ✅ C++ for `ros2_control`/BT.CPP/MPC, Python for perception/RL/tooling.                           | Locked             |

> **The single most important open decision** is the **ROS distro ↔ Jetson generation coupling** (§3.2). Everything else is settled.

---

## 1. Critical Analysis of the Two Prior Proposals

You asked me to weigh both documents and **double-verify the logic** of `new_flow_proposal.md`. Here is the honest audit.

### 1.1 `new_flow_proposal.md` (had old-repo context)

| Claim                                                               | Verdict       | Correction / Note                                                                                                                                                                               |
| ------------------------------------------------------------------- | ------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Split **Proprioception** (internal) vs **Exteroception** (external) | ✅ Correct    | This is the right mental model and we keep it.                                                                                                                                                  |
| **RL policy runs at 500–1000 Hz**                                   | ❌ **Wrong**  | Modern humanoid RL **policies run at ~50 Hz** (some 100–200 Hz). The **1 kHz loop is the low-level PD/torque controller**, not the neural net. Conflating them is the proposal's biggest error. |
| State estimation EKF at **500–1000 Hz**                             | ⚠️ Misleading | IMU streams fast, but `robot_localization` EKF typically fuses at **100–400 Hz**. 1 kHz fusion is atypical.                                                                                     |
| RL as a `ros2_control` controller plugin, "zero network latency"    | ⚠️ Half-right | Good _intent_ (run policy close to hardware). But the policy is **~50 Hz**; the **1 kHz PD belongs on the microcontroller**, not the Jetson.                                                    |
| YOLO + TensorRT on ZED feed                                         | ✅ Correct    | Keep.                                                                                                                                                                                           |
| **Behavior Trees over FSM**                                         | ✅ Correct    | Keep.                                                                                                                                                                                           |
| `ros2_control` as the sim/real abstraction boundary                 | ✅ Correct    | Keep — this is the linchpin of sim-to-real parity.                                                                                                                                              |

### 1.2 `new-architecture-conversation.md` (no old-repo context, Gemini)

| Claim                                                     | Verdict              | Correction / Note                                                                                                                                                         |
| --------------------------------------------------------- | -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ROS 2 + Docker + monorepo + micro-ROS + Ansible           | ✅ Solid             | Professional and current; we adopt it.                                                                                                                                    |
| **Don't wire motors to Jetson; use an MCU for 1 kHz PID** | ✅ Correct           | This is exactly why the 1 kHz loop is _not_ on the Jetson.                                                                                                                |
| **Heartbeat safe-halt** if Jetson process dies            | ✅ Correct           | MCU watchdog must zero torque on bus silence.                                                                                                                             |
| **No single master robot** (decentralized)                | ✅ Correct           | Matches RoboCup rules — robots must be autonomous on-field.                                                                                                               |
| First answer's **off-field "Coach" issuing commands**     | ❌ Rules conflict    | Humanoid League forbids external computation/human control during play. Only **robot↔robot team communication** is allowed. The revised answer correctly drops the coach. |
| **STP (Skills/Tactics/Plays)**                            | ⚠️ Borrowed from SSL | STP is from the _wheeled_ Small-Size League. For Humanoid, per-robot **Behavior Trees + lightweight role negotiation** is the better fit.                                 |
| **Isaac over MuJoCo** for sensor rendering                | ✅ Aligns            | Historically true; and you chose Isaac Lab, so moot. (Note: MuJoCo + NVIDIA _Newton_ are converging.)                                                                     |

**Net:** Both proposals broadly agree on the modern stack (ROS 2 · `ros2_control` · Behavior Trees · Docker · Isaac · decentralized fleet). The corrections that matter: **fix the control-loop frequencies**, **kill the off-field coach**, and **prefer Behavior Trees + role negotiation over STP**.

---

## 2. The Big Picture — Layered Architecture & Frequency Domains

The cardinal rule of professional robotics: **isolate loops by frequency**. Heavy, slow cognition (vision, strategy) must never block the fast, real-time balance loop.

```mermaid
flowchart TB
    classDef ext fill:#f9d5e5,stroke:#333,stroke-width:1px,color:#000;
    classDef slow fill:#cfe8ff,stroke:#333,stroke-width:1px,color:#000;
    classDef mid fill:#d5f5d5,stroke:#333,stroke-width:1px,color:#000;
    classDef fast fill:#ffe2b3,stroke:#333,stroke-width:1px,color:#000;
    classDef rt fill:#ffc9c9,stroke:#b00,stroke-width:2px,color:#000;

    GC["RoboCup GameController<br/>(UDP referee)"]:::ext

    subgraph L5["MISSION · ~2 Hz"]
        GCB["game_controller_bridge<br/>parse 3838 / reply 3939"]:::slow
    end

    subgraph L4["STRATEGY · 5-20 Hz"]
        BT["Behavior Tree engine<br/>role + play selection"]:::mid
        TEAM["team_comm<br/>shared world model"]:::mid
    end

    subgraph L3["PERCEPTION · 30-60 Hz"]
        VIS["Vision: YOLO + TensorRT"]:::slow
        LOC["Localization: field + ball 3D"]:::slow
    end

    subgraph L2["STATE ESTIMATION · 100-400 Hz"]
        EKF["EKF (robot_localization)<br/>IMU + kinematics + visual odom"]:::fast
    end

    subgraph L1["WHOLE-BODY CONTROL · 50-100 Hz (on Jetson)"]
        MPC["MPC / footstep planner<br/>(reference trajectory)"]:::fast
        RLP["Residual RL policy<br/>(TensorRT) — disturbance absorber"]:::fast
    end

    subgraph L0["REAL-TIME ACTUATION · 1000 Hz (on MCU)"]
        PD["PD / torque control + watchdog"]:::rt
        MOT["Motors · Encoders · Current sense · IMU"]:::rt
    end

    GC --> GCB --> BT
    TEAM <--> BT
    VIS --> LOC --> BT
    LOC --> MPC
    BT -->|"goal: walk/kick/stand"| MPC
    MPC --> RLP
    EKF --> MPC
    EKF --> RLP
    EKF --> BT
    RLP -->|"joint targets @ ~50 Hz"| PD
    PD -->|"torque @ 1 kHz"| MOT
    MOT -->|"raw IMU/joint/current"| EKF
    MOT -->|"CAN/serial telemetry"| PD
```

**Why this layering is the standard:** a segfault in the vision node (L3) cannot stall the balance controller (L1) or the MCU safety loop (L0). Each layer degrades gracefully.

---

## 3. Technology Stack — Decisions & Justification

### 3.1 ROS 2 Distribution — **Recommend Jazzy Jalisco**

```mermaid
timeline
    title ROS 2 LTS Support Windows (verified June 2026)
    Humble Hawksbill : Released 2022 : EOL May 2027 (your current distro)
    Jazzy Jalisco : Released 2024 : EOL May 2029 (RECOMMENDED)
    Kilted Kaiju : Released 2025 : EOL Dec 2026 (non-LTS, avoid)
    Lyrical Luth : Released May 2026 : EOL 2031 (too new, thin support)
```

- You are starting a **multi-year, multi-robot** program. Building it on **Humble (EOL May 2027)** means a forced migration mid-competition-cycle.
- **Jazzy** is the current LTS (→2029), and critically, **the latest NVIDIA Isaac ROS and ZED SDK v5 both target Jazzy / Ubuntu 24.04**.
- Avoid Kilted (non-LTS, dies Dec 2026) and Lyrical (released weeks ago; third-party support is thin).

### 3.2 ⚠️ The Decision You Must Make: Jetson Generation ↔ Distro Coupling

Research surfaced a hard coupling that the prior docs missed. NVIDIA's **latest Isaac ROS (release-4.x, Apr 2026)** is documented as: _"All Isaac ROS packages are designed and tested to be compatible with ROS 2 Jazzy"_ — on **JetPack 7.1 / Jetson Thor**. Orin runs the **older** JetPack 6 (Ubuntu 22.04 = Humble-era).

```mermaid
flowchart LR
    classDef a fill:#d5f5d5,stroke:#333,color:#000;
    classDef b fill:#cfe8ff,stroke:#333,color:#000;
    classDef c fill:#ffe2b3,stroke:#333,color:#000;

    Q{"Onboard Jetson<br/>+ Isaac ROS path?"}

    Q --> PathA["Path A — Cutting edge<br/>Jetson Thor · JetPack 7.1<br/>Jazzy · Isaac ROS 4.x"]:::a
    Q --> PathB["Path B — Pragmatic now<br/>Orin NX/AGX · JetPack 6<br/>Jazzy in Docker · TensorRT<br/>Isaac ROS optional / 3.x"]:::b
    Q --> PathC["Path C — Status quo<br/>Orin · JetPack 6 · Humble<br/>EOL 2027 — not advised"]:::c

    PathA --> A1["+ Latest Isaac ROS accel<br/>+ Most future-proof<br/>− Thor cost, brand new, port effort"]
    PathB --> B1["+ Uses affordable Orin<br/>+ Jazzy via container userspace<br/>− Validate ZED SDK+CUDA in 24.04 container"]
    PathC --> C1["+ Zero change today<br/>− Forced migration < 12 months"]
```

**My recommendation — Path B as default, Path A for one "pilot" robot:**

- Standardize the **project on Jazzy** regardless (it's the LTS the whole ecosystem is converging on).
- Run Jazzy via **Docker containers** on your existing **Orin NX/AGX** (the container provides the Ubuntu 24.04 userspace; the L4T host kernel stays JetPack 6). You do **not** strictly need Isaac ROS — you can run **plain TensorRT** for YOLO acceleration. Isaac ROS is a _nice-to-have accelerator_, not a hard dependency.
- **Spike to de-risk:** before committing the fleet, verify `ZED SDK v5 + CUDA + YOLO/TensorRT` runs inside an Ubuntu-24.04/Jazzy container on one Orin. If it's painful, buy **one Jetson Thor** as the lead-robot pilot (Path A) and keep Orins as backups.

> **Important nuance for your sim choice:** **Isaac Lab runs on the x86 training workstation**, exports **ONNX** policy weights, and is therefore **decoupled from the robot's ROS distro**. Your sim choice does _not_ force the onboard distro — only Isaac _ROS_ (the runtime accel libs) does.

### 3.3 Full Stack Summary

| Layer              | Technology                                            | Language      | Runs on           |
| ------------------ | ----------------------------------------------------- | ------------- | ----------------- |
| Learning / Sim     | Isaac Lab + Isaac Sim (PhysX/Newton)                  | Python        | Workstation (RTX) |
| Perception         | YOLO → TensorRT, ZED SDK v5, `robot_localization` EKF | Python / C++  | Jetson            |
| Strategy           | BehaviorTree.CPP + Groot2 (visual debug)              | C++           | Jetson            |
| Whole-body control | MPC + residual RL (ONNX→TensorRT) via `ros2_control`  | C++           | Jetson            |
| Real-time          | PD/torque + watchdog (micro-ROS)                      | C/C++         | STM32/Teensy MCU  |
| Referee            | GameController bridge (UDP 3838/3939)                 | C++ or Python | Jetson            |
| Team comms         | DDS / CycloneDDS over 5 GHz Wi-Fi                     | —             | Jetson↔Jetson     |
| DevOps             | Docker + GHCR + Ansible + GitHub Actions              | YAML          | Everywhere        |

---

## 4. Control Architecture — Hierarchical MPC + Residual RL (Your Design)

Your chosen paradigm is genuinely state-of-the-art. The professional implementation splits responsibility across three frequency bands:

```mermaid
flowchart TB
    classDef plan fill:#cfe8ff,stroke:#333,color:#000;
    classDef rl fill:#d5f5d5,stroke:#333,color:#000;
    classDef rt fill:#ffc9c9,stroke:#b00,stroke-width:2px,color:#000;

    GOAL["Strategy goal:<br/>walk(v) · kick · stand-up"]:::plan

    subgraph MIDLEVEL["Model-based layer · 20-100 Hz · C++ on Jetson"]
        FSP["Footstep planner"]:::plan
        MPC["MPC / ZMP whole-body<br/>→ reference q*, qd*, contact"]:::plan
    end

    subgraph RLLAYER["Residual RL · ~50 Hz · TensorRT on Jetson"]
        POL["Policy π(obs)<br/>obs = q, qd, IMU, contact, command"]:::rl
        SUM["q_target = q* + Δq_RL<br/>(bounded residual)"]:::rl
    end

    subgraph LOWLEVEL["Real-time · 1000 Hz · MCU"]
        PD["PD + feed-forward torque"]:::rt
        WD["Watchdog: bus-silence → zero torque"]:::rt
    end

    GOAL --> FSP --> MPC --> SUM
    POL --> SUM
    SUM -->|"joint targets"| PD
    PD --> WD
    WD --> MOTORS["Actuators"]
    MOTORS -->|"q, qd, current, IMU"| POL
    MOTORS -->|"state feedback"| MPC
```

**Why residual RL (not end-to-end):** the MPC guarantees a **physically grounded, debuggable** trajectory (essential for precise kicks and rules compliance), while the **bounded RL residual** absorbs model error, contact shocks, and pushes — the things classical MPC handles poorly. Because the residual is bounded, a misbehaving policy can't drive the robot into instability. This is the safest way to get RL's adaptability without losing determinism.

**Frequency contract (the corrected truth vs `new_flow_proposal.md`):**

| Loop                       | Frequency   | Where             |
| -------------------------- | ----------- | ----------------- |
| Footstep / MPC reference   | 20–100 Hz   | Jetson (C++)      |
| RL residual policy         | ~50 Hz      | Jetson (TensorRT) |
| **PD / torque + watchdog** | **1000 Hz** | **MCU**           |
| State estimator (EKF)      | 100–400 Hz  | Jetson            |

---

## 5. Perception & State Estimation (Proprioception vs Exteroception)

```mermaid
flowchart LR
    classDef ext fill:#cfe8ff,stroke:#333,color:#000;
    classDef int fill:#ffe2b3,stroke:#333,color:#000;
    classDef out fill:#d5f5d5,stroke:#333,color:#000;

    subgraph EXTERO["EXTEROCEPTION · 30-60 Hz"]
        ZED["ZED Mini<br/>RGB + depth + visual odom"]:::ext
        YOLO["YOLO → TensorRT<br/>ball · goalposts · lines · robots"]:::ext
        PROJ["3D projection (TF2)<br/>pixels → field coords"]:::ext
        ZED --> YOLO --> PROJ
    end

    subgraph PROPRIO["PROPRIOCEPTION · 100-1000 Hz"]
        IMU["IMU"]:::int
        ENC["Joint encoders"]:::int
        FOOT["Foot contact / pressure"]:::int
    end

    EKF["EKF fusion<br/>(robot_localization)"]:::out
    WM["World Model / TF tree<br/>self-pose + ball + field"]:::out

    IMU --> EKF
    ENC --> EKF
    FOOT --> EKF
    ZED -->|"visual-inertial odom"| EKF
    EKF -->|"/odom, base state"| WM
    PROJ -->|"object poses"| WM
    WM --> STRAT["→ Strategy (BT)"]
    EKF --> CTRL["→ Control (MPC + RL)"]
```

- **Exteroception** (slow, GPU-heavy): ZED → YOLO/TensorRT → 3D projection. Decoupled from the balance loop.
- **Proprioception** (fast): IMU + encoders + foot contact, fused by the EKF into a drift-resistant base state.
- The **World Model + TF tree** is the single shared truth that Strategy and Control read from.

---

## 6. Strategy — Behavior Trees + Lightweight Role Negotiation

We replace the old monolithic FSM (and reject heavyweight STP) with **per-robot Behavior Trees** (BehaviorTree.CPP + **Groot2** for live visual debugging). Team coordination is a **decentralized role auction**, not a master.

```mermaid
flowchart TB
    classDef bt fill:#d5f5d5,stroke:#333,color:#000;
    classDef role fill:#cfe8ff,stroke:#333,color:#000;

    GS["GameState (from bridge)"] --> ROOT
    WM["World Model"] --> ROOT

    subgraph TREE["Per-robot Behavior Tree"]
        ROOT["Root: Game-state gate"]:::bt
        ROOT --> PEN{"Penalized / Halt?"}:::bt
        PEN -->|yes| SAFE["Freeze / stand"]:::bt
        PEN -->|no| ROLE["Selected role subtree"]:::bt
        ROLE --> STR["Striker: approach → align → kick"]:::bt
        ROLE --> SUP["Supporter: position for pass"]:::bt
        ROLE --> GK["Goalie: track → block → clear"]:::bt
    end

    subgraph NEG["Decentralized role auction · team_comm"]
        BID["Each robot bids on roles<br/>(cost = dist to ball, etc.)"]:::role
        ASSIGN["Highest-utility assignment<br/>dynamic re-bid on dropout"]:::role
        BID --> ASSIGN --> ROLE
    end
```

- Roles are **XML trees** loaded at launch (`striker.xml`, `goalie.xml`, …). Swapping a robot's behavior is a config change, not a code change.
- **Role auction:** robots broadcast bids (e.g., distance-to-ball) over team comms; each independently computes the same optimal assignment. If a robot drops out, the team **re-bids automatically** — no master, no single point of failure (RoboCup-legal).

---

## 7. GameController Integration (Verified Protocol)

The RoboCup-Humanoid-TC GameController (Rust/Tauri, 2025 rework) speaks a precise UDP protocol. **Verified details:**

- **Control → robots:** UDP **broadcast**, port **3838**, **2 Hz**, struct `HlRoboCupGameControlData`.
- **Status ← robots:** UDP **unicast**, port **3939**, **0.5–2 Hz**, struct `HlRoboCupGameControlReturnData`. _(Robots must reply, or the GC flags them.)_
- Competitions: `KidSize`, `AdultSize`, `DropIn`. Note: after goals/transitions the GC may broadcast a "fake" prior state for up to 15 s — your bridge must not over-react.

```mermaid
sequenceDiagram
    participant GC as GameController (referee PC)
    participant BR as game_controller_bridge (per robot)
    participant BT as Behavior Tree
    participant MCU as Safety MCU

    loop Every 0.5s (2 Hz)
        GC-->>BR: UDP broadcast :3838 (HlRoboCupGameControlData)
        BR->>BT: publish /gc/game_state (READY/SET/PLAYING/...)
    end

    loop Every ~1s
        BR-->>GC: UDP unicast :3939 (alive + player status)
    end

    Note over BT: PLAYING → run role subtree
    Note over BT: HALT/PENALIZED overrides AI
    Note over MCU: MCU watchdog enforces zero-torque
```

**Design:** a thin `game_controller_bridge` node parses 3838, publishes a clean `/gc/game_state` topic (all namespaced robots subscribe), and emits the mandatory 3939 heartbeat. It is the **ultimate authority** — `HALT`/`PENALIZED` overrides all AI.

---

## 8. Multi-Robot — Fully Decentralized (No Master)

```mermaid
flowchart TB
    classDef r fill:#d5f5d5,stroke:#333,color:#000;
    classDef net fill:#f9d5e5,stroke:#333,color:#000;

    GC["GameController PC"]:::net
    WIFI(("5 GHz team Wi-Fi<br/>CycloneDDS")):::net

    subgraph R1["Robot 1 — /robot_1 (full stack)"]
        S1["Perception · EKF · BT · MPC+RL"]:::r
    end
    subgraph R2["Robot 2 — /robot_2 (full stack)"]
        S2["Perception · EKF · BT · MPC+RL"]:::r
    end
    subgraph R3["Robot 3 — /robot_3 (full stack)"]
        S3["Perception · EKF · BT · MPC+RL"]:::r
    end

    GC -. "UDP 3838/3939" .-> R1
    GC -.-> R2
    GC -.-> R3
    R1 <-->|"world model + role bids"| WIFI
    R2 <--> WIFI
    R3 <--> WIFI
```

- Every robot runs the **identical container**; identity is just a **ROS namespace** (`/robot_1`…) + TF prefix. Launching the fleet is a parameter, not a fork.
- Robots share only a **lightweight world model + role bids** over DDS. Each is **independently autonomous** — losing one degrades the team gracefully.
- **Domain ID / discovery** isolation per team to avoid cross-talk with opponents.

---

## 9. Hardware ↔ Firmware Bridge & the RL State Vector

**Do not connect motors directly to the Jetson.** The Jetson runs Linux (not hard-real-time). A dedicated **MCU** (STM32/Teensy) on the firmware team's PCB owns the 1 kHz loop and safety.

```mermaid
flowchart LR
    classDef j fill:#cfe8ff,stroke:#333,color:#000;
    classDef m fill:#ffc9c9,stroke:#b00,stroke-width:2px,color:#000;
    classDef h fill:#ffe2b3,stroke:#333,color:#000;

    subgraph JET["Jetson (Linux, ~50 Hz)"]
        RL["MPC + RL → q_target"]:::j
        HWI["ros2_control HardwareInterface"]:::j
        RL --> HWI
    end

    subgraph MCUBOX["MCU / PCB (real-time, 1 kHz)"]
        MCU["micro-ROS · PD/torque · FOC"]:::m
        WD["Watchdog (bus-silence → safe stop)"]:::m
    end

    subgraph HW["Actuators & sensors"]
        MOT["Motors"]:::h
        ENC["Encoders → position q"]:::h
        CUR["Current sense → torque τ"]:::h
        IMU["IMU"]:::h
    end

    HWI <-->|"CAN / serial @ 1 kHz"| MCU
    MCU --> WD --> MOT
    ENC --> MCU
    CUR --> MCU
    IMU --> MCU
    MCU -->|"q, qd, τ, IMU"| HWI
```

### The RL state vector — what hardware MUST provide

For the policy $\pi_\theta(a_t \mid s_t)$ to work **identically in sim and reality**, the PCB must measure, per joint:

| Symbol     | Quantity                   | Hardware requirement                            |
| ---------- | -------------------------- | ----------------------------------------------- |
| $q_t$      | Joint position             | Encoder on each output                          |
| $\dot q_t$ | Joint velocity             | MCU differentiates encoder                      |
| $\tau_t$   | Joint torque/effort        | **Current-sense resistors** in the motor driver |
| IMU        | Base orientation/ang. vel. | Body IMU (the ZED Mini IMU can cross-check)     |
| contact    | Foot contact               | Foot pressure / contact switches                |

> **Tell the firmware team:** the **current-sense path is non-negotiable** for residual RL — without measured $\tau$, your observation vector differs between sim and robot and the policy won't transfer. Expose everything as standard `sensor_msgs/JointState` + `sensor_msgs/Imu` (via micro-ROS) so the `ros2_control` interface is identical in sim and on hardware.

**Actuator note (advisory):** RoboCup KidSize humanoids commonly use **Dynamixel X-series** servos (current-based control mode gives you $\tau$). If the team is designing custom BLDC + FOC drivers, ensure the driver firmware exposes commanded/measured current. Either way, the `ros2_control` boundary stays identical.

---

## 10. Sim-to-Real Pipeline (Isaac Lab)

The `ros2_control` boundary means **the exact same ROS graph runs in sim and on the robot** — only the `HardwareInterface` plugin swaps. RL transfer uses the modern **teacher → student distillation + domain randomization** recipe (confirmed in Isaac Lab docs).

```mermaid
flowchart LR
    classDef sim fill:#d5f5d5,stroke:#333,color:#000;
    classDef bridge fill:#cfe8ff,stroke:#333,color:#000;
    classDef real fill:#ffe2b3,stroke:#333,color:#000;

    URDF["URDF / USD<br/>(single source of truth)"]:::sim
    TEACH["1 Teacher policy<br/>(privileged sim state)"]:::sim
    STUD["2 Student policy<br/>(only real-available obs)"]:::sim
    DR["Domain randomization<br/>mass, friction, latency, sensor noise"]:::sim
    EXPORT["3 Export ONNX → TensorRT"]:::bridge

    URDF --> TEACH --> STUD
    DR --> TEACH
    DR --> STUD
    STUD --> EXPORT

    subgraph DEPLOY["Same ros2_control graph"]
        SIMHW["MuJoCo/Isaac HardwareInterface"]:::sim
        REALHW["MCU HardwareInterface"]:::real
    end
    EXPORT --> SIMHW
    EXPORT --> REALHW

    REALHW -->|"telemetry logs"| SYSID["System Identification<br/>tune sim to match reality"]:::bridge
    SYSID --> DR
```

**Closing the reality gap (beyond "make sim pretty"):**

1. **Domain randomization** — randomize physics + sensor noise so the policy is forced to be robust.
2. **Teacher → student distillation** — teacher learns with privileged sim-only info; student is distilled to use _only_ sensors the real robot has.
3. **System identification** — log real motor responses, fit sim parameters to reality, feed back into DR.
4. **Avoid live on-robot RL fine-tuning** — erratic exploration destroys hardware. Iterate in sim; deploy validated weights.

---

## 11. Repository Organization (Monorepo)

You prefer a **monorepo** — correct for guaranteeing that one commit pins a matching set of CAD + firmware + software. Heavy Isaac Lab training assets and large binaries use **Git LFS** or a `git submodule`, so cloning the robot runtime stays lightweight.

```text
soccerbot/                          # one monorepo, pinned-together history
├── .github/workflows/              # CI/CD: build, test, multi-arch image push
├── docs/                           # MkDocs/Sphinx; architecture, runbooks
│
├── ros2_ws/src/                    # ── ROS 2 workspace (deployed to robots) ──
│   ├── soccer_msgs/                # custom interfaces (C++/IDL)
│   ├── soccer_bringup/             # launch + params + per-robot namespacing
│   ├── soccer_description/         # URDF / xacro (mirrors USD in /sim)
│   ├── soccer_hardware/            # ros2_control HW interfaces (real + sim)
│   ├── soccer_control/             # [C++] MPC, residual-RL runner, controllers
│   ├── soccer_perception/          # [Py] YOLO/TensorRT, ZED, 3D projection
│   ├── soccer_localization/        # [C++] EKF config (robot_localization)
│   ├── soccer_strategy/            # [C++] BehaviorTree.CPP + role auction
│   │   └── trees/                  #   striker.xml, goalie.xml, supporter.xml
│   ├── soccer_teamcomm/            # decentralized DDS world model + bids
│   └── game_controller_bridge/     # UDP 3838/3939 ↔ /gc/game_state
│
├── soccer-firmware/                # ── [submodule] STM32 Master/Slave (NOT deployed via ROS) ──
│   └── (Master: safety + aggregation · CAN-FD MIT bridge to Robostride)
│
├── sim/                            # ── Isaac Lab (runs on workstation) ──
│   ├── tasks/                      # RL envs (walk, kick, getup, push-recovery)
│   ├── assets/                     # USD models (Git LFS)
│   └── export/                     # ONNX → TensorRT conversion
│
├── hardware/                       # CAD, PCB schematics (Git LFS)
│
├── deploy/                         # ── DevOps ──
│   ├── docker/                     # Dockerfiles (dev + lean runtime, multi-arch)
│   ├── compose/                    # local multi-robot sim bring-up
│   └── ansible/                    # fleet: pull image + restart on N robots
│
└── tools/                          # dev scripts, dataset tooling, calibration
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
    DESC --> HW & CTRL
    HW --> CTRL
    LOC --> CTRL
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

## 12. Deployment & DevOps Workflow

**Git holds the recipe; the registry holds the baked cake; Ansible serves it to the robots.** You never `git clone` onto a robot.

```mermaid
flowchart LR
    classDef dev fill:#cfe8ff,stroke:#333,color:#000;
    classDef ci fill:#d5f5d5,stroke:#333,color:#000;
    classDef reg fill:#ffe2b3,stroke:#333,color:#000;
    classDef bot fill:#ffc9c9,stroke:#b00,color:#000;

    DEV["Developer push"]:::dev --> GH["GitHub repo"]:::dev
    GH --> CI["GitHub Actions<br/>build · lint · test · build arm64 image"]:::ci
    WS["Isaac Lab (workstation)<br/>train → export ONNX"]:::dev --> CI
    CI --> REG["Container registry (GHCR)<br/>versioned images + policy weights"]:::reg
    REG --> ANS["Ansible playbook (one command)"]:::reg
    ANS -->|SSH| B1["Robot 1: pull + restart"]:::bot
    ANS -->|SSH| B2["Robot 2: pull + restart"]:::bot
    ANS -->|SSH| B3["Robot 3: pull + restart"]:::bot
```

**Answering the questions raised in the prior docs:**

- **"Docker registry vs Git?"** Git = source code. A **registry** (GitHub Container Registry, free with your repo) stores **built images**. Robots pull a tagged image; they never see source or unneeded files.
- **"Do we dump the whole repo on robots?"** No. The **lean runtime Docker image** contains only the built `install/` space + dependencies. No CAD, no sim assets, no training code.
- **"What is Ansible / fleet management?"** One command on your laptop SSHes into all robots, pulls the new image, and restarts containers — reproducible, no manual per-robot fiddling. For 1–3 robots this is the right-sized professional tool (heavier fleet tools like Greengrass/balena are overkill now).
- **"CV/control/strategy — nodes or packages?"** Both: a **package** is a code folder; a **node** is a running process. `zed-ros2-wrapper` is one package that spawns several nodes. You'll write a `soccer_strategy` _package_ that runs a strategy _node_.
- **Secrets:** registry tokens / Wi-Fi creds live in CI secrets or `.env` files in `.gitignore` — **never** logged to stdout/stderr or committed.

---

## 13. Budget Allocation (~$10,000, approximate 2026 pricing)

Two viable allocations depending on the §3.2 decision. **Verify live prices before purchase.**

### Recommended: "Pragmatic + 1 pilot" (Path B + one Thor)

| Item                                                           | Qty | Approx. unit | Approx. total |
| -------------------------------------------------------------- | --- | ------------ | ------------- |
| RTX 4090/5090 training workstation                             | 1   | $3,500       | $3,500        |
| Jetson Orin NX 16 GB (module + carrier)                        | 2   | $850         | $1,700        |
| Jetson Thor T4000 (lead-robot pilot, latest Isaac ROS)         | 1   | $2,800       | $2,800        |
| ZED Mini (supplement existing)                                 | 1   | $450         | $450          |
| 5 GHz Wi-Fi router/AP (team comms)                             | 1   | $250         | $250          |
| NVMe SSDs (Jetson/Thor storage)                                | 3   | $130         | $390          |
| Misc: cables, MCU dev boards, current-sense parts, contingency | —   | —            | $900          |
| **Total**                                                      |     |              | **≈ $9,990**  |

### Alternative: "All-Orin, max savings" (Path B only)

| Item                                          | Qty | Approx. total                                            |
| --------------------------------------------- | --- | -------------------------------------------------------- |
| RTX 4090/5090 workstation                     | 1   | $3,500                                                   |
| Jetson Orin NX 16 GB                          | 3   | $2,550                                                   |
| ZED Mini supplement / ZED upgrade             | —   | $900                                                     |
| Networking + SSDs + misc + larger contingency | —   | $1,800                                                   |
| **Total**                                     |     | **≈ $8,750** (≈ $1,250 reserve for motors/PCB iteration) |

> Motors, frame, and the custom PCB are typically separate (firmware/mechanical) budgets. If they must come from this $10k, lead with the **all-Orin** allocation to preserve reserve.

---

## 14. Phased Roadmap (sequence, not schedule)

```mermaid
flowchart LR
    classDef p fill:#cfe8ff,stroke:#333,color:#000;

    P0["Phase 0 — Foundations<br/>Monorepo + CI + Docker<br/>Pick distro (Jazzy) · §3.2 spike<br/>URDF/USD single source"]:::p
    P1["Phase 1 — Sim loop<br/>ros2_control + Isaac HW iface<br/>Teleop walk in sim<br/>GameController bridge"]:::p
    P2["Phase 2 — Perception<br/>ZED + YOLO/TensorRT<br/>EKF localization<br/>World model + TF"]:::p
    P3["Phase 3 — Control<br/>MPC reference + residual RL<br/>Domain randomization<br/>ONNX→TensorRT export"]:::p
    P4["Phase 4 — Sim-to-real<br/>MCU micro-ROS + watchdog<br/>Real HW iface · sysID<br/>Deploy weights to 1 robot"]:::p
    P5["Phase 5 — Team play<br/>Behavior Trees + roles<br/>Decentralized team comms<br/>Ansible fleet to N robots"]:::p

    P0 --> P1 --> P2 --> P3 --> P4 --> P5
```

**Exit criteria per phase** (what "done" means):

- **P0:** one `colcon build` + CI green; distro decided; ZED+CUDA validated in target container.
- **P1:** robot walks in Isaac via teleop through the _same_ `ros2_control` graph you'll use on hardware.
- **P2:** robot reliably detects + localizes ball/goal in sim and from a real ZED feed.
- **P3:** push-recovery + walk policy stable in sim with domain randomization.
- **P4:** the _identical_ ROS graph drives the real robot via the MCU; watchdog safe-halt verified.
- **P5:** 2–3 robots auto-assign roles and play a scrimmage with GameController control.

---

## 15. What I Recommend You Confirm Next

1. **§3.2 decision** — approve **Jazzy** as the project distro, and choose **Path B (Orin + Jazzy container)** vs **buying one Thor pilot**. _(This unblocks all purchasing.)_
2. **Actuator type** — Dynamixel (fast path, current-mode torque) vs custom BLDC/FOC (more capable, more firmware work). Drives the RL state-vector wiring.
3. **Where motors/PCB/frame are funded** — from this $10k or a separate budget? Determines which allocation in §13.
4. **RoboCup sub-league** — KidSize vs AdultSize? Affects robot count rules, field size, and GameController `Competition` config.

---

### Appendix — Sources (verified June 2026)

- ROS 2 distributions & EOL dates — docs.ros.org Releases (Humble EOL May 2027; Jazzy LTS → 2029; Lyrical Luth May 2026).
- NVIDIA Isaac ROS getting-started — latest release targets **ROS 2 Jazzy**, **JetPack 7.1 / Jetson Thor**, Ubuntu 24.04 (updated Apr 2026).
- Stereolabs ZED ROS 2 — SDK v5.x supports Humble **and** Jazzy (Ubuntu 22.04 / 24.04).
- RoboCup-Humanoid-TC GameController — UDP broadcast **3838 @ 2 Hz** (`HlRoboCupGameControlData`); status return **3939 @ 0.5–2 Hz**; competitions KidSize/AdultSize/DropIn.
- Isaac Lab docs — domain randomization + teacher→student distillation sim-to-real; Newton physics integration (experimental).
