# ZED Mini + Jetson Integration — Implementation Report

> **What this is.** A justified, as-built record of the changes made to
> `soccer-bot` so the repository mirrors the **proven** ZED Mini + ROS 2
> bring-up on the Jetson Orin Nano (JetPack 7.2 / L4T R39.2). It documents the
> findings, the design decisions (with rationale and the alternatives rejected),
> every file changed, and the on-device verification procedure.
>
> **Provenance.** The recipe encoded here was validated end-to-end on real
> hardware; the blow-by-blow debugging log lives in the journey doc
> (`ZED_JETPACK72_DOCKER_JOURNEY.md`). The forward-looking strategy / workflow
> rationale lives in [`docs/jetson_zed_workflow.md`](jetson_zed_workflow.md).
> **Where any document disagrees with the hardware, the hardware wins** — and the
> facts below are taken from the hardware.

---

## 1. Executive summary

The repo was already **architecturally** ready for a real camera: every consumer
subscribes to a small, driver-agnostic **topic contract** (`camera/image_raw`,
`camera/depth`, `camera_info`, `imu/data`) that a synthetic `sim_camera_node`
fills today. What was missing was the **operational** layer: a GPU/JetPack image,
the ZED→contract glue, the GPU container wiring, and the host fixes that make
`--gpus` work at all on this toolkit version.

This change set adds exactly that layer — and nothing more invasive:

| Outcome                                                   | How                                                                                                       |
| --------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| The ZED runs in a container on the proven stack           | New multi-target `Dockerfile.jetson` (CUDA 13.2 + ZED SDK 5.4 + TensorRT 10.16.2.10 + `zed-ros2-wrapper`) |
| The ZED's native topics feed the existing graph unchanged | ZED **component** loaded by `camera.launch.py` with topic **remaps**; `robot.launch.py camera:=zed` consumes                              |
| Real depth + real calibration are used                    | `projection_node` now reads `camera_info` and uses the depth path by default                              |
| One command brings up camera + app on a robot             | New `deploy/compose/robot.compose.yaml`; updated `deploy/ansible/deploy.yml`                              |
| The mandatory host fixes are reproducible per robot       | New `deploy/ansible/provision.yml`                                                                        |
| The docs state the real facts                             | Corrected `jetson_zed_workflow.md`; CI note added                                                         |

**No application logic was rewritten.** The sim flow is untouched (`camera:=sim`
remains the default), so laptops and CI keep working with zero GPU.

---

## 2. The proven environment (authoritative facts)

| Property               | Value                                                 | Consequence for the repo                                                  |
| ---------------------- | ----------------------------------------------------- | ------------------------------------------------------------------------- |
| Board                  | Jetson Orin Nano (Super), 8 GB, **sm_87**             | 8 GB ⇒ capped build parallelism + swap; engines built on-device           |
| JetPack / L4T          | **7.2 / R39.2** (`/etc/nv_tegra_release`)             | No `l4t-jetpack:r39` image exists → use a generic CUDA base               |
| OS / CUDA              | Ubuntu 24.04.4 / **CUDA 13.2**                        | ROS 2 **Jazzy is native** (24.04); base = `cuda:13.2.1-devel-ubuntu24.04` |
| Container toolkit      | `nvidia-container-toolkit 1.19.1`                     | Ships a **buggy `enable-cuda-compat` CDI hook** → must be disabled        |
| Docker default runtime | `nvidia`                                              | `--gpus all` is the supported GPU path (CDI)                              |
| Camera                 | **ZED Mini** USB3 (`2b03:f682` UVC + `2b03:f681` IMU) | Must be on a **USB 3.0** port; bridge subscribes with sensor QoS          |
| ZED SDK / wrapper      | **5.4** / `zed-ros2-wrapper` (release_5.4)            | Installer pinned to `l4t39.2/5.4`; wrapper built from source              |
| TensorRT               | **10.16.2.10** (JetPack build)                        | Default `depth_mode: NEURAL_LIGHT` needs it; pinned + symlinked           |

**Verified live topic names and rates** (these drove the bridge defaults):

| ZED topic                                  | Type                                       | Rate     |
| ------------------------------------------ | ------------------------------------------ | -------- |
| `/zed/zed_node/rgb/color/rect/image`       | `sensor_msgs/Image`                        | ~30 Hz   |
| `/zed/zed_node/rgb/color/rect/camera_info` | `sensor_msgs/CameraInfo`                   | ~30 Hz   |
| `/zed/zed_node/depth/depth_registered`     | `sensor_msgs/Image` (32-bit float, metres) | depth on |
| `/zed/zed_node/imu/data`                   | `sensor_msgs/Imu`                          | ~99 Hz   |

> The SDK 5.x wrapper publishes RGB at `…/rgb/color/rect/image` — **not** the old
> `…/rgb/image_rect_color` that earlier drafts of the workflow doc assumed.

---

## 3. Architecture — the one boundary that makes this cheap

The single most important decision (pre-existing, now realised): **quarantine the
vendor coupling.** The ZED SDK + CUDA + JetPack live in exactly one container,
behind a stable topic contract. Everything else is portable, arch-independent
Jazzy application code.

```mermaid
flowchart TB
    classDef host fill:#0b3d91,color:#fff;
    classDef drv fill:#ffe2b3,stroke:#333,color:#000;
    classDef bridge fill:#f9d5e5,stroke:#333,color:#000;
    classDef contract fill:#d5f5d5,stroke:#333,color:#000;
    classDef app fill:#cfe8ff,stroke:#333,color:#000;

    HOST["HOST · JetPack 7.2 / L4T R39.2<br/>nvidia driver + container-toolkit<br/>(mode=cdi, enable-cuda-compat disabled)"]:::host

    subgraph CAM["camera container — zed-driver-image (ONLY GPU/SDK/JetPack-coupled piece)"]
        ZN["stereolabs::ZedCamera component · camera.launch.py<br/>ZED SDK 5.4 + TensorRT (NEURAL_LIGHT)<br/>use_intra_process_comms → zero-copy ready"]:::drv
        subgraph CONTRACT["generic camera contract (published via topic remaps)"]
            T1["camera/image_raw"]:::contract
            T2["camera/depth"]:::contract
            T3["camera_info"]:::contract
            T4["imu/data"]:::contract
        end
    end

    subgraph APPC["app container — soccer-app-image (portable Jazzy, CPU today)"]
        DET["detector_node"]:::app
        FL["fieldline_node"]:::app
        PROJ["projection_node"]:::app
        EKF["ekf_node"]:::app
    end

    HOST -. "--gpus all --privileged -v /dev:/dev" .-> ZN
    ZN -->|"remap ~/rgb/color/rect/image → camera/image_raw"| T1
    ZN --> T2 & T3 & T4
    T1 -->|"best-effort SensorData (DDS)"| DET
    T1 --> FL
    T2 --> PROJ
    T3 --> PROJ
    T4 --> EKF

    SIM["sim_camera_node<br/>(camera:=sim, no GPU)"]:::drv -. "same contract" .-> T1
```

**Why two containers, not one.** The driver image is ~19 GB and JetPack-coupled;
the app image is lean, CPU-only, and changes every commit. Splitting them means a
code change rebuilds only the small image, the camera can restart independently of
strategy/control, and a no-GPU laptop can run everything to the right of the
contract from a rosbag. (Single-container dev is still available via
`robot.launch.py launch_driver:=true`, which starts the ZED component in-process —
only in an image carrying the ZED SDK + wrapper.)

---

## 4. The image lineage — one multi-stage Dockerfile, two targets

`deploy/docker/Dockerfile.jetson` is a single multi-stage file with a shared base
and two independently-selectable targets.

```mermaid
flowchart TB
    classDef base fill:#1f4e79,color:#fff;
    classDef img fill:#14532d,color:#fff;

    B["STAGE 1 · jetson-base<br/>FROM nvcr.io/nvidia/cuda:13.2.1-devel-ubuntu24.04<br/>rm cuda/compat · fake nv_tegra_release R39.2<br/>+ ROS 2 Jazzy apt repo + entrypoint"]:::base
    Z["STAGE 2 · zed-driver-image<br/>ZED SDK 5.4 (l4t39.2) + TensorRT 10.16.2.10<br/>(pin + unversioned symlinks + compat-mode)<br/>+ zed-ros2-wrapper (colcon)"]:::img
    A["STAGE 3 · soccer-app-image<br/>ros-jazzy-ros-base + deps<br/>colcon build ros2_ws (--merge-install)"]:::img

    B --> Z
    B --> A
```

**Why this shape** (the option you approved — _multi-stage build with multi-target
outputs_):

- The CUDA + ROS-repo foundation is defined **once** in `jetson-base` and shared,
  so the two images can never drift apart on their base layer.
- `docker build --target zed-driver-image` and `--target soccer-app-image` produce
  two clean, minimal images from the same recipe — no orphaned SDK layers in the
  app image, no duplicated base maintenance.
- It encodes, in code, the hard-won fixes from the live bring-up (each with a
  comment): remove the dGPU `cuda/compat`, fake `/etc/nv_tegra_release` so the ZED
  installer accepts a non-L4T base, pin TensorRT to the JetPack build, add the
  unversioned `libnvinfer.so` symlinks + `ZED_SDK_TENSORRT_VERSION_COMPATIBILITY_MODE=1`,
  link against the CUDA driver **stub** and `--packages-ignore zed_debug`, and cap
  build parallelism so the 8 GB Orin doesn't OOM.

> **Why a generic CUDA base, not `l4t-jetpack`.** There is no `l4t-jetpack` image
> for r39 (the `l4t-*` family stops at r36). JetPack 7.2 is a stock Ubuntu 24.04 +
> CUDA 13 userspace and the container toolkit injects the host Tegra driver libs
> at run time — so the generic CUDA image is the correct, forward-compatible base.

---

## 5. The camera source — direct component remapping (no bridge)

The ZED wrapper publishes under `/zed/zed_node/...`; the graph expects the generic
contract. The modern, lowest-overhead way to reconcile them is to make the ZED
publish the contract topics **itself** — not to shuttle every frame through a
relay. `camera.launch.py` loads the Stereolabs **`stereolabs::ZedCamera`
component** directly (a `ComposableNode`) and passes `remappings=`, which
`launch_ros` sends to the container as the component's `remap_rules`; the camera
therefore advertises the contract names natively:

| ZED SDK 5.x topic (private `~/…`) | contract topic     |
| --------------------------------- | ------------------ |
| `~/rgb/color/rect/image`          | `camera/image_raw` |
| `~/rgb/color/rect/camera_info`    | `camera_info`      |
| `~/depth/depth_registered`        | `camera/depth`     |
| `~/imu/data`                      | `imu/data`         |

The component runs under the `robot_name` namespace, so the relative targets
resolve to `/<robot_name>/camera/...` — exactly what the stack subscribes to.

```mermaid
flowchart LR
    classDef z fill:#ffe2b3,stroke:#333,color:#000;
    classDef c fill:#d5f5d5,stroke:#333,color:#000;
    Z["stereolabs::ZedCamera component<br/>camera.launch.py · use_intra_process_comms"]:::z
    Z -->|"remap ~/rgb/color/rect/image"| C1["camera/image_raw"]:::c
    Z -->|"remap ~/rgb/color/rect/camera_info"| C3["camera_info"]:::c
    Z -->|"remap ~/depth/depth_registered"| C2["camera/depth"]:::c
    Z -->|"remap ~/imu/data"| C4["imu/data"]:::c
```

**Why this replaced the earlier `camera_bridge` node.** A previous revision ran a
tiny rclpy relay that subscribed to the native topics and re-published them on the
contract. It worked, but it was not the most efficient shape — confirmed against
the `rclpy` / `launch_ros` / `rclcpp` sources:

| Concern                                       | `camera_bridge` (rclpy relay)                                                                          | Direct component remap (now)                                                                                                                              |
| --------------------------------------------- | ----------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Inter-process hops for full-res image + depth | **2** (ZED → bridge → consumers)                                                                       | **1** (ZED → consumers)                                                                                                                                   |
| Per-frame work in the relay                   | Full **deserialize + re-serialize** of every multi-MB frame, under the Python **GIL**, single-threaded | **None** — the camera publishes the contract topic itself                                                                                                 |
| Zero-copy to a future C++ perception node     | **Impossible** — rclpy has no intra-process comms, so a relay forecloses it                            | **Preserved** — loaded with `use_intra_process_comms`; a C++ node co-loaded into `zed_container` gets zero-copy (inter-process subscribers still use DDS) |
| Renaming a *composable* node's topics         | The bridge existed partly because `SetRemap` was thought unreliable for composable nodes              | `ComposableNode(remappings=…)` is first-class → sent as `remap_rules` (verified in `launch_ros` tests)                                                    |

So `camera_bridge.py` + its script wrapper were **deleted**, and `camera.launch.py`
now owns the ZED component. Because it only references `zed_wrapper` +
`zed_components`, the driver image runs it by file path
(`/opt/soccer/camera.launch.py`) — no soccer_ws build in that image.

### QoS — best-effort SensorData end-to-end

The contract is now **best-effort SensorData QoS** on both ends — the idiomatic
choice for high-rate sensor streams (REP-2003), and the correction of the old
relay's backwards reconciliation (it forced camera data up to RELIABLE):

| Where                                                                | Setting                                                                                                                 | Why                                                                                           |
| -------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| ZED publishers                                                       | `qos_overrides` in `zed_params_override.yaml` → `reliability: best_effort` (keyed by the resolved contract topic names) | The wrapper enables rclcpp `QosOverridingOptions` on every publisher — QoS in config, no code |
| Consumers (`detector`/`fieldline`/`projection`/`ekf`) + `sim_camera` | `qos_profile_sensor_data`                                                                                                | Match the source; drop the odd frame rather than stall                                        |

**Why best-effort matters at the DDS layer.** An HD image / float32 depth sample is
far larger than a datagram, so RTPS fragments it into hundreds–thousands of pieces.
Under RELIABLE, one lost fragment triggers NACK-driven retransmissions that compete
with fresh frames on a busy link (head-of-line blocking, latency spikes), and the
writer holds every sample in history. Best-effort drops the incomplete frame and
takes the next — invisible at 30 fps. On today's single-host SHM transport loss is
~nil, but best-effort is correct and future-proofs any hop that crosses WiFi (a
remote Foxglove/RViz viewer, multi-robot team comm).

Files: [`camera.launch.py`](../../ros2_ws/src/soccer_bringup/launch/camera.launch.py),
[`zed_params_override.yaml`](../../deploy/compose/zed_params_override.yaml), and the
best-effort subscriptions in the perception / localization nodes.

---

## 6. Host prerequisites — the real unblock (CDI)

This is the part that has nothing to do with our code and everything to do with
whether a GPU container starts at all. The `nvidia-container-toolkit 1.19.1`
ships an `enable-cuda-compat` CDI hook that **panics** (`slice bounds out of range
[:73]`) and aborts every `--gpus`/CDI container.

```mermaid
sequenceDiagram
    participant D as docker run --gpus all
    participant R as nvidia-container-runtime
    participant S as /var/run/cdi/nvidia.yaml
    participant H as enable-cuda-compat hook
    D->>R: create container
    R->>S: read CDI spec (devices + hooks)
    S-->>R: createContainer hook: enable-cuda-compat
    R->>H: run hook
    H--xR: PANIC slice bounds [:73] cap 71
    R--xD: container init fails
    Note over D,H: Fix 1 removes the hook from the spec.<br/>Fix 2 (mode=cdi) stops --runtime nvidia re-adding it via the legacy CSV path.
```

Two mandatory host changes (idempotently applied by
[`deploy/ansible/provision.yml`](../../deploy/ansible/provision.yml)):

1. `/etc/nvidia-container-toolkit/nvidia-cdi-refresh.env` →
   `NVIDIA_CTK_CDI_GENERATE_DISABLED_HOOKS=enable-cuda-compat`, then restart
   `nvidia-cdi-refresh.service` (regenerates the CDI spec without the hook).
2. `/etc/nvidia-container-runtime/config.toml` → `mode = "cdi"` (forces the modern
   path so a bare `--runtime nvidia` cannot fall back to the CSV hook).

Plus, for **on-device builds only**, an 8 GB swapfile (the template-heavy
`zed_components` compile exhausts 8 GB and freezes the board). The playbook
verifies the regenerated CDI spec no longer contains the broken hook before
declaring success.

> **Never use `--runtime nvidia` as the GPU path here.** Use `--gpus all` (CDI).
> The compose/Ansible files express this as a `device_requests` / `devices:
[driver: nvidia]` reservation, which is the toolkit's CDI injection.

---

## 7. TensorRT — why the pin, the symlinks, and the env flag

The wrapper's default `depth_mode` is `NEURAL_LIGHT`, which `dlopen`s
`libnvinfer.so.10`. Three independent issues had to be solved, all encoded in the
driver stage:

```mermaid
flowchart TD
    classDef bad fill:#7a1f1f,color:#fff;
    classDef good fill:#1f7a1f,color:#fff;
    S["Need libnvinfer.so.10 for NEURAL_LIGHT"] --> P1["NGC sbsa repo (prio 600)<br/>installs 10.16.1.11"]:::bad
    P1 --> FIX1["Pin L4T repo (prio 700)<br/>→ 10.16.2.10 (JetPack build)"]:::good
    FIX1 --> P2["ZED dlopens UNVERSIONED libnvinfer.so<br/>(runtime pkg ships only .so.10)"]:::bad
    P2 --> FIX2["Add unversioned symlinks + ldconfig"]:::good
    FIX2 --> P3["SDK version gate"]:::bad
    P3 --> FIX3["ENV ZED_SDK_TENSORRT_VERSION_COMPATIBILITY_MODE=1"]:::good
    FIX3 --> OK["NEURAL_LIGHT engine builds (sm_87 via PTX),<br/>cached in the zed_resources volume"]:::good
```

The optimized engine is cached in the **`zed_resources`** named volume, so the
multi-minute optimization runs once. The compose and Ansible files mount that
volume on `/usr/local/zed/resources`.

---

## 8. Depth + real intrinsics in `projection_node`

Two small, justified changes let the existing projection use real ZED data:

- **`camera_info` subscription.** `projection_node` previously hard-coded
  `fx=550, …`. It now subscribes to `camera_info` and adopts the ZED's calibrated
  `K` (and width/height), falling back to the constructor defaults until the first
  message. Accurate intrinsics are required for the depth back-projection to be
  metric.
- **Depth on by default.** The launch override that forced `use_depth:=false` was
  removed; the node's own default is `True`. When `camera/depth` is flowing
  (real ZED) it uses the accurate stereo-depth path; when it is silent (sim, or
  depth disabled) it **automatically** falls back to the flat-ground homography.
  So the same node is correct in sim and on hardware with no per-mode flags.

```mermaid
flowchart LR
    classDef c fill:#cfe8ff,stroke:#333,color:#000;
    D["detections (pixel u,v)"]:::c --> Q{"camera/depth<br/>arrived & valid?"}
    Q -->|yes| DP["project_with_depth<br/>(metric, uses camera_info K)"]:::c
    Q -->|no| FG["project_flat_ground<br/>(homography fallback)"]:::c
    DP --> O["ball/point · object_features"]:::c
    FG --> O
```

---

## 9. Deployment & CI — what builds where

```mermaid
flowchart TB
    classDef ci fill:#0b3d91,color:#fff;
    classDef dev fill:#ffe2b3,stroke:#333,color:#000;
    classDef bot fill:#14532d,color:#fff;

    subgraph CLOUD["GitHub Actions (cloud, x86)"]
        C1["build + test ros2_ws (Jazzy)"]:::ci
        C2["lint + firmware host tests"]:::ci
        C3["build CPU Dockerfile.runtime (arm64) → GHCR"]:::ci
    end
    subgraph ONDEV["On the Jetson / self-hosted arm64 runner"]
        D1["docker build -f Dockerfile.jetson<br/>--target zed-driver-image"]:::dev
        D2["docker build -f Dockerfile.jetson<br/>--target soccer-app-image"]:::dev
    end
    subgraph ROBOT["Robot (Ansible)"]
        R0["provision.yml (once): CDI fixes + swap"]:::bot
        R1["deploy.yml: camera + app containers (--gpus, /dev, zed_resources)"]:::bot
    end
    C3 -. "optional registry path" .-> R1
    D1 --> R1
    D2 --> R1
    R0 --> R1
```

**Why the Jetson image is not built in cloud CI.** A 19 GB image that downloads a
licensed ZED SDK and compiles the wrapper, built for arm64 under QEMU emulation,
is impractical and slow in hosted CI. CI keeps building the lean **CPU**
`Dockerfile.runtime` for sim and the no-GPU developer flow; the **GPU** image is
built on-device (or a self-hosted arm64 runner) and shipped via Ansible. This is
documented inline in [`.github/workflows/ci.yml`](../../.github/workflows/ci.yml).

---

## 10. Files changed

_Original ZED-integration change set. The later **Option B** follow-up (bridge
removal → direct component remap) is captured in §10b below._

| File                                                                 | Change                                                                                                         | Justification                                                                                  |
| -------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| `deploy/docker/Dockerfile.jetson`                                    | **New.** Multi-stage, two targets (`zed-driver-image`, `soccer-app-image`).                                    | Encodes the proven CUDA 13.2 + ZED SDK 5.4 + TensorRT + wrapper recipe; isolates GPU coupling. |
| `deploy/docker/entrypoint.sh`                                        | Source both `/ws/install` and `/ros2_ws/install`.                                                              | One entrypoint serves both Jetson targets.                                                     |
| `ros2_ws/src/soccer_bringup/soccer_bringup/camera_bridge.py`         | **New** `camera_bridge` node.                                                                                  | Maps ZED topics → contract with QoS correction (composable node can't be remapped).            |
| `ros2_ws/src/soccer_bringup/scripts/camera_bridge_node`              | **New** executable wrapper.                                                                                    | Lets the node be launched, mirroring `sim_camera_node`.                                        |
| `ros2_ws/src/soccer_bringup/launch/camera.launch.py`                 | **New.** Runs the bridge; optional in-process ZED via `launch_driver`.                                         | The real-hardware analogue of the sim camera.                                                  |
| `ros2_ws/src/soccer_bringup/launch/robot.launch.py`                  | Add `camera` (sim\|zed) + `camera_model` args; branch sim node vs `camera.launch.py`; drop `use_depth:=false`. | Decouples camera source from the sim/real hardware plugin; default stays `sim`.                |
| `ros2_ws/src/soccer_bringup/CMakeLists.txt`                          | Install `camera_bridge_node`.                                                                                  | So the new executable is found at runtime.                                                     |
| `ros2_ws/src/soccer_bringup/package.xml`                             | Add `rclpy`, `sensor_msgs` exec-deps.                                                                          | The bridge's runtime deps (rosdep correctness).                                                |
| `ros2_ws/src/soccer_perception/soccer_perception/projection_node.py` | Subscribe `camera_info`; depth on by default.                                                                  | Use the ZED's real calibration + accurate depth, with auto flat-ground fallback.               |
| `deploy/compose/robot.compose.yaml`                                  | **New.** `camera` + `robot` services (GPU/CDI, host net, `/dev`, `zed_resources`).                             | One-command on-device bring-up matching the proven run contract.                               |
| `deploy/ansible/provision.yml`                                       | **New.** CDI hook fix + `mode=cdi` + swap (+ verify).                                                          | Makes the mandatory host fixes reproducible per robot.                                         |
| `deploy/ansible/deploy.yml`                                          | Two-container GPU stack; registry **or** on-device images.                                                     | Deploys the new camera + app images, not the old single runtime.                               |
| `deploy/ansible/README.md`                                           | Document `provision.yml` + the two deploy models.                                                              | Operator guidance.                                                                             |
| `docs/jetson_zed_workflow.md`                                        | Status banner + corrected facts (R39.2, CUDA base, topic names, `--gpus`, bridge).                             | Remove now-disproven assumptions; point to this report.                                        |
| `.github/workflows/ci.yml`                                           | Comment: Jetson image builds on-device, not cloud CI.                                                          | Sets the build-location expectation.                                                           |

---

## 10b. Follow-up — bridge removed, direct component remap (Option B)

A later change replaced the `camera_bridge` rclpy relay with **direct ZED-component
remapping** (see the rewritten §5). It removes a full-res inter-process hop and the
per-frame Python (de)serialization, adopts best-effort SensorData QoS end-to-end,
and keeps the pipeline zero-copy-ready. File changes on top of the table above:

| File                                                                                              | Change                                                                                       | Justification                                                              |
| ------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------- |
| `soccer_bringup/soccer_bringup/camera_bridge.py` + `scripts/camera_bridge_node`                   | **Deleted.**                                                                                 | The relay is gone; the ZED component publishes the contract itself.       |
| `soccer_bringup/launch/camera.launch.py`                                                          | Loads `stereolabs::ZedCamera` as a component with the four topic remaps + `use_intra_process_comms`. | One process, one hop, zero-copy-ready.                                     |
| `soccer_bringup/CMakeLists.txt`                                                                    | Drop the `camera_bridge_node` install.                                                       | The bridge executable no longer exists.                                   |
| `soccer_bringup/launch/robot.launch.py`                                                            | `camera:=zed` consumes the external contract; `launch_driver:=true` starts the ZED in-process. | The ZED runs in its own container; app container only consumes.           |
| `detector_node` / `fieldline_node` / `projection_node` / `ekf_node` / `sim_camera`                | Camera + IMU subs/pub → `qos_profile_sensor_data`.                                            | Best-effort SensorData QoS end-to-end (REP-2003).                         |
| `deploy/compose/zed_params_override.yaml`                                                          | Add `qos_overrides` → `best_effort` on the four contract topics.                              | Best-effort at the source; the wrapper honours rclcpp QoS overrides.      |
| `deploy/docker/Dockerfile.jetson`                                                                  | `COPY` `camera.launch.py` into the driver image; CMD runs it by path.                        | The component launch (not the stock wrapper launch) drives the camera.    |
| `deploy/compose/robot.compose.yaml`                                                                | `camera` service runs `camera.launch.py`; app `robot` service only consumes the contract.    | Two-container split preserved; no relay in the app container.             |

---

## 11. Verification on the device

Build the two images on the Jetson (the camera image `robosoccer-zed-ros:jazzy`
from the journey is equivalent; here we build via the repo's Dockerfile):

```bash
cd ~/soccer-bot      # the repo on the robot

# Camera driver image (~19 GB; uses swap from provision.yml).
sudo docker build -f deploy/docker/Dockerfile.jetson \
  --target zed-driver-image -t soccer-zed:jazzy .

# App image (lean).
sudo docker build -f deploy/docker/Dockerfile.jetson \
  --target soccer-app-image -t soccer-app:jazzy .
```

Bring the stack up (ZED Mini on a **USB 3.0** port; host fixes already applied):

```bash
cd deploy/compose && docker compose -f robot.compose.yaml up
```

Check the contract is alive under the robot namespace (second shell, on the host
or any same-`ROS_DOMAIN_ID=42` machine):

```bash
ros2 topic hz  /robot_1/camera/image_raw      # expect ~30 Hz
ros2 topic hz  /robot_1/imu/data              # expect ~99 Hz
ros2 topic echo /robot_1/camera_info --once   # K populated from the ZED
ros2 topic hz  /robot_1/camera/depth          # depth_mode NEURAL_LIGHT (needs TensorRT)
```

> **One item to confirm with depth enabled:** the exact depth topic name. The
> remap in `camera.launch.py` assumes `~/depth/depth_registered`
> (`/zed/zed_node/depth/depth_registered`); if your wrapper config differs, adjust
> that one remap entry — no code change elsewhere.

---

## 12. Risks, limits, and follow-ups

| Item                                  | Status / mitigation                                                                                                                                             |
| ------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ZED Mini is rolling-shutter           | Fine for PoC; if line blur hurts MCL, a global-shutter ZED X is a `camera_model` arg, not a code change.                                                        |
| App image apt dep names               | The soccer-app stage lists common deps explicitly **and** runs `rosdep`; if a name drifts on a future Jazzy sync, rosdep still resolves it.                     |
| RF-DETR detector (GPU)                | Not enabled yet; when it lands, uncomment the GPU reservation on the `robot` service and build the `.engine` **on the target** (engines are per-GPU + per-TRT). |
| Single camera/Jetson                  | Record rosbags of the contract topics so no-GPU devs work without hardware (workflow doc §6, §8).                                                               |
| Cloud CI does not build the GPU image | By design; build on-device / self-hosted arm64 runner.                                                                                                          |

---

## 13. Decision log (what was chosen, and why)

1. **Mirror the proven recipe into the repo** rather than keep it as external
   notes — so deployment is reproducible and reviewable.
2. **Multi-stage, multi-target Dockerfile** (your selection) — one shared base,
   two minimal images, no base drift.
3. **A bridge node, not remap or `topic_tools relay`** — the ZED node is
   composable (can't be remapped) and a fixed-QoS relay drops sensor frames.
4. **Keep consumers driver-agnostic** — the contract is unchanged, so the sim
   flow and every downstream node are untouched.
5. **Depth on by default with auto fallback** — one node is correct in sim and on
   hardware; no per-mode flags to forget.
6. **Host fixes in Ansible `provision.yml`** — the CDI hook bug + `mode=cdi` are
   mandatory and easy to get wrong; encode them once.
7. **`--gpus all` (CDI), never `--runtime nvidia`** — the legacy CSV path re-adds
   the broken hook.
8. **GPU image built on-device, CPU image in CI** — match each artifact to where
   it can actually, practically build.
9. **Match versions to the platform, not the newest** — TensorRT pinned to the
   JetPack build (10.16.2.10), base pinned to CUDA 13.2.

```

```
