# Troubleshooting, Performance Profile & Detector Roadmap

> **What this is.** A living reference for the failure modes that are hard to
> diagnose from the symptom alone — the ones where the stack reports success,
> logs nothing alarming, and simply does not work. Each entry records the
> **symptom**, the **root cause**, the **fix**, and the **command that proves
> it**. It also records the measured performance envelope of the robot and the
> plan that follows from it.
>
> **Provenance.** Findings from a live diagnostic session on `robot1` (Jetson
> Orin Nano Super, JetPack 7.2 / L4T R39.2) on **2026-09-02**. Every number
> below was measured on that hardware, not estimated. Where this document
> disagrees with an earlier one, this one is newer — the earlier docs carry
> dated pointers back here.
>
> **Related.** [`zed_jetson_integration.md`](zed_jetson_integration.md) is the
> as-built integration record; [`bring_up_investigation_report.md`](bring_up_investigation_report.md)
> is the June 2026 bring-up investigation. This document supersedes specific
> claims in both, called out in §8.

---

## 0. TL;DR — what was broken

| #  | Symptom the operator sees                         | Real cause                                                   | Status  |
| -- | ------------------------------------------------- | ------------------------------------------------------------ | ------- |
| §1 | Every `docker run` fails after a reboot           | CDI spec written to tmpfs, generated before the driver loads  | Fixed   |
| §2 | ZED advertises all topics, publishes zero frames   | `camera.launch.py` was missing `robot_state_publisher`        | Fixed   |
| §3 | App container exits instantly, `ros2: not found`   | Dockerfile stage 4 inherited a base with no ROS runtime       | Fixed   |
| §4 | `colcon build` warns on every controller cycle     | `ros2_control` Jazzy deprecated the `get_value` / `set_value` API | Fixed |
| §5 | Camera drops 30 Hz → 20 Hz when the stack runs     | CPU saturation, **not** DDS loss or GPU load                  | Open    |

The three "Fixed" platform bugs (§1–§3) shared one property: **nothing logged an
error**. The CDI failure surfaced as a Docker message that named the wrong
subsystem, the ZED node stalled on an `INFO` line, and the app container failed
before any ROS logging existed to record it. Assume silence is not success.

---

## 1. NVIDIA CDI spec vanishes on every reboot

> **Severity: critical.** Blocks *every* container on the robot, not just GPU
> ones.

### 1.1 Symptom

```
docker: Error response from daemon: failed to inject CDI devices:
unresolvable CDI devices nvidia.com/gpu=all
```

This appears for **any** image, including ones with no GPU code, because
`/etc/docker/daemon.json` sets `"default-runtime": "nvidia"` and the images carry
`NVIDIA_VISIBLE_DEVICES=all`. Every container therefore requests
`nvidia.com/gpu=all` and every container fails.

### 1.2 Root cause

Two independent problems compound:

| Problem                                                         | Consequence                                                    |
| --------------------------------------------------------------- | -------------------------------------------------------------- |
| The vendor unit sets `NVIDIA_CTK_CDI_OUTPUT_FILE_PATH=/var/run/cdi/nvidia.yaml` | `/var/run` is **tmpfs** — the spec is destroyed on every boot |
| `nvidia-cdi-refresh.service` runs **before** the NVIDIA kernel module is loaded | Regeneration fails, so the destroyed spec is never rebuilt      |

The boot ordering failure is not obvious because the unit *looks* guarded:

```ini
ExecCondition=/bin/sh -c '/usr/bin/grep -qE "/(nvidia|nvidia-current)[.]ko" \
    /lib/modules/%v/modules.dep || [ -e /dev/dxg ]'
```

That condition greps `modules.dep` — a **file on disk**. It passes whether or not
the module is actually loaded into memory. `nvidia-ctk cdi generate` then runs,
fails to reach the driver, and exits:

```
failed to initialize NVML: Driver Not Loaded
```

> **The diagnostic tell.** The failed run is journalled with a timestamp of
> `Dec 31 19:00:30 1969` — the service runs before the RTC is synced, which is
> itself the proof that it ran too early in boot.

```mermaid
sequenceDiagram
    participant B as boot
    participant S as nvidia-cdi-refresh.service
    participant K as nvidia.ko
    participant F as /var/run/cdi (tmpfs)
    participant D as docker run
    B->>F: tmpfs mounted — previous spec is gone
    B->>S: start (ExecCondition passes: modules.dep exists on disk)
    S->>S: nvidia-ctk cdi generate
    S--xK: NVML: Driver Not Loaded (module not yet in memory)
    S--xF: nothing written
    B->>K: module finally loads
    D->>F: resolve nvidia.com/gpu=all
    F--xD: no spec -> unresolvable CDI devices
```

### 1.3 Fix

Redirect the output to persistent storage. The unit reads `EnvironmentFile`
**after** `Environment=`, so a single line in the config file overrides the
vendor default without editing the unit:

```bash
# /etc/nvidia-container-toolkit/nvidia-cdi-refresh.env
NVIDIA_CTK_CDI_GENERATE_DISABLED_HOOKS=enable-cuda-compat
NVIDIA_CTK_CDI_OUTPUT_FILE_PATH=/etc/cdi/nvidia.yaml
```

Then drop the stale tmpfs copy and regenerate:

```bash
sudo rm -f /var/run/cdi/nvidia.yaml   # two specs of the same kind conflict
sudo systemctl restart nvidia-cdi-refresh.service
```

**Why this is sufficient.** With the spec on disk, a failed boot-time
regeneration becomes harmless — `nvidia-ctk` writes only on success, so the last
good spec survives. The race still happens; it just no longer matters.

Codified as **Fix 0** in [`deploy/ansible/provision.yml`](../deploy/ansible/provision.yml),
which also removes the tmpfs copy. Note that `provision.yml` already declared
`cdi_spec: /etc/cdi/nvidia.yaml` and read that path to verify the result — the
playbook had always *assumed* this layout; the environment did not match it.

### 1.4 Verify

```bash
ls -la /etc/cdi/nvidia.yaml                       # ~95 kB, CDI spec version 0.7.0
systemctl show -p Result --value nvidia-cdi-refresh.service   # success
docker run --rm nvcr.io/nvidia/cuda:13.2.1-runtime-ubuntu24.04 nvidia-smi -L
# GPU 0: Orin (nvgpu) (UUID: ...)
```

---

## 2. ZED node advertises every topic and publishes nothing

> **Severity: critical.** The camera appears healthy in `ros2 topic list` and in
> the node's own startup banner.

### 2.1 Symptom

Every contract topic exists and reports `Publisher count: 1`, but no data flows:

- `ros2 topic hz /robot_1/camera/image_raw` — prints nothing, ever
- `ros2 topic bw` — subscribes successfully, receives zero messages
- The node log ends on an **`INFO`** line and never advances:

```
[zed_node]: === Starting Positional Tracking ===
[zed_node]:  * Waiting for valid static transformations...
```

There is no error, no warning, and no timeout. The grab loop simply never starts.

### 2.2 Root cause

`camera.launch.py` loaded the `stereolabs::ZedCamera` component and nothing else.
The ZED SDK blocks startup until the camera's own TF chain
(`<camera_name>_left_camera_frame` → `<camera_name>_camera_link`) is resolvable.
That chain is published by a `robot_state_publisher` running the ZED's URDF —
which the stock `zed_camera.launch.py` starts, and ours did not:

```bash
ros2 topic info /tf_static
# Publisher count: 0        <-- the entire diagnosis
```

The trap is structural: adopting the component **without** adopting the rest of
the vendor launch file silently removes a hard dependency. The component does not
declare it, and does not complain when it is absent.

### 2.3 Fix

[`camera.launch.py`](../ros2_ws/src/soccer_bringup/launch/camera.launch.py) now
returns `[rsp_node, container]`. `robot_description` is remapped to
`<camera_name>_description` so it cannot collide with the robot's own description
published by `robot.launch.py`:

```python
rsp_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="zed_state_publisher",
    namespace=robot_name,
    parameters=[{
        "robot_description": Command(
            ["xacro ", zed_descr,
             " camera_name:=", camera_name,
             " camera_model:=", camera_model]
        ),
    }],
    remappings=[("robot_description", f"{camera_name}_description")],
)
```

All three dependencies were already present in `zed-driver-image`
(`/opt/ros/jazzy/bin/xacro`, `robot_state_publisher`, and
`zed_description/urdf/zed_descr.urdf.xacro`) — only the node was missing.

### 2.4 Verify

A healthy start logs the publisher, then resolves the transforms and proceeds:

```
[robot_1.zed_state_publisher]: Robot initialized
[robot_1.zed_node]:  Static transform ref. CMOS Sensor to Base
                     [zed_left_camera_frame -> zed_camera_link]
```

Measured immediately afterwards on a fresh container, HD720, MAXN_SUPER:

| Topic                        | Rate       | Notes                                    |
| ---------------------------- | ---------- | ---------------------------------------- |
| `/robot_1/camera/image_raw`  | 30.45 Hz   | 1280×720 BGRA, 3.69 MB/frame, 107 MB/s   |
| `/robot_1/camera/depth`      | 29.3 Hz    | `NEURAL_LIGHT`                           |
| `/robot_1/imu/data`          | 99–102 Hz  |                                          |
| `/robot_1/zed_node/odom`     | 29.8 Hz    | VIO                                      |

---

## 3. App image contains no ROS runtime

> **Severity: critical.** `soccer-app:jazzy` could not execute its own `CMD`.

### 3.1 Symptom

```
$ docker run --rm --entrypoint bash soccer-app:jazzy -c 'ros2 --help'
bash: ros2: command not found
```

Missing: `ros-jazzy-ros-base`, `ros-jazzy-rclpy`, `ros-jazzy-ros2-control`,
`ros-jazzy-ros2-controllers`, `ros-jazzy-cv-bridge`,
`ros-jazzy-robot-state-publisher`. The image still weighed 11.4 GB, which is why
this went unnoticed — the bulk is the CUDA base.

### 3.2 Root cause

`Dockerfile.jetson` stage 4 was:

```dockerfile
FROM jetson-base AS soccer-app-image
WORKDIR /ws
COPY --from=soccer-app-build /ws/install ./install
RUN apt-get install -y ros-jazzy-rmw-cyclonedds-cpp
```

`jetson-base` adds the **ROS apt repository**, not ROS. Every runtime dependency
was installed only in stage 3 (`soccer-app-build`), which is discarded. Stage 4
copied the compiled workspace but none of the libraries it links against, leaving
`install/lib/libresidual_rl_controller.so` with no `libcontroller_interface.so`
to load against and no `ros2_control_node` to load it.

The only reason `/opt/ros/jazzy` existed at all was that
`ros-jazzy-rmw-cyclonedds-cpp` drags in a slice of ROS core as dependencies —
enough to look plausible, not enough to run.

> **What hid the bug.** The header of
> [`soccer-app-deps.apt`](../deploy/docker/soccer-app-deps.apt) claimed it was
> used by "the `soccer-app-image` stage". It was only ever applied to the build
> stage. The comment described the intended design, so reading it confirmed a
> behaviour that did not exist.

### 3.3 Fix

Apply the same dependency file in stage 4, **before** the `install/` copy so the
multi-GB apt layer stays cache-stable when only sources change:

```dockerfile
COPY deploy/docker/soccer-app-deps.apt /tmp/deps.apt
RUN apt-get update \
    && grep -v '^\s*#' /tmp/deps.apt | tr -d '\r' | xargs apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/deps.apt

COPY --from=soccer-app-build /ws/install ./install
```

Using the one shared file keeps the runtime image byte-identical in package set
to what the workspace was compiled against. The deps file header was corrected to
match reality.

### 3.4 Verify

```bash
docker run --rm --entrypoint bash soccer-app:jazzy -c '
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  ldd /ws/install/lib/libresidual_rl_controller.so | grep -c "not found"
  ros2 launch soccer_bringup robot.launch.py --show-args | head'
```

Expect `0` unresolved libraries. A full bring-up now reaches:

```
[controller_manager]: Activating controllers: [ residual_rl_controller ]
[controller_manager]: Successfully switched controllers!
```

> **Generalise this.** A multi-stage image should be tested by running its own
> `CMD`, not by inspecting that the artefacts were copied.

---

## 4. `ros2_control` Jazzy API migration

### 4.1 What changed

Jazzy deprecated the direct-value accessors on loaned interfaces. Both are
scheduled for removal in **Kilted Kaiju**:

| Old (deprecated)                        | New                                                        |
| --------------------------------------- | ---------------------------------------------------------- |
| `double LoanedStateInterface::get_value()` | `std::optional<T> get_optional(unsigned int max_tries = 10)` |
| `LoanedCommandInterface::set_value(v)` (return ignored) | `[[nodiscard]] bool set_value(const T&, unsigned int max_tries = 10)` |

Both new forms are `[[nodiscard]]`. The change exists because the interfaces are
now mutex-guarded: a read can fail if the handle stays contended for all retries,
and a write can fail for the same reason. The old API had no way to say so.

### 4.2 Fix applied

[`residual_rl_controller.cpp`](../ros2_ws/src/soccer_control/src/residual_rl_controller.cpp)
now treats both outcomes as real control-loop events rather than discarding them:

- **Unreadable state** → hold the previous command and return `OK`. Commanding on
  a garbage reading is worse than repeating a known-good one for one cycle.
- **Failed write** → throttled warning. The cycle is already lost; do not also
  flood the log at 100 Hz.

`RCLCPP_WARN_THROTTLE` at 1000 ms is used for both, because this code runs at
100 Hz and an unthrottled warning would be its own failure mode.

A repo-wide search confirmed this was the only file using either deprecated call.

### 4.3 Verify

```bash
# The host has no ROS; build in the dev container (see §7.2)
colcon build --packages-select soccer_control
wc -c ros2_ws/log/latest_build/soccer_control/stderr.log   # 0
```

An empty `stderr.log` is the pass condition.

---

## 5. Performance profile — the bottleneck is CPU

> **This is the single most important operational fact about the robot.**

### 5.1 The measurement

Camera frame rate, measured on the publisher side, fresh containers in both
cases, 55 °C, `MAXN_SUPER`:

| Configuration                    | `image_raw` rate | Delta       |
| -------------------------------- | ---------------- | ----------- |
| Camera container alone           | **30.45 Hz**     | baseline    |
| Camera + full app stack          | **19.84 Hz**     | **−35 %**   |

The drop is on the **publisher** side. This rules out DDS loss, QoS mismatch and
subscriber slowness — `zed_node` itself is being starved of CPU.

### 5.2 Where the CPU goes

Six cores, so 600 % is full saturation. Measured with the full stack running:

| Process                | %CPU  | Notes                                       |
| ---------------------- | ----- | ------------------------------------------- |
| `zed_node`             | 82    | grab loop + `NEURAL_LIGHT` depth + VIO      |
| `detector_node`        | 54    | **Python HSV over 1280×720 frames**         |
| `mcl_node`             | 49    | particle filter                             |
| `fieldline_node`       | 48    | Python/OpenCV over full-res frames          |
| `ekf_node`             | 43    |                                             |
| `projection_node`      | 15    |                                             |
| `ros2_control_node`    | 13    |                                             |
| `teamcomm` / `gc_bridge` | 14  | combined                                    |

Container totals: `soccer-app` **234 %**, `soccer-camera` **76–108 %**. Load
average **7.1** on 6 cores.

Meanwhile **`GR3D_FREQ` (GPU) sits at 0–20 %.**

### 5.3 What this rules out

Each of these was tested and eliminated before concluding CPU:

| Hypothesis                        | Test                                                          | Result                                 |
| --------------------------------- | ------------------------------------------------------------- | -------------------------------------- |
| DDS dropping large samples        | Compared publisher-side vs subscriber-side rate                | Both ≈19 Hz — loss is upstream of DDS  |
| Socket buffers too small          | `sysctl net.core.rmem_max`                                     | 16 MB, correctly applied by `provision.yml` |
| Thermal throttling                | `tj-thermal` during the run                                   | 55 °C; throttle point is ~95 °C        |
| Degradation over uptime           | Restarted camera fresh, re-measured                            | 30.45 Hz again — not cumulative        |
| GPU contention                    | `tegrastats` during the run                                    | GPU 0–20 % idle                        |

### 5.4 Consequence

The robot is **CPU-bound with an idle GPU**. Any work that can be moved from the
CPU to the GPU buys back frame rate directly. `detector_node` at 54 % of a core
doing Python HSV thresholding is the clearest example on the platform.

---

## 6. RF-DETR on this hardware — measured, and the plan

### 6.1 Benchmarks

Measured on-device with `trtexec`, TensorRT **10.16.2.10**, FP16, batch 1,
CUDA graphs, `MAXN_SUPER`. Engines built from ONNX exported by `rfdetr` 1.9.4.

| Model  | Input   | Median GPU compute | p99      | Throughput | Engine | Published T4 | Ratio |
| ------ | ------- | ------------------ | -------- | ---------- | ------ | ------------ | ----- |
| Nano   | 384²    | **5.84 ms**        | 5.85 ms  | 171 qps    | 59 MB  | 2.3 ms       | 2.5×  |
| Small  | 512²    | **10.85 ms**       | 10.88 ms | 92 qps     | 61 MB  | 3.5 ms       | 3.1×  |
| Medium | 576²    | **14.15 ms**       | 14.48 ms | 71 qps     | 64 MB  | 4.4 ms       | 3.2×  |

The p99 sits within 0.35 ms of the median in every case — the latency is
effectively jitter-free, which matters more for a control loop than the mean.

> **Always benchmark on the target.** Roboflow publishes T4 numbers. This board
> is consistently 2.5–3.2× slower. A plan built on the published figures would
> have been wrong by a factor of three.

### 6.2 Under contention

Nano running flat-out **while the ZED streamed at 30 Hz**:

| Metric                | Idle       | Under contention | Change     |
| --------------------- | ---------- | ---------------- | ---------- |
| Median GPU compute    | 5.84 ms    | **5.85 ms**      | none       |
| p99                   | 5.85 ms    | 11.42 ms         | +5.6 ms    |
| Throughput            | 171 qps    | 154 qps          | −10 %      |
| ZED `image_raw`       | —          | **29.2 Hz**      | held       |
| GPU utilisation       | —          | **99 %**         |            |
| Power (`VDD_IN`)      | 5.8 W      | **21.3 W**       | +15.5 W    |
| Junction temperature  | 54 °C      | 66 °C            | no throttle |

Both workloads coexist. At a realistic 30 Hz duty cycle rather than flat-out,
Nano would occupy roughly **18 % of the GPU**.

### 6.3 Recommendation

**Adopt RF-DETR Nano** and retire the HSV fallback in `detector_node`.

| Consideration          | Assessment                                                                 |
| ---------------------- | -------------------------------------------------------------------------- |
| Latency budget         | 5.84 ms against a 33 ms frame period — 18 % of one frame                    |
| CPU impact             | **Net negative.** Inference moves to the idle GPU; frees most of `detector_node`'s 54 % |
| Accuracy              | 48.4 COCO AP at 384², well above what HSV thresholding achieves on a ball    |
| Thermal / power        | +15.5 W flat-out, 66 °C — inside envelope even at MAXN                       |
| Memory                 | 59 MB engine; ~3.7 GB RAM free with the camera running                       |
| Licensing              | Apache-2.0                                                                   |

Prefer **Small** only if ball detection at range proves insufficient — it costs
5 ms more and still clears 30 Hz. **Medium** has no headroom advantage here.

The `robot` service's GPU reservation in
[`robot.compose.yaml`](../deploy/compose/robot.compose.yaml) is commented out
with the note *"Uncomment to give it the GPU once the RF-DETR TensorRT detector
lands"*. §5 inverts the assumption behind that ordering: the GPU is not a
scarce resource being protected, it is idle capacity the CPU-bound stack needs.

> **Engines are not portable.** A TensorRT engine is specific to the GPU
> architecture *and* the TensorRT build. Build on the target, or rebuild in CI on
> a matching image — never commit an `.engine` and expect it to load.

### 6.4 Reproducing the benchmark

[`Dockerfile.rfdetr-bench`](../deploy/docker/Dockerfile.rfdetr-bench) builds
`FROM soccer-zed:jazzy` deliberately, so engines are compiled against the exact
TensorRT the deployed detector will link against.

```bash
docker build -f deploy/docker/Dockerfile.rfdetr-bench -t soccer-rfdetr-bench:jazzy .
docker run --rm --runtime nvidia -v ~/rfdetr-bench-models:/models \
  soccer-rfdetr-bench:jazzy python3 /opt/soccer/rfdetr_bench.py --sizes nano small medium
```

Two pinning details matter:

- `trtexec` comes from `libnvinfer-bin` pinned to the **L4T** repo. The NGC sbsa
  repo offers 11.x, which produces engines the ZED SDK runtime cannot
  deserialise.
- ONNX export uses **CPU** `torch` (`--index-url .../whl/cpu`). Export does not
  need the GPU, and the CPU wheel avoids pulling a second CUDA userspace into an
  already 21 GB image.

---

## 7. Environment notes that cost time

### 7.1 Power mode

The board ships in mode 1 (25 W). All figures above are mode 2:

```bash
sudo nvpmodel -m 2      # 0=15W, 1=25W, 2=MAXN_SUPER
```

### 7.2 There is no ROS on the host

`make build` cannot work on `robot1` — the host has neither `/opt/ros` nor
`colcon`. Workspace builds happen inside the VS Code dev container (image
`vsc-soccer-bot-<hash>`, repo mounted at `/ws`, workspace at `/ws/ros2_ws`).
Neither `soccer-app:jazzy` nor `soccer-zed:jazzy` can substitute:
`soccer-zed:jazzy` has `colcon` but no `controller_interface`; `soccer-app:jazzy`
has the runtime but no `colcon`.

To build one package without VS Code:

```bash
docker run --rm -v "$PWD:/ws" -w /ws/ros2_ws --entrypoint bash \
  vsc-soccer-bot-<hash> \
  -c 'source /opt/ros/jazzy/setup.bash && colcon build --packages-select soccer_control'
```

### 7.3 Benign log noise — do not chase these

All four are ZED-X / GMSL code paths that do not apply to a USB ZED Mini:

| Message                                                           | Verdict                        |
| ----------------------------------------------------------------- | ------------------------------ |
| `(Argus) Error 0x00030003: Connecting to nvargus-daemon failed`    | Expected — no CSI camera       |
| `[ZED-X][Warning] Failed to connect to zed_x_daemon`               | Expected — not a ZED X         |
| `GMSL DRIVER Diagnostic : Failed`                                  | Expected — not a GMSL board    |
| `Gravity alignment issues detected. Recomputing alignment...`      | One-shot at startup, harmless  |

`ZED_Diagnostic` also reports `Error: unable to open display` — it is a GUI tool
being run headless. Use `-c` and read the OK/Failed lines only.

---

## 8. Known tech debt

| Item                                                | Impact                                                                 | Next step                                              |
| --------------------------------------------------- | ---------------------------------------------------------------------- | ------------------------------------------------------ |
| **QoS overrides are dead for image + depth**        | Config claims best-effort; publishers are RELIABLE                     | See §8.1 — decide whether it is worth pursuing         |
| **CDI fix not reboot-validated**                    | Fix is verified live but the reboot path is untested                    | Reboot and confirm `/etc/cdi/nvidia.yaml` survives     |
| **CPU saturation (§5)**                             | Camera runs at 20 Hz instead of 30 Hz whenever the stack is up          | RF-DETR migration (§6.3), then re-measure              |
| **`fieldline_node` at 48 % CPU**                    | Second-largest avoidable CPU consumer after `detector_node`             | Profile; candidate for the same GPU treatment          |
| **`camera_info` double-advertised**                 | Topic reports ~59 Hz because two publishers exist                       | Cosmetic; confirm no consumer double-counts            |

### 8.1 The QoS overrides do not apply to the image topics

[`zed_params_override.yaml`](../deploy/compose/zed_params_override.yaml) sets
`best_effort` on four topics. Measured result:

| Topic                            | Override parameter        | Effective QoS   |
| -------------------------------- | ------------------------- | --------------- |
| `/robot_1/imu/data`              | accepted                  | **BEST_EFFORT** |
| `/robot_1/camera/camera_info`    | accepted (wrapper default)| RELIABLE        |
| `/robot_1/camera/image_raw`      | **"Parameter not set"**   | **RELIABLE**    |
| `/robot_1/camera/depth`          | **"Parameter not set"**   | **RELIABLE**    |

Querying them makes the node log the reason:

```
[rclcpp]: Failed to get parameters:
qos_overrides./robot_1/camera/image_raw.publisher.reliability
```

The image and depth publishers are created without `QosOverridingOptions`, so the
parameters are never declared and the YAML entries are inert. The file's own
comment — *"The ZED wrapper enables rclcpp QoS overrides on every publisher"* —
is incorrect, and so is the "end-to-end" claim in
[`zed_jetson_integration.md`](zed_jetson_integration.md) §5.

Two further defects in the same file: the key `/robot_1/camera_info` does not
match any topic (the real one is `/robot_1/camera/camera_info`), and the entire
block is keyed to `robot_1`, so it silently stops applying if `robot_name`
changes.

> **Is it worth fixing?** Probably not urgently. A RELIABLE publisher and a
> BEST_EFFORT subscriber **are** QoS-compatible, so nothing is broken, and §5
> proved the frame loss is CPU-bound rather than DDS-bound. Fixing it requires a
> patch to the ZED wrapper's publisher creation. Record it, do not chase it.

### 8.2 Claims in earlier docs that this document supersedes

| Document                                        | Claim                                                          | Correction |
| ----------------------------------------------- | -------------------------------------------------------------- | ---------- |
| `zed_jetson_integration.md` §4                  | Stage lineage produces a working app image                      | §3         |
| `zed_jetson_integration.md` §5                  | "best-effort SensorData **end-to-end**"                         | §8.1       |
| `zed_jetson_integration.md` §6                  | CDI covered by the two hook/mode fixes                          | §1         |
| `zed_jetson_integration.md` §12                 | "App image apt dep names" listed as a *risk*                    | §3 — it was a live defect |
| `bring_up_investigation_report.md` §12          | Same app-image risk entry                                       | §3         |

---

## 9. Diagnostic playbook

Ordered cheapest-first. Each step isolates a layer.

```bash
# 1. Can any container start at all?  (catches §1)
docker run --rm nvcr.io/nvidia/cuda:13.2.1-runtime-ubuntu24.04 nvidia-smi -L

# 2. Does the camera see the sensor?
docker run --rm --privileged --runtime nvidia -v /dev:/dev \
  -v compose_zed_resources:/usr/local/zed/resources \
  --entrypoint bash soccer-zed:jazzy -c '/usr/local/zed/tools/ZED_Diagnostic -c'

# 3. Is the TF chain up?  (catches §2 — this is the fast tell)
docker exec soccer-camera bash -c \
  'source /opt/ros/jazzy/setup.bash && ros2 topic info /tf_static'
#   Publisher count: 0  ->  robot_state_publisher is missing

# 4. Is the app image runnable?  (catches §3)
docker run --rm --entrypoint bash soccer-app:jazzy -c \
  'source /opt/ros/jazzy/setup.bash && ros2 pkg list | wc -l'

# 5. Publisher-side vs subscriber-side rate  (distinguishes §5 from DDS loss)
docker exec soccer-camera bash -c '... ros2 topic hz /robot_1/camera/image_raw'
docker exec soccer-app    bash -c '... ros2 topic hz /robot_1/camera/image_raw'
#   both low   -> CPU starvation at the publisher (§5)
#   pub high, sub low -> genuine transport problem

# 6. Where is the time going?
tegrastats --interval 1000     # GPU %, power, junction temp
docker stats --no-stream       # per-container CPU
top -bn2 -o %CPU               # per-process
```

> **Sourcing.** `docker exec` does not run the image `ENTRYPOINT`, so `ros2` will
> not be on `PATH`. Every `exec` needs
> `source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash` first —
> `/ros2_ws` in the camera container, `/ws` in the app container.
