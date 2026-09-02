# tools — developer scripts & calibration

Host-side helpers that are **not** deployed to robots.

| Script                       | Purpose                                                                                       |
| ---------------------------- | --------------------------------------------------------------------------------------------- |
| `preflight.sh`               | On-device health check. Run before any hardware bring-up; `make robot` runs it automatically. |
| `check_repo_invariants.py`   | Static checks for the failure modes in `docs/TROUBLESHOOTING.md`. Runs in CI and `make check`. |
| `mock_gamecontroller.py`     | Broadcast GameController packets on :3838 to drive the stack in sim (no real referee needed). |
| `calibrate_camera.py`        | Produce monocular intrinsics (`fx, fy, cx, cy`) for `soccer_perception`.                      |
| `dev_shell.sh`               | Drop into the dev Docker container with the repo mounted.                                     |
| `rfdetr_bench.py`            | Export RF-DETR to ONNX, build an FP16 TensorRT engine and measure on-device latency.          |

## Before touching hardware

```bash
make check       # static, seconds, no ROS/GPU/Docker needed
make preflight   # on-device: GPU, CDI, images, host tuning, camera
```

`preflight.sh` exits non-zero on any FAIL and prints the fix for each one. Each
check exists because that failure actually happened on this robot — see
[docs/TROUBLESHOOTING.md §10](../docs/TROUBLESHOOTING.md) for the mapping from
failure to guard.

```bash
# Start a 2-robot sim, then put them in PLAYING:
ros2 launch soccer_bringup team.launch.py num_robots:=2
python tools/mock_gamecontroller.py --state playing
```

`rfdetr_bench.py` runs **on the Jetson**, inside an image built from the ZED image
so the engines are compiled against the exact TensorRT the deployed detector links:

```bash
docker build -f deploy/docker/Dockerfile.rfdetr-bench -t soccer-rfdetr-bench:jazzy .
docker run --rm --runtime nvidia -v ~/rfdetr-bench-models:/models \
  soccer-rfdetr-bench:jazzy python3 /opt/soccer/rfdetr_bench.py --sizes nano small
```
