# tools — developer scripts & calibration

Host-side helpers that are **not** deployed to robots.

| Script                   | Purpose                                                                                       |
| ------------------------ | --------------------------------------------------------------------------------------------- |
| `mock_gamecontroller.py` | Broadcast GameController packets on :3838 to drive the stack in sim (no real referee needed). |
| `calibrate_camera.py`    | Produce monocular intrinsics (`fx, fy, cx, cy`) for `soccer_perception`.                      |
| `dev_shell.sh`           | Drop into the dev Docker container with the repo mounted.                                     |
| `rfdetr_bench.py`        | Export RF-DETR to ONNX, build an FP16 TensorRT engine and measure on-device latency.          |

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
