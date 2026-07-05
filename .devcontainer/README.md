# VSCode Dev Container Setup

This directory configures VSCode to develop inside a container with the full ROS 2 Jazzy + build toolchain pre-installed.

## Quick Start

### Option 1: VSCode Dev Container (Recommended)

1. Install the **Dev Containers** extension in VSCode (ms-vscode-remote.remote-containers)
2. Open this repo in VSCode
3. Click the green `><` icon in the bottom-left → **Reopen in Container**
4. VSCode opens the folder **inside** the container automatically

Then in the integrated terminal:

```bash
# Build the workspace
make build

# Source and launch
cd ros2_ws && source install/setup.bash
ros2 launch soccer_bringup robot.launch.py sim:=true
```

See [Makefile](../Makefile) for all available build targets.

**Benefits:**

- Full ROS 2 environment without local install
- All extensions (CMake, C++, Python, ROS) work inside the container
- Code editing on Windows; compilation/execution in Linux
- One-click setup for new team members

### Option 2: Compose CLI (Lightweight)

```bash
# Start the sim stack
make sim

# Or start an interactive dev shell
docker compose -f deploy/compose/sim.compose.yaml run dev bash

# Inside the container:
make build
cd ros2_ws && source install/setup.bash
ros2 launch soccer_bringup robot.launch.py sim:=true
```

## What's Pre-installed

- ROS 2 Jazzy + `ros2_control` framework
- C++ build tools (CMake, GCC, Clang)
- Python 3.12 + NumPy, OpenCV, ruff
- BehaviorTree.CPP
- Git

## File Syncing

The container mounts your Windows repo as a volume (`-v ../..:/ws`). Any edits on Windows are **instantly visible** inside the container — no manual sync needed.

```
Windows (C:\dev\soccer-bot)
          ↓ (Docker volume mount)
Linux container (/ws)
```

## Persistent Build Cache

The `.devcontainer/devcontainer.json` runs `make build` (`colcon build --symlink-install`) on container creation. Subsequent edits to Python/launch/yaml files take effect immediately without rebuilding. C++ changes require `make build-pkg pkg=<name>`.

## Requirements

- **Docker Desktop for Windows** with WSL2 backend
- **VSCode** + **Dev Containers extension** (for Option 1)

## Troubleshooting

**Container won't start:**

```bash
# Check Docker is running
docker ps

# Rebuild from scratch
docker compose -f deploy/compose/sim.compose.yaml build --no-cache dev
```

**ROS 2 commands not found:**

```bash
# Re-source the setup scripts in your shell
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
```

**Volume mount not updating:**

```bash
# Restart the container
docker compose -f deploy/compose/sim.compose.yaml restart dev
```
