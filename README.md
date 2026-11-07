# soccer-software

ROS 2 Jazzy (C++20) software for the UTRA RoboSoccer humanoid: the real-time actuator stack, a
MuJoCo software-in-the-loop (SIL) simulation, and the robot model generators.

| Path | What it is |
|---|---|
| `src/humanoid_transport` | The `ActuatorTransport` interface and control period. No ROS. |
| `src/humanoid_transport_mujoco` | MuJoCo transport for SIL |
| `src/humanoid_transport_stm32` | USB CDC transport to the master STM32 |
| `src/humanoid_actuator_system` | ros2_control hardware component that runs a transport behind the SafetyKernel |
| `src/humanoid_safety` | SafetyKernel |
| `src/humanoid_mit_controller` | `MitImpedanceController` |
| `src/humanoid_bringup` | Launch files and configuration |
| `model/` | Robot model sources and the generators that emit URDF, MJCF and manifests |

## Setup

Development happens in the dev container (`.devcontainer/`), which has ROS 2, MuJoCo and every
dependency. You need Docker, VS Code, and the VS Code Dev Containers extension.

### Linux

1. Install Docker Engine and add yourself to the `docker` group.
2. Clone the repository, `cd` into it, and run `code .`.
3. Run "Dev Containers: Reopen in Container".

The GUI (RViz2, slider GUI) uses the host's X display. The STM32 master appears in the container as
`/host-dev/ttyACM0`, or `/host-dev/robosoccer-master` with the udev rule from soccer-firmware
(`tools/udev/99-robosoccer-master.rules`).

### Windows (WSL2)

1. Install WSL2 with Ubuntu, Docker Desktop with the WSL2 backend (enable integration for your
   Ubuntu distribution), and VS Code with the WSL and Dev Containers extensions.
2. Clone the repository **inside WSL** (e.g. `~/soccer-software`, not `/mnt/c/...`).
3. From a WSL shell, `cd` into it and run `code .`, then "Dev Containers: Reopen in Container".

Open the folder this way. If VS Code opens it from Windows instead, the container cannot reach
WSLg's display, so the GUI does not work.

The STM32 master is a USB device on Windows, so it has to be attached to WSL with
[usbipd-win](https://github.com/dorssel/usbipd-win). In an administrator PowerShell:

```powershell
winget install usbipd
usbipd list                                 # find the master's BUSID
usbipd bind --busid <BUSID>                 # once
usbipd attach --wsl --busid <BUSID> --auto-attach
```

It then appears in the container as `/host-dev/ttyACM0`. Pass `serial_device:=/host-dev/ttyACM0` to
the bench launch: the `robosoccer-master` symlink comes from a host udev rule that is not set up
here. This path is untested with the master; if `ls /host-dev/ttyACM*` in the container shows
nothing, the board is not reaching it.

### Jetson

Remote-SSH to the Jetson, then "Reopen in Container". It has no usable display: launch with
`rviz:=false foxglove:=true` and view in Foxglove. See [docs/dev-container.md](docs/dev-container.md).

## Build and test

In the container:

```bash
colcon build
colcon test && colcon test-result --verbose
python3 -m model.generators.cli check   # after any change under model/
```

## Run

Every launch needs a Zenoh router running first:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

```bash
ros2 launch humanoid_bringup sil_stand.launch.py   # MuJoCo SIL stand scenario (G1)
ros2 launch humanoid_bringup bench.launch.py       # dial bench, slider view, no hardware
ros2 launch humanoid_bringup leg.launch.py         # leg prototype on its stand, MuJoCo SIL
```

## Docs

- [docs/bench.md](docs/bench.md): the dial benches, simulated and real
- [docs/leg-prototype.md](docs/leg-prototype.md): the leg prototype, its four-bar ankle, SIL, and
  the first hardware session
- [docs/transport-stm32.md](docs/transport-stm32.md): the STM32 transport and first contact with hardware
- [docs/dev-container.md](docs/dev-container.md): the container's display handling and SIL viewers
- [docs/model-second-robot.md](docs/model-second-robot.md): adding a robot to `model/`
- [AGENTS.md](AGENTS.md) and [docs/standards/CPP_GUIDELINES.md](docs/standards/CPP_GUIDELINES.md):
  the rules for changing code
