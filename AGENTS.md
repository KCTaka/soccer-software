# Agent and contributor instructions

ROS 2 Jazzy (C++20) software for a RoboCup humanoid. Read
[docs/standards/CPP_GUIDELINES.md](docs/standards/CPP_GUIDELINES.md) before changing C++. It holds
the rules below with their reasons. Architecture decisions are cited as ADR-xxx and live in the
project's research-docs repository.

## Build and verify

```bash
source /opt/ros/jazzy/setup.bash
colcon build
colcon test && colcon test-result --verbose   # must report 0 failures; skips are not passes
python3 -m model.generators.cli check         # after any change under model/
```

SIL launch (`ros2 launch humanoid_bringup sil_stand.launch.py`, `robot:=<robot>` to choose one
with a `config/<robot>/sil_stand.yaml`; default `unitree_g1`) needs a Zenoh router first:
`ros2 run rmw_zenoh_cpp rmw_zenohd`. It starts RViz2 by default; on a host without a usable X
display (the Jetson over Remote-SSH) use `rviz:=false foxglove:=true` and open Foxglove at
`ws://localhost:8765`. See [docs/dev-container.md](docs/dev-container.md).

## Non-negotiables

- Nothing on the real-time path (`read`/`update`/`write`/`exchange`/`SafetyKernel`) allocates,
  logs, blocks, or calls ROS. Cross-thread data goes through `LatestValueBuffer`.
- `humanoid_transport` never depends on ROS.
- No environment variables for configuration: declare ROS 2 parameters.
- No numbers copied from the robot model into code: derive them from the loaded model, or
  regenerate through `model/generators` from `model/robots/<robot>/robot_model.yaml`. It writes
  the description (URDF, MJCF, meshes, safety manifest, STM32 wiring) to
  `src/humanoid_description/robots/<robot>/` and the firmware export to
  `model/robots/<robot>/generated/`. Never hand-edit generated files.
- The control period is defined only in `humanoid_transport/timing.hpp`; check runtime rates
  against it.
- Return by value (`std::optional` for "may be absent"). The only out-parameter exception is
  filling a caller-owned preallocated batch, e.g. `exchange()`'s feedback.
- No counter without a reader; account for a call at its single caller.
- Formatting is `ament_uncrustify` only. Do not add a `.clang-format`.
- Commit messages follow [.github/git-commit-instructions.md](.github/git-commit-instructions.md).
