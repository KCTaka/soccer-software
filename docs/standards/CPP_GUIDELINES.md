# C++ Engineering Guidelines

Rules for C++ in this repository. Each rule says **why** it exists and **how it is enforced**,
and points to real code. Where a tool enforces a rule, the tool is the authority and this page
only names it. Architecture decisions (ADR-xxx) live in the project's research-docs
repository; this page cites them by ID and never restates them.

Every rule here was learned by fixing a real defect in this codebase. Before breaking one, read
why it exists.

## 1. Tooling is the first authority

| Concern | Authority | Run it |
|---|---|---|
| Formatting (2 spaces, 100 cols) | `ament_uncrustify` (`ament_code_style_0_78.cfg`) | `colcon test`; fix with `ament_uncrustify --reformat <files>` |
| Lint, include order | `ament_cpplint`, `ament_cppcheck` | `colcon test` |
| `override` on every override | `-Wsuggest-override -Werror` in every package | `colcon build` |
| Generated model artifacts | regenerate-and-diff | `python3 -m model.generators.cli check` |
| Commit messages | [.github/git-commit-instructions.md](../../.github/git-commit-instructions.md) | review |

- There is deliberately **no `.clang-format`**. A second formatter drifts from uncrustify and
  fails the lint on freshly formatted code. [.editorconfig](../../.editorconfig) only makes
  editors agree with uncrustify.
- cpplint's header list predates C++20: put `#include <format>` in its own block ahead of the
  other standard headers, or `build/include_order` fails.
- A change is not done until `colcon build` and `colcon test` pass with zero failures. A test
  that silently skips counts as a failure (§8).

## 2. Real-time path

The real-time path is `read()`, controller `update()`, `write()`, `SafetyKernel`, and
`ActuatorTransport::exchange()` (ADR-001).

- **No** allocation, logging, streams, filesystem access, ROS or Zenoh calls, parameter lookup,
  or unbounded wait. Logging belongs on a non-real-time thread; see
  `GroundTruthPublisher::run()`.
- Resolve everything by name at configure time and store indices. `exchange()` never searches
  the model; see `MujocoActuatorTransport::map_joints()`.
- Hand data between a real-time and a non-real-time thread only through a lock-free buffer:
  [`LatestValueBuffer`](../../src/humanoid_transport/include/humanoid_transport/latest_value_buffer.hpp).
  Its frame types must be trivially copyable; add a `static_assert`, as
  [ground_truth.hpp](../../src/humanoid_transport_mujoco/include/humanoid_transport_mujoco/ground_truth.hpp) does.
- `humanoid_transport` is the ROS-free hardware seam. It must never include or link ROS, even
  to pass something as convenient as an executor. ROS integration for a backend stays inside
  that backend; see
  [sim_node.hpp](../../src/humanoid_transport_mujoco/include/humanoid_transport_mujoco/sim_node.hpp).

## 3. Returning values

- **Return by value** (Core Guidelines F.20/F.21). A read that can find nothing returns
  `std::optional<T>`; a configure step that can fail with a reason returns
  `std::optional<std::string>` holding the error. Do not return `bool` and fill a reference.
  Examples: `LatestValueBuffer::read()`, `MujocoDisturbance::configure()`,
  `lowest_collision_z()`.
- **Exception: filling a caller-owned preallocated buffer.** `ActuatorTransport::exchange()`
  fills `FeedbackBatch & feedback` because the batch is a reused ~1.3 KB POD array and ADR-001
  requires preallocated cycle data. That is F.20's own stated exception. Returning it by value
  would add a copy per cycle, because it cannot be elided into an existing object.
- **In-out parameters are fine when the function transforms its argument** (F.17), e.g.
  `SafetyKernel::project(…, CommandBatch & command, …)`.
- Every out- or in-out parameter's contract must say what it holds **on failure**. An
  unspecified failure state once let a stale feedback batch be republished as current. Now
  `exchange()` documents it, and `answers_command()` checks the echoed sequence.

## 4. Configuration: ROS parameters, never environment variables

- **Do not read environment variables** in C++ or launch files for anything that configures
  behaviour. They bypass type checking and range validation, are invisible to
  `ros2 param`, and silently vanish under `colcon test`, which does not inherit the shell.
- Declare configuration as **ROS 2 parameters** with a `ParameterDescriptor` description. Mark
  them `read_only` if they are read once at configure, so `ros2 param set` fails instead of
  appearing to work. See `MujocoActuatorTransport::declare_parameters()`.
- Read all parameters **once, into a plain struct**, in one place, and pass values to the
  ROS-free steps that follow (`SimParameters`).
- Put stable settings in the node's section of the ros2_control parameter file
  ([hardware_sil.yaml](../../src/humanoid_bringup/config/hardware_sil.yaml)). Expose per-run
  knobs as launch arguments passed through `ParameterValue(value_type=float)`: YAML and CLI
  integers (`50`) are a type error for a double parameter.
- A node must be spun, or its parameter services never answer and `ros2 param get` hangs. A
  node created inside a plugin needs an executor of its own (`SimNode`).
- Tests get paths as compile definitions (`HUMANOID_TEST_MJCF_PATH`) and parameters as
  `--ros-args -p` overrides, never through the environment.

## 5. Model data and invariants: one source, derived or checked, never copied

- **Never copy a number out of a model into C++.** The model pipeline is the single source of
  truth: `model/source/robot_model.yaml` → `model/generators` → `model/generated/`. Generated
  files are never hand-edited; `cli check` rejects drift.
- If code needs a geometric quantity, **derive it from the loaded model at configure time**.
  The initial root height is computed from collision geometry
  ([mujoco_placement.hpp](../../src/humanoid_transport_mujoco/include/humanoid_transport_mujoco/mujoco_placement.hpp)).
  It once was a hand-typed 0.793 that would have gone stale the moment the legs changed.
- Do not identify model elements by conventional names like `"root"` when structure identifies
  them. The robot is the kinematic tree that holds the commanded joints; scene objects such as a
  ball are other trees.
- **Architectural invariants are compile-time constants defined once, and every runtime setting
  is checked against them rather than trusted.** The 200 Hz / 5 ms cycle (ADR-001-03) lives
  only in [timing.hpp](../../src/humanoid_transport/include/humanoid_transport/timing.hpp).
  `controller_manager`'s rate and each transport's reported cycle are verified against it at
  configure, and a mismatch is a configuration error.

## 6. Class and file layout

- Header order: `public:`, then `protected:`, then `private:`. Within `private:`: types, then
  member functions, then constants, then data, grouped under short `// --- Group ---` labels.
- Mark every override `override`; `-Wsuggest-override` makes this a build error.
- A `.cpp` defines functions in header order: special members, then overrides of the base
  interface, then private helpers, separated by `// ===` banners. This way a reader can tell
  which functions are required by inheritance without opening the header.
- File-local helpers go in an anonymous namespace, **declared** at the top and **defined** at
  the bottom of the file. A helper with one caller stays file-local. Promote a helper to a
  shared header only when a second package needs it (e.g. `transport::is_finite()`), never
  into a new package for one function.
- **Declaration order is a correctness property.** Members are destroyed in reverse order:
  - an object whose deleter lives in a loader is declared **after** the loader
    (`loader_` / `transport_` in `HumanoidActuatorSystem`);
  - a subscription, thread, or executor is declared **after** every member its callback
    touches, so it is torn down first (`ref_sub_` in `MitImpedanceController`,
    `ground_truth_` after `sim_node_`).
  Leave a comment wherever the order is load-bearing.
- Qualify MuJoCo C enumerators with their enum type (`mjtObj::mjOBJ_BODY`) instead of wrapping
  them. Include `<mujoco/mujoco.h>` only in `.cpp` files; headers forward-declare `mjModel_` and
  `mjData_`.

## 7. State and telemetry

- **Replace sets of boolean flags with state that cannot be inconsistent**: an `enum class`,
  or better, state derived from data you already have. The scheduled push needs no flags,
  because its time window decides (`MujocoDisturbance::apply()`).
- **A counter needs a reader.** Do not add telemetry nothing consumes. Delete it, or connect it
  to a consumer at once, even a log line at the end of a run.
- **Account where the information originates, once.** Everything derivable from a call belongs
  to its single caller, not to every implementation. `ExchangeStats` lives in
  `HumanoidActuatorSystem`, and transports report only what they alone observe
  (`HealthSnapshot`). The earlier design duplicated five counters per transport, and no one
  read any of them.
- Group telemetry into a trivially copyable struct with a `record()` method
  ([exchange_stats.hpp](../../src/humanoid_transport/include/humanoid_transport/exchange_stats.hpp)).
  Such structs are **not synchronised**: read them on the owning thread, or publish them
  through a `LatestValueBuffer`.

## 8. Tests and evidence

- Tests must run under `colcon test`, **not skip**. If a test needs a resource, the build
  supplies it.
- Put a rule you want tested in a pure function (`answers_command()`,
  `lowest_collision_z()`), so it can be tested without ros2_control or a running simulation.
  Use synthetic in-memory MJCF (MuJoCo VFS) to cover geometry cases the robot model lacks.
- Test the physics, not just the code path: include one check against an independent
  reference, such as the G1 height against Menagerie's value.
- **SIL timing is never timing evidence** (ADR-007-05). A deadline miss seen in simulation is
  a diagnostic, not a measurement.
- SIL launch tests need a Zenoh router: `ros2 run rmw_zenoh_cpp rmw_zenohd` (ADR-003).

## 9. Simulation versus fault injection

- A simulator transport must behave like the hardware at the `ActuatorTransport` seam, and
  nothing more (ADR-007-01).
- Scenario control, such as pushes and resets, is simulator-side, because only the simulator
  has bodies to push (`MujocoDisturbance`). Transport fault injection, such as corrupt,
  dropped, delayed, or replayed batches, is a `FaultInjectingTransport` decorator selected by
  configuration (ADR-007-08). Test flags inside production components are prohibited.
