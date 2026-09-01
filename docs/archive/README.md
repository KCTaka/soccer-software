# Archive — historical documents

These are the **original working documents** of the project, retained for
provenance. They record how decisions were reached, including the investigations and
dead ends that the current documentation only summarises.

> **They are not maintained.** Where an archived document disagrees with
> [the current documentation](../README.md), the current documentation is correct.
> Read these for _why something was investigated_, not for _what the system does_.

---

## Contents

| Document                                                                                     | What it was                                                                  | Superseded by                                                                                              |
| -------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| [IMPLEMENTATION.md](IMPLEMENTATION.md)                                                       | Package-by-package implementation notes                                      | [01](../01-system-overview.md), [03](../03-ros-2-interfaces.md), and chapters 04–10                        |
| [bring_up_investigation_report.md](bring_up_investigation_report.md)                         | Detailed record of the Jetson + ZED bring-up, including every failed attempt | [11](../11-compute-platform.md), [13](../13-operations-runbook.md)                                         |
| [jetson_zed_workflow.md](jetson_zed_workflow.md)                                             | Operator workflow for the camera stack                                       | [12](../12-build-test-deploy.md), [13](../13-operations-runbook.md)                                        |
| [zed_jetson_integration.md](zed_jetson_integration.md)                                       | Integration design for the ZED on Jetson                                     | [11](../11-compute-platform.md)                                                                            |
| [ros2-diagram.md](ros2-diagram.md)                                                           | Node and topic graph sketches                                                | [03](../03-ros-2-interfaces.md)                                                                            |
| [architecture/new_architecture_blueprint.md](architecture/new_architecture_blueprint.md)     | The original layered-architecture proposal                                   | [01](../01-system-overview.md), [02](../02-design-decisions.md)                                            |
| [architecture/localization_strategy_report.md](architecture/localization_strategy_report.md) | Survey of localization approaches and the case for a particle filter         | [05](../05-localization.md), [D-13](../02-design-decisions.md#d-13)–[D-16](../02-design-decisions.md#d-16) |
| [architecture/soccer_hardware_rewrite.md](architecture/soccer_hardware_rewrite.md)           | Proposal for the `ros2_control` hardware interface                           | [08](../08-hardware-interface.md), [D-03](../02-design-decisions.md#d-03)                                  |
| [architecture/jetson_master_protocol.md](architecture/jetson_master_protocol.md)             | Host ↔ MCU wire-protocol design                                              | [09](../09-firmware-and-actuators.md)                                                                      |

Retained because the reasoning in them is the source material for
[02 — Design Decisions](../02-design-decisions.md). Deleting them would have
discarded the _why_ behind D-01…D-44.

---

## Known-outdated content

Read these documents with the following corrections in mind.

| Outdated statement                                                | Current reality                                                                                                                                                                        | Where                                                                                                                                           |
| ----------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| A `camera_bridge` node relays ZED topics onto the camera contract | Removed in commit `63c80be`. The ZED **component itself** is remapped inside the composable container — one fewer process, one fewer copy                                              | [D-09](../02-design-decisions.md#d-09) · affects `bring_up_investigation_report.md` §3 and §13, and the diagrams in `jetson_zed_workflow.md` §3 |
| The robot is called `minibot`                                     | Renamed to `soccerbot` in commit `42924dd`. Package names, the URDF and all topics changed                                                                                             | [08](../08-hardware-interface.md)                                                                                                               |
| The MCU runs a 1 kHz PD loop that tracks position commands        | Superseded. The Robostride actuator closes the impedance loop **onboard**; the MCU forwards MIT tuples and the Jetson streams setpoints at 100 Hz                                      | [D-24](../02-design-decisions.md#d-24)                                                                                                          |
| An off-field "coach" process assigns roles centrally              | **Rules-illegal.** RoboCup Humanoid prohibits external computation during play. Role assignment is fully decentralized, computed identically on every robot                            | [D-06](../02-design-decisions.md#d-06), [D-18](../02-design-decisions.md#d-18)                                                                  |
| Localization accuracy "in millimetres"                            | An over-claim. Realistic expectation is decimetre-level position and a few degrees of heading, dominated by projection error                                                           | [05 §6](../05-localization.md#5-accuracy-expectations)                                                                                          |
| Relative paths into `ros2_ws/`, `sim/`, `deploy/`                 | These documents moved down one directory when archived. Source paths quoted inside them are still correct **relative to the repository root**, but any `../` links are one level short | —                                                                                                                                               |

---

## Superseded proposals

Ideas that appear in the archive and were deliberately **not** adopted. The
reasoning is in [02 §Decisions that were reversed](../02-design-decisions.md).

| Proposal                                           | Why it was dropped                                                                                                         |
| -------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| Central coach / master robot                       | Rules-illegal ([D-06](../02-design-decisions.md#d-06))                                                                     |
| Visual SLAM as the field localizer                 | A soccer field is a known map with strong symmetry; SLAM solves the wrong problem ([D-14](../02-design-decisions.md#d-14)) |
| `camera_bridge` relay node                         | Unnecessary process and copy ([D-09](../02-design-decisions.md#d-09))                                                      |
| `l4t-jetpack` base image                           | Does not exist for L4T r39 ([D-33](../02-design-decisions.md#d-33))                                                        |
| Kernel buffer tuning as the fix for low frame rate | Measured 1.3 → 1.5 Hz. The SHM segment size was the actual fix ([D-38](../02-design-decisions.md#d-38))                    |
| `--runtime nvidia` for GPU access                  | Triggers a container-toolkit panic on Jetson ([D-36](../02-design-decisions.md#d-36))                                      |
