# Stm32SerialTransport

The production `ActuatorTransport` (ADR-001-04), in `src/humanoid_transport_stm32`. It speaks the
master-link protocol of [soccer-firmware](https://github.com/utra-robosoccer/soccer-firmware) over
USB CDC to the master STM32:

```
Jetson --USB CDC--> master STM32 --SPI, 200 Hz--> slave STM32 --CAN--> RobStride drives
```

The master, not the Jetson, schedules the 200 Hz cycle (ADR-001-03).

## Protocol pin

| | |
|---|---|
| Firmware tag | `fw-1.1.0` (commit `8daea59`) |
| `PROTO_VERSION` | 10 |
| Schema hash | `0xE25161C4` |
| Source of truth | the firmware's `firmware/common/protocol.yaml` |

`generated/FIRMWARE_PIN` records the pin. `generated/vendor.sh <soccer-firmware checkout>` copies
that tag's generated C++ header, golden frames, `protocol.yaml`, and the firmware's drive and
host-watchdog state machines into `generated/`. Never edit them by hand. `wire_pin_test` fails when
the header's schema hash differs from the pin or the golden frames do not round-trip, and
`configure()` refuses a master on another protocol version or schema hash.

To move to a new release, update `FIRMWARE_PIN`, check that tag out, run `vendor.sh`, and port the
transport to what changed.

## Configuration

Parameters live on the transport's own node, `stm32_serial`, and are read once at configure
(read-only). Defaults shown:

```yaml
stm32_serial:
  ros__parameters:
    serial_device: /dev/robosoccer-master  # required
    monitor_only: false        # true: connect and stream telemetry, never arm
    handshake_timeout_s: 2.0
    discovery_timeout_s: 3.0
    arming_timeout_s: 3.0
    release_timeout_s: 1.0
    fault_grace_s: 0.5
    joints:
      left_knee_joint:
        chain: 0               # slave STM32
        motor: 1               # index within that slave's CAN chain
        direction_sign: -1     # joint = sign * (wire - zero_offset_rad)
        zero_offset_rad: 0.5   # wire angle at the joint's zero pose
```

Select it with `transport_plugin: humanoid_transport_stm32/Stm32SerialTransport`.

The `joints` block is generated from the robot model as `robots/<robot>/config/stm32_wiring.yaml`
in `humanoid_description` (chain = index of the actuator's `bus_segment`, motor = its `slot`); never
write it by hand. Its fields have no default: the telemetry carries no CAN id, and a plausible guess
goes unnoticed until a joint runs the wrong way. Velocity and torque take the sign only; stiffness
and damping are never negated.

`configure()` fails unless:

- every `SafetyManifest` envelope fits the wire once mapped: position within ±π, velocity and torque
  within ±327.67, Kp up to 6553.5, Kd up to 655.35;
- the master answers within `handshake_timeout_s` on the pinned version and schema hash, and sends
  one telemetry frame per poll cycle;
- within `discovery_timeout_s`, every wired chain is `ALIVE` with at least the wired number of
  motors, every wired motor has left `BOOT`/`DISCOVERING` and is not `ABSENT`, and no slave fails
  the version gate (`PROTO_MISMATCH`).

## Activation, running, deactivation

| Phase | Request per motor | Done when |
|---|---|---|
| `activate()`: clear | `RESET` | every motor `DISABLED` with fresh feedback; one still `FAULT` after `fault_grace_s` fails activation |
| `activate()`: capture | none | the first fresh frame with every motor `DISABLED`; its positions become the hold targets |
| `activate()`: arm | `ENABLE` with a valid tuple holding the captured position at the slave's configured gains (`USE_CONFIG_GAINS`) | every motor `ENABLED` with fresh feedback; any `FAULT` fails activation |
| running | `ENABLE` with the live tuple | |
| isolated joint | `DISABLE` until the next activation | |
| `deactivate()` | `STOP` (ramp), then `DISABLE` for any motor not `DISABLED` within `release_timeout_s` | every motor `DISABLED` |
| failed activation | `DISABLE` | |
| `monitor_only` | `DISABLE` every cycle; nothing arms | |

`arming_timeout_s` bounds the three activation steps together. Why in this order:

- `RESET` clears what the last session left latched, in a motor or in the master's `HOST_LOST`.
  Activation is the operator's acknowledgement (ADR-002). `ENABLE` is accepted only from
  `DISABLED`, and `RESET` never enables.
- The slave polls a `DISABLED` motor every cycle, so fresh feedback there means its position is
  current.
- `ENABLED` is limp until the first valid tuple, so the hold tuple goes with the first `ENABLE`. It
  holds the joint where it is, so arming does not step, and it is captured once, so a sagging joint
  cannot drag it.

Nothing re-arms a motor that drops out while running: recovery is `deactivate()` then `activate()`.

## One cycle

`exchange()` sends one command for the whole robot, then waits (on an eventfd, bounded by the
caller's deadline) for the master's next telemetry frame. That phase-locks the Jetson loop to the
master's cycle; two free-running 200 Hz clocks would beat, and the master would see late or
duplicate commands. `feedback.sequence = command.sequence` means "the freshest telemetry for this
cycle", not "this command was applied"; the firmware's `last_applied_seq` echo is not read.

| Condition | Result |
|---|---|
| No telemetry by the deadline | `kTimeout` |
| Port failed or unplugged | `kHardwareFault` |
| Master reset (cycle counter or clock ran backwards) | `kHardwareFault` until the next activation; a reset STM32 boots disarmed |
| `robot_state` is `HOST_LOST` or `PROTO_MISMATCH` | `kHardwareFault` |
| A commanded joint is `FAULT` or `ABSENT`, or not reported | `kHardwareFault`: the cycle fails rather than run some joints |
| A chain reports fewer motors than the wiring | `kManifestMismatch` |
| A tuple the wire cannot carry | sent saturated, and `kManifestMismatch`; unreachable behind the SafetyKernel |
| A drive silent for longer than `feedback_max_age_us` | that joint's feedback is not fresh |
| Corrupt, foreign-version or duplicated frames | dropped, counted in `HealthSnapshot` |

On any error `feedback` is left untouched, so its sequence stays behind the command's.
`humanoid_actuator_system` counts the cycle as bad, and three in a row are command loss (ADR-002).

`JointFeedback::fault_bits`: bits 0-5 the drive's fault bits, bit 6 set while the motor is `FAULT`,
bits 8-15 its latched `DriveFault` cause.

No thread sends commands on the real-time loop's behalf, and none may: the master's host-death
watchdog relies on it. While any motor is armed, `HOST_LOST_CYCLES` (5 master cycles, 25 ms)
without a fresh command makes the master force `STOP` on every motor, latched until a `RESET`.

## Evidence

`colcon test` runs the codec against the firmware's golden frames, the joint policy, the serial
link, the ADR-001 conformance suite (which checks that `exchange()` allocates nothing), and the
whole transport against `FakeMaster`: a pseudo-terminal master running the firmware's own drive
state machine and host watchdog, with simulated drives and fault injection.

This is **not hardware-in-the-loop evidence**. ADR-007 rejects a host-side loopback as the H1
emulator, and SIL timing is never timing evidence (ADR-007-05).

## First contact with hardware

Install soccer-firmware's `tools/udev/99-robosoccer-master.rules` on the host: it creates
`/dev/robosoccer-master` and keeps ModemManager off the port.

`stm32_link_probe` sends nothing, and the master ignores the DTR change that opening the port
causes, so it is safe against a powered robot in any state:

```bash
ros2 run humanoid_transport_stm32 stm32_link_probe /dev/robosoccer-master
```

It prints the master's protocol version, schema hash, uptime and rates; the telemetry rate over
about a second of the master's own clock; link error counts; and for every chain and motor the
link, drive state, fault cause, flags, feedback age, and position, velocity and torque in wire
units. In the dev container the master is `/host-dev/robosoccer-master` (see
[bench.md](bench.md)).

## Not verified, and known gaps

1. No bench run on `fw-1.1.0` is recorded. The bench runs of 2026-10 were on protocol v8.
2. The gap between `activate()` returning and the first `exchange()` must stay under the master's
   host-death limit of 25 ms, or activation ends in `HOST_LOST`. The last arming request can
   precede the return by up to one 20 ms frame wait, and on the 2026-10 bench `controller_manager`'s
   first cycle took 5 to 11 ms.
3. The master's `cmd_on_time` / `cmd_late` / `cmd_missing` / `cmd_duplicate` counters are not read,
   so pacing under load is unmeasured.
4. Identity (ADR-002): `configure()` checks the schema hash and each chain's motor count. That
   catches firmware built for another wiring, not a swapped motor model: the master does not report
   motor models, and the firmware holds no digest of its config.
5. A drive's gain span is narrower than the wire's and depends on its model (RS00/RS02: Kp 0..500,
   Kd 0..5). The SafetyKernel refuses gains above the caps in `model/motors.yaml`, so those caps
   must match the drives.

Firmware limits at `fw-1.1.0`:

- Neither MCU enables a hardware watchdog.
- A slave that stops getting commands holds its last tuple (velocity and torque zeroed) for 50 ms,
  then damps for 300 ms and ends in `FAULT` (`MASTER_LOST`). ADR-002 wants a safe command within
  20 ms.
- CAN transmit is one-shot. An enable is re-sent every cycle until the drive confirms it or it
  fails as `ENABLE_FAILED`, but the frame that finally disables a drive is sent once, unconfirmed.
- The `max_tau` torque trip fires on a single sample.
- The udev rule names any ST virtual COM port (`0483:5740`) `robosoccer-master`, so attach only one.
