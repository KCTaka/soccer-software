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
| Firmware branch | `akp/single_motor_rework` (not yet merged to firmware `main`) |
| Commit | `e30d756` |
| `PROTO_VERSION` | 8 |
| Source of truth | `firmware/common/include/protocol.h` |

`wire_protocol.hpp` mirrors that header byte for byte. The golden frames under
`test/golden/` are built by compiling the firmware's own header (`generate_golden.cpp`), so a layout
mistake fails a test. The master drops frames of another version and so does this transport, and
`configure()` says which version a mismatched master speaks. When the firmware changes a struct it
bumps `PROTO_VERSION`; bump `kProtocolVersion` and regenerate the golden frames in the same change.

The research docs describe an earlier firmware (2026-09-02): no per-cycle sequence echo, no tuple
beyond position and velocity, no host-death watchdog. All three changed. Where they disagree with
the firmware at the commit above, the firmware wins.

## Configuration

Parameters live on the transport's own node, `stm32_serial`, and are read once at configure
(read-only). Put them next to the hardware component's parameters:

```yaml
stm32_serial:
  ros__parameters:
    serial_device: /dev/robosoccer-master      # tools/udev/99-robosoccer-master.rules in soccer-firmware
    handshake_timeout_s: 2.0
    arming_timeout_s: 3.0
    release_timeout_s: 1.0
    fault_grace_s: 0.5
    joints:
      left_knee_joint:
        chain: 0                # firmware chain id = slave STM32
        motor: 1                # index within that slave's CAN chain
        direction_sign: -1      # joint = sign * (wire - zero_offset_rad)
        zero_offset_rad: 0.5    # wire angle at the joint's zero pose
```

and select it: `transport_plugin: humanoid_transport_stm32/Stm32SerialTransport`.

`chain`, `motor`, `direction_sign` and `zero_offset_rad` have **no default** on purpose. A value that
merely looks plausible goes unnoticed until the joint runs the wrong way. The mapping is a parameter
because the telemetry carries no CAN id, so nothing on the wire can say which joint a motor is, and
the generated model describes the G1 placeholder, not the real robot. When a real robot model
exists, `bus_segment` and `can_node_id` in the manifest are the better source (AGENTS.md: one
source, never copied) and this parameter block should be generated from it.

Angle convention: `joint = direction_sign * (wire - zero_offset_rad)`. Velocity and torque take the
sign only. Stiffness and damping are magnitudes and are never negated.

## How one cycle works

`exchange()` writes one command for the whole robot, then waits (on an eventfd, bounded by the
caller's deadline) for the master's next telemetry frame. That phase-locks the Jetson loop to the
master's cycle. Two free-running 200 Hz clocks would beat: some exchanges would see no new frame and
the next would see two, and the master would classify commands as late or duplicate.

A frame that does not arrive by the deadline is `kTimeout`. `humanoid_actuator_system` counts it as
a bad cycle, and three in a row are command loss (ADR-002).

`feedback` is untouched on every error, so its sequence stays behind the command's and a failed
exchange can never be mistaken for a current sample.

The firmware echoes `cmd_seq` as `last_applied_seq`, with a measured median of ~24 ms from command
to echo. That is several cycles, so `exchange()` cannot wait for its own command's echo.
`feedback.sequence = command.sequence` therefore means "the freshest telemetry the master produced
for this cycle", not "this command was applied". `last_applied_seq` is not consumed: nothing would
read it (AGENTS.md: no counter without a reader). A command-path liveness check is a follow-up.

## What the transport asks of each motor

| Phase | Request | Why |
|---|---|---|
| `activate()` | HOLD with fault reset, until every motor reports HOLD | HOLD is the only request that arms from IDLE, and it captures the current position, so arming never steps. Activation is the operator's acknowledgement (ADR-002); the reset lets a motor latched by the last session arm. |
| running | MIT with the full tuple, **never HOLD** | A motor that dropped to IDLE or FAULT stays there until the next activation. Auto-rearm is prohibited. |
| isolated joint | IDLE | `request_joint_disable()` and `request_all_disable()` latch until the next activation. |
| tuple the wire cannot carry | DAMPED on the slave's configured gains, and `kManifestMismatch` | Never clamped. `feedback` is left stale, so the caller counts the cycle as bad. |
| `deactivate()` | IDLE, until the master confirms it | |

`configure()` also rejects any `SafetyManifest` envelope the wire cannot carry (position outside
the home-frame range of ±π after mapping, velocity or torque over ±327.67, Kp over 6553.5, Kd over
655.35).

## Failures it detects

| Condition | Result |
|---|---|
| No telemetry by the deadline | `kTimeout` |
| Master reset (cycle counter or clock ran backwards) | `kHardwareFault` and no further commands, until the next activation. A reset STM32 boots disarmed. |
| Master reports `HOST_LOST` (its dead-man tripped) | `kHardwareFault`. Recovery is re-activation. |
| Port failed or unplugged | `kHardwareFault`, waiters woken at once |
| A chain stops answering | Its joints are not fresh and not available; the others carry on. All chains gone: `kIncompleteBatch`. |
| Chain reports fewer motors than the wiring | `kManifestMismatch` |
| Drive silent for longer than `feedback_max_age_us` | That joint is not fresh |
| Motor faulted | `fault_bits` and the availability mask say so; it is not rearmed |
| Corrupt, lost, or duplicated frames | Dropped and counted in `HealthSnapshot` |

`JointFeedback::fault_bits`: bits 0-5 the drive's fault bits (undervoltage, driver, overheat,
encoder, stall, uncalibrated); bit 6 the motor's firmware state is FAULT; bits 8-15 the latched
`FaultCause`.

Nothing here keeps a stalled real-time loop alive and nothing may: no thread sends commands on the
loop's behalf. The firmware's host-death watchdog (damped after 12 master cycles without a fresh
command while any motor is armed, then idle) depends on that.

## Evidence, and what it is not

`colcon test` runs the codec against firmware-built golden frames, the pure joint rules, the serial
link, and the whole transport against `FakeMaster`, a pseudo-terminal stand-in for the master that
reproduces the firmware's mode state machine, mailbox, and dead-man. A test confirms `exchange()`
allocates nothing on the calling thread.

This is **not hardware-in-the-loop evidence**. ADR-007 rejects a host-side loopback as the H1
emulator: it leaves the USB stack, `cdc_acm`, and real timing untested. SIL timing is never timing
evidence (ADR-007-05).

## First contact with hardware

`stm32_link_probe` is read-only. It sends nothing, so it is safe against a powered robot in any
state:

```bash
ros2 run humanoid_transport_stm32 stm32_link_probe /dev/robosoccer-master
```

It reports the master's version and rates, the observed telemetry rate, link error counters, and
every motor's state, cause, drive faults, report age and measured values (in wire units).

## Not verified, and known gaps

Needs the real master and a model whose joint names match the attached motors:

1. Real cmd-to-telemetry behaviour over USB CDC. The firmware notes the first commands after a
   fresh connect may not land for ~1 s; `arming_timeout_s` tolerates it, but confirm.
2. The gap between `activate()` returning and the first `write()` must stay under the master's
   60 ms host-death limit under `controller_manager`. If it does not, activation is followed by
   `HOST_LOST`.
3. Pacing under load: `exchange()` duration and deadline misses (`ExchangeStats`), and the master's
   `cmd_on_time` / `cmd_late` / `cmd_missing` / `cmd_duplicate` counters. This transport does not
   read those four counters, so a diagnostics reader is the missing piece for timing work.
4. The drive's own span is narrower than the wire's (RS00/RS02: Kp 0..500, Kd 0..5) and the slave
   clamps to it silently. The master cannot report a motor's model, so this transport cannot check
   it. The robot's `SafetyManifest` must keep stiffness and damping inside the drives' spans.
5. The identity interlock of ADR-002. No digest exists in the firmware, and the master's motor
   count is compile-time with no in-band check. `configure()` verifies chain and motor counts
   against the wiring, which catches a mismatched build but not a swapped motor model.

Firmware gaps that bound what this transport can promise, checked against `e30d756`: neither MCU
enables a hardware watchdog (`HAL_IWDG_MODULE_ENABLED` and `HAL_WWDG_MODULE_ENABLED` are commented
out in both `stm32f4xx_hal_conf.h`), and a slave whose SPI exchanges stop holds for 50 ms, damps
for 300 ms, then idles, where ADR-002 wants a safe command within 20 ms. From the firmware's own
documentation: arming blocks a slave for ~40 ms per motor, CAN transmit is one-shot (about 0.2%
enable loss), and `max_tau` trips are scalar.
