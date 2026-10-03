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
| Commit | `218126a` (`protocol.h` is unchanged since `e30d756`) |
| `PROTO_VERSION` | 8 |
| Source of truth | `firmware/common/include/protocol.h` |

`218126a` matters beyond the wire layout. `0555751` makes the master stamp each slave's own
`chain_id` in telemetry. Before it, every slave reported chain 0, so this transport could not tell
two chains apart. The master on the robot reported chains 0 and 1 on 2026-10-03, so it carries that
fix. Nothing on the wire says which commit a master runs.

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
| `activate()`, step 1 | IDLE with fault reset, until every motor reports IDLE with fresh feedback | Activation is the operator's acknowledgement (ADR-002). The reset clears a motor latched by the last session and a master latched in `HOST_LOST`. The slave polls an IDLE motor every tick but never polls a FAULT one, so fresh feedback in IDLE is what proves its reported position is current again. |
| `activate()`, step 2 | HOLD **without** fault reset, until every motor reports HOLD with fresh feedback | HOLD from IDLE arms at the captured position, which step 1 made current, so arming does not step. Fresh feedback is required because a motor with no drive behind it reports HOLD for the slave's 100 ms CAN-timeout grace. |
| running | MIT with the full tuple, **never HOLD** | A motor that dropped to IDLE or FAULT stays there until the next activation. Auto-rearm is prohibited. |
| isolated joint | IDLE | `request_joint_disable()` and `request_all_disable()` latch until the next activation. |
| tuple the wire cannot carry | DAMPED on the slave's configured gains, and `kManifestMismatch` | Never clamped. `feedback` is left stale, so the caller counts the cycle as bad. |
| `deactivate()` | IDLE, until the master confirms it | |

Why two steps. In the slave firmware (`motor_runtime.c`), HOLD with a fault reset on a FAULT motor
arms it in the same request, at the position the slave last heard. The slave stopped polling the
motor when it faulted, so that position is stale. For a motor the slave never discovered it is 0.
The same request also skips the slave's discovery and wound-shaft checks, which it applies only to
a motor that is already IDLE. If the leg moved while faulted, or a drive was powered after its slave
booted, that request steps the joint. No request this transport sends combines HOLD with a fault
reset, and a test pins that.

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
| Motor faulted | `fault_bits` and the availability mask say so; it is not rearmed. The slave stops polling it, so its sample also goes stale |
| Drive not answering at activation | `activate()` times out in step 1, naming the joint and its feedback age |
| Motor its slave did not discover at boot | Reported as FAULT, cause NONE, feedback age 255 ms. Step 1 clears it to IDLE, and the slave then rejects HOLD. `activate()` times out saying to reset that slave with the drive powered, because the slave never rediscovers a motor |
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

### Bench, hardware in the loop (2026-10-03)

The full stack was run against the robot's master, both slaves, and ten free RobStride drives on a
bench (no load): a non-real-time ROS node publishing `~/joint_references`, `MitImpedanceController`,
`HumanoidActuatorSystem` with its SafetyKernel, this transport, the master, the slaves, and the
drives. The description, manifest and wiring were bench-only: ten joints named after the firmware's
`robot_legs` motors, chain 0 and 1, motors 0 to 4, sign +1, offset 0, envelope inside every
drive's span, kp 5, kd 0.2. `controller_manager` ran at 200 Hz with FIFO priority 50 on the Jetson.

- Configure and the two-step activation succeeded every time, in 120 to 125 ms with the drives
  IDLE.
- Two runs of 31 s and 21 s: 6230 and 4230 exchanges, 0 failed, 0 deadline misses, worst exchange
  4.73 ms and 2.87 ms. `/joint_states` came back at 200.0 Hz. Four later runs of
  `stm32_bench.launch.py` had 2 isolated deadline misses in about 18 500 exchanges (worst 5.08 ms),
  each one bad cycle, far from the three in a row that stop the robot.
- Every joint followed the references: from rest positions up to 1.67 rad away to within 0.05 rad
  of 0 in a 3 s ease, then +-0.3 rad swings reaching +-0.24 to 0.29 rad, RMS error 0.025 to 0.042
  rad. With kp 5 the error is what drive friction leaves against a soft spring; it says nothing
  about latency.
- One startup in 23 failed: the first `read()` cycles reported three consecutive bad cycles about
  25 ms after activation, and the component was dropped. The cause was not recorded (see item 2
  below); the component now logs it.
- With joints resting outside the envelope (moved by hand while idle), every start failed at the
  controller's first cycle on the position envelope. The controller holds the measured position
  until a reference arrives, and the SafetyKernel checks that position even at zero stiffness. On
  the robot a joint cannot rest outside an envelope taken from its mechanical limits; a tighter
  envelope would stop the robot from starting.

## Bench

`stm32_bench.launch.py` runs the whole pipeline against free drives: the bench description,
manifest and wiring in `humanoid_bringup/config/bench/` (ten joints named after the firmware's
`robot_legs` motors), and `bench_player.py`, a non-real-time node that eases every joint from where
it rests to 0, swings it +-0.3 rad, and returns to 0. **The drives move. Run it only with nothing
attached to them.** Check where they rest with the probe first: a joint outside the bench envelope
(+-1.8 rad) stops the run at the controller's first cycle.

The dev container does not pass the serial device through, so run it from the container image with
the device, host networking, and real-time scheduling, plus a Zenoh router:

```bash
docker run --rm -it --entrypoint bash --device=/dev/ttyACM0 --net=host --ipc=host --cap-add=SYS_NICE --ulimit rtprio=99:99 -v "$PWD":/ws -w /ws <dev-container-image>
```

and inside it:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd &
ros2 launch humanoid_bringup stm32_bench.launch.py serial_device:=/dev/ttyACM0
```

Record with `ros2 bag record /joint_states /humanoid_mit_controller/joint_references`, stopped with
Ctrl-C so the bag is closed. Ctrl-C on the launch deactivates the hardware, which idles every drive
and logs the run's exchange stats.

## First contact with hardware

`stm32_link_probe` is read-only. It sends nothing, so it is safe against a powered robot in any
state. The master ignores the DTR change that opening the port causes (`CDC_SET_CONTROL_LINE_STATE`
is a no-op in `usbd_cdc_if.c`):

```bash
ros2 run humanoid_transport_stm32 stm32_link_probe /dev/robosoccer-master
```

It reports the master's version and rates, the telemetry rate over about a second on the master's
own clock, link error counters, and every motor's state, cause, drive faults, telemetry flags,
report age and measured values (in wire units). The rate is not measured on the host: frames queued
before the port opened arrive in a burst, and the first frame after opening can be minutes old,
left in the device's USB buffer by the previous reader.

The dev container does not pass the serial device through. To run the probe from its image:

```bash
docker run --rm --entrypoint bash --device=/dev/ttyACM0 -v "$PWD":/ws -w /ws <dev-container-image> -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run humanoid_transport_stm32 stm32_link_probe /dev/ttyACM0'
```

Observed on 2026-10-03 against the robot's master (read-only):

- `PROTO_VERSION` 8, `READY`, polling and reporting at 200 Hz. 1007 consecutive cycles were 5.000 ms
  apart on the master's clock, with a 20 Hz status frame. `host_cmd_hz` reads 50. Nothing in the
  master uses it.
- Chains 0 and 1, five motors each, no SPI CRC or CAN transmit errors.
- First, every motor read FAULT with cause NONE, feedback age 255 ms and all values zero:
  discovery had failed at slave boot. After the slaves were reset with the drives powered, all ten
  read IDLE with feedback 0-1 ms old, 24-26 C, and positions within +-0.043 rad.
- The probe reported a master reset that happened before it opened the port. The first frame it
  read was left from the earlier boot. `activate()` acknowledges any reset seen before it, so this
  does not affect the transport.

## Not verified, and known gaps

Needs the real master and a model whose joint names match the attached motors:

1. Command to telemetry over USB CDC works on the bench (above): activation, arming, and 10 000
   cycles without a failed exchange. The firmware notes that the first commands after a fresh
   connect may not land for ~1 s; activation never needed that long on the bench.
2. The gap between `activate()` returning and the first `write()` must stay under the master's
   60 ms host-death limit under `controller_manager`. If it does not, activation is followed by
   `HOST_LOST`. The one failed bench startup fits that: the last arming request can precede
   `activate()`'s return by up to 20 ms, and that start's first `controller_manager` cycle took
   10 ms. It is not confirmed. The component's error log now names the transport error, which
   would read "hardware fault" for `HOST_LOST`.
3. Pacing under load. On the idle Jetson `ExchangeStats` showed about one isolated deadline miss
   per 10 000 exchanges, nothing else running; the first
   `controller_manager` cycle sometimes overran (5 to 11 ms). The master's `cmd_on_time` /
   `cmd_late` / `cmd_missing` / `cmd_duplicate` counters are still unread, so a diagnostics reader
   is the missing piece for timing work.
4. The drive's own span is narrower than the wire's and depends on the model (RS00/RS02: Kp
   0..500, Kd 0..5; RS03/RS06: Kp 0..5000, Kd 0..100; the `robot_legs` configuration mixes them).
   The slave clamps to it silently. The master cannot report a motor's model, so this transport
   cannot check it, and the SafetyKernel does not check stiffness or damping against the
   envelope. The robot's `SafetyManifest` must keep stiffness and damping inside the drives'
   spans, and something must enforce it.
5. The identity interlock of ADR-002. No digest exists in the firmware, and the master's motor
   count is compile-time with no in-band check. `configure()` verifies chain and motor counts
   against the wiring, which catches a mismatched build but not a swapped motor model.

Firmware gaps that bound what this transport can promise, checked against `218126a`: neither MCU
enables a hardware watchdog (`HAL_IWDG_MODULE_ENABLED` and `HAL_WWDG_MODULE_ENABLED` are commented
out in both `stm32f4xx_hal_conf.h`), and a slave whose SPI exchanges stop holds for 50 ms, damps
for 300 ms, then idles, where ADR-002 wants a safe command within 20 ms. From the firmware's own
documentation: arming blocks a slave for ~40 ms per motor, CAN transmit is one-shot (about 0.2%
enable loss), and `max_tau` trips are scalar.

Also in `motor_runtime.c`, and the reason activation takes two steps. A slave discovers its motors
only at boot and never again. It sends nothing to a FAULT motor, so that motor's reported position
goes stale. HOLD with a fault reset on a FAULT motor arms it at that stale position, without the
discovery and wound checks. This transport avoids that request, but other hosts (the firmware's own
`host/` tools) may not. The fix belongs in the firmware: apply both checks to any HOLD that arms,
and capture the position after the drive's enable reply.
