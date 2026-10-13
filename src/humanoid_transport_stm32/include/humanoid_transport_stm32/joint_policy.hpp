// Copyright 2026 UTRA-RoboSoccer
// What the transport asks of each motor in each phase, and how it reads each motor's reply.
//
// These are pure functions of values, so the rules that matter for safety (a faulted motor is
// never silently re-armed, an isolated one is never commanded, an unrepresentable tuple is never
// sent) are tested without a serial port.
#ifndef HUMANOID_TRANSPORT_STM32__JOINT_POLICY_HPP_
#define HUMANOID_TRANSPORT_STM32__JOINT_POLICY_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport_stm32/joint_wiring.hpp"
#include "humanoid_transport_stm32/wire_protocol.hpp"

namespace humanoid::transport_stm32
{

/// What the transport is doing with the motors. activate() runs kClearing then kArming; exchange()
/// runs only in kRunning; deactivate() runs kStopping (ramp) then kReleasing (immediate) as a
/// fallback.
enum class Phase : std::uint8_t
{
  kClearing,
  kArming,
  kRunning,
  kStopping,
  kReleasing,
};

/// The request for one motor this cycle (protocol v10: a DriveReq plus, independently, whether a
/// VALID MIT tuple rides alongside it).
struct JointRequest
{
  wire::DriveReq request{wire::DriveReq::DRV_REQ_DISABLE};
  /// A VALID five-field tuple is carried. In v10 an ENABLED motor applies every valid tuple, so
  /// both arming (the captured hold tuple) and running (the live tuple) set this.
  bool send_tuple{false};
  /// Let the slave apply its configured gains instead of the wire kp/kd.
  bool use_config_gains{false};
};

/// The request for a motor (protocol v10; the drive request and the tuple are independent).
///
///   kClearing   RESET, no tuple. RESET clears what the previous session left latched, in a motor
///               (FAULT -> CLEARING -> DISABLED) or in a master stuck in HOST_LOST: activation is
///               the operator's acknowledgement (ADR-002). The slave polls a DISABLED motor every
///               tick, so once clearing reaches DISABLED the reported position is current again.
///   kArming     ENABLE while streaming the VALID captured-hold tuple every cycle (ENABLED is limp
///               until the first valid tuple, so the tuple must flow from the first ENABLE). The
///               tuple holds at the position captured when clearing reached DISABLED, with
///               USE_CONFIG_GAINS so the slave applies its own gains (the single source of hold
///               gains; see the transport). A motor latched in FAULT is not re-armed here; clearing
///               must reach DISABLED first.
///   kRunning    ENABLE with the live tuple. An isolated motor is asked for DISABLE. A tuple the
///               wire cannot carry fails the cycle in the caller (which counts a bad cycle), not
///               turned into a damping request here.
///   kStopping   STOP: ramp an ENABLED motor down (STOPPING -> DISABLED) for a gentle deactivation.
///   kReleasing  DISABLE (immediate): the deactivation fallback if the ramp does not land in time.
[[nodiscard]] constexpr JointRequest plan_request(
  Phase phase, bool isolated, bool /*tuple_representable*/) noexcept
{
  switch (phase) {
    case Phase::kClearing:
      return {wire::DriveReq::DRV_REQ_RESET, false, false};
    case Phase::kArming:
      // ENABLE streaming the captured hold tuple; USE_CONFIG_GAINS so the slave's configured gains
      // hold the joint. The caller fills the captured position into the tuple.
      return {wire::DriveReq::DRV_REQ_ENABLE, true, true};
    case Phase::kStopping:
      return {wire::DriveReq::DRV_REQ_STOP, false, false};
    case Phase::kReleasing:
      return {wire::DriveReq::DRV_REQ_DISABLE, false, false};
    case Phase::kRunning:
      break;
  }
  if (isolated) {
    return {wire::DriveReq::DRV_REQ_DISABLE, false, false};
  }
  return {wire::DriveReq::DRV_REQ_ENABLE, true, false};
}

/// True when a motor in `state` is armed and applies an MIT tuple. In v10 that is ENABLED (limp
/// until the first valid tuple, then executing it); ENABLING/STOPPING are transient, not
/// commandable.
[[nodiscard]] constexpr bool is_commandable(wire::DriveState state) noexcept
{
  return state == wire::DriveState::DRV_ENABLED;
}

// JointFeedback::fault_bits layout. The seam gives the field no meaning of its own; this is the
// meaning this transport gives it, in terms of the drive rather than the bus.
/// Bits 0-5: the drive's own fault bits.
inline constexpr std::uint16_t kFaultBitsDriveMask = 0x003FU;
/// Bit 6: the motor's firmware state is FAULT.
inline constexpr std::uint16_t kFaultBitFirmwareFault = 1U << 6U;
/// Bits 8-15: the latched FaultCause.
inline constexpr unsigned kFaultCauseShift = 8U;

/// One motor's reply as the seam's joint feedback.
struct JointReading
{
  transport::JointFeedback feedback;
  /// The motor is armed and can be commanded. The basis of the availability mask.
  bool commandable;
};

/// Reads a motor's telemetry into joint units. An absent motor (its chain was not in the frame)
/// reads as not fresh and not commandable. A sample is fresh while the drive last reported within
/// `max_age`; the wire saturates that age at 255 ms, so `max_age` must be below that.
[[nodiscard]] JointReading read_joint(
  const JointWiring & wiring, const std::optional<wire::TeleMotor> & motor,
  std::chrono::microseconds max_age) noexcept;

/// The drive last reported within `max_age`. The wire saturates the age at 255 ms, which also
/// means "never heard from", so `max_age` must be below that.
[[nodiscard]] bool reports_within(
  const wire::TeleMotor & motor, std::chrono::microseconds max_age) noexcept;

/// The telemetry for (chain, motor), or nothing if the master did not report that chain or the
/// chain reports fewer motors. Chains are matched by id, not position: the master omits a dead
/// slave's chain, so position in the frame does not identify it.
[[nodiscard]] std::optional<wire::TeleMotor> find_motor(
  const wire::RobotTelemetry & telemetry, std::uint8_t chain, std::uint8_t motor) noexcept;

/// How many motors the master reports on `chain`, or nothing if that chain is not reporting usable
/// data. v10 always emits every configured chain (Option A); one whose link is not ALIVE carries
/// zero motors, which reads the same as an omitted chain -- nothing. This separates "that slave is
/// not answering" from "that slave reports FEWER (but nonzero) motors than the wiring expects",
/// which is a firmware built from a different configuration.
[[nodiscard]] std::optional<std::uint8_t> chain_motor_count(
  const wire::RobotTelemetry & telemetry, std::uint8_t chain) noexcept;

/// How one step of activation is going, judged from one telemetry frame.
struct ArmingVerdict
{
  enum class Status : std::uint8_t
  {
    kWaiting,
    /// Every joint reached the step's target.
    kReached,
    kFailed,
  };
  Status status{Status::kWaiting};
  /// The joint holding the step up (kWaiting) or the one that failed it (kFailed).
  std::uint8_t joint{0};
};

/// The kClearing step. kReached once every joint reports IDLE with feedback no older than
/// `max_age`. Fresh feedback is what proves the slave is polling the motor again. Until then the
/// position HOLD would capture is the one the motor had when it faulted, or zero for a motor that
/// has never answered. kFailed for a joint still FAULT after `fault_grace`: the fault reset lands
/// a few cycles after it is sent, so a fault that outlives the grace means the requests are not
/// landing. Everything else is kWaiting, and the activation timeout bounds a drive that never
/// answers.
[[nodiscard]] ArmingVerdict judge_clearing(
  const wire::RobotTelemetry & telemetry, const WiringLayout & layout,
  std::chrono::microseconds max_age, std::chrono::nanoseconds elapsed,
  std::chrono::nanoseconds fault_grace) noexcept;

/// The kArming step, which starts only once clearing reached IDLE everywhere. kReached once every
/// joint reports HOLD with feedback no older than `max_age`. A motor with no drive behind it
/// reports HOLD for the slave's 100 ms CAN-timeout grace before it faults, so HOLD alone proves
/// nothing. kFailed for any FAULT, which from IDLE can only be this arming's own: a wound shaft,
/// or a drive that did not answer the enable. A slave that rejects HOLD leaves the motor IDLE, and
/// that is kWaiting until the timeout.
[[nodiscard]] ArmingVerdict judge_arming(
  const wire::RobotTelemetry & telemetry, const WiringLayout & layout,
  std::chrono::microseconds max_age) noexcept;

}  // namespace humanoid::transport_stm32

#endif  // HUMANOID_TRANSPORT_STM32__JOINT_POLICY_HPP_
