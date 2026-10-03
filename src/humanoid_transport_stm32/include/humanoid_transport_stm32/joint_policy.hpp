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

/// What the transport is doing with the motors. exchange() runs only in kRunning.
enum class Phase : std::uint8_t
{
  kArming,
  kRunning,
  kReleasing,
};

/// The request for one motor this cycle.
struct JointRequest
{
  wire::ModeRequest mode{wire::ModeRequest::kIdle};
  /// The five-field tuple is carried. False for HOLD, IDLE and DAMPED, which take no setpoint.
  bool send_tuple{false};
  bool fault_reset{false};
  /// Let the slave apply its configured gains. Used for DAMPED, so the damping a motor falls back
  /// to is the one its own configuration vouches for, not a number this transport invented.
  bool use_config_gains{false};
};

/// The request for a motor.
///
///   kArming     HOLD with fault reset. HOLD is the only request that arms a motor from IDLE, and
///               it captures the current position, so arming never steps. The fault reset is
///               what lets a motor latched by the previous session, or a master latched in
///               HOST_LOST, arm: activation is the operator's acknowledgement (ADR-002).
///   kRunning    MIT with the tuple. Never HOLD: a motor that dropped to IDLE or FAULT stays
///               there, because auto-rearm is prohibited. An isolated motor is asked for IDLE. A
///               tuple the wire cannot carry is replaced by DAMPED rather than clamped.
///   kReleasing  IDLE.
[[nodiscard]] constexpr JointRequest plan_request(
  Phase phase, bool isolated, bool tuple_representable) noexcept
{
  switch (phase) {
    case Phase::kArming:
      return {wire::ModeRequest::kHold, false, true, false};
    case Phase::kReleasing:
      return {wire::ModeRequest::kIdle, false, false, false};
    case Phase::kRunning:
      break;
  }
  if (isolated) {
    return {wire::ModeRequest::kIdle, false, false, false};
  }
  if (!tuple_representable) {
    return {wire::ModeRequest::kDamped, false, false, true};
  }
  return {wire::ModeRequest::kMit, true, false, false};
}

/// True when a motor in `state` is armed and takes an MIT request.
[[nodiscard]] constexpr bool is_commandable(wire::Lifecycle state) noexcept
{
  return state == wire::Lifecycle::kHold || state == wire::Lifecycle::kMit ||
         state == wire::Lifecycle::kDamped;
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

/// The telemetry for (chain, motor), or nothing if the master did not report that chain or the
/// chain reports fewer motors. Chains are matched by id, not position: the master omits a dead
/// slave's chain, so position in the frame does not identify it.
[[nodiscard]] std::optional<wire::TeleMotor> find_motor(
  const wire::RobotTelemetry & telemetry, std::uint8_t chain, std::uint8_t motor) noexcept;

/// How many motors the master reports on `chain`, or nothing if it did not report that chain. This
/// separates "that slave is not answering" from "that slave reports fewer motors than the wiring
/// expects", which is a firmware built from a different configuration.
[[nodiscard]] std::optional<std::uint8_t> chain_motor_count(
  const wire::RobotTelemetry & telemetry, std::uint8_t chain) noexcept;

/// How arming is going, judged from one telemetry frame.
struct ArmingVerdict
{
  enum class Status : std::uint8_t
  {
    kWaiting,
    kArmed,
    kFailed,
  };
  Status status{Status::kWaiting};
  /// The joint holding arming up (kWaiting) or the one that failed it (kFailed).
  std::uint8_t joint{0};
};

/// kArmed once every joint reports HOLD. kFailed for a wound shaft, which the firmware refuses to
/// arm and which no retry fixes, and for a faulted joint that stays faulted past `fault_grace`
/// after arming began (a latched fault from a previous session clears on the first reset, which the
/// firmware applies a few cycles after it is sent). Everything else is kWaiting: the activation
/// timeout, not this function, bounds a slave that is still booting.
[[nodiscard]] ArmingVerdict judge_arming(
  const wire::RobotTelemetry & telemetry, const WiringLayout & layout,
  std::chrono::nanoseconds elapsed, std::chrono::nanoseconds fault_grace) noexcept;

}  // namespace humanoid::transport_stm32

#endif  // HUMANOID_TRANSPORT_STM32__JOINT_POLICY_HPP_
