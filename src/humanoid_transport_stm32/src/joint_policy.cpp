// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_stm32/joint_policy.hpp"

namespace humanoid::transport_stm32
{

JointReading read_joint(
  const JointWiring & wiring, const std::optional<wire::TeleMotor> & motor,
  std::chrono::microseconds max_age) noexcept
{
  JointReading reading{};
  if (!motor) {
    return reading;  // zeros, not fresh, not commandable
  }

  auto & feedback = reading.feedback;
  feedback.position_rad = joint_position(wiring, wire::decode_i16(motor->pos, wire::kPosScale));
  feedback.velocity_rad_s = signed_value(wiring, wire::decode_i16(motor->vel, wire::kVelScale));
  feedback.effort_nm = signed_value(wiring, wire::decode_i16(motor->tau, wire::kTauScale));
  feedback.temperature_c = static_cast<float>(motor->temp_c);
  feedback.bus_voltage_v = 0.0F;  // the master does not report it (provides_bus_voltage is false)

  const auto state = static_cast<wire::Lifecycle>(motor->state);
  auto bits = static_cast<std::uint16_t>(motor->motor_fault & kFaultBitsDriveMask);
  if (state == wire::Lifecycle::kFault) {
    bits = static_cast<std::uint16_t>(bits | kFaultBitFirmwareFault);
  }
  bits = static_cast<std::uint16_t>(bits | (motor->cause << kFaultCauseShift));
  feedback.fault_bits = bits;

  feedback.fresh = reports_within(*motor, max_age);
  reading.commandable = is_commandable(state);
  return reading;
}

std::optional<wire::TeleMotor> find_motor(
  const wire::RobotTelemetry & telemetry, std::uint8_t chain, std::uint8_t motor) noexcept
{
  const std::size_t chain_count = telemetry.header.n_chains;
  for (std::size_t i = 0; i < chain_count && i < telemetry.chains.size(); ++i) {
    const wire::TeleChain & candidate = telemetry.chains[i];
    if (candidate.chain_id == chain) {
      if (motor < candidate.n_motors) {
        return candidate.motors[motor];
      }
      return std::nullopt;
    }
  }
  return std::nullopt;
}

std::optional<std::uint8_t> chain_motor_count(
  const wire::RobotTelemetry & telemetry, std::uint8_t chain) noexcept
{
  const std::size_t chain_count = telemetry.header.n_chains;
  for (std::size_t i = 0; i < chain_count && i < telemetry.chains.size(); ++i) {
    if (telemetry.chains[i].chain_id == chain) {
      return telemetry.chains[i].n_motors;
    }
  }
  return std::nullopt;
}

ArmingVerdict judge_clearing(
  const wire::RobotTelemetry & telemetry, const WiringLayout & layout,
  std::chrono::microseconds max_age, std::chrono::nanoseconds elapsed,
  std::chrono::nanoseconds fault_grace) noexcept
{
  ArmingVerdict verdict{ArmingVerdict::Status::kReached, 0};
  for (std::uint8_t joint = 0; joint < layout.joint_count; ++joint) {
    const JointWiring & wiring = layout.joints[joint];
    const auto motor = find_motor(telemetry, wiring.chain, wiring.motor);
    if (motor && motor->state == static_cast<std::uint8_t>(wire::Lifecycle::kFault) &&
      elapsed >= fault_grace)
    {
      return {ArmingVerdict::Status::kFailed, joint};
    }
    const bool cleared = motor &&
      motor->state == static_cast<std::uint8_t>(wire::Lifecycle::kIdle) &&
      reports_within(*motor, max_age);
    if (!cleared && verdict.status == ArmingVerdict::Status::kReached) {
      verdict = {ArmingVerdict::Status::kWaiting, joint};
    }
  }
  return verdict;
}

ArmingVerdict judge_arming(
  const wire::RobotTelemetry & telemetry, const WiringLayout & layout,
  std::chrono::microseconds max_age) noexcept
{
  ArmingVerdict verdict{ArmingVerdict::Status::kReached, 0};
  for (std::uint8_t joint = 0; joint < layout.joint_count; ++joint) {
    const JointWiring & wiring = layout.joints[joint];
    const auto motor = find_motor(telemetry, wiring.chain, wiring.motor);
    if (motor && motor->state == static_cast<std::uint8_t>(wire::Lifecycle::kFault)) {
      return {ArmingVerdict::Status::kFailed, joint};
    }
    const bool armed = motor &&
      motor->state == static_cast<std::uint8_t>(wire::Lifecycle::kHold) &&
      reports_within(*motor, max_age);
    if (!armed && verdict.status == ArmingVerdict::Status::kReached) {
      verdict = {ArmingVerdict::Status::kWaiting, joint};
    }
  }
  return verdict;
}

bool reports_within(const wire::TeleMotor & motor, std::chrono::microseconds max_age) noexcept
{
  const std::chrono::microseconds age{static_cast<std::int64_t>(motor.fb_age_ms) * 1000};
  return age <= max_age;
}

}  // namespace humanoid::transport_stm32
