// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_stm32/joint_wiring.hpp"

#include <format>
#include <numbers>

#include <algorithm>
#include <cmath>
#include <limits>

namespace humanoid::transport_stm32
{

namespace
{

// The largest magnitude each wire field carries, derived from the fixed-point scales rather than
// retyped: a scale change in wire_protocol.hpp moves the limit with it.
constexpr double kMaxVelocityRadS =
  static_cast<double>(std::numeric_limits<std::int16_t>::max()) /
  static_cast<double>(wire::kVelScale);
constexpr double kMaxTorqueNm =
  static_cast<double>(std::numeric_limits<std::int16_t>::max()) /
  static_cast<double>(wire::kTauScale);
constexpr double kMaxStiffness =
  static_cast<double>(std::numeric_limits<std::uint16_t>::max()) /
  static_cast<double>(wire::kKpScale);
constexpr double kMaxDamping =
  static_cast<double>(std::numeric_limits<std::uint16_t>::max()) /
  static_cast<double>(wire::kKdScale);

std::optional<std::string> exceeds(const char * what, double value, double limit);

}  // namespace

LayoutOrError make_layout(std::span<const JointWiring> joints)
{
  if (joints.empty()) {
    return std::string{"no joints are wired"};
  }
  if (joints.size() > wire::kMaxMotors) {
    return std::format(
      "{} joints are wired but the wire carries at most {}", joints.size(), wire::kMaxMotors);
  }

  WiringLayout layout;
  layout.joint_count = static_cast<std::uint8_t>(joints.size());

  // Which (chain, motor) slots are taken, to reject two joints on one motor.
  std::array<std::array<bool, wire::kMaxMotorsPerChain>, wire::kMaxChains> taken{};
  std::array<std::uint8_t, wire::kMaxChains> slots{};

  for (std::size_t i = 0; i < joints.size(); ++i) {
    const JointWiring & w = joints[i];
    if (w.chain >= wire::kMaxChains) {
      return std::format(
        "joint {}: chain {} is beyond the wire maximum of {} chains", i, w.chain,
        wire::kMaxChains);
    }
    if (w.motor >= wire::kMaxMotorsPerChain) {
      return std::format(
        "joint {}: motor {} is beyond the wire maximum of {} motors per chain", i, w.motor,
        wire::kMaxMotorsPerChain);
    }
    if (w.direction_sign != 1 && w.direction_sign != -1) {
      return std::format("joint {}: direction_sign must be 1 or -1, got {}", i, w.direction_sign);
    }
    if (!std::isfinite(w.zero_offset_rad)) {
      return std::format("joint {}: zero_offset_rad is not finite", i);
    }
    if (taken[w.chain][w.motor]) {
      return std::format(
        "joint {}: chain {} motor {} is already wired to another joint", i, w.chain, w.motor);
    }
    taken[w.chain][w.motor] = true;
    slots[w.chain] = std::max(slots[w.chain], static_cast<std::uint8_t>(w.motor + 1U));
    layout.joints[i] = w;
  }

  for (std::uint8_t chain = 0; chain < wire::kMaxChains; ++chain) {
    if (slots[chain] > 0) {
      layout.chains[layout.chain_count] = {chain, slots[chain]};
      ++layout.chain_count;
    }
  }
  return layout;
}

std::optional<std::string> check_envelope(
  const JointWiring & wiring, const transport::JointEnvelope & envelope)
{
  const double wire_a = wire_position(wiring, envelope.position_min_rad);
  const double wire_b = wire_position(wiring, envelope.position_max_rad);
  const double wire_lo = std::min(wire_a, wire_b);
  const double wire_hi = std::max(wire_a, wire_b);
  if (!(wire_lo >= -std::numbers::pi && wire_hi <= std::numbers::pi)) {
    return std::format(
      "position range [{:.4f}, {:.4f}] rad becomes [{:.4f}, {:.4f}] rad on the wire, outside "
      "the home-frame range of +-pi", envelope.position_min_rad, envelope.position_max_rad,
      wire_lo, wire_hi);
  }
  if (auto error = exceeds("velocity_max_rad_s", envelope.velocity_max_rad_s, kMaxVelocityRadS)) {
    return error;
  }
  if (auto error = exceeds("torque_peak_nm", envelope.torque_peak_nm, kMaxTorqueNm)) {
    return error;
  }
  if (auto error = exceeds("stiffness_max_nm_rad", envelope.stiffness_max_nm_rad, kMaxStiffness)) {
    return error;
  }
  return exceeds("damping_max_nm_s_rad", envelope.damping_max_nm_s_rad, kMaxDamping);
}

wire::MitSetpoint to_setpoint(
  const JointWiring & wiring, const transport::JointCommand & command) noexcept
{
  wire::MitSetpoint setpoint;
  setpoint.position_rad = static_cast<float>(wire_position(wiring, command.position_rad));
  setpoint.velocity_rad_s = static_cast<float>(signed_value(wiring, command.velocity_rad_s));
  setpoint.effort_nm = static_cast<float>(signed_value(wiring, command.effort_nm));
  setpoint.stiffness_nm_rad = static_cast<float>(command.stiffness_nm_rad);
  setpoint.damping_nm_s_rad = static_cast<float>(command.damping_nm_s_rad);
  return setpoint;
}

namespace
{

// A limit of exactly the wire range still fits; NaN fails the comparison and is reported.
std::optional<std::string> exceeds(const char * what, double value, double limit)
{
  if (std::isfinite(value) && std::abs(value) <= limit) {
    return std::nullopt;
  }
  return std::format("{} = {} does not fit the wire, which carries at most {}", what, value, limit);
}

}  // namespace

}  // namespace humanoid::transport_stm32
