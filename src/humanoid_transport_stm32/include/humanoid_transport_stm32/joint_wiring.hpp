// Copyright 2026 UTRA-RoboSoccer
// Where each manifest joint lives on the firmware's chain/motor hierarchy, and how a joint angle
// relates to the angle on the wire.
//
// The firmware names a motor by (chain, motor): the chain is a slave STM32 and the motor is its
// position in that slave's CAN chain. The manifest names it by joint. The telemetry carries no
// CAN id, so nothing on the wire can recover the pairing and it has to be configured. It is
// validated against what the master reports at activation (see joint_policy.hpp).
#ifndef HUMANOID_TRANSPORT_STM32__JOINT_WIRING_HPP_
#define HUMANOID_TRANSPORT_STM32__JOINT_WIRING_HPP_

#include <span>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>

#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/safety_manifest.hpp"
#include "humanoid_transport_stm32/wire_protocol.hpp"

namespace humanoid::transport_stm32
{

/// One joint's address on the wire and its angle convention.
///
///   joint angle = direction_sign * (wire angle - zero_offset_rad)
///   wire angle  = direction_sign * joint angle + zero_offset_rad      (the sign is +-1)
///
/// Velocity and torque take the sign only. Stiffness and damping are magnitudes and are never
/// negated, so a mirrored joint keeps a positive gain.
struct JointWiring
{
  std::uint8_t chain{0};
  std::uint8_t motor{0};
  std::int8_t direction_sign{1};
  double zero_offset_rad{0.0};
};

/// The command frame this wiring needs for one chain.
struct ChainLayout
{
  std::uint8_t chain_id{0};
  /// Slots the chain's command must carry: highest mapped motor index + 1. A slot between mapped
  /// motors is sent without kCmdFlagValid, which the slave reads as "no request for this motor".
  std::uint8_t motor_slots{0};
};

struct WiringLayout
{
  std::array<JointWiring, wire::kMaxMotors> joints{};
  std::uint8_t joint_count{0};
  /// Ascending chain_id; the first chain_count entries are valid.
  std::array<ChainLayout, wire::kMaxChains> chains{};
  std::uint8_t chain_count{0};
};

/// The layout, or why the wiring was rejected. (std::expected is C++23.)
using LayoutOrError = std::variant<WiringLayout, std::string>;

/// Validates `joints` (manifest order) and derives the per-chain command layout. Rejects an empty
/// or oversized joint list, a chain or motor index beyond the wire maximums, two joints on one
/// motor, a direction_sign other than +-1, and a non-finite offset. `names`, if given, parallels
/// `joints` and names the joint in each message; otherwise a message gives its position.
[[nodiscard]] LayoutOrError make_layout(
  std::span<const JointWiring> joints, std::span<const std::string> names = {});

/// Why `envelope` cannot be carried by the wire for this joint, or nothing if it can.
///
/// The SafetyKernel projects every command into the envelope, so a command the wire cannot carry
/// is a defect upstream. Catching it at configure turns a silent clamp into a refusal to start.
/// Checked after mapping into the wire frame: position in the home-frame range of +-pi, and
/// velocity, torque, stiffness and damping within the fixed-point ranges.
///
/// This is the wire only. The drive's own span is narrower for RS00/RS02 (Kp 0..500, Kd 0..5) and
/// the master cannot report which model a motor is, so that check belongs to the firmware.
[[nodiscard]] std::optional<std::string> check_envelope(
  const JointWiring & wiring, const transport::JointEnvelope & envelope);

// --- Joint <-> wire conversions -------------------------------------------------------------

[[nodiscard]] constexpr double joint_position(const JointWiring & w, double wire_rad) noexcept
{
  return static_cast<double>(w.direction_sign) * (wire_rad - w.zero_offset_rad);
}

[[nodiscard]] constexpr double wire_position(const JointWiring & w, double joint_rad) noexcept
{
  return static_cast<double>(w.direction_sign) * joint_rad + w.zero_offset_rad;
}

/// Velocity and torque, in either direction: the sign is its own inverse.
[[nodiscard]] constexpr double signed_value(const JointWiring & w, double value) noexcept
{
  return static_cast<double>(w.direction_sign) * value;
}

/// The wire setpoint for a joint command. Narrowing to float is the wire's own precision.
[[nodiscard]] wire::MitSetpoint to_setpoint(
  const JointWiring & wiring, const transport::JointCommand & command) noexcept;

}  // namespace humanoid::transport_stm32

#endif  // HUMANOID_TRANSPORT_STM32__JOINT_WIRING_HPP_
