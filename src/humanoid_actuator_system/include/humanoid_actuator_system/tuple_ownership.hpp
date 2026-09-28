// Copyright 2026 UTRA-RoboSoccer
// Ownership of the five-field MIT tuple (ADR-001): which interface claims are legal, which joints
// a mode switch hands over, and whose command write() may send as a result.
//
// Pure functions, so the rules can be tested without ros2_control. They never allocate, because
// ros2_control calls the mode-switch hooks from its real-time thread as well as from the service
// thread (ControllerManager::perform_hardware_command_mode_change, ros2_control 4.48.1).
#ifndef HUMANOID_ACTUATOR_SYSTEM__TUPLE_OWNERSHIP_HPP_
#define HUMANOID_ACTUATOR_SYSTEM__TUPLE_OWNERSHIP_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>  // NOLINT(build/include_order)
#include <string>
#include <string_view>
#include <vector>

#include "humanoid_transport/batch_types.hpp"

namespace humanoid::actuator_system
{

/// One bit per manifest joint, bit j for joint j.
using JointMask = std::uint32_t;
static_assert(transport::kMaxJoints <= 32U, "JointMask must hold one bit per joint");

[[nodiscard]] constexpr JointMask all_joints(std::size_t joint_count) noexcept
{
  return joint_count >= 32U ? ~JointMask{0U} : (JointMask{1U} << joint_count) - 1U;
}

/// The joints whose tuples one mode switch starts and stops.
struct TupleClaims
{
  JointMask start{0U};
  JointMask stop{0U};
};

/// The joints named in `interfaces`, or nullopt unless each named joint is named with all five
/// fields. A name that is not `<joint>/<field>` for one of `joint_names` is also nullopt.
///
/// The lists are the union over every controller in the switch, so two controllers that split
/// one joint's tuple between them in a single request cannot be told apart from one controller
/// claiming it whole. Across separate requests the controller manager's exclusive claims already
/// prevent that split.
[[nodiscard]] inline std::optional<JointMask> complete_tuples(
  const std::vector<std::string> & interfaces, std::span<const std::string> joint_names) noexcept
{
  std::array<std::uint8_t, transport::kMaxJoints> fields{};
  for (const std::string_view name : interfaces) {
    const auto slash = name.rfind('/');
    if (slash == std::string_view::npos) {
      return std::nullopt;
    }
    const auto suffix = name.substr(slash + 1U);
    const auto field = std::find_if(
      transport::kMitFields.begin(), transport::kMitFields.end(),
      [suffix](const auto & f) {return f.name == suffix;});
    const auto joint = std::find(joint_names.begin(), joint_names.end(), name.substr(0U, slash));
    if (field == transport::kMitFields.end() || joint == joint_names.end()) {
      return std::nullopt;
    }
    const auto j = static_cast<std::size_t>(joint - joint_names.begin());
    if (j >= fields.size()) {
      return std::nullopt;
    }
    fields[j] =
      static_cast<std::uint8_t>(fields[j] | (1U << (field - transport::kMitFields.begin())));
  }

  constexpr std::uint8_t kWholeTuple = (1U << transport::kMitFields.size()) - 1U;
  JointMask joints{0U};
  for (std::size_t j = 0U; j < std::min(joint_names.size(), fields.size()); ++j) {
    if (fields[j] == 0U) {
      continue;
    }
    if (fields[j] != kWholeTuple) {
      return std::nullopt;
    }
    joints |= JointMask{1U} << j;
  }
  return joints;
}

/// ADR-001: a switch may start or stop a joint's tuple only whole. nullopt rejects the switch.
[[nodiscard]] inline std::optional<TupleClaims> tuple_claims(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces,
  std::span<const std::string> joint_names) noexcept
{
  const auto start = complete_tuples(start_interfaces, joint_names);
  const auto stop = complete_tuples(stop_interfaces, joint_names);
  if (!start || !stop) {
    return std::nullopt;
  }
  return TupleClaims{*start, *stop};
}

/// The joints owned after `claims` takes effect. Stops apply before starts, so a switch that
/// hands a joint from one controller to another leaves it owned throughout.
[[nodiscard]] constexpr JointMask apply_claims(JointMask owned, TupleClaims claims) noexcept
{
  return (owned & ~claims.stop) | claims.start;
}

/// Whose command write() sends. One state rather than flags, so "protective" and "owned" cannot
/// disagree.
enum class CommandAuthority : std::uint8_t
{
  /// Not every joint's tuple is owned. write() sends the damping tuple: ADR-002 allows only zero
  /// stiffness, zero feed-forward, and damping before the controller handshake, and forbids
  /// holding a position indefinitely.
  kAwaitingClaim,
  /// Every tuple has just been claimed. On the real-time switch path the controller manager hands
  /// interfaces over after controllers have updated, so the new owner has not written a command
  /// yet. write() sends damping once more instead of whatever the storage last held.
  kClaimed,
  /// The owner's tuples go through the SafetyKernel to the transport.
  kController,
  /// Latched protective damping (ADR-002). Only on_activate() leaves it.
  kProtective,
};

/// One write() cycle's transition. Releasing the tuple while a controller commands it is
/// command loss (ADR-002 ACTIVE -> PROTECTIVE_DAMPING), and it latches.
[[nodiscard]] constexpr CommandAuthority next_authority(
  CommandAuthority current, bool all_owned) noexcept
{
  switch (current) {
    case CommandAuthority::kAwaitingClaim:
      return all_owned ? CommandAuthority::kClaimed : CommandAuthority::kAwaitingClaim;
    case CommandAuthority::kClaimed:
      return all_owned ? CommandAuthority::kController : CommandAuthority::kAwaitingClaim;
    case CommandAuthority::kController:
      return all_owned ? CommandAuthority::kController : CommandAuthority::kProtective;
    case CommandAuthority::kProtective:
      return CommandAuthority::kProtective;
  }
  return CommandAuthority::kProtective;
}

}  // namespace humanoid::actuator_system

#endif  // HUMANOID_ACTUATOR_SYSTEM__TUPLE_OWNERSHIP_HPP_
