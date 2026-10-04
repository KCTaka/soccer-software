// Copyright 2026 UTRA-RoboSoccer
// Per-joint impedance gains of the MIT controller, resolved and checked once at configure.
#ifndef HUMANOID_MIT_CONTROLLER__GAINS_HPP_
#define HUMANOID_MIT_CONTROLLER__GAINS_HPP_

#include <format>

#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace humanoid::control
{

/// The resolved gains, one per joint in the controller's joint order, or why they were rejected.
/// (std::expected is C++23.)
using GainsOrError = std::variant<std::vector<double>, std::string>;

/// Resolves the gain parameter `name` for `joint_count` joints. `per_joint` empty means every
/// joint takes `fallback`; otherwise it must hold exactly one value per joint. A policy is trained
/// against specific gains, so a missing or short list is a configuration error, never a silent
/// fill. Every gain, fallback included, must be finite and non-negative.
[[nodiscard]] inline GainsOrError resolve_gains(
  std::string_view name, const std::vector<double> & per_joint, double fallback,
  std::size_t joint_count)
{
  const auto valid = [](double gain) {return std::isfinite(gain) && gain >= 0.0;};
  if (per_joint.empty()) {
    if (!valid(fallback)) {
      return std::format("default_{} must be finite and non-negative, got {}", name, fallback);
    }
    return std::vector<double>(joint_count, fallback);
  }
  if (per_joint.size() != joint_count) {
    return std::format(
      "'{}' has {} values for {} joints; give one per joint or none", name, per_joint.size(),
      joint_count);
  }
  for (std::size_t i = 0; i < per_joint.size(); ++i) {
    if (!valid(per_joint[i])) {
      return std::format(
        "'{}'[{}] must be finite and non-negative, got {}", name, i, per_joint[i]);
    }
  }
  return per_joint;
}

}  // namespace humanoid::control

#endif  // HUMANOID_MIT_CONTROLLER__GAINS_HPP_
