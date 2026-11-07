// Copyright 2026 UTRA-RoboSoccer
// How each joint a controller commands reaches its motor, read from the <ros2_control> block that
// model/generators/emit.py writes.
//
// Controllers and their state interfaces work in joint space: the kinematic tree's joints. The
// transport and the SafetyKernel work in motor space: one entry per motor, named by the joint the
// motor turns. The two coincide unless a linkage sits between them. A planar four-bar linkage is
// a <transmission> whose plugin is kFourBarPlugin, with the commanded joint as role "output", the
// motor's joint as its <actuator> (role "input") and the rod's pin as role "coupler"; the motor's
// joint and the coupler are state-only <joint>s, which the component reports through the linkage.
// ros2_control's own transmission_interface never loads it: its transmissions are linear.
#ifndef HUMANOID_ACTUATOR_SYSTEM__DRIVE_LAYOUT_HPP_
#define HUMANOID_ACTUATOR_SYSTEM__DRIVE_LAYOUT_HPP_

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "humanoid_actuator_system/four_bar_linkage.hpp"

namespace humanoid::actuator_system
{

inline constexpr std::string_view kFourBarPlugin = "humanoid_actuator_system/FourBarLinkage";

/// The <transmission> <param>s of a four-bar, each required and no other accepted.
inline constexpr std::array<std::string_view, 10> kFourBarParams{
  "ground_length_m", "input_crank_length_m", "coupler_length_m", "output_crank_length_m",
  "input_zero_angle_rad", "output_zero_angle_rad", "coupler_zero_angle_rad",
  "input_axis_sign", "output_axis_sign", "coupler_axis_sign"};

/// A commanded joint driven through a linkage.
struct LinkageDrive
{
  FourBarLinkage linkage;
  /// Indices into DriveLayout::passive_joints of the linkage's input (the motor's joint) and
  /// coupler.
  std::size_t input_state;
  std::size_t coupler_state;
};

struct DriveLayout
{
  /// The joints controllers command, with every MIT command interface, in description order.
  std::vector<std::string> command_joints;
  /// The joints the component only reports, each a linkage's input or coupler.
  std::vector<std::string> passive_joints;
  /// Per command joint: the joint its motor turns, the name the transport and the safety manifest
  /// know the motor by.
  std::vector<std::string> transport_joints;
  /// Per command joint: its linkage, or nullopt where the motor turns the joint itself.
  std::vector<std::optional<LinkageDrive>> linkages;
};

/// The layout `info` describes, or why it is not one this component can run. `info` must already
/// satisfy hardware_contract_violation(). Every transmission is a four-bar whose output is a
/// command joint with position limits, whose input and coupler are state-only joints, and whose
/// linkage FourBarLinkage::create() accepts over the output's limits; every state-only joint
/// belongs to exactly one, and no joint to two.
[[nodiscard]] std::variant<DriveLayout, std::string> drive_layout(
  const hardware_interface::HardwareInfo & info);

}  // namespace humanoid::actuator_system

#endif  // HUMANOID_ACTUATOR_SYSTEM__DRIVE_LAYOUT_HPP_
