// Copyright 2026 UTRA-RoboSoccer
// What HumanoidActuatorSystem requires of the <ros2_control> block that model/generators/emit.py
// writes into the robot description.
//
// Only facts about the robot belong there. Choices that differ between SIL and hardware, such as
// the transport, are ROS parameters of the component's node instead, because one generated
// description serves both.
#ifndef HUMANOID_ACTUATOR_SYSTEM__HARDWARE_CONTRACT_HPP_
#define HUMANOID_ACTUATOR_SYSTEM__HARDWARE_CONTRACT_HPP_

#include <array>
#include <optional>
#include <string>
#include <string_view>

#include "hardware_interface/hardware_info.hpp"

namespace humanoid::actuator_system
{

/// <hardware><param> holding the safety manifest's path, absolute or relative to the share
/// directory of kSafetyManifestPackageParam. The manifest is generated with the description.
inline constexpr std::string_view kSafetyManifestPathParam = "safety_manifest_path";
inline constexpr std::string_view kSafetyManifestPackageParam = "safety_manifest_package";

/// Every <hardware><param> this component reads. Each is required, and no other is accepted, so a
/// misspelled or stale name fails at load instead of being ignored.
inline constexpr std::array<std::string_view, 2> kHardwareParams{
  kSafetyManifestPathParam, kSafetyManifestPackageParam};

/// Why `info` breaks the contract, or nullopt if it holds:
///   - it sets exactly kHardwareParams, none of them empty;
///   - every joint declares exactly the kMitFields command interfaces and the kJointStateFields
///     state interfaces. The resource manager filters mode switches by these declarations, so a
///     missing one would make every claim look partial.
[[nodiscard]] std::optional<std::string> hardware_contract_violation(
  const hardware_interface::HardwareInfo & info);

}  // namespace humanoid::actuator_system

#endif  // HUMANOID_ACTUATOR_SYSTEM__HARDWARE_CONTRACT_HPP_
