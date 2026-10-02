// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_actuator_system/hardware_contract.hpp"

#include <format>

#include <algorithm>
#include <string>
#include <vector>

#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/joint_manifest.hpp"

namespace humanoid::actuator_system
{

namespace
{
template<typename Fields>
std::optional<std::string> interfaces_violation(
  const std::string & joint, const char * kind,
  const std::vector<hardware_interface::InterfaceInfo> & declared, const Fields & required);
}  // namespace

std::optional<std::string> hardware_contract_violation(
  const hardware_interface::HardwareInfo & info)
{
  for (const auto & [name, _] : info.hardware_parameters) {
    if (std::ranges::find(kHardwareParams, name) == std::ranges::end(kHardwareParams)) {
      return std::format("unknown <hardware> parameter '{}'", name);
    }
  }
  for (const auto name : kHardwareParams) {
    const auto it = info.hardware_parameters.find(std::string{name});
    if (it == info.hardware_parameters.end() || it->second.empty()) {
      return std::format("required <hardware> parameter '{}' is not set", name);
    }
  }
  if (info.joints.empty() || info.joints.size() > transport::kMaxJoints) {
    return std::format(
      "{} joints declared; between 1 and {} are supported", info.joints.size(),
      transport::kMaxJoints);
  }
  for (const auto & joint : info.joints) {
    if (joint.name.empty() || joint.name.size() >= transport::kMaxNameLength) {
      return std::format(
        "joint name '{}' has {} characters; between 1 and {} are supported", joint.name,
        joint.name.size(), transport::kMaxNameLength - 1U);
    }
    if (auto v = interfaces_violation(
        joint.name, "command", joint.command_interfaces, transport::kMitFields))
    {
      return v;
    }
    if (auto v = interfaces_violation(
        joint.name, "state", joint.state_interfaces, transport::kJointStateFields))
    {
      return v;
    }
  }
  return std::nullopt;
}

namespace
{

template<typename Fields>
std::optional<std::string> interfaces_violation(
  const std::string & joint, const char * kind,
  const std::vector<hardware_interface::InterfaceInfo> & declared, const Fields & required)
{
  for (const auto & interface : declared) {
    const auto matches = [&interface](const auto & field) {return field.name == interface.name;};
    if (std::count_if(required.begin(), required.end(), matches) != 1 ||
      std::count_if(
        declared.begin(), declared.end(),
        [&interface](const auto & other) {return other.name == interface.name;}) != 1)
    {
      return std::format(
        "joint '{}' declares {} interface '{}', which is unknown or repeated", joint, kind,
        interface.name);
    }
  }
  if (declared.size() != required.size()) {
    return std::format(
      "joint '{}' declares {} {} interfaces; exactly {} are required", joint, declared.size(), kind,
      required.size());
  }
  return std::nullopt;
}

}  // namespace

}  // namespace humanoid::actuator_system
