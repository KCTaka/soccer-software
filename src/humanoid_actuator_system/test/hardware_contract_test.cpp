// Copyright 2026 UTRA-RoboSoccer

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "humanoid_actuator_system/hardware_contract.hpp"
#include "humanoid_transport/batch_types.hpp"

namespace
{

using humanoid::actuator_system::hardware_contract_violation;
using humanoid::actuator_system::kSafetyManifestPackageParam;
using humanoid::actuator_system::kSafetyManifestPathParam;

hardware_interface::HardwareInfo valid_info()
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters[std::string{kSafetyManifestPathParam}] = "config/safety_manifest.yaml";
  info.hardware_parameters[std::string{kSafetyManifestPackageParam}] = "humanoid_bringup";
  hardware_interface::ComponentInfo joint;
  joint.name = "left_knee_joint";
  for (const auto & field : humanoid::transport::kMitFields) {
    hardware_interface::InterfaceInfo interface;
    interface.name = std::string{field.name};
    joint.command_interfaces.push_back(interface);
  }
  for (const auto & field : humanoid::transport::kJointStateFields) {
    hardware_interface::InterfaceInfo interface;
    interface.name = std::string{field.name};
    joint.state_interfaces.push_back(interface);
  }
  info.joints.push_back(joint);
  return info;
}

TEST(HardwareContract, HoldsForTheGeneratedDescription)
{
  // The contract with model/generators/emit.py: the description it writes must load.
  std::ifstream file(HUMANOID_TEST_ROBOT_URDF_PATH);
  ASSERT_TRUE(file) << HUMANOID_TEST_ROBOT_URDF_PATH;
  std::stringstream urdf;
  urdf << file.rdbuf();
  const auto components = hardware_interface::parse_control_resources_from_urdf(urdf.str());
  ASSERT_EQ(components.size(), 1U);
  EXPECT_EQ(hardware_contract_violation(components.front()).value_or(""), "");
}

TEST(HardwareContract, HoldsForAValidDescription)
{
  EXPECT_EQ(hardware_contract_violation(valid_info()).value_or(""), "");
}

TEST(HardwareContract, RejectsAnUnknownParameter)
{
  // The transport is a deployment choice, a node parameter, not a fact about the robot.
  auto info = valid_info();
  info.hardware_parameters["transport_plugin"] =
    "humanoid_transport_mujoco/MujocoActuatorTransport";
  EXPECT_NE(
    hardware_contract_violation(info).value_or("").find("'transport_plugin'"), std::string::npos);
}

TEST(HardwareContract, RequiresEveryParameter)
{
  auto missing = valid_info();
  missing.hardware_parameters.erase(std::string{kSafetyManifestPackageParam});
  EXPECT_TRUE(hardware_contract_violation(missing));

  auto empty = valid_info();
  empty.hardware_parameters[std::string{kSafetyManifestPathParam}] = "";
  EXPECT_TRUE(hardware_contract_violation(empty));
}

TEST(HardwareContract, RequiresEveryMitCommandInterface)
{
  auto info = valid_info();
  info.joints.front().command_interfaces.pop_back();
  EXPECT_TRUE(hardware_contract_violation(info));
}

TEST(HardwareContract, RejectsUnknownOrRepeatedInterfaces)
{
  auto unknown = valid_info();
  unknown.joints.front().command_interfaces.back().name = "torque";
  EXPECT_TRUE(hardware_contract_violation(unknown));

  auto repeated = valid_info();
  repeated.joints.front().state_interfaces.back().name = "position";
  EXPECT_TRUE(hardware_contract_violation(repeated));
}

}  // namespace
