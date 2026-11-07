// Copyright 2026 UTRA-RoboSoccer

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "humanoid_actuator_system/drive_layout.hpp"
#include "humanoid_actuator_system/hardware_contract.hpp"
#include "humanoid_actuator_system/safety_manifest_parser.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

using humanoid::actuator_system::drive_layout;
using humanoid::actuator_system::DriveLayout;
using humanoid::actuator_system::hardware_contract_violation;

hardware_interface::HardwareInfo parse(const char * path)
{
  std::ifstream file(path);
  std::stringstream urdf;
  urdf << file.rdbuf();
  const auto components = hardware_interface::parse_control_resources_from_urdf(urdf.str());
  EXPECT_EQ(components.size(), 1U);
  return components.front();
}

std::string error_of(const hardware_interface::HardwareInfo & info)
{
  const auto layout = drive_layout(info);
  const auto * error = std::get_if<std::string>(&layout);
  return error ? *error : std::string{};
}

TEST(DriveLayout, ReadsTheGeneratedLegsLinkage)
{
  // The contract with model/generators/emit.py for a robot with a linkage.
  const auto info = parse(HUMANOID_TEST_LEG_URDF_PATH);
  ASSERT_EQ(hardware_contract_violation(info).value_or(""), "");
  const auto result = drive_layout(info);
  ASSERT_EQ(error_of(info), "");
  const auto & layout = std::get<DriveLayout>(result);
  EXPECT_EQ(layout.command_joints, std::vector<std::string>{"left_ankle_pitch_joint"});
  EXPECT_EQ(
    layout.passive_joints,
    (std::vector<std::string>{"left_ankle_motor_joint", "left_ankle_rod_joint"}));
  EXPECT_EQ(layout.transport_joints, std::vector<std::string>{"left_ankle_motor_joint"});
  ASSERT_TRUE(layout.linkages.front());
  EXPECT_EQ(layout.linkages.front()->input_state, 0U);
  EXPECT_EQ(layout.linkages.front()->coupler_state, 1U);
}

TEST(DriveLayout, NamesTheSafetyManifestsEnvelopesByMotor)
{
  // The component parses the manifest with the motors' names, so the generated manifest must key
  // its envelopes by the joint each motor turns.
  const auto layout = std::get<DriveLayout>(drive_layout(parse(HUMANOID_TEST_LEG_URDF_PATH)));
  const auto manifest = humanoid::actuator_system::parse_safety_manifest(
    YAML::LoadFile(HUMANOID_TEST_LEG_SAFETY_MANIFEST_PATH), layout.transport_joints);
  const auto * error = std::get_if<std::string>(&manifest);
  EXPECT_EQ(error ? *error : std::string{}, "");
}

TEST(DriveLayout, DrivesEveryJointDirectlyWithoutTransmissions)
{
  const auto info = parse(HUMANOID_TEST_ROBOT_URDF_PATH);
  const auto layout = std::get<DriveLayout>(drive_layout(info));
  EXPECT_TRUE(layout.passive_joints.empty());
  EXPECT_EQ(layout.transport_joints, layout.command_joints);
  EXPECT_EQ(layout.command_joints.size(), info.joints.size());
  for (const auto & drive : layout.linkages) {
    EXPECT_FALSE(drive);
  }
}

TEST(DriveLayout, RejectsAnotherTransmissionPlugin)
{
  auto info = parse(HUMANOID_TEST_LEG_URDF_PATH);
  info.transmissions.front().type = "transmission_interface/SimpleTransmission";
  EXPECT_NE(error_of(info).find("not supported"), std::string::npos);
}

TEST(DriveLayout, RequiresTheRoles)
{
  auto output = parse(HUMANOID_TEST_LEG_URDF_PATH);
  output.transmissions.front().joints.front().role = "coupler";
  EXPECT_NE(error_of(output).find("role 'output'"), std::string::npos);

  auto input = parse(HUMANOID_TEST_LEG_URDF_PATH);
  input.transmissions.front().actuators.front().role = "output";
  EXPECT_NE(error_of(input).find("role 'input'"), std::string::npos);
}

TEST(DriveLayout, RequiresEveryParameterAndNoOther)
{
  auto missing = parse(HUMANOID_TEST_LEG_URDF_PATH);
  missing.transmissions.front().parameters.erase("coupler_length_m");
  EXPECT_NE(error_of(missing).find("'coupler_length_m'"), std::string::npos);

  auto unknown = parse(HUMANOID_TEST_LEG_URDF_PATH);
  unknown.transmissions.front().parameters["gear_ratio"] = "1.0";
  EXPECT_NE(error_of(unknown).find("'gear_ratio'"), std::string::npos);

  auto garbled = parse(HUMANOID_TEST_LEG_URDF_PATH);
  garbled.transmissions.front().parameters["ground_length_m"] = "0.12m";
  EXPECT_NE(error_of(garbled).find("not a number"), std::string::npos);
}

TEST(DriveLayout, RequiresTheOutputsPositionLimits)
{
  auto info = parse(HUMANOID_TEST_LEG_URDF_PATH);
  info.limits.erase("left_ankle_pitch_joint");
  EXPECT_NE(error_of(info).find("position limits"), std::string::npos);
}

TEST(DriveLayout, RejectsAStateOnlyJointNoLinkageOwns)
{
  auto info = parse(HUMANOID_TEST_LEG_URDF_PATH);
  info.transmissions.clear();
  EXPECT_NE(error_of(info).find("belongs to 0 transmissions"), std::string::npos);
}

TEST(DriveLayout, RejectsAnInputThatControllersCommand)
{
  auto info = parse(HUMANOID_TEST_LEG_URDF_PATH);
  info.transmissions.front().actuators.front().name = "left_ankle_pitch_joint";
  EXPECT_NE(error_of(info).find("state-only"), std::string::npos);
}

}  // namespace
