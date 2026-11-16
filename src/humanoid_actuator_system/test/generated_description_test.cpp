// Copyright 2026 UTRA-RoboSoccer
// The contract with model/generators/emit.py, once per generated robot: whatever it writes, the
// component must load. These read the files humanoid_description installs, the ones the component
// loads at launch, so an emitter change this package does not accept fails here rather than there.

#include <gmock/gmock.h>

#include <cmath>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "generated_description.hpp"
#include "humanoid_actuator_system/drive_layout.hpp"
#include "humanoid_actuator_system/hardware_contract.hpp"
#include "humanoid_actuator_system/safety_manifest_parser.hpp"
#include "humanoid_test_support/generated_robots.hpp"
#include "humanoid_test_support/matchers.hpp"
#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/safety_manifest.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

using humanoid::actuator_system::drive_layout;
using humanoid::actuator_system::DriveLayout;
using humanoid::actuator_system::hardware_contract_violation;
using humanoid::actuator_system::parse_safety_manifest;
using humanoid::actuator_system::testing::generated_linkages;
using humanoid::actuator_system::testing::GeneratedLinkage;
using humanoid::actuator_system::testing::LinkageName;
using humanoid::actuator_system::testing::load;
using humanoid::actuator_system::testing::LoadedLinkage;
using humanoid::actuator_system::testing::parse_description;
using humanoid::actuator_system::testing::samples;
using humanoid::test_support::generated_robots;
using humanoid::test_support::GeneratedRobot;
using humanoid::test_support::HoldsValue;
using humanoid::test_support::RobotName;
using humanoid::transport::JointCommand;
using humanoid::transport::SafetyManifest;

class GeneratedDescription : public ::testing::TestWithParam<GeneratedRobot>
{
protected:
  void SetUp() override
  {
    auto parsed = parse_description(GetParam().urdf());
    ASSERT_TRUE(parsed) << GetParam().urdf();
    info_ = *parsed;
  }

  hardware_interface::HardwareInfo info_;
};

TEST_P(GeneratedDescription, HoldsTheHardwareContract)
{
  EXPECT_EQ(hardware_contract_violation(info_), std::nullopt);
}

TEST_P(GeneratedDescription, DrivesEveryCommandJointThroughOneMotor)
{
  const auto result = drive_layout(info_);
  ASSERT_THAT(result, HoldsValue());
  const auto & layout = std::get<DriveLayout>(result);
  ASSERT_EQ(layout.transport_joints.size(), layout.command_joints.size());
  ASSERT_EQ(layout.linkages.size(), layout.command_joints.size());
  std::size_t linkages = 0;
  for (std::size_t i = 0; i < layout.command_joints.size(); ++i) {
    if (layout.linkages[i]) {
      ++linkages;
      EXPECT_NE(layout.transport_joints[i], layout.command_joints[i]);
    } else {
      // Driven directly: the motor turns the joint itself and is known by its name.
      EXPECT_EQ(layout.transport_joints[i], layout.command_joints[i]);
    }
  }
  // Every state-only joint is a linkage's input or coupler.
  EXPECT_EQ(layout.passive_joints.size(), 2U * linkages);
  EXPECT_EQ(linkages, info_.transmissions.size());
}

TEST_P(GeneratedDescription, KeysTheSafetyManifestsEnvelopesByMotor)
{
  // The component parses the manifest with the motors' names, so the generated manifest must key
  // its envelopes by the joint each motor turns.
  const auto layout = drive_layout(info_);
  ASSERT_THAT(layout, HoldsValue());
  EXPECT_THAT(
    parse_safety_manifest(
      YAML::LoadFile(GetParam().safety_manifest()),
      std::get<DriveLayout>(layout).transport_joints),
    HoldsValue());
}

INSTANTIATE_TEST_SUITE_P(
  EveryRobot, GeneratedDescription, ::testing::ValuesIn(generated_robots()), RobotName{});

class GeneratedLinkageDrive : public ::testing::TestWithParam<GeneratedLinkage> {};

TEST_P(GeneratedLinkageDrive, ReportsItsTransmissionsInputAndCouplerAndIsKnownByItsInput)
{
  const auto loaded = load(GetParam());
  ASSERT_THAT(loaded, HoldsValue());
  const auto & l = std::get<LoadedLinkage>(loaded);
  const auto role = [&l](const std::string & wanted) {
      for (const auto & joint : l.transmission.joints) {
        if (joint.role == wanted) {
          return joint.name;
        }
      }
      return std::string{};
    };
  EXPECT_EQ(l.output_joint(), role("output"));
  EXPECT_EQ(l.coupler_joint(), role("coupler"));
  EXPECT_EQ(l.input_joint(), l.transmission.actuators.front().name);
  // The transport and the safety manifest know the motor by the joint it turns.
  EXPECT_EQ(l.layout.transport_joints[l.output], l.input_joint());
}

TEST_P(GeneratedLinkageDrive, HoldsTheImageOfEveryCommandTheJointsLimitsAllowInItsEnvelope)
{
  // The component checks the output joint's command against the joint's limits, then the kernel
  // checks the motor's against the motor's envelope. The envelope must hold the whole image of the
  // joint's limits as the component computes it, or a command the joint allows, such as one at
  // its limit, stops the robot.
  const auto loaded = load(GetParam());
  ASSERT_THAT(loaded, HoldsValue());
  const auto & l = std::get<LoadedLinkage>(loaded);
  const auto manifest = parse_safety_manifest(
    YAML::LoadFile(GetParam().robot.safety_manifest()), l.layout.transport_joints);
  ASSERT_THAT(manifest, HoldsValue());
  const auto & envelope = std::get<SafetyManifest>(manifest).envelopes[l.output];
  const auto & limits = l.info.limits.at(l.output_joint());

  for (const double q : samples(l.lower, l.upper)) {
    for (const double sign : {-1.0, 1.0}) {
      SCOPED_TRACE("output " + std::to_string(q) + " rad, sign " + std::to_string(sign));
      const auto motor = l.linkage().to_input(
        JointCommand{
          .position_rad = q, .velocity_rad_s = sign * limits.max_velocity,
          .effort_nm = sign * limits.max_effort});
      ASSERT_TRUE(motor);
      EXPECT_GE(motor->position_rad, envelope.position_min_rad);
      EXPECT_LE(motor->position_rad, envelope.position_max_rad);
      EXPECT_LE(std::abs(motor->velocity_rad_s), envelope.velocity_max_rad_s);
      EXPECT_LE(std::abs(motor->effort_nm), envelope.torque_peak_nm);
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  EveryLinkage, GeneratedLinkageDrive, ::testing::ValuesIn(generated_linkages()),
  LinkageName{});

}  // namespace
