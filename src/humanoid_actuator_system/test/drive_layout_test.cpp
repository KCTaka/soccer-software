// Copyright 2026 UTRA-RoboSoccer
// drive_layout()'s refusals, each on a generated description with a linkage that one edit breaks.
// That the generated descriptions themselves load is generated_description_test.cpp's.

#include <gmock/gmock.h>

#include <algorithm>
#include <string>
#include <variant>

#include "generated_description.hpp"
#include "humanoid_actuator_system/drive_layout.hpp"
#include "humanoid_test_support/matchers.hpp"

namespace
{

using humanoid::actuator_system::drive_layout;
using humanoid::actuator_system::testing::generated_linkages;
using humanoid::actuator_system::testing::parse_description;
using humanoid::test_support::HoldsError;
using humanoid::test_support::HoldsValue;
using ::testing::HasSubstr;

class DriveLayoutOfALinkage : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto linkages = generated_linkages();
    ASSERT_FALSE(linkages.empty()) << "no generated robot has a linkage to break";
    const auto parsed = parse_description(linkages.front().robot.urdf());
    ASSERT_TRUE(parsed);
    info_ = *parsed;
    ASSERT_THAT(drive_layout(info_), HoldsValue()) << "the unbroken description must load";
  }

  hardware_interface::TransmissionInfo & transmission() {return info_.transmissions.front();}

  /// The transmission's joint with `role`.
  std::string joint_with_role(const std::string & role)
  {
    const auto & joints = transmission().joints;
    const auto it = std::ranges::find(joints, role, &hardware_interface::JointInfo::role);
    return it == joints.end() ? std::string{} : it->name;
  }

  hardware_interface::HardwareInfo info_;
};

TEST_F(DriveLayoutOfALinkage, RejectsAnotherTransmissionPlugin)
{
  transmission().type = "transmission_interface/SimpleTransmission";
  EXPECT_THAT(drive_layout(info_), HoldsError(HasSubstr("not supported")));
}

TEST_F(DriveLayoutOfALinkage, RequiresTheOutputRole)
{
  transmission().joints.front().role = "coupler";
  EXPECT_THAT(drive_layout(info_), HoldsError(HasSubstr("role 'output'")));
}

TEST_F(DriveLayoutOfALinkage, RequiresTheInputRole)
{
  transmission().actuators.front().role = "output";
  EXPECT_THAT(drive_layout(info_), HoldsError(HasSubstr("role 'input'")));
}

TEST_F(DriveLayoutOfALinkage, RequiresEveryParameter)
{
  transmission().parameters.erase("coupler_length_m");
  EXPECT_THAT(drive_layout(info_), HoldsError(HasSubstr("'coupler_length_m'")));
}

TEST_F(DriveLayoutOfALinkage, AcceptsNoOtherParameter)
{
  transmission().parameters["gear_ratio"] = "1.0";
  EXPECT_THAT(drive_layout(info_), HoldsError(HasSubstr("'gear_ratio'")));
}

TEST_F(DriveLayoutOfALinkage, RejectsAParameterThatIsNotANumber)
{
  transmission().parameters["ground_length_m"] = "0.12m";
  EXPECT_THAT(drive_layout(info_), HoldsError(HasSubstr("not a number")));
}

TEST_F(DriveLayoutOfALinkage, RequiresTheOutputsPositionLimits)
{
  const auto output = joint_with_role("output");
  ASSERT_FALSE(output.empty());
  info_.limits.erase(output);
  EXPECT_THAT(drive_layout(info_), HoldsError(HasSubstr("position limits")));
}

TEST_F(DriveLayoutOfALinkage, RejectsAStateOnlyJointNoLinkageOwns)
{
  info_.transmissions.clear();
  EXPECT_THAT(drive_layout(info_), HoldsError(HasSubstr("belongs to 0 transmissions")));
}

TEST_F(DriveLayoutOfALinkage, RejectsAnInputThatControllersCommand)
{
  const auto output = joint_with_role("output");
  ASSERT_FALSE(output.empty());
  transmission().actuators.front().name = output;
  EXPECT_THAT(drive_layout(info_), HoldsError(HasSubstr("state-only")));
}

}  // namespace
