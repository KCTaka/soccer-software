// Copyright 2026 UTRA-RoboSoccer
// The helpers every test package relies on: a matcher that always matched, or a robot list that
// came back empty, would make the tests that use them pass vacuously.

#include <gmock/gmock.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <variant>
#include <vector>

#include "humanoid_test_support/generated_robots.hpp"
#include "humanoid_test_support/manifests.hpp"
#include "humanoid_test_support/matchers.hpp"

namespace
{

using humanoid::test_support::generated_robots;
using humanoid::test_support::HoldsError;
using humanoid::test_support::HoldsValue;
using humanoid::test_support::joint_manifest;
using humanoid::test_support::safety_manifest;
using ::testing::HasSubstr;
using ::testing::Not;

using Result = std::variant<int, std::string>;

TEST(Matchers, HoldsValueMatchesOnlyAValue)
{
  EXPECT_THAT(Result{7}, HoldsValue());
  EXPECT_THAT(Result{"no"}, Not(HoldsValue()));
}

TEST(Matchers, HoldsErrorMatchesOnlyAnErrorItsMatcherAccepts)
{
  EXPECT_THAT(Result{"joint 1 already wired"}, HoldsError(HasSubstr("already wired")));
  EXPECT_THAT(Result{"joint 1 already wired"}, Not(HoldsError(HasSubstr("sign"))));
  EXPECT_THAT(Result{7}, Not(HoldsError(HasSubstr(""))));
  EXPECT_THAT(Result{"exact"}, HoldsError(std::string{"exact"}));
}

TEST(Manifests, NameTheJointsInOrderAndCountThem)
{
  const auto manifest = joint_manifest({"hip", "knee"});
  EXPECT_EQ(manifest.joint_count, 2U);
  EXPECT_STREQ(manifest.joints[0].name.data(), "hip");
  EXPECT_STREQ(manifest.joints[1].name.data(), "knee");

  humanoid::transport::JointEnvelope envelope;
  envelope.torque_peak_nm = 30.0;
  const auto safety = safety_manifest(2U, envelope);
  EXPECT_EQ(safety.joint_count, 2U);
  EXPECT_EQ(safety.envelopes[1].torque_peak_nm, 30.0);
  EXPECT_EQ(safety.envelopes[2].torque_peak_nm, 0.0);  // beyond the count, untouched
}

TEST(GeneratedRobots, ListsEveryInstalledRobotWithItsDescription)
{
  const auto robots = generated_robots();
  ASSERT_FALSE(robots.empty()) << "no robot under " << HUMANOID_TEST_ROBOTS_DIR;
  for (const auto & robot : robots) {
    EXPECT_TRUE(std::filesystem::exists(robot.urdf())) << robot.name;
    EXPECT_TRUE(std::filesystem::exists(robot.safety_manifest())) << robot.name;
  }
  EXPECT_TRUE(std::ranges::is_sorted(robots, {}, &humanoid::test_support::GeneratedRobot::name));
}

}  // namespace
