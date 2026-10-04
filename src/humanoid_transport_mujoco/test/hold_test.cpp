// Copyright 2026 UTRA-RoboSoccer
// The SIL start state: hold_until_commanded, and an initial pose of left_knee_joint = 0.5 rad
// (set by the test's main).

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "humanoid_transport_mujoco/mujoco_actuator_transport.hpp"

namespace
{

using humanoid::transport::CommandBatch;
using humanoid::transport::FeedbackBatch;

class HoldTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Only the joints the tests read need to exist; the transport finds them by name.
    const std::vector<std::string> names = {
      "left_hip_pitch_joint", "left_knee_joint", "left_ankle_pitch_joint"};
    manifest.joint_count = static_cast<std::uint8_t>(names.size());
    for (std::size_t i = 0; i < names.size(); ++i) {
      std::strncpy(
        manifest.joints[i].name.data(), names[i].c_str(),
        humanoid::transport::kMaxNameLength - 1);
    }
    safety.joint_count = manifest.joint_count;
    transport = std::make_unique<humanoid::transport_mujoco::MujocoActuatorTransport>();
    ASSERT_TRUE(transport->configure(manifest, safety));
    ASSERT_TRUE(transport->activate());
    command.joint_count = manifest.joint_count;
  }

  FeedbackBatch exchange()
  {
    ++command.sequence;
    FeedbackBatch feedback{};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    EXPECT_TRUE(transport->exchange(command, feedback, deadline).ok());
    return feedback;
  }

  humanoid::transport::JointManifest manifest{};
  humanoid::transport::SafetyManifest safety{};
  std::unique_ptr<humanoid::transport::ActuatorTransport> transport;
  CommandBatch command{};
};

TEST_F(HoldTest, RobotStandsStillWhileNoStiffnessIsCommanded)
{
  const auto first = exchange();
  for (int i = 0; i < 200; ++i) {  // a full second of simulated time, were it advancing
    const auto now = exchange();
    for (int j = 0; j < 3; ++j) {
      ASSERT_EQ(now.joints[j].position_rad, first.joints[j].position_rad);
    }
  }
  const auto last = exchange();
  EXPECT_EQ(last.imu.orientation_w, first.imu.orientation_w);
  EXPECT_EQ(last.imu.linear_acceleration_z, first.imu.linear_acceleration_z);
}

TEST_F(HoldTest, ReleasedByTheFirstStiffCommandAndStaysReleased)
{
  const auto held = exchange();

  // A slack knee falls under gravity once simulated time runs.
  command.joints[1].stiffness_nm_rad = 100.0;
  command.joints[1].position_rad = held.joints[1].position_rad;
  FeedbackBatch moving{};
  for (int i = 0; i < 200; ++i) {
    moving = exchange();
  }
  EXPECT_NE(moving.joints[0].position_rad, held.joints[0].position_rad);

  // Going slack again does not re-hold: the robot keeps falling.
  command.joints[1].stiffness_nm_rad = 0.0;
  const auto before = exchange();
  FeedbackBatch after{};
  for (int i = 0; i < 100; ++i) {
    after = exchange();
  }
  EXPECT_NE(after.joints[0].position_rad, before.joints[0].position_rad);
}

TEST_F(HoldTest, RobotStartsAtTheInitialPose)
{
  const auto first = exchange();
  EXPECT_DOUBLE_EQ(first.joints[1].position_rad, 0.5);  // left_knee_joint
  EXPECT_DOUBLE_EQ(first.joints[0].position_rad, 0.0);  // not listed: the model's zero
}

}  // namespace
