// Copyright 2026 UTRA-RoboSoccer
// The SIL start state, for this binary's robot: hold_until_commanded keeps the robot still at its
// initial pose until a controller commands stiffness, and the initial pose puts the first actuated
// joint away from the model's zero. Its own main hands the transport those parameters.

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <format>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "humanoid_test_support/mujoco_model.hpp"
#include "sim_fixture.hpp"

namespace
{

using humanoid::transport::CommandBatch;
using humanoid::transport::FeedbackBatch;
using humanoid::transport::JointCommand;
using humanoid::transport_mujoco::testing::actuated_joints;
using humanoid::transport_mujoco::testing::SimulatedRobot;
using humanoid::transport_mujoco::testing::this_mjcf;

/// The joint the initial pose places, and where: a quarter of the way through its range, which is
/// not its zero for a range around zero.
struct Pose
{
  std::string joint;
  double position_rad;
};

Pose initial_pose()
{
  const auto joint = actuated_joints(this_mjcf()).front();
  const auto model = humanoid::test_support::load_model(this_mjcf());
  const int id = mj_name2id(model.get(), mjOBJ_JOINT, joint.c_str());
  const double lower = model->jnt_range[2 * id];
  const double upper = model->jnt_range[2 * id + 1];
  return {joint, lower + 0.25 * (upper - lower)};
}

class HoldTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(sim_.transport.configure(sim_.manifest, sim_.safety));
    ASSERT_TRUE(sim_.transport.activate());
    command_.joint_count = sim_.manifest.joint_count;
  }

  FeedbackBatch exchange()
  {
    ++command_.sequence;
    FeedbackBatch feedback{};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    EXPECT_TRUE(sim_.transport.exchange(command_, feedback, deadline).ok());
    return feedback;
  }

  /// `cycles` exchanges; the last feedback.
  FeedbackBatch exchange_many(int cycles)
  {
    FeedbackBatch feedback{};
    for (int i = 0; i < cycles; ++i) {
      feedback = exchange();
    }
    return feedback;
  }

  SimulatedRobot sim_;
  CommandBatch command_{};
};

TEST_F(HoldTest, RobotStandsStillWhileNoStiffnessIsCommanded)
{
  // Slack joints and an effort: either would move the robot if simulated time ran.
  command_.joints[0] = JointCommand{.effort_nm = 1.0};
  const auto first = exchange();
  for (int i = 0; i < 200; ++i) {  // a full second of simulated time, were it advancing
    const auto now = exchange();
    for (std::uint8_t j = 0; j < sim_.manifest.joint_count; ++j) {
      ASSERT_EQ(now.joints[j].position_rad, first.joints[j].position_rad) << +j;
    }
  }
  const auto last = exchange();
  EXPECT_EQ(last.imu.orientation_w, first.imu.orientation_w);
  EXPECT_EQ(last.imu.linear_acceleration_z, first.imu.linear_acceleration_z);
}

TEST_F(HoldTest, ReleasedByTheFirstStiffCommandAndStaysReleased)
{
  const auto held = exchange();

  // A stiff command toward a new target moves the joint only once simulated time runs.
  command_.joints[0] =
    JointCommand{.position_rad = held.joints[0].position_rad + 0.1, .stiffness_nm_rad = 100.0};
  const auto released = exchange_many(200);
  EXPECT_NE(released.joints[0].position_rad, held.joints[0].position_rad);

  // Going slack again does not re-hold: an effort still moves the joint.
  command_.joints[0] = JointCommand{.effort_nm = 1.0};
  const auto slack = exchange();
  EXPECT_NE(exchange_many(100).joints[0].position_rad, slack.joints[0].position_rad);
}

TEST_F(HoldTest, RobotStartsAtTheInitialPose)
{
  const auto first = exchange();
  EXPECT_DOUBLE_EQ(first.joints[0].position_rad, initial_pose().position_rad);
  for (std::uint8_t j = 1; j < sim_.manifest.joint_count; ++j) {
    EXPECT_DOUBLE_EQ(first.joints[j].position_rad, 0.0) << +j << ": not posed, the model's zero";
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const auto pose = initial_pose();
  const std::vector<std::string> parameters{
    "hold_until_commanded:=true",
    "initial_pose.joints:=[" + pose.joint + "]",
    // Scientific notation always reads as a double, and round-trips exactly.
    std::format("initial_pose.positions:=[{:.17e}]", pose.position_rad)};
  return humanoid::transport_mujoco::testing::run_tests(argc, argv, parameters);
}
