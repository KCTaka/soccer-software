// Copyright 2026 UTRA-RoboSoccer
// What only the simulator can show about MujocoActuatorTransport, for this binary's robot: that
// the physics moves each joint the way its command pushes, and that an actuator's torque is the
// model's to limit. The behaviour every transport shares is the conformance suite's
// (conformance_test.cpp).

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <chrono>
#include <string>

#include "humanoid_test_support/mujoco_model.hpp"
#include "sim_fixture.hpp"

namespace
{

using humanoid::transport::CommandBatch;
using humanoid::transport::FeedbackBatch;
using humanoid::transport::JointCommand;
using humanoid::transport_mujoco::testing::SimulatedRobot;
using humanoid::transport_mujoco::testing::this_mjcf;

class SimulatedTransport : public ::testing::Test
{
protected:
  void SetUp() override
  {
    sim_ = std::make_unique<SimulatedRobot>();
    ASSERT_GT(sim_->manifest.joint_count, 0U);
    ASSERT_TRUE(sim_->transport.configure(sim_->manifest, sim_->safety));
    command_.joint_count = sim_->manifest.joint_count;
  }

  void TearDown() override {sim_.reset();}

  /// Activates from the initial pose and runs `cycles` exchanges of command_; the last feedback.
  FeedbackBatch run_from_rest(int cycles)
  {
    sim_->transport.deactivate();
    EXPECT_TRUE(sim_->transport.activate());
    FeedbackBatch feedback{};
    for (int i = 0; i < cycles; ++i) {
      ++command_.sequence;
      EXPECT_TRUE(
        sim_->transport.exchange(
          command_, feedback,
          std::chrono::steady_clock::now() + std::chrono::milliseconds{100}).ok());
    }
    return feedback;
  }

  std::unique_ptr<SimulatedRobot> sim_;
  CommandBatch command_{};
};

TEST_F(SimulatedTransport, APositiveTorqueTurnsEachJointPositive)
{
  // ADR-007 gate 2 in the simulator: a joint whose sign the model inverts moves the wrong way.
  // From the same state at rest, a joint pushed by +tau ends ahead of one pushed by -tau whatever
  // gravity, contacts and the rest of the robot do: to second order in time the difference is
  // tau t^2 times the joint's diagonal entry of the (constrained) inverse inertia, which is
  // positive definite. Each joint is pushed alone, so no coupling term enters.
  constexpr double kTorqueNm = 1.0;
  constexpr int kCycles = 10;
  for (std::uint8_t j = 0; j < sim_->manifest.joint_count; ++j) {
    SCOPED_TRACE(sim_->manifest.joints[j].name.data());
    command_.joints[j] = JointCommand{.effort_nm = kTorqueNm};
    const double pushed_forward = run_from_rest(kCycles).joints[j].position_rad;
    command_.joints[j] = JointCommand{.effort_nm = -kTorqueNm};
    const double pushed_back = run_from_rest(kCycles).joints[j].position_rad;
    command_.joints[j] = JointCommand{};
    EXPECT_GT(pushed_forward, pushed_back);
  }
}

TEST_F(SimulatedTransport, ATorqueBeyondTheActuatorsRangeIsHeldAtItsLimit)
{
  // The limit is the model's, read from the same file the transport loads.
  const auto model = humanoid::test_support::load_model(this_mjcf());
  ASSERT_NE(model, nullptr);
  std::vector<double> limits(sim_->manifest.joint_count, 0.0);
  for (std::uint8_t j = 0; j < sim_->manifest.joint_count; ++j) {
    const int joint = mj_name2id(model.get(), mjOBJ_JOINT, sim_->manifest.joints[j].name.data());
    for (int a = 0; a < model->nu; ++a) {
      if (model->actuator_trnid[2 * a] == joint && model->actuator_forcelimited[a]) {
        limits[j] = model->actuator_forcerange[2 * a + 1];
      }
    }
    ASSERT_GT(limits[j], 0.0) << sim_->manifest.joints[j].name.data() << " has no force range";
    command_.joints[j] = JointCommand{.effort_nm = 10.0 * limits[j]};
  }
  const auto feedback = run_from_rest(1);
  for (std::uint8_t j = 0; j < sim_->manifest.joint_count; ++j) {
    EXPECT_DOUBLE_EQ(feedback.joints[j].effort_nm, limits[j])
      << sim_->manifest.joints[j].name.data();
  }
}

}  // namespace
