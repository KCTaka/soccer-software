// Copyright 2026 UTRA-RoboSoccer
// The SafetyKernel, the final host command gate of ADR-002, against hand-built batches: every
// check it makes, at and just past each bound, on every joint, and the damping action it leaves
// in the batch when it refuses one.

#include <gmock/gmock.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <ostream>
#include <string>
#include <vector>

#include "humanoid_safety/safety_kernel.hpp"
#include "humanoid_test_support/allocation_counter.hpp"
#include "humanoid_test_support/manifests.hpp"

namespace
{

using humanoid::safety::SafetyKernel;
using humanoid::safety::Trigger;
using humanoid::safety::Verdict;
using humanoid::test_support::joint_manifest;
using humanoid::test_support::safety_manifest;
using humanoid::transport::CommandBatch;
using humanoid::transport::FeedbackBatch;
using humanoid::transport::JointCommand;
using humanoid::transport::JointEnvelope;
using humanoid::transport::kMitFields;
using humanoid::transport::MonotonicStamp;
using std::chrono::milliseconds;

constexpr std::uint8_t kJoints = 3;

/// Every joint's envelope. Each bound is a round number, so a value just past it is plainly past.
JointEnvelope envelope()
{
  JointEnvelope e;
  e.position_min_rad = -1.0;
  e.position_max_rad = 1.0;
  e.velocity_max_rad_s = 10.0;
  e.torque_peak_nm = 20.0;
  e.torque_slew_max_nm_s = 1000.0;
  e.stiffness_max_nm_rad = 100.0;
  e.damping_max_nm_s_rad = 5.0;
  return e;
}

/// ADR-002's damping action, as the kernel writes it over a refused batch.
MATCHER(IsDamped, "is the damping action: every field zero")
{
  return arg.position_rad == 0.0 && arg.velocity_rad_s == 0.0 && arg.effort_nm == 0.0 &&
         arg.stiffness_nm_rad == 0.0 && arg.damping_nm_s_rad == 0.0;
}

class SafetyKernelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto joints = joint_manifest({"hip", "knee", "ankle"});
    ASSERT_TRUE(kernel_.configure(joints, safety_manifest(kJoints, envelope())));
  }

  /// A batch of `kJoints` that holds every joint at zero with gains inside the envelope.
  static CommandBatch batch(MonotonicStamp stamp)
  {
    CommandBatch command{};
    command.joint_count = kJoints;
    command.stamp = stamp;
    for (std::uint8_t i = 0; i < kJoints; ++i) {
      command.joints[i] = JointCommand{.stiffness_nm_rad = 50.0, .damping_nm_s_rad = 1.0};
    }
    return command;
  }

  Verdict project(CommandBatch & command)
  {
    return kernel_.project(measured_, command, command.stamp);
  }

  /// The joints of `command` the kernel would see.
  static auto joints_of(const CommandBatch & command)
  {
    return std::vector<JointCommand>(command.joints.begin(), command.joints.begin() + kJoints);
  }

  SafetyKernel kernel_;
  FeedbackBatch measured_{};
  const MonotonicStamp t0_{std::chrono::seconds{100}};
};

// --- Configuration -----------------------------------------------------------------------------

TEST(SafetyKernel, FailsClosedUntilConfigured)
{
  SafetyKernel kernel;
  EXPECT_FALSE(kernel.configured());
  EXPECT_EQ(kernel.last_trigger(), Trigger::kNotConfigured);

  CommandBatch command{};
  command.joint_count = 1;
  command.joints[0] = JointCommand{.position_rad = 0.1, .stiffness_nm_rad = 10.0};
  const auto verdict = kernel.project(FeedbackBatch{}, command, MonotonicStamp{});
  EXPECT_EQ(verdict.trigger, Trigger::kNotConfigured);
  EXPECT_TRUE(verdict.protective_state_required);
  EXPECT_THAT(command.joints[0], IsDamped());
}

TEST(SafetyKernel, RefusesManifestsThatDoNotDescribeTheSameJoints)
{
  SafetyKernel kernel;
  EXPECT_FALSE(kernel.configure(joint_manifest({}), safety_manifest(0)));
  EXPECT_FALSE(kernel.configure(joint_manifest({"hip", "knee"}), safety_manifest(3, envelope())));
  EXPECT_FALSE(kernel.configured());
  EXPECT_TRUE(kernel.configure(joint_manifest({"hip", "knee"}), safety_manifest(2, envelope())));
  EXPECT_TRUE(kernel.configured());
  EXPECT_EQ(kernel.last_trigger(), Trigger::kNone);
}

// --- The envelope ------------------------------------------------------------------------------

TEST_F(SafetyKernelTest, PassesACommandInsideTheEnvelopeUnchanged)
{
  auto command = batch(t0_);
  command.joints[1] = JointCommand{
    .position_rad = 0.5, .velocity_rad_s = -3.0, .effort_nm = 4.0, .stiffness_nm_rad = 80.0,
    .damping_nm_s_rad = 2.0};
  const auto sent = command;
  const auto verdict = project(command);
  EXPECT_TRUE(verdict.accepted());
  EXPECT_FALSE(verdict.protective_state_required);
  EXPECT_EQ(kernel_.last_trigger(), Trigger::kNone);
  for (std::uint8_t i = 0; i < kJoints; ++i) {
    for (const auto & field : kMitFields) {
      EXPECT_EQ(command.joints[i].*field.member, sent.joints[i].*field.member)
        << "joint " << +i << " " << field.name;
    }
  }
}

/// One bound of the envelope, by the field it limits, a value at it and one just past it.
struct Bound
{
  const char * name;
  double JointCommand::* field;
  double at;
  double past;
  Trigger trigger;
};

std::ostream & operator<<(std::ostream & out, const Bound & bound) {return out << bound.name;}

class EnvelopeBound : public SafetyKernelTest, public ::testing::WithParamInterface<Bound> {};

TEST_P(EnvelopeBound, AcceptsTheBoundItself)
{
  for (std::uint8_t joint = 0; joint < kJoints; ++joint) {
    SCOPED_TRACE("joint " + std::to_string(joint));
    auto command = batch(t0_ + milliseconds{joint});
    command.joints[joint].*GetParam().field = GetParam().at;
    if (GetParam().field == &JointCommand::effort_nm) {
      SetUp();  // a fresh slew reference: this is the torque bound, not the slew limit
      ASSERT_FALSE(HasFatalFailure());
    }
    EXPECT_TRUE(project(command).accepted());
  }
}

TEST_P(EnvelopeBound, RefusesTheWholeBatchJustPastItAndNamesTheJoint)
{
  for (std::uint8_t joint = 0; joint < kJoints; ++joint) {
    SCOPED_TRACE("joint " + std::to_string(joint));
    SetUp();
    ASSERT_FALSE(HasFatalFailure());
    auto command = batch(t0_);
    command.joints[joint].*GetParam().field = GetParam().past;
    const auto verdict = project(command);
    EXPECT_EQ(verdict.trigger, GetParam().trigger);
    EXPECT_EQ(verdict.first_offending_joint, joint);
    EXPECT_TRUE(verdict.protective_state_required);
    EXPECT_EQ(kernel_.last_trigger(), GetParam().trigger);
    // Not only the offending joint: the whole robot gets the damping action.
    EXPECT_THAT(joints_of(command), ::testing::Each(IsDamped()));
  }
}

INSTANTIATE_TEST_SUITE_P(
  EveryBound, EnvelopeBound, ::testing::Values(
    Bound{"PositionMax", &JointCommand::position_rad, 1.0, 1.001, Trigger::kPositionEnvelope},
    Bound{"PositionMin", &JointCommand::position_rad, -1.0, -1.001, Trigger::kPositionEnvelope},
    Bound{"Velocity", &JointCommand::velocity_rad_s, 10.0, 10.001, Trigger::kVelocityEnvelope},
    Bound{"NegativeVelocity", &JointCommand::velocity_rad_s, -10.0, -10.001,
      Trigger::kVelocityEnvelope},
    Bound{"Torque", &JointCommand::effort_nm, 20.0, 20.001, Trigger::kTorqueEnvelope},
    Bound{"NegativeTorque", &JointCommand::effort_nm, -20.0, -20.001, Trigger::kTorqueEnvelope},
    // Gains are refused past their caps, never clipped, and are magnitudes: never negative.
    Bound{"Stiffness", &JointCommand::stiffness_nm_rad, 100.0, 100.001,
      Trigger::kStiffnessEnvelope},
    Bound{"NegativeStiffness", &JointCommand::stiffness_nm_rad, 0.0, -0.001,
      Trigger::kStiffnessEnvelope},
    Bound{"Damping", &JointCommand::damping_nm_s_rad, 5.0, 5.001, Trigger::kDampingEnvelope},
    Bound{"NegativeDamping", &JointCommand::damping_nm_s_rad, 0.0, -0.001,
      Trigger::kDampingEnvelope}),
  [](const auto & row) {return std::string{row.param.name};});

TEST_F(SafetyKernelTest, RefusesABatchThatIsNotTheConfiguredJoints)
{
  // Fewer joints would leave some uncommanded; more would be checked against envelopes the
  // manifest never gave.
  constexpr std::uint8_t kWrongCounts[] = {0, kJoints - 1, kJoints + 1};
  for (const std::uint8_t count : kWrongCounts) {
    SCOPED_TRACE("joint_count " + std::to_string(count));
    auto command = batch(t0_);
    command.joint_count = count;
    const auto verdict = project(command);
    EXPECT_EQ(verdict.trigger, Trigger::kManifestMismatch);
    EXPECT_TRUE(verdict.protective_state_required);
  }
}

TEST_F(SafetyKernelTest, RefusesANonFiniteFieldOfAnyJoint)
{
  // A NaN compares false with every bound, so without this check it would pass all of them.
  for (const auto & field : kMitFields) {
    for (const double bad :
      {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
    {
      for (std::uint8_t joint = 0; joint < kJoints; ++joint) {
        SCOPED_TRACE(
          std::string{field.name} + " = " + std::to_string(bad) + " on joint " +
          std::to_string(joint));
        auto command = batch(t0_);
        command.joints[joint].*field.member = bad;
        const auto verdict = project(command);
        EXPECT_EQ(verdict.trigger, Trigger::kNonFiniteCommand);
        EXPECT_EQ(verdict.first_offending_joint, joint);
        EXPECT_THAT(joints_of(command), ::testing::Each(IsDamped()));
      }
    }
  }
}

// --- Torque slew -------------------------------------------------------------------------------

TEST_F(SafetyKernelTest, LimitsHowFastTheTorqueChangesBetweenAcceptedCommands)
{
  // 1000 N*m/s over one 5 ms cycle is 5 N*m.
  auto first = batch(t0_);
  ASSERT_TRUE(project(first).accepted());

  auto within = batch(t0_ + milliseconds{5});
  within.joints[2].effort_nm = 4.9;
  EXPECT_TRUE(project(within).accepted());

  auto too_fast = batch(t0_ + milliseconds{10});
  too_fast.joints[2].effort_nm = 4.9 + 5.1;
  const auto verdict = project(too_fast);
  EXPECT_EQ(verdict.trigger, Trigger::kSlewLimit);
  EXPECT_EQ(verdict.first_offending_joint, 2U);
}

TEST_F(SafetyKernelTest, MeasuresTheSlewFromTheLastAcceptedCommandNotARefusedOne)
{
  auto accepted = batch(t0_);
  ASSERT_TRUE(project(accepted).accepted());
  auto refused = batch(t0_ + milliseconds{5});
  refused.joints[0].position_rad = 2.0;
  refused.joints[0].effort_nm = 15.0;
  ASSERT_FALSE(project(refused).accepted());

  // 9 N*m over the 10 ms since the accepted command is 900 N*m/s: inside the limit.
  auto next = batch(t0_ + milliseconds{10});
  next.joints[0].effort_nm = 9.0;
  EXPECT_TRUE(project(next).accepted());
}

TEST_F(SafetyKernelTest, ReconfiguringForgetsTheSlewReference)
{
  auto accepted = batch(t0_);
  ASSERT_TRUE(project(accepted).accepted());
  SetUp();
  ASSERT_FALSE(HasFatalFailure());
  auto jump = batch(t0_ + milliseconds{5});
  jump.joints[0].effort_nm = 15.0;
  EXPECT_TRUE(project(jump).accepted());
}

// --- The damping action ------------------------------------------------------------------------

TEST_F(SafetyKernelTest, DampingIsNotAFaultAndRecordsNoTrigger)
{
  // It is also what the robot receives while no controller owns the tuple.
  auto command = batch(t0_);
  command.joints[0].position_rad = 0.3;
  kernel_.apply_damping(command);
  EXPECT_THAT(joints_of(command), ::testing::Each(IsDamped()));
  EXPECT_EQ(kernel_.last_trigger(), Trigger::kNone);
}

TEST_F(SafetyKernelTest, EnteringProtectiveDampsAndRecordsWhy)
{
  auto command = batch(t0_);
  kernel_.enter_protective(Trigger::kCommandLoss, command);
  EXPECT_THAT(joints_of(command), ::testing::Each(IsDamped()));
  EXPECT_EQ(kernel_.last_trigger(), Trigger::kCommandLoss);
}

// --- Real time ---------------------------------------------------------------------------------

TEST_F(SafetyKernelTest, ProjectAllocatesNothing)
{
  const std::size_t before = humanoid::test_support::thread_allocations();
  for (int i = 0; i < 100; ++i) {
    auto command = batch(t0_ + milliseconds{5 * i});
    command.joints[i % kJoints].position_rad = (i % 2 == 0) ? 0.5 : 5.0;  // accepted, refused
    static_cast<void>(project(command));
  }
  EXPECT_EQ(humanoid::test_support::thread_allocations() - before, 0U);
}

}  // namespace
