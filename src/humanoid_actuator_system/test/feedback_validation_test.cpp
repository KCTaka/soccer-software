// Copyright 2026 UTRA-RoboSoccer

#include <gtest/gtest.h>

#include <optional>

#include "humanoid_actuator_system/feedback_validation.hpp"

namespace
{

using humanoid::actuator_system::answers_command;
using humanoid::transport::FeedbackBatch;

constexpr std::uint8_t kJoints = 3;

FeedbackBatch answer(humanoid::transport::CycleSequence sequence)
{
  FeedbackBatch batch{};
  batch.sequence = sequence;
  batch.joint_count = kJoints;
  for (std::uint8_t i = 0; i < kJoints; ++i) {
    batch.joints[i].fresh = true;
  }
  return batch;
}

TEST(AnswersCommand, AcceptsTheAnswerToTheLastCommand)
{
  EXPECT_TRUE(answers_command(answer(7), kJoints, 7U));
}

TEST(AnswersCommand, RejectsFeedbackLeftOverFromAnEarlierCycle)
{
  // A failed exchange for command 8 left command 7's answer in place.
  EXPECT_FALSE(answers_command(answer(7), kJoints, 8U));
}

TEST(AnswersCommand, RejectsBeforeAnyCommandWasSent)
{
  EXPECT_FALSE(answers_command(answer(0), kJoints, std::nullopt));
}

TEST(AnswersCommand, RejectsJointCountMismatch)
{
  EXPECT_FALSE(answers_command(answer(7), kJoints + 1, 7U));
}

TEST(AnswersCommand, RejectsWhenNoJointIsFresh)
{
  auto batch = answer(7);
  for (auto & joint : batch.joints) {
    joint.fresh = false;
  }
  EXPECT_FALSE(answers_command(batch, kJoints, 7U));

  batch.joints[1].fresh = true;
  EXPECT_TRUE(answers_command(batch, kJoints, 7U));
}

TEST(AnswersCommand, IgnoresFreshFlagsBeyondTheManifest)
{
  auto batch = answer(7);
  for (std::uint8_t i = 0; i < kJoints; ++i) {
    batch.joints[i].fresh = false;
  }
  batch.joints[kJoints].fresh = true;  // not a manifest joint
  EXPECT_FALSE(answers_command(batch, kJoints, 7U));
}

}  // namespace
