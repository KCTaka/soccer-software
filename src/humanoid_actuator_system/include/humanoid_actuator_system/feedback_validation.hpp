// Copyright 2026 UTRA-RoboSoccer
// Whether a feedback batch is a current answer to the command last sent.
//
// A pure function so the freshness rule -- ADR-001: reject duplicate, stale,
// or structurally invalid feedback -- can be tested without ros2_control.
#ifndef HUMANOID_ACTUATOR_SYSTEM__FEEDBACK_VALIDATION_HPP_
#define HUMANOID_ACTUATOR_SYSTEM__FEEDBACK_VALIDATION_HPP_

#include <algorithm>
#include <cstdint>
#include <optional>

#include "humanoid_transport/batch_types.hpp"

namespace humanoid::actuator_system
{

/// True only if `batch` answers the command with `sent_sequence`:
///   - a command has been sent at all (`sent_sequence` has a value);
///   - the batch echoes that sequence, so it is not a sample left over from
///     an earlier cycle by a failed exchange (ActuatorTransport::exchange);
///   - it covers exactly the manifest's joints;
///   - at least one joint contributed a fresh sample.
[[nodiscard]] constexpr bool answers_command(
  const transport::FeedbackBatch & batch, std::uint8_t joint_count,
  std::optional<transport::CycleSequence> sent_sequence) noexcept
{
  if (!sent_sequence || batch.sequence != *sent_sequence || batch.joint_count != joint_count) {
    return false;
  }
  return std::any_of(
    batch.joints.begin(), batch.joints.begin() + joint_count,
    [](const transport::JointFeedback & j) {return j.fresh;});
}

}  // namespace humanoid::actuator_system

#endif  // HUMANOID_ACTUATOR_SYSTEM__FEEDBACK_VALIDATION_HPP_
