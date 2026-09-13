// The final host command gate of ADR-002. Runs inside every real-time write().
//
// Deliberately not a ROS node and not a class with dependencies: it takes a state and a command
// tuple and returns a decision. That is the whole reason it can be exhaustively unit-tested.
#ifndef HUMANOID_SAFETY__SAFETY_KERNEL_HPP_
#define HUMANOID_SAFETY__SAFETY_KERNEL_HPP_

#include <cstdint>

#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/joint_manifest.hpp"
#include "humanoid_transport/safety_manifest.hpp"

namespace humanoid::safety
{

enum class Trigger : std::uint8_t
{
  kNone = 0,
  kNotConfigured,
  kStaleFeedback,
  kNonFiniteCommand,
  kPositionEnvelope,
  kVelocityEnvelope,
  kTorqueEnvelope,
  kPowerEnvelope,
  kSlewLimit,
  kAvailabilityMaskChange,
  kSequenceRejected,
  kManifestMismatch,
  kTransportError,
  kCommandLoss,
  kDeadlineMissed,
  kOperatorStop
};

/// What the kernel did to the command, reported so the non-real-time SafetyCoordinator can
/// distinguish "projected a marginal command" from "the controller is asking for the impossible".
struct Verdict
{
  Trigger trigger{Trigger::kNone};
  transport::JointIndex first_offending_joint{0U};
  std::uint8_t joints_projected{0U};
  bool projected{false};
  bool protective_state_required{false};

  [[nodiscard]] constexpr bool accepted() const noexcept { return trigger == Trigger::kNone; }
};

class SafetyKernel
{
public:
  SafetyKernel() = default;

  /// Non-real-time. Copies both manifests; no reference to caller storage is retained.
  [[nodiscard]] bool configure(
    const transport::JointManifest & joints, const transport::SafetyManifest & safety) noexcept;

  /// Real-time. Projects the whole MIT tuple for every joint into the ADR-002 feasible set,
  /// in place. Never allocates, never logs, never blocks. Fails closed when unconfigured.
  [[nodiscard]] Verdict project(
    const transport::FeedbackBatch & measured, transport::CommandBatch & command,
    transport::MonotonicStamp now) noexcept;

  /// Real-time. Overwrites the batch with the qualified local damping action of ADR-002:
  /// zero position and velocity targets, zero feed-forward torque, zero stiffness, damping only.
  void enter_protective(Trigger trigger, transport::CommandBatch & command) noexcept;

  [[nodiscard]] Trigger last_trigger() const noexcept { return last_trigger_; }

  [[nodiscard]] bool configured() const noexcept { return configured_; }

private:
  transport::JointManifest joints_{};
  transport::SafetyManifest safety_{};
  transport::CommandBatch previous_command_{};
  Trigger last_trigger_{Trigger::kNotConfigured};
  std::uint8_t consecutive_bad_cycles_{0U};
  bool configured_{false};
  bool have_previous_{false};
};

}  // namespace humanoid::safety

#endif  // HUMANOID_SAFETY__SAFETY_KERNEL_HPP_
