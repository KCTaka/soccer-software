#include "humanoid_safety/safety_kernel.hpp"

namespace humanoid::safety
{

// SCAFFOLD. Every entry point below fails closed: an unimplemented safety kernel must refuse
// motion, never permit it. Implementing these is bootstrap step B6 and is gated on the
// model/robot_model.yaml envelope fields (Model Gate 0, G1 and G2).

bool SafetyKernel::configure(
  const transport::JointManifest & joints, const transport::SafetyManifest & safety) noexcept
{
  if (joints.joint_count == 0U || joints.joint_count != safety.joint_count) {
    return false;
  }
  if (joints.joint_count > transport::kMaxJoints) {
    return false;
  }
  joints_ = joints;
  safety_ = safety;
  previous_command_ = transport::CommandBatch{};
  have_previous_ = false;
  consecutive_bad_cycles_ = 0U;
  configured_ = false;  // Flipped to true only when project() is implemented.
  last_trigger_ = Trigger::kNotConfigured;
  return true;
}

Verdict SafetyKernel::project(
  const transport::FeedbackBatch & /*measured*/, transport::CommandBatch & command,
  transport::MonotonicStamp /*now*/) noexcept
{
  enter_protective(Trigger::kNotConfigured, command);
  last_trigger_ = Trigger::kNotConfigured;
  return Verdict{Trigger::kNotConfigured, 0U, command.joint_count, true, true};
}

void SafetyKernel::enter_protective(Trigger trigger, transport::CommandBatch & command) noexcept
{
  for (std::size_t i = 0; i < transport::kMaxJoints; ++i) {
    auto & joint = command.joints[i];
    joint.position_rad = 0.0;
    joint.velocity_rad_s = 0.0;
    joint.effort_nm = 0.0;
    joint.stiffness_nm_rad = 0.0;
    joint.damping_nm_s_rad = 0.0;  // Replaced by the qualified damping value once G3 closes.
  }
  last_trigger_ = trigger;
}

}  // namespace humanoid::safety
