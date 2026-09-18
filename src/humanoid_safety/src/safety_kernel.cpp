// Copyright 2026 Humanoid Robotics Team

#include "humanoid_safety/safety_kernel.hpp"

#include <cmath>

namespace humanoid::safety
{

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
  configured_ = true;
  last_trigger_ = Trigger::kNone;
  return true;
}

Verdict SafetyKernel::project(
  const transport::FeedbackBatch & /*measured*/, transport::CommandBatch & command,
  transport::MonotonicStamp now) noexcept
{
  if (!configured_) {
    enter_protective(Trigger::kNotConfigured, command);
    return Verdict{Trigger::kNotConfigured, 0U, command.joint_count, true, true};
  }

  // --- Check 1: finiteness ---
  for (std::uint8_t i = 0; i < command.joint_count; ++i) {
    const auto & j = command.joints[i];
    if (!std::isfinite(j.position_rad) || !std::isfinite(j.velocity_rad_s) ||
      !std::isfinite(j.effort_nm) || !std::isfinite(j.stiffness_nm_rad) ||
      !std::isfinite(j.damping_nm_s_rad))
    {
      enter_protective(Trigger::kNonFiniteCommand, command);
      return Verdict{Trigger::kNonFiniteCommand, i, command.joint_count, true, true};
    }
  }

  // --- Checks 2-5: per-joint envelope ---
  for (std::uint8_t i = 0; i < command.joint_count; ++i) {
    const auto & cmd = command.joints[i];
    const auto & env = safety_.envelopes[i];

    // Position bounds
    if (cmd.position_rad < env.position_min_rad ||
      cmd.position_rad > env.position_max_rad)
    {
      enter_protective(Trigger::kPositionEnvelope, command);
      return Verdict{Trigger::kPositionEnvelope, i, command.joint_count, true, true};
    }

    // Velocity bounds
    if (std::abs(cmd.velocity_rad_s) > env.velocity_max_rad_s) {
      enter_protective(Trigger::kVelocityEnvelope, command);
      return Verdict{Trigger::kVelocityEnvelope, i, command.joint_count, true, true};
    }

    // Fixed torque bound (POC: no thermal model, constant peak)
    if (std::abs(cmd.effort_nm) > env.torque_peak_nm) {
      enter_protective(Trigger::kTorqueEnvelope, command);
      return Verdict{Trigger::kTorqueEnvelope, i, command.joint_count, true, true};
    }

    // Slew limit: |τ - τ_prev| / dt ≤ slew_max
    if (have_previous_) {
      const double dt_s =
        std::chrono::duration<double>(now - previous_command_.stamp).count();
      if (dt_s > 0.0) {
        const double d_tau = std::abs(cmd.effort_nm - previous_command_.joints[i].effort_nm);
        if (d_tau / dt_s > env.torque_slew_max_nm_s) {
          enter_protective(Trigger::kSlewLimit, command);
          return Verdict{Trigger::kSlewLimit, i, command.joint_count, true, true};
        }
      }
    }
  }

  // --- Accepted: update previous command for next slew check ---
  previous_command_ = command;
  have_previous_ = true;
  consecutive_bad_cycles_ = 0U;

  return Verdict{Trigger::kNone, 0U, 0U, false, false};
}

void SafetyKernel::enter_protective(Trigger trigger, transport::CommandBatch & command) noexcept
{
  // ADR-002 protective damping: zero everything except a small damping term.
  // In the POC, damping is zero because we have no qualified profile yet.
  for (std::uint8_t i = 0; i < command.joint_count; ++i) {
    auto & joint = command.joints[i];
    joint.position_rad = 0.0;
    joint.velocity_rad_s = 0.0;
    joint.effort_nm = 0.0;
    joint.stiffness_nm_rad = 0.0;
    joint.damping_nm_s_rad = 0.0;
  }
  last_trigger_ = trigger;
}

}  // namespace humanoid::safety
