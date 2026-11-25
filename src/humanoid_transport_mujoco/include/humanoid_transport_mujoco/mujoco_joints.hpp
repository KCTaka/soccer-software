// Copyright 2026 UTRA-RoboSoccer
// The robot's commanded joints in a loaded model: the MIT actuator that drives each, and the pose
// the robot starts in.
//
// The generated MJCF puts one actuator on each commanded joint with gain 1 and an affine bias.
// With ctrl = kp q_d + kd qdot_d + tau_ff and bias (0, -kp, -kd), MuJoCo evaluates the drive's law
// tau = kp (q_d - q) + kd (qdot_d - qdot) + tau_ff at every physics step, within its forcerange.
#ifndef HUMANOID_TRANSPORT_MUJOCO__MUJOCO_JOINTS_HPP_
#define HUMANOID_TRANSPORT_MUJOCO__MUJOCO_JOINTS_HPP_

#include <span>

#include <string>
#include <variant>
#include <vector>

#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/joint_manifest.hpp"

struct mjModel_;
struct mjData_;

namespace humanoid::transport_mujoco
{

/// A commanded joint and its MIT actuator, by MuJoCo address.
struct JointDrive
{
  int qpos_adr{-1};
  int dof_adr{-1};
  int actuator{-1};

  /// Loads `command` into the actuator, where it holds until the next call. Real-time safe.
  void command(mjModel_ & model, mjData_ & data, const transport::JointCommand & command)
  const noexcept;
  /// Zero torque: no stiffness, damping or feedforward. Real-time safe.
  void release(mjModel_ & model, mjData_ & data) const noexcept;
  /// The joint's state, and the actuator force as of the last physics step. Real-time safe.
  [[nodiscard]] transport::JointFeedback feedback(const mjData_ & data) const noexcept;
};

struct JointDrives
{
  std::vector<JointDrive> joints;  // in manifest order
  int root_body{-1};               // root of the kinematic tree the joints share: the robot
};

/// The drive of every joint `manifest` names, or why `model` cannot drive one of them.
[[nodiscard]] std::variant<JointDrives, std::string> map_joint_drives(
  const mjModel_ & model, const transport::JointManifest & manifest);

/// A joint's starting angle (or position, for a slide).
struct InitialJoint
{
  int qpos_adr{-1};
  double value{0.0};
};
using InitialPose = std::vector<InitialJoint>;

/// The pose that starts each of `joints` at its `positions` entry and the rest at the model's
/// reference configuration, with each linkage closed around the named joints; or why it is
/// invalid. `data` is scratch.
[[nodiscard]] std::variant<InitialPose, std::string> resolve_initial_pose(
  const mjModel_ & model, mjData_ & data, std::span<const std::string> joints,
  std::span<const double> positions);

/// Resets `data` to the model's reference configuration, then applies `pose`.
void reset_to(const mjModel_ & model, mjData_ & data, const InitialPose & pose) noexcept;

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__MUJOCO_JOINTS_HPP_
