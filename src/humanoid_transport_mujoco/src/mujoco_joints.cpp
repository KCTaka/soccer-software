// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_mujoco/mujoco_joints.hpp"

#include <mujoco/mujoco.h>

#include <format>
#include <span>

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "humanoid_transport_mujoco/mujoco_placement.hpp"

namespace
{

bool is_hinge_or_slide(const mjModel & model, int joint) noexcept;
std::variant<int, std::string> find_mit_actuator(const mjModel & model, int joint);
void set_gains(mjModel & model, int actuator, double stiffness, double damping) noexcept;

}  // namespace

namespace humanoid::transport_mujoco
{

void JointDrive::command(
  mjModel_ & model, mjData_ & data, const transport::JointCommand & command) const noexcept
{
  data.ctrl[actuator] = command.stiffness_nm_rad * command.position_rad +
    command.damping_nm_s_rad * command.velocity_rad_s + command.effort_nm;
  set_gains(model, actuator, command.stiffness_nm_rad, command.damping_nm_s_rad);
}

void JointDrive::release(mjModel_ & model, mjData_ & data) const noexcept
{
  data.ctrl[actuator] = 0.0;
  set_gains(model, actuator, 0.0, 0.0);
}

transport::JointFeedback JointDrive::feedback(const mjData_ & data) const noexcept
{
  return {
    .position_rad = data.qpos[qpos_adr],
    .velocity_rad_s = data.qvel[dof_adr],
    .effort_nm = data.actuator_force[actuator],
    .fresh = true,
  };
}

std::variant<JointDrives, std::string> map_joint_drives(
  const mjModel_ & model, const transport::JointManifest & manifest)
{
  JointDrives drives;
  for (std::uint8_t i = 0; i < manifest.joint_count; ++i) {
    const char * name = manifest.joints[i].name.data();
    const int joint = mj_name2id(&model, mjtObj::mjOBJ_JOINT, name);
    if (joint < 0) {
      return std::format("joint '{}' not found in model", name);
    }
    if (!is_hinge_or_slide(model, joint)) {
      return std::format("joint '{}' is not hinge/slide (type={})", name, model.jnt_type[joint]);
    }
    const auto actuator = find_mit_actuator(model, joint);
    if (const auto * error = std::get_if<std::string>(&actuator)) {
      return *error;
    }
    drives.joints.push_back({
        .qpos_adr = model.jnt_qposadr[joint],
        .dof_adr = model.jnt_dofadr[joint],
        .actuator = std::get<int>(actuator),
    });

    // Joints in two kinematic trees would be two robots.
    const int root = model.body_rootid[model.jnt_bodyid[joint]];
    if (i == 0) {
      drives.root_body = root;
    } else if (root != drives.root_body) {
      return std::format(
        "joint '{}' is not in the same kinematic tree as '{}'", name,
        manifest.joints[0].name.data());
    }
  }
  return drives;
}

std::variant<InitialPose, std::string> resolve_initial_pose(
  const mjModel_ & model, mjData_ & data, std::span<const std::string> joints,
  std::span<const double> positions)
{
  if (joints.size() != positions.size()) {
    return std::format(
      "initial_pose has {} joints but {} positions", joints.size(), positions.size());
  }
  InitialPose pose;
  std::vector<int> held_dofs;
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const auto & name = joints[i];
    const double value = positions[i];
    const int joint = mj_name2id(&model, mjtObj::mjOBJ_JOINT, name.c_str());
    if (joint < 0) {
      return std::format("initial_pose joint '{}' not found in model", name);
    }
    if (!is_hinge_or_slide(model, joint)) {
      return std::format("initial_pose joint '{}' is not hinge/slide", name);
    }
    const mjtNum * range = model.jnt_range + 2 * joint;
    if (model.jnt_limited[joint] && (value<range[0] || value> range[1])) {
      return std::format("initial_pose angle {} for '{}' is outside its range", value, name);
    }
    pose.push_back({.qpos_adr = model.jnt_qposadr[joint], .value = value});
    held_dofs.push_back(model.jnt_dofadr[joint]);
  }

  // A linkage's input and coupler are not commanded joints, so the pose leaves them at the zero
  // pose; left open, the closure would snap the loop shut on the first step.
  reset_to(model, data, pose);
  if (const auto error = close_equality_constraints(model, data, held_dofs)) {
    return *error;
  }
  for (int j = 0; j < model.njnt; ++j) {
    const int adr = model.jnt_qposadr[j];
    const bool named = std::ranges::any_of(
      pose, [adr](const InitialJoint & joint) {return joint.qpos_adr == adr;});
    if (is_hinge_or_slide(model, j) && !named && data.qpos[adr] != model.qpos0[adr]) {
      pose.push_back({.qpos_adr = adr, .value = data.qpos[adr]});
    }
  }
  return pose;
}

void reset_to(const mjModel_ & model, mjData_ & data, const InitialPose & pose) noexcept
{
  mj_resetData(&model, &data);
  for (const auto & joint : pose) {
    data.qpos[joint.qpos_adr] = joint.value;
  }
}

}  // namespace humanoid::transport_mujoco

namespace
{

bool is_hinge_or_slide(const mjModel & model, int joint) noexcept
{
  const int type = model.jnt_type[joint];
  return type == mjtJoint::mjJNT_HINGE || type == mjtJoint::mjJNT_SLIDE;
}

// Found by what it drives, not by name: the generated actuator, force = ctrl + b0 + b1 q + b2 qdot,
// on the joint itself (gear 1), with no activation state.
std::variant<int, std::string> find_mit_actuator(const mjModel & model, int joint)
{
  const char * name = mj_id2name(&model, mjtObj::mjOBJ_JOINT, joint);
  std::optional<int> found;
  for (int a = 0; a < model.nu; ++a) {
    if (model.actuator_trntype[a] != mjtTrn::mjTRN_JOINT || model.actuator_trnid[2 * a] != joint) {
      continue;
    }
    if (found) {
      return std::format("joint '{}' is driven by more than one actuator", name);
    }
    const mjtNum gain = model.actuator_gainprm[a * mjNGAIN];
    const mjtNum gear = model.actuator_gear[a * 6];
    if (model.actuator_gaintype[a] != mjtGain::mjGAIN_FIXED || gain != 1.0 ||
      model.actuator_biastype[a] != mjtBias::mjBIAS_AFFINE ||
      model.actuator_dyntype[a] != mjtDyn::mjDYN_NONE || gear != 1.0 ||
      model.actuator_ctrllimited[a])
    {
      return std::format(
        "actuator on joint '{}' is not an MIT actuator (fixed gain 1, affine bias, no dynamics, "
        "gear 1, no ctrlrange); regenerate the MJCF with model/generators", name);
    }
    found = a;
  }
  if (!found) {
    return std::format(
      "joint '{}' has no actuator; regenerate the MJCF with model/generators", name);
  }
  return *found;
}

// The affine bias b1 q + b2 qdot carries the stiffness and damping terms.
void set_gains(mjModel & model, int actuator, double stiffness, double damping) noexcept
{
  mjtNum * bias = model.actuator_biasprm + actuator * mjNBIAS;
  bias[1] = -stiffness;
  bias[2] = -damping;
}

}  // namespace
