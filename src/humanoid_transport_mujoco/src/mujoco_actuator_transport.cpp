// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_mujoco/mujoco_actuator_transport.hpp"

#include <mujoco/mujoco.h>

#include <format>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <chrono>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "humanoid_transport/timing.hpp"
#include "humanoid_transport_mujoco/mujoco_placement.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace
{

// File-local helpers. Declared here, defined at the end of this file, so the
// class implementation reads first.
bool fail(std::string_view msg);
bool commands_stiffness(const humanoid::transport::CommandBatch & command) noexcept;

}  // namespace

namespace humanoid::transport_mujoco
{

void MjModelDeleter::operator()(mjModel_ * m) const noexcept
{
  if (m) {
    mj_deleteModel(m);
  }
}

void MjDataDeleter::operator()(mjData_ * d) const noexcept
{
  if (d) {
    mj_deleteData(d);
  }
}

MujocoActuatorTransport::~MujocoActuatorTransport() = default;

// ===========================================================================
// ActuatorTransport overrides
// ===========================================================================

// ---------------------------------------------------------------------------
// configure: load model, map joints, assert integer substep ratio
// ---------------------------------------------------------------------------

bool MujocoActuatorTransport::configure(
  const transport::JointManifest & joints,
  const transport::SafetyManifest & /*safety*/)
{
  const auto params = declare_parameters();
  if (params) {
    hold_until_commanded_ = params->hold_until_commanded;
  }
  return params &&
         load_model(params->mjcf_path) &&
         validate_substeps() &&
         map_joints(joints) &&
         map_imu() &&
         map_initial_pose(*params) &&
         resolve_initial_placement() &&
         configure_disturbance(params->push);
}

// ---------------------------------------------------------------------------
// activate / deactivate
// ---------------------------------------------------------------------------

bool MujocoActuatorTransport::activate()
{
  if (!model_ || !data_) {
    return false;
  }
  reset_to_initial_pose();

  // Start with the lowest collision geom on the ground plane. The height is
  // derived from the model in resolve_initial_placement(), not hardcoded.
  if (root_qpos_adr_ >= 0) {
    data_->qpos[root_qpos_adr_ + 2] = initial_root_z_;
  }

  // Settle the contact state.
  mj_forward(model_.get(), data_.get());

  active_ = true;
  held_ = hold_until_commanded_;

  ground_truth_.start(sim_node_->node());

  return true;
}

void MujocoActuatorTransport::deactivate()
{
  active_ = false;
  ground_truth_.stop();
}

// ---------------------------------------------------------------------------
// capabilities
// ---------------------------------------------------------------------------

transport::TransportCapabilities MujocoActuatorTransport::capabilities() const
{
  transport::TransportCapabilities caps{};
  std::strncpy(
    caps.implementation_name.data(),
    "MujocoActuatorTransport",
    caps.implementation_name.size() - 1);
  caps.transport_class = transport::TransportClass::kSimulated;
  caps.joint_count = joint_count_;
  // Lockstep: one exchange() advances simulated time by exactly one period.
  caps.nominal_cycle_period_us =
    static_cast<std::uint32_t>(transport::kControlPeriod.count());
  caps.worst_case_exchange_us = caps.nominal_cycle_period_us;
  caps.supports_per_joint_disable = false;
  caps.supports_availability_mask = true;
  caps.provides_temperature = false;
  caps.provides_bus_voltage = false;
  caps.provides_imu = imu_sensors_.present();
  caps.is_deterministic = false;  // kSimulated, not kReplay
  caps.tuple_completeness = transport::TupleCompleteness::kFull;
  return caps;
}

// ---------------------------------------------------------------------------
// exchange: the real-time lockstep step
// ---------------------------------------------------------------------------

transport::ExchangeResult MujocoActuatorTransport::exchange(
  const transport::CommandBatch & command,
  transport::FeedbackBatch & feedback,
  transport::MonotonicStamp /*deadline*/) noexcept
{
  // The deadline is not enforced here: a lockstep step cannot be abandoned
  // halfway without corrupting simulator state, and SIL timing is not
  // evidence anyway (ADR-007-05). The caller measures any miss.
  transport::ExchangeResult result{};
  const auto t_start = std::chrono::steady_clock::now();

  if (!active_ || !model_ || !data_) {
    result.error = transport::TransportError::kNotActive;
    return result;
  }

  // While held, the robot stands still and time does not advance. The first stiff command ends
  // the hold for good, even if later commands go slack.
  held_ = held_ && !commands_stiffness(command);
  if (!held_) {
    advance_physics(command);
  }

  const auto t_end = std::chrono::steady_clock::now();

  // Hand the pelvis ground-truth pose to the non-real-time publisher. This
  // is a plain memory copy into a lock-free buffer: no allocation, no ROS or
  // Zenoh call, no unbounded wait (ADR-001). The actual TF publish happens on
  // the publisher's thread, never here.
  if (root_qpos_adr_ >= 0) {
    GroundTruthFrame frame;
    frame.valid = true;
    frame.sequence = command.sequence;
    frame.captured = t_end;
    frame.sim_time_s = data_->time;
    frame.position[0] = data_->qpos[root_qpos_adr_ + 0];
    frame.position[1] = data_->qpos[root_qpos_adr_ + 1];
    frame.position[2] = data_->qpos[root_qpos_adr_ + 2];
    frame.orientation_wxyz[0] = data_->qpos[root_qpos_adr_ + 3];
    frame.orientation_wxyz[1] = data_->qpos[root_qpos_adr_ + 4];
    frame.orientation_wxyz[2] = data_->qpos[root_qpos_adr_ + 5];
    frame.orientation_wxyz[3] = data_->qpos[root_qpos_adr_ + 6];
    ground_truth_.publish(frame);
  }

  // --- Read feedback ---
  feedback.sequence = command.sequence;
  feedback.stamp = t_end;
  feedback.joint_count = joint_count_;
  feedback.availability_mask = command.availability_mask;
  feedback.availability_epoch = command.availability_epoch;

  for (std::uint8_t i = 0; i < joint_count_; ++i) {
    const int dof = joint_map_[i].dof_adr;
    const int qp = joint_map_[i].qpos_adr;

    auto & fb = feedback.joints[i];
    if (dof >= 0 && qp >= 0) {
      fb.position_rad = data_->qpos[qp];
      fb.velocity_rad_s = data_->qvel[dof];
      fb.effort_nm = data_->qfrc_applied[dof];
      fb.fresh = true;
    } else {
      fb.position_rad = 0.0;
      fb.velocity_rad_s = 0.0;
      fb.effort_nm = 0.0;
      fb.fresh = false;
    }
    fb.temperature_c = 0.0F;
    fb.bus_voltage_v = 0.0F;
    fb.fault_bits = 0;
  }

  // Sensors are evaluated by mj_step() at the start of its last substep, so the reading lags the
  // joint state above by one physics step (1 ms), as a real sampled IMU lags by its latency.
  if (imu_sensors_.present()) {
    feedback.imu = read_imu(*data_, imu_sensors_);
  }

  const auto round_trip = std::chrono::duration_cast<std::chrono::nanoseconds>(
    t_end - t_start);
  result.error = transport::TransportError::kNone;
  result.joints_reported = joint_count_;
  result.round_trip = round_trip;
  return result;
}

// ---------------------------------------------------------------------------
// Disable requests (sim: no-op, returns true)
// ---------------------------------------------------------------------------

bool MujocoActuatorTransport::request_joint_disable(
  transport::JointIndex /*joint*/) noexcept
{
  // In simulation, "disable" means zero the applied torque for that joint.
  // The SafetyKernel handles the protective command; the transport just
  // acknowledges the request.
  return true;
}

bool MujocoActuatorTransport::request_all_disable() noexcept
{
  // Zero all applied forces.
  if (data_) {
    for (int i = 0; i < model_->nv; ++i) {
      data_->qfrc_applied[i] = 0.0;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// health_snapshot
// ---------------------------------------------------------------------------

transport::HealthSnapshot MujocoActuatorTransport::health_snapshot() const noexcept
{
  transport::HealthSnapshot snap{};
  snap.active = active_;
  return snap;
}

// ===========================================================================
// Private helpers
// ===========================================================================

// ---------------------------------------------------------------------------
// configure() steps, in call order
// ---------------------------------------------------------------------------

std::optional<MujocoActuatorTransport::SimParameters>
MujocoActuatorTransport::declare_parameters()
{
  // A fresh node per configure(): declare_parameter() throws if a name is
  // declared twice, and reconfigure must re-read the parameter overrides.
  // Destroy any previous node (joining its executor) before the new one
  // exists, so two "mujoco_sim" nodes are never alive at once.
  sim_node_.reset();
  sim_node_ = std::make_unique<SimNode>("mujoco_sim");
  const auto & node = sim_node_->node();

  // Read once here. exchange() never touches parameters, so a runtime change
  // could not take effect; read_only makes `ros2 param set` say so instead
  // of silently doing nothing.
  const auto describe = [](const char * text) {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = text;
      d.read_only = true;
      return d;
    };

  SimParameters params;
  const PushConfig defaults;
  try {
    params.mjcf_path = node->declare_parameter<std::string>(
      "mjcf_path", "", describe("Absolute path to the generated MJCF model"));
    params.hold_until_commanded = node->declare_parameter<bool>(
      "hold_until_commanded", false,
      describe(
        "Hold the robot still and do not advance simulated time until the first command with a "
        "nonzero stiffness arrives, so it does not fall while controllers start"));
    params.initial_pose_joints = node->declare_parameter<std::vector<std::string>>(
      "initial_pose.joints", std::vector<std::string>{},
      describe(
        "Joints the robot starts at a given angle, by name; pairs with initial_pose.positions"));
    params.initial_pose_positions = node->declare_parameter<std::vector<double>>(
      "initial_pose.positions", std::vector<double>{},
      describe(
        "Starting angle of each initial_pose.joints entry, rad. Joints not listed start at 0"));
    params.push.force_n = node->declare_parameter<double>(
      "disturbance.push.force_n", defaults.force_n,
      describe("Push force magnitude, N. 0 disables the push"));
    params.push.start_time_s = node->declare_parameter<double>(
      "disturbance.push.start_time_s", defaults.start_time_s,
      describe("Simulation time at which the push starts, s"));
    params.push.duration_s = node->declare_parameter<double>(
      "disturbance.push.duration_s", defaults.duration_s,
      describe("Push duration, s"));
    params.push.body = node->declare_parameter<std::string>(
      "disturbance.push.body", defaults.body, describe("MJCF body the push is applied to"));
    const auto direction = node->declare_parameter<std::vector<double>>(
      "disturbance.push.direction",
      std::vector<double>(defaults.direction.begin(), defaults.direction.end()),
      describe("World-frame push direction [x, y, z]; normalised"));
    if (direction.size() != params.push.direction.size()) {
      fail("disturbance.push.direction must have exactly 3 elements");
      return std::nullopt;
    }
    std::copy(direction.begin(), direction.end(), params.push.direction.begin());
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    // Typical cause: an integer literal in YAML (50) where a double (50.0) is expected.
    fail(std::format("parameter type mismatch: {}", e.what()));
    return std::nullopt;
  }

  if (params.mjcf_path.empty()) {
    fail("parameter 'mjcf_path' is not set");
    return std::nullopt;
  }
  // Serviced only once every parameter is declared.
  sim_node_->spin();
  return params;
}

bool MujocoActuatorTransport::load_model(const std::string & mjcf_path)
{
  char error_buf[1024] = {};
  model_.reset(mj_loadXML(mjcf_path.c_str(), nullptr, error_buf, sizeof(error_buf)));
  if (!model_) {
    return fail(std::format("mj_loadXML failed: {}", error_buf));
  }

  data_.reset(mj_makeData(model_.get()));
  if (!data_) {
    return fail("mj_makeData failed");
  }

  return true;
}

bool MujocoActuatorTransport::validate_substeps()
{
  // Assert integer substep ratio (ADR-007-03): the physics timestep must
  // divide the control period exactly, or control and physics clocks beat.
  constexpr double kControlPeriodS =
    std::chrono::duration<double>(transport::kControlPeriod).count();
  const double dt = model_->opt.timestep;
  if (dt <= 0.0) {
    return fail(std::format("invalid timestep {}", dt));
  }

  const double ratio = kControlPeriodS / dt;
  n_substeps_ = static_cast<int>(std::lround(ratio));

  if (n_substeps_ < 1 ||
    std::abs(static_cast<double>(n_substeps_) * dt - kControlPeriodS) > 1e-9)
  {
    return fail(std::format(
      "timestep {} s does not divide the {} s control period by an integer. ratio={}",
      dt, kControlPeriodS, ratio));
  }
  return true;
}

bool MujocoActuatorTransport::map_joints(const transport::JointManifest & joints)
{
  joint_count_ = joints.joint_count;
  joint_map_.resize(joint_count_);

  for (std::uint8_t i = 0; i < joint_count_; ++i) {
    const char * name = joints.joints[i].name.data();
    const int jnt_id = mj_name2id(model_.get(), mjtObj::mjOBJ_JOINT, name);
    if (jnt_id < 0) {
      return fail(std::format("joint '{}' not found in model", name));
    }

    // Verify it is a hinge (revolute) or slide (prismatic) — 1-DOF joint.
    const int type = model_->jnt_type[jnt_id];
    if (type != mjtJoint::mjJNT_HINGE && type != mjtJoint::mjJNT_SLIDE) {
      return fail(std::format("joint '{}' is not hinge/slide (type={})", name, type));
    }

    joint_map_[i] = {
      .qpos_adr = model_->jnt_qposadr[jnt_id],
      .dof_adr = model_->jnt_dofadr[jnt_id],
    };

    // The robot is the kinematic tree the commanded joints belong to; a
    // manifest spanning two trees would mean two robots.
    const int root = model_->body_rootid[model_->jnt_bodyid[jnt_id]];
    if (i == 0) {
      robot_root_body_ = root;
    } else if (root != robot_root_body_) {
      return fail(std::format("joint '{}' is not in the same kinematic tree as '{}'",
        name, joints.joints[0].name.data()));
    }
  }
  return true;
}

void MujocoActuatorTransport::advance_physics(const transport::CommandBatch & command) noexcept
{
  // --- Apply MIT torque to qfrc_applied ---
  // tau = K_p * (q_d - q) + K_d * (qdot_d - qdot) + tau_ff
  for (std::uint8_t i = 0; i < joint_count_; ++i) {
    const int dof = joint_map_[i].dof_adr;
    const int qp = joint_map_[i].qpos_adr;
    if (dof < 0 || qp < 0) {
      continue;
    }

    const auto & cmd = command.joints[i];
    const double q = data_->qpos[qp];
    const double qdot = data_->qvel[dof];

    const double tau =
      cmd.stiffness_nm_rad * (cmd.position_rad - q) +
      cmd.damping_nm_s_rad * (cmd.velocity_rad_s - qdot) +
      cmd.effort_nm;

    data_->qfrc_applied[dof] = tau;
  }

  // SIL scenario control: external push, if one is scheduled.
  disturbance_.apply(*data_);

  // --- Step physics: n_substeps of dt_physics ---
  for (int s = 0; s < n_substeps_; ++s) {
    mj_step(model_.get(), data_.get());
  }
}

bool MujocoActuatorTransport::map_imu()
{
  auto found = find_imu_sensors(*model_);
  if (const auto * error = std::get_if<std::string>(&found)) {
    return fail(*error);
  }
  imu_sensors_ = std::get<ImuSensors>(found);
  return true;
}

bool MujocoActuatorTransport::map_initial_pose(const SimParameters & params)
{
  initial_pose_.clear();
  if (params.initial_pose_joints.size() != params.initial_pose_positions.size()) {
    return fail(std::format(
      "initial_pose has {} joints but {} positions", params.initial_pose_joints.size(),
      params.initial_pose_positions.size()));
  }
  for (std::size_t i = 0; i < params.initial_pose_joints.size(); ++i) {
    const auto & name = params.initial_pose_joints[i];
    const double value = params.initial_pose_positions[i];
    const int jnt_id = mj_name2id(model_.get(), mjtObj::mjOBJ_JOINT, name.c_str());
    if (jnt_id < 0) {
      return fail(std::format("initial_pose joint '{}' not found in model", name));
    }
    const int type = model_->jnt_type[jnt_id];
    if (type != mjtJoint::mjJNT_HINGE && type != mjtJoint::mjJNT_SLIDE) {
      return fail(std::format("initial_pose joint '{}' is not hinge/slide", name));
    }
    if (model_->jnt_limited[jnt_id] &&
      (value<model_->jnt_range[2 * jnt_id] || value> model_->jnt_range[2 * jnt_id + 1]))
    {
      return fail(std::format("initial_pose angle {} for '{}' is outside its range", value, name));
    }
    initial_pose_.push_back({model_->jnt_qposadr[jnt_id], value});
  }
  return true;
}

void MujocoActuatorTransport::reset_to_initial_pose() noexcept
{
  mj_resetData(model_.get(), data_.get());
  for (const auto & joint : initial_pose_) {
    data_->qpos[joint.qpos_adr] = joint.value;
  }
}

bool MujocoActuatorTransport::resolve_initial_placement()
{
  root_qpos_adr_ = free_joint_qpos_adr(*model_, robot_root_body_).value_or(-1);
  if (root_qpos_adr_ < 0) {
    return true;  // fixed-base: nothing to place, no ground truth to publish
  }

  // Kinematics at the model's reference configuration (qpos0), which is the
  // configuration activate() resets to.
  reset_to_initial_pose();
  mj_kinematics(model_.get(), data_.get());
  const auto ground = ground_plane_z(*model_, *data_);
  if (!ground) {
    return fail("floating-base robot but no horizontal ground plane in the model");
  }
  const auto lowest = lowest_collision_z(*model_, *data_, robot_root_body_);
  if (!lowest) {
    return fail("floating-base robot has no collidable geometry to stand on");
  }
  initial_root_z_ = data_->qpos[root_qpos_adr_ + 2] + (*ground - *lowest);
  std::cerr << std::format(
    "MujocoActuatorTransport: initial root height {:.4f} m, derived from collision geometry\n",
    initial_root_z_);
  return true;
}

bool MujocoActuatorTransport::configure_disturbance(const PushConfig & config)
{
  active_ = false;

  if (const auto error = disturbance_.configure(*model_, config)) {
    return fail(*error);
  }
  if (config.force_n > 0.0) {
    std::cerr << std::format(
      "MujocoActuatorTransport: push {} N on '{}' at t={} s for {} s, direction [{}, {}, {}]\n",
      config.force_n, config.body, config.start_time_s, config.duration_s,
      config.direction[0], config.direction[1], config.direction[2]);
  }
  return true;
}

}  // namespace humanoid::transport_mujoco

namespace
{

bool fail(std::string_view msg)
{
  std::cerr << "MujocoActuatorTransport: " << msg << "\n";
  return false;
}

// True when any joint of `command` has a nonzero stiffness: a controller is driving the robot.
bool commands_stiffness(const humanoid::transport::CommandBatch & command) noexcept
{
  return std::any_of(
    command.joints.begin(), command.joints.begin() + command.joint_count,
    [](const humanoid::transport::JointCommand & joint) {return joint.stiffness_nm_rad != 0.0;});
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(
  humanoid::transport_mujoco::MujocoActuatorTransport,
  humanoid::transport::ActuatorTransport)
