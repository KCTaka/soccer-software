// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_mujoco/mujoco_actuator_transport.hpp"

#include <mujoco/mujoco.h>

#include <format>
#include <span>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "humanoid_transport/timing.hpp"
#include "humanoid_transport_mujoco/mujoco_placement.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace
{

bool fail(std::string_view message);
template<typename T>
std::optional<T> value_or_fail(std::variant<T, std::string> result);
std::variant<int, std::string> substeps_per_period(double timestep);

}  // namespace

namespace humanoid::transport_mujoco
{

void MjModelDeleter::operator()(mjModel_ * model) const noexcept {mj_deleteModel(model);}
void MjDataDeleter::operator()(mjData_ * data) const noexcept {mj_deleteData(data);}

MujocoActuatorTransport::~MujocoActuatorTransport() = default;

// ===========================================================================
// ActuatorTransport overrides
// ===========================================================================

bool MujocoActuatorTransport::configure(
  const transport::JointManifest & joints,
  const transport::SafetyManifest & /*safety*/)
{
  // Nothing from a previous configure may outlive a failed one.
  active_ = false;
  drives_.clear();
  initial_pose_.clear();
  root_qpos_adr_ = -1;

  const auto params = declare_parameters();
  if (!params || !load_model(params->mjcf_path)) {
    return false;
  }
  const auto substeps = value_or_fail(substeps_per_period(model_->opt.timestep));
  auto drives = value_or_fail(map_joint_drives(*model_, joints));
  if (!substeps || !drives) {
    return false;
  }
  const auto imu = value_or_fail(find_imu_sensors(*model_));
  auto pose = value_or_fail(
    resolve_initial_pose(
      *model_, *data_, params->initial_pose_joints, params->initial_pose_positions));
  if (!imu || !pose) {
    return false;
  }
  substeps_ = *substeps;
  drives_ = std::move(drives->joints);
  imu_ = *imu;
  initial_pose_ = std::move(*pose);

  root_qpos_adr_ = free_joint_qpos_adr(*model_, drives->root_body).value_or(-1);
  if (root_qpos_adr_ >= 0) {
    reset_to(*model_, *data_, initial_pose_);
    const auto root_z = value_or_fail(
      standing_root_z(*model_, *data_, drives->root_body, root_qpos_adr_));
    if (!root_z) {
      return false;
    }
    root_z_ = *root_z;
    std::cerr << std::format(
      "MujocoActuatorTransport: initial root height {:.4f} m, derived from collision geometry\n",
      root_z_);
  }

  if (const auto error = disturbance_.configure(*model_, params->push)) {
    return fail(*error);
  }
  if (const auto & push = params->push; push.force_n > 0.0) {
    std::cerr << std::format(
      "MujocoActuatorTransport: push {} N on '{}' at t={} s for {} s, direction [{}, {}, {}]\n",
      push.force_n, push.body, push.start_time_s, push.duration_s,
      push.direction[0], push.direction[1], push.direction[2]);
  }
  hold_until_commanded_ = params->hold_until_commanded;
  return true;
}

bool MujocoActuatorTransport::activate()
{
  if (!model_ || !data_) {
    return false;
  }
  reset();
  mj_forward(model_.get(), data_.get());  // the first feedback's sensors and contacts
  held_ = hold_until_commanded_;
  active_ = true;
  ground_truth_.start(sim_node_->node());
  return true;
}

void MujocoActuatorTransport::deactivate()
{
  active_ = false;
  ground_truth_.stop();
}

transport::TransportCapabilities MujocoActuatorTransport::capabilities() const
{
  transport::TransportCapabilities caps{};
  std::strncpy(
    caps.implementation_name.data(), "MujocoActuatorTransport",
    caps.implementation_name.size() - 1);
  caps.transport_class = transport::TransportClass::kSimulated;
  caps.joint_count = joint_count();
  // Lockstep: one exchange() advances simulated time by exactly one period.
  caps.nominal_cycle_period_us = static_cast<std::uint32_t>(transport::kControlPeriod.count());
  caps.worst_case_exchange_us = caps.nominal_cycle_period_us;
  caps.supports_per_joint_disable = false;
  caps.supports_availability_mask = true;
  caps.provides_temperature = false;
  caps.provides_bus_voltage = false;
  caps.provides_imu = imu_.present();
  caps.is_deterministic = false;  // kSimulated, not kReplay
  return caps;
}

transport::ExchangeResult MujocoActuatorTransport::exchange(
  const transport::CommandBatch & command,
  transport::FeedbackBatch & feedback,
  transport::MonotonicStamp /*deadline*/) noexcept
{
  // The deadline is not enforced: a lockstep step cannot be abandoned halfway, and SIL timing is
  // not evidence (ADR-007-05).
  transport::ExchangeResult result{};
  const auto start = std::chrono::steady_clock::now();
  if (!active_) {
    result.error = transport::TransportError::kNotActive;
    return result;
  }

  // As on hardware, a batch for another manifest or with a non-finite field never reaches the
  // model: NaN in one actuator would spread through the whole simulation state.
  const auto commands = std::span{command.joints}.first(joint_count());
  if (command.joint_count != joint_count() ||
    !std::ranges::all_of(
      commands, [](const transport::JointCommand & joint) {return transport::is_finite(joint);}))
  {
    result.error = transport::TransportError::kManifestMismatch;
    return result;
  }

  // The first stiff command ends the hold for good, even if later commands go slack.
  held_ = held_ && std::ranges::none_of(
    commands, [](const transport::JointCommand & joint) {return joint.stiffness_nm_rad != 0.0;});
  if (!held_) {
    advance_physics(commands);
  }
  const auto end = std::chrono::steady_clock::now();
  publish_ground_truth(command.sequence, end);

  feedback.sequence = command.sequence;
  feedback.stamp = end;
  feedback.joint_count = joint_count();
  feedback.availability_mask = command.availability_mask;
  feedback.availability_epoch = command.availability_epoch;
  std::ranges::transform(
    drives_, feedback.joints.begin(),
    [this](const JointDrive & drive) {return drive.feedback(*data_);});
  // mj_step evaluates sensors at the start of its last substep, so the IMU lags the joints by one
  // physics step, as a real IMU lags by its latency.
  if (imu_.present()) {
    feedback.imu = read_imu(*data_, imu_);
  }

  result.error = transport::TransportError::kNone;
  result.joints_reported = joint_count();
  result.round_trip = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
  return result;
}

bool MujocoActuatorTransport::request_joint_disable(transport::JointIndex /*joint*/) noexcept
{
  // The SafetyKernel commands the protective response; there is nothing to disable here.
  return true;
}

bool MujocoActuatorTransport::request_all_disable() noexcept
{
  if (model_ && data_) {
    release_all();
  }
  return true;
}

transport::HealthSnapshot MujocoActuatorTransport::health_snapshot() const noexcept
{
  transport::HealthSnapshot snapshot{};
  snapshot.active = active_;
  return snapshot;
}

// ===========================================================================
// Private helpers
// ===========================================================================

std::optional<MujocoActuatorTransport::SimParameters>
MujocoActuatorTransport::declare_parameters()
{
  // A fresh node per configure(): a name cannot be declared twice, and a reconfigure must re-read
  // the overrides. The old node is destroyed first, so two "mujoco_sim" nodes never coexist.
  sim_node_.reset();
  sim_node_ = std::make_unique<SimNode>("mujoco_sim");
  const auto & node = sim_node_->node();

  // Read once, so read_only makes `ros2 param set` fail instead of appearing to work.
  const auto describe = [](const char * text) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description = text;
      descriptor.read_only = true;
      return descriptor;
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
      "disturbance.push.duration_s", defaults.duration_s, describe("Push duration, s"));
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
    std::ranges::copy(direction, params.push.direction.begin());
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    // Typically an integer literal in YAML (50) where a double (50.0) is expected.
    fail(std::format("parameter type mismatch: {}", e.what()));
    return std::nullopt;
  }

  if (params.mjcf_path.empty()) {
    fail("parameter 'mjcf_path' is not set");
    return std::nullopt;
  }
  sim_node_->spin();  // only once every parameter is declared
  return params;
}

bool MujocoActuatorTransport::load_model(const std::string & mjcf_path)
{
  std::array<char, 1024> error{};
  model_.reset(
    mj_loadXML(mjcf_path.c_str(), nullptr, error.data(), static_cast<int>(error.size())));
  if (!model_) {
    return fail(std::format("mj_loadXML failed: {}", error.data()));
  }
  data_.reset(mj_makeData(model_.get()));
  if (!data_) {
    return fail("mj_makeData failed");
  }
  return true;
}

void MujocoActuatorTransport::reset() noexcept
{
  reset_to(*model_, *data_, initial_pose_);
  release_all();
  if (root_qpos_adr_ >= 0) {
    data_->qpos[root_qpos_adr_ + 2] = root_z_;
  }
}

void MujocoActuatorTransport::release_all() noexcept
{
  for (const auto & drive : drives_) {
    drive.release(*model_, *data_);
  }
}

void MujocoActuatorTransport::advance_physics(
  std::span<const transport::JointCommand> commands) noexcept
{
  for (std::size_t i = 0; i < drives_.size(); ++i) {
    drives_[i].command(*model_, *data_, commands[i]);
  }
  disturbance_.apply(*data_);
  for (int step = 0; step < substeps_; ++step) {
    mj_step(model_.get(), data_.get());
  }
}

void MujocoActuatorTransport::publish_ground_truth(
  transport::CycleSequence sequence, transport::MonotonicStamp stamp) noexcept
{
  if (root_qpos_adr_ < 0) {
    return;  // a fixed base has no pose of its own
  }
  // A copy into a lock-free buffer; the TF publish happens on the publisher's thread (ADR-001).
  GroundTruthFrame frame{
    .valid = true, .sequence = sequence, .captured = stamp, .sim_time_s = data_->time};
  const mjtNum * root = data_->qpos + root_qpos_adr_;  // free joint: position, then wxyz
  std::copy_n(root, frame.position.size(), frame.position.begin());
  std::copy_n(
    root + frame.position.size(), frame.orientation_wxyz.size(), frame.orientation_wxyz.begin());
  ground_truth_.publish(frame);
}

}  // namespace humanoid::transport_mujoco

namespace
{

bool fail(std::string_view message)
{
  std::cerr << "MujocoActuatorTransport: " << message << "\n";
  return false;
}

template<typename T>
std::optional<T> value_or_fail(std::variant<T, std::string> result)
{
  if (const auto * error = std::get_if<std::string>(&result)) {
    fail(*error);
    return std::nullopt;
  }
  return std::get<T>(std::move(result));
}

// Physics must divide the control period exactly, or the two clocks beat (ADR-007-03).
std::variant<int, std::string> substeps_per_period(double timestep)
{
  constexpr double kPeriodS = std::chrono::duration<double>(humanoid::transport::kControlPeriod)
    .count();
  const double ratio = kPeriodS / timestep;
  const auto substeps = timestep > 0.0 ? std::lround(ratio) : 0L;
  if (substeps < 1 || std::abs(static_cast<double>(substeps) * timestep - kPeriodS) > 1e-9) {
    return std::format(
      "timestep {} s does not divide the {} s control period by an integer (ratio {})",
      timestep, kPeriodS, ratio);
  }
  return static_cast<int>(substeps);
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(
  humanoid::transport_mujoco::MujocoActuatorTransport,
  humanoid::transport::ActuatorTransport)
