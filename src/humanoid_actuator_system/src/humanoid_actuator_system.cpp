// Copyright 2026 Humanoid Robotics Team

#include "humanoid_actuator_system/humanoid_actuator_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace humanoid::actuator_system
{

namespace
{
constexpr char kLogName[] = "HumanoidActuatorSystem";
constexpr char kTransportPluginParam[] = "transport_plugin";
constexpr char kTransportPluginDefault[] =
  "humanoid_transport_mujoco/MujocoActuatorTransport";
constexpr char kSafetyManifestParam[] = "safety_manifest_path";
constexpr char kSafetyManifestPackageParam[] = "safety_manifest_package";
constexpr char kAcceptedDegradationParam[] = "accepted_degradation";

// Parse joint names from the ros2_control URDF <joint> tags.
// The framework provides them via info_.joints.
std::vector<std::string> joint_names_from_info(
  const hardware_interface::HardwareInfo & info)
{
  std::vector<std::string> names;
  names.reserve(info.joints.size());
  for (const auto & j : info.joints) {
    names.push_back(j.name);
  }
  return names;
}

bool is_finite(const transport::JointCommand & c)
{
  return std::isfinite(c.position_rad) && std::isfinite(c.velocity_rad_s) &&
         std::isfinite(c.effort_nm) && std::isfinite(c.stiffness_nm_rad) &&
         std::isfinite(c.damping_nm_s_rad);
}
}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  info_ = params.hardware_info;
  joint_names_ = joint_names_from_info(info_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger(kLogName);

  // 1. Read parameters
  std::string transport_plugin = kTransportPluginDefault;
  std::string safety_manifest_rel_path = "config/safety_manifest.yaml";
  std::string safety_manifest_package = "humanoid_bringup";
  std::string degradation_str;

  if (info_.hardware_parameters.count(kTransportPluginParam)) {
    transport_plugin = info_.hardware_parameters.at(kTransportPluginParam);
  }
  if (info_.hardware_parameters.count(kSafetyManifestParam)) {
    safety_manifest_rel_path = info_.hardware_parameters.at(kSafetyManifestParam);
  }
  if (info_.hardware_parameters.count(kSafetyManifestPackageParam)) {
    safety_manifest_package = info_.hardware_parameters.at(kSafetyManifestPackageParam);
  }
  if (info_.hardware_parameters.count(kAcceptedDegradationParam)) {
    degradation_str = info_.hardware_parameters.at(kAcceptedDegradationParam);
  }

  // Parse accepted degradation
  if (degradation_str == "position_velocity_only") {
    accepted_degradation_ = transport::TupleCompleteness::kPositionVelocityOnly;
  } else {
    accepted_degradation_ = transport::TupleCompleteness::kFull;
  }

  // 2. Build joint manifest from URDF info
  joint_names_ = joint_names_from_info(info_);
  if (joint_names_.empty()) {
    RCLCPP_ERROR(logger, "No joints declared in ros2_control config");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (joint_names_.size() > transport::kMaxJoints) {
    RCLCPP_ERROR(logger, "Joint count %zu exceeds kMaxJoints=%zu",
      joint_names_.size(), transport::kMaxJoints);
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_manifest_.joint_count = static_cast<std::uint8_t>(joint_names_.size());
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    std::snprintf(
      joint_manifest_.joints[i].name.data(),
      joint_manifest_.joints[i].name.size(),
      "%s",
      joint_names_[i].c_str());
  }

  // 3. Build safety manifest
  std::string safety_manifest_path;
  if (!safety_manifest_rel_path.empty() && safety_manifest_rel_path[0] == '/') {
    safety_manifest_path = safety_manifest_rel_path;
  } else {
    std::string package_share_dir;
    try {
      package_share_dir = ament_index_cpp::get_package_share_directory(safety_manifest_package);
    } catch (const ament_index_cpp::PackageNotFoundError & e) {
      RCLCPP_ERROR(logger, "Package '%s' not found: %s", safety_manifest_package.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    safety_manifest_path = package_share_dir + "/" + safety_manifest_rel_path;
  }
  try {
    const YAML::Node manifest = YAML::LoadFile(safety_manifest_path);
    const auto manifest_joint_count = manifest["joint_count"].as<std::size_t>();
    if (manifest_joint_count != joint_manifest_.joint_count) {
      RCLCPP_ERROR(logger, "Safety manifest joint_count=%zu, URDF joint_count=%u",
        manifest_joint_count, joint_manifest_.joint_count);
      return hardware_interface::CallbackReturn::ERROR;
    }

    safety_manifest_.joint_count = joint_manifest_.joint_count;
    safety_manifest_.max_consecutive_bad_cycles =
      manifest["max_consecutive_bad_cycles"].as<std::uint8_t>();
    safety_manifest_.feedback_max_age_us = manifest["feedback_max_age_us"].as<std::uint32_t>();
    const YAML::Node envelopes = manifest["envelopes"];
    for (std::uint8_t i = 0; i < safety_manifest_.joint_count; ++i) {
      const YAML::Node node = envelopes[joint_names_[i]];
      if (!node) {
        RCLCPP_ERROR(logger, "Safety manifest has no envelope for '%s'", joint_names_[i].c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      auto & env = safety_manifest_.envelopes[i];
      env.position_min_rad = node["position_min_rad"].as<double>();
      env.position_max_rad = node["position_max_rad"].as<double>();
      env.velocity_max_rad_s = node["velocity_max_rad_s"].as<double>();
      env.torque_continuous_nm = node["torque_continuous_nm"].as<double>();
      env.torque_peak_nm = node["torque_peak_nm"].as<double>();
      env.torque_peak_duration_s = node["torque_peak_duration_s"].as<double>();
      env.stiffness_max_nm_rad = node["stiffness_max_nm_rad"].as<double>();
      env.damping_max_nm_s_rad = node["damping_max_nm_s_rad"].as<double>();
      env.torque_slew_max_nm_s = node["torque_slew_max_nm_s"].as<double>();
      env.power_max_w = node["power_max_w"].as<double>();
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(logger, "Failed to load safety manifest '%s': %s", safety_manifest_path.c_str(),
        e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 4. Configure safety kernel
  if (!safety_kernel_.configure(joint_manifest_, safety_manifest_)) {
    RCLCPP_ERROR(logger, "SafetyKernel configuration failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 5. Load transport via pluginlib
  try {
    loader_ = std::make_unique<pluginlib::ClassLoader<transport::ActuatorTransport>>(
      "humanoid_transport", "humanoid::transport::ActuatorTransport");
    transport_ = loader_->createUniqueInstance(transport_plugin);
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_ERROR(logger, "Failed to load transport '%s': %s",
      transport_plugin.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 6. Configure transport
  if (!transport_->configure(joint_manifest_, safety_manifest_)) {
    RCLCPP_ERROR(logger, "Transport configure() failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 7. Check tuple capability
  if (!check_tuple_capability()) {
    RCLCPP_ERROR(logger,
      "Tuple capability mismatch: controller claims 5 fields but transport "
      "cannot deliver them and degradation was not accepted");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 8. Preallocate batches
  command_snapshot_ = transport::CommandBatch{};
  feedback_ = transport::FeedbackBatch{};
  command_snapshot_.joint_count = joint_manifest_.joint_count;
  feedback_.joint_count = joint_manifest_.joint_count;

  RCLCPP_INFO(logger, "Configured: %u joints, transport=%s",
    joint_manifest_.joint_count, transport_plugin.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger(kLogName);

  if (!transport_->activate()) {
    RCLCPP_ERROR(logger, "Transport activate() failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  cycle_ = 0;
  consecutive_bad_cycles_ = 0;
  mit_tuple_claimed_ = false;
  protective_state_ = false;

  // Zero out command snapshot
  for (std::uint8_t i = 0; i < joint_manifest_.joint_count; ++i) {
    command_snapshot_.joints[i] = transport::JointCommand{};
    feedback_.joints[i] = transport::JointFeedback{};
  }

  RCLCPP_INFO(logger, "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  enter_protective_state(safety::Trigger::kOperatorStop);
  if (transport_) {
    transport_->deactivate();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  transport_.reset();
  loader_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  enter_protective_state(safety::Trigger::kTransportError);
  if (transport_) {
    transport_->deactivate();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  enter_protective_state(safety::Trigger::kOperatorStop);
  if (transport_) {
    transport_->deactivate();
  }
  transport_.reset();
  loader_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interface export
// ---------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
HumanoidActuatorSystem::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> interfaces;
  // 3 state interfaces per joint: position, velocity, effort
  interfaces.reserve(joint_names_.size() * 3);

  state_storage_.resize(joint_names_.size() * 3, 0.0);

  for (std::size_t j = 0; j < joint_names_.size(); ++j) {
    interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
      joint_names_[j], "position", &state_storage_[j * 3 + 0]));
    interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
      joint_names_[j], "velocity", &state_storage_[j * 3 + 1]));
    interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
      joint_names_[j], "effort", &state_storage_[j * 3 + 2]));
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
HumanoidActuatorSystem::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
  // 5 command interfaces per joint
  interfaces.reserve(joint_names_.size() * 5);

  command_storage_.resize(joint_names_.size() * 5, 0.0);

  for (std::size_t j = 0; j < joint_names_.size(); ++j) {
    interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
      joint_names_[j], "position", &command_storage_[j * 5 + 0]));
    interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
      joint_names_[j], "velocity", &command_storage_[j * 5 + 1]));
    interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
      joint_names_[j], "effort", &command_storage_[j * 5 + 2]));
    interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
      joint_names_[j], "stiffness", &command_storage_[j * 5 + 3]));
    interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
      joint_names_[j], "damping", &command_storage_[j * 5 + 4]));
  }
  return interfaces;
}

// ---------------------------------------------------------------------------
// Mode switching
// ---------------------------------------------------------------------------

hardware_interface::return_type HumanoidActuatorSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // ADR-001: all-or-none. Count how many of our interfaces are being started/stopped.
  std::size_t our_start = 0;
  std::size_t our_stop = 0;
  const std::size_t total = joint_names_.size() * 5;

  for (const auto & name : start_interfaces) {
    // Check if any of our command interfaces match
    for (const auto & joint : joint_names_) {
      if (name.find(joint + "/") == 0) {
        ++our_start;
        break;
      }
    }
  }
  for (const auto & name : stop_interfaces) {
    for (const auto & joint : joint_names_) {
      if (name.find(joint + "/") == 0) {
        ++our_stop;
        break;
      }
    }
  }

  // Reject partial claims
  if (our_start > 0 && our_start < total) {
    return hardware_interface::return_type::ERROR;
  }
  if (our_stop > 0 && our_stop < total) {
    return hardware_interface::return_type::ERROR;
  }

  if (our_start == total) {
    mit_tuple_claimed_ = true;
  }
  if (our_stop == total) {
    mit_tuple_claimed_ = false;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HumanoidActuatorSystem::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  // Decision already made in prepare. Nothing to do here.
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Real-time cycle
// ---------------------------------------------------------------------------

hardware_interface::return_type HumanoidActuatorSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!transport_) {
    return hardware_interface::return_type::ERROR;
  }

  // Copy feedback stored by the previous write() into state interfaces.
  // No transport call here. The single exchange() happens in write().
  //
  // On the first cycle after activation, feedback_ is zeroed, which is
  // acceptable: the controller sees zero state and uses its fallback
  // reference until the first write() produces real feedback.

  if (!accept_feedback(feedback_)) {
    ++consecutive_bad_cycles_;
    if (consecutive_bad_cycles_ >= safety_manifest_.max_consecutive_bad_cycles) {
      enter_protective_state(safety::Trigger::kSequenceRejected);
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  consecutive_bad_cycles_ = 0;

  // Write feedback to state interfaces
  for (std::uint8_t i = 0; i < joint_manifest_.joint_count; ++i) {
    state_storage_[i * 3 + 0] = feedback_.joints[i].position_rad;
    state_storage_[i * 3 + 1] = feedback_.joints[i].velocity_rad_s;
    state_storage_[i * 3 + 2] = feedback_.joints[i].effort_nm;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HumanoidActuatorSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!transport_) {
    return hardware_interface::return_type::ERROR;
  }

  if (protective_state_) {
    // In protective state, send damping command
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5);
    transport_->exchange(command_snapshot_, feedback_, deadline);
    return hardware_interface::return_type::OK;
  }

  // 1. Snapshot commands from interface storage
  snapshot_commands();

  // 2. Validate finiteness
  for (std::uint8_t i = 0; i < joint_manifest_.joint_count; ++i) {
    if (!is_finite(command_snapshot_.joints[i])) {
      enter_protective_state(safety::Trigger::kNonFiniteCommand);
      return hardware_interface::return_type::ERROR;
    }
  }

  // 3. Safety kernel projection
  auto now = std::chrono::steady_clock::now();
  auto verdict = safety_kernel_.project(feedback_, command_snapshot_, now);
  if (verdict.protective_state_required) {
    enter_protective_state(verdict.trigger);
    return hardware_interface::return_type::ERROR;
  }

  // 4. Single exchange with transport. This is the ONLY call to exchange()
  //    per cycle. In SIL it applies torque, steps physics, and returns
  //    the resulting state. In production it sends the command and reads
  //    hardware feedback.
  command_snapshot_.sequence = cycle_;
  auto deadline = now + std::chrono::milliseconds(5);
  auto result = transport_->exchange(command_snapshot_, feedback_, deadline);

  if (!result.ok()) {
    ++consecutive_bad_cycles_;
    if (consecutive_bad_cycles_ >= safety_manifest_.max_consecutive_bad_cycles) {
      enter_protective_state(safety::Trigger::kTransportError);
      return hardware_interface::return_type::ERROR;
    }
  } else {
    consecutive_bad_cycles_ = 0;
  }

  ++cycle_;
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void HumanoidActuatorSystem::snapshot_commands() noexcept
{
  command_snapshot_.joint_count = joint_manifest_.joint_count;
  command_snapshot_.sequence = cycle_;
  command_snapshot_.stamp = std::chrono::steady_clock::now();

  for (std::uint8_t i = 0; i < joint_manifest_.joint_count; ++i) {
    command_snapshot_.joints[i].position_rad = command_storage_[i * 5 + 0];
    command_snapshot_.joints[i].velocity_rad_s = command_storage_[i * 5 + 1];
    command_snapshot_.joints[i].effort_nm = command_storage_[i * 5 + 2];
    command_snapshot_.joints[i].stiffness_nm_rad = command_storage_[i * 5 + 3];
    command_snapshot_.joints[i].damping_nm_s_rad = command_storage_[i * 5 + 4];
  }
}

bool HumanoidActuatorSystem::accept_feedback(
  const transport::FeedbackBatch & candidate) noexcept
{
  // Reject wrong joint count.
  if (candidate.joint_count != joint_manifest_.joint_count) {
    return false;
  }

  // Reject if all joints report not-fresh (no exchange has happened yet).
  bool any_fresh = false;
  for (std::uint8_t i = 0; i < candidate.joint_count; ++i) {
    if (candidate.joints[i].fresh) {
      any_fresh = true;
      break;
    }
  }
  if (!any_fresh) {
    return false;
  }

  return true;
}

void HumanoidActuatorSystem::enter_protective_state(
  safety::Trigger trigger) noexcept
{
  safety_kernel_.enter_protective(trigger, command_snapshot_);
  protective_state_ = true;
}

bool HumanoidActuatorSystem::check_tuple_capability() noexcept
{
  if (!transport_) {
    return false;
  }
  const auto caps = transport_->capabilities();
  if (mit_tuple_claimed_ &&
    caps.tuple_completeness == transport::TupleCompleteness::kPositionVelocityOnly &&
    accepted_degradation_ != transport::TupleCompleteness::kPositionVelocityOnly)
  {
    return false;
  }
  return true;
}

}  // namespace humanoid::actuator_system

PLUGINLIB_EXPORT_CLASS(
  humanoid::actuator_system::HumanoidActuatorSystem,
  hardware_interface::SystemInterface)
