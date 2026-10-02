// Copyright 2026 Humanoid Robotics Team

#include "humanoid_actuator_system/humanoid_actuator_system.hpp"

#include <format>

#include <algorithm>
#include <chrono>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "humanoid_actuator_system/feedback_validation.hpp"
#include "humanoid_actuator_system/hardware_contract.hpp"
#include "humanoid_actuator_system/safety_manifest_parser.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "humanoid_transport/timing.hpp"
#include "yaml-cpp/yaml.h"

namespace humanoid::actuator_system
{

namespace
{
// ROS parameters of the component's own node, which ros2_control names after the description's
// <ros2_control name>, lower-cased. They choose the deployment, so they have no defaults.
constexpr char kTransportPluginParam[] = "transport_plugin";
constexpr char kAcceptedDegradationParam[] = "accepted_degradation";
constexpr std::string_view kNoDegradation = "none";
constexpr std::string_view kPositionVelocityOnly = "position_velocity_only";

// File-local helpers. Declared here, defined at the end of this file.
std::vector<std::string> joint_names_from_info(const hardware_interface::HardwareInfo & info);
std::optional<transport::TupleCompleteness> accepted_degradation_from(std::string_view value);
}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  info_ = params.hardware_info;
  if (const auto violation = hardware_contract_violation(info_)) {
    return fail(std::format("Robot description: {}", *violation));
  }
  joint_names_ = joint_names_from_info(info_);

  auto parameters = read_parameters();
  if (const auto * error = std::get_if<std::string>(&parameters)) {
    return fail(*error);
  }
  parameters_ = std::move(std::get<Parameters>(parameters));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // ADR-001-03: the rate is an invariant, so a mismatched controller_manager
  // update_rate is a configuration error, not a different operating point.
  if (info_.rw_rate != transport::kControlRateHz) {
    return fail(std::format(
      "Component read/write rate is {} Hz, but the control cycle is fixed at {} Hz "
      "(ADR-001-03). Set controller_manager update_rate to {}.",
      info_.rw_rate, transport::kControlRateHz, transport::kControlRateHz));
  }

  // 1. Build joint manifest from URDF info. hardware_contract_violation() has bounded the joint
  //    count and each name's length, so every name fits its zeroed entry with the terminator.
  const auto joint_count = static_cast<std::uint8_t>(joint_names_.size());
  joint_manifest_ = transport::JointManifest{};
  joint_manifest_.joint_count = joint_count;
  for (std::size_t i = 0; i < joint_count; ++i) {
    std::ranges::copy(joint_names_[i], joint_manifest_.joints[i].name.begin());
  }

  // 2. Load the safety manifest
  YAML::Node manifest_yaml;
  try {
    manifest_yaml = YAML::LoadFile(parameters_.safety_manifest_path);
  } catch (const YAML::Exception & e) {
    return fail(std::format(
      "Failed to load safety manifest '{}': {}", parameters_.safety_manifest_path, e.what()));
  }
  auto parsed = parse_safety_manifest(manifest_yaml, joint_names_);
  if (const auto * error = std::get_if<std::string>(&parsed)) {
    return fail(std::format("Safety manifest '{}': {}", parameters_.safety_manifest_path, *error));
  }
  safety_manifest_ = std::get<transport::SafetyManifest>(parsed);

  // 3. Configure safety kernel
  if (!safety_kernel_.configure(joint_manifest_, safety_manifest_)) {
    return fail("SafetyKernel configuration failed");
  }

  // 4. Load transport via pluginlib
  try {
    loader_ = std::make_unique<pluginlib::ClassLoader<transport::ActuatorTransport>>(
      "humanoid_transport", "humanoid::transport::ActuatorTransport");
    transport_ = loader_->createUniqueInstance(parameters_.transport_plugin);
  } catch (const pluginlib::PluginlibException & e) {
    return fail(std::format(
      "Failed to load transport '{}': {}", parameters_.transport_plugin, e.what()));
  }

  // 5. Configure transport
  if (!transport_->configure(joint_manifest_, safety_manifest_)) {
    return fail("Transport configure() failed");
  }

  // 6. The transport must actually exchange at the control rate.
  const auto caps = transport_->capabilities();
  if (caps.nominal_cycle_period_us != transport::kControlPeriod.count()) {
    return fail(std::format(
      "Transport cycle period is {} us, but the control period is {} us",
      caps.nominal_cycle_period_us, transport::kControlPeriod.count()));
  }

  // 7. Check tuple capability
  if (!check_tuple_capability()) {
    return fail(
      "Tuple capability mismatch: the transport cannot deliver the five-field MIT tuple, the only "
      "claim a controller may make, and degradation was not accepted");
  }

  // 8. Preallocate batches
  command_snapshot_ = transport::CommandBatch{};
  feedback_ = transport::FeedbackBatch{};
  command_snapshot_.joint_count = joint_count;
  feedback_.joint_count = joint_count;

  // 9. No claim survives UNCONFIGURED: its interfaces were withdrawn, and a controller must be
  //    switched in again, which perform_command_mode_switch() records.
  owned_joints_.store(0U, std::memory_order_release);

  RCLCPP_INFO(get_logger(), "Configured: %u joints at %u Hz, transport=%s",
    joint_count, info_.rw_rate, parameters_.transport_plugin.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!transport_->activate()) {
    return fail("Transport activate() failed");
  }

  cycle_ = 0;
  stats_ = transport::ExchangeStats{};
  last_sent_sequence_.reset();
  last_exchange_error_ = transport::TransportError::kNone;
  consecutive_bad_cycles_ = 0;
  // Claims persist across INACTIVE, so owned_joints_ is kept. A controller that still owns every
  // tuple regains authority after one damping cycle.
  authority_ = CommandAuthority::kAwaitingClaim;

  command_snapshot_.joints.fill(transport::JointCommand{});
  feedback_.joints.fill(transport::JointFeedback{});

  RCLCPP_INFO(get_logger(), "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  enter_protective_state(safety::Trigger::kOperatorStop);
  if (transport_) {
    transport_->deactivate();
  }
  log_exchange_stats();
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
  const rclcpp_lifecycle::State & previous_state)
{
  enter_protective_state(safety::Trigger::kOperatorStop);
  if (transport_) {
    transport_->deactivate();
  }
  // From INACTIVE, on_deactivate() has already reported this run.
  if (previous_state.label() == hardware_interface::lifecycle_state_names::ACTIVE) {
    log_exchange_stats();
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
  constexpr auto & kFields = transport::kJointStateFields;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> interfaces;
  interfaces.reserve(joint_names_.size() * kFields.size());

  state_storage_.resize(joint_names_.size() * kFields.size(), 0.0);

  for (std::size_t j = 0; j < joint_names_.size(); ++j) {
    for (std::size_t f = 0; f < kFields.size(); ++f) {
      interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
        joint_names_[j], std::string{kFields[f].name}, &state_storage_[j * kFields.size() + f]));
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
HumanoidActuatorSystem::on_export_command_interfaces()
{
  constexpr auto & kFields = transport::kMitFields;
  std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
  interfaces.reserve(joint_names_.size() * kFields.size());

  command_storage_.resize(joint_names_.size() * kFields.size(), 0.0);

  for (std::size_t j = 0; j < joint_names_.size(); ++j) {
    for (std::size_t f = 0; f < kFields.size(); ++f) {
      interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
        joint_names_[j], std::string{kFields[f].name}, &command_storage_[j * kFields.size() + f]));
    }
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
  // No logging: this can run on the real-time thread. The resource manager reports the rejection.
  return tuple_claims(start_interfaces, stop_interfaces, joint_names_) ?
         hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type HumanoidActuatorSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Parsed again rather than cached from prepare: a prepared switch can be abandoned, and the
  // real-time error path calls prepare and perform back to back with lists of its own.
  const auto claims = tuple_claims(start_interfaces, stop_interfaces, joint_names_);
  if (!claims) {
    return hardware_interface::return_type::ERROR;
  }
  JointMask owned = owned_joints_.load(std::memory_order_relaxed);
  while (!owned_joints_.compare_exchange_weak(
      owned, apply_claims(owned, *claims), std::memory_order_acq_rel, std::memory_order_relaxed))
  {
  }
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

  // The single judge of each cycle (ADR-002 command-loss counting): feedback that does not answer
  // the last command -- including one left stale by a failed exchange -- is a bad cycle.
  if (!answers_command(feedback_, joint_manifest_.joint_count, last_sent_sequence_)) {
    ++consecutive_bad_cycles_;
    if (consecutive_bad_cycles_ >= safety_manifest_.max_consecutive_bad_cycles) {
      enter_protective_state(
        last_exchange_error_ != transport::TransportError::kNone ?
        safety::Trigger::kTransportError : safety::Trigger::kSequenceRejected);
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  consecutive_bad_cycles_ = 0;

  // Write feedback to state interfaces
  constexpr auto & kFields = transport::kJointStateFields;
  for (std::size_t i = 0; i < joint_manifest_.joint_count; ++i) {
    for (std::size_t f = 0; f < kFields.size(); ++f) {
      state_storage_[i * kFields.size() + f] = feedback_.joints[i].*kFields[f].member;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HumanoidActuatorSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!transport_) {
    return hardware_interface::return_type::ERROR;
  }

  const bool all_owned = owned_joints_.load(std::memory_order_acquire) ==
    all_joints(joint_manifest_.joint_count);
  const CommandAuthority previous = authority_;
  authority_ = next_authority(previous, all_owned);
  if (previous == CommandAuthority::kController && authority_ == CommandAuthority::kProtective) {
    // ADR-002: the owner released the tuple, so nothing commands the robot. That is command loss;
    // holding the last command indefinitely is prohibited.
    enter_protective_state(safety::Trigger::kCommandLoss);
  }

  if (authority_ == CommandAuthority::kProtective) {
    // In protective state, send damping command
    const auto deadline = std::chrono::steady_clock::now() + transport::kControlPeriod;
    static_cast<void>(exchange_once(deadline));
    return hardware_interface::return_type::OK;
  }

  // 1. Snapshot commands from interface storage
  snapshot_commands();
  auto now = std::chrono::steady_clock::now();

  if (authority_ == CommandAuthority::kController) {
    // 2. Validate finiteness
    for (std::uint8_t i = 0; i < joint_manifest_.joint_count; ++i) {
      if (!transport::is_finite(command_snapshot_.joints[i])) {
        enter_protective_state(safety::Trigger::kNonFiniteCommand);
        return hardware_interface::return_type::ERROR;
      }
    }

    // 3. Safety kernel projection
    auto verdict = safety_kernel_.project(feedback_, command_snapshot_, now);
    if (verdict.protective_state_required) {
      enter_protective_state(verdict.trigger);
      return hardware_interface::return_type::ERROR;
    }
  } else {
    // No controller has written every tuple yet: send damping, not what the storage last held.
    safety_kernel_.apply_damping(command_snapshot_);
  }

  // 4. Single exchange with transport. This is the ONLY call to exchange()
  //    per cycle. In SIL it applies torque, steps physics, and returns
  //    the resulting state. In production it sends the command and reads
  //    hardware feedback.
  command_snapshot_.sequence = cycle_;
  const auto deadline = now + transport::kControlPeriod;
  // A failed exchange is not counted here: it leaves feedback_ unanswered, and the next read()
  // counts it. Counting in both places made every failure count twice.
  static_cast<void>(exchange_once(deadline));

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

  constexpr auto & kFields = transport::kMitFields;
  for (std::size_t i = 0; i < joint_manifest_.joint_count; ++i) {
    for (std::size_t f = 0; f < kFields.size(); ++f) {
      command_snapshot_.joints[i].*kFields[f].member = command_storage_[i * kFields.size() + f];
    }
  }
}

void HumanoidActuatorSystem::log_exchange_stats() const
{
  // Until the ADR-008 in-cycle record exists, this is where a run's exchange accounting becomes
  // visible.
  const std::string summary = std::format(
    "Exchange stats: attempted={} failed={} deadline_misses={} worst_exchange={:.1f} us "
    "last_sequence={}",
    stats_.attempted, stats_.failed, stats_.deadline_misses,
    std::chrono::duration<double, std::micro>(stats_.worst_exchange).count(),
    stats_.last_sequence);
  RCLCPP_INFO(get_logger(), "%s", summary.c_str());
}

transport::ExchangeResult HumanoidActuatorSystem::exchange_once(
  transport::MonotonicStamp deadline) noexcept
{
  const auto start = std::chrono::steady_clock::now();
  last_sent_sequence_ = command_snapshot_.sequence;
  const auto result = transport_->exchange(command_snapshot_, feedback_, deadline);
  const auto end = std::chrono::steady_clock::now();
  last_exchange_error_ = result.error;
  stats_.record(
    command_snapshot_.sequence, result,
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - start), end > deadline);
  return result;
}

void HumanoidActuatorSystem::enter_protective_state(
  safety::Trigger trigger) noexcept
{
  safety_kernel_.enter_protective(trigger, command_snapshot_);
  authority_ = CommandAuthority::kProtective;
}

HumanoidActuatorSystem::ParametersOrError HumanoidActuatorSystem::read_parameters() const
{
  const auto node = get_node();
  if (!node) {
    return std::string{"ros2_control created no node for this component; cannot declare its "
      "parameters"};
  }
  // Read once. Nothing re-reads them, so read_only makes `ros2 param set` fail instead of
  // appearing to work.
  const auto describe = [](const char * text) {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = text;
      d.read_only = true;
      return d;
    };

  Parameters parameters;
  std::string degradation;
  try {
    parameters.transport_plugin = node->declare_parameter<std::string>(
      kTransportPluginParam,
      describe("ActuatorTransport plugin, e.g. humanoid_transport_mujoco/MujocoActuatorTransport"));
    degradation = node->declare_parameter<std::string>(
      kAcceptedDegradationParam,
      describe("Tuple degradation accepted from the transport: 'none', or "
      "'position_velocity_only' to run without stiffness, damping, and feed-forward torque"));
  } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException & e) {
    return std::format(
      "{}. Set it under '{}' in the controller_manager parameter file.", e.what(),
      node->get_fully_qualified_name());
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    return std::format("Parameter type mismatch: {}", e.what());
  }
  if (parameters.transport_plugin.empty()) {
    return std::format("Parameter '{}' is empty", kTransportPluginParam);
  }
  const auto accepted = accepted_degradation_from(degradation);
  if (!accepted) {
    return std::format(
      "Parameter '{}' is '{}'; expected '{}' or '{}'", kAcceptedDegradationParam, degradation,
      kNoDegradation, kPositionVelocityOnly);
  }
  parameters.accepted_degradation = *accepted;

  // The manifest location is part of the generated description; hardware_contract_violation()
  // has already required both entries.
  const auto & path = info_.hardware_parameters.at(std::string{kSafetyManifestPathParam});
  if (path.front() == '/') {
    parameters.safety_manifest_path = path;
  } else {
    const auto & package = info_.hardware_parameters.at(std::string{kSafetyManifestPackageParam});
    try {
      parameters.safety_manifest_path =
        ament_index_cpp::get_package_share_directory(package) + "/" + path;
    } catch (const ament_index_cpp::PackageNotFoundError & e) {
      return std::format("Package '{}' not found: {}", package, e.what());
    }
  }
  return parameters;
}

hardware_interface::CallbackReturn HumanoidActuatorSystem::fail(std::string_view reason) const
{
  RCLCPP_ERROR(get_logger(), "%.*s", static_cast<int>(reason.size()), reason.data());
  return hardware_interface::CallbackReturn::ERROR;
}

bool HumanoidActuatorSystem::check_tuple_capability() noexcept
{
  if (!transport_) {
    return false;
  }
  const auto caps = transport_->capabilities();
  if (caps.tuple_completeness == transport::TupleCompleteness::kPositionVelocityOnly &&
    parameters_.accepted_degradation != transport::TupleCompleteness::kPositionVelocityOnly)
  {
    return false;
  }
  return true;
}

namespace
{

// Parse joint names from the ros2_control URDF <joint> tags.
// The framework provides them via info_.joints.
std::vector<std::string> joint_names_from_info(
  const hardware_interface::HardwareInfo & info)
{
  std::vector<std::string> names;
  names.reserve(info.joints.size());
  for (const auto & joint : info.joints) {
    names.push_back(joint.name);
  }
  return names;
}

std::optional<transport::TupleCompleteness> accepted_degradation_from(std::string_view value)
{
  if (value == kNoDegradation) {
    return transport::TupleCompleteness::kFull;
  }
  if (value == kPositionVelocityOnly) {
    return transport::TupleCompleteness::kPositionVelocityOnly;
  }
  return std::nullopt;
}

}  // namespace

}  // namespace humanoid::actuator_system

PLUGINLIB_EXPORT_CLASS(
  humanoid::actuator_system::HumanoidActuatorSystem,
  hardware_interface::SystemInterface)
