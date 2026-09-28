// Copyright 2026 Your Organization Name

#include "humanoid_mit_controller/mit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "pluginlib/class_list_macros.hpp"

namespace humanoid::control
{

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

controller_interface::CallbackReturn MitImpedanceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_configure: 'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (joint_names_.size() > transport::kMaxJoints) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "on_configure: %zu joints exceeds kMaxJoints=%zu",
      joint_names_.size(), transport::kMaxJoints);
    return controller_interface::CallbackReturn::ERROR;
  }
  num_joints_ = joint_names_.size();

  auto_declare<double>("default_kp", 0.0);
  auto_declare<double>("default_kd", 0.0);
  default_kp_ = get_node()->get_parameter("default_kp").as_double();
  default_kd_ = get_node()->get_parameter("default_kd").as_double();

  // Non-RT subscription. Callback runs on executor thread, writes to SPSC buffer.
  ref_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "~/joint_references", rclcpp::QoS(1).best_effort(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      ReferenceFrame frame{};
      frame.valid = true;
      frame.sequence = ++ref_sequence_;
      frame.stamp = std::chrono::steady_clock::now();
      frame.joint_count = static_cast<std::uint8_t>(
        std::min(msg->name.size(), static_cast<std::size_t>(num_joints_)));
      for (std::size_t i = 0; i < frame.joint_count && i < num_joints_; ++i) {
        frame.joints[i].position_rad =
        (i < msg->position.size()) ? msg->position[i] : 0.0;
        frame.joints[i].velocity_rad_s =
        (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
        frame.joints[i].effort_nm =
        (i < msg->effort.size()) ? msg->effort[i] : 0.0;
        frame.joints[i].stiffness_nm_rad = default_kp_;
        frame.joints[i].damping_nm_s_rad = default_kd_;
      }
      reference_buffer_.publish(frame);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Verify we received the expected number of command interfaces.
  const std::size_t expected = num_joints_ * transport::kMitFields.size();
  if (command_interfaces_.size() != expected) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "on_activate: expected %zu command interfaces, got %zu",
      expected, command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Build the fallback reference from the current measured state.
  // Read the state interfaces to get current positions.
  // State interfaces are ordered as state_interface_configuration() lists them.
  fallback_reference_.joint_count = static_cast<std::uint8_t>(num_joints_);
  fallback_reference_.valid = true;
  fallback_reference_.sequence = 0U;
  fallback_reference_.stamp = std::chrono::steady_clock::now();

  for (std::size_t j = 0U; j < num_joints_; ++j) {
    auto & cmd = fallback_reference_.joints[j];
    const std::size_t state_idx = j * transport::kJointStateFields.size();
    if (state_idx < state_interfaces_.size()) {
      cmd.position_rad = state_interfaces_[state_idx].get_optional().value_or(0.0);
    } else {
      cmd.position_rad = 0.0;
    }
    cmd.velocity_rad_s = 0.0;
    cmd.effort_nm = 0.0;
    cmd.stiffness_nm_rad = 0.0;
    cmd.damping_nm_s_rad = 0.0;
  }
  have_fallback_ = true;

  // Reset the reference buffer so the first update uses the fallback.
  reference_buffer_.reset();
  cycle_count_ = 0U;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Command zero stiffness and damping to release the joints.
  // This is safe because the SafetyKernel and transport handle the
  // actual protective damping. We just stop commanding.
  reference_buffer_.reset();
  have_fallback_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitImpedanceController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_.clear();
  num_joints_ = 0U;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitImpedanceController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitImpedanceController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interface configuration
// ---------------------------------------------------------------------------

controller_interface::InterfaceConfiguration
MitImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(num_joints_ * transport::kMitFields.size());
  for (const auto & joint : joint_names_) {
    for (const auto & field : transport::kMitFields) {
      config.names.push_back(interface_name(joint, field.name));
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
MitImpedanceController::state_interface_configuration() const
{
  // Read position, velocity, effort for each joint.
  // These are used to build the fallback reference on activation.
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(num_joints_ * transport::kJointStateFields.size());
  for (const auto & joint : joint_names_) {
    for (const auto & field : transport::kJointStateFields) {
      config.names.push_back(interface_name(joint, field.name));
    }
  }
  return config;
}

// ---------------------------------------------------------------------------
// The 200 Hz loop
// ---------------------------------------------------------------------------

controller_interface::return_type MitImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto latest = reference_buffer_.read();

  if (!latest) {
    if (!have_fallback_) {
      // No reference and no fallback. Write zeros to prevent uncommanded motion.
      for (auto & interface : command_interfaces_) {
        if (!interface.set_value(0.0)) {
          return controller_interface::return_type::ERROR;
        }
      }
      return controller_interface::return_type::OK;
    }
  }
  // Bind, don't copy: the fallback is read in place rather than copied over
  // the latest frame (a 1.3 KB copy every cycle without a reference).
  const ReferenceFrame & ref = latest ? *latest : fallback_reference_;

  // Write all five fields for all joints from the reference.
  // Command interfaces are ordered: [joint0/pos, joint0/vel, ..., jointN/damping]
  for (std::size_t j = 0U; j < num_joints_; ++j) {
    const std::size_t base = j * transport::kMitFields.size();
    for (std::size_t f = 0U; f < transport::kMitFields.size(); ++f) {
      const double value = ref.joints[j].*transport::kMitFields[f].member;
      if (!command_interfaces_[base + f].set_value(value)) {
        return controller_interface::return_type::ERROR;
      }
    }
  }

  ++cycle_count_;
  return controller_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Non-real-time reference injection
// ---------------------------------------------------------------------------

void MitImpedanceController::publish_reference(
  const ReferenceFrame & frame) noexcept
{
  reference_buffer_.publish(frame);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

std::string MitImpedanceController::interface_name(
  const std::string & joint, std::string_view suffix)
{
  std::string s;
  s.reserve(joint.size() + 1U + suffix.size());
  s += joint;
  s += '/';
  s += suffix;
  return s;
}

}  // namespace humanoid::control

PLUGINLIB_EXPORT_CLASS(
  humanoid::control::MitImpedanceController,
  controller_interface::ControllerInterface)
