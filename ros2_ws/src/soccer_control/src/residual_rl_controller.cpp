#include "soccer_control/residual_rl_controller.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace soccer_control
{

controller_interface::CallbackReturn ResidualRLController::on_init()
{
  // Declare the parameters wired up in soccer_description/config/controllers.yaml.
  auto_declare<std::string>("joint", "neck_pan");
  auto_declare<std::string>("reference_topic", "control/mpc_reference");
  auto_declare<std::string>("policy_path", "");
  auto_declare<double>("residual_limit_rad", 0.20);
  auto_declare<double>("effort_limit", 3.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ResidualRLController::command_interface_configuration() const
{
  // Claim the position + effort command interfaces of the one joint.
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {joint_ + "/" + hardware_interface::HW_IF_POSITION,
     joint_ + "/" + hardware_interface::HW_IF_EFFORT}};
}

controller_interface::InterfaceConfiguration
ResidualRLController::state_interface_configuration() const
{
  // Read back position, velocity, and measured effort (τ from current sense).
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {joint_ + "/" + hardware_interface::HW_IF_POSITION,
     joint_ + "/" + hardware_interface::HW_IF_VELOCITY,
     joint_ + "/" + hardware_interface::HW_IF_EFFORT}};
}

controller_interface::CallbackReturn ResidualRLController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_ = get_node()->get_parameter("joint").as_string();
  reference_topic_ = get_node()->get_parameter("reference_topic").as_string();
  residual_limit_ = get_node()->get_parameter("residual_limit_rad").as_double();
  effort_limit_ = get_node()->get_parameter("effort_limit").as_double();

  const std::string policy_path = get_node()->get_parameter("policy_path").as_string();
  if (policy_.load(policy_path)) {
    RCLCPP_INFO(get_node()->get_logger(), "Loaded RL policy: %s", policy_path.c_str());
  } else {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "No RL policy — running PURE MPC (zero residual). Train + export from sim/.");
  }

  // Seed the buffer so update() before the first MPC message holds still.
  mpc_ref_.writeFromNonRT(std::array<double, 2>{0.0, 0.0});
  ref_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    reference_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (msg->data.size() >= 2) {
        mpc_ref_.writeFromNonRT(std::array<double, 2>{msg->data[0], msg->data[1]});
      }
    });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ResidualRLController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ResidualRLController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ResidualRLController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1. Measured state. state_interfaces_[2] is measured effort (τ), part of the
  //    obs vector. A nullopt means the handle stayed contended for all retries.
  const auto q_opt = state_interfaces_[0].get_optional();
  const auto qd_opt = state_interfaces_[1].get_optional();
  const auto tau_opt = state_interfaces_[2].get_optional();
  if (!q_opt || !qd_opt || !tau_opt) {
    // Holding the previous command beats commanding on a NaN reading.
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Joint '%s': state interface unreadable this cycle; holding last command.",
      joint_.c_str());
    return controller_interface::return_type::OK;
  }
  const double q = *q_opt;
  const double qd = *qd_opt;

  // 2. Latest MPC reference (q*, qd*).
  const auto ref = *mpc_ref_.readFromRT();
  const double q_ref = ref[0];
  const double qd_ref = ref[1];

  // 3. Bounded RL residual. obs MUST match the Isaac Lab task layout.
  const std::vector<float> obs = {
    static_cast<float>(q), static_cast<float>(qd),
    static_cast<float>(q_ref), static_cast<float>(qd_ref),
    static_cast<float>(*tau_opt), 0.0f};
  double delta = policy_.residual(obs);
  delta = std::clamp(delta, -residual_limit_, residual_limit_);  // safety clamp

  // 4. Write q_target = q* + Δq. The position command is what the MCU's 1 kHz PD
  //    loop tracks; effort feed-forward is left at zero here (the MCU owns the
  //    torque term), but the interface is claimed so a future gravity/feed-forward
  //    term can be added without a graph change.
  const double q_target = q_ref + delta;
  const bool position_written = command_interfaces_[0].set_value(q_target);
  const bool effort_written = command_interfaces_[1].set_value(0.0);
  if (!position_written || !effort_written) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Joint '%s': command interface write failed (position=%d, effort=%d).",
      joint_.c_str(), position_written, effort_written);
  }
  return controller_interface::return_type::OK;
}

}  // namespace soccer_control

PLUGINLIB_EXPORT_CLASS(
  soccer_control::ResidualRLController, controller_interface::ControllerInterface)
