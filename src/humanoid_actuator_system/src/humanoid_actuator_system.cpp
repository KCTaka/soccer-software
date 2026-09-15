#include "humanoid_actuator_system/humanoid_actuator_system.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace humanoid::actuator_system
{

// SCAFFOLD. Every entry point fails closed. An unimplemented actuator system must refuse to reach
// ACTIVE, never reach it and command something undefined. Implementation is bootstrap step B7 and
// depends on the generated joint manifest (Model Gate 0, G1).

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & /*params*/)
{
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::ERROR;
}

bool HumanoidActuatorSystem::check_tuple_capability() noexcept
{
  if (!transport_) {
  return false;
  }

  const auto caps = transport_->capabilities();

  // A full-tuple controller (all five interfaces claimed) requires a full-tuple transport,
  // unless degradation was explicitly accepted in the hardware configuration.
  if (mit_tuple_claimed_ &&
  caps.tuple_completeness == transport::TupleCompleteness::kPositionVelocityOnly &&
  accepted_degradation_ != transport::TupleCompleteness::kPositionVelocityOnly)
  {
  // Fail closed. A reduced transport silently passing as complete is Topic 1 finding 14.
  return false;
  }

  // If degradation is accepted, record it. The tuple mapping at the transport boundary will
  // drop stiffness/damping/effort before exchange() and note the degradation in every log record.
  // That mapping logic lives in snapshot_commands() once B7 is implemented.

  return true;
}


hardware_interface::CallbackReturn HumanoidActuatorSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  enter_protective_state(safety::Trigger::kOperatorStop);
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
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidActuatorSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  enter_protective_state(safety::Trigger::kOperatorStop);
  transport_.reset();
  loader_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
HumanoidActuatorSystem::on_export_state_interfaces()
{
  return {};
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
HumanoidActuatorSystem::on_export_command_interfaces()
{
  return {};
}

hardware_interface::return_type HumanoidActuatorSystem::prepare_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type HumanoidActuatorSystem::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type HumanoidActuatorSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type HumanoidActuatorSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Target ordering, fixed by ADR-001 and ADR-002 and not to be reordered:
  //   1. snapshot_commands()               -- one immutable all-joint tuple snapshot
  //   2. safety_kernel_.project(...)       -- projects the snapshot into the feasible set
  //   3. transport_->exchange(...)         -- bounded by a cycle deadline
  //   4. record into the lock-free ring    -- drained to MCAP by a non-real-time thread
  return hardware_interface::return_type::ERROR;
}

void HumanoidActuatorSystem::snapshot_commands() noexcept {}

bool HumanoidActuatorSystem::accept_feedback(
  const transport::FeedbackBatch & /*candidate*/) noexcept
{
  return false;
}

void HumanoidActuatorSystem::enter_protective_state(safety::Trigger trigger) noexcept
{
  safety_kernel_.enter_protective(trigger, command_snapshot_);
  protective_state_ = true;
}

}  // namespace humanoid::actuator_system

PLUGINLIB_EXPORT_CLASS(
  humanoid::actuator_system::HumanoidActuatorSystem, hardware_interface::SystemInterface)
