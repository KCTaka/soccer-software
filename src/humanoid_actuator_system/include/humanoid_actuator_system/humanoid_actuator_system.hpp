// The only production hardware_interface::SystemInterface in this robot (ADR-001).
//
// Verified against ros2_control 4.47.0 (Jazzy). In that release SystemInterface is a four-line
// class whose entire body is `return_type write(...) override = 0;`; everything else is inherited
// from HardwareComponentInterface. Consequently:
//
//   * export_state_interfaces()   and export_command_interfaces()   are DEPRECATED. The framework
//     owns interface memory now. Override on_export_state_interfaces() and
//     on_export_command_interfaces() instead, which return ConstSharedPtr / SharedPtr vectors.
//   * on_init(const HardwareInfo &) is DEPRECATED. Override
//     on_init(const HardwareComponentInterfaceParams &).
//   * prepare_command_mode_switch is documented upstream as "a non-realtime evaluation";
//     perform_command_mode_switch is documented as "part of the realtime update loop, and should
//     be fast". The MIT tuple-ownership check therefore belongs in prepare, not perform.
#ifndef HUMANOID_ACTUATOR_SYSTEM__HUMANOID_ACTUATOR_SYSTEM_HPP_
#define HUMANOID_ACTUATOR_SYSTEM__HUMANOID_ACTUATOR_SYSTEM_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "humanoid_safety/safety_kernel.hpp"
#include "humanoid_transport/actuator_transport.hpp"
#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/joint_manifest.hpp"
#include "humanoid_transport/safety_manifest.hpp"

namespace humanoid::actuator_system
{

class HumanoidActuatorSystem : public hardware_interface::SystemInterface
{
public:
  HumanoidActuatorSystem() = default;
  ~HumanoidActuatorSystem() override = default;

  HumanoidActuatorSystem(const HumanoidActuatorSystem &) = delete;
  HumanoidActuatorSystem & operator=(const HumanoidActuatorSystem &) = delete;
  HumanoidActuatorSystem(HumanoidActuatorSystem &&) = delete;
  HumanoidActuatorSystem & operator=(HumanoidActuatorSystem &&) = delete;

  // --- Lifecycle. Non-real-time. ---

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  // --- Interface export. 4.47.0 API: the framework owns the memory. ---

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
    override;

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
    override;

  // --- Mode switching. ---

  /// Non-real-time. Rejects partial claims, mixed ownership, and partial release of the five-field
  /// MIT tuple. A controller that claims `position` alone would silently leave stiffness, damping,
  /// and feed-forward torque owned by nobody, which is not a weaker command but an undefined one.
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  /// Real-time. Applies the decision already validated in prepare. Must be fast.
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // --- The 5 ms cycle. No ROS, no allocation, no logging, no filesystem, no unbounded wait. ---

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Copies all five fields for all joints into one immutable cycle snapshot. Taken BEFORE
  /// validation, so that safety projection and transport both consume the same object and no
  /// transport can observe a partially updated tuple.
  void snapshot_commands() noexcept;

  /// Rejects duplicate, out-of-order, structurally invalid, stale, or manifest-mismatched feedback.
  [[nodiscard]] bool accept_feedback(const transport::FeedbackBatch & candidate) noexcept;

  void enter_protective_state(safety::Trigger trigger) noexcept;

  /// Called from on_configure. Returns false (configuration error) if the claimed interface set
  /// requires fields the transport cannot deliver and degradation has not been explicitly accepted.
  [[nodiscard]] bool check_tuple_capability() noexcept;

  // Declaration order is load-bearing. Members are destroyed in reverse declaration order, so the
  // loader declared first is destroyed last -- after the instance whose deleter it owns. Reversing
  // these two lines produces a crash on shutdown that reproduces only sometimes.
  std::unique_ptr<pluginlib::ClassLoader<transport::ActuatorTransport>> loader_;
  pluginlib::UniquePtr<transport::ActuatorTransport> transport_;

  safety::SafetyKernel safety_kernel_;

  transport::JointManifest joint_manifest_{};
  transport::SafetyManifest safety_manifest_{};

  // Preallocated. Nothing in the cycle may allocate.
  transport::CommandBatch command_snapshot_{};
  transport::FeedbackBatch feedback_{};

  transport::CycleSequence cycle_{0U};
  std::uint8_t consecutive_bad_cycles_{0U};
  bool mit_tuple_claimed_{false};
  bool protective_state_{false};

  transport::TupleCompleteness accepted_degradation_{transport::TupleCompleteness::kFull};
};

}  // namespace humanoid::actuator_system

#endif  // HUMANOID_ACTUATOR_SYSTEM__HUMANOID_ACTUATOR_SYSTEM_HPP_
