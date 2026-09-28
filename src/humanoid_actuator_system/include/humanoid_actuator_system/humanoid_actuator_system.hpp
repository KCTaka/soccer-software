// Copyright 2026 Humanoid Robotics Team
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
//   * prepare_command_mode_switch is documented upstream as "a non-realtime evaluation" and
//     perform_command_mode_switch as "part of the realtime update loop". Neither holds in
//     general: the controller manager calls both from its real-time thread when a read, update,
//     or write fails (perform_hardware_command_mode_change), and calls perform from the service
//     thread when a switch is not requested as soon as possible (the spawner's default). Both
//     hooks are therefore allocation-free, and neither assumes which thread it is on.
#ifndef HUMANOID_ACTUATOR_SYSTEM__HUMANOID_ACTUATOR_SYSTEM_HPP_
#define HUMANOID_ACTUATOR_SYSTEM__HUMANOID_ACTUATOR_SYSTEM_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "humanoid_actuator_system/tuple_ownership.hpp"
#include "humanoid_safety/safety_kernel.hpp"
#include "humanoid_transport/actuator_transport.hpp"
#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/exchange_stats.hpp"
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

  /// Vetoes a switch that starts or stops part of any joint's five-field MIT tuple (ADR-001). A
  /// controller that claims `position` alone would leave stiffness, damping, and feed-forward
  /// torque owned by nobody, which is not a weaker command but an undefined one. Changes no state:
  /// a switch this accepts can still be abandoned, and nothing tells the hardware.
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  /// Records which joints' tuples are owned once the switch happens. write() sends a controller's
  /// command only while every joint is owned.
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

  void enter_protective_state(safety::Trigger trigger) noexcept;

  /// Non-real-time. Logs stats_; called when a run ends (deactivate or shutdown).
  void log_exchange_stats() const;

  /// The only call site of transport_->exchange(). Times the call and accounts for it in stats_,
  /// so no path -- normal or protective -- can exchange without being counted.
  [[nodiscard]] transport::ExchangeResult exchange_once(
    transport::MonotonicStamp deadline) noexcept;

  /// Called from on_configure. Returns false (configuration error) if the transport cannot deliver
  /// the five-field tuple and degradation has not been explicitly accepted. The tuple is the only
  /// claim prepare_command_mode_switch() accepts, so this needs no knowledge of the claim.
  [[nodiscard]] bool check_tuple_capability() noexcept;

  // Declaration order is load-bearing. Members are destroyed in reverse declaration order, so the
  // loader declared first is destroyed last -- after the instance whose deleter it owns. Reversing
  // these two lines produces a crash on shutdown that reproduces only sometimes.
  std::unique_ptr<pluginlib::ClassLoader<transport::ActuatorTransport>> loader_;
  pluginlib::UniquePtr<transport::ActuatorTransport> transport_;

  safety::SafetyKernel safety_kernel_;

  transport::JointManifest joint_manifest_{};
  transport::SafetyManifest safety_manifest_{};

  std::vector<std::string> joint_names_;
  std::vector<double> command_storage_;
  std::vector<double> state_storage_;
  hardware_interface::HardwareInfo info_;

  // Preallocated. Nothing in the cycle may allocate.
  transport::CommandBatch command_snapshot_{};
  transport::FeedbackBatch feedback_{};

  // Written only by write() on the real-time thread; read only by lifecycle callbacks, which the
  // controller_manager never runs concurrently with read()/write() of the same component.
  transport::ExchangeStats stats_{};

  // Sequence of the most recent command handed to exchange(), whether or not it succeeded. The
  // next read() accepts feedback only if it answers this sequence. nullopt until the first send.
  std::optional<transport::CycleSequence> last_sent_sequence_;
  // Outcome of that exchange, so read() can report why a cycle was bad.
  transport::TransportError last_exchange_error_{transport::TransportError::kNone};

  transport::CycleSequence cycle_{0U};
  std::uint8_t consecutive_bad_cycles_{0U};
  // Written by write() and by lifecycle callbacks, which never run concurrently with it (as for
  // stats_). on_activate() is the one exit from kProtective.
  CommandAuthority authority_{CommandAuthority::kAwaitingClaim};

  // Joints whose tuples a controller owns. Written by perform_command_mode_switch(), which the
  // controller manager may call from the service thread or the real-time thread, and read by
  // write(). Not a LatestValueBuffer: that is single-producer, and a switch is a read-modify-write
  // that must land as one step, or a hand-over between controllers is seen half done.
  std::atomic<JointMask> owned_joints_{0U};

  transport::TupleCompleteness accepted_degradation_{transport::TupleCompleteness::kFull};
};

}  // namespace humanoid::actuator_system

#endif  // HUMANOID_ACTUATOR_SYSTEM__HUMANOID_ACTUATOR_SYSTEM_HPP_
