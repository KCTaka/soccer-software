// MitImpedanceController: the sole owner of the five-field MIT tuple (ADR-001-02).
//
// Claims all five command interfaces for every controlled joint.
// prepare_command_mode_switch rejects partial claims, mixed ownership,
// and partial release.
//
// update() reads one reference from the SPSC buffer and writes all five
// fields for all joints. No ROS, no allocation, no logging, no filesystem.
#ifndef HUMANOID_MIT_CONTROLLER__MIT_CONTROLLER_HPP_
#define HUMANOID_MIT_CONTROLLER__MIT_CONTROLLER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "humanoid_mit_controller/reference_buffer.hpp"
#include "humanoid_transport/batch_types.hpp"

namespace humanoid::control
{

/// The five interface suffixes claimed per joint, in fixed order.
inline constexpr std::array<const char *, 5> kMitInterfaceNames = {
  "position", "velocity", "effort", "stiffness", "damping"};

class MitImpedanceController : public controller_interface::ControllerInterface
{
public:
  MitImpedanceController() = default;
  ~MitImpedanceController() override = default;

  // --- Lifecycle ---
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  // --- Interface configuration ---
  controller_interface::InterfaceConfiguration command_interface_configuration()
    const override;
  controller_interface::InterfaceConfiguration state_interface_configuration()
    const override;

  // --- The 200 Hz loop. No ROS, no allocation, no logging. ---
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // --- Non-real-time reference injection ---
  /// Called by a non-RT thread (trajectory player, Zenoh subscriber).
  /// Thread-safe. The next update() will see the new reference.
  void publish_reference(const ReferenceFrame & frame) noexcept;

  /// Access for external non-RT components (launch, trajectory loader).
  ReferenceBuffer & reference_buffer() noexcept { return reference_buffer_; }

private:
  /// Builds the full interface name: "joint_name/suffix".
  static std::string interface_name(
    const std::string & joint, const char * suffix);

  // Parameters
  std::vector<std::string> joint_names_;

  // Interface handles, ordered: [joint0/pos, joint0/vel, ..., jointN/damping]
  // The framework assigns them in the order returned by
  // command_interface_configuration(). We store indices for fast access.
  std::size_t num_joints_{0U};

  // Reference buffer
  ReferenceBuffer reference_buffer_;

  // Fallback reference used until the first valid reference arrives.
  // Holds the last measured position with zero velocity, zero effort,
  // zero stiffness, zero damping. Prevents a step on activation.
  ReferenceFrame fallback_reference_{};
  bool have_fallback_{false};

  // Cycle counter for diagnostics
  transport::CycleSequence cycle_count_{0U};
};

}  // namespace humanoid::control

#endif  // HUMANOID_MIT_CONTROLLER__MIT_CONTROLLER_HPP_