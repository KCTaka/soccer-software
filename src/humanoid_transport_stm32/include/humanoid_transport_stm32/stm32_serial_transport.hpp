// Copyright 2026 UTRA-RoboSoccer
// Stm32SerialTransport: the production ActuatorTransport (ADR-001-04).
//
//   Jetson --USB CDC--> master STM32 --SPI, 200 Hz--> slave STM32 --CAN--> RobStride drives
//
// The master STM32, not this process, schedules the 200 Hz cycle (ADR-001-03). Each exchange()
// sends one command for the whole robot and waits for the master's next telemetry frame, so the
// Jetson loop is phase-locked to the master's cycle instead of beating against it. A frame that
// does not arrive by the deadline is a bad cycle, and three in a row are command loss (ADR-002).
//
// Safety, and where it lives. This transport asks for MIT on every joint every cycle and never for
// HOLD: a motor that dropped to IDLE or FAULT stays there until the next activation, because
// auto-rearm is prohibited. The firmware owns the independent protections: the host-death
// watchdog (the master damps and then idles the motors if no fresh command arrives for 12 cycles
// while any motor is armed), the slave's soft limits, and the drive's own protection. Nothing here
// keeps a stalled real-time loop alive, and nothing may: no background thread sends commands.
//
// Real-time. exchange() is the only real-time entry point besides the two disable requests. It
// allocates nothing, takes no lock, and waits only on an eventfd with the caller's deadline.
//
// Parameters (node "stm32_serial", all read once at configure, all read-only):
//   serial_device          the master's USB CDC device, e.g. /dev/robosoccer-master (required)
//   handshake_timeout_s    how long configure() waits for the master to answer
//   arming_timeout_s       how long activate() waits for every motor to clear to IDLE and then
//                          report HOLD, both with fresh feedback
//   release_timeout_s      how long deactivate() waits for every motor to report IDLE
//   fault_grace_s          how long a motor may stay faulted after its fault reset is sent
//   joints.<name>.chain, .motor, .direction_sign, .zero_offset_rad
//                          where each manifest joint lives and its angle convention
// direction_sign and zero_offset_rad have no default: a plausible-looking default is exactly the
// kind of value that goes unnoticed until the joint runs the wrong way.
#ifndef HUMANOID_TRANSPORT_STM32__STM32_SERIAL_TRANSPORT_HPP_
#define HUMANOID_TRANSPORT_STM32__STM32_SERIAL_TRANSPORT_HPP_

#include <array>
#include <atomic>
#include <bitset>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "humanoid_transport/actuator_transport.hpp"
#include "humanoid_transport_stm32/joint_policy.hpp"
#include "humanoid_transport_stm32/joint_wiring.hpp"
#include "humanoid_transport_stm32/master_link.hpp"
#include "humanoid_transport_stm32/parameter_node.hpp"
#include "humanoid_transport_stm32/wire_protocol.hpp"

namespace humanoid::transport_stm32
{

class Stm32SerialTransport : public transport::ActuatorTransport
{
public:
  /// Everything configure() reads from ROS parameters, as plain values.
  struct Parameters
  {
    std::string serial_device;
    std::chrono::milliseconds handshake_timeout{2000};
    std::chrono::milliseconds arming_timeout{3000};
    std::chrono::milliseconds release_timeout{1000};
    std::chrono::milliseconds fault_grace{500};
    /// One entry per manifest joint, in manifest order.
    std::vector<JointWiring> wiring;
  };

  Stm32SerialTransport() = default;
  ~Stm32SerialTransport() override;

  Stm32SerialTransport(const Stm32SerialTransport &) = delete;
  Stm32SerialTransport & operator=(const Stm32SerialTransport &) = delete;
  Stm32SerialTransport(Stm32SerialTransport &&) = delete;
  Stm32SerialTransport & operator=(Stm32SerialTransport &&) = delete;

  // --- Non-real-time ---

  /// Reads the parameters from the "stm32_serial" node, then configures.
  [[nodiscard]] bool configure(
    const transport::JointManifest & joints, const transport::SafetyManifest & safety) override;

  /// The ROS-free core of configure(): validates the manifests and wiring, opens the port, and
  /// checks what the master reports. Public so tests, which have no parameter server, can use it.
  [[nodiscard]] bool configure_with(
    const Parameters & parameters, const transport::JointManifest & joints,
    const transport::SafetyManifest & safety);

  /// Clears every motor to IDLE with a fault reset, then arms it into HOLD, and returns once all
  /// report HOLD with fresh feedback. Fails leaving them idle. See plan_request for why it takes
  /// two steps.
  [[nodiscard]] bool activate() override;

  /// Idles every motor and waits for the master to confirm it.
  void deactivate() override;

  [[nodiscard]] transport::TransportCapabilities capabilities() const override;

  // --- Real-time ---

  [[nodiscard]] transport::ExchangeResult exchange(
    const transport::CommandBatch & command, transport::FeedbackBatch & feedback,
    transport::MonotonicStamp deadline) noexcept override;

  /// Asks for IDLE on one joint from the next exchange on, until the next activation.
  [[nodiscard]] bool request_joint_disable(transport::JointIndex joint) noexcept override;

  /// Asks for IDLE on every joint from the next exchange on, until the next activation.
  [[nodiscard]] bool request_all_disable() noexcept override;

  [[nodiscard]] transport::HealthSnapshot health_snapshot() const noexcept override;

private:
  // --- configure() steps, in order ---
  [[nodiscard]] std::optional<Parameters> declare_parameters(
    const transport::JointManifest & joints);
  [[nodiscard]] bool check_manifests(
    const Parameters & parameters, const transport::JointManifest & joints,
    const transport::SafetyManifest & safety);
  [[nodiscard]] bool open_link(const Parameters & parameters);
  [[nodiscard]] bool check_master(const Parameters & parameters);

  // --- Lifecycle, non-real-time ---
  [[nodiscard]] bool arm_motors();
  /// Streams `phase`'s request until every joint reaches its target, one fails, or `give_up`.
  [[nodiscard]] bool run_activation_step(Phase phase, MonotonicStamp start, MonotonicStamp give_up);
  /// Why `joint` has not reached `phase`'s target, from the last frame seen, for the timeout.
  [[nodiscard]] std::string explain_waiting(
    Phase phase, std::uint8_t joint, const std::optional<TelemetryFrame> & frame) const;
  /// Streams IDLE until every motor reports it. Returns the first joint that never did.
  [[nodiscard]] std::optional<std::uint8_t> release_motors(std::chrono::milliseconds timeout);
  [[nodiscard]] bool send_phase_frame(Phase phase, std::uint16_t cycle_id) noexcept;
  [[nodiscard]] std::string joint_name(std::uint8_t joint) const;

  // --- Shared with the real-time path ---
  [[nodiscard]] WriteStatus transmit(
    const wire::CmdRobot & command,
    MonotonicStamp deadline) noexcept;
  [[nodiscard]] std::uint16_t next_cmd_seq() noexcept;
  /// A command frame with its chain headers set and every slot unrequested.
  [[nodiscard]] wire::CmdRobot blank_command(std::uint16_t cycle_id) noexcept;
  [[nodiscard]] wire::CmdMotor & slot_of(wire::CmdRobot & command, const JointWiring & w) noexcept;

  // --- Constants ---
  static constexpr std::uint8_t kNoChain = 0xFF;
  /// How long arming and releasing wait for one telemetry frame before sending the next request.
  /// Well inside the master's 12-cycle host-death limit, so a slow frame cannot trip it.
  static constexpr std::chrono::milliseconds kFrameWait{20};

  // --- Configuration, read once at configure ---
  // Declared first so it is destroyed last: nothing below touches the node.
  std::unique_ptr<ParameterNode> parameter_node_;
  Parameters parameters_;
  WiringLayout layout_;
  std::vector<std::string> joint_names_;
  /// Index into layout_.chains of each wire chain id, or kNoChain.
  std::array<std::uint8_t, wire::kMaxChains> chain_index_{};
  std::uint8_t joint_count_{0};
  std::chrono::microseconds feedback_max_age_{0};
  std::uint32_t nominal_cycle_period_us_{0};

  // --- Link ---
  // Declared after everything its reader thread could touch, so it is stopped first.
  MasterLink link_;

  // --- Lifecycle, shared across threads ---
  std::atomic<bool> active_{false};
  std::atomic<std::uint32_t> isolated_mask_{0};
  std::atomic<std::uint32_t> availability_epoch_{0};

  // --- Real-time state ---
  // Touched by the thread that calls exchange() while active, and by the lifecycle thread while
  // not; never by both at once.
  std::array<std::uint8_t, wire::kRobotCmdFrameSize> tx_buffer_{};
  std::uint16_t cmd_seq_{0};
  std::uint16_t header_seq_{0};
  /// cycle_id of the last telemetry frame consumed: the next command echoes it, and the next
  /// frame must be ahead of it.
  std::uint16_t last_cycle_{0};
  std::bitset<transport::kMaxJoints> availability_;
};

}  // namespace humanoid::transport_stm32

#endif  // HUMANOID_TRANSPORT_STM32__STM32_SERIAL_TRANSPORT_HPP_
