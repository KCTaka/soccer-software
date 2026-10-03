// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_stm32/stm32_serial_transport.hpp"

#include <format>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <string_view>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "humanoid_transport/timing.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace
{

// File-local helpers. Declared here, defined at the end of this file, so the class implementation
// reads first.
bool fail(std::string_view message);
void warn(std::string_view message);
std::chrono::milliseconds to_milliseconds(double seconds);
humanoid::transport_stm32::wire::CmdMotor plain_request(
  const humanoid::transport_stm32::JointRequest & request) noexcept;

}  // namespace

namespace humanoid::transport_stm32
{

Stm32SerialTransport::~Stm32SerialTransport()
{
  if (active_.load()) {
    deactivate();
  }
  link_.close();
}

// ===========================================================================
// ActuatorTransport overrides, non-real-time
// ===========================================================================

bool Stm32SerialTransport::configure(
  const transport::JointManifest & joints, const transport::SafetyManifest & safety)
{
  const auto parameters = declare_parameters(joints);
  return parameters && configure_with(*parameters, joints, safety);
}

bool Stm32SerialTransport::configure_with(
  const Parameters & parameters, const transport::JointManifest & joints,
  const transport::SafetyManifest & safety)
{
  active_.store(false);
  link_.close();
  const bool configured = check_manifests(parameters, joints, safety) && open_link(parameters) &&
    check_master(parameters);
  if (!configured) {
    joint_count_ = 0;  // so activate() refuses a half-configured transport
  }
  return configured;
}

bool Stm32SerialTransport::activate()
{
  if (active_.load()) {
    return true;
  }
  if (joint_count_ == 0 || link_.failed()) {
    return fail("activate() called before a successful configure(), or the link is down");
  }

  // Activation is the operator's acknowledgement (ADR-002): it clears anything the previous
  // session left latched, here and in the master.
  link_.acknowledge_master_reset();
  isolated_mask_.store(0);

  if (!arm_motors()) {
    // Never leave a motor armed behind a failed activation.
    if (const auto stuck = release_motors(parameters_.release_timeout)) {
      warn(std::format("joint '{}' did not confirm IDLE after the failed activation",
        joint_name(*stuck)));
    }
    return false;
  }

  availability_epoch_.store(0);
  active_.store(true, std::memory_order_release);
  return true;
}

void Stm32SerialTransport::deactivate()
{
  if (!active_.exchange(false)) {
    return;
  }
  if (const auto stuck = release_motors(parameters_.release_timeout)) {
    warn(std::format(
        "joint '{}' did not confirm IDLE within {} ms of deactivation", joint_name(*stuck),
        parameters_.release_timeout.count()));
  }
}

transport::TransportCapabilities Stm32SerialTransport::capabilities() const
{
  transport::TransportCapabilities caps{};
  std::strncpy(
    caps.implementation_name.data(), "Stm32SerialTransport", caps.implementation_name.size() - 1);
  caps.transport_class = transport::TransportClass::kPhysical;
  caps.joint_count = joint_count_;
  // The master's own cycle: the rate at which fresh feedback exists, whatever the host does.
  caps.nominal_cycle_period_us = nominal_cycle_period_us_;
  // exchange() returns by its deadline whether or not the master answered.
  caps.worst_case_exchange_us = nominal_cycle_period_us_;
  caps.supports_per_joint_disable = true;
  caps.supports_availability_mask = true;
  caps.provides_temperature = true;
  caps.provides_bus_voltage = false;
  caps.is_deterministic = false;
  caps.tuple_completeness = transport::TupleCompleteness::kFull;
  return caps;
}

// ===========================================================================
// ActuatorTransport overrides, real-time
// ===========================================================================

transport::ExchangeResult Stm32SerialTransport::exchange(
  const transport::CommandBatch & command, transport::FeedbackBatch & feedback,
  transport::MonotonicStamp deadline) noexcept
{
  transport::ExchangeResult result{};
  const auto t_start = std::chrono::steady_clock::now();

  if (!active_.load(std::memory_order_acquire)) {
    result.error = transport::TransportError::kNotActive;
    return result;
  }
  if (command.joint_count != joint_count_) {
    result.error = transport::TransportError::kManifestMismatch;
    return result;
  }
  // A reset master boots disarmed (ADR-002). Until the next activation says otherwise, nothing it
  // reports is a state this transport armed.
  if (link_.failed() || link_.master_reset_seen()) {
    result.error = transport::TransportError::kHardwareFault;
    return result;
  }

  // --- Build the command: one request per joint, from the tuple and the joint's own state ---
  const std::uint32_t isolated = isolated_mask_.load(std::memory_order_relaxed);
  wire::CmdRobot wire_command = blank_command(last_cycle_);
  bool tuples_representable = true;
  for (std::uint8_t j = 0; j < joint_count_; ++j) {
    const JointWiring & wiring = layout_.joints[j];
    const bool is_isolated = ((isolated >> j) & 1U) != 0U;
    const wire::EncodedCommand encoded = wire::encode_command(
      wire::ModeRequest::kMit, to_setpoint(wiring, command.joints[j]), wire::kCmdFlagValid);
    const JointRequest request = plan_request(Phase::kRunning, is_isolated, encoded.representable);
    slot_of(wire_command, wiring) = request.send_tuple ? encoded.command : plain_request(request);
    if (!encoded.representable && !is_isolated) {
      tuples_representable = false;
    }
  }

  // --- Send, then wait for the master's next cycle ---
  const WriteStatus sent = transmit(wire_command, deadline);
  if (sent != WriteStatus::kOk) {
    result.error = sent == WriteStatus::kTimeout ?
      transport::TransportError::kTimeout : transport::TransportError::kHardwareFault;
    return result;
  }
  const auto frame = link_.wait_for_newer(last_cycle_, deadline);
  if (!frame) {
    result.error = link_.failed() ?
      transport::TransportError::kHardwareFault : transport::TransportError::kTimeout;
    return result;
  }
  last_cycle_ = frame->robot.header.cycle_id;

  // --- Interpret. feedback is untouched on every error below, so a failed exchange can never be
  // mistaken for a current sample: its sequence stays behind the command's. ---
  if (link_.master_reset_seen() ||
    frame->robot.header.robot_state == static_cast<std::uint8_t>(wire::RobotState::kHostLost))
  {
    result.error = transport::TransportError::kHardwareFault;
    return result;
  }

  std::array<transport::JointFeedback, transport::kMaxJoints> readings{};
  std::bitset<transport::kMaxJoints> available;
  std::uint8_t present = 0;
  std::uint8_t fresh = 0;
  for (std::uint8_t j = 0; j < joint_count_; ++j) {
    const JointWiring & wiring = layout_.joints[j];
    const auto motors_on_chain = chain_motor_count(frame->robot, wiring.chain);
    if (motors_on_chain && *motors_on_chain <= wiring.motor) {
      // The chain answers but has fewer motors than the wiring: firmware built for another robot.
      result.error = transport::TransportError::kManifestMismatch;
      return result;
    }
    const auto motor = find_motor(frame->robot, wiring.chain, wiring.motor);
    const JointReading reading = read_joint(wiring, motor, feedback_max_age_);
    readings[j] = reading.feedback;
    if (motor) {
      ++present;
    }
    if (reading.feedback.fresh) {
      ++fresh;
    }
    available.set(j, reading.commandable && ((isolated >> j) & 1U) == 0U);
  }
  if (present == 0) {
    result.error = transport::TransportError::kIncompleteBatch;
    return result;
  }
  if (!tuples_representable) {
    // The offending joints were sent DAMPED rather than clamped. Failing the exchange makes the
    // caller count the cycle as bad, so a SafetyKernel that lets this through trips protective.
    result.error = transport::TransportError::kManifestMismatch;
    return result;
  }

  if (available != availability_) {
    availability_ = available;
    availability_epoch_.fetch_add(1, std::memory_order_relaxed);
  }

  feedback.sequence = command.sequence;
  feedback.stamp = frame->received;
  feedback.joint_count = joint_count_;
  feedback.availability_mask = availability_;
  feedback.availability_epoch = availability_epoch_.load(std::memory_order_relaxed);
  std::copy_n(readings.begin(), joint_count_, feedback.joints.begin());

  result.error = transport::TransportError::kNone;
  result.joints_reported = fresh;
  result.round_trip = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now() - t_start);
  return result;
}

bool Stm32SerialTransport::request_joint_disable(transport::JointIndex joint) noexcept
{
  if (joint >= joint_count_) {
    return false;
  }
  isolated_mask_.fetch_or(1U << joint, std::memory_order_relaxed);
  return active_.load(std::memory_order_acquire);
}

bool Stm32SerialTransport::request_all_disable() noexcept
{
  // Only joint_count_ low bits: a mask bit for a joint that does not exist would be noise.
  const std::uint32_t all = (joint_count_ >= 32U) ? ~0U : ((1U << joint_count_) - 1U);
  isolated_mask_.fetch_or(all, std::memory_order_relaxed);
  return active_.load(std::memory_order_acquire);
}

transport::HealthSnapshot Stm32SerialTransport::health_snapshot() const noexcept
{
  const MasterLink::Counters counters = link_.counters();
  transport::HealthSnapshot snapshot{};
  snapshot.framing_errors = counters.framing_errors;
  snapshot.sequence_rejections = counters.duplicate_frames;
  snapshot.availability_epoch = availability_epoch_.load(std::memory_order_relaxed);
  snapshot.active = active_.load(std::memory_order_acquire);
  return snapshot;
}

// ===========================================================================
// configure() steps, in call order
// ===========================================================================

std::optional<Stm32SerialTransport::Parameters> Stm32SerialTransport::declare_parameters(
  const transport::JointManifest & joints)
{
  // A fresh node per configure(): declare_parameter() throws if a name is declared twice, and a
  // reconfigure must re-read the overrides. The old node is destroyed (joining its executor)
  // before the new one exists, so two "stm32_serial" nodes are never alive at once.
  parameter_node_.reset();
  parameter_node_ = std::make_unique<ParameterNode>("stm32_serial");
  const auto & node = parameter_node_->node();

  // Read once here; nothing re-reads them, so read_only makes `ros2 param set` say so instead of
  // appearing to work.
  const auto describe = [](const char * text) {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = text;
      d.read_only = true;
      return d;
    };

  Parameters parameters;
  try {
    parameters.serial_device = node->declare_parameter<std::string>(
      "serial_device", "",
        describe("The master STM32's USB CDC device, e.g. /dev/robosoccer-master"));
    const Parameters defaults;
    parameters.handshake_timeout = to_milliseconds(node->declare_parameter<double>(
      "handshake_timeout_s",
      std::chrono::duration<double>(defaults.handshake_timeout).count(),
      describe("How long configure waits for the master to answer, s")));
    parameters.arming_timeout = to_milliseconds(node->declare_parameter<double>(
      "arming_timeout_s", std::chrono::duration<double>(defaults.arming_timeout).count(),
      describe("How long activate waits for every motor to clear to IDLE, then report HOLD, s")));
    parameters.release_timeout = to_milliseconds(node->declare_parameter<double>(
      "release_timeout_s", std::chrono::duration<double>(defaults.release_timeout).count(),
      describe("How long deactivate waits for every motor to report IDLE, s")));
    parameters.fault_grace = to_milliseconds(node->declare_parameter<double>(
      "fault_grace_s", std::chrono::duration<double>(defaults.fault_grace).count(),
      describe("How long a motor may stay faulted after its fault reset is sent, s")));

    // Per joint. Chain, motor, sign and offset have no usable default: a value that merely looks
    // plausible would go unnoticed until the joint ran the wrong way. -1 and NaN are rejected by
    // make_layout and named in its message.
    constexpr double kUnsetOffset = std::numeric_limits<double>::quiet_NaN();
    for (std::uint8_t i = 0; i < joints.joint_count; ++i) {
      const std::string prefix = std::format("joints.{}.", joints.joints[i].name.data());
      JointWiring wiring;
      const auto chain = node->declare_parameter<std::int64_t>(
        prefix + "chain", -1,
          describe("Firmware chain id (the slave STM32) the joint's motor is on"));
      const auto motor = node->declare_parameter<std::int64_t>(
        prefix + "motor", -1, describe("Index of the joint's motor within its chain"));
      const auto sign = node->declare_parameter<std::int64_t>(
        prefix + "direction_sign", 0,
          describe("+1 or -1: joint = sign * (wire - zero_offset_rad)"));
      wiring.zero_offset_rad = node->declare_parameter<double>(
        prefix + "zero_offset_rad", kUnsetOffset,
        describe("Wire angle, rad, at the joint's zero pose"));
      // Out-of-range values are carried into the wiring as impossible indices so make_layout
      // reports them by joint name, instead of wrapping silently in a narrower type.
      wiring.chain = static_cast<std::uint8_t>(std::clamp<std::int64_t>(chain, 0, 255));
      wiring.motor = static_cast<std::uint8_t>(std::clamp<std::int64_t>(motor, 0, 255));
      if (chain < 0 || motor < 0) {
        fail(std::format(
            "parameters '{}chain' and '{}motor' are required for joint '{}'", prefix, prefix,
            joints.joints[i].name.data()));
        return std::nullopt;
      }
      wiring.direction_sign = static_cast<std::int8_t>(std::clamp<std::int64_t>(sign, -128, 127));
      parameters.wiring.push_back(wiring);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    // Typical cause: an integer literal in YAML (1) where a double (1.0) is expected, or the
    // reverse.
    fail(std::format("parameter type mismatch: {}", e.what()));
    return std::nullopt;
  }

  if (parameters.serial_device.empty()) {
    fail("parameter 'serial_device' is not set");
    return std::nullopt;
  }
  // Serviced only once every parameter is declared.
  parameter_node_->spin();
  return parameters;
}

bool Stm32SerialTransport::check_manifests(
  const Parameters & parameters, const transport::JointManifest & joints,
  const transport::SafetyManifest & safety)
{
  joint_count_ = 0;
  if (joints.joint_count == 0) {
    return fail("the joint manifest is empty");
  }
  if (parameters.wiring.size() != joints.joint_count) {
    return fail(std::format(
        "the wiring covers {} joints but the manifest has {}", parameters.wiring.size(),
        joints.joint_count));
  }
  if (safety.joint_count != joints.joint_count) {
    return fail(std::format(
        "the safety manifest covers {} joints but the joint manifest has {}", safety.joint_count,
        joints.joint_count));
  }

  joint_names_.clear();
  for (std::uint8_t i = 0; i < joints.joint_count; ++i) {
    joint_names_.emplace_back(joints.joints[i].name.data());
  }

  auto layout = make_layout(parameters.wiring, joint_names_);
  if (std::holds_alternative<std::string>(layout)) {
    return fail(std::format("wiring: {}", std::get<std::string>(layout)));
  }
  layout_ = std::get<WiringLayout>(layout);

  // The age at which a drive's report stops counting as fresh. The wire saturates the age at 255
  // ms and uses 255 for "never heard from this drive", so a limit that reaches it could never
  // reject a silent drive.
  constexpr std::uint32_t kWireAgeSaturationUs = 255'000;
  if (safety.feedback_max_age_us == 0 || safety.feedback_max_age_us >= kWireAgeSaturationUs) {
    return fail(std::format(
        "feedback_max_age_us = {} must be in (0, {}): the wire reports a drive's age in whole "
        "milliseconds and saturates at 255", safety.feedback_max_age_us, kWireAgeSaturationUs));
  }
  feedback_max_age_ = std::chrono::microseconds{safety.feedback_max_age_us};

  for (std::uint8_t i = 0; i < joints.joint_count; ++i) {
    if (const auto error = check_envelope(layout_.joints[i], safety.envelopes[i])) {
      return fail(std::format("joint '{}': {}", joint_names_[i], *error));
    }
  }

  chain_index_.fill(kNoChain);
  for (std::uint8_t c = 0; c < layout_.chain_count; ++c) {
    chain_index_[layout_.chains[c].chain_id] = c;
  }
  joint_count_ = joints.joint_count;
  parameters_ = parameters;
  return true;
}

bool Stm32SerialTransport::open_link(const Parameters & parameters)
{
  if (const auto error = link_.open(parameters.serial_device)) {
    return fail(*error);
  }
  return true;
}

bool Stm32SerialTransport::check_master(const Parameters & parameters)
{
  const auto status = link_.wait_for_status(parameters.handshake_timeout);
  if (!status) {
    link_.close();
    if (link_.foreign_version() != 0) {
      return fail(std::format(
          "the master speaks protocol version {} but this build speaks {}; reflash the master or "
          "rebuild the host from the matching soccer-firmware commit", link_.foreign_version(),
          wire::kProtocolVersion));
    }
    return fail(std::format(
        "no MASTER_STATUS from '{}' within {} ms: is the master powered, and flashed with "
        "PROTO_VERSION {} firmware?", parameters.serial_device,
        parameters.handshake_timeout.count(),
        wire::kProtocolVersion));
  }

  // One telemetry frame per cycle is what exchange() relies on. The firmware emits telemetry from
  // inside the poll cycle today, but the two are separate settings in its configuration, so a
  // master that polls at a different rate than it reports is one whose behaviour is unknown.
  if (status->telemetry_hz == 0 || status->master_poll_hz != status->telemetry_hz) {
    link_.close();
    return fail(std::format(
        "the master polls at {} Hz but reports telemetry at {} Hz; exchange() needs one telemetry "
        "frame per cycle", status->master_poll_hz, status->telemetry_hz));
  }
  constexpr std::uint32_t kMicrosecondsPerSecond = 1'000'000;
  if (kMicrosecondsPerSecond % status->telemetry_hz != 0) {
    link_.close();
    return fail(std::format(
        "a {} Hz cycle is not a whole number of microseconds", status->telemetry_hz));
  }
  nominal_cycle_period_us_ = kMicrosecondsPerSecond / status->telemetry_hz;

  // The wiring is a claim about the robot. Check it against what answers, while nothing is armed.
  const auto frame = link_.wait_for_frame(0, parameters.handshake_timeout);
  if (!frame) {
    link_.close();
    return fail(std::format(
        "the master answers but reports no telemetry within {} ms; no slave is responding",
        parameters.handshake_timeout.count()));
  }
  for (std::uint8_t c = 0; c < layout_.chain_count; ++c) {
    const ChainLayout & chain = layout_.chains[c];
    const auto motors = chain_motor_count(frame->robot, chain.chain_id);
    if (!motors) {
      link_.close();
      return fail(std::format(
          "chain {} is wired to joints but is not reporting: is that slave powered and running "
          "firmware built from the same configuration?", chain.chain_id));
    }
    if (*motors < chain.motor_slots) {
      link_.close();
      return fail(std::format(
          "chain {} reports {} motors but the wiring needs {}: the firmware was built from a "
          "different configuration", chain.chain_id, *motors, chain.motor_slots));
    }
  }

  last_cycle_ = frame->robot.header.cycle_id;
  return true;
}

// ===========================================================================
// Lifecycle helpers, non-real-time
// ===========================================================================

bool Stm32SerialTransport::arm_motors()
{
  // Two steps, never merged into one request: see plan_request. One timeout covers both.
  const auto start = std::chrono::steady_clock::now();
  const auto give_up = start + parameters_.arming_timeout;
  if (!run_activation_step(Phase::kClearing, start, give_up) ||
    !run_activation_step(Phase::kArming, start, give_up))
  {
    return false;
  }
  availability_.reset();
  for (std::uint8_t j = 0; j < joint_count_; ++j) {
    availability_.set(j);
  }
  return true;
}

bool Stm32SerialTransport::run_activation_step(
  Phase phase, MonotonicStamp start, MonotonicStamp give_up)
{
  const char * const step = phase == Phase::kClearing ? "clearing faults" : "arming";
  std::uint64_t seen = link_.frame_count();
  std::uint16_t cycle = last_cycle_;
  std::optional<TelemetryFrame> last_frame;
  std::uint8_t waiting_on = 0;

  for (;; ) {
    if (std::chrono::steady_clock::now() >= give_up) {
      return fail(std::format(
          "timed out after {} ms {}: {}", parameters_.arming_timeout.count(), step,
          explain_waiting(phase, waiting_on, last_frame)));
    }
    if (link_.failed()) {
      return fail(std::format("the serial link failed while {}", step));
    }
    // A request goes out at least every kFrameWait, so the master's host-death watchdog never
    // trips on a slow frame.
    if (!send_phase_frame(phase, cycle)) {
      return fail(std::format("cannot write to the master while {}", step));
    }

    const auto frame = link_.wait_for_frame(seen, kFrameWait);
    if (!frame) {
      continue;
    }
    seen = link_.frame_count();
    cycle = frame->robot.header.cycle_id;
    last_frame = frame;

    const ArmingVerdict verdict = phase == Phase::kClearing ?
      judge_clearing(frame->robot, layout_, feedback_max_age_,
        std::chrono::steady_clock::now() - start, parameters_.fault_grace) :
      judge_arming(frame->robot, layout_, feedback_max_age_);
    waiting_on = verdict.joint;
    switch (verdict.status) {
      case ArmingVerdict::Status::kReached:
        last_cycle_ = cycle;
        return true;
      case ArmingVerdict::Status::kFailed:
        {
          const JointWiring & wiring = layout_.joints[verdict.joint];
          const auto motor = find_motor(frame->robot, wiring.chain, wiring.motor);
          const bool wound =
            motor && motor->cause == static_cast<std::uint8_t>(wire::FaultCause::kWound);
          return fail(std::format(
              "joint '{}' failed while {}: its motor is faulted (cause {}){}",
              joint_name(verdict.joint), step, motor ? wire::cause_name(motor->cause) : "?",
              wound ? ": the shaft is wound beyond a single turn and must be re-zeroed offline" :
              ""));
        }
      case ArmingVerdict::Status::kWaiting:
        break;
    }
  }
}

std::string Stm32SerialTransport::explain_waiting(
  Phase phase, std::uint8_t joint, const std::optional<TelemetryFrame> & frame) const
{
  const char * const target = phase == Phase::kClearing ? "IDLE" : "HOLD";
  const auto max_age_ms = std::chrono::duration<double, std::milli>(feedback_max_age_).count();
  const std::string head = std::format(
    "joint '{}' never reported {} with feedback fresher than {} ms", joint_name(joint), target,
    max_age_ms);
  if (!frame) {
    return head + "; no telemetry arrived";
  }
  const JointWiring & wiring = layout_.joints[joint];
  const auto motor = find_motor(frame->robot, wiring.chain, wiring.motor);
  if (!motor) {
    return std::format("{}; chain {} is not reporting", head, wiring.chain);
  }
  std::string text = std::format(
    "{} (last seen {}, cause {}, feedback age {} ms)", head, wire::lifecycle_name(motor->state),
    wire::cause_name(motor->cause), motor->fb_age_ms);
  // The slave rejects HOLD on an IDLE motor only when it did not discover that motor at boot. It
  // never rediscovers one, so a drive powered after its slave can only be armed after a slave
  // reset.
  const bool hold_rejected = phase == Phase::kArming &&
    motor->state == static_cast<std::uint8_t>(wire::Lifecycle::kIdle) &&
    (motor->flags & wire::kTeleFlagRequestRejected) != 0;
  if (!reports_within(*motor, feedback_max_age_)) {
    text += ": its drive is not answering on CAN. Is it powered, and on the bus?";
  } else if (hold_rejected) {
    text += std::format(
      ": slave {} rejected HOLD because it did not discover this motor when it booted. With the "
      "drive powered, reset that slave", wiring.chain);
  }
  return text;
}

std::optional<std::uint8_t> Stm32SerialTransport::release_motors(std::chrono::milliseconds timeout)
{
  const auto give_up = std::chrono::steady_clock::now() + timeout;
  std::uint64_t seen = link_.frame_count();
  std::uint16_t cycle = last_cycle_;
  std::uint8_t unconfirmed = 0;

  while (std::chrono::steady_clock::now() < give_up && !link_.failed()) {
    if (!send_phase_frame(Phase::kReleasing, cycle)) {
      break;
    }
    const auto frame = link_.wait_for_frame(seen, kFrameWait);
    if (!frame) {
      continue;
    }
    seen = link_.frame_count();
    cycle = frame->robot.header.cycle_id;

    bool all_idle = true;
    for (std::uint8_t j = 0; j < joint_count_; ++j) {
      const JointWiring & wiring = layout_.joints[j];
      const auto motor = find_motor(frame->robot, wiring.chain, wiring.motor);
      if (!motor || motor->state != static_cast<std::uint8_t>(wire::Lifecycle::kIdle)) {
        all_idle = false;
        unconfirmed = j;
        break;
      }
    }
    if (all_idle) {
      return std::nullopt;
    }
  }
  return unconfirmed;
}

bool Stm32SerialTransport::send_phase_frame(Phase phase, std::uint16_t cycle_id) noexcept
{
  wire::CmdRobot command = blank_command(cycle_id);
  for (std::uint8_t j = 0; j < joint_count_; ++j) {
    const JointWiring & wiring = layout_.joints[j];
    slot_of(command, wiring) = plain_request(plan_request(phase, false, true));
  }
  return transmit(command, std::chrono::steady_clock::now() + kFrameWait) == WriteStatus::kOk;
}

std::string Stm32SerialTransport::joint_name(std::uint8_t joint) const
{
  return joint < joint_names_.size() ? joint_names_[joint] : std::format("#{}", joint);
}

// ===========================================================================
// Shared with the real-time path
// ===========================================================================

WriteStatus Stm32SerialTransport::transmit(
  const wire::CmdRobot & command, MonotonicStamp deadline) noexcept
{
  std::array<std::uint8_t, sizeof(wire::CmdRobot)> payload;
  std::memcpy(payload.data(), &command, sizeof(command));
  const auto ts_ms = static_cast<std::uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  const auto size = wire::build_frame(
    wire::MessageType::kRobotCmd, header_seq_++, wire::NodeId::kJetson, wire::NodeId::kMaster,
    ts_ms, payload, tx_buffer_);
  if (!size) {
    return WriteStatus::kError;
  }
  return link_.send({tx_buffer_.data(), *size}, deadline);
}

std::uint16_t Stm32SerialTransport::next_cmd_seq() noexcept
{
  // Zero means "no command applied yet" to the firmware, so it is skipped on wrap.
  cmd_seq_ = cmd_seq_ == 0xFFFFU ? std::uint16_t{1} : static_cast<std::uint16_t>(cmd_seq_ + 1U);
  return cmd_seq_;
}

wire::CmdRobot Stm32SerialTransport::blank_command(std::uint16_t cycle_id) noexcept
{
  wire::CmdRobot command{};
  command.cycle_id = cycle_id;
  command.cmd_seq = next_cmd_seq();
  command.n_chains = layout_.chain_count;
  for (std::uint8_t c = 0; c < layout_.chain_count; ++c) {
    command.chains[c].chain_id = layout_.chains[c].chain_id;
    command.chains[c].n_motors = layout_.chains[c].motor_slots;
  }
  return command;
}

wire::CmdMotor & Stm32SerialTransport::slot_of(
  wire::CmdRobot & command, const JointWiring & wiring) noexcept
{
  return command.chains[chain_index_[wiring.chain]].motors[wiring.motor];
}

}  // namespace humanoid::transport_stm32

namespace
{

bool fail(std::string_view message)
{
  std::cerr << "Stm32SerialTransport: " << message << "\n";
  return false;
}

void warn(std::string_view message)
{
  std::cerr << "Stm32SerialTransport: warning: " << message << "\n";
}

std::chrono::milliseconds to_milliseconds(double seconds)
{
  return std::chrono::milliseconds{static_cast<std::int64_t>(std::lround(seconds * 1000.0))};
}

// A request that carries no setpoint: IDLE, HOLD, or DAMPED on the slave's own gains.
humanoid::transport_stm32::wire::CmdMotor plain_request(
  const humanoid::transport_stm32::JointRequest & request) noexcept
{
  namespace wire = humanoid::transport_stm32::wire;
  wire::CmdMotor motor{};
  motor.mode_req = static_cast<std::uint8_t>(request.mode);
  motor.flags = wire::kCmdFlagValid;
  if (request.fault_reset) {
    motor.flags = static_cast<std::uint8_t>(motor.flags | wire::kCmdFlagFaultReset);
  }
  if (request.use_config_gains) {
    motor.flags = static_cast<std::uint8_t>(motor.flags | wire::kCmdFlagUseConfigGains);
  }
  return motor;
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(
  humanoid::transport_stm32::Stm32SerialTransport, humanoid::transport::ActuatorTransport)
