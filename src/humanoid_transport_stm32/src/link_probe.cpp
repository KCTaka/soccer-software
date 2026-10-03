// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_stm32/link_probe.hpp"

#include <format>

#include <string_view>

#include "humanoid_transport_stm32/master_link.hpp"
#include "humanoid_transport_stm32/wire_protocol.hpp"

namespace humanoid::transport_stm32
{

namespace
{

std::string_view lifecycle_name(std::uint8_t state);
std::string_view cause_name(std::uint8_t cause);
std::string_view robot_state_name(std::uint8_t state);

}  // namespace

bool probe_master(
  const std::string & device, std::chrono::milliseconds window, std::ostream & out)
{
  MasterLink link;
  if (const auto error = link.open(device)) {
    out << "FAIL: " << *error << "\n";
    return false;
  }
  out << "opened " << device << "; listening for " << window.count()
      << " ms, sending nothing\n";

  const auto status = link.wait_for_status(window);
  if (!status) {
    if (link.foreign_version() != 0) {
      out << std::format(
        "FAIL: the master speaks protocol version {}; this build speaks {}\n",
        link.foreign_version(), wire::kProtocolVersion);
    } else {
      out << std::format(
        "FAIL: no MASTER_STATUS within {} ms. Is the master powered and running firmware built "
        "for PROTO_VERSION {}?\n", window.count(), wire::kProtocolVersion);
    }
    return false;
  }

  const auto start = std::chrono::steady_clock::now();
  const auto first = link.frame_count();
  const auto last_frame = link.wait_for_frame(first + 20, window);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  const auto frames = link.frame_count() - first;

  out << std::format(
    "protocol version {} (matches)\nmaster: {}, up {} ms, polls at {} Hz, telemetry {} Hz, "
    "slave tick {} Hz, expects host commands at {} Hz\n",
    wire::kProtocolVersion, robot_state_name(status->robot_state), status->uptime_ms,
    status->master_poll_hz, status->telemetry_hz, status->slave_tick_hz, status->host_cmd_hz);

  if (!last_frame) {
    out << "FAIL: the master answers but sends no telemetry: no slave is responding\n";
    return false;
  }

  const double seconds = std::chrono::duration<double>(elapsed).count();
  out << std::format("telemetry observed: {} frames in {:.2f} s = {:.1f} Hz\n", frames, seconds,
    seconds > 0.0 ? static_cast<double>(frames) / seconds : 0.0);
  const auto counters = link.counters();
  out << std::format(
    "link errors: {} framing, {} duplicate frames; master reset seen: {}\n",
    counters.framing_errors, counters.duplicate_frames, link.master_reset_seen() ? "YES" : "no");

  const auto & robot = last_frame->robot;
  out << std::format("robot_state={}, {} chain(s) reporting\n",
    robot_state_name(robot.header.robot_state), robot.header.n_chains);
  for (std::size_t c = 0; c < robot.header.n_chains; ++c) {
    const auto & chain = robot.chains[c];
    out << std::format(
      "  chain {}: {} motor(s), slave up {} us, cmd CRC errors {}, CAN TX errors {}\n",
      chain.chain_id, chain.n_motors, chain.slave_time_us, chain.cmd_crc_errors,
      chain.can_tx_errors);
    for (std::size_t m = 0; m < chain.n_motors; ++m) {
      const auto & motor = chain.motors[m];
      out << std::format(
        "    motor {}: {:<11} cause={:<11} drive_fault=0x{:02X} age={:>3} ms  "
        "pos={:+.4f} rad vel={:+.2f} rad/s tau={:+.2f} N*m temp={} C\n",
        m, lifecycle_name(motor.state), cause_name(motor.cause), motor.motor_fault,
        motor.fb_age_ms, wire::decode_i16(motor.pos, wire::kPosScale),
        wire::decode_i16(motor.vel, wire::kVelScale),
        wire::decode_i16(motor.tau, wire::kTauScale), motor.temp_c);
    }
  }
  out << "OK\n";
  return true;
}

namespace
{

std::string_view lifecycle_name(std::uint8_t state)
{
  switch (static_cast<wire::Lifecycle>(state)) {
    case wire::Lifecycle::kBoot: return "BOOT";
    case wire::Lifecycle::kDiscovering: return "DISCOVERING";
    case wire::Lifecycle::kIdle: return "IDLE";
    case wire::Lifecycle::kHold: return "HOLD";
    case wire::Lifecycle::kMit: return "MIT";
    case wire::Lifecycle::kDamped: return "DAMPED";
    case wire::Lifecycle::kToZero: return "TO_ZERO";
    case wire::Lifecycle::kFault: return "FAULT";
  }
  return "UNKNOWN";
}

std::string_view cause_name(std::uint8_t cause)
{
  switch (static_cast<wire::FaultCause>(cause)) {
    case wire::FaultCause::kNone: return "NONE";
    case wire::FaultCause::kOvertorque: return "OVERTORQUE";
    case wire::FaultCause::kCanTimeout: return "CAN_TIMEOUT";
    case wire::FaultCause::kWatchdog: return "WATCHDOG";
    case wire::FaultCause::kMotorFault: return "MOTOR_FAULT";
    case wire::FaultCause::kZeroTimeout: return "ZERO_TIMEOUT";
    case wire::FaultCause::kNotEnabled: return "NOT_ENABLED";
    case wire::FaultCause::kWound: return "WOUND";
    case wire::FaultCause::kMasterLost: return "MASTER_LOST";
  }
  return "UNKNOWN";
}

std::string_view robot_state_name(std::uint8_t state)
{
  switch (static_cast<wire::RobotState>(state)) {
    case wire::RobotState::kInit: return "INIT";
    case wire::RobotState::kReady: return "READY";
    case wire::RobotState::kDegraded: return "DEGRADED";
    case wire::RobotState::kHostLost: return "HOST_LOST";
  }
  return "UNKNOWN";
}

}  // namespace

}  // namespace humanoid::transport_stm32
