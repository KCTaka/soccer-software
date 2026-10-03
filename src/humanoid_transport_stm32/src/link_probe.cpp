// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_stm32/link_probe.hpp"

#include <format>

#include "humanoid_transport_stm32/master_link.hpp"
#include "humanoid_transport_stm32/wire_protocol.hpp"

namespace humanoid::transport_stm32
{

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

  // Rate over a second of frames, measured on the master's own clock. The host's arrival times say
  // nothing: frames queued before the port was opened arrive in one burst, and the first one can
  // be minutes old, left in the device's USB buffer by the previous reader. So it is skipped.
  constexpr std::uint64_t kSkipped = 2;
  constexpr std::uint64_t kSpan = 200;
  const auto from = link.wait_for_frame(link.frame_count() + kSkipped, window);
  const auto to = from ? link.wait_for_frame(link.frame_count() + kSpan, window) : from;
  const auto last_frame = to ? to : from;

  out << std::format(
    "protocol version {} (matches)\nmaster: {}, up {} ms, polls at {} Hz, telemetry {} Hz, "
    "slave tick {} Hz, expects host commands at {} Hz\n",
    wire::kProtocolVersion, wire::robot_state_name(status->robot_state), status->uptime_ms,
    status->master_poll_hz, status->telemetry_hz, status->slave_tick_hz, status->host_cmd_hz);

  if (!last_frame) {
    out << "FAIL: the master answers but sends no telemetry: no slave is responding\n";
    return false;
  }

  if (from && to) {
    const auto cycles = static_cast<std::uint16_t>(
      to->robot.header.cycle_id - from->robot.header.cycle_id);
    const std::uint32_t span_us = to->robot.header.master_time_us -
      from->robot.header.master_time_us;
    out << std::format(
      "telemetry: {} master cycles in {:.3f} s of master time = {:.1f} Hz\n", cycles,
      span_us / 1e6, span_us > 0 ? cycles / (span_us / 1e6) : 0.0);
  } else {
    out << std::format("telemetry: too few frames within {} ms to measure a rate\n",
      window.count());
  }
  const auto counters = link.counters();
  out << std::format(
    "link errors: {} framing, {} duplicate frames; master reset seen: {}\n",
    counters.framing_errors, counters.duplicate_frames,
    link.master_reset_seen() ?
    "YES (possibly before this run: the first frame read can be left over from an earlier boot)" :
    "no");

  const auto & robot = last_frame->robot;
  out << std::format("robot_state={}, {} chain(s) reporting\n",
    wire::robot_state_name(robot.header.robot_state), robot.header.n_chains);
  for (std::size_t c = 0; c < robot.header.n_chains; ++c) {
    const auto & chain = robot.chains[c];
    out << std::format(
      "  chain {}: {} motor(s), slave up {} us, cmd CRC errors {}, CAN TX errors {}\n",
      chain.chain_id, chain.n_motors, chain.slave_time_us, chain.cmd_crc_errors,
      chain.can_tx_errors);
    for (std::size_t m = 0; m < chain.n_motors; ++m) {
      const auto & motor = chain.motors[m];
      out << std::format(
        "    motor {}: {:<11} cause={:<11} drive_fault=0x{:02X} flags=0x{:02X} age={:>3} ms  "
        "pos={:+.4f} rad vel={:+.2f} rad/s tau={:+.2f} N*m temp={} C\n",
        m, wire::lifecycle_name(motor.state), wire::cause_name(motor.cause), motor.motor_fault,
        motor.flags, motor.fb_age_ms, wire::decode_i16(motor.pos, wire::kPosScale),
        wire::decode_i16(motor.vel, wire::kVelScale),
        wire::decode_i16(motor.tau, wire::kTauScale), motor.temp_c);
    }
  }
  out << "OK\n";
  return true;
}

}  // namespace humanoid::transport_stm32
