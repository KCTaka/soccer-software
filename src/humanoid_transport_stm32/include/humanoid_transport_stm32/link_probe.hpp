// Copyright 2026 UTRA-RoboSoccer
// A read-only description of what a master STM32 is reporting.
//
// For the first contact with real hardware: it opens the port and listens. It never sends a byte,
// so it cannot arm, move, or disable anything, and it is safe to run against a robot that is
// powered and in any state.
#ifndef HUMANOID_TRANSPORT_STM32__LINK_PROBE_HPP_
#define HUMANOID_TRANSPORT_STM32__LINK_PROBE_HPP_

#include <chrono>
#include <ostream>
#include <string>

namespace humanoid::transport_stm32
{

/// Opens `device`, listens, and writes a report to `out`: the master's protocol version and
/// configured rates, the telemetry rate over about a second measured on the master's own clock,
/// link error counters, and each chain's motors with state, latched cause, drive fault bits,
/// telemetry flags, drive report age and measured values in WIRE units (no joint transform
/// applied). `window` bounds each wait. Returns whether a compatible master answered.
[[nodiscard]] bool probe_master(
  const std::string & device, std::chrono::milliseconds window, std::ostream & out);

}  // namespace humanoid::transport_stm32

#endif  // HUMANOID_TRANSPORT_STM32__LINK_PROBE_HPP_
