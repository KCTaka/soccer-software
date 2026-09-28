// Copyright 2026 UTRA-RoboSoccer
// The control-cycle period: the one place it is defined.
//
// ADR-001-03 fixes a genuine 200 Hz exchange: fresh command and feedback for
// every joint every 5 ms, physically scheduled by the actuator-side hardware.
// It is an architectural invariant, not a tuning parameter: a different rate
// is a new qualification decision, not a configuration change. It is
// therefore a compile-time constant, and every runtime rate setting is
// checked against it rather than trusted:
//   - controller_manager's update rate, seen by the hardware component as
//     HardwareInfo::rw_rate, must equal kControlRateHz;
//   - each transport reports its actual cycle in
//     TransportCapabilities::nominal_cycle_period_us, which must equal
//     kControlPeriod.
#ifndef HUMANOID_TRANSPORT__TIMING_HPP_
#define HUMANOID_TRANSPORT__TIMING_HPP_

#include <chrono>
#include <cstdint>

namespace humanoid::transport
{

inline constexpr std::uint32_t kControlRateHz = 200U;

inline constexpr std::chrono::microseconds kControlPeriod{
  std::chrono::microseconds{std::chrono::seconds{1}} / kControlRateHz};

static_assert(
  kControlPeriod * kControlRateHz == std::chrono::seconds{1},
  "the control period must be an integer number of microseconds");

}  // namespace humanoid::transport

#endif  // HUMANOID_TRANSPORT__TIMING_HPP_
