// Copyright 2026 UTRA-RoboSoccer
// The rig most transport tests run against the fake master: three joints on two chains, chain 0
// carrying two motors and chain 1 one. Joint 1 is mirrored and offset, so a missed sign or offset
// shows wherever a value crosses the wire.
#ifndef FAKE_RIG_HPP_
#define FAKE_RIG_HPP_

#include <chrono>
#include <cstdint>
#include <string>

#include "fake_master.hpp"
#include "humanoid_test_support/manifests.hpp"
#include "humanoid_transport/joint_manifest.hpp"
#include "humanoid_transport/safety_manifest.hpp"
#include "humanoid_transport_stm32/stm32_serial_transport.hpp"

namespace humanoid::transport_stm32::testing
{

inline constexpr std::uint8_t kRigJoints = 3;

/// The master: chain 0 with two motors, chain 1 with one.
inline FakeMaster::Config rig_master()
{
  FakeMaster::Config config;
  config.chain_ids = {0, 1};
  config.motors_per_chain = {2, 1};
  return config;
}

/// The transport's parameters for the rig on `device`, with timeouts short enough for a test.
inline Stm32SerialTransport::Parameters rig_parameters(const std::string & device)
{
  using std::chrono::milliseconds;
  Stm32SerialTransport::Parameters params;
  params.serial_device = device;
  params.handshake_timeout = milliseconds{500};
  params.discovery_timeout = milliseconds{500};
  params.arming_timeout = milliseconds{1500};
  params.release_timeout = milliseconds{500};
  params.fault_grace = milliseconds{150};
  params.wiring = {
    {.chain = 0, .motor = 0},
    {.chain = 0, .motor = 1, .direction_sign = -1, .zero_offset_rad = 0.5},
    {.chain = 1, .motor = 0}};
  return params;
}

/// A joint envelope inside every wire range.
inline transport::JointEnvelope wire_safe_envelope()
{
  transport::JointEnvelope e;
  e.position_min_rad = -1.5;
  e.position_max_rad = 1.5;
  e.velocity_max_rad_s = 20.0;
  e.torque_peak_nm = 30.0;
  e.stiffness_max_nm_rad = 100.0;
  e.damping_max_nm_s_rad = 5.0;
  return e;
}

inline transport::JointManifest rig_manifest()
{
  return test_support::joint_manifest({"hip", "knee", "ankle"});
}

inline transport::SafetyManifest rig_safety()
{
  return test_support::safety_manifest(kRigJoints, wire_safe_envelope());
}

}  // namespace humanoid::transport_stm32::testing

#endif  // FAKE_RIG_HPP_
