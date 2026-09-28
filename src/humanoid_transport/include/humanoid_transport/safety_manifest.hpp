// Copyright 2026 Humanoid Robotics Team
// The ADR-002 command envelope, as data. Hand-written: the key names below are the contract with
// the manifest that model/generators/emit.py writes, and humanoid_actuator_system's
// safety_manifest_parser_test reads the generated file to hold both sides to it.
#ifndef HUMANOID_TRANSPORT__SAFETY_MANIFEST_HPP_
#define HUMANOID_TRANSPORT__SAFETY_MANIFEST_HPP_

#include <array>
#include <cstdint>
#include <string_view>

#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/joint_manifest.hpp"

namespace humanoid::transport
{

/// Per-joint feasible set U_j(x, s). ADR-002 projects the whole MIT tuple into this as one object;
/// clamping the five fields independently does not bound the resulting torque.
struct JointEnvelope
{
  double position_min_rad{0.0};
  double position_max_rad{0.0};
  double velocity_max_rad_s{0.0};
  double torque_continuous_nm{0.0};
  double torque_peak_nm{0.0};
  double torque_peak_duration_s{0.0};
  double stiffness_max_nm_rad{0.0};
  double damping_max_nm_s_rad{0.0};
  double torque_slew_max_nm_s{0.0};
  double power_max_w{0.0};
};

/// Manifest key of each envelope field. Every key is required and no other is accepted.
inline constexpr std::array<NamedField<JointEnvelope>, 10> kEnvelopeFields{{
  {"position_min_rad", &JointEnvelope::position_min_rad},
  {"position_max_rad", &JointEnvelope::position_max_rad},
  {"velocity_max_rad_s", &JointEnvelope::velocity_max_rad_s},
  {"torque_continuous_nm", &JointEnvelope::torque_continuous_nm},
  {"torque_peak_nm", &JointEnvelope::torque_peak_nm},
  {"torque_peak_duration_s", &JointEnvelope::torque_peak_duration_s},
  {"stiffness_max_nm_rad", &JointEnvelope::stiffness_max_nm_rad},
  {"damping_max_nm_s_rad", &JointEnvelope::damping_max_nm_s_rad},
  {"torque_slew_max_nm_s", &JointEnvelope::torque_slew_max_nm_s},
  {"power_max_w", &JointEnvelope::power_max_w},
}};
static_assert(
  kEnvelopeFields.size() * sizeof(double) == sizeof(JointEnvelope), "a field has no key");

/// The manifest format this build reads. ADR-001: compatibility is explicit, never guessed, so
/// any other version is rejected rather than read as if it were this one.
inline constexpr std::string_view kSafetyManifestSchemaVersion = "0.1.0-provisional";

struct SafetyManifest
{
  std::array<JointEnvelope, kMaxJoints> envelopes{};
  std::uint8_t joint_count{0U};
  /// ADR-002: three consecutive missing or invalid 5 ms cycles is command loss.
  std::uint8_t max_consecutive_bad_cycles{3U};
  std::uint32_t feedback_max_age_us{0U};
  Digest manifest_digest{};
};

}  // namespace humanoid::transport

#endif  // HUMANOID_TRANSPORT__SAFETY_MANIFEST_HPP_
