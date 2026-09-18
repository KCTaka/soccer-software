// Copyright 2026 Humanoid Robotics Team
// The ADR-002 command envelope, as data. Generated from model/robot_model.yaml; never hand-edited.
#ifndef HUMANOID_TRANSPORT__SAFETY_MANIFEST_HPP_
#define HUMANOID_TRANSPORT__SAFETY_MANIFEST_HPP_

#include <array>
#include <cstdint>

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
