// Joint identity crossing the transport boundary. Generated from model/robot_model.yaml.
// Fixed-size character arrays, not std::string: this struct is held by the real-time path.
#ifndef HUMANOID_TRANSPORT__JOINT_MANIFEST_HPP_
#define HUMANOID_TRANSPORT__JOINT_MANIFEST_HPP_

#include <array>
#include <cstdint>

#include "humanoid_transport/batch_types.hpp"

namespace humanoid::transport
{

inline constexpr std::size_t kMaxNameLength = 64;
inline constexpr std::size_t kDigestBytes = 32;

using Digest = std::array<std::uint8_t, kDigestBytes>;

struct JointEntry
{
  std::array<char, kMaxNameLength> name{};
  std::uint8_t bus_segment{0U};
  std::uint32_t can_node_id{0U};
  double gear_ratio{0.0};
  /// Separate from the ratio magnitude so a sign error cannot hide inside a gear ratio.
  std::int8_t direction_sign{0};
  double zero_offset_rad{0.0};
};

struct JointManifest
{
  std::array<JointEntry, kMaxJoints> joints{};
  std::uint8_t joint_count{0U};
  /// SHA-256 of model/robot_model.yaml. Participates in the ADR-002 startup identity interlock.
  Digest model_digest{};
};

}  // namespace humanoid::transport

#endif  // HUMANOID_TRANSPORT__JOINT_MANIFEST_HPP_
