// Copyright 2026 Humanoid Robotics Team
// Fixed-capacity value types crossing the ActuatorTransport boundary (ADR-001).
// Every type here must be trivially copyable: the real-time path copies batches, never allocates.
#ifndef HUMANOID_TRANSPORT__BATCH_TYPES_HPP_
#define HUMANOID_TRANSPORT__BATCH_TYPES_HPP_

#include <array>
#include <bitset>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace humanoid::transport
{

/// Compile-time capacity. Sized above the 20+ DOF scaling target of ADR-001, not the current build.
inline constexpr std::size_t kMaxJoints = 32;

using JointIndex = std::uint8_t;
using CycleSequence = std::uint64_t;

/// ADR-001 requires monotonic timestamps. steady_clock cannot step; system_clock can.
using MonotonicStamp = std::chrono::steady_clock::time_point;

/// The five MIT fields. One controller owns all five for every joint it controls (ADR-001).
struct JointCommand
{
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double effort_nm{0.0};
  double stiffness_nm_rad{0.0};
  double damping_nm_s_rad{0.0};
};
static_assert(std::is_trivially_copyable_v<JointCommand>);

/// True when all five MIT fields are finite. A single NaN or Inf field makes the whole tuple
/// undefined, so callers must reject the tuple, not patch the field.
[[nodiscard]] inline bool is_finite(const JointCommand & c) noexcept
{
  return std::isfinite(c.position_rad) && std::isfinite(c.velocity_rad_s) &&
         std::isfinite(c.effort_nm) && std::isfinite(c.stiffness_nm_rad) &&
         std::isfinite(c.damping_nm_s_rad);
}

struct JointFeedback
{
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double effort_nm{0.0};
  float temperature_c{0.0F};
  float bus_voltage_v{0.0F};
  std::uint16_t fault_bits{0U};
  /// False means this joint contributed no new sample to this cycle. Not the same as zero.
  bool fresh{false};
};
static_assert(std::is_trivially_copyable_v<JointFeedback>);

struct CommandBatch
{
  CycleSequence sequence{0U};
  MonotonicStamp stamp{};
  std::uint8_t joint_count{0U};
  /// Bit set means the joint is commandable this cycle. ADR-002 requires explicit acknowledgement
  /// of any change, which is what availability_epoch carries.
  std::bitset<kMaxJoints> availability_mask{};
  std::uint32_t availability_epoch{0U};
  std::array<JointCommand, kMaxJoints> joints{};
};

struct FeedbackBatch
{
  CycleSequence sequence{0U};
  MonotonicStamp stamp{};
  std::uint8_t joint_count{0U};
  std::bitset<kMaxJoints> availability_mask{};
  std::uint32_t availability_epoch{0U};
  std::array<JointFeedback, kMaxJoints> joints{};
};

enum class TransportError : std::uint8_t
{
  kNone = 0,
  kTimeout,
  kFraming,
  kSequenceMismatch,
  kManifestMismatch,
  kIncompleteBatch,
  kNotActive,
  kHardwareFault
};

struct ExchangeResult
{
  TransportError error{TransportError::kNone};
  std::uint8_t joints_reported{0U};
  std::chrono::nanoseconds round_trip{0};

  [[nodiscard]] constexpr bool ok() const noexcept {return error == TransportError::kNone;}
};
static_assert(std::is_trivially_copyable_v<ExchangeResult>);

/// State only the transport itself can observe. Per-exchange accounting (attempts, failures,
/// deadline misses, durations, last sequence) is derivable from each exchange() call and is kept
/// once by the caller in ExchangeStats, not re-implemented here by every transport.
///
/// Plain values, unsynchronised: read it from the thread that calls exchange(), or through a
/// hand-off, never directly from another thread. Never used for control decisions in-cycle.
struct HealthSnapshot
{
  /// Frames or packets the transport discarded as corrupt before they became a batch.
  std::uint64_t framing_errors{0U};
  /// Batches the transport rejected as duplicate, stale, or out of order.
  std::uint64_t sequence_rejections{0U};
  std::uint32_t availability_epoch{0U};
  bool active{false};
};
static_assert(std::is_trivially_copyable_v<HealthSnapshot>);

}  // namespace humanoid::transport

#endif  // HUMANOID_TRANSPORT__BATCH_TYPES_HPP_
