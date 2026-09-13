// Fixed-capacity value types crossing the ActuatorTransport boundary (ADR-001).
// Every type here must be trivially copyable: the real-time path copies batches, never allocates.
#ifndef HUMANOID_TRANSPORT__BATCH_TYPES_HPP_
#define HUMANOID_TRANSPORT__BATCH_TYPES_HPP_

#include <array>
#include <bitset>
#include <chrono>
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

  [[nodiscard]] constexpr bool ok() const noexcept { return error == TransportError::kNone; }
};
static_assert(std::is_trivially_copyable_v<ExchangeResult>);

/// Copied out by a non-real-time thread. Never read inside the cycle for control decisions.
struct HealthSnapshot
{
  CycleSequence last_sequence{0U};
  std::uint64_t exchanges_attempted{0U};
  std::uint64_t exchanges_failed{0U};
  std::uint64_t deadline_misses{0U};
  std::uint64_t framing_errors{0U};
  std::uint64_t sequence_rejections{0U};
  std::chrono::nanoseconds worst_round_trip{0};
  std::uint32_t availability_epoch{0U};
  bool active{false};
};

}  // namespace humanoid::transport

#endif  // HUMANOID_TRANSPORT__BATCH_TYPES_HPP_
