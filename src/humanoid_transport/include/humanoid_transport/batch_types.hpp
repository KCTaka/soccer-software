// Copyright 2026 Humanoid Robotics Team
// Fixed-capacity value types crossing the ActuatorTransport boundary (ADR-001).
// Every type here must be trivially copyable: the real-time path copies batches, never allocates.
#ifndef HUMANOID_TRANSPORT__BATCH_TYPES_HPP_
#define HUMANOID_TRANSPORT__BATCH_TYPES_HPP_

#include <algorithm>
#include <array>
#include <bitset>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string_view>
#include <type_traits>

namespace humanoid::transport
{

/// Compile-time capacity. Sized above the 20+ DOF scaling target of ADR-001, not the current build.
inline constexpr std::size_t kMaxJoints = 32;

using JointIndex = std::uint8_t;
using CycleSequence = std::uint64_t;

/// ADR-001 requires monotonic timestamps. steady_clock cannot step; system_clock can.
using MonotonicStamp = std::chrono::steady_clock::time_point;

/// A named scalar field of `Owner`: the name an interface or a manifest key carries, bound to the
/// member that stores it. A table of these spells each name once and cannot pair it with the
/// wrong member, which parallel lists of names and indices can.
template<typename Owner>
struct NamedField
{
  std::string_view name;
  double Owner::* member;
};

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

/// ADR-001: the five command interfaces every controlled joint exports, in storage order.
inline constexpr std::array<NamedField<JointCommand>, 5> kMitFields{{
  {"position", &JointCommand::position_rad},
  {"velocity", &JointCommand::velocity_rad_s},
  {"effort", &JointCommand::effort_nm},
  {"stiffness", &JointCommand::stiffness_nm_rad},
  {"damping", &JointCommand::damping_nm_s_rad},
}};
static_assert(kMitFields.size() * sizeof(double) == sizeof(JointCommand), "a field has no name");

/// True when all five MIT fields are finite. A single NaN or Inf field makes the whole tuple
/// undefined, so callers must reject the tuple, not patch the field.
[[nodiscard]] inline bool is_finite(const JointCommand & c) noexcept
{
  return std::all_of(
    kMitFields.begin(), kMitFields.end(),
    [&c](const NamedField<JointCommand> & field) {return std::isfinite(c.*field.member);});
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

/// ADR-001: the state interfaces every joint exports at minimum, in storage order.
inline constexpr std::array<NamedField<JointFeedback>, 3> kJointStateFields{{
  {"position", &JointFeedback::position_rad},
  {"velocity", &JointFeedback::velocity_rad_s},
  {"effort", &JointFeedback::effort_nm},
}};

/// One reading of the robot's inertial sensor, in the sensor's own frame and ROS conventions
/// (REP-103): orientation is the sensor frame's attitude in the world as an x-y-z-w quaternion,
/// angular velocity in rad/s, linear acceleration in m/s^2 including gravity (a specific force).
struct ImuSample
{
  double orientation_x{0.0};
  double orientation_y{0.0};
  double orientation_z{0.0};
  double orientation_w{1.0};
  double angular_velocity_x{0.0};
  double angular_velocity_y{0.0};
  double angular_velocity_z{0.0};
  double linear_acceleration_x{0.0};
  double linear_acceleration_y{0.0};
  double linear_acceleration_z{0.0};
  /// False means the transport has no IMU, or none answered this cycle. The fields are then
  /// meaningless, and consumers must not read them as a robot at rest.
  bool valid{false};
};
static_assert(std::is_trivially_copyable_v<ImuSample>);

/// The state interfaces an IMU exports, in storage order. The names are the ones
/// imu_sensor_broadcaster reads, and model/generators/emit.py (IMU_INTERFACES) declares the same.
inline constexpr std::array<NamedField<ImuSample>, 10> kImuStateFields{{
  {"orientation.x", &ImuSample::orientation_x},
  {"orientation.y", &ImuSample::orientation_y},
  {"orientation.z", &ImuSample::orientation_z},
  {"orientation.w", &ImuSample::orientation_w},
  {"angular_velocity.x", &ImuSample::angular_velocity_x},
  {"angular_velocity.y", &ImuSample::angular_velocity_y},
  {"angular_velocity.z", &ImuSample::angular_velocity_z},
  {"linear_acceleration.x", &ImuSample::linear_acceleration_x},
  {"linear_acceleration.y", &ImuSample::linear_acceleration_y},
  {"linear_acceleration.z", &ImuSample::linear_acceleration_z},
}};

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
  /// Left at its default (invalid) by a transport with no IMU. A transport that has one sets it
  /// on every successful exchange.
  ImuSample imu{};
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
