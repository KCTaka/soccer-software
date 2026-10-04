// Copyright 2026 UTRA-RoboSoccer
// The robot's IMU in a loaded MuJoCo model, found by sensor type.
//
// The generated MJCF carries one framequat, one gyro and one accelerometer on the IMU site
// (model/generators/emit.py). They are found by type, never by a conventional name: the structure
// says what they are, and a model that carries two of a type is ambiguous, not "the first one".
#ifndef HUMANOID_TRANSPORT_MUJOCO__MUJOCO_IMU_HPP_
#define HUMANOID_TRANSPORT_MUJOCO__MUJOCO_IMU_HPP_

#include <string>
#include <variant>

#include "humanoid_transport/batch_types.hpp"

struct mjModel_;
struct mjData_;

namespace humanoid::transport_mujoco
{

/// Offsets into mjData::sensordata. All -1 when the model has no IMU.
struct ImuSensors
{
  int orientation_adr{-1};          // framequat, w-x-y-z
  int angular_velocity_adr{-1};     // gyro, 3 values
  int linear_acceleration_adr{-1};  // accelerometer, 3 values

  [[nodiscard]] bool present() const noexcept {return orientation_adr >= 0;}
};

/// The IMU sensors of `model`: all three present, none present (a model without an IMU), or the
/// reason the model is ambiguous: a partial set, or more than one sensor of a type.
[[nodiscard]] std::variant<ImuSensors, std::string> find_imu_sensors(const mjModel_ & model);

/// Copies the IMU reading in `data` (as of its last forward pass) into a valid sample. The
/// orientation is reordered from MuJoCo's w-x-y-z to REP-103's x-y-z-w. Real-time safe.
[[nodiscard]] transport::ImuSample read_imu(
  const mjData_ & data, const ImuSensors & sensors) noexcept;

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__MUJOCO_IMU_HPP_
