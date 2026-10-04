// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_mujoco/mujoco_imu.hpp"

#include <mujoco/mujoco.h>

#include <format>

#include <array>

namespace humanoid::transport_mujoco
{

std::variant<ImuSensors, std::string> find_imu_sensors(const mjModel_ & model)
{
  constexpr std::array<mjtSensor, 3> kTypes{
    mjtSensor::mjSENS_FRAMEQUAT, mjtSensor::mjSENS_GYRO, mjtSensor::mjSENS_ACCELEROMETER};
  std::array<int, 3> adr{-1, -1, -1};
  std::array<int, 3> count{0, 0, 0};
  for (int s = 0; s < model.nsensor; ++s) {
    for (std::size_t k = 0; k < kTypes.size(); ++k) {
      if (model.sensor_type[s] == kTypes[k]) {
        adr[k] = model.sensor_adr[s];
        ++count[k];
      }
    }
  }

  const int found = (count[0] > 0) + (count[1] > 0) + (count[2] > 0);
  if (found == 0) {
    return ImuSensors{};
  }
  if (found != 3 || count[0] != 1 || count[1] != 1 || count[2] != 1) {
    return std::format(
      "the model has {} framequat, {} gyro and {} accelerometer sensors; an IMU is exactly one "
      "of each", count[0], count[1], count[2]);
  }
  return ImuSensors{
    .orientation_adr = adr[0], .angular_velocity_adr = adr[1], .linear_acceleration_adr = adr[2]};
}

transport::ImuSample read_imu(const mjData_ & data, const ImuSensors & sensors) noexcept
{
  const mjtNum * q = data.sensordata + sensors.orientation_adr;
  const mjtNum * w = data.sensordata + sensors.angular_velocity_adr;
  const mjtNum * a = data.sensordata + sensors.linear_acceleration_adr;
  transport::ImuSample imu;
  imu.orientation_x = q[1];
  imu.orientation_y = q[2];
  imu.orientation_z = q[3];
  imu.orientation_w = q[0];
  imu.angular_velocity_x = w[0];
  imu.angular_velocity_y = w[1];
  imu.angular_velocity_z = w[2];
  imu.linear_acceleration_x = a[0];
  imu.linear_acceleration_y = a[1];
  imu.linear_acceleration_z = a[2];
  imu.valid = true;
  return imu;
}

}  // namespace humanoid::transport_mujoco
