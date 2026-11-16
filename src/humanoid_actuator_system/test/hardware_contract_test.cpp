// Copyright 2026 UTRA-RoboSoccer

#include <gmock/gmock.h>

#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "humanoid_actuator_system/hardware_contract.hpp"
#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/joint_manifest.hpp"

namespace
{

using humanoid::actuator_system::hardware_contract_violation;
using humanoid::actuator_system::kSafetyManifestPackageParam;
using humanoid::actuator_system::kSafetyManifestPathParam;
using ::testing::HasSubstr;
using ::testing::Optional;

hardware_interface::HardwareInfo valid_info()
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters[std::string{kSafetyManifestPathParam}] =
    "robots/unitree_g1/config/safety_manifest.yaml";
  info.hardware_parameters[std::string{kSafetyManifestPackageParam}] = "humanoid_description";
  hardware_interface::ComponentInfo joint;
  joint.name = "left_knee_joint";
  for (const auto & field : humanoid::transport::kMitFields) {
    hardware_interface::InterfaceInfo interface;
    interface.name = std::string{field.name};
    joint.command_interfaces.push_back(interface);
  }
  for (const auto & field : humanoid::transport::kJointStateFields) {
    hardware_interface::InterfaceInfo interface;
    interface.name = std::string{field.name};
    joint.state_interfaces.push_back(interface);
  }
  info.joints.push_back(joint);
  return info;
}

TEST(HardwareContract, HoldsForAValidDescription)
{
  EXPECT_EQ(hardware_contract_violation(valid_info()), std::nullopt);
}

TEST(HardwareContract, RejectsAnUnknownParameter)
{
  // The transport is a deployment choice, a node parameter, not a fact about the robot.
  auto info = valid_info();
  info.hardware_parameters["transport_plugin"] =
    "humanoid_transport_mujoco/MujocoActuatorTransport";
  EXPECT_THAT(hardware_contract_violation(info), Optional(HasSubstr("'transport_plugin'")));
}

TEST(HardwareContract, RequiresEveryParameter)
{
  auto missing = valid_info();
  missing.hardware_parameters.erase(std::string{kSafetyManifestPackageParam});
  EXPECT_NE(hardware_contract_violation(missing), std::nullopt);

  auto empty = valid_info();
  empty.hardware_parameters[std::string{kSafetyManifestPathParam}] = "";
  EXPECT_NE(hardware_contract_violation(empty), std::nullopt);
}

TEST(HardwareContract, RejectsAJointNameThatWouldBeTruncated)
{
  constexpr auto kLongest = humanoid::transport::kMaxNameLength - 1U;
  auto fits = valid_info();
  fits.joints.front().name = std::string(kLongest, 'j');
  EXPECT_EQ(hardware_contract_violation(fits), std::nullopt);

  auto too_long = valid_info();
  too_long.joints.front().name = std::string(kLongest + 1U, 'j');
  EXPECT_NE(hardware_contract_violation(too_long), std::nullopt);
}

TEST(HardwareContract, RequiresBetweenOneAndMaxJoints)
{
  auto none = valid_info();
  none.joints.clear();
  EXPECT_NE(hardware_contract_violation(none), std::nullopt);

  auto too_many = valid_info();
  for (std::size_t i = 1U; i <= humanoid::transport::kMaxJoints; ++i) {
    auto joint = too_many.joints.front();
    joint.name += std::to_string(i);
    too_many.joints.push_back(joint);
  }
  EXPECT_NE(hardware_contract_violation(too_many), std::nullopt);
}

TEST(HardwareContract, RequiresEveryMitCommandInterface)
{
  auto info = valid_info();
  info.joints.front().command_interfaces.pop_back();
  EXPECT_NE(hardware_contract_violation(info), std::nullopt);
}

TEST(HardwareContract, RejectsUnknownOrRepeatedInterfaces)
{
  auto unknown = valid_info();
  unknown.joints.front().command_interfaces.back().name = "torque";
  EXPECT_NE(hardware_contract_violation(unknown), std::nullopt);

  auto repeated = valid_info();
  repeated.joints.front().state_interfaces.back().name = "position";
  EXPECT_NE(hardware_contract_violation(repeated), std::nullopt);
}

hardware_interface::ComponentInfo imu_sensor()
{
  hardware_interface::ComponentInfo sensor;
  sensor.name = "imu";
  for (const auto & field : humanoid::transport::kImuStateFields) {
    hardware_interface::InterfaceInfo interface;
    interface.name = std::string{field.name};
    sensor.state_interfaces.push_back(interface);
  }
  return sensor;
}

TEST(HardwareContract, AcceptsAnImuSensor)
{
  auto info = valid_info();
  info.sensors.push_back(imu_sensor());
  EXPECT_EQ(hardware_contract_violation(info), std::nullopt);
}

TEST(HardwareContract, RejectsAnImuMissingAnInterface)
{
  auto info = valid_info();
  info.sensors.push_back(imu_sensor());
  info.sensors.front().state_interfaces.pop_back();
  EXPECT_THAT(hardware_contract_violation(info), Optional(HasSubstr("'imu'")));
}

TEST(HardwareContract, RejectsAnImuInterfaceItDoesNotKnow)
{
  auto info = valid_info();
  info.sensors.push_back(imu_sensor());
  info.sensors.front().state_interfaces.back().name = "linear_acceleration.w";
  EXPECT_NE(hardware_contract_violation(info), std::nullopt);
}

TEST(HardwareContract, RejectsASecondSensor)
{
  auto info = valid_info();
  info.sensors.push_back(imu_sensor());
  info.sensors.push_back(imu_sensor());
  EXPECT_NE(hardware_contract_violation(info), std::nullopt);
}

}  // namespace
