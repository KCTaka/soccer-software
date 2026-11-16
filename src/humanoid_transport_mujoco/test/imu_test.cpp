// Copyright 2026 UTRA-RoboSoccer

#include <mujoco/mujoco.h>
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <variant>

#include "humanoid_test_support/mujoco_model.hpp"
#include "humanoid_transport_mujoco/mujoco_imu.hpp"

namespace
{

using humanoid::test_support::make_data;
using humanoid::test_support::load_model_xml;
using humanoid::transport_mujoco::find_imu_sensors;
using humanoid::transport_mujoco::ImuSensors;
using humanoid::transport_mujoco::read_imu;

// A free body carrying one IMU site, with `sensors` as the model's sensor block.
std::string model_with(const std::string & sensors)
{
  return "<mujoco><worldbody><body name='base' pos='0 0 1'><freejoint/>"
         "<geom type='box' size='0.1 0.1 0.1' mass='1'/>"
         "<site name='imu' pos='0 0 0'/></body></worldbody><sensor>" + sensors +
         "</sensor></mujoco>";
}

const char kFullImu[] =
  "<framequat objtype='site' objname='imu'/><gyro site='imu'/><accelerometer site='imu'/>";

TEST(Imu, FindsOneOfEachType)
{
  const auto model = load_model_xml(model_with(kFullImu));
  const auto found = find_imu_sensors(*model);
  ASSERT_TRUE(std::holds_alternative<ImuSensors>(found));
  EXPECT_TRUE(std::get<ImuSensors>(found).present());
}

TEST(Imu, ModelWithoutSensorsHasNoImu)
{
  const auto model = load_model_xml(model_with(""));
  const auto found = find_imu_sensors(*model);
  ASSERT_TRUE(std::holds_alternative<ImuSensors>(found));
  EXPECT_FALSE(std::get<ImuSensors>(found).present());
}

TEST(Imu, PartialSetIsRejected)
{
  const auto model = load_model_xml(model_with("<gyro site='imu'/>"));
  EXPECT_TRUE(std::holds_alternative<std::string>(find_imu_sensors(*model)));
}

TEST(Imu, TwoOfOneTypeIsRejected)
{
  const auto model = load_model_xml(model_with(std::string{kFullImu} + "<gyro site='imu'/>"));
  EXPECT_TRUE(std::holds_alternative<std::string>(find_imu_sensors(*model)));
}

// Independent reference: a box resting on the ground, its IMU site rotated 90 degrees about x.
// A resting body's accelerometer reads the ground's reaction to gravity, +g along world up, which
// in the rotated site frame lies on the site's y axis. Its attitude is the rotation we imposed.
TEST(Imu, ReadsAttitudeAndGravityOfAStaticBody)
{
  const auto model = load_model_xml(
    "<mujoco><worldbody><geom type='plane' size='1 1 0.1'/>"
    "<body name='base' pos='0 0 0.1'><freejoint/>"
    "<geom type='box' size='0.1 0.1 0.1' mass='1'/>"
    "<site name='imu' quat='0.70710678 0.70710678 0 0'/></body></worldbody><sensor>" +
    std::string{kFullImu} + "</sensor></mujoco>");
  const auto data = make_data(*model);
  for (int i = 0; i < 2000; ++i) {  // let the contact settle
    mj_step(model.get(), data.get());
  }

  const auto found = find_imu_sensors(*model);
  const auto imu = read_imu(*data, std::get<ImuSensors>(found));

  ASSERT_TRUE(imu.valid);
  // MuJoCo's w-x-y-z (0.7071, 0.7071, 0, 0) arrives as x-y-z-w.
  EXPECT_NEAR(imu.orientation_x, std::sqrt(0.5), 1e-3);
  EXPECT_NEAR(imu.orientation_w, std::sqrt(0.5), 1e-3);
  EXPECT_NEAR(imu.orientation_y, 0.0, 1e-3);
  EXPECT_NEAR(imu.orientation_z, 0.0, 1e-3);
  EXPECT_NEAR(imu.angular_velocity_x, 0.0, 1e-3);
  const double g = -model->opt.gravity[2];
  EXPECT_NEAR(imu.linear_acceleration_y, g, 1e-2);
  EXPECT_NEAR(imu.linear_acceleration_x, 0.0, 1e-2);
  EXPECT_NEAR(imu.linear_acceleration_z, 0.0, 1e-2);
}

}  // namespace
