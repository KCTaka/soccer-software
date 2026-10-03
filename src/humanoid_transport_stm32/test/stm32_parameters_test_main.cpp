// Copyright 2026 UTRA-RoboSoccer
// Custom main: Stm32SerialTransport::configure() declares ROS parameters on its own node, which
// needs rclcpp::init(). The overrides are handed in as --ros-args, exactly as ros2_control_node
// receives them from the hardware YAML, so the test never reads the environment. The fake master's
// device path is only known at run time, which is why the arguments are built here.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fake_master.hpp"
#include "stm32_parameters_fixture.hpp"

humanoid::transport_stm32::testing::FakeMaster * g_fake_master = nullptr;

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  humanoid::transport_stm32::testing::FakeMaster::Config config;
  config.chain_ids = {0, 1};
  config.motors_per_chain = {2, 1};
  humanoid::transport_stm32::testing::FakeMaster fake(config);
  fake.start();
  g_fake_master = &fake;

  // Joints "hip", "knee" and "ankle" are given; "elbow" is deliberately not.
  const std::vector<std::string> overrides = {
    "serial_device:=" + fake.device(),
    "handshake_timeout_s:=0.5",
    "arming_timeout_s:=1.5",
    "release_timeout_s:=0.5",
    "fault_grace_s:=0.15",
    "joints.hip.chain:=0", "joints.hip.motor:=0", "joints.hip.direction_sign:=1",
    "joints.hip.zero_offset_rad:=0.0",
    "joints.knee.chain:=0", "joints.knee.motor:=1", "joints.knee.direction_sign:=-1",
    "joints.knee.zero_offset_rad:=0.5",
    "joints.ankle.chain:=1", "joints.ankle.motor:=0", "joints.ankle.direction_sign:=1",
    "joints.ankle.zero_offset_rad:=0.0",
  };
  std::vector<const char *> args(argv, argv + argc);
  args.push_back("--ros-args");
  for (const auto & override : overrides) {
    args.push_back("-p");
    args.push_back(override.c_str());
  }
  rclcpp::init(static_cast<int>(args.size()), args.data());

  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  g_fake_master = nullptr;
  return result;
}
