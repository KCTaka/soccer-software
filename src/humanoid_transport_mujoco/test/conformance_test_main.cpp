// Copyright 2026 UTRA-RoboSoccer

#include <gtest/gtest.h>

#include <vector>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // Append the model path as a global ROS parameter override, so the
  // transport's mujoco_sim node reads it the same way it does in a launch.
  std::vector<const char *> args(argv, argv + argc);
  args.insert(args.end(), {"--ros-args", "-p", "mjcf_path:=" HUMANOID_TEST_MJCF_PATH});
  rclcpp::init(static_cast<int>(args.size()), args.data());

  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
