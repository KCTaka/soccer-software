// Copyright 2026 UTRA-RoboSoccer
// MujocoActuatorTransport as SIL runs it, for the generated robot this test binary is built for
// (HUMANOID_TEST_ROBOT, one binary per robot: CMake's humanoid_generated_robots()).
//
// configure() reads the transport's parameters from the node it creates, so each binary's main
// initialises rclcpp once with them as --ros-args, the way ros2_control_node hands them over.
// rclcpp is initialised once per process, not per test: under rmw_zenoh every init waits for a
// router, up to ZENOH_ROUTER_CHECK_ATTEMPTS seconds when none runs, as in a plain colcon test.
#ifndef SIM_FIXTURE_HPP_
#define SIM_FIXTURE_HPP_

#ifndef HUMANOID_TEST_ROBOT
#error "HUMANOID_TEST_ROBOT is not defined: build this test once per robot"
#endif

#include <mujoco/mujoco.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "humanoid_test_support/generated_robots.hpp"
#include "humanoid_test_support/manifests.hpp"
#include "humanoid_test_support/mujoco_model.hpp"
#include "humanoid_transport/joint_manifest.hpp"
#include "humanoid_transport/safety_manifest.hpp"
#include "humanoid_transport_mujoco/mujoco_actuator_transport.hpp"

namespace humanoid::transport_mujoco::testing
{

/// The robot this binary tests, which CMake only builds it for if it has an MJCF.
inline const test_support::GeneratedRobot & this_robot()
{
  static const auto robot = test_support::generated_robot(HUMANOID_TEST_ROBOT);
  if (!robot || !robot->mjcf_scene()) {
    std::cerr << "no generated MJCF for robot '" HUMANOID_TEST_ROBOT "'\n";
    std::abort();
  }
  return *robot;
}

inline std::filesystem::path this_mjcf() {return *this_robot().mjcf_scene();}

/// The joints the MJCF at `path` drives, in actuator order.
inline std::vector<std::string> actuated_joints(const std::filesystem::path & path)
{
  std::vector<std::string> joints;
  const auto model = test_support::load_model(path);
  if (!model) {
    return joints;
  }
  for (int i = 0; i < model->nu; ++i) {
    if (model->actuator_trntype[i] == mjTRN_JOINT) {
      joints.emplace_back(mj_id2name(model.get(), mjOBJ_JOINT, model->actuator_trnid[2 * i]));
    }
  }
  return joints;
}

/// A transport for this robot and the manifest of every joint its MJCF actuates.
struct SimulatedRobot
{
  transport::JointManifest manifest = test_support::joint_manifest(actuated_joints(this_mjcf()));
  transport::SafetyManifest safety = test_support::safety_manifest(manifest.joint_count);
  MujocoActuatorTransport transport;
};

/// A test binary's main: runs every test with rclcpp initialised once, its global parameter
/// overrides being this robot's mjcf_path and `parameters` ("name:=value").
inline int run_tests(int argc, char ** argv, std::vector<std::string> parameters)
{
  ::testing::InitGoogleTest(&argc, argv);
  parameters.push_back("mjcf_path:=" + this_mjcf().string());
  std::vector<std::string> ros_args{"--ros-args"};
  for (const auto & parameter : parameters) {
    ros_args.push_back("-p");
    ros_args.push_back(parameter);
  }
  std::vector<const char *> args(argv, argv + argc);
  for (const auto & arg : ros_args) {
    args.push_back(arg.c_str());
  }
  rclcpp::init(static_cast<int>(args.size()), args.data());
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

}  // namespace humanoid::transport_mujoco::testing

#endif  // SIM_FIXTURE_HPP_
