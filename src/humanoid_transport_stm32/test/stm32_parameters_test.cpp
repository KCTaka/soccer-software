// Copyright 2026 UTRA-RoboSoccer
// The ROS-facing path of Stm32SerialTransport: parameter declaration, configure(), and loading the
// plugin through pluginlib the way HumanoidActuatorSystem does.

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <memory>

#include <pluginlib/class_loader.hpp>

#include "humanoid_transport/actuator_transport.hpp"
#include "humanoid_transport_stm32/stm32_serial_transport.hpp"
#include "stm32_parameters_fixture.hpp"

namespace
{

using humanoid::transport::JointManifest;
using humanoid::transport::SafetyManifest;
namespace wire = humanoid::transport_stm32::wire;

JointManifest manifest_of(std::initializer_list<const char *> names)
{
  JointManifest manifest{};
  std::uint8_t i = 0;
  for (const char * name : names) {
    std::strncpy(
      manifest.joints[i++].name.data(), name, humanoid::transport::kMaxNameLength - 1);
  }
  manifest.joint_count = i;
  return manifest;
}

SafetyManifest safety_for(std::uint8_t joints)
{
  SafetyManifest safety{};
  safety.joint_count = joints;
  safety.feedback_max_age_us = 15000;
  for (std::uint8_t i = 0; i < joints; ++i) {
    auto & e = safety.envelopes[i];
    e.position_min_rad = -1.5;
    e.position_max_rad = 1.5;
    e.velocity_max_rad_s = 20.0;
    e.torque_peak_nm = 30.0;
    e.stiffness_max_nm_rad = 100.0;
    e.damping_max_nm_s_rad = 5.0;
  }
  return safety;
}

TEST(Stm32Parameters, ConfigureReadsTheDeviceAndTheWiringFromRosParameters)
{
  ASSERT_NE(g_fake_master, nullptr);
  humanoid::transport_stm32::Stm32SerialTransport transport;
  ASSERT_TRUE(transport.configure(manifest_of({"hip", "knee", "ankle"}), safety_for(3)));
  ASSERT_TRUE(transport.activate());

  // The parameters took effect: knee is on chain 0 motor 1, mirrored, with a 0.5 rad offset.
  humanoid::transport::CommandBatch command{};
  humanoid::transport::FeedbackBatch feedback{};
  command.joint_count = 3;
  command.sequence = 1;
  command.joints[1].position_rad = 0.3;
  for (int i = 0; i < 5; ++i) {
    ++command.sequence;
    ASSERT_TRUE(transport.exchange(
        command, feedback, std::chrono::steady_clock::now() + std::chrono::milliseconds{100}).ok());
  }
  const auto sent = g_fake_master->last_command();
  ASSERT_TRUE(sent.has_value());
  EXPECT_EQ(sent->chains[0].motors[1].pos, 2000);  // -1 * 0.3 + 0.5 = 0.2 rad
  EXPECT_EQ(g_fake_master->state(1, 0), wire::Lifecycle::kMit);
  transport.deactivate();
}

TEST(Stm32Parameters, ConfigureFailsForAJointWithNoWiringParameters)
{
  // "elbow" has no joints.elbow.* overrides: chain and motor have no default, because a value that
  // merely looked plausible would go unnoticed until the joint ran the wrong way.
  humanoid::transport_stm32::Stm32SerialTransport transport;
  EXPECT_FALSE(transport.configure(manifest_of({"hip", "elbow"}), safety_for(2)));
}

TEST(Stm32Parameters, ThePluginLoadsThroughPluginlibAsHumanoidActuatorSystemLoadsIt)
{
  pluginlib::ClassLoader<humanoid::transport::ActuatorTransport> loader(
    "humanoid_transport", "humanoid::transport::ActuatorTransport");
  std::shared_ptr<humanoid::transport::ActuatorTransport> transport;
  ASSERT_NO_THROW(
    transport = loader.createSharedInstance("humanoid_transport_stm32/Stm32SerialTransport"));
  ASSERT_TRUE(transport);

  ASSERT_TRUE(transport->configure(manifest_of({"hip", "knee", "ankle"}), safety_for(3)));
  EXPECT_STREQ(transport->capabilities().implementation_name.data(), "Stm32SerialTransport");
  EXPECT_EQ(transport->capabilities().nominal_cycle_period_us, 5000U);
}

}  // namespace
