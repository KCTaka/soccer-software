// Copyright 2026 UTRA-RoboSoccer
// The ROS-facing path of Stm32SerialTransport: parameter declaration, configure(), and loading the
// plugin through pluginlib the way HumanoidActuatorSystem does.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include <pluginlib/class_loader.hpp>

#include "humanoid_test_support/manifests.hpp"
#include "humanoid_transport/actuator_transport.hpp"
#include "humanoid_transport_stm32/stm32_serial_transport.hpp"
#include "parameters_fixture.hpp"

namespace
{

using humanoid::test_support::joint_manifest;
using humanoid::transport::SafetyManifest;
namespace wire = humanoid::transport_stm32::wire;

SafetyManifest safety_for(std::uint8_t joints)
{
  return humanoid::test_support::safety_manifest(
    joints, humanoid::transport_stm32::testing::wire_safe_envelope());
}

TEST(Stm32Parameters, ConfigureReadsTheDeviceAndTheWiringFromRosParameters)
{
  ASSERT_NE(g_fake_master, nullptr);
  humanoid::transport_stm32::Stm32SerialTransport transport;
  ASSERT_TRUE(transport.configure(joint_manifest({"hip", "knee", "ankle"}), safety_for(3)));
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
  EXPECT_EQ(g_fake_master->state(1, 0), wire::DriveState::DRV_ENABLED);
  transport.deactivate();
}

TEST(Stm32Parameters, ConfigureFailsForAJointWithNoWiringParameters)
{
  // "elbow" has no joints.elbow.* overrides: chain and motor have no default, because a value that
  // merely looked plausible would go unnoticed until the joint ran the wrong way.
  humanoid::transport_stm32::Stm32SerialTransport transport;
  EXPECT_FALSE(transport.configure(joint_manifest({"hip", "elbow"}), safety_for(2)));
}

TEST(Stm32Parameters, ThePluginLoadsThroughPluginlibAsHumanoidActuatorSystemLoadsIt)
{
  pluginlib::ClassLoader<humanoid::transport::ActuatorTransport> loader(
    "humanoid_transport", "humanoid::transport::ActuatorTransport");
  std::shared_ptr<humanoid::transport::ActuatorTransport> transport;
  ASSERT_NO_THROW(
    transport = loader.createSharedInstance("humanoid_transport_stm32/Stm32SerialTransport"));
  ASSERT_TRUE(transport);

  ASSERT_TRUE(transport->configure(joint_manifest({"hip", "knee", "ankle"}), safety_for(3)));
  EXPECT_STREQ(transport->capabilities().implementation_name.data(), "Stm32SerialTransport");
  EXPECT_EQ(transport->capabilities().nominal_cycle_period_us, 5000U);
}

}  // namespace
