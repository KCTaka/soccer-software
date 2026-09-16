#include <gtest/gtest.h>
#include "humanoid_transport_mujoco/mujoco_actuator_transport.hpp"
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>

class ConformanceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const char * mjcf_path = std::getenv("HUMANOID_MJCF_PATH");
    if (!mjcf_path) {
      GTEST_SKIP() << "HUMANOID_MJCF_PATH not set";
    }

    std::vector<std::string> joint_names = {
      "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
      "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
      "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
      "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
      "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
      "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
      "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
      "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
    };

    manifest.joint_count = static_cast<std::uint8_t>(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
      std::strncpy(manifest.joints[i].name.data(), joint_names[i].c_str(), humanoid::transport::kMaxNameLength - 1);
      manifest.joints[i].name[humanoid::transport::kMaxNameLength - 1] = '\0';
    }
    safety.joint_count = manifest.joint_count;

    transport = std::make_unique<humanoid::transport_mujoco::MujocoActuatorTransport>();
  }

  humanoid::transport::JointManifest manifest{};
  humanoid::transport::SafetyManifest safety{};
  std::unique_ptr<humanoid::transport::ActuatorTransport> transport;
};

TEST_F(ConformanceTest, C1_ConfigureAcceptsValidManifests)
{
  EXPECT_TRUE(transport->configure(manifest, safety));
}

TEST_F(ConformanceTest, C2_ActivateSucceedsAfterConfigure)
{
  ASSERT_TRUE(transport->configure(manifest, safety));
  EXPECT_TRUE(transport->activate());
}

TEST_F(ConformanceTest, C3_ExchangeReturnsWithinDeadline)
{
  ASSERT_TRUE(transport->configure(manifest, safety));
  ASSERT_TRUE(transport->activate());

  humanoid::transport::CommandBatch cmd{};
  humanoid::transport::FeedbackBatch fb{};
  cmd.joint_count = manifest.joint_count;
  cmd.sequence = 1;

  auto start = std::chrono::steady_clock::now();
  auto res = transport->exchange(cmd, fb, start + std::chrono::milliseconds(100));
  EXPECT_TRUE(res.ok());
}

TEST_F(ConformanceTest, C4_SequenceNumbersAdvanceAndEcho)
{
  ASSERT_TRUE(transport->configure(manifest, safety));
  ASSERT_TRUE(transport->activate());

  humanoid::transport::CommandBatch cmd{};
  humanoid::transport::FeedbackBatch fb{};
  cmd.joint_count = manifest.joint_count;
  cmd.sequence = 42;

  transport->exchange(cmd, fb, std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
  EXPECT_EQ(fb.sequence, 42U);
}

TEST_F(ConformanceTest, C6_CommandBatchApplied_SignFlipTest)
{
  ASSERT_TRUE(transport->configure(manifest, safety));
  ASSERT_TRUE(transport->activate());

  humanoid::transport::CommandBatch cmd{};
  humanoid::transport::FeedbackBatch fb{};
  cmd.joint_count = manifest.joint_count;
  cmd.sequence = 1;

  // command a positive torque on the first joint
  cmd.joints[0].effort_nm = 10.0;
  cmd.joints[0].stiffness_nm_rad = 0.0;
  cmd.joints[0].damping_nm_s_rad = 0.0;

  transport->exchange(cmd, fb, std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
  double initial_pos = fb.joints[0].position_rad;

  // step a few times
  for (int i = 0; i < 10; ++i) {
    transport->exchange(cmd, fb, std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
  }

  EXPECT_GT(fb.joints[0].position_rad, initial_pos);
}