// Copyright 2026 UTRA-RoboSoccer

#include <mujoco/mujoco.h>
#include <gtest/gtest.h>

#include <memory>

#include "humanoid_transport_mujoco/mujoco_disturbance.hpp"

namespace
{

using humanoid::transport_mujoco::MujocoDisturbance;
using humanoid::transport_mujoco::PushConfig;

class DisturbanceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char error[1024] = {};
    model_.reset(mj_loadXML(HUMANOID_TEST_MJCF_PATH, nullptr, error, sizeof(error)));
    ASSERT_NE(model_, nullptr) << error;
    data_.reset(mj_makeData(model_.get()));
    body_ = mj_name2id(model_.get(), mjtObj::mjOBJ_BODY, "torso_link");
    ASSERT_GE(body_, 0);
  }

  double force_y() const {return data_->xfrc_applied[6 * body_ + 1];}

  struct ModelDeleter {void operator()(mjModel * m) const {mj_deleteModel(m);}};
  struct DataDeleter {void operator()(mjData * d) const {mj_deleteData(d);}};
  std::unique_ptr<mjModel, ModelDeleter> model_;
  std::unique_ptr<mjData, DataDeleter> data_;
  int body_{-1};
};

TEST_F(DisturbanceTest, AppliesForceOnlyInsideWindow)
{
  MujocoDisturbance disturbance;
  const PushConfig config{.force_n = 30.0, .start_time_s = 1.0, .duration_s = 0.2};
  ASSERT_FALSE(disturbance.configure(*model_, config).has_value());

  data_->time = 0.5;
  disturbance.apply(*data_);
  EXPECT_DOUBLE_EQ(force_y(), 0.0);

  data_->time = 1.1;
  disturbance.apply(*data_);
  EXPECT_DOUBLE_EQ(force_y(), 30.0);

  data_->time = 1.2;  // end is exclusive
  disturbance.apply(*data_);
  EXPECT_DOUBLE_EQ(force_y(), 0.0);

  data_->time = 1.1;  // simulation reset re-arms the pulse
  disturbance.apply(*data_);
  EXPECT_DOUBLE_EQ(force_y(), 30.0);
}

TEST_F(DisturbanceTest, NormalisesDirection)
{
  MujocoDisturbance disturbance;
  const PushConfig config{.force_n = 10.0, .direction = {0.0, 3.0, 4.0}};
  ASSERT_FALSE(disturbance.configure(*model_, config).has_value());
  disturbance.apply(*data_);
  EXPECT_NEAR(data_->xfrc_applied[6 * body_ + 1], 6.0, 1e-12);
  EXPECT_NEAR(data_->xfrc_applied[6 * body_ + 2], 8.0, 1e-12);
}

TEST_F(DisturbanceTest, ZeroForceIsDisabledAndTouchesNothing)
{
  MujocoDisturbance disturbance;
  ASSERT_FALSE(disturbance.configure(*model_, PushConfig{.body = "no_such_body"}).has_value());
  data_->xfrc_applied[6 * body_ + 1] = 7.0;
  disturbance.apply(*data_);
  EXPECT_DOUBLE_EQ(force_y(), 7.0);
}

TEST_F(DisturbanceTest, RejectsInvalidConfig)
{
  MujocoDisturbance disturbance;
  EXPECT_TRUE(
    disturbance.configure(*model_, PushConfig{.force_n = 1.0, .body = "no_such_body"}).has_value());
  EXPECT_TRUE(
    disturbance.configure(*model_, PushConfig{.force_n = 1.0, .direction = {0.0, 0.0, 0.0}})
    .has_value());
  EXPECT_TRUE(
    disturbance.configure(*model_, PushConfig{.force_n = 1.0, .duration_s = 0.0}).has_value());
  EXPECT_TRUE(disturbance.configure(*model_, PushConfig{.force_n = -5.0}).has_value());
}

}  // namespace
