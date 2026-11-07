// Copyright 2026 UTRA-RoboSoccer
// The linkage the component builds from the generated description against the plant SIL runs:
// the generated MJCF, whose loop MuJoCo closes with its own kinematic tree. A sign, zero or length
// the two disagree on shows up here as a pin that does not meet, and the ratio MuJoCo's constraint
// forces carry statically must be the linkage's.

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <variant>

#include "hardware_interface/component_parser.hpp"
#include "humanoid_actuator_system/drive_layout.hpp"

namespace
{

using humanoid::actuator_system::drive_layout;
using humanoid::actuator_system::DriveLayout;

struct ModelDeleter
{
  void operator()(mjModel * m) const {mj_deleteModel(m);}
};
struct DataDeleter
{
  void operator()(mjData * d) const {mj_deleteData(d);}
};

class FourBarAgainstMujoco : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::ifstream file(HUMANOID_TEST_LEG_URDF_PATH);
    std::stringstream urdf;
    urdf << file.rdbuf();
    const auto info = hardware_interface::parse_control_resources_from_urdf(urdf.str()).front();
    auto layout = drive_layout(info);
    ASSERT_TRUE(std::holds_alternative<DriveLayout>(layout));
    layout_ = std::get<DriveLayout>(layout);
    const auto & limits = info.limits.at(layout_.command_joints.front());
    lower_ = limits.min_position;
    upper_ = limits.max_position;

    char error[1000] = {};
    model_.reset(mj_loadXML(HUMANOID_TEST_LEG_MJCF_PATH, nullptr, error, sizeof(error)));
    ASSERT_TRUE(model_) << error;
    data_.reset(mj_makeData(model_.get()));
  }

  [[nodiscard]] int qpos(const std::string & joint) const
  {
    return model_->jnt_qposadr[mj_name2id(model_.get(), mjOBJ_JOINT, joint.c_str())];
  }

  [[nodiscard]] int dof(const std::string & joint) const
  {
    return model_->jnt_dofadr[mj_name2id(model_.get(), mjOBJ_JOINT, joint.c_str())];
  }

  /// The two closure-pin sites' distance with the linkage set by the component's kinematics.
  [[nodiscard]] double pin_gap(double output)
  {
    const auto & linkage = layout_.linkages.front()->linkage;
    const auto c = linkage.from_output(output);
    mj_resetData(model_.get(), data_.get());
    data_->qpos[qpos(layout_.command_joints.front())] = output;
    data_->qpos[qpos(layout_.passive_joints[0])] = c.input_rad;
    data_->qpos[qpos(layout_.passive_joints[1])] = c.coupler_rad;
    mj_kinematics(model_.get(), data_.get());
    return pin_distance();
  }

  [[nodiscard]] double pin_distance() const
  {
    const int a = mj_name2id(model_.get(), mjOBJ_SITE, "left_ankle_pitch_linkage_coupler_pin");
    const int b = mj_name2id(model_.get(), mjOBJ_SITE, "left_ankle_pitch_linkage_input_pin");
    return std::hypot(
      data_->site_xpos[3 * a] - data_->site_xpos[3 * b],
      data_->site_xpos[3 * a + 1] - data_->site_xpos[3 * b + 1],
      data_->site_xpos[3 * a + 2] - data_->site_xpos[3 * b + 2]);
  }

  DriveLayout layout_;
  double lower_{0.0};
  double upper_{0.0};
  std::unique_ptr<mjModel, ModelDeleter> model_;
  std::unique_ptr<mjData, DataDeleter> data_;
};

TEST_F(FourBarAgainstMujoco, ClosesTheMjcfLoopOverTheJointsRange)
{
  for (int k = 0; k <= 100; ++k) {
    const double output = lower_ + (upper_ - lower_) * k / 100.0;
    EXPECT_LT(pin_gap(output), 1e-9) << "at output " << output << " rad";
  }
}

TEST_F(FourBarAgainstMujoco, CarriesTheMotorsTorqueByItsRatio)
{
  // Gravity off, the motor pushing with a constant torque, a stiff spring holding the ankle at its
  // setpoint: at rest the spring's torque is the motor's torque divided by the ratio, as MuJoCo's
  // constraint forces carry it round the loop. The spring stands in for a load on the foot.
  const auto & linkage = layout_.linkages.front()->linkage;
  const std::string ankle = layout_.command_joints.front();
  const std::string motor = layout_.passive_joints[0];
  const std::string rod = layout_.passive_joints[1];
  constexpr double kMotorTorque = 2.0;
  constexpr double kSpring = 2000.0;
  for (const double setpoint : {lower_, 0.0, upper_}) {
    mj_resetData(model_.get(), data_.get());
    model_->opt.gravity[2] = 0.0;
    const auto c = linkage.from_output(setpoint);
    data_->qpos[qpos(ankle)] = setpoint;
    data_->qpos[qpos(motor)] = c.input_rad;
    data_->qpos[qpos(rod)] = c.coupler_rad;
    double spring_torque = 0.0;
    for (int step = 0; step < 4000; ++step) {
      const double q = data_->qpos[qpos(ankle)];
      spring_torque = -kSpring * (q - setpoint) - 2.0 * data_->qvel[dof(ankle)];
      mju_zero(data_->qfrc_applied, model_->nv);
      data_->qfrc_applied[dof(motor)] = kMotorTorque * std::min(1.0, step / 500.0);
      data_->qfrc_applied[dof(ankle)] = spring_torque;
      mj_step(model_.get(), data_.get());
    }
    const double ratio = linkage.from_output(data_->qpos[qpos(ankle)]).ratio;
    EXPECT_NEAR(-spring_torque, kMotorTorque / ratio, 1e-3 * kMotorTorque) << "at " << setpoint;
    EXPECT_LT(pin_distance(), 1e-5) << "the pin stretched at " << setpoint;
  }
}

}  // namespace
