// Copyright 2026 UTRA-RoboSoccer
// Each linkage the component builds from a generated description, against the plant SIL runs: the
// robot's generated MJCF, whose loop MuJoCo closes with its own kinematic tree. A sign, zero or
// length the two disagree on shows up here as a pin that does not meet, and the ratio MuJoCo's
// constraint forces carry statically must be the linkage's.

#include <gmock/gmock.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <variant>

#include "generated_description.hpp"
#include "humanoid_test_support/generated_robots.hpp"
#include "humanoid_test_support/matchers.hpp"
#include "humanoid_test_support/mujoco_model.hpp"

namespace
{

using humanoid::actuator_system::testing::generated_linkages_where;
using humanoid::actuator_system::testing::GeneratedLinkage;
using humanoid::actuator_system::testing::LinkageName;
using humanoid::actuator_system::testing::load;
using humanoid::actuator_system::testing::LoadedLinkage;
using humanoid::actuator_system::testing::samples;
using humanoid::test_support::DataPtr;
using humanoid::test_support::GeneratedRobot;
using humanoid::test_support::HoldsValue;
using humanoid::test_support::ModelPtr;

class FourBarAgainstMujoco : public ::testing::TestWithParam<GeneratedLinkage>
{
protected:
  void SetUp() override
  {
    auto loaded = load(GetParam());
    ASSERT_THAT(loaded, HoldsValue());
    l_.emplace(std::get<LoadedLinkage>(std::move(loaded)));
    model_ = humanoid::test_support::load_model(*GetParam().robot.mjcf_scene());
    ASSERT_NE(model_, nullptr);
    data_ = humanoid::test_support::make_data(*model_);
    // model/generators/mjcf.py names the closure's sites after the linkage.
    coupler_pin_ = site(l_->transmission.name + "_coupler_pin");
    input_pin_ = site(l_->transmission.name + "_input_pin");
  }

  int site(const std::string & name) const
  {
    const int id = mj_name2id(model_.get(), mjOBJ_SITE, name.c_str());
    EXPECT_GE(id, 0) << "no site " << name;
    return id;
  }

  [[nodiscard]] int qpos(const std::string & joint) const
  {
    return model_->jnt_qposadr[mj_name2id(model_.get(), mjOBJ_JOINT, joint.c_str())];
  }

  [[nodiscard]] int dof(const std::string & joint) const
  {
    return model_->jnt_dofadr[mj_name2id(model_.get(), mjOBJ_JOINT, joint.c_str())];
  }

  /// Puts the linkage at `output` by the component's kinematics, from rest.
  void pose(double output)
  {
    const auto c = l_->linkage().from_output(output);
    mj_resetData(model_.get(), data_.get());
    data_->qpos[qpos(l_->output_joint())] = output;
    data_->qpos[qpos(l_->input_joint())] = c.input_rad;
    data_->qpos[qpos(l_->coupler_joint())] = c.coupler_rad;
  }

  /// The distance between the closure's two pin sites.
  [[nodiscard]] double pin_distance() const
  {
    const auto * a = &data_->site_xpos[3 * coupler_pin_];
    const auto * b = &data_->site_xpos[3 * input_pin_];
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  }

  std::optional<LoadedLinkage> l_;
  ModelPtr model_;
  DataPtr data_;
  int coupler_pin_{-1};
  int input_pin_{-1};
};

TEST_P(FourBarAgainstMujoco, ClosesTheMjcfLoopOverTheJointsRange)
{
  for (const double output : samples(l_->lower, l_->upper, 101)) {
    pose(output);
    mj_kinematics(model_.get(), data_.get());
    EXPECT_LT(pin_distance(), 1e-9) << "at output " << output << " rad";
  }
}

TEST_P(FourBarAgainstMujoco, CarriesTheMotorsTorqueByItsRatio)
{
  // Gravity off, the motor pushing with a constant torque, a stiff spring holding the output at
  // its setpoint: at rest the spring's torque is the motor's torque divided by the ratio, as
  // MuJoCo's constraint forces carry it round the loop. The spring stands in for a load.
  const auto & output = l_->output_joint();
  constexpr double kMotorTorque = 2.0;
  constexpr double kSpring = 2000.0;
  for (const double setpoint : {l_->lower, 0.0, l_->upper}) {
    pose(setpoint);
    model_->opt.gravity[2] = 0.0;
    double spring_torque = 0.0;
    for (int step = 0; step < 4000; ++step) {
      const double q = data_->qpos[qpos(output)];
      spring_torque = -kSpring * (q - setpoint) - 2.0 * data_->qvel[dof(output)];
      mju_zero(data_->qfrc_applied, model_->nv);
      data_->qfrc_applied[dof(l_->input_joint())] = kMotorTorque * std::min(1.0, step / 500.0);
      data_->qfrc_applied[dof(output)] = spring_torque;
      mj_step(model_.get(), data_.get());
    }
    const double ratio = l_->linkage().from_output(data_->qpos[qpos(output)]).ratio;
    EXPECT_NEAR(-spring_torque, kMotorTorque / ratio, 1e-3 * kMotorTorque) << "at " << setpoint;
    EXPECT_LT(pin_distance(), 1e-5) << "the pin stretched at " << setpoint;
  }
}

INSTANTIATE_TEST_SUITE_P(
  EverySimulatedLinkage, FourBarAgainstMujoco,
  ::testing::ValuesIn(
    generated_linkages_where(
      [](const GeneratedRobot & robot) {return robot.mjcf_scene().has_value();})),
  LinkageName{});

}  // namespace
