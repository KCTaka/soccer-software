// Copyright 2026 UTRA-RoboSoccer

#include <mujoco/mujoco.h>
#include <gmock/gmock.h>

#include <string>
#include <variant>
#include <vector>

#include "humanoid_test_support/manifests.hpp"
#include "humanoid_test_support/matchers.hpp"
#include "humanoid_test_support/mujoco_model.hpp"
#include "humanoid_transport_mujoco/mujoco_joints.hpp"

namespace
{

using humanoid::test_support::HoldsError;
using humanoid::test_support::HoldsValue;
using humanoid::test_support::joint_manifest;
using humanoid::test_support::load_model_xml;
using humanoid::test_support::make_data;
using humanoid::transport_mujoco::InitialPose;
using humanoid::transport_mujoco::JointDrives;
using humanoid::transport_mujoco::map_joint_drives;
using humanoid::transport_mujoco::resolve_initial_pose;
using ::testing::HasSubstr;

// The actuator model/generators emits for a commanded joint.
const char kMitActuator[] =
  "gaintype='fixed' gainprm='1' biastype='affine' biasprm='0 0 0' forcelimited='true' "
  "forcerange='-10 10'";

// A floating box on a plane with hinge `a` (range +-1 rad), hinge `b` on a second tree, and
// `actuators` as the model's actuator block.
std::string model_with(const std::string & actuators)
{
  return "<mujoco><compiler angle='radian'/><worldbody><geom type='plane' size='1 1 0.1'/>"
         "<body name='base' pos='0 0 1'><freejoint/><geom type='box' size='0.1 0.1 0.1'/>"
         "<body name='link'><joint name='a' type='hinge' range='-1 1'/>"
         "<geom type='box' size='0.1 0.1 0.1' pos='0 0 -0.3'/></body></body>"
         "<body name='other' pos='1 0 1'><joint name='b' type='hinge'/>"
         "<geom type='sphere' size='0.1'/></body>"
         "</worldbody><actuator>" + actuators + "</actuator></mujoco>";
}

std::string mit_on(const std::string & joint)
{
  return "<general joint='" + joint + "' " + kMitActuator + "/>";
}

TEST(JointDrives, MapsEachJointToItsMitActuator)
{
  const auto model = load_model_xml(model_with(mit_on("b") + mit_on("a")));
  ASSERT_NE(model, nullptr);
  const auto drives = map_joint_drives(*model, joint_manifest({"a"}));
  ASSERT_THAT(drives, HoldsValue());
  const auto & joints = std::get<JointDrives>(drives).joints;
  ASSERT_EQ(joints.size(), 1U);
  const int a = mj_name2id(model.get(), mjtObj::mjOBJ_JOINT, "a");
  EXPECT_EQ(joints[0].qpos_adr, model->jnt_qposadr[a]);
  EXPECT_EQ(joints[0].dof_adr, model->jnt_dofadr[a]);
  EXPECT_EQ(joints[0].actuator, 1);
  EXPECT_EQ(std::get<JointDrives>(drives).root_body,
      mj_name2id(model.get(), mjtObj::mjOBJ_BODY, "base"));
}

TEST(JointDrives, RejectsAJointTheModelCannotDrive)
{
  struct Case
  {
    const char * name;
    std::string actuators;
    std::vector<std::string> joints;
    const char * error;
  };
  const std::vector<Case> cases{
    {"unknown joint", mit_on("a"), {"missing"}, "not found"},
    {"no actuator", mit_on("b"), {"a"}, "has no actuator"},
    {"two actuators", mit_on("a") + mit_on("a"), {"a"}, "more than one actuator"},
    {"not an MIT actuator", "<motor joint='a'/>", {"a"}, "not an MIT actuator"},
    {"two trees", mit_on("a") + mit_on("b"), {"a", "b"}, "same kinematic tree"},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    const auto model = load_model_xml(model_with(c.actuators));
    ASSERT_NE(model, nullptr);
    EXPECT_THAT(map_joint_drives(*model, joint_manifest(c.joints)), HoldsError(HasSubstr(c.error)));
  }
}

TEST(JointDrives, TheActuatorAppliesTheMitLaw)
{
  const auto model = load_model_xml(model_with(mit_on("a")));
  ASSERT_NE(model, nullptr);
  const auto data = make_data(*model);
  const auto drives = map_joint_drives(*model, joint_manifest({"a"}));
  ASSERT_THAT(drives, HoldsValue());
  const auto drive = std::get<JointDrives>(drives).joints[0];
  data->qpos[drive.qpos_adr] = 0.1;
  data->qvel[drive.dof_adr] = -0.2;

  drive.command(
    *model, *data, {.position_rad = 0.3, .velocity_rad_s = 0.5, .effort_nm = 0.25,
      .stiffness_nm_rad = 4.0, .damping_nm_s_rad = 2.0});
  mj_forward(model.get(), data.get());
  // tau = kp (q_d - q) + kd (qdot_d - qdot) + tau_ff
  EXPECT_NEAR(drive.feedback(*data).effort_nm, 4.0 * 0.2 + 2.0 * 0.7 + 0.25, 1e-12);

  drive.release(*model, *data);
  mj_forward(model.get(), data.get());
  EXPECT_EQ(drive.feedback(*data).effort_nm, 0.0);
}

TEST(InitialPose, StartsTheNamedJointsAtTheirAngles)
{
  const auto model = load_model_xml(model_with(mit_on("a")));
  ASSERT_NE(model, nullptr);
  const auto data = make_data(*model);
  const std::vector<std::string> joints{"a"};
  const std::vector<double> positions{0.5};
  const auto pose = resolve_initial_pose(*model, *data, joints, positions);
  ASSERT_THAT(pose, HoldsValue());
  ASSERT_EQ(std::get<InitialPose>(pose).size(), 1U);
  EXPECT_EQ(std::get<InitialPose>(pose)[0].value, 0.5);
}

TEST(InitialPose, RejectsAnInvalidPose)
{
  struct Case
  {
    const char * name;
    std::vector<std::string> joints;
    std::vector<double> positions;
    const char * error;
  };
  const std::vector<Case> cases{
    {"sizes differ", {"a"}, {}, "1 joints but 0 positions"},
    {"unknown joint", {"missing"}, {0.0}, "not found"},
    {"outside the range", {"a"}, {1.5}, "outside its range"},
  };
  const auto model = load_model_xml(model_with(mit_on("a")));
  ASSERT_NE(model, nullptr);
  const auto data = make_data(*model);
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    EXPECT_THAT(
      resolve_initial_pose(*model, *data, c.joints, c.positions), HoldsError(HasSubstr(c.error)));
  }
}

}  // namespace
