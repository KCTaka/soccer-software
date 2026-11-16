// Copyright 2026 UTRA-RoboSoccer

#include <mujoco/mujoco.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "humanoid_test_support/generated_robots.hpp"
#include "humanoid_test_support/mujoco_model.hpp"
#include "humanoid_transport_mujoco/mujoco_placement.hpp"

namespace
{

using humanoid::test_support::GeneratedRobot;
using humanoid::test_support::load_model;
using humanoid::test_support::load_model_xml;
using humanoid::test_support::make_data;
using humanoid::test_support::RobotName;
using humanoid::transport_mujoco::close_equality_constraints;
using humanoid::transport_mujoco::free_joint_qpos_adr;
using humanoid::transport_mujoco::ground_plane_z;
using humanoid::transport_mujoco::lowest_collision_z;

// Placement at qpos0: the height lift that puts the tree on the ground.
struct Placement
{
  double ground;
  double lowest;
};

std::optional<Placement> place(const mjModel & model, const char * root_name)
{
  const auto data = make_data(model);
  mj_kinematics(&model, data.get());
  const int root = mj_name2id(&model, mjtObj::mjOBJ_BODY, root_name);
  const auto ground = ground_plane_z(model, *data);
  const auto lowest = lowest_collision_z(model, *data, root);
  if (!ground || !lowest) {
    return std::nullopt;
  }
  return Placement{*ground, *lowest};
}

std::string robot(const std::string & body_contents, const std::string & scene = "")
{
  return "<mujoco><worldbody><geom type='plane' size='1 1 0.1'/>"
         "<body name='base'><freejoint/>" + body_contents + "</body>" + scene +
         "</worldbody></mujoco>";
}

TEST(Placement, BoxRestsOnItsHalfHeight)
{
  const auto model = load_model_xml(robot("<geom type='box' size='0.1 0.2 0.3'/>"));
  const auto p = place(*model, "base");
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->ground - p->lowest, 0.3, 1e-12);
}

TEST(Placement, AccountsForGeomRotation)
{
  // MJCF euler angles are degrees. Rotated 90 degrees about x, the 0.2
  // half-size points down.
  const auto quarter =
    load_model_xml(robot("<geom type='box' size='0.1 0.2 0.3' euler='90 0 0'/>"));
  const auto p = place(*quarter, "base");
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->ground - p->lowest, 0.2, 1e-9);

  // An oblique rotation: the lowest corner is 0.3 cos(a) + 0.2 sin(a) down.
  const auto oblique =
    load_model_xml(robot("<geom type='box' size='0.1 0.2 0.3' euler='30 0 0'/>"));
  const auto q = place(*oblique, "base");
  ASSERT_TRUE(q);
  const double a = 30.0 * mjPI / 180.0;
  EXPECT_NEAR(q->ground - q->lowest, 0.3 * std::cos(a) + 0.2 * std::sin(a), 1e-9);
}

TEST(Placement, IgnoresVisualOnlyGeoms)
{
  const auto model = load_model_xml(robot(
      "<geom type='box' size='0.1 0.1 0.1'/>"
      "<geom type='sphere' size='0.05' pos='0 0 -1' contype='0' conaffinity='0'/>"));
  const auto p = place(*model, "base");
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->ground - p->lowest, 0.1, 1e-12);
}

TEST(Placement, IgnoresOtherTreesSuchAsABall)
{
  const auto model = load_model_xml(robot(
      "<geom type='box' size='0.1 0.1 0.1'/>",
      "<body name='ball' pos='1 0 -0.5'><freejoint/><geom type='sphere' size='0.1'/></body>"));
  const auto p = place(*model, "base");
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->ground - p->lowest, 0.1, 1e-12);
}

TEST(Placement, LongerLegStartsHigherByTheSameAmount)
{
  const auto leg = [](double length) {
      return robot(
        "<geom type='box' size='0.1 0.1 0.1'/>"
        "<body name='shin' pos='0 0 -" + std::to_string(length) + "'>"
        "<joint type='hinge'/><geom type='sphere' size='0.02'/></body>");
    };
  const auto short_leg = place(*load_model_xml(leg(0.4)), "base");
  const auto long_leg = place(*load_model_xml(leg(0.5)), "base");
  ASSERT_TRUE(short_leg && long_leg);
  EXPECT_NEAR(
    (long_leg->ground - long_leg->lowest) - (short_leg->ground - short_leg->lowest), 0.1, 1e-9);
}

TEST(Placement, NoGroundPlaneIsReported)
{
  const auto model = load_model_xml(
    "<mujoco><worldbody><body name='base'><freejoint/>"
    "<geom type='box' size='0.1 0.1 0.1'/></body></worldbody></mujoco>");
  const auto data = make_data(*model);
  mj_kinematics(model.get(), data.get());
  EXPECT_FALSE(ground_plane_z(*model, *data));
}

TEST(Placement, FixedBaseHasNoFreeJoint)
{
  const auto model = load_model_xml(
    "<mujoco><worldbody><geom type='plane' size='1 1 0.1'/>"
    "<body name='arm'><joint type='hinge'/><geom type='box' size='0.1 0.1 0.1'/></body>"
    "</worldbody></mujoco>");
  EXPECT_FALSE(free_joint_qpos_adr(*model, mj_name2id(model.get(), mjtObj::mjOBJ_BODY, "arm")));
}

// --- The generated MJCFs -----------------------------------------------------------------------

/// The body a model's free joint carries: the root a floating robot is placed by.
std::optional<std::string> floating_root(const mjModel & model)
{
  for (int j = 0; j < model.njnt; ++j) {
    if (model.jnt_type[j] == mjJNT_FREE) {
      return mj_id2name(&model, mjOBJ_BODY, model.jnt_bodyid[j]);
    }
  }
  return std::nullopt;
}

/// The generated robots whose MJCF has a floating root: the ones placed on the ground.
std::vector<GeneratedRobot> floating_robots()
{
  return humanoid::test_support::generated_robots_where(
    [](const GeneratedRobot & robot) {
      const auto mjcf = robot.mjcf_scene();
      const auto model = mjcf ? load_model(*mjcf) : nullptr;
      return model && floating_root(*model);
    });
}

class GeneratedPlacement : public ::testing::TestWithParam<GeneratedRobot> {};

TEST_P(GeneratedPlacement, StandsOnTheGroundAtQpos0)
{
  const auto model = load_model(*GetParam().mjcf_scene());
  ASSERT_NE(model, nullptr);
  const auto root = floating_root(*model);
  ASSERT_TRUE(root);
  const auto p = place(*model, root->c_str());
  ASSERT_TRUE(p);
  // The model generator stands robot.xml by this same rule, rounded up to the micrometre: qpos0 is
  // on the ground, never in it.
  EXPECT_GE(p->lowest - p->ground, -1e-9);
  EXPECT_LE(p->lowest - p->ground, 1e-6);
}

INSTANTIATE_TEST_SUITE_P(
  EveryFloatingRobot, GeneratedPlacement, ::testing::ValuesIn(floating_robots()), RobotName{});

TEST(Placement, PlacesTheG1WhereMenagerieDoesByHand)
{
  // An independent reference, so it names its robot: Menagerie's g1.xml places the pelvis at
  // z = 0.793 by hand. The height derived from geometry alone must agree to within a centimetre.
  const auto g1 = humanoid::test_support::generated_robot("unitree_g1");
  ASSERT_TRUE(g1 && g1->mjcf_scene());
  const auto model = load_model(*g1->mjcf_scene());
  ASSERT_NE(model, nullptr);
  const int pelvis = mj_name2id(model.get(), mjtObj::mjOBJ_BODY, "pelvis");
  const auto root_adr = free_joint_qpos_adr(*model, pelvis);
  ASSERT_TRUE(root_adr);
  const auto p = place(*model, "pelvis");
  ASSERT_TRUE(p);
  EXPECT_NEAR(model->qpos0[*root_adr + 2] + (p->ground - p->lowest), 0.793, 0.01);
}

// A parallelogram four-bar: cranks a and b, 0.5 long, pivot 1 apart on the ground; the coupler
// hangs on a's tip and is pinned to b's by a connect equality. With a at q, the loop closes with b
// at q and the coupler at -q. A tree cut open the same way is what the generator emits for a
// linkage.
constexpr const char * kParallelogram =
  "<mujoco><worldbody>"
  "<body name='a'><joint name='a' axis='0 1 0'/><geom type='capsule' fromto='0 0 0 0 0 0.5'"
  " size='0.01'/>"
  "<body name='coupler' pos='0 0 0.5'><joint name='coupler' axis='0 1 0'/>"
  "<geom type='capsule' fromto='0 0 0 1 0 0' size='0.01'/><site name='tip' pos='1 0 0'/></body>"
  "</body>"
  "<body name='b' pos='1 0 0'><joint name='b' axis='0 1 0'/><geom type='capsule'"
  " fromto='0 0 0 0 0 0.5' size='0.01'/><site name='pin' pos='0 0 0.5'/></body>"
  "</worldbody><equality><connect site1='tip' site2='pin'/></equality></mujoco>";

TEST(CloseEqualityConstraints, MovesTheLinkageAroundAHeldJoint)
{
  const auto model = load_model_xml(kParallelogram);
  ASSERT_NE(model, nullptr);
  const auto data = make_data(*model);
  const auto qpos = [&](const char * joint) {
      return data->qpos[model->jnt_qposadr[mj_name2id(model.get(), mjtObj::mjOBJ_JOINT, joint)]];
    };
  const int a = mj_name2id(model.get(), mjtObj::mjOBJ_JOINT, "a");
  data->qpos[model->jnt_qposadr[a]] = 0.3;
  const int held[] = {model->jnt_dofadr[a]};
  EXPECT_EQ(close_equality_constraints(*model, *data, held), std::nullopt);
  EXPECT_DOUBLE_EQ(qpos("a"), 0.3);
  EXPECT_NEAR(qpos("b"), 0.3, 1e-9);
  EXPECT_NEAR(qpos("coupler"), -0.3, 1e-9);
}

TEST(CloseEqualityConstraints, ReportsALoopThatCannotClose)
{
  // Holding both cranks at different angles leaves the coupler alone to close a gap it cannot.
  const auto model = load_model_xml(kParallelogram);
  ASSERT_NE(model, nullptr);
  const auto data = make_data(*model);
  const int a = mj_name2id(model.get(), mjtObj::mjOBJ_JOINT, "a");
  const int b = mj_name2id(model.get(), mjtObj::mjOBJ_JOINT, "b");
  data->qpos[model->jnt_qposadr[a]] = 0.3;
  data->qpos[model->jnt_qposadr[b]] = -0.3;
  const int held[] = {model->jnt_dofadr[a], model->jnt_dofadr[b]};
  const auto error = close_equality_constraints(*model, *data, held);
  ASSERT_TRUE(error);
  EXPECT_THAT(*error, ::testing::HasSubstr("cannot be closed"));
}

}  // namespace
