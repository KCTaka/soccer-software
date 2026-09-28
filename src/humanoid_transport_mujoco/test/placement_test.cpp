// Copyright 2026 UTRA-RoboSoccer

#include <mujoco/mujoco.h>
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "humanoid_transport_mujoco/mujoco_placement.hpp"

namespace
{

using humanoid::transport_mujoco::free_joint_qpos_adr;
using humanoid::transport_mujoco::ground_plane_z;
using humanoid::transport_mujoco::lowest_collision_z;

struct ModelDeleter {void operator()(mjModel * m) const {mj_deleteModel(m);}};
struct DataDeleter {void operator()(mjData * d) const {mj_deleteData(d);}};
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

ModelPtr load_string(const std::string & xml)
{
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "model.xml", xml.data(), static_cast<int>(xml.size()));
  char error[1024] = {};
  ModelPtr model(mj_loadXML("model.xml", &vfs, error, sizeof(error)));
  mj_deleteVFS(&vfs);
  EXPECT_NE(model, nullptr) << error;
  return model;
}

// Placement at qpos0: the height lift that puts the tree on the ground.
struct Placement
{
  double ground;
  double lowest;
};

std::optional<Placement> place(const mjModel & model, const char * root_name)
{
  DataPtr data(mj_makeData(&model));
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
  const auto model = load_string(robot("<geom type='box' size='0.1 0.2 0.3'/>"));
  const auto p = place(*model, "base");
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->ground - p->lowest, 0.3, 1e-12);
}

TEST(Placement, AccountsForGeomRotation)
{
  // MJCF euler angles are degrees. Rotated 90 degrees about x, the 0.2
  // half-size points down.
  const auto quarter = load_string(robot("<geom type='box' size='0.1 0.2 0.3' euler='90 0 0'/>"));
  const auto p = place(*quarter, "base");
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->ground - p->lowest, 0.2, 1e-9);

  // An oblique rotation: the lowest corner is 0.3 cos(a) + 0.2 sin(a) down.
  const auto oblique = load_string(robot("<geom type='box' size='0.1 0.2 0.3' euler='30 0 0'/>"));
  const auto q = place(*oblique, "base");
  ASSERT_TRUE(q);
  const double a = 30.0 * mjPI / 180.0;
  EXPECT_NEAR(q->ground - q->lowest, 0.3 * std::cos(a) + 0.2 * std::sin(a), 1e-9);
}

TEST(Placement, IgnoresVisualOnlyGeoms)
{
  const auto model = load_string(robot(
      "<geom type='box' size='0.1 0.1 0.1'/>"
      "<geom type='sphere' size='0.05' pos='0 0 -1' contype='0' conaffinity='0'/>"));
  const auto p = place(*model, "base");
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->ground - p->lowest, 0.1, 1e-12);
}

TEST(Placement, IgnoresOtherTreesSuchAsABall)
{
  const auto model = load_string(robot(
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
  const auto short_leg = place(*load_string(leg(0.4)), "base");
  const auto long_leg = place(*load_string(leg(0.5)), "base");
  ASSERT_TRUE(short_leg && long_leg);
  EXPECT_NEAR(
    (long_leg->ground - long_leg->lowest) - (short_leg->ground - short_leg->lowest), 0.1, 1e-9);
}

TEST(Placement, NoGroundPlaneIsReported)
{
  const auto model = load_string(
    "<mujoco><worldbody><body name='base'><freejoint/>"
    "<geom type='box' size='0.1 0.1 0.1'/></body></worldbody></mujoco>");
  DataPtr data(mj_makeData(model.get()));
  mj_kinematics(model.get(), data.get());
  EXPECT_FALSE(ground_plane_z(*model, *data));
}

TEST(Placement, FixedBaseHasNoFreeJoint)
{
  const auto model = load_string(
    "<mujoco><worldbody><geom type='plane' size='1 1 0.1'/>"
    "<body name='arm'><joint type='hinge'/><geom type='box' size='0.1 0.1 0.1'/></body>"
    "</worldbody></mujoco>");
  EXPECT_FALSE(free_joint_qpos_adr(*model, mj_name2id(model.get(), mjtObj::mjOBJ_BODY, "arm")));
}

TEST(Placement, G1HeightMatchesUpstreamMenagerieValue)
{
  char error[1024] = {};
  ModelPtr model(mj_loadXML(HUMANOID_TEST_MJCF_PATH, nullptr, error, sizeof(error)));
  ASSERT_NE(model, nullptr) << error;
  const int pelvis = mj_name2id(model.get(), mjtObj::mjOBJ_BODY, "pelvis");
  ASSERT_TRUE(free_joint_qpos_adr(*model, pelvis));
  const auto p = place(*model, "pelvis");
  ASSERT_TRUE(p);
  // Menagerie's g1.xml places the pelvis at z = 0.793 by hand. The derived
  // value must agree to within a centimetre, from geometry alone.
  EXPECT_NEAR(p->ground - p->lowest, 0.793, 0.01);
}

}  // namespace
