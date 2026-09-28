// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_mujoco/mujoco_placement.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>

namespace humanoid::transport_mujoco
{

std::optional<int> free_joint_qpos_adr(const mjModel_ & model, int root_body) noexcept
{
  if (root_body <= 0 || root_body >= model.nbody) {
    return std::nullopt;
  }
  const int first = model.body_jntadr[root_body];
  for (int j = first; first >= 0 && j < first + model.body_jntnum[root_body]; ++j) {
    if (model.jnt_type[j] == mjtJoint::mjJNT_FREE) {
      return model.jnt_qposadr[j];
    }
  }
  return std::nullopt;
}

std::optional<double> ground_plane_z(const mjModel_ & model, const mjData_ & data) noexcept
{
  // A plane geom's normal is its local +z axis; horizontal means that axis
  // is world +z (geom_xmat is row-major, element [2][2] at index 8).
  constexpr double kHorizontalTolerance = 1e-9;
  std::optional<double> highest;
  for (int g = 0; g < model.ngeom; ++g) {
    if (model.geom_bodyid[g] != 0 || model.geom_type[g] != mjtGeom::mjGEOM_PLANE) {
      continue;
    }
    if (std::abs(data.geom_xmat[9 * g + 8] - 1.0) > kHorizontalTolerance) {
      continue;
    }
    const double z = data.geom_xpos[3 * g + 2];
    highest = highest ? std::max(*highest, z) : z;
  }
  return highest;
}

std::optional<double> lowest_collision_z(
  const mjModel_ & model, const mjData_ & data, int root_body) noexcept
{
  std::optional<double> lowest;
  for (int g = 0; g < model.ngeom; ++g) {
    const int body = model.geom_bodyid[g];
    const bool collidable = model.geom_contype[g] != 0 || model.geom_conaffinity[g] != 0;
    if (body == 0 || model.body_rootid[body] != root_body || !collidable ||
      model.geom_type[g] == mjtGeom::mjGEOM_PLANE)
    {
      continue;
    }
    // geom_aabb is (center, half-size) in the geom frame. The world-z row of
    // the rotation maps it to a world centre height and a world half-height.
    const mjtNum * aabb = model.geom_aabb + 6 * g;
    const mjtNum * rz = data.geom_xmat + 9 * g + 6;
    const double centre_z =
      data.geom_xpos[3 * g + 2] + rz[0] * aabb[0] + rz[1] * aabb[1] + rz[2] * aabb[2];
    const double half_height =
      std::abs(rz[0]) * aabb[3] + std::abs(rz[1]) * aabb[4] + std::abs(rz[2]) * aabb[5];
    const double bottom = centre_z - half_height;
    lowest = lowest ? std::min(*lowest, bottom) : bottom;
  }
  return lowest;
}

}  // namespace humanoid::transport_mujoco
