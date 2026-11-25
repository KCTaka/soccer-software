// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_mujoco/mujoco_placement.hpp"

#include <mujoco/mujoco.h>

#include <format>
#include <span>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <variant>
#include <vector>

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

std::variant<double, std::string> standing_root_z(
  const mjModel_ & model, mjData_ & data, int root_body, int root_qpos_adr)
{
  mj_kinematics(&model, &data);
  const auto ground = ground_plane_z(model, data);
  if (!ground) {
    return std::string{"floating-base robot but no horizontal ground plane in the model"};
  }
  const auto lowest = lowest_collision_z(model, data, root_body);
  if (!lowest) {
    return std::string{"floating-base robot has no collidable geometry to stand on"};
  }
  return data.qpos[root_qpos_adr + 2] + (*ground - *lowest);
}

std::optional<std::string> close_equality_constraints(
  const mjModel_ & model, mjData_ & data, std::span<const int> held_dofs)
{
  // A closure pin's residual is a CAD-exported model's float noise at the zero pose, ~1e-16 m;
  // 1 nm is far below any physical meaning and far above what Gauss-Newton converges to.
  constexpr double kClosedM = 1e-9;
  constexpr int kMaxIterations = 50;
  // Keeps J J^T positive definite when the constraint rows are dependent.
  constexpr double kRegularisation = 1e-12;

  std::vector<int> movable;  // DOF addresses Gauss-Newton may move
  for (int j = 0; j < model.njnt; ++j) {
    const int type = model.jnt_type[j];
    const int dof = model.jnt_dofadr[j];
    if ((type == mjtJoint::mjJNT_HINGE || type == mjtJoint::mjJNT_SLIDE) &&
      std::find(held_dofs.begin(), held_dofs.end(), dof) == held_dofs.end())
    {
      movable.push_back(dof);
    }
  }
  const auto nv = static_cast<int>(model.nv);  // MuJoCo's mju_* functions take int sizes
  const auto cols = static_cast<int>(movable.size());
  std::vector<mjtNum> jac_full;
  std::vector<mjtNum> jac;
  std::vector<mjtNum> residual;
  std::vector<mjtNum> normal;
  std::vector<mjtNum> step(static_cast<std::size_t>(nv));
  double worst = 0.0;
  for (int iteration = 0; iteration <= kMaxIterations; ++iteration) {
    mj_fwdPosition(&model, &data);
    std::vector<int> rows;
    worst = 0.0;
    for (int i = 0; i < data.nefc; ++i) {
      if (data.efc_type[i] == mjtConstraint::mjCNSTR_EQUALITY) {
        rows.push_back(i);
        worst = std::max(worst, std::abs(data.efc_pos[i]));
      }
    }
    if (worst < kClosedM) {
      return std::nullopt;
    }
    if (iteration == kMaxIterations || cols == 0) {
      break;
    }
    // The equality rows of the constraint Jacobian, restricted to the movable DOFs.
    jac_full.assign(static_cast<std::size_t>(data.nefc) * nv, 0.0);
    if (mj_isSparse(&model)) {
      mju_sparse2dense(
        jac_full.data(), data.efc_J, data.nefc, nv, data.efc_J_rownnz, data.efc_J_rowadr,
        data.efc_J_colind);
    } else {
      std::copy(data.efc_J, data.efc_J + jac_full.size(), jac_full.begin());
    }
    const auto n = static_cast<int>(rows.size());
    jac.assign(static_cast<std::size_t>(n) * cols, 0.0);
    residual.assign(static_cast<std::size_t>(n), 0.0);
    for (int r = 0; r < n; ++r) {
      residual[r] = data.efc_pos[rows[r]];
      for (int c = 0; c < cols; ++c) {
        jac[r * cols + c] = jac_full[static_cast<std::size_t>(rows[r]) * nv + movable[c]];
      }
    }
    // Least-norm step: dq = -J^T (J J^T)^-1 r.
    normal.assign(static_cast<std::size_t>(n) * n, 0.0);
    mju_mulMatMatT(normal.data(), jac.data(), jac.data(), n, cols, n);
    for (int r = 0; r < n; ++r) {
      normal[r * n + r] += kRegularisation;
    }
    if (mju_cholFactor(normal.data(), n, 0.0) != n) {
      break;
    }
    std::vector<mjtNum> y(static_cast<std::size_t>(n));
    mju_cholSolve(y.data(), normal.data(), residual.data(), n);
    std::fill(step.begin(), step.end(), 0.0);
    for (int c = 0; c < cols; ++c) {
      for (int r = 0; r < n; ++r) {
        step[movable[c]] -= jac[r * cols + c] * y[r];
      }
    }
    mj_integratePos(&model, data.qpos, step.data(), 1.0);
  }
  return std::format(
    "the equality constraints cannot be closed at the initial pose: a residual of {:.3g} remains",
    worst);
}

}  // namespace humanoid::transport_mujoco
