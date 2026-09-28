// Copyright 2026 UTRA-RoboSoccer
// Initial placement of a floating-base robot, derived from the loaded model.
//
// The generated MJCF deliberately carries no initial height: the height at
// which the feet touch the ground is a consequence of the kinematic tree and
// collision geometry, so it is computed here from the model that is actually
// loaded, never stored as a number that goes stale when the legs change.
//
// "The robot" is the kinematic tree rooted at one direct child of the world
// body. Scene objects with their own free joints (e.g. a ball) are separate
// trees and are ignored.
#ifndef HUMANOID_TRANSPORT_MUJOCO__MUJOCO_PLACEMENT_HPP_
#define HUMANOID_TRANSPORT_MUJOCO__MUJOCO_PLACEMENT_HPP_

#include <optional>

struct mjModel_;
struct mjData_;

namespace humanoid::transport_mujoco
{

/// qpos address of the free joint on `root_body`, or nullopt if that body is
/// not a floating base (fixed-base robot).
[[nodiscard]] std::optional<int> free_joint_qpos_adr(
  const mjModel_ & model, int root_body) noexcept;

/// World-frame z of the highest horizontal ground plane on the world body,
/// or nullopt if the model has none. Uses the kinematics in `data`.
[[nodiscard]] std::optional<double> ground_plane_z(
  const mjModel_ & model, const mjData_ & data) noexcept;

/// Lowest world-frame z over every collidable geom in the tree rooted at
/// `root_body`, for the kinematics in `data`. Uses each geom's bounding box,
/// so it never underestimates the lowest point: a robot placed with it may
/// start marginally above the ground, but never inside it. nullopt if the
/// tree has no collidable geom.
[[nodiscard]] std::optional<double> lowest_collision_z(
  const mjModel_ & model, const mjData_ & data, int root_body) noexcept;

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__MUJOCO_PLACEMENT_HPP_
