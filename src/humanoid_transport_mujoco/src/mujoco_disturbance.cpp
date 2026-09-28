// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_mujoco/mujoco_disturbance.hpp"

#include <mujoco/mujoco.h>

#include <format>

#include <cmath>
#include <string>

namespace humanoid::transport_mujoco
{

std::optional<std::string> MujocoDisturbance::configure(
  const mjModel_ & model, const PushConfig & config)
{
  body_id_ = -1;
  if (config.force_n <= 0.0) {
    return std::nullopt;  // disabled
  }
  if (!(config.duration_s > 0.0) || !(config.start_time_s >= 0.0)) {
    return std::format(
      "push window invalid: start={} s, duration={} s", config.start_time_s, config.duration_s);
  }
  const auto & d = config.direction;
  const double norm = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
  if (!(norm > 0.0) || !std::isfinite(norm)) {
    return std::string{"push direction must be a finite, non-zero vector"};
  }
  const int body_id = mj_name2id(&model, mjtObj::mjOBJ_BODY, config.body.c_str());
  if (body_id < 0) {
    return std::format("push body '{}' not found in model", config.body);
  }

  for (std::size_t k = 0; k < force_n_.size(); ++k) {
    force_n_[k] = config.force_n * d[k] / norm;
  }
  start_time_s_ = config.start_time_s;
  end_time_s_ = config.start_time_s + config.duration_s;
  body_id_ = body_id;
  return std::nullopt;
}

void MujocoDisturbance::apply(mjData_ & data) const noexcept
{
  if (body_id_ < 0) {
    return;
  }
  // xfrc_applied is [force(3), torque(3)] per body, world frame, and MuJoCo
  // retains it across steps until overwritten.
  const bool in_window = data.time >= start_time_s_ && data.time < end_time_s_;
  double * wrench = data.xfrc_applied + 6 * body_id_;
  for (std::size_t k = 0; k < force_n_.size(); ++k) {
    wrench[k] = in_window ? force_n_[k] : 0.0;
  }
}

}  // namespace humanoid::transport_mujoco
