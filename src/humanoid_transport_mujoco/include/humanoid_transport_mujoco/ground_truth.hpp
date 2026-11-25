// Copyright 2026 UTRA-RoboSoccer
// SIL-only ground truth, handed from exchange() to a non-real-time consumer. Free of MuJoCo and
// ROS types: a plain snapshot of simulator state.
#ifndef HUMANOID_TRANSPORT_MUJOCO__GROUND_TRUTH_HPP_
#define HUMANOID_TRANSPORT_MUJOCO__GROUND_TRUTH_HPP_

#include <array>

#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/latest_value_buffer.hpp"

namespace humanoid::transport_mujoco
{

struct GroundTruthFrame
{
  bool valid{false};
  // The exchange() that produced it: the consumer publishes a frame once, so a stalled simulation
  // never republishes a stale pose under a fresh stamp.
  transport::CycleSequence sequence{0};
  transport::MonotonicStamp captured{};  // after the physics step; the published stamp
  double sim_time_s{0.0};                // after the step
  std::array<double, 3> position{0.0, 0.0, 0.0};  // sim_world frame, metres
  std::array<double, 4> orientation_wxyz{1.0, 0.0, 0.0, 0.0};
};

// Lock-free single-producer hand-off: exchange() writes, the publisher thread reads.
using GroundTruthBuffer = transport::LatestValueBuffer<GroundTruthFrame>;

static_assert(
  transport::LatestValueFrame<GroundTruthFrame>,
  "GroundTruthFrame must stay trivially copyable to be real-time safe");

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__GROUND_TRUTH_HPP_
