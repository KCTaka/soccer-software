// Copyright 2026 UTRA-RoboSoccer
// SIL-only ground truth handed from the real-time exchange() thread to a
// non-real-time consumer.
//
// Deliberately free of MuJoCo and ROS types: a frame is a plain snapshot of
// simulator state, so any simulator backend can produce it and any
// diagnostic consumer can read it. Neither side touches the other's state.
#ifndef HUMANOID_TRANSPORT_MUJOCO__GROUND_TRUTH_HPP_
#define HUMANOID_TRANSPORT_MUJOCO__GROUND_TRUTH_HPP_

#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/latest_value_buffer.hpp"

namespace humanoid::transport_mujoco
{

// Extension point: additional SIL-only ground-truth fields (true CoM,
// per-foot contact state, ...) can be added here without touching the
// ActuatorTransport contract or the exchange() signature.
struct GroundTruthFrame
{
  bool valid{false};
  // Control-cycle sequence of the exchange() that produced this sample. The
  // consumer publishes only when it changes, so a stalled or deactivated
  // simulation never republishes a stale pose under a fresh timestamp.
  transport::CycleSequence sequence{0};
  // Monotonic instant the sample was taken (after the physics step). Lets the
  // consumer stamp its output with the sample time, not its own publish time.
  transport::MonotonicStamp captured{};
  // Simulation time after the step, seconds.
  double sim_time_s{0.0};
  double position[3]{0.0, 0.0, 0.0};        // sim_world frame, metres
  double orientation_wxyz[4]{1.0, 0.0, 0.0, 0.0};
};

// Lock-free, allocation-free, fixed three-slot SPSC hand-off. Producer is the
// real-time exchange() call; consumer is the non-real-time publisher thread.
using GroundTruthBuffer = transport::LatestValueBuffer<GroundTruthFrame>;

static_assert(
  transport::LatestValueFrame<GroundTruthFrame>,
  "GroundTruthFrame must stay trivially copyable to be real-time safe");

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__GROUND_TRUTH_HPP_
