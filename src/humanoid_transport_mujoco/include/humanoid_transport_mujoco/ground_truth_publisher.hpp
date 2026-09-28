// Copyright 2026 UTRA-RoboSoccer
// Diagnostic-only RViz2 view of SIL ground truth (ADR-001).
//
// The real-time side only ever calls publish(), a plain copy into a lock-free
// buffer. All ROS/TF work runs on a private non-real-time thread that is not
// part of the controller_manager read()/update()/write() path.
//
// Frames: sim_world -> ground_truth/pelvis. Both ends are deliberately
// outside the production TF tree:
//   - sim_world (the simulator floor) is not odom or map: this is exact,
//     noise-free ground truth, not an estimator output (Topic 6 D1).
//   - The child is NOT the URDF root "pelvis". tf2 permits one parent per
//     frame, and the Tier 0 estimator owns odom -> pelvis (REP-105). Were
//     the simulator to also parent "pelvis", the two would fight in the
//     tree, and every production consumer resolving odom -> pelvis in SIL
//     would see the simulator's truth or a flip-flop between the two.
//     That breaks ADR-007-01: SIL would no longer exercise the production
//     data path it claims to test.
// A second robot_state_publisher with frame_prefix "ground_truth/" hangs
// the link tree off ground_truth/pelvis for RViz (see sil_stand.launch.py).
#ifndef HUMANOID_TRANSPORT_MUJOCO__GROUND_TRUTH_PUBLISHER_HPP_
#define HUMANOID_TRANSPORT_MUJOCO__GROUND_TRUTH_PUBLISHER_HPP_

#include <atomic>
#include <memory>
#include <thread>

#include "humanoid_transport_mujoco/ground_truth.hpp"

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace tf2_ros
{
class TransformBroadcaster;
}  // namespace tf2_ros

namespace humanoid::transport_mujoco
{

class GroundTruthPublisher
{
public:
  GroundTruthPublisher() = default;
  ~GroundTruthPublisher();

  GroundTruthPublisher(const GroundTruthPublisher &) = delete;
  GroundTruthPublisher & operator=(const GroundTruthPublisher &) = delete;
  GroundTruthPublisher(GroundTruthPublisher &&) = delete;
  GroundTruthPublisher & operator=(GroundTruthPublisher &&) = delete;

  // --- Non-real-time ---

  /// Starts the publishing thread on `node`. Discards any frame left over
  /// from a previous run, and restarts cleanly if already running.
  void start(std::shared_ptr<rclcpp::Node> node);

  /// Stops and joins the publishing thread. Idempotent.
  void stop() noexcept;

  // --- Real-time ---

  /// Producer side. No allocation, no ROS call, no unbounded wait.
  void publish(const GroundTruthFrame & frame) noexcept {buffer_.publish(frame);}

private:
  void run();

  static constexpr const char * kWorldFrame = "sim_world";
  static constexpr const char * kGroundTruthRootFrame = "ground_truth/pelvis";

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  GroundTruthBuffer buffer_;
  std::thread thread_;
  std::atomic<bool> running_{false};
};

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__GROUND_TRUTH_PUBLISHER_HPP_
