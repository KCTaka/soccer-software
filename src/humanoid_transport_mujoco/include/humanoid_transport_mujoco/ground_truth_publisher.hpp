// Copyright 2026 UTRA-RoboSoccer
// Diagnostic-only TF view of SIL ground truth, sim_world -> ground_truth/pelvis (ADR-001).
//
// exchange() only copies a frame into a lock-free buffer; all ROS work runs on this class's thread.
// Both frames stay outside the production TF tree (ADR-007-01): sim_world is exact truth, not odom
// or map, and the child is not "pelvis", which the estimator parents (odom -> pelvis, REP-105).
// sil_stand.launch.py hangs the link tree off ground_truth/pelvis with a second
// robot_state_publisher.
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
