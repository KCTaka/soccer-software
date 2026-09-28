// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_mujoco/ground_truth_publisher.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace humanoid::transport_mujoco
{

// Defensive: the owner stops us on deactivate, but destroying a joinable
// std::thread calls std::terminate, e.g. on a shutdown that skips lifecycle.
GroundTruthPublisher::~GroundTruthPublisher()
{
  stop();
}

void GroundTruthPublisher::start(std::shared_ptr<rclcpp::Node> node)
{
  stop();
  node_ = std::move(node);
  if (!broadcaster_) {
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*node_);
  }
  // No producer runs while we are stopped (exchange() is inactive), so the
  // consumer-side reset is safe here.
  buffer_.reset();
  running_.store(true, std::memory_order_relaxed);
  thread_ = std::thread(&GroundTruthPublisher::run, this);
}

void GroundTruthPublisher::stop() noexcept
{
  running_.store(false, std::memory_order_relaxed);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void GroundTruthPublisher::run()
{
  using namespace std::chrono_literals;

  transport::CycleSequence last_published = 0;
  bool published_any = false;
  // Logging lives here, not in exchange(): a stream write is a syscall that
  // can block on a slow terminal or pipe, which ADR-001 forbids in-cycle.
  double next_log_time_s = 0.0;
  while (running_.load(std::memory_order_relaxed)) {
    // Publish only a new sample: re-sending the last pose under a fresh
    // stamp would present a stalled simulation as a live one.
    const auto sample = buffer_.read();
    if (sample && (!published_any || sample->sequence != last_published)) {
      const GroundTruthFrame & frame = *sample;
      // Stamp with the sample time, not the publish time. The sample is
      // 0-50 ms old here; convert its monotonic age into the node clock so
      // the pose lines up with the joint_states from the same cycle.
      const auto age = transport::MonotonicStamp::clock::now() - frame.captured;
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = node_->now() - rclcpp::Duration(
        std::chrono::duration_cast<std::chrono::nanoseconds>(age));
      t.header.frame_id = kWorldFrame;
      t.child_frame_id = kGroundTruthRootFrame;
      t.transform.translation.x = frame.position[0];
      t.transform.translation.y = frame.position[1];
      t.transform.translation.z = frame.position[2];
      t.transform.rotation.w = frame.orientation_wxyz[0];
      t.transform.rotation.x = frame.orientation_wxyz[1];
      t.transform.rotation.y = frame.orientation_wxyz[2];
      t.transform.rotation.z = frame.orientation_wxyz[3];
      broadcaster_->sendTransform(t);
      last_published = frame.sequence;
      published_any = true;

      // pelvis_z is the freejoint height, i.e. the pelvis body origin.
      if (frame.sim_time_s >= next_log_time_s) {
        std::cerr << "[MuJoCo] t=" << frame.sim_time_s
                  << " pelvis_z=" << frame.position[2] << "\n";
        next_log_time_s = std::floor(frame.sim_time_s) + 1.0;
      }
    }
    std::this_thread::sleep_for(50ms);
  }
}

}  // namespace humanoid::transport_mujoco
