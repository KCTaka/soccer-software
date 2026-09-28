// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_mujoco/sim_node.hpp"

#include <rclcpp/rclcpp.hpp>

namespace humanoid::transport_mujoco
{

SimNode::SimNode(const std::string & name)
: node_(rclcpp::Node::make_shared(name)),
  executor_(std::make_unique<rclcpp::executors::SingleThreadedExecutor>())
{
  executor_->add_node(node_);
}

SimNode::~SimNode()
{
  executor_->cancel();
  if (thread_.joinable()) {
    thread_.join();
  }
  executor_->remove_node(node_);
}

void SimNode::spin()
{
  if (!thread_.joinable()) {
    thread_ = std::thread([this] {executor_->spin();});
  }
}

}  // namespace humanoid::transport_mujoco
