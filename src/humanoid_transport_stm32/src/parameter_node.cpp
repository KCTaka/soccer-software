// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_stm32/parameter_node.hpp"

#include <rclcpp/rclcpp.hpp>

namespace humanoid::transport_stm32
{

ParameterNode::ParameterNode(const std::string & name)
: node_(rclcpp::Node::make_shared(name)),
  executor_(std::make_unique<rclcpp::executors::SingleThreadedExecutor>())
{
  executor_->add_node(node_);
}

ParameterNode::~ParameterNode()
{
  executor_->cancel();
  if (thread_.joinable()) {
    thread_.join();
  }
  executor_->remove_node(node_);
}

void ParameterNode::spin()
{
  if (!thread_.joinable()) {
    thread_ = std::thread([this] {executor_->spin();});
  }
}

}  // namespace humanoid::transport_stm32
