// Copyright 2026 UTRA-RoboSoccer
// The ROS node that carries Stm32SerialTransport's parameters, together with the executor that
// services it.
//
// A node that is never spun still reads parameter overrides at declare time, but its parameter
// services never answer, so `ros2 param get /stm32_serial ...` hangs. This class owns a
// single-threaded executor and its thread, so the node is serviced for its whole lifetime.
//
// The node belongs to the transport, not to ros2_control's executor: reaching that executor from
// here would put an rclcpp type into ActuatorTransport, a seam that must not depend on ROS.
// MujocoActuatorTransport solves the same problem with SimNode; this is the same class, kept here
// because humanoid_transport must stay ROS-free and a shared helper package for one 40-line class
// is not yet worth its cost. A third transport should extract it.
#ifndef HUMANOID_TRANSPORT_STM32__PARAMETER_NODE_HPP_
#define HUMANOID_TRANSPORT_STM32__PARAMETER_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

namespace rclcpp
{
class Node;
namespace executors
{
class SingleThreadedExecutor;
}  // namespace executors
}  // namespace rclcpp

namespace humanoid::transport_stm32
{

class ParameterNode
{
public:
  /// Creates the node. Nothing is serviced until spin() is called, so parameters can be declared
  /// first without racing their own services.
  explicit ParameterNode(const std::string & name);

  /// Cancels the executor and joins its thread.
  ~ParameterNode();

  ParameterNode(const ParameterNode &) = delete;
  ParameterNode & operator=(const ParameterNode &) = delete;
  ParameterNode(ParameterNode &&) = delete;
  ParameterNode & operator=(ParameterNode &&) = delete;

  /// Starts servicing the node on a dedicated non-real-time thread. Idempotent.
  void spin();

  [[nodiscard]] const std::shared_ptr<rclcpp::Node> & node() const noexcept {return node_;}

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;
};

}  // namespace humanoid::transport_stm32

#endif  // HUMANOID_TRANSPORT_STM32__PARAMETER_NODE_HPP_
