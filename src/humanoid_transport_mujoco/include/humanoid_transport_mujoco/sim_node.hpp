// Copyright 2026 UTRA-RoboSoccer
// The SIL-only ROS node of MujocoActuatorTransport and the executor that services it: a node that
// is never spun reads its parameters but never answers `ros2 param get`. It has its own executor
// because controller_manager's would put an rclcpp type into the ROS-free ActuatorTransport seam.
#ifndef HUMANOID_TRANSPORT_MUJOCO__SIM_NODE_HPP_
#define HUMANOID_TRANSPORT_MUJOCO__SIM_NODE_HPP_

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

namespace humanoid::transport_mujoco
{

class SimNode
{
public:
  /// Creates the node. Nothing is serviced until spin() is called, so
  /// parameters can be declared first without racing their own services.
  explicit SimNode(const std::string & name);

  /// Cancels the executor and joins its thread.
  ~SimNode();

  SimNode(const SimNode &) = delete;
  SimNode & operator=(const SimNode &) = delete;
  SimNode(SimNode &&) = delete;
  SimNode & operator=(SimNode &&) = delete;

  /// Starts servicing the node on a dedicated non-real-time thread. Idempotent.
  void spin();

  [[nodiscard]] const std::shared_ptr<rclcpp::Node> & node() const noexcept {return node_;}

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;
};

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__SIM_NODE_HPP_
