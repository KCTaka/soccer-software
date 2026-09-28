// Copyright 2026 UTRA-RoboSoccer
// The SIL-only ROS node of MujocoActuatorTransport, together with the
// executor that services it.
//
// A node that is never spun still publishes (TF) and still reads parameter
// overrides at declare time, but its parameter services never answer, so
// `ros2 param get /mujoco_sim ...` hangs. This class owns a single-threaded
// executor and its thread, so the node is serviced for its whole lifetime.
//
// Why not add the node to controller_manager's executor, which ros2_control
// hands to hardware components in HardwareComponentInterfaceParams: the node
// belongs to the transport, and reaching it from there would put an rclcpp
// type into ActuatorTransport, a seam that must not depend on ROS.
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
