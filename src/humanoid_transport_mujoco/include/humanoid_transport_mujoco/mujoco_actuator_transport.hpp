// Copyright 2026 UTRA-RoboSoccer
// MujocoActuatorTransport: SIL transport at the ActuatorTransport seam (ADR-007-01).
//
// exchange() loads each joint's MIT tuple into its actuator (mujoco_joints.hpp) and steps physics
// in lockstep: control_period / timestep substeps, an integer checked at configure (ADR-007-03).
// Its parameters are read once at configure from the SIL-only "mujoco_sim" node.
#ifndef HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_
#define HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_

#include <span>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "humanoid_transport/actuator_transport.hpp"
#include "humanoid_transport_mujoco/ground_truth_publisher.hpp"
#include "humanoid_transport_mujoco/mujoco_disturbance.hpp"
#include "humanoid_transport_mujoco/mujoco_imu.hpp"
#include "humanoid_transport_mujoco/mujoco_joints.hpp"
#include "humanoid_transport_mujoco/sim_node.hpp"

struct mjModel_;
struct mjData_;

namespace humanoid::transport_mujoco
{

struct MjModelDeleter
{
  void operator()(mjModel_ * model) const noexcept;
};
struct MjDataDeleter
{
  void operator()(mjData_ * data) const noexcept;
};
using MjModelPtr = std::unique_ptr<mjModel_, MjModelDeleter>;
using MjDataPtr = std::unique_ptr<mjData_, MjDataDeleter>;

class MujocoActuatorTransport : public transport::ActuatorTransport
{
public:
  MujocoActuatorTransport() = default;
  ~MujocoActuatorTransport() override;

  MujocoActuatorTransport(const MujocoActuatorTransport &) = delete;
  MujocoActuatorTransport & operator=(const MujocoActuatorTransport &) = delete;
  MujocoActuatorTransport(MujocoActuatorTransport &&) = delete;
  MujocoActuatorTransport & operator=(MujocoActuatorTransport &&) = delete;

  // --- Non-real-time ---
  [[nodiscard]] bool configure(
    const transport::JointManifest & joints,
    const transport::SafetyManifest & safety) override;

  [[nodiscard]] bool activate() override;
  void deactivate() override;
  [[nodiscard]] transport::TransportCapabilities capabilities() const override;

  // --- Real-time ---
  [[nodiscard]] transport::ExchangeResult exchange(
    const transport::CommandBatch & command,
    transport::FeedbackBatch & feedback,
    transport::MonotonicStamp deadline) noexcept override;

  [[nodiscard]] bool request_joint_disable(transport::JointIndex joint) noexcept override;
  [[nodiscard]] bool request_all_disable() noexcept override;
  [[nodiscard]] transport::HealthSnapshot health_snapshot() const noexcept override;

private:
  // --- Types ---

  // Everything configure() reads from ROS parameters, as plain values.
  struct SimParameters
  {
    std::string mjcf_path;
    bool hold_until_commanded{false};
    // Joints the robot starts at a given angle, by name; empty means the model's zero pose.
    std::vector<std::string> initial_pose_joints;
    std::vector<double> initial_pose_positions;
    PushConfig push;
  };

  // --- configure() steps ---
  [[nodiscard]] std::optional<SimParameters> declare_parameters();
  [[nodiscard]] bool load_model(const std::string & mjcf_path);

  // --- Shared by the lifecycle and exchange() ---
  /// The initial pose, every actuator released (the gains live in the model, so they outlive a
  /// reset), and a floating root standing on the ground.
  void reset() noexcept;
  void release_all() noexcept;
  void advance_physics(std::span<const transport::JointCommand> commands) noexcept;
  void publish_ground_truth(
    transport::CycleSequence sequence, transport::MonotonicStamp stamp) noexcept;
  [[nodiscard]] std::uint8_t joint_count() const noexcept
  {
    return static_cast<std::uint8_t>(drives_.size());
  }

  // --- MuJoCo state, set by configure() ---
  MjModelPtr model_;
  MjDataPtr data_;
  std::vector<JointDrive> drives_;  // in manifest order
  int substeps_{0};
  int root_qpos_adr_{-1};  // the robot's free joint, or -1 for a fixed base
  double root_z_{0.0};     // the free root's height standing on the ground at the initial pose
  InitialPose initial_pose_;
  ImuSensors imu_;
  MujocoDisturbance disturbance_;

  // --- Lifecycle ---
  // Until the first command with a nonzero stiffness, a held robot stands still and simulated time
  // does not advance, so the controllers' start-up does not cost it a fall.
  bool hold_until_commanded_{false};
  bool held_{false};
  bool active_{false};

  // --- SIL-only side channel, never touched by exchange() ---
  // ground_truth_ publishes on sim_node_'s node, so it is declared after it and torn down first.
  std::unique_ptr<SimNode> sim_node_;
  GroundTruthPublisher ground_truth_;
};

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_
