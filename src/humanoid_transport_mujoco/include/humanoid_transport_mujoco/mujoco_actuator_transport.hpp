// Copyright 2026 UTRA-RoboSoccer
// MujocoActuatorTransport: SIL transport at the ActuatorTransport seam (ADR-007-01).
//
// The generated MJCF contains no <actuator> section. This transport computes the MIT
// impedance law and applies the resulting torque directly to qfrc_applied, making the
// executed law exactly:
//
//   tau = K_p * (q_d - q) + K_d * (qdot_d - qdot) + tau_ff
//
// Physics is stepped in lockstep inside exchange(): n substeps of dt_physics per
// control period, where n = control_period / dt_physics is asserted integer at
// configure (ADR-007-03). For the unitree_g1 model: dt_physics = 1 ms, n = 5.
//
// Configuration comes from ROS 2 parameters on the SIL-only "mujoco_sim" node,
// read once in configure(): mjcf_path, and disturbance.push.* (see
// MujocoDisturbance). Set them in the ros2_control_node parameter file.
#ifndef HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_
#define HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "humanoid_transport/actuator_transport.hpp"
#include "humanoid_transport_mujoco/ground_truth_publisher.hpp"
#include "humanoid_transport_mujoco/mujoco_disturbance.hpp"
#include "humanoid_transport_mujoco/sim_node.hpp"

// Forward-declare MuJoCo types to avoid pulling the full header into dependents.
struct mjModel_;
struct mjData_;

namespace humanoid::transport_mujoco
{

// Deleter bodies live in the .cpp, where <mujoco/mujoco.h> is visible; the
// header only forward-declares mjModel_/mjData_ so dependents don't pull in
// the full MuJoCo header.
struct MjModelDeleter
{
  void operator()(mjModel_ * m) const noexcept;
};
struct MjDataDeleter
{
  void operator()(mjData_ * d) const noexcept;
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

  // Per-joint mapping from manifest index to MuJoCo addresses.
  struct JointMapping
  {
    int qpos_adr{-1};  // index into d->qpos
    int dof_adr{-1};   // index into d->qvel, d->qfrc_applied
  };

  // Everything configure() reads from ROS parameters, as plain values.
  struct SimParameters
  {
    std::string mjcf_path;
    PushConfig push;
  };

  // --- configure() steps, in order ---
  [[nodiscard]] std::optional<SimParameters> declare_parameters();
  [[nodiscard]] bool load_model(const std::string & mjcf_path);
  [[nodiscard]] bool validate_substeps();
  [[nodiscard]] bool map_joints(const transport::JointManifest & joints);
  [[nodiscard]] bool resolve_initial_placement();
  [[nodiscard]] bool configure_disturbance(const PushConfig & config);

  // --- MuJoCo state ---
  MjModelPtr model_;
  MjDataPtr data_;
  std::vector<JointMapping> joint_map_;
  std::uint8_t joint_count_{0};
  int n_substeps_{0};
  // Root body of the kinematic tree holding the commanded joints.
  int robot_root_body_{-1};
  // qpos offset of that tree's free joint, or -1 for a fixed base. Resolved
  // at configure so exchange() never searches the model in-cycle.
  int root_qpos_adr_{-1};
  // Root height that puts the lowest collision geom on the ground plane.
  double initial_root_z_{0.0};

  // --- Lifecycle ---
  // Per-exchange accounting is the caller's (ExchangeStats); this is all the
  // health the simulator can observe that the caller cannot.
  bool active_{false};

  // --- SIL scenario control ---
  MujocoDisturbance disturbance_;

  // --- SIL-only non-real-time side channel ---
  // Simulator parameters and diagnostics, serviced on its own executor
  // thread. Never touched by exchange().
  std::unique_ptr<SimNode> sim_node_;
  // exchange() hands ground truth over with a lock-free copy; all TF work
  // happens on the publisher's own thread (ADR-001).
  GroundTruthPublisher ground_truth_;
};

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_
