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
// Model path is read from the HUMANOID_MJCF_PATH environment variable.
#ifndef HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_
#define HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "humanoid_transport/actuator_transport.hpp"
#include "humanoid_transport_mujoco/ground_truth_publisher.hpp"
#include "humanoid_transport_mujoco/mujoco_disturbance.hpp"

// Forward-declare MuJoCo types to avoid pulling the full header into dependents.
struct mjModel_;
struct mjData_;

namespace rclcpp
{
class Node;
}  // namespace rclcpp

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

  // --- configure() steps, in order ---
  [[nodiscard]] bool load_model();
  [[nodiscard]] bool validate_substeps();
  [[nodiscard]] bool map_joints(const transport::JointManifest & joints);
  [[nodiscard]] bool configure_disturbance();

  // --- MuJoCo state ---
  MjModelPtr model_;
  MjDataPtr data_;
  std::vector<JointMapping> joint_map_;
  std::uint8_t joint_count_{0};
  int n_substeps_{0};
  // qpos offset of the pelvis freejoint, resolved once at load time so
  // exchange() never does a string-based MuJoCo lookup on the real-time path.
  int root_qpos_adr_{-1};

  // --- Health counters ---
  transport::CycleSequence last_sequence_{0};
  std::uint64_t exchanges_attempted_{0};
  std::uint64_t exchanges_failed_{0};
  std::uint64_t deadline_misses_{0};
  std::chrono::nanoseconds worst_round_trip_{0};
  bool active_{false};

  // --- SIL scenario control ---
  MujocoDisturbance disturbance_;

  // --- SIL-only non-real-time side channel ---
  // Hosts simulator diagnostics. Never touched by exchange().
  std::shared_ptr<rclcpp::Node> sim_node_;
  // exchange() hands ground truth over with a lock-free copy; all TF work
  // happens on the publisher's own thread (ADR-001).
  GroundTruthPublisher ground_truth_;
};

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_
