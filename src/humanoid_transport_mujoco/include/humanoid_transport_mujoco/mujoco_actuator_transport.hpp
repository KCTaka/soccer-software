// Copyright 2026 Your Organization Name
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

#include <cstdint>
#include <vector>

#include "humanoid_transport/actuator_transport.hpp"

// Forward-declare MuJoCo types to avoid pulling the full header into dependents.
struct mjModel_;
struct mjData_;

namespace humanoid::transport_mujoco
{

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
  mjModel_ * model_{nullptr};
  mjData_ * data_{nullptr};

  // Per-joint mapping from manifest index to MuJoCo addresses.
  struct JointMapping
  {
    int qpos_adr{-1};  // index into d->qpos
    int dof_adr{-1};   // index into d->qvel, d->qfrc_applied
  };
  std::vector<JointMapping> joint_map_;

  std::uint8_t joint_count_{0};
  int n_substeps_{0};

  // Health counters.
  transport::CycleSequence last_sequence_{0};
  std::uint64_t exchanges_attempted_{0};
  std::uint64_t exchanges_failed_{0};
  std::uint64_t deadline_misses_{0};
  std::chrono::nanoseconds worst_round_trip_{0};
  bool active_{false};

  // Perturbation parameters (set via environment or parameters)
  double push_force_n_{0.0};
  double push_time_s_{0.0};
  int push_body_id_{-1};
  int push_axis_{1};  // 1 = lateral (y)
  bool push_started_{false};
  bool push_active_{false};
};

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_
