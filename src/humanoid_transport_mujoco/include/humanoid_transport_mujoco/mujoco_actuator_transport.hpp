#ifndef HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_
#define HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_

#include "humanoid_transport/actuator_transport.hpp"
#include <mujoco/mujoco.h>
#include <vector>
#include <string>

namespace humanoid::transport_mujoco
{

class MujocoActuatorTransport : public transport::ActuatorTransport
{
public:
  MujocoActuatorTransport();
  ~MujocoActuatorTransport() override;

  [[nodiscard]] bool configure(
    const transport::JointManifest & joints, const transport::SafetyManifest & safety) override;
  [[nodiscard]] bool activate() override;
  void deactivate() override;
  [[nodiscard]] transport::TransportCapabilities capabilities() const override;

  [[nodiscard]] transport::ExchangeResult exchange(
    const transport::CommandBatch & command, transport::FeedbackBatch & feedback,
    transport::MonotonicStamp deadline) noexcept override;

  [[nodiscard]] bool request_joint_disable(transport::JointIndex joint) noexcept override;
  [[nodiscard]] bool request_all_disable() noexcept override;
  [[nodiscard]] transport::HealthSnapshot health_snapshot() const noexcept override;

private:
  mjModel * m_{nullptr};
  mjData * d_{nullptr};
  transport::JointManifest joints_{};
  transport::SafetyManifest safety_{};
  std::vector<int> mj_qposadr_;
  std::vector<int> mj_dofadr_;
  bool active_{false};
  std::uint64_t exchanges_attempted_{0U};
  std::uint64_t exchanges_failed_{0U};
};

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_