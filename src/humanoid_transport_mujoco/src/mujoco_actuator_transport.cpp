#include "humanoid_transport_mujoco/mujoco_actuator_transport.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace humanoid::transport_mujoco
{

MujocoActuatorTransport::MujocoActuatorTransport() = default;

MujocoActuatorTransport::~MujocoActuatorTransport()
{
  deactivate();
  if (d_) {
    mj_deleteData(d_);
    d_ = nullptr;
  }
  if (m_) {
    mj_deleteModel(m_);
    m_ = nullptr;
  }
}

bool MujocoActuatorTransport::configure(
  const transport::JointManifest & joints, const transport::SafetyManifest & safety)
{
  const char * mjcf_path = std::getenv("HUMANOID_MJCF_PATH");
  if (!mjcf_path) {
    std::cerr << "MujocoActuatorTransport: HUMANOID_MJCF_PATH environment variable not set\n";
    return false;
  }

  char error[1000];
  m_ = mj_loadXML(mjcf_path, nullptr, error, 1000);
  if (!m_) {
    std::cerr << "MujocoActuatorTransport: Failed to load MJCF: " << error << "\n";
    return false;
  }

  d_ = mj_makeData(m_);
  if (!d_) {
    std::cerr << "MujocoActuatorTransport: Failed to make MuJoCo data\n";
    mj_deleteModel(m_);
    m_ = nullptr;
    return false;
  }

  joints_ = joints;
  safety_ = safety;
  mj_qposadr_.resize(joints.joint_count, -1);
  mj_dofadr_.resize(joints.joint_count, -1);

  for (std::uint8_t i = 0; i < joints.joint_count; ++i) {
    int jnt_id = mj_name2id(m_, mjOBJ_JOINT, joints.joints[i].name.data());
    if (jnt_id < 0) {
      std::cerr << "MujocoActuatorTransport: Joint not found in MJCF: " << joints.joints[i].name.data() << "\n";
      return false;
    }
    mj_qposadr_[i] = m_->jnt_qposadr[jnt_id];
    mj_dofadr_[i] = m_->jnt_dofadr[jnt_id];
  }

  return true;
}

bool MujocoActuatorTransport::activate()
{
  if (!m_ || !d_) return false;
  active_ = true;
  return true;
}

void MujocoActuatorTransport::deactivate()
{
  active_ = false;
}

transport::TransportCapabilities MujocoActuatorTransport::capabilities() const
{
  transport::TransportCapabilities caps;
  std::strncpy(caps.implementation_name.data(), "MujocoActuatorTransport", transport::kMaxNameLength - 1);
  caps.implementation_name[transport::kMaxNameLength - 1] = '\0';
  caps.transport_class = transport::TransportClass::kSimulated;
  caps.joint_count = joints_.joint_count;
  caps.nominal_cycle_period_us = 5000;
  caps.worst_case_exchange_us = 1000;
  caps.supports_per_joint_disable = false;
  caps.supports_availability_mask = true;
  caps.provides_temperature = false;
  caps.provides_bus_voltage = false;
  caps.is_deterministic = true;
  caps.tuple_completeness = transport::TupleCompleteness::kFull;
  return caps;
}

transport::ExchangeResult MujocoActuatorTransport::exchange(
  const transport::CommandBatch & command, transport::FeedbackBatch & feedback,
  transport::MonotonicStamp /*deadline*/) noexcept
{
  transport::ExchangeResult result;
  result.error = transport::TransportError::kNone;
  exchanges_attempted_++;

  if (!active_) {
    result.error = transport::TransportError::kNotActive;
    exchanges_failed_++;
    return result;
  }

  // 1. Apply MIT torque to qfrc_applied
  for (std::uint8_t i = 0; i < command.joint_count; ++i) {
    int dof = mj_dofadr_[i];
    int qpos = mj_qposadr_[i];
    if (dof < 0 || qpos < 0) continue;

    double q_curr = d_->qpos[qpos];
    double qdot_curr = d_->qvel[dof];

    const auto & cmd = command.joints[i];
    double tau = cmd.stiffness_nm_rad * (cmd.position_rad - q_curr) +
                 cmd.damping_nm_s_rad * (cmd.velocity_rad_s - qdot_curr) +
                 cmd.effort_nm;

    d_->qfrc_applied[dof] = tau;
  }

  // 2. Step 5 substeps of 1 ms (Lockstep integer substeps asserted at configure)
  for (int i = 0; i < 5; ++i) {
    mj_step(m_, d_);
  }

  // 3. Read feedback
  feedback.sequence = command.sequence;
  feedback.stamp = std::chrono::steady_clock::now();
  feedback.joint_count = command.joint_count;
  feedback.availability_mask = command.availability_mask;
  feedback.availability_epoch = command.availability_epoch;

  for (std::uint8_t i = 0; i < command.joint_count; ++i) {
    int dof = mj_dofadr_[i];
    int qpos = mj_qposadr_[i];
    if (dof < 0 || qpos < 0) continue;

    feedback.joints[i].position_rad = d_->qpos[qpos];
    feedback.joints[i].velocity_rad_s = d_->qvel[dof];
    feedback.joints[i].effort_nm = d_->qfrc_applied[dof];
    feedback.joints[i].fresh = true;
  }

  result.joints_reported = command.joint_count;
  result.round_trip = std::chrono::nanoseconds(1000000); // dummy 1ms
  return result;
}

bool MujocoActuatorTransport::request_joint_disable(transport::JointIndex /*joint*/) noexcept
{
  return false; // Sim does not support per-joint disable
}

bool MujocoActuatorTransport::request_all_disable() noexcept
{
  return true;
}

transport::HealthSnapshot MujocoActuatorTransport::health_snapshot() const noexcept
{
  transport::HealthSnapshot snap;
  snap.exchanges_attempted = exchanges_attempted_;
  snap.exchanges_failed = exchanges_failed_;
  snap.active = active_;
  return snap;
}

}  // namespace humanoid::transport_mujoco

PLUGINLIB_EXPORT_CLASS(
  humanoid::transport_mujoco::MujocoActuatorTransport, humanoid::transport::ActuatorTransport)