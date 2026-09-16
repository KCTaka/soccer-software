#include "humanoid_transport_mujoco/mujoco_actuator_transport.hpp"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include <mujoco/mujoco.h>

#include "pluginlib/class_list_macros.hpp"

namespace humanoid::transport_mujoco
{

// ---------------------------------------------------------------------------
// Destructor
// ---------------------------------------------------------------------------

MujocoActuatorTransport::~MujocoActuatorTransport()
{
  if (data_) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }
}

// ---------------------------------------------------------------------------
// configure: load model, map joints, assert integer substep ratio
// ---------------------------------------------------------------------------

bool MujocoActuatorTransport::configure(
  const transport::JointManifest & joints,
  const transport::SafetyManifest & /*safety*/)
{
  // 1. Resolve model path.
  const char * mjcf_path = std::getenv("HUMANOID_MJCF_PATH");
  if (!mjcf_path || mjcf_path[0] == '\0') {
    std::cerr << "MujocoActuatorTransport: HUMANOID_MJCF_PATH not set\n";
    return false;
  }

  // 2. Load model.
  char error_buf[1024] = {};
  model_ = mj_loadXML(mjcf_path, nullptr, error_buf, sizeof(error_buf));
  if (!model_) {
    std::cerr << "MujocoActuatorTransport: mj_loadXML failed: " << error_buf << "\n";
    return false;
  }

  // 3. Create data.
  data_ = mj_makeData(model_);
  if (!data_) {
    std::cerr << "MujocoActuatorTransport: mj_makeData failed\n";
    mj_deleteModel(model_);
    model_ = nullptr;
    return false;
  }

  // 4. Assert integer substep ratio (ADR-007-03).
  //    control_period = 5 ms = 5000 us.
  //    dt_physics = model_->opt.timestep (expected 0.001 s = 1 ms).
  //    n_substeps = control_period / dt_physics must be a positive integer.
  constexpr double kControlPeriodS = 0.005;
  const double dt = model_->opt.timestep;
  if (dt <= 0.0) {
    std::cerr << "MujocoActuatorTransport: invalid timestep " << dt << "\n";
    return false;
  }
  const double ratio = kControlPeriodS / dt;
  n_substeps_ = static_cast<int>(ratio + 0.5);  // round to nearest
  if (n_substeps_ < 1 ||
      static_cast<double>(n_substeps_) * dt > kControlPeriodS + 1e-9 ||
      static_cast<double>(n_substeps_) * dt < kControlPeriodS - 1e-9)
  {
    std::cerr << "MujocoActuatorTransport: timestep " << dt
              << " does not divide 5 ms control period by an integer. "
              << "ratio=" << ratio << "\n";
    return false;
  }

  // 5. Map joint names to MuJoCo DOF addresses.
  joint_count_ = joints.joint_count;
  joint_map_.resize(joint_count_);

  for (std::uint8_t i = 0; i < joint_count_; ++i) {
    const char * name = joints.joints[i].name.data();
    const int jnt_id = mj_name2id(model_, mjOBJ_JOINT, name);
    if (jnt_id < 0) {
      std::cerr << "MujocoActuatorTransport: joint '" << name
                << "' not found in model\n";
      return false;
    }
    // Verify it is a hinge (revolute) or slide (prismatic) — 1-DOF joint.
    if (model_->jnt_type[jnt_id] != mjJNT_HINGE &&
        model_->jnt_type[jnt_id] != mjJNT_SLIDE)
    {
      std::cerr << "MujocoActuatorTransport: joint '" << name
                << "' is not hinge/slide (type=" << model_->jnt_type[jnt_id] << ")\n";
      return false;
    }
    joint_map_[i].qpos_adr = model_->jnt_qposadr[jnt_id];
    joint_map_[i].dof_adr = model_->jnt_dofadr[jnt_id];
  }

  active_ = false;
  return true;
}

// ---------------------------------------------------------------------------
// activate / deactivate
// ---------------------------------------------------------------------------

bool MujocoActuatorTransport::activate()
{
  if (!model_ || !data_) {
    return false;
  }
  // Reset simulation state to initial configuration.
  mj_resetData(model_, data_);
  active_ = true;
  return true;
}

void MujocoActuatorTransport::deactivate()
{
  active_ = false;
}

// ---------------------------------------------------------------------------
// capabilities
// ---------------------------------------------------------------------------

transport::TransportCapabilities MujocoActuatorTransport::capabilities() const
{
  transport::TransportCapabilities caps{};
  std::strncpy(
    caps.implementation_name.data(),
    "MujocoActuatorTransport",
    caps.implementation_name.size() - 1);
  caps.transport_class = transport::TransportClass::kSimulated;
  caps.joint_count = joint_count_;
  caps.nominal_cycle_period_us = 5000;
  caps.worst_case_exchange_us = 5000;  // SIL: bounded by lockstep stepping
  caps.supports_per_joint_disable = false;
  caps.supports_availability_mask = true;
  caps.provides_temperature = false;
  caps.provides_bus_voltage = false;
  caps.is_deterministic = false;  // kSimulated, not kReplay
  caps.tuple_completeness = transport::TupleCompleteness::kFull;
  return caps;
}

// ---------------------------------------------------------------------------
// exchange: the real-time lockstep step
// ---------------------------------------------------------------------------

transport::ExchangeResult MujocoActuatorTransport::exchange(
  const transport::CommandBatch & command,
  transport::FeedbackBatch & feedback,
  transport::MonotonicStamp deadline) noexcept
{
  transport::ExchangeResult result{};
  const auto t_start = std::chrono::steady_clock::now();

  ++exchanges_attempted_;

  if (!active_ || !model_ || !data_) {
    result.error = transport::TransportError::kNotActive;
    ++exchanges_failed_;
    return result;
  }

  // --- Apply MIT torque to qfrc_applied ---
  // tau = K_p * (q_d - q) + K_d * (qdot_d - qdot) + tau_ff
  for (std::uint8_t i = 0; i < joint_count_; ++i) {
    const int dof = joint_map_[i].dof_adr;
    const int qp = joint_map_[i].qpos_adr;
    if (dof < 0 || qp < 0) {
      continue;
    }

    const auto & cmd = command.joints[i];
    const double q = data_->qpos[qp];
    const double qdot = data_->qvel[dof];

    const double tau =
      cmd.stiffness_nm_rad * (cmd.position_rad - q) +
      cmd.damping_nm_s_rad * (cmd.velocity_rad_s - qdot) +
      cmd.effort_nm;

    data_->qfrc_applied[dof] = tau;
  }

  // --- Step physics: n_substeps of dt_physics ---
  for (int s = 0; s < n_substeps_; ++s) {
    mj_step(model_, data_);
  }

  // --- Check deadline ---
  const auto t_end = std::chrono::steady_clock::now();
  if (t_end > deadline) {
    ++deadline_misses_;
  }

  // --- Read feedback ---
  feedback.sequence = command.sequence;
  feedback.stamp = t_end;
  feedback.joint_count = joint_count_;
  feedback.availability_mask = command.availability_mask;
  feedback.availability_epoch = command.availability_epoch;

  for (std::uint8_t i = 0; i < joint_count_; ++i) {
    const int dof = joint_map_[i].dof_adr;
    const int qp = joint_map_[i].qpos_adr;

    auto & fb = feedback.joints[i];
    if (dof >= 0 && qp >= 0) {
      fb.position_rad = data_->qpos[qp];
      fb.velocity_rad_s = data_->qvel[dof];
      fb.effort_nm = data_->qfrc_applied[dof];
      fb.fresh = true;
    } else {
      fb.position_rad = 0.0;
      fb.velocity_rad_s = 0.0;
      fb.effort_nm = 0.0;
      fb.fresh = false;
    }
    fb.temperature_c = 0.0F;
    fb.bus_voltage_v = 0.0F;
    fb.fault_bits = 0;
  }

  // --- Update health ---
  last_sequence_ = command.sequence;
  const auto round_trip = std::chrono::duration_cast<std::chrono::nanoseconds>(
    t_end - t_start);
  if (round_trip > worst_round_trip_) {
    worst_round_trip_ = round_trip;
  }

  result.error = transport::TransportError::kNone;
  result.joints_reported = joint_count_;
  result.round_trip = round_trip;
  return result;
}

// ---------------------------------------------------------------------------
// Disable requests (sim: no-op, returns true)
// ---------------------------------------------------------------------------

bool MujocoActuatorTransport::request_joint_disable(
  transport::JointIndex /*joint*/) noexcept
{
  // In simulation, "disable" means zero the applied torque for that joint.
  // The SafetyKernel handles the protective command; the transport just
  // acknowledges the request.
  return true;
}

bool MujocoActuatorTransport::request_all_disable() noexcept
{
  // Zero all applied forces.
  if (data_) {
    for (int i = 0; i < model_->nv; ++i) {
      data_->qfrc_applied[i] = 0.0;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// health_snapshot
// ---------------------------------------------------------------------------

transport::HealthSnapshot MujocoActuatorTransport::health_snapshot() const noexcept
{
  transport::HealthSnapshot snap{};
  snap.last_sequence = last_sequence_;
  snap.exchanges_attempted = exchanges_attempted_;
  snap.exchanges_failed = exchanges_failed_;
  snap.deadline_misses = deadline_misses_;
  snap.framing_errors = 0;
  snap.sequence_rejections = 0;
  snap.worst_round_trip = worst_round_trip_;
  snap.availability_epoch = 0;
  snap.active = active_;
  return snap;
}

}  // namespace humanoid::transport_mujoco

PLUGINLIB_EXPORT_CLASS(
  humanoid::transport_mujoco::MujocoActuatorTransport,
  humanoid::transport::ActuatorTransport)