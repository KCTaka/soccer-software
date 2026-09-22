// Copyright 2026 Your Organization Name

#include "humanoid_transport_mujoco/mujoco_actuator_transport.hpp"

#include <mujoco/mujoco.h>

#include <cstdlib>
#include <cstring>
#include <chrono>
#include <iostream>

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

  viz_node_ = rclcpp::Node::make_shared("mujoco_viz");
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*viz_node_);

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

  const char * push_force = std::getenv("HUMANOID_PUSH_FORCE_N");
  const char * push_time = std::getenv("HUMANOID_PUSH_TIME_S");
  const char * push_body = std::getenv("HUMANOID_PUSH_BODY");
  if (push_force) {push_force_n_ = std::atof(push_force);}
  if (push_time) {push_time_s_ = std::atof(push_time);}
  if (!push_body && push_force_n_ > 0.0) {push_body = "torso_link";}
  if (push_body) {push_body_id_ = mj_name2id(model_, mjOBJ_BODY, push_body);}
  if (push_force_n_ > 0.0 && push_body_id_ < 0) {
    std::cerr << "MujocoActuatorTransport: push body not found in model\n";
    return false;
  }

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
  mj_resetData(model_, data_);

  // Set the freejoint height so feet rest on the ground plane.
  // The freejoint qpos layout is [x, y, z, qw, qx, qy, qz].
  // Compute the pelvis height needed for the lowest foot contact
  // sphere to sit on the z=0 ground plane.
  int root_jnt = mj_name2id(model_, mjOBJ_JOINT, "root");
  if (root_jnt >= 0 && model_->jnt_type[root_jnt] == mjJNT_FREE) {
    int qpos_adr = model_->jnt_qposadr[root_jnt];

    // Compute pelvis height from the leg chain geometry.
    // Sum the z-offsets from pelvis to ankle_roll, plus the
    // contact sphere offset and radius.
    //
    // hip_pitch z: -0.1027
    // hip_roll  z: -0.030465
    // hip_yaw   z: -0.12412
    // knee      z: -0.17734
    // ankle_pitch z: -0.30001
    // ankle_roll  z: -0.017558
    // contact sphere z offset: -0.03, radius: 0.005
    // Total leg length: ~0.787 m
    constexpr double kPelvisHeight = 0.793;

    data_->qpos[qpos_adr + 2] = kPelvisHeight;  // z
    data_->qpos[qpos_adr + 3] = 1.0;            // qw (identity quaternion)
  }

  // Set the initial joint positions to match the first keyframe
  // so the controller starts from the slumped pose, not all-zeros.
  // This avoids a large initial tracking error impulse.
  const double initial_positions[] = {
    -0.5, 0.0, 0.0, 1.0, -0.5, 0.0,   // left leg
    -0.5, 0.0, 0.0, 1.0, -0.5, 0.0,   // right leg
     0.0, 0.0, 0.3,                    // waist
     0.3, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,  // left arm
     0.3, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0   // right arm
  };
  for (std::uint8_t i = 0; i < joint_count_ && i < 29; ++i) {
    int qp = joint_map_[i].qpos_adr;
    if (qp >= 0) {
      data_->qpos[qp] = initial_positions[i];
    }
  }

  // Settle the contact state.
  mj_forward(model_, data_);

  push_started_ = false;
  push_active_ = false;
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

  // Apply a single lateral force pulse before the first step at or after the
  // configured simulation time. MuJoCo retains xfrc_applied until cleared.
  constexpr double kPushDurationS = 0.2;
  const double sim_time = data_->time;
  if (push_force_n_ > 0.0 && push_body_id_ >= 0) {
    if (!push_started_ && sim_time >= push_time_s_) {
      push_started_ = true;
      push_active_ = true;
    }
    if (push_active_) {
      if (sim_time < push_time_s_ + kPushDurationS) {
        data_->xfrc_applied[6 * push_body_id_ + push_axis_] = push_force_n_;
      } else {
        data_->xfrc_applied[6 * push_body_id_ + push_axis_] = 0.0;
        push_active_ = false;
      }
    }
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

  if (command.sequence % 200 == 0) {  // log once per second
    int pelvis_body = mj_name2id(model_, mjOBJ_BODY, "pelvis");
    if (pelvis_body >= 0) {
      std::cerr << "[MuJoCo] t=" << data_->time
        << " pelvis_z=" << data_->xpos[3 * pelvis_body + 2]
        << "\n";
    }
  }

  // Publish base pose at 20 Hz (every 10th cycle) for RViz2 visualization.
  // This is diagnostic-only, not part of the control path.
  if (++viz_counter_ % 10 == 0 && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = viz_node_->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "pelvis";
    t.transform.translation.x = data_->qpos[0];
    t.transform.translation.y = data_->qpos[1];
    t.transform.translation.z = data_->qpos[2];
    t.transform.rotation.w = data_->qpos[3];
    t.transform.rotation.x = data_->qpos[4];
    t.transform.rotation.y = data_->qpos[5];
    t.transform.rotation.z = data_->qpos[6];
    tf_broadcaster_->sendTransform(t);
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
