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

#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "humanoid_transport/actuator_transport.hpp"
#include "humanoid_transport/latest_value_buffer.hpp"

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

// SIL-only ground-truth snapshot, handed from the real-time exchange() thread
// to the non-real-time visualization thread. Deliberately holds no reference
// to mjData/mjModel: the viz thread never touches MuJoCo state directly, only
// this plain copy.
//
// Extension point: additional SIL-only ground-truth fields (true CoM,
// per-foot contact state, ...) can be added here without touching the
// ActuatorTransport contract or the exchange() signature.
struct GroundTruthFrame
{
  bool valid{false};
  // Control-cycle sequence of the exchange() that produced this sample. The
  // viz thread publishes only when it changes, so a stalled or deactivated
  // simulation never republishes a stale pose under a fresh timestamp.
  transport::CycleSequence sequence{0};
  // Monotonic instant the sample was taken (after the physics step). Lets the
  // viz thread stamp the TF with the sample time, not its own publish time.
  transport::MonotonicStamp captured{};
  // MuJoCo simulation time after the step, seconds. Drives the viz thread's
  // once-per-simulated-second diagnostic log.
  double sim_time_s{0.0};
  double position[3]{0.0, 0.0, 0.0};        // sim_world frame, metres
  double orientation_wxyz[4]{1.0, 0.0, 0.0, 0.0};
};

// Thin wrapper over humanoid_transport::LatestValueBuffer<GroundTruthFrame>.
// Producer is the real-time exchange() call, consumer is the non-real-time
// visualization thread (roles reversed relative to humanoid_mit_controller's
// ReferenceBuffer, which shares the same underlying template).
class GroundTruthBuffer
{
public:
  void publish(const GroundTruthFrame & frame) noexcept
  {
    buffer_.publish(frame);
  }

  [[nodiscard]] bool read(GroundTruthFrame & out) noexcept
  {
    return buffer_.read(out);
  }

private:
  transport::LatestValueBuffer<GroundTruthFrame> buffer_;
};

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
  // --- configure() steps, in order ---
  [[nodiscard]] bool load_model();
  [[nodiscard]] bool validate_substeps();
  [[nodiscard]] bool map_joints(const transport::JointManifest & joints);
  [[nodiscard]] bool parse_perturbation_params();

  MjModelPtr model_;
  MjDataPtr data_;

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

  // qpos offset of the pelvis freejoint, resolved once at load time so
  // exchange() never does a string-based MuJoCo lookup on the real-time path.
  int root_qpos_adr_{-1};

  // --- Diagnostic-only RViz2 visualization (ADR-001) ---
  //
  // exchange() only ever writes a plain snapshot into ground_truth_buffer_.
  // All ROS/TF work (node, publisher, sendTransform) happens on viz_thread_,
  // which is not part of the real-time read()/update()/write() path.
  //
  // Frames: sim_world -> ground_truth/pelvis. Both ends are deliberately
  // outside the production TF tree:
  //   - sim_world (the MuJoCo floor) is not odom or map: this is exact,
  //     noise-free ground truth, not an estimator output (Topic 6 D1).
  //   - The child is NOT the URDF root "pelvis". tf2 permits one parent per
  //     frame, and the Tier 0 estimator owns odom -> pelvis (REP-105). Were
  //     the simulator to also parent "pelvis", the two would fight in the
  //     tree, and every production consumer resolving odom -> pelvis in SIL
  //     would see the simulator's truth or a flip-flop between the two.
  //     That breaks ADR-007-01: SIL would no longer exercise the production
  //     data path it claims to test.
  // A second robot_state_publisher with frame_prefix "ground_truth/" hangs
  // the link tree off ground_truth/pelvis for RViz (see sil_stand.launch.py).
  static constexpr const char * kWorldFrame = "sim_world";
  static constexpr const char * kGroundTruthRootFrame = "ground_truth/pelvis";

  void viz_thread_main();

  rclcpp::Node::SharedPtr viz_node_{nullptr};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
  GroundTruthBuffer ground_truth_buffer_;
  std::thread viz_thread_;
  std::atomic<bool> viz_thread_running_{false};
};

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__MUJOCO_ACTUATOR_TRANSPORT_HPP_
