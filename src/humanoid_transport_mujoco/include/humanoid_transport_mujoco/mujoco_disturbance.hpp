// Copyright 2026 UTRA-RoboSoccer
// SIL-only external disturbance: a scheduled force pulse on one body.
//
// This is simulator scenario control, not transport fault injection. A
// FaultInjectingTransport (ADR-007-08) decorates any ActuatorTransport and can
// only corrupt, drop, delay, or replay command/feedback batches; it cannot
// push a body, and on hardware there is no body to push. ADR-007 lists "fall
// or impact: applied external wrench" under simulator-side injection, which is
// what this class is. MujocoActuatorTransport owns it only because physics is
// stepped inside exchange(), and only that thread may touch mjData.
#ifndef HUMANOID_TRANSPORT_MUJOCO__MUJOCO_DISTURBANCE_HPP_
#define HUMANOID_TRANSPORT_MUJOCO__MUJOCO_DISTURBANCE_HPP_

#include <array>
#include <optional>
#include <string>

struct mjModel_;
struct mjData_;

namespace humanoid::transport_mujoco
{

struct PushConfig
{
  /// Force magnitude. Zero disables the push.
  double force_n{0.0};
  /// Simulation time at which the pulse starts.
  double start_time_s{0.0};
  double duration_s{0.2};
  std::string body{"torso_link"};
  /// World-frame direction. Normalised by configure(); default is lateral (+y).
  std::array<double, 3> direction{0.0, 1.0, 0.0};
};

class MujocoDisturbance
{
public:
  // --- Non-real-time ---

  /// Resolves the body and validates the config against `model`.
  /// Returns an error message, or nullopt on success.
  [[nodiscard]] std::optional<std::string> configure(
    const mjModel_ & model, const PushConfig & config);

  // --- Real-time ---

  /// Writes the pulse force into d.xfrc_applied for the current step, or
  /// zero outside the pulse window. Stateless in time: the window alone
  /// decides, so a reset simulation (time back to 0) re-arms it for free.
  void apply(mjData_ & data) const noexcept;

private:
  int body_id_{-1};  // -1 = disabled
  std::array<double, 3> force_n_{0.0, 0.0, 0.0};
  double start_time_s_{0.0};
  double end_time_s_{0.0};
};

}  // namespace humanoid::transport_mujoco

#endif  // HUMANOID_TRANSPORT_MUJOCO__MUJOCO_DISTURBANCE_HPP_
