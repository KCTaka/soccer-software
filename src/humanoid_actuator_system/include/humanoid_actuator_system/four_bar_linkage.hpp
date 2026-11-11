// Copyright 2026 UTRA-RoboSoccer
// The exact closed-form kinematics of a planar four-bar linkage: the transmission between a joint
// controllers command (the linkage's output) and the joint its motor turns (the input).
//
// The equations are model/generators/linkage.py's, which derives the parameters from the robot
// model and writes them into the description's <transmission>. Given either crank's angle, the loop
// closes at an intersection of two circles; the assembly mode is the one the zero pose closes in,
// and it cannot change without passing a toggle, which create() rejects inside the output's limits.
// Only there is the map one-to-one, so commands are accepted only there. No table and no fit: the
// map is exact, smooth, and costs a few trigonometric calls.
#ifndef HUMANOID_ACTUATOR_SYSTEM__FOUR_BAR_LINKAGE_HPP_
#define HUMANOID_ACTUATOR_SYSTEM__FOUR_BAR_LINKAGE_HPP_

#include <optional>
#include <string>
#include <variant>

#include "humanoid_transport/batch_types.hpp"

namespace humanoid::actuator_system
{

/// A planar four-bar in the frame model/generators/linkage.py defines: origin at the output pivot,
/// x toward the input pivot, normal along the output joint's axis. The zero angles are the cranks'
/// and the coupler's directions in that frame with every joint at 0 (the coupler's relative to the
/// output crank). A joint whose axis points against the normal has sign -1.
struct FourBarParameters
{
  double ground_length_m{0.0};
  double input_crank_length_m{0.0};
  double coupler_length_m{0.0};
  double output_crank_length_m{0.0};
  double input_zero_angle_rad{0.0};
  double output_zero_angle_rad{0.0};
  double coupler_zero_angle_rad{0.0};
  int input_axis_sign{1};
  int output_axis_sign{1};
  int coupler_axis_sign{1};
};

/// The three joint angles at one configuration and the linkage's velocity ratios there. Every field
/// is NaN where the linkage cannot close.
struct FourBarConfiguration
{
  double input_rad;
  double output_rad;
  double coupler_rad;
  /// d(output) / d(input). Torque goes the other way: output torque = input torque / ratio.
  double ratio;
  /// d(coupler) / d(output).
  double coupler_per_output;
};

/// The output, input and coupler joints' states, derived from the motor's feedback.
struct FourBarStates
{
  transport::JointFeedback output;
  transport::JointFeedback input;
  transport::JointFeedback coupler;
};

class FourBarLinkage
{
public:
  /// Non-real-time. The linkage `parameters` describe, or why they cannot be used. It must close
  /// at its zero pose, with the input at 0 when the output is, and over all of
  /// [output_lower_rad, output_upper_rad] without passing a toggle, where the ratio is unbounded
  /// and the motor's angle no longer determines the output's. to_input() accepts only that range.
  [[nodiscard]] static std::variant<FourBarLinkage, std::string> create(
    const FourBarParameters & parameters, double output_lower_rad, double output_upper_rad);

  // --- Real-time: no allocation, no exceptions. ---

  [[nodiscard]] FourBarConfiguration from_output(double output_rad) const noexcept;

  [[nodiscard]] FourBarConfiguration from_input(double input_rad) const noexcept;

  /// The motor's MIT tuple that realises `output`, an MIT tuple for the output joint, or nullopt
  /// if its position is outside the output's limits. Outside them the map is not one-to-one: past
  /// a toggle, or a turn away, a command can reach a motor angle that an output angle inside the
  /// limits also maps to, so no check in motor space could reject it.
  ///   position  the input angle at the commanded output angle;
  ///   velocity  output velocity / ratio;
  ///   effort    output torque * ratio (virtual work);
  ///   stiffness, damping  output gains * ratio^2,
  /// with the ratio r at the commanded position. There, the output torque is the commanded one.
  /// So is its static stiffness, unless a feed-forward torque is commanded, which adds
  /// effort * r' / r (r' = dr/d(output)). Away from it the spring's torque error is second order
  /// in the distance.
  [[nodiscard]] std::optional<transport::JointCommand> to_input(
    const transport::JointCommand & output) const noexcept;

  /// The joints' states from the motor's feedback `input`: the output's angle through the linkage,
  /// its velocity (input velocity * ratio) and torque (input torque / ratio, neglecting the light
  /// crank's and coupler's inertia); the input's as measured; the coupler's angle and velocity, and
  /// zero torque, since a free pin carries none about its axis. The fault, freshness and electrical
  /// fields are the motor's. NaN angles, velocities and torques where the linkage cannot close.
  [[nodiscard]] FourBarStates to_joints(const transport::JointFeedback & input) const noexcept;

private:
  // --- Member functions ---

  FourBarLinkage(
    const FourBarParameters & parameters, double output_lower_rad, double output_upper_rad,
    double side_from_output, double side_from_input) noexcept;

  // --- Data ---

  FourBarParameters p_;
  /// The output's limits, which create() checked the linkage over.
  double output_lower_rad_;
  double output_upper_rad_;
  /// Which intersection (+1: left of the line between the circles' centres) the zero pose closes
  /// in, solving for the input tip from the output and for the output tip from the input.
  double side_from_output_;
  double side_from_input_;
};

}  // namespace humanoid::actuator_system

#endif  // HUMANOID_ACTUATOR_SYSTEM__FOUR_BAR_LINKAGE_HPP_
