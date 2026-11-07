// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_actuator_system/four_bar_linkage.hpp"

#include <format>
#include <numbers>

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <variant>

namespace humanoid::actuator_system
{

namespace
{
struct Vec
{
  double x;
  double y;
};

// The zero pose must close to this. The description carries the parameters to 10 significant
// digits, which leaves ~1e-9 rad; anything larger is a model error, not rounding.
constexpr double kZeroPoseToleranceRad = 1e-6;
// Samples of the output's range that create() checks closure, toggles and the round trip at.
constexpr int kRangeSamples = 2001;
constexpr double kRoundTripToleranceRad = 1e-9;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double wrap(double angle) noexcept;
double cross(Vec u, Vec v) noexcept;
Vec input_tip(const FourBarParameters & p, double input_rad) noexcept;
Vec output_tip(const FourBarParameters & p, double output_rad) noexcept;
std::optional<Vec> intersect(Vec c0, double r0, Vec c1, double r1, double side) noexcept;
FourBarConfiguration configuration(const FourBarParameters & p, Vec input, Vec output) noexcept;
FourBarConfiguration unreachable() noexcept;
std::optional<std::string> parameters_violation(const FourBarParameters & p);
}  // namespace

// ===========================================================================
// Construction
// ===========================================================================

std::variant<FourBarLinkage, std::string> FourBarLinkage::create(
  const FourBarParameters & parameters, double output_lower_rad, double output_upper_rad)
{
  if (const auto violation = parameters_violation(parameters)) {
    return *violation;
  }
  if (!(std::isfinite(output_lower_rad) && std::isfinite(output_upper_rad) &&
    output_lower_rad < 0.0 && 0.0 < output_upper_rad))
  {
    return std::format(
      "output limits [{}, {}] must be finite and contain the zero pose", output_lower_rad,
      output_upper_rad);
  }

  // The assembly mode: the intersection that reproduces the zero pose, solving either way.
  const Vec ground{parameters.ground_length_m, 0.0};
  const auto side_that_closes = [](auto solve, double expected_angle) -> std::optional<double> {
      for (const double side : {1.0, -1.0}) {
        if (const auto tip = solve(side)) {
          if (std::abs(wrap(std::atan2(tip->y, tip->x) - expected_angle)) <
            kZeroPoseToleranceRad)
          {
            return side;
          }
        }
      }
      return std::nullopt;
    };
  const auto from_output_side = side_that_closes(
    [&](double side) -> std::optional<Vec> {
      const auto tip = intersect(
        ground, parameters.input_crank_length_m, output_tip(parameters, 0.0),
        parameters.coupler_length_m, side);
      return tip ? std::optional<Vec>{Vec{tip->x - ground.x, tip->y}} : std::nullopt;
    }, parameters.input_zero_angle_rad);
  const auto from_input_side = side_that_closes(
    [&](double side) {
      return intersect(
        Vec{0.0, 0.0}, parameters.output_crank_length_m, input_tip(parameters, 0.0),
        parameters.coupler_length_m, side);
    }, parameters.output_zero_angle_rad);
  if (!from_output_side || !from_input_side) {
    return std::string{"the linkage does not close at its zero pose"};
  }

  const FourBarLinkage linkage{parameters, *from_output_side, *from_input_side};
  double first_ratio = 0.0;
  for (int k = 0; k < kRangeSamples; ++k) {
    const double output = output_lower_rad + (output_upper_rad - output_lower_rad) * k /
      (kRangeSamples - 1);
    const auto forward = linkage.from_output(output);
    const auto back = linkage.from_input(forward.input_rad);
    if (!std::isfinite(forward.ratio) || forward.ratio == 0.0) {
      return std::format("the linkage cannot close at output {} rad, inside its limits", output);
    }
    if (k == 0) {
      first_ratio = forward.ratio;
    } else if ((forward.ratio > 0.0) != (first_ratio > 0.0)) {
      return std::format("the linkage passes a toggle at output {} rad, inside its limits", output);
    }
    if (!(std::abs(back.output_rad - output) < kRoundTripToleranceRad)) {
      return std::format(
        "the motor's angle does not determine the output at {} rad (round trip gives {})", output,
        back.output_rad);
    }
  }
  return linkage;
}

FourBarLinkage::FourBarLinkage(
  const FourBarParameters & parameters, double side_from_output, double side_from_input) noexcept
: p_{parameters}, side_from_output_{side_from_output}, side_from_input_{side_from_input}
{
}

// ===========================================================================
// Real-time maps
// ===========================================================================

FourBarConfiguration FourBarLinkage::from_output(double output_rad) const noexcept
{
  const Vec output = output_tip(p_, output_rad);
  const auto input = intersect(
    Vec{p_.ground_length_m, 0.0}, p_.input_crank_length_m, output, p_.coupler_length_m,
    side_from_output_);
  return input ? configuration(p_, *input, output) : unreachable();
}

FourBarConfiguration FourBarLinkage::from_input(double input_rad) const noexcept
{
  const Vec input = input_tip(p_, input_rad);
  const auto output = intersect(
    Vec{0.0, 0.0}, p_.output_crank_length_m, input, p_.coupler_length_m, side_from_input_);
  return output ? configuration(p_, input, *output) : unreachable();
}

transport::JointCommand FourBarLinkage::to_input(
  const transport::JointCommand & output) const noexcept
{
  const auto c = from_output(output.position_rad);
  const double ratio_squared = c.ratio * c.ratio;
  return transport::JointCommand{
    .position_rad = c.input_rad,
    .velocity_rad_s = output.velocity_rad_s / c.ratio,
    .effort_nm = output.effort_nm * c.ratio,
    .stiffness_nm_rad = output.stiffness_nm_rad * ratio_squared,
    .damping_nm_s_rad = output.damping_nm_s_rad * ratio_squared,
  };
}

FourBarStates FourBarLinkage::to_joints(const transport::JointFeedback & input) const noexcept
{
  const auto c = from_input(input.position_rad);
  FourBarStates states{input, input, input};
  states.output.position_rad = c.output_rad;
  states.output.velocity_rad_s = c.ratio * input.velocity_rad_s;
  states.output.effort_nm = input.effort_nm / c.ratio;
  states.coupler.position_rad = c.coupler_rad;
  states.coupler.velocity_rad_s = c.coupler_per_output * states.output.velocity_rad_s;
  states.coupler.effort_nm = std::isfinite(c.ratio) ? 0.0 : kNaN;
  return states;
}

namespace
{

/// `angle` in [-pi, pi].
double wrap(double angle) noexcept
{
  return std::remainder(angle, 2.0 * std::numbers::pi);
}

double cross(Vec u, Vec v) noexcept
{
  return u.x * v.y - u.y * v.x;
}

Vec input_tip(const FourBarParameters & p, double input_rad) noexcept
{
  const double angle = p.input_zero_angle_rad + p.input_axis_sign * input_rad;
  return Vec{
    p.ground_length_m + p.input_crank_length_m * std::cos(angle),
    p.input_crank_length_m * std::sin(angle)};
}

Vec output_tip(const FourBarParameters & p, double output_rad) noexcept
{
  const double angle = p.output_zero_angle_rad + p.output_axis_sign * output_rad;
  return Vec{p.output_crank_length_m * std::cos(angle), p.output_crank_length_m * std::sin(angle)};
}

/// The intersection of circle (c0, r0) and circle (c1, r1) on `side` (+1: left of c0 -> c1), or
/// nullopt where they do not meet.
std::optional<Vec> intersect(Vec c0, double r0, Vec c1, double r1, double side) noexcept
{
  const Vec d{c1.x - c0.x, c1.y - c0.y};
  const double distance = std::hypot(d.x, d.y);
  const double along = (distance * distance + r0 * r0 - r1 * r1) / (2.0 * distance);
  const double across_squared = r0 * r0 - along * along;
  if (!(across_squared >= 0.0)) {
    return std::nullopt;
  }
  const Vec u{d.x / distance, d.y / distance};
  const double across = side * std::sqrt(across_squared);
  return Vec{c0.x + along * u.x - across * u.y, c0.y + along * u.y + across * u.x};
}

FourBarConfiguration configuration(const FourBarParameters & p, Vec input, Vec output) noexcept
{
  const Vec e_in{input.x - p.ground_length_m, input.y};   // input crank, pivot to tip
  const Vec e_out = output;                                // output crank, pivot to tip
  const Vec r{input.x - output.x, input.y - output.y};     // coupler, output tip to input tip
  const double output_angle = std::atan2(e_out.y, e_out.x);

  // Velocity closure r . (v_in - v_out) = 0, with each tip's velocity the crank's rate times the
  // crank turned 90 degrees, gives the planar rate ratio.
  const double dout_din = cross(e_in, r) / cross(e_out, r);
  // The coupler's planar rate per unit output rate, from the tips' velocities.
  const Vec v_in{-e_in.y / dout_din, e_in.x / dout_din};
  const Vec v_out{-e_out.y, e_out.x};
  const double dcoupler =
    cross(r, Vec{v_in.x - v_out.x, v_in.y - v_out.y}) / (r.x * r.x + r.y * r.y);

  return FourBarConfiguration{
    .input_rad = p.input_axis_sign * wrap(std::atan2(e_in.y, e_in.x) - p.input_zero_angle_rad),
    .output_rad = p.output_axis_sign * wrap(output_angle - p.output_zero_angle_rad),
    .coupler_rad = p.coupler_axis_sign *
      wrap(std::atan2(r.y, r.x) - output_angle - p.coupler_zero_angle_rad),
    .ratio = p.output_axis_sign * p.input_axis_sign * dout_din,
    .coupler_per_output = p.coupler_axis_sign * p.output_axis_sign * (dcoupler - 1.0),
  };
}

FourBarConfiguration unreachable() noexcept
{
  return FourBarConfiguration{kNaN, kNaN, kNaN, kNaN, kNaN};
}

std::optional<std::string> parameters_violation(const FourBarParameters & p)
{
  for (const double length : {p.ground_length_m, p.input_crank_length_m, p.coupler_length_m,
      p.output_crank_length_m})
  {
    if (!(std::isfinite(length) && length > 0.0)) {
      return std::format("link length {} m is not finite and positive", length);
    }
  }
  for (const double angle : {p.input_zero_angle_rad, p.output_zero_angle_rad,
      p.coupler_zero_angle_rad})
  {
    if (!std::isfinite(angle)) {
      return std::string{"a zero angle is not finite"};
    }
  }
  for (const int sign : {p.input_axis_sign, p.output_axis_sign, p.coupler_axis_sign}) {
    if (sign != 1 && sign != -1) {
      return std::format("axis sign {} is not +1 or -1", sign);
    }
  }
  return std::nullopt;
}

}  // namespace

}  // namespace humanoid::actuator_system
