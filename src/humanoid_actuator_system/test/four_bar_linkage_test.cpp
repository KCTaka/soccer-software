// Copyright 2026 UTRA-RoboSoccer
// FourBarLinkage: exact on a parallelogram, whose kinematics are known in closed form, and then on
// every linkage a generated robot holds, as the component builds it from the description. Each
// generated check is against geometry rebuilt here from the transmission's parameters, never
// against numbers copied from a robot.

#include <gmock/gmock.h>

#include <numbers>

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "generated_description.hpp"
#include "humanoid_actuator_system/four_bar_linkage.hpp"
#include "humanoid_test_support/matchers.hpp"
#include "humanoid_transport/batch_types.hpp"

namespace
{

using humanoid::actuator_system::FourBarLinkage;
using humanoid::actuator_system::FourBarParameters;
using humanoid::actuator_system::testing::generated_linkages;
using humanoid::actuator_system::testing::GeneratedLinkage;
using humanoid::actuator_system::testing::LinkageName;
using humanoid::actuator_system::testing::load;
using humanoid::actuator_system::testing::LoadedLinkage;
using humanoid::actuator_system::testing::samples;
using humanoid::test_support::HoldsError;
using humanoid::test_support::HoldsValue;
using humanoid::transport::JointCommand;
using humanoid::transport::JointFeedback;
using ::testing::_;

constexpr double kPi = std::numbers::pi;

// ---------------------------------------------------------------------------
// A parallelogram
// ---------------------------------------------------------------------------

/// Equal cranks pointing up at zero, the coupler as long as the ground. Its kinematics are known
/// exactly: the input follows the output one to one and the coupler stays parallel to the ground,
/// so relative to the output crank it turns the other way.
FourBarParameters parallelogram()
{
  FourBarParameters p;
  p.ground_length_m = 0.12;
  p.input_crank_length_m = 0.04;
  p.coupler_length_m = 0.12;
  p.output_crank_length_m = 0.04;
  p.input_zero_angle_rad = kPi / 2.0;
  p.output_zero_angle_rad = kPi / 2.0;
  p.coupler_zero_angle_rad = -kPi / 2.0;
  return p;
}

TEST(FourBarLinkage, ReproducesAParallelogramExactly)
{
  const auto made = FourBarLinkage::create(parallelogram(), -1.0, 1.0);
  ASSERT_THAT(made, HoldsValue());
  const auto & linkage = std::get<FourBarLinkage>(made);
  for (const double q : samples(-1.0, 1.0)) {
    SCOPED_TRACE("output " + std::to_string(q) + " rad");
    const auto c = linkage.from_output(q);
    EXPECT_NEAR(c.input_rad, q, 1e-12);
    EXPECT_NEAR(c.coupler_rad, -q, 1e-12);
    EXPECT_NEAR(c.ratio, 1.0, 1e-12);
    EXPECT_NEAR(c.coupler_per_output, -1.0, 1e-12);
  }
}

TEST(FourBarLinkage, RejectsALinkageThatDoesNotCloseAtZero)
{
  auto p = parallelogram();
  p.coupler_length_m += 0.001;
  EXPECT_THAT(FourBarLinkage::create(p, -1.0, 1.0), HoldsError(_));
}

TEST(FourBarLinkage, RejectsMalformedParameters)
{
  auto length = parallelogram();
  length.input_crank_length_m = 0.0;
  auto sign = parallelogram();
  sign.coupler_axis_sign = 0;
  auto angle = parallelogram();
  angle.output_zero_angle_rad = std::nan("");
  for (const auto & p : {length, sign, angle}) {
    EXPECT_THAT(FourBarLinkage::create(p, -1.0, 1.0), HoldsError(_));
  }
  // Limits that do not contain the zero pose.
  EXPECT_THAT(FourBarLinkage::create(parallelogram(), 0.1, 1.0), HoldsError(_));
}

// ---------------------------------------------------------------------------
// Every generated linkage
// ---------------------------------------------------------------------------

/// The crank tips in the linkage's frame (generated_description.hpp's FourBarParameters):
/// the output crank's from the output pivot at the origin, the input crank's from the input pivot
/// at (ground, 0), each crank at its zero angle plus its axis sign times its joint angle.
struct Tips
{
  double output_angle;
  double input_angle;
  double bx, by;  // output crank tip
  double tx, ty;  // input crank tip
};

Tips tips(const FourBarParameters & p, double output_rad, double input_rad)
{
  Tips t{};
  t.output_angle = p.output_zero_angle_rad + p.output_axis_sign * output_rad;
  t.input_angle = p.input_zero_angle_rad + p.input_axis_sign * input_rad;
  t.bx = p.output_crank_length_m * std::cos(t.output_angle);
  t.by = p.output_crank_length_m * std::sin(t.output_angle);
  t.tx = p.ground_length_m + p.input_crank_length_m * std::cos(t.input_angle);
  t.ty = p.input_crank_length_m * std::sin(t.input_angle);
  return t;
}

class GeneratedFourBar : public ::testing::TestWithParam<GeneratedLinkage>
{
protected:
  void SetUp() override
  {
    auto loaded = load(GetParam());
    ASSERT_THAT(loaded, HoldsValue());
    l_.emplace(std::get<LoadedLinkage>(std::move(loaded)));
  }

  const FourBarLinkage & linkage() const {return l_->linkage();}
  const FourBarParameters & p() const {return l_->parameters;}

  /// The outputs across one full turn that are not inside the limits.
  std::vector<double> outside_the_limits() const
  {
    std::vector<double> outside;
    for (const double q : samples(l_->upper, l_->lower + 2.0 * kPi, 721)) {
      if (q > l_->upper && q < l_->lower + 2.0 * kPi) {
        outside.push_back(q);
      }
    }
    return outside;
  }

  std::optional<LoadedLinkage> l_;
};

TEST_P(GeneratedFourBar, ClosesTheLoopAtEveryAngle)
{
  // Independent of how the solve went: rebuild both tips from the returned angles and check the
  // coupler's length and direction.
  for (const double q : samples(l_->lower, l_->upper)) {
    SCOPED_TRACE("output " + std::to_string(q) + " rad");
    const auto c = linkage().from_output(q);
    const auto t = tips(p(), c.output_rad, c.input_rad);
    EXPECT_NEAR(c.output_rad, q, 1e-12);
    EXPECT_NEAR(std::hypot(t.tx - t.bx, t.ty - t.by), p().coupler_length_m, 1e-12);
    EXPECT_NEAR(
      std::remainder(
        std::atan2(t.ty - t.by, t.tx - t.bx) -
        (t.output_angle + p().coupler_axis_sign * c.coupler_rad + p().coupler_zero_angle_rad),
        2.0 * kPi),
      0.0, 1e-12);
  }
}

TEST_P(GeneratedFourBar, MapsTheZeroPoseToZero)
{
  EXPECT_NEAR(linkage().from_output(0.0).input_rad, 0.0, 1e-9);
  EXPECT_NEAR(linkage().from_output(0.0).coupler_rad, 0.0, 1e-9);
  EXPECT_NEAR(linkage().from_input(0.0).output_rad, 0.0, 1e-9);
}

TEST_P(GeneratedFourBar, RoundTripsBetweenMotorAndJoint)
{
  for (const double q : samples(l_->lower, l_->upper)) {
    EXPECT_NEAR(linkage().from_input(linkage().from_output(q).input_rad).output_rad, q, 1e-12)
      << "output " << q << " rad";
  }
}

TEST_P(GeneratedFourBar, RatiosAreTheDerivativesOfThePositionMap)
{
  constexpr double kStep = 1e-6;
  for (const double q : samples(l_->lower + kStep, l_->upper - kStep)) {
    SCOPED_TRACE("output " + std::to_string(q) + " rad");
    const auto c = linkage().from_output(q);
    const auto ahead = linkage().from_output(q + kStep);
    const auto behind = linkage().from_output(q - kStep);
    const double din_dout = (ahead.input_rad - behind.input_rad) / (2.0 * kStep);
    const double dcpl_dout = (ahead.coupler_rad - behind.coupler_rad) / (2.0 * kStep);
    EXPECT_NEAR(c.ratio, 1.0 / din_dout, 1e-7);
    EXPECT_NEAR(c.coupler_per_output, dcpl_dout, 1e-7);
  }
}

TEST_P(GeneratedFourBar, TheRatioIsTheRigidCouplersVelocityRatio)
{
  // The coupler keeps its length, so both tips' velocities have the same component along it:
  // r_in * phi_in' * (n_in . u) = r_out * phi_out' * (n_out . u), with u the coupler's direction
  // and n a crank's direction turned a quarter. In joint angles, with the axis signs:
  // d(output)/d(input) = s_in s_out r_in (n_in . u) / (r_out (n_out . u)), exact at every angle.
  for (const double q : samples(l_->lower, l_->upper)) {
    SCOPED_TRACE("output " + std::to_string(q) + " rad");
    const auto c = linkage().from_output(q);
    const auto t = tips(p(), c.output_rad, c.input_rad);
    const double ux = t.tx - t.bx;
    const double uy = t.ty - t.by;
    const double input_along =
      -std::sin(t.input_angle) * ux + std::cos(t.input_angle) * uy;
    const double output_along =
      -std::sin(t.output_angle) * ux + std::cos(t.output_angle) * uy;
    const double expected = p().input_axis_sign * p().output_axis_sign *
      (p().input_crank_length_m * input_along) / (p().output_crank_length_m * output_along);
    EXPECT_NEAR(c.ratio, expected, 1e-9 * std::abs(expected));
  }
}

TEST_P(GeneratedFourBar, MapsTheMitTupleByVirtualWork)
{
  for (const double q : samples(l_->lower, l_->upper, 9)) {
    SCOPED_TRACE("output " + std::to_string(q) + " rad");
    const JointCommand joint{
      .position_rad = q, .velocity_rad_s = 1.5, .effort_nm = 0.7, .stiffness_nm_rad = 12.0,
      .damping_nm_s_rad = 0.4};
    const auto motor = linkage().to_input(joint);
    ASSERT_TRUE(motor);
    const double ratio = linkage().from_output(q).ratio;
    EXPECT_NEAR(motor->position_rad, linkage().from_output(q).input_rad, 1e-15);
    // Power is the same on both sides of the linkage.
    EXPECT_NEAR(
      motor->effort_nm * motor->velocity_rad_s, joint.effort_nm * joint.velocity_rad_s, 1e-12);
    EXPECT_NEAR(motor->stiffness_nm_rad, joint.stiffness_nm_rad * ratio * ratio, 1e-12);
    EXPECT_NEAR(motor->damping_nm_s_rad, joint.damping_nm_s_rad * ratio * ratio, 1e-12);
  }
}

TEST_P(GeneratedFourBar, MotorStiffnessGivesTheCommandedJointStiffness)
{
  // Displace the output by d from the setpoint: the motor's spring torque, carried back through
  // the linkage, is the commanded joint stiffness times d, to first order in d.
  constexpr double kStiffness = 20.0;
  for (const double q : samples(l_->lower + 0.01, l_->upper - 0.01, 9)) {
    const auto motor =
      linkage().to_input(JointCommand{.position_rad = q, .stiffness_nm_rad = kStiffness});
    ASSERT_TRUE(motor);
    for (const double d : {-1e-4, 1e-4}) {
      const auto displaced = linkage().from_output(q + d);
      const double torque = motor->stiffness_nm_rad * (motor->position_rad - displaced.input_rad);
      EXPECT_NEAR(torque / displaced.ratio, -kStiffness * d, 1e-3 * kStiffness * std::abs(d))
        << "output " << q << " rad, displaced by " << d;
    }
  }
}

TEST_P(GeneratedFourBar, CommandsOnlyInsideTheOutputsLimits)
{
  // Outside the limits the map is not one-to-one: past a toggle the motor swings back through
  // its range, and a full turn away lands on the same motor angle. Either way a command there can
  // reach a motor angle an output inside the limits also maps to, so no check in motor space
  // could reject it: to_input() must, by the output angle.
  for (const double q : {l_->lower, l_->upper}) {
    EXPECT_TRUE(linkage().to_input(JointCommand{.position_rad = q})) << q;
  }
  for (const double q : {std::nextafter(l_->lower, -kPi), std::nextafter(l_->upper, kPi)}) {
    EXPECT_FALSE(linkage().to_input(JointCommand{.position_rad = q})) << q;
  }
  for (const double q : outside_the_limits()) {
    EXPECT_FALSE(linkage().to_input(JointCommand{.position_rad = q})) << q;
  }
  const double motor_lower = linkage().from_output(l_->lower).input_rad;
  const double motor_upper = linkage().from_output(l_->upper).input_rad;
  for (const double q : samples(l_->lower, l_->upper, 9)) {
    const double a_turn_away = q + 2.0 * kPi;
    const double motor = linkage().from_output(a_turn_away).input_rad;
    EXPECT_GE(motor, std::min(motor_lower, motor_upper) - 1e-9) << a_turn_away;
    EXPECT_LE(motor, std::max(motor_lower, motor_upper) + 1e-9) << a_turn_away;
    EXPECT_FALSE(linkage().to_input(JointCommand{.position_rad = a_turn_away})) << a_turn_away;
  }
}

TEST_P(GeneratedFourBar, ReportsTheJointsFromTheMotor)
{
  const double mid = 0.5 * (l_->lower + l_->upper);
  const JointFeedback motor{
    .position_rad = linkage().from_output(mid).input_rad, .velocity_rad_s = -2.0,
    .effort_nm = 1.1, .temperature_c = 41.0F, .bus_voltage_v = 47.5F, .fault_bits = 0U,
    .fresh = true};
  const auto c = linkage().from_input(motor.position_rad);
  const auto states = linkage().to_joints(motor);
  EXPECT_NEAR(states.output.position_rad, mid, 1e-9);
  EXPECT_DOUBLE_EQ(states.output.position_rad, c.output_rad);
  EXPECT_DOUBLE_EQ(states.output.velocity_rad_s, c.ratio * motor.velocity_rad_s);
  EXPECT_DOUBLE_EQ(states.output.effort_nm, motor.effort_nm / c.ratio);
  EXPECT_DOUBLE_EQ(states.input.position_rad, motor.position_rad);
  EXPECT_DOUBLE_EQ(states.coupler.position_rad, c.coupler_rad);
  EXPECT_DOUBLE_EQ(
    states.coupler.velocity_rad_s, c.coupler_per_output * states.output.velocity_rad_s);
  EXPECT_EQ(states.coupler.effort_nm, 0.0);
  EXPECT_TRUE(states.output.fresh && states.coupler.fresh);
}

TEST_P(GeneratedFourBar, IsNaNWhereTheLoopCannotClose)
{
  // The motor's horn is the rocker: over a full turn of the output it swings through a range, and
  // past that range no output angle closes the loop.
  double swing_lower = std::numeric_limits<double>::infinity();
  double swing_upper = -swing_lower;
  for (const double q : samples(l_->lower, l_->lower + 2.0 * kPi, 3601)) {
    const double input = linkage().from_output(q).input_rad;
    if (std::isfinite(input)) {
      swing_lower = std::min(swing_lower, input);
      swing_upper = std::max(swing_upper, input);
    }
  }
  ASSERT_LT(swing_upper - swing_lower, 2.0 * kPi) << "the horn turns fully: it is not a rocker";
  for (const double beyond : {swing_upper + 0.05, swing_lower - 0.05}) {
    SCOPED_TRACE("motor " + std::to_string(beyond) + " rad, past the horn's swing");
    const auto c = linkage().from_input(beyond);
    EXPECT_TRUE(std::isnan(c.output_rad) && std::isnan(c.ratio));
    const auto states = linkage().to_joints(JointFeedback{.position_rad = beyond, .fresh = true});
    EXPECT_TRUE(std::isnan(states.output.position_rad) && std::isnan(states.coupler.effort_nm));
  }
}

TEST_P(GeneratedFourBar, RejectsLimitsThatReachAToggle)
{
  // A toggle is where the motor stops and turns back as the output keeps going: there
  // d(input)/d(output) = 1 / ratio changes sign. Find the first one above the upper limit, or the
  // first angle where the loop no longer closes, which create() must refuse as well.
  const auto turning_back = [this](double q, bool direction_at_limit) {
      const double ratio = linkage().from_output(q).ratio;
      return !std::isfinite(ratio) || std::signbit(1.0 / ratio) != direction_at_limit;
    };
  const bool at_limit = std::signbit(1.0 / linkage().from_output(l_->upper).ratio);
  std::optional<double> toggle;
  for (const double q : outside_the_limits()) {
    if (turning_back(q, at_limit)) {
      toggle = q;
      break;
    }
  }
  ASSERT_TRUE(toggle) << "no toggle within a turn above the upper limit";
  EXPECT_THAT(FourBarLinkage::create(p(), l_->lower, *toggle + 0.02), HoldsError(_));
  EXPECT_THAT(FourBarLinkage::create(p(), l_->lower, l_->upper), HoldsValue());
}

INSTANTIATE_TEST_SUITE_P(
  EveryLinkage, GeneratedFourBar, ::testing::ValuesIn(generated_linkages()), LinkageName{});

}  // namespace
