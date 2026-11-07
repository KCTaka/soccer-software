// Copyright 2026 UTRA-RoboSoccer

#include <gtest/gtest.h>

#include <numbers>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "humanoid_actuator_system/drive_layout.hpp"
#include "humanoid_actuator_system/four_bar_linkage.hpp"
#include "humanoid_transport/batch_types.hpp"

namespace
{

using humanoid::actuator_system::drive_layout;
using humanoid::actuator_system::DriveLayout;
using humanoid::actuator_system::FourBarLinkage;
using humanoid::actuator_system::FourBarParameters;
using humanoid::transport::JointCommand;
using humanoid::transport::JointFeedback;

constexpr double kPi = std::numbers::pi;

/// A parallelogram: equal cranks pointing up at zero, the coupler as long as the ground. Its
/// kinematics are known exactly: the input follows the output one to one and the coupler stays
/// parallel to the ground, so relative to the output crank it turns the other way.
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

FourBarLinkage make(const FourBarParameters & p, double lower, double upper)
{
  auto made = FourBarLinkage::create(p, lower, upper);
  if (const auto * error = std::get_if<std::string>(&made)) {
    ADD_FAILURE() << *error;
    return std::get<FourBarLinkage>(FourBarLinkage::create(parallelogram(), -1.0, 1.0));
  }
  return std::get<FourBarLinkage>(made);
}

/// The generated leg's linkage and its output limits, from the description the component loads.
struct Leg
{
  DriveLayout layout;
  FourBarParameters parameters;
  double lower;
  double upper;
};

Leg leg()
{
  std::ifstream file(HUMANOID_TEST_LEG_URDF_PATH);
  std::stringstream urdf;
  urdf << file.rdbuf();
  const auto info = hardware_interface::parse_control_resources_from_urdf(urdf.str()).front();
  auto layout = drive_layout(info);
  const auto & output = info.transmissions.front().joints.front().name;
  const auto & limits = info.limits.at(output);
  const auto & params = info.transmissions.front().parameters;
  FourBarParameters p;
  p.ground_length_m = std::stod(params.at("ground_length_m"));
  p.input_crank_length_m = std::stod(params.at("input_crank_length_m"));
  p.coupler_length_m = std::stod(params.at("coupler_length_m"));
  p.output_crank_length_m = std::stod(params.at("output_crank_length_m"));
  p.input_zero_angle_rad = std::stod(params.at("input_zero_angle_rad"));
  p.output_zero_angle_rad = std::stod(params.at("output_zero_angle_rad"));
  p.coupler_zero_angle_rad = std::stod(params.at("coupler_zero_angle_rad"));
  p.input_axis_sign = std::stoi(params.at("input_axis_sign"));
  p.output_axis_sign = std::stoi(params.at("output_axis_sign"));
  p.coupler_axis_sign = std::stoi(params.at("coupler_axis_sign"));
  return Leg{std::get<DriveLayout>(layout), p, limits.min_position, limits.max_position};
}

std::vector<double> samples(double lower, double upper, int count = 81)
{
  std::vector<double> out;
  for (int k = 0; k < count; ++k) {
    out.push_back(lower + (upper - lower) * k / (count - 1));
  }
  return out;
}

TEST(FourBarLinkage, ReproducesAParallelogramExactly)
{
  const auto linkage = make(parallelogram(), -1.0, 1.0);
  for (const double q : samples(-1.0, 1.0)) {
    const auto c = linkage.from_output(q);
    EXPECT_NEAR(c.input_rad, q, 1e-12);
    EXPECT_NEAR(c.coupler_rad, -q, 1e-12);
    EXPECT_NEAR(c.ratio, 1.0, 1e-12);
    EXPECT_NEAR(c.coupler_per_output, -1.0, 1e-12);
  }
}

TEST(FourBarLinkage, ClosesTheLegsLoopAtEveryAngle)
{
  // Independent of how the solve went: rebuild both tips from the returned angles and check the
  // coupler's length and direction.
  const auto l = leg();
  const auto & p = l.parameters;
  const auto & linkage = l.layout.linkages.front()->linkage;
  for (const double q : samples(l.lower, l.upper)) {
    const auto c = linkage.from_output(q);
    const double out = p.output_zero_angle_rad + p.output_axis_sign * c.output_rad;
    const double in = p.input_zero_angle_rad + p.input_axis_sign * c.input_rad;
    const double bx = p.output_crank_length_m * std::cos(out);
    const double by = p.output_crank_length_m * std::sin(out);
    const double tx = p.ground_length_m + p.input_crank_length_m * std::cos(in);
    const double ty = p.input_crank_length_m * std::sin(in);
    EXPECT_NEAR(c.output_rad, q, 1e-12);
    EXPECT_NEAR(std::hypot(tx - bx, ty - by), p.coupler_length_m, 1e-12);
    EXPECT_NEAR(
      std::remainder(
        std::atan2(ty - by, tx - bx) -
        (out + p.coupler_axis_sign * c.coupler_rad + p.coupler_zero_angle_rad), 2.0 * kPi),
      0.0, 1e-12);
  }
}

TEST(FourBarLinkage, MapsTheLegsZeroPoseToZero)
{
  const auto l = leg();
  const auto & linkage = l.layout.linkages.front()->linkage;
  EXPECT_NEAR(linkage.from_output(0.0).input_rad, 0.0, 1e-9);
  EXPECT_NEAR(linkage.from_output(0.0).coupler_rad, 0.0, 1e-9);
  EXPECT_NEAR(linkage.from_input(0.0).output_rad, 0.0, 1e-9);
}

TEST(FourBarLinkage, RoundTripsBetweenMotorAndJoint)
{
  const auto l = leg();
  const auto & linkage = l.layout.linkages.front()->linkage;
  for (const double q : samples(l.lower, l.upper)) {
    EXPECT_NEAR(linkage.from_input(linkage.from_output(q).input_rad).output_rad, q, 1e-12);
  }
}

TEST(FourBarLinkage, RatiosAreTheDerivativesOfThePositionMap)
{
  const auto l = leg();
  const auto & linkage = l.layout.linkages.front()->linkage;
  constexpr double kStep = 1e-6;
  for (const double q : samples(l.lower + kStep, l.upper - kStep)) {
    const auto c = linkage.from_output(q);
    const auto ahead = linkage.from_output(q + kStep);
    const auto behind = linkage.from_output(q - kStep);
    const double din_dout = (ahead.input_rad - behind.input_rad) / (2.0 * kStep);
    const double dcpl_dout = (ahead.coupler_rad - behind.coupler_rad) / (2.0 * kStep);
    EXPECT_NEAR(c.ratio, 1.0 / din_dout, 1e-7);
    EXPECT_NEAR(c.coupler_per_output, dcpl_dout, 1e-7);
  }
}

TEST(FourBarLinkage, TheLegsMotorTurnsLessThanTheAnkle)
{
  // The 45 mm horn drives a 40 mm crank: near neutral the ankle turns 45/40 times the motor, more
  // at the ends of travel as the transmission angle departs from 90 degrees.
  const auto l = leg();
  const auto & linkage = l.layout.linkages.front()->linkage;
  EXPECT_NEAR(linkage.from_output(0.0).ratio, 0.045 / 0.04, 1e-3);
  EXPECT_GT(linkage.from_output(l.upper).ratio, linkage.from_output(0.0).ratio);
  EXPECT_GT(linkage.from_output(l.lower).ratio, linkage.from_output(0.0).ratio);
}

TEST(FourBarLinkage, MapsTheMitTupleByVirtualWork)
{
  const auto l = leg();
  const auto & linkage = l.layout.linkages.front()->linkage;
  for (const double q : samples(l.lower, l.upper, 9)) {
    const JointCommand joint{
      .position_rad = q, .velocity_rad_s = 1.5, .effort_nm = 0.7, .stiffness_nm_rad = 12.0,
      .damping_nm_s_rad = 0.4};
    const auto motor = linkage.to_input(joint);
    const double ratio = linkage.from_output(q).ratio;
    EXPECT_NEAR(motor.position_rad, linkage.from_output(q).input_rad, 1e-15);
    // Power is the same on both sides of the linkage.
    EXPECT_NEAR(motor.effort_nm * motor.velocity_rad_s, joint.effort_nm * joint.velocity_rad_s,
      1e-12);
    EXPECT_NEAR(motor.stiffness_nm_rad, joint.stiffness_nm_rad * ratio * ratio, 1e-12);
    EXPECT_NEAR(motor.damping_nm_s_rad, joint.damping_nm_s_rad * ratio * ratio, 1e-12);
  }
}

TEST(FourBarLinkage, MotorStiffnessGivesTheCommandedJointStiffness)
{
  // Displace the ankle by d from the setpoint: the motor's spring torque, carried back through the
  // linkage, is the commanded joint stiffness times d, to first order in d.
  const auto l = leg();
  const auto & linkage = l.layout.linkages.front()->linkage;
  constexpr double kStiffness = 20.0;
  for (const double q : samples(l.lower + 0.01, l.upper - 0.01, 9)) {
    const auto motor =
      linkage.to_input(JointCommand{.position_rad = q, .stiffness_nm_rad = kStiffness});
    for (const double d : {-1e-4, 1e-4}) {
      const auto displaced = linkage.from_output(q + d);
      const double torque = motor.stiffness_nm_rad * (motor.position_rad - displaced.input_rad);
      EXPECT_NEAR(torque / displaced.ratio, -kStiffness * d, 1e-3 * kStiffness * std::abs(d));
    }
  }
}

TEST(FourBarLinkage, ReportsTheJointsFromTheMotor)
{
  const auto l = leg();
  const auto & linkage = l.layout.linkages.front()->linkage;
  const JointFeedback motor{
    .position_rad = 0.3, .velocity_rad_s = -2.0, .effort_nm = 1.1, .temperature_c = 41.0F,
    .bus_voltage_v = 47.5F, .fault_bits = 0U, .fresh = true};
  const auto c = linkage.from_input(motor.position_rad);
  const auto states = linkage.to_joints(motor);
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

TEST(FourBarLinkage, IsNaNWhereTheLoopCannotClose)
{
  // The horn is the rocker: past its swing no output angle closes the loop.
  const auto l = leg();
  const auto & linkage = l.layout.linkages.front()->linkage;
  const auto c = linkage.from_input(1.5);
  EXPECT_TRUE(std::isnan(c.output_rad) && std::isnan(c.ratio));
  const auto states = linkage.to_joints(JointFeedback{.position_rad = 1.5, .fresh = true});
  EXPECT_TRUE(std::isnan(states.output.position_rad) && std::isnan(states.coupler.effort_nm));
}

TEST(FourBarLinkage, RejectsALinkageThatDoesNotCloseAtZero)
{
  auto p = parallelogram();
  p.coupler_length_m += 0.001;
  EXPECT_TRUE(std::holds_alternative<std::string>(FourBarLinkage::create(p, -1.0, 1.0)));
}

TEST(FourBarLinkage, RejectsLimitsThatReachAToggle)
{
  // The leg's toes-down toggle is near 1.4 rad.
  const auto l = leg();
  EXPECT_TRUE(
    std::holds_alternative<std::string>(FourBarLinkage::create(l.parameters, l.lower, 1.6)));
  EXPECT_TRUE(
    std::holds_alternative<FourBarLinkage>(
      FourBarLinkage::create(l.parameters, l.lower, l.upper)));
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
    EXPECT_TRUE(std::holds_alternative<std::string>(FourBarLinkage::create(p, -1.0, 1.0)));
  }
  EXPECT_TRUE(
    std::holds_alternative<std::string>(FourBarLinkage::create(parallelogram(), 0.1, 1.0)));
}

}  // namespace
