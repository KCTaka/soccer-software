// Copyright 2026 UTRA-RoboSoccer
// Joint wiring: the angle convention, layout validation, and the envelope-versus-wire check.

#include <gtest/gtest.h>

#include <numbers>

#include <cmath>
#include <limits>
#include <string>
#include <variant>
#include <vector>

#include "humanoid_transport_stm32/joint_wiring.hpp"

namespace
{

using humanoid::transport::JointCommand;
using humanoid::transport::JointEnvelope;
using namespace humanoid::transport_stm32;  // NOLINT(build/namespaces)

JointWiring wiring(
  std::uint8_t chain, std::uint8_t motor, std::int8_t sign = 1, double offset = 0.0)
{
  JointWiring w;
  w.chain = chain;
  w.motor = motor;
  w.direction_sign = sign;
  w.zero_offset_rad = offset;
  return w;
}

// An envelope well inside every wire range.
JointEnvelope modest_envelope()
{
  JointEnvelope e;
  e.position_min_rad = -1.0;
  e.position_max_rad = 1.5;
  e.velocity_max_rad_s = 20.0;
  e.torque_peak_nm = 14.0;
  e.stiffness_max_nm_rad = 500.0;
  e.damping_max_nm_s_rad = 10.0;
  return e;
}

std::string error_of(const LayoutOrError & result)
{
  return std::holds_alternative<std::string>(result) ? std::get<std::string>(result) : "";
}

// ---------------------------------------------------------------------------
// Angle convention
// ---------------------------------------------------------------------------

TEST(JointWiringConvention, JointAngleIsSignTimesWireAngleMinusOffset)
{
  EXPECT_DOUBLE_EQ(joint_position(wiring(0, 0, 1, 0.5), 0.8), 0.3);
  EXPECT_DOUBLE_EQ(joint_position(wiring(0, 0, -1, 0.5), 0.2), 0.3);
}

TEST(JointWiringConvention, WireAngleInvertsJointAngle)
{
  for (const std::int8_t sign : {static_cast<std::int8_t>(1), static_cast<std::int8_t>(-1)}) {
    for (const double offset : {0.0, 0.5, -1.25}) {
      for (const double q : {-2.0, -0.1, 0.0, 0.7, 3.0}) {
        const auto w = wiring(0, 0, sign, offset);
        EXPECT_NEAR(wire_position(w, joint_position(w, q)), q, 1e-12);
        EXPECT_NEAR(joint_position(w, wire_position(w, q)), q, 1e-12);
      }
    }
  }
}

TEST(JointWiringConvention, SignedValueFlipsOnlyForAMirroredJoint)
{
  EXPECT_DOUBLE_EQ(signed_value(wiring(0, 0, 1), 2.5), 2.5);
  EXPECT_DOUBLE_EQ(signed_value(wiring(0, 0, -1), 2.5), -2.5);
  EXPECT_DOUBLE_EQ(signed_value(wiring(0, 0, -1), signed_value(wiring(0, 0, -1), 2.5)), 2.5);
}

TEST(JointWiringConvention, SetpointNegatesMotionAndTorqueButNeverTheGains)
{
  JointCommand command;
  command.position_rad = 0.3;
  command.velocity_rad_s = 2.0;
  command.effort_nm = 1.5;
  command.stiffness_nm_rad = 40.0;
  command.damping_nm_s_rad = 2.0;

  const auto setpoint = to_setpoint(wiring(0, 0, -1, 0.5), command);
  EXPECT_FLOAT_EQ(setpoint.position_rad, 0.2F);  // -1 * 0.3 + 0.5
  EXPECT_FLOAT_EQ(setpoint.velocity_rad_s, -2.0F);
  EXPECT_FLOAT_EQ(setpoint.effort_nm, -1.5F);
  EXPECT_FLOAT_EQ(setpoint.stiffness_nm_rad, 40.0F);  // a mirrored joint keeps a positive gain
  EXPECT_FLOAT_EQ(setpoint.damping_nm_s_rad, 2.0F);
}

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------

TEST(JointWiringLayout, DerivesTheCommandSlotsPerChainInAscendingChainOrder)
{
  const std::vector<JointWiring> joints = {
    wiring(2, 3), wiring(0, 0), wiring(0, 1), wiring(2, 0)};
  const auto result = make_layout(joints);
  ASSERT_TRUE(std::holds_alternative<WiringLayout>(result)) << error_of(result);
  const auto & layout = std::get<WiringLayout>(result);

  EXPECT_EQ(layout.joint_count, 4);
  EXPECT_EQ(layout.joints[0].chain, 2);  // manifest order is preserved
  ASSERT_EQ(layout.chain_count, 2);
  EXPECT_EQ(layout.chains[0].chain_id, 0);
  EXPECT_EQ(layout.chains[0].motor_slots, 2);
  EXPECT_EQ(layout.chains[1].chain_id, 2);
  EXPECT_EQ(layout.chains[1].motor_slots, 4);  // highest mapped motor (3) + 1
}

TEST(JointWiringLayout, AcceptsTheWholeWire)
{
  std::vector<JointWiring> joints;
  for (std::uint8_t c = 0; c < wire::kMaxChains; ++c) {
    for (std::uint8_t m = 0; m < wire::kMaxMotorsPerChain; ++m) {
      joints.push_back(wiring(c, m));
    }
  }
  const auto result = make_layout(joints);
  ASSERT_TRUE(std::holds_alternative<WiringLayout>(result)) << error_of(result);
  EXPECT_EQ(std::get<WiringLayout>(result).joint_count, wire::kMaxMotors);
}

TEST(JointWiringLayout, RejectsWhatTheWireCannotAddress)
{
  EXPECT_FALSE(error_of(make_layout({})).empty());
  EXPECT_FALSE(error_of(make_layout(std::vector<JointWiring>{wiring(wire::kMaxChains,
        0)})).empty());
  EXPECT_FALSE(
    error_of(make_layout(std::vector<JointWiring>{wiring(0, wire::kMaxMotorsPerChain)})).empty());

  std::vector<JointWiring> too_many(wire::kMaxMotors + 1, wiring(0, 0));
  EXPECT_FALSE(error_of(make_layout(too_many)).empty());
}

TEST(JointWiringLayout, RejectsTwoJointsOnOneMotor)
{
  const auto result = make_layout(std::vector<JointWiring>{wiring(1, 2), wiring(1, 2)});
  EXPECT_NE(error_of(result).find("already wired"), std::string::npos);
}

TEST(JointWiringLayout, RejectsASignThatIsNotPlusOrMinusOne)
{
  // A sign of 0 would zero every joint; 2 would double it. Both must stop at configure.
  EXPECT_FALSE(error_of(make_layout(std::vector<JointWiring>{wiring(0, 0, 0)})).empty());
  EXPECT_FALSE(error_of(make_layout(std::vector<JointWiring>{wiring(0, 0, 2)})).empty());
}

TEST(JointWiringLayout, NamesTheOffendingJointWhenNamesAreGiven)
{
  const std::vector<JointWiring> joints = {wiring(0, 0), wiring(0, 0)};
  const std::vector<std::string> names = {"left_knee", "right_knee"};
  const auto error = error_of(make_layout(joints, names));
  EXPECT_NE(error.find("'right_knee'"), std::string::npos);

  // Without names it falls back to the position.
  EXPECT_NE(error_of(make_layout(joints)).find("joint 1"), std::string::npos);
}

TEST(JointWiringLayout, RejectsANonFiniteOffset)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(error_of(make_layout(std::vector<JointWiring>{wiring(0, 0, 1, nan)})).empty());
}

// ---------------------------------------------------------------------------
// Envelope versus wire
// ---------------------------------------------------------------------------

TEST(JointWiringEnvelope, AcceptsAnEnvelopeInsideEveryWireRange)
{
  EXPECT_FALSE(check_envelope(wiring(0, 0), modest_envelope()).has_value());
}

TEST(JointWiringEnvelope, AcceptsTheGeneratedModelsLargestLimits)
{
  // The shipped model's hip envelope: position -2.51..2.86 rad, 88 N*m, Kp 500, Kd 10.
  JointEnvelope e;
  e.position_min_rad = -2.5107;
  e.position_max_rad = 2.8598;
  e.velocity_max_rad_s = 20.0;
  e.torque_peak_nm = 88.0;
  e.stiffness_max_nm_rad = 500.0;
  e.damping_max_nm_s_rad = 10.0;
  EXPECT_FALSE(check_envelope(wiring(0, 0), e).has_value());
}

TEST(JointWiringEnvelope, RejectsAPositionRangeBeyondTheHomeFrame)
{
  JointEnvelope e = modest_envelope();
  e.position_max_rad = std::numbers::pi + 0.01;
  const auto error = check_envelope(wiring(0, 0), e);
  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("home-frame"), std::string::npos);
}

TEST(JointWiringEnvelope, ChecksThePositionRangeAfterMappingIntoTheWireFrame)
{
  // Inside +-pi in joint terms, outside it once a 2 rad zero offset moves it on the wire.
  JointEnvelope e = modest_envelope();
  e.position_max_rad = 1.5;
  EXPECT_FALSE(check_envelope(wiring(0, 0, 1, 0.0), e).has_value());
  EXPECT_TRUE(check_envelope(wiring(0, 0, 1, 2.0), e).has_value());

  // A mirrored joint flips the range rather than shrinking it.
  e.position_min_rad = -3.0;
  e.position_max_rad = 0.1;
  EXPECT_FALSE(check_envelope(wiring(0, 0, -1, 0.0), e).has_value());
}

TEST(JointWiringEnvelope, RejectsEachFieldThatExceedsItsWireRange)
{
  JointEnvelope e = modest_envelope();
  e.velocity_max_rad_s = 400.0;
  EXPECT_NE(check_envelope(wiring(0, 0), e)->find("velocity_max_rad_s"), std::string::npos);

  e = modest_envelope();
  e.torque_peak_nm = 400.0;
  EXPECT_NE(check_envelope(wiring(0, 0), e)->find("torque_peak_nm"), std::string::npos);

  e = modest_envelope();
  e.stiffness_max_nm_rad = 7000.0;
  EXPECT_NE(check_envelope(wiring(0, 0), e)->find("stiffness_max_nm_rad"), std::string::npos);

  e = modest_envelope();
  e.damping_max_nm_s_rad = 700.0;
  EXPECT_NE(check_envelope(wiring(0, 0), e)->find("damping_max_nm_s_rad"), std::string::npos);
}

TEST(JointWiringEnvelope, RejectsANonFiniteLimit)
{
  JointEnvelope e = modest_envelope();
  e.torque_peak_nm = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(check_envelope(wiring(0, 0), e).has_value());
}

}  // namespace
