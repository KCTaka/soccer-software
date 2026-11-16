// Copyright 2026 UTRA-RoboSoccer
// Joint wiring: the angle convention, layout validation, and the envelope-versus-wire check.

#include <gmock/gmock.h>

#include <numbers>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "humanoid_test_support/matchers.hpp"
#include "humanoid_transport_stm32/joint_wiring.hpp"

namespace
{

using humanoid::test_support::HoldsError;
using humanoid::test_support::HoldsValue;
using humanoid::transport::JointCommand;
using humanoid::transport::JointEnvelope;
using namespace humanoid::transport_stm32;  // NOLINT(build/namespaces)
using ::testing::_;
using ::testing::HasSubstr;
using ::testing::Optional;

/// Mirrored and offset, so a missed sign or offset shows.
constexpr JointWiring kMirrored{.direction_sign = -1, .zero_offset_rad = 0.5};

/// make_layout() of a braced list, which a std::span cannot be built from.
LayoutOrError layout(const std::vector<JointWiring> & joints) {return make_layout(joints);}

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

// ---------------------------------------------------------------------------
// Angle convention
// ---------------------------------------------------------------------------

TEST(JointWiringConvention, JointAngleIsSignTimesWireAngleMinusOffset)
{
  EXPECT_DOUBLE_EQ(joint_position(JointWiring{.zero_offset_rad = 0.5}, 0.8), 0.3);
  EXPECT_DOUBLE_EQ(joint_position(kMirrored, 0.2), 0.3);
}

TEST(JointWiringConvention, WireAngleInvertsJointAngle)
{
  for (const std::int8_t sign : {std::int8_t{1}, std::int8_t{-1}}) {
    for (const double offset : {0.0, 0.5, -1.25}) {
      const JointWiring w{.direction_sign = sign, .zero_offset_rad = offset};
      for (const double q : {-2.0, -0.1, 0.0, 0.7, 3.0}) {
        SCOPED_TRACE(
          "sign " + std::to_string(sign) + ", offset " + std::to_string(offset) + ", q " +
          std::to_string(q));
        EXPECT_NEAR(wire_position(w, joint_position(w, q)), q, 1e-12);
        EXPECT_NEAR(joint_position(w, wire_position(w, q)), q, 1e-12);
      }
    }
  }
}

TEST(JointWiringConvention, SignedValueFlipsOnlyForAMirroredJoint)
{
  EXPECT_DOUBLE_EQ(signed_value(JointWiring{}, 2.5), 2.5);
  EXPECT_DOUBLE_EQ(signed_value(kMirrored, 2.5), -2.5);
  EXPECT_DOUBLE_EQ(signed_value(kMirrored, signed_value(kMirrored, 2.5)), 2.5);
}

TEST(JointWiringConvention, SetpointNegatesMotionAndTorqueButNeverTheGains)
{
  const JointCommand command{
    .position_rad = 0.3, .velocity_rad_s = 2.0, .effort_nm = 1.5, .stiffness_nm_rad = 40.0,
    .damping_nm_s_rad = 2.0};

  const auto setpoint = to_setpoint(kMirrored, command);
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
    {.chain = 2, .motor = 3}, {.chain = 0, .motor = 0}, {.chain = 0, .motor = 1},
    {.chain = 2, .motor = 0}};
  const auto result = make_layout(joints);
  ASSERT_THAT(result, HoldsValue());
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
      joints.push_back({.chain = c, .motor = m});
    }
  }
  const auto result = make_layout(joints);
  ASSERT_THAT(result, HoldsValue());
  EXPECT_EQ(std::get<WiringLayout>(result).joint_count, wire::kMaxMotors);
}

TEST(JointWiringLayout, RejectsWhatTheWireCannotAddress)
{
  const auto beyond_chains = static_cast<std::uint8_t>(wire::kMaxChains);
  const auto beyond_motors = static_cast<std::uint8_t>(wire::kMaxMotorsPerChain);
  EXPECT_THAT(layout({}), HoldsError(_));
  EXPECT_THAT(layout({{.chain = beyond_chains}}), HoldsError(_));
  EXPECT_THAT(layout({{.motor = beyond_motors}}), HoldsError(_));
  EXPECT_THAT(
    layout(std::vector<JointWiring>(wire::kMaxMotors + 1)), HoldsError(_));
}

TEST(JointWiringLayout, RejectsTwoJointsOnOneMotor)
{
  EXPECT_THAT(
    layout({{.chain = 1, .motor = 2}, {.chain = 1, .motor = 2}}),
    HoldsError(HasSubstr("already wired")));
}

TEST(JointWiringLayout, RejectsASignThatIsNotPlusOrMinusOne)
{
  // A sign of 0 would zero every joint; 2 would double it. Both must stop at configure.
  for (const std::int8_t sign : {std::int8_t{0}, std::int8_t{2}}) {
    SCOPED_TRACE("sign " + std::to_string(sign));
    EXPECT_THAT(layout({{.direction_sign = sign}}), HoldsError(_));
  }
}

TEST(JointWiringLayout, NamesTheOffendingJointWhenNamesAreGiven)
{
  const std::vector<JointWiring> joints(2);  // both on chain 0, motor 0
  EXPECT_THAT(
    make_layout(joints, std::vector<std::string>{"left_knee", "right_knee"}),
    HoldsError(HasSubstr("'right_knee'")));
  // Without names it falls back to the position.
  EXPECT_THAT(make_layout(joints), HoldsError(HasSubstr("joint 1")));
}

TEST(JointWiringLayout, RejectsANonFiniteOffset)
{
  EXPECT_THAT(
    layout({{.zero_offset_rad = std::numeric_limits<double>::quiet_NaN()}}),
    HoldsError(_));
}

// ---------------------------------------------------------------------------
// Envelope versus wire
// ---------------------------------------------------------------------------

TEST(JointWiringEnvelope, AcceptsAnEnvelopeInsideEveryWireRange)
{
  EXPECT_EQ(check_envelope(JointWiring{}, modest_envelope()), std::nullopt);
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
  EXPECT_EQ(check_envelope(JointWiring{}, e), std::nullopt);
}

TEST(JointWiringEnvelope, RejectsAPositionRangeBeyondTheHomeFrame)
{
  JointEnvelope e = modest_envelope();
  e.position_max_rad = std::numbers::pi + 0.01;
  EXPECT_THAT(check_envelope(JointWiring{}, e), Optional(HasSubstr("home-frame")));
}

TEST(JointWiringEnvelope, ChecksThePositionRangeAfterMappingIntoTheWireFrame)
{
  // Inside +-pi in joint terms, outside it once a 2 rad zero offset moves it on the wire.
  JointEnvelope e = modest_envelope();
  e.position_max_rad = 1.5;
  EXPECT_EQ(check_envelope(JointWiring{}, e), std::nullopt);
  EXPECT_NE(check_envelope(JointWiring{.zero_offset_rad = 2.0}, e), std::nullopt);

  // A mirrored joint flips the range rather than shrinking it.
  e.position_min_rad = -3.0;
  e.position_max_rad = 0.1;
  EXPECT_EQ(check_envelope(JointWiring{.direction_sign = -1}, e), std::nullopt);
}

TEST(JointWiringEnvelope, RejectsEachFieldThatExceedsItsWireRange)
{
  struct Excess
  {
    const char * field;
    double JointEnvelope::* member;
    double value;
  };
  for (const auto & [field, member, value] : {
      Excess{"velocity_max_rad_s", &JointEnvelope::velocity_max_rad_s, 400.0},
      Excess{"torque_peak_nm", &JointEnvelope::torque_peak_nm, 400.0},
      Excess{"stiffness_max_nm_rad", &JointEnvelope::stiffness_max_nm_rad, 7000.0},
      Excess{"damping_max_nm_s_rad", &JointEnvelope::damping_max_nm_s_rad, 700.0}})
  {
    JointEnvelope e = modest_envelope();
    e.*member = value;
    EXPECT_THAT(check_envelope(JointWiring{}, e), Optional(HasSubstr(field)));
  }
}

TEST(JointWiringEnvelope, RejectsANonFiniteLimit)
{
  JointEnvelope e = modest_envelope();
  e.torque_peak_nm = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(check_envelope(JointWiring{}, e), std::nullopt);
}

}  // namespace
