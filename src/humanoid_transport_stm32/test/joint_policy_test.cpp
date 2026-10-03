// Copyright 2026 UTRA-RoboSoccer
// The per-phase request rules and the reading of a motor's telemetry.

#include <gtest/gtest.h>

#include <chrono>
#include <vector>

#include "humanoid_transport_stm32/joint_policy.hpp"

namespace
{

using namespace humanoid::transport_stm32;  // NOLINT(build/namespaces)
using std::chrono::microseconds;
using std::chrono::milliseconds;
using wire::FaultCause;
using wire::Lifecycle;
using wire::ModeRequest;

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

wire::TeleMotor motor_in(Lifecycle state, FaultCause cause = FaultCause::kNone)
{
  wire::TeleMotor m{};
  m.state = static_cast<std::uint8_t>(state);
  m.cause = static_cast<std::uint8_t>(cause);
  m.fb_age_ms = 2;
  return m;
}

// A telemetry frame holding `chains`, each as (chain_id, motors).
using ChainSpec = std::pair<std::uint8_t, std::vector<wire::TeleMotor>>;
wire::RobotTelemetry telemetry_of(const std::vector<ChainSpec> & chains)
{
  wire::RobotTelemetry t{};
  t.header.n_chains = static_cast<std::uint8_t>(chains.size());
  for (std::size_t i = 0; i < chains.size(); ++i) {
    t.chains[i].chain_id = chains[i].first;
    t.chains[i].n_motors = static_cast<std::uint8_t>(chains[i].second.size());
    for (std::size_t m = 0; m < chains[i].second.size(); ++m) {
      t.chains[i].motors[m] = chains[i].second[m];
    }
  }
  return t;
}

WiringLayout layout_of(const std::vector<JointWiring> & joints)
{
  return std::get<WiringLayout>(make_layout(joints));
}

// ---------------------------------------------------------------------------
// plan_request
// ---------------------------------------------------------------------------

TEST(PlanRequest, ArmingHoldsWithFaultResetAndNoTuple)
{
  const auto r = plan_request(Phase::kArming, false, true);
  EXPECT_EQ(r.mode, ModeRequest::kHold);
  EXPECT_TRUE(r.fault_reset);
  EXPECT_FALSE(r.send_tuple);
}

TEST(PlanRequest, RunningSendsTheTupleAsMitAndNeverFaultResets)
{
  const auto r = plan_request(Phase::kRunning, false, true);
  EXPECT_EQ(r.mode, ModeRequest::kMit);
  EXPECT_TRUE(r.send_tuple);
  EXPECT_FALSE(r.fault_reset);  // a fault must stay latched until a human re-activates
}

TEST(PlanRequest, RunningNeverRequestsHoldSoAFaultedMotorIsNeverRearmed)
{
  for (const bool isolated : {false, true}) {
    for (const bool representable : {false, true}) {
      EXPECT_NE(plan_request(Phase::kRunning, isolated, representable).mode, ModeRequest::kHold);
    }
  }
}

TEST(PlanRequest, AnIsolatedMotorIsAskedForIdleWhateverTheTuple)
{
  EXPECT_EQ(plan_request(Phase::kRunning, true, true).mode, ModeRequest::kIdle);
  EXPECT_EQ(plan_request(Phase::kRunning, true, false).mode, ModeRequest::kIdle);
  EXPECT_FALSE(plan_request(Phase::kRunning, true, true).send_tuple);
}

TEST(PlanRequest, AnUnrepresentableTupleBecomesDampedWithConfiguredGains)
{
  const auto r = plan_request(Phase::kRunning, false, false);
  EXPECT_EQ(r.mode, ModeRequest::kDamped);
  EXPECT_FALSE(r.send_tuple);
  EXPECT_TRUE(r.use_config_gains);
}

TEST(PlanRequest, ReleasingIdlesEveryMotor)
{
  for (const bool isolated : {false, true}) {
    const auto r = plan_request(Phase::kReleasing, isolated, true);
    EXPECT_EQ(r.mode, ModeRequest::kIdle);
    EXPECT_FALSE(r.send_tuple);
  }
}

TEST(IsCommandable, OnlyArmedStatesThatTakeAnMitRequest)
{
  EXPECT_TRUE(is_commandable(Lifecycle::kHold));
  EXPECT_TRUE(is_commandable(Lifecycle::kMit));
  EXPECT_TRUE(is_commandable(Lifecycle::kDamped));
  for (const auto state : {Lifecycle::kBoot, Lifecycle::kDiscovering, Lifecycle::kIdle,
      Lifecycle::kFault, Lifecycle::kToZero})
  {
    EXPECT_FALSE(is_commandable(state));
  }
}

// ---------------------------------------------------------------------------
// read_joint
// ---------------------------------------------------------------------------

TEST(ReadJoint, ConvertsToJointUnitsWithSignAndOffset)
{
  wire::TeleMotor m = motor_in(Lifecycle::kMit);
  m.pos = 2000;   // 0.2 rad
  m.vel = 150;    // 1.5 rad/s
  m.tau = -250;   // -2.5 N*m
  m.temp_c = 37;

  const auto reading = read_joint(wiring(0, 0, -1, 0.5), m, milliseconds{15});
  EXPECT_NEAR(reading.feedback.position_rad, 0.3, 1e-9);  // -1 * (0.2 - 0.5)
  EXPECT_NEAR(reading.feedback.velocity_rad_s, -1.5, 1e-9);
  EXPECT_NEAR(reading.feedback.effort_nm, 2.5, 1e-9);
  EXPECT_FLOAT_EQ(reading.feedback.temperature_c, 37.0F);
  EXPECT_TRUE(reading.feedback.fresh);
  EXPECT_TRUE(reading.commandable);
}

TEST(ReadJoint, FreshnessFollowsTheDrivesReportAgeAgainstTheManifestLimit)
{
  wire::TeleMotor m = motor_in(Lifecycle::kMit);
  m.fb_age_ms = 15;
  EXPECT_TRUE(read_joint(wiring(0, 0), m, milliseconds{15}).feedback.fresh);
  m.fb_age_ms = 16;
  EXPECT_FALSE(read_joint(wiring(0, 0), m, milliseconds{15}).feedback.fresh);
  m.fb_age_ms = 255;  // "never heard from this drive"
  EXPECT_FALSE(read_joint(wiring(0, 0), m, milliseconds{15}).feedback.fresh);
}

TEST(ReadJoint, AnAbsentMotorIsNeitherFreshNorCommandable)
{
  const auto reading = read_joint(wiring(0, 0), std::nullopt, milliseconds{15});
  EXPECT_FALSE(reading.feedback.fresh);
  EXPECT_FALSE(reading.commandable);
  EXPECT_EQ(reading.feedback.fault_bits, 0);
}

TEST(ReadJoint, PacksDriveFaultsFirmwareFaultAndCauseIntoFaultBits)
{
  wire::TeleMotor m = motor_in(Lifecycle::kFault, FaultCause::kOvertorque);
  m.motor_fault = 0x04;  // overheat
  const auto bits = read_joint(wiring(0, 0), m, milliseconds{15}).feedback.fault_bits;

  EXPECT_EQ(bits & kFaultBitsDriveMask, 0x04);
  EXPECT_NE(bits & kFaultBitFirmwareFault, 0);
  EXPECT_EQ(bits >> kFaultCauseShift, static_cast<unsigned>(FaultCause::kOvertorque));
}

TEST(ReadJoint, AHealthyMotorHasNoFaultBits)
{
  EXPECT_EQ(read_joint(wiring(0, 0), motor_in(Lifecycle::kMit), milliseconds{15})
    .feedback.fault_bits, 0);
}

TEST(ReadJoint, AFaultedMotorIsFreshButNotCommandable)
{
  const auto reading =
    read_joint(wiring(0, 0), motor_in(Lifecycle::kFault, FaultCause::kWatchdog), milliseconds{15});
  EXPECT_TRUE(reading.feedback.fresh);
  EXPECT_FALSE(reading.commandable);
}

// ---------------------------------------------------------------------------
// find_motor
// ---------------------------------------------------------------------------

TEST(FindMotor, MatchesAChainByIdNotByPositionInTheFrame)
{
  // The master omits a dead slave's chain, so chain 1 can be the first chain in the frame.
  const auto t = telemetry_of({{1, {motor_in(Lifecycle::kIdle), motor_in(Lifecycle::kHold)}}});
  EXPECT_FALSE(find_motor(t, 0, 0).has_value());
  const auto m = find_motor(t, 1, 1);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->state, static_cast<std::uint8_t>(Lifecycle::kHold));
}

TEST(FindMotor, ReportsAMotorBeyondTheChainsReportedCountAsAbsent)
{
  const auto t = telemetry_of({{0, {motor_in(Lifecycle::kIdle)}}});
  EXPECT_TRUE(find_motor(t, 0, 0).has_value());
  EXPECT_FALSE(find_motor(t, 0, 1).has_value());
}

// ---------------------------------------------------------------------------
// judge_arming
// ---------------------------------------------------------------------------

constexpr auto kGrace = std::chrono::milliseconds{500};

TEST(JudgeArming, ArmedOnceEveryJointHolds)
{
  const auto layout = layout_of({wiring(0, 0), wiring(0, 1), wiring(1, 0)});
  const auto t = telemetry_of({
      {0, {motor_in(Lifecycle::kHold), motor_in(Lifecycle::kHold)}},
      {1, {motor_in(Lifecycle::kHold)}}});
  EXPECT_EQ(judge_arming(t, layout, milliseconds{100}, kGrace).status,
    ArmingVerdict::Status::kArmed);
}

TEST(JudgeArming, WaitsAndNamesTheJointThatIsStillIdle)
{
  const auto layout = layout_of({wiring(0, 0), wiring(0, 1)});
  const auto t = telemetry_of({{0, {motor_in(Lifecycle::kHold), motor_in(Lifecycle::kIdle)}}});
  const auto verdict = judge_arming(t, layout, milliseconds{100}, kGrace);
  EXPECT_EQ(verdict.status, ArmingVerdict::Status::kWaiting);
  EXPECT_EQ(verdict.joint, 1);
}

TEST(JudgeArming, AMotorAlreadyInMitIsNotYetHolding)
{
  // Arming is complete only when the firmware has applied OUR hold request, which captures the
  // position. A motor left armed by an earlier session has not done that.
  const auto layout = layout_of({wiring(0, 0)});
  const auto t = telemetry_of({{0, {motor_in(Lifecycle::kMit)}}});
  EXPECT_EQ(judge_arming(t, layout, milliseconds{100}, kGrace).status,
    ArmingVerdict::Status::kWaiting);
}

TEST(JudgeArming, WaitsForAChainThatIsNotReportingYet)
{
  const auto layout = layout_of({wiring(0, 0), wiring(1, 0)});
  const auto t = telemetry_of({{0, {motor_in(Lifecycle::kHold)}}});
  const auto verdict = judge_arming(t, layout, milliseconds{100}, kGrace);
  EXPECT_EQ(verdict.status, ArmingVerdict::Status::kWaiting);
  EXPECT_EQ(verdict.joint, 1);
}

TEST(JudgeArming, ATransientFaultGetsAGraceForTheResetToLand)
{
  const auto layout = layout_of({wiring(0, 0)});
  const auto t = telemetry_of({{0, {motor_in(Lifecycle::kFault, FaultCause::kOvertorque)}}});
  EXPECT_EQ(judge_arming(t, layout, milliseconds{100}, kGrace).status,
    ArmingVerdict::Status::kWaiting);
}

TEST(JudgeArming, AFaultThatSurvivesTheGraceFailsArming)
{
  const auto layout = layout_of({wiring(0, 0), wiring(0, 1)});
  const auto t = telemetry_of({
      {0, {motor_in(Lifecycle::kHold), motor_in(Lifecycle::kFault, FaultCause::kMotorFault)}}});
  const auto verdict = judge_arming(t, layout, milliseconds{600}, kGrace);
  EXPECT_EQ(verdict.status, ArmingVerdict::Status::kFailed);
  EXPECT_EQ(verdict.joint, 1);
}

TEST(JudgeArming, AWoundShaftFailsAtOnceBecauseNoRetryFixesIt)
{
  const auto layout = layout_of({wiring(0, 0)});
  const auto t = telemetry_of({{0, {motor_in(Lifecycle::kFault, FaultCause::kWound)}}});
  EXPECT_EQ(judge_arming(t, layout, milliseconds{0}, kGrace).status,
    ArmingVerdict::Status::kFailed);
}

}  // namespace
