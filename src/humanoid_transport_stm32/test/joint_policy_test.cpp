// Copyright 2026 UTRA-RoboSoccer
// The per-phase request rules (protocol v10) and the reading of a motor's telemetry.

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

#include "humanoid_transport_stm32/joint_policy.hpp"

namespace
{

using namespace humanoid::transport_stm32;  // NOLINT(build/namespaces)
using std::chrono::microseconds;
using std::chrono::milliseconds;
using wire::DriveFault;
using wire::DriveReq;
using wire::DriveState;

wire::TeleMotor motor_in(DriveState state, DriveFault fault = DriveFault::DRVF_NONE)
{
  wire::TeleMotor m{};
  m.drive_state = static_cast<std::uint8_t>(state);
  m.drive_fault = static_cast<std::uint8_t>(fault);
  m.fb_age_ms = 2;
  return m;
}

// A telemetry frame holding `chains`, each as (chain_id, motors). The v10 tele_robot_t is flat.
using ChainSpec = std::pair<std::uint8_t, std::vector<wire::TeleMotor>>;
wire::RobotTelemetry telemetry_of(const std::vector<ChainSpec> & chains)
{
  wire::RobotTelemetry t{};
  t.n_chains = static_cast<std::uint8_t>(chains.size());
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
// plan_request: the drive request and, independently, whether a VALID tuple rides along (v10)
// ---------------------------------------------------------------------------

/// One row of plan_request's table, as joint_policy.hpp documents it. Isolation is a running-phase
/// concept: every other phase is planned for a motor that is not isolated.
struct PlanCase
{
  const char * name;
  Phase phase;
  bool isolated;
  DriveReq request;
  bool send_tuple;
  bool use_config_gains;
};

class PlanRequest : public ::testing::TestWithParam<PlanCase> {};

TEST_P(PlanRequest, FollowsThePhaseTable)
{
  const auto & c = GetParam();
  // Representability never changes the request: the caller fails a cycle whose tuple the wire
  // cannot carry, and there is no DAMPED request any more.
  for (const bool representable : {true, false}) {
    SCOPED_TRACE(representable ? "representable tuple" : "unrepresentable tuple");
    const auto r = plan_request(c.phase, c.isolated, representable);
    EXPECT_EQ(r.request, c.request);
    EXPECT_EQ(r.send_tuple, c.send_tuple);
    EXPECT_EQ(r.use_config_gains, c.use_config_gains);
  }
}

INSTANTIATE_TEST_SUITE_P(
  EveryPhase, PlanRequest, ::testing::Values(
    // RESET clears what the previous session left latched.
    PlanCase{"Clearing", Phase::kClearing, false, DriveReq::DRV_REQ_RESET, false, false},
    // ENABLE streaming the captured hold tuple, so the motor holds instead of coming up limp, with
    // the slave's own gains.
    PlanCase{"Arming", Phase::kArming, false, DriveReq::DRV_REQ_ENABLE, true, true},
    // ENABLE and the live tuple every cycle, never RESET, so a faulted motor is never rearmed.
    PlanCase{"Running", Phase::kRunning, false, DriveReq::DRV_REQ_ENABLE, true, false},
    PlanCase{"RunningIsolated", Phase::kRunning, true, DriveReq::DRV_REQ_DISABLE, false, false},
    PlanCase{"Stopping", Phase::kStopping, false, DriveReq::DRV_REQ_STOP, false, false},
    PlanCase{"Releasing", Phase::kReleasing, false, DriveReq::DRV_REQ_DISABLE, false, false}),
  [](const auto & row) {return std::string{row.param.name};});

TEST(IsCommandable, OnlyEnabled)
{
  EXPECT_TRUE(is_commandable(DriveState::DRV_ENABLED));
  for (const auto state : {DriveState::DRV_BOOT, DriveState::DRV_DISCOVERING,
      DriveState::DRV_ABSENT, DriveState::DRV_DISABLED, DriveState::DRV_ENABLING,
      DriveState::DRV_STOPPING, DriveState::DRV_CLEARING, DriveState::DRV_FAULT})
  {
    EXPECT_FALSE(is_commandable(state));
  }
}

// ---------------------------------------------------------------------------
// read_joint
// ---------------------------------------------------------------------------

TEST(ReadJoint, ConvertsToJointUnitsWithSignAndOffset)
{
  wire::TeleMotor m = motor_in(DriveState::DRV_ENABLED);
  m.pos = 2000;   // 0.2 rad
  m.vel = 150;    // 1.5 rad/s
  m.tau = -250;   // -2.5 N*m
  m.temp_c = 37;

  const JointWiring mirrored{.direction_sign = -1, .zero_offset_rad = 0.5};
  const auto reading = read_joint(mirrored, m, milliseconds{15});
  EXPECT_NEAR(reading.feedback.position_rad, 0.3, 1e-9);  // -1 * (0.2 - 0.5)
  EXPECT_NEAR(reading.feedback.velocity_rad_s, -1.5, 1e-9);
  EXPECT_NEAR(reading.feedback.effort_nm, 2.5, 1e-9);
  EXPECT_FLOAT_EQ(reading.feedback.temperature_c, 37.0F);
  EXPECT_TRUE(reading.feedback.fresh);
  EXPECT_TRUE(reading.commandable);
}

TEST(ReadJoint, FreshnessFollowsTheDrivesReportAgeAgainstTheManifestLimit)
{
  wire::TeleMotor m = motor_in(DriveState::DRV_ENABLED);
  m.fb_age_ms = 15;
  EXPECT_TRUE(read_joint(JointWiring{}, m, milliseconds{15}).feedback.fresh);
  m.fb_age_ms = 16;
  EXPECT_FALSE(read_joint(JointWiring{}, m, milliseconds{15}).feedback.fresh);
  m.fb_age_ms = 255;  // "never heard from this drive"
  EXPECT_FALSE(read_joint(JointWiring{}, m, milliseconds{15}).feedback.fresh);
}

TEST(ReadJoint, AnAbsentMotorIsNeitherFreshNorCommandable)
{
  const auto reading = read_joint(JointWiring{}, std::nullopt, milliseconds{15});
  EXPECT_FALSE(reading.feedback.fresh);
  EXPECT_FALSE(reading.commandable);
  EXPECT_EQ(reading.feedback.fault_bits, 0);
}

TEST(ReadJoint, PacksDriveBitsTheFirmwareFaultFlagAndTheLatchedFaultIntoFaultBits)
{
  wire::TeleMotor m = motor_in(DriveState::DRV_FAULT, DriveFault::DRVF_OVERTORQUE);
  m.rs_fault_bits = 0x04;  // the drive's own bits
  const auto bits = read_joint(JointWiring{}, m, milliseconds{15}).feedback.fault_bits;

  EXPECT_EQ(bits & kFaultBitsDriveMask, 0x04);
  EXPECT_NE(bits & kFaultBitFirmwareFault, 0);
  EXPECT_EQ(bits >> kFaultCauseShift, static_cast<unsigned>(DriveFault::DRVF_OVERTORQUE));
}

TEST(ReadJoint, AHealthyMotorHasNoFaultBits)
{
  EXPECT_EQ(read_joint(JointWiring{}, motor_in(DriveState::DRV_ENABLED), milliseconds{15})
    .feedback.fault_bits, 0);
}

TEST(ReadJoint, AFaultedMotorIsFreshButNotCommandable)
{
  const auto reading = read_joint(
    JointWiring{}, motor_in(DriveState::DRV_FAULT, DriveFault::DRVF_CAN_TIMEOUT), milliseconds{15});
  EXPECT_TRUE(reading.feedback.fresh);
  EXPECT_FALSE(reading.commandable);
}

// ---------------------------------------------------------------------------
// find_motor
// ---------------------------------------------------------------------------

TEST(FindMotor, MatchesAChainByIdNotByPositionInTheFrame)
{
  // The master omits a dead slave's chain, so chain 1 can be the first chain in the frame.
  const auto t = telemetry_of({{1, {motor_in(DriveState::DRV_DISABLED),
          motor_in(DriveState::DRV_ENABLED)}}});
  EXPECT_FALSE(find_motor(t, 0, 0).has_value());
  const auto m = find_motor(t, 1, 1);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->drive_state, static_cast<std::uint8_t>(DriveState::DRV_ENABLED));
}

TEST(FindMotor, ReportsAMotorBeyondTheChainsReportedCountAsAbsent)
{
  const auto t = telemetry_of({{0, {motor_in(DriveState::DRV_DISABLED)}}});
  EXPECT_TRUE(find_motor(t, 0, 0).has_value());
  EXPECT_FALSE(find_motor(t, 0, 1).has_value());
}

TEST(ChainMotorCount, SeparatesAnAbsentChainFromOneReportingFewerMotors)
{
  const auto t = telemetry_of({{1, {motor_in(DriveState::DRV_DISABLED),
          motor_in(DriveState::DRV_DISABLED)}}});
  EXPECT_EQ(chain_motor_count(t, 1), 2);
  EXPECT_FALSE(chain_motor_count(t, 0).has_value());
}

// ---------------------------------------------------------------------------
// judge_clearing: every joint DISABLED and fresh
// ---------------------------------------------------------------------------

constexpr auto kGrace = std::chrono::milliseconds{500};
constexpr auto kMaxAge = std::chrono::microseconds{15000};

wire::TeleMotor aged(wire::TeleMotor motor, std::uint8_t age_ms)
{
  motor.fb_age_ms = age_ms;
  return motor;
}

TEST(JudgeClearing, ReachedOnceEveryJointIsDisabledAndFresh)
{
  const auto layout = layout_of(
    {{.chain = 0, .motor = 0}, {.chain = 0, .motor = 1}, {.chain = 1, .motor = 0}});
  const auto t = telemetry_of({
      {0, {motor_in(DriveState::DRV_DISABLED), motor_in(DriveState::DRV_DISABLED)}},
      {1, {motor_in(DriveState::DRV_DISABLED)}}});
  EXPECT_EQ(judge_clearing(t, layout, kMaxAge, milliseconds{100}, kGrace).status,
    ArmingVerdict::Status::kReached);
}

TEST(JudgeClearing, ADisabledMotorWithStaleFeedbackIsNotYetCleared)
{
  // Its reported position is not current yet.
  const auto layout = layout_of({{.chain = 0, .motor = 0}, {.chain = 0, .motor = 1}});
  const auto t = telemetry_of({
      {0, {motor_in(DriveState::DRV_DISABLED), aged(motor_in(DriveState::DRV_DISABLED), 40)}}});
  const auto verdict = judge_clearing(t, layout, kMaxAge, milliseconds{100}, kGrace);
  EXPECT_EQ(verdict.status, ArmingVerdict::Status::kWaiting);
  EXPECT_EQ(verdict.joint, 1);
}

TEST(JudgeClearing, AMotorThatNeverAnsweredIsNotCleared)
{
  const auto layout = layout_of({{.chain = 0, .motor = 0}});
  const auto t = telemetry_of({{0, {aged(motor_in(DriveState::DRV_DISABLED), 255)}}});
  EXPECT_EQ(judge_clearing(t, layout, kMaxAge, milliseconds{100}, kGrace).status,
    ArmingVerdict::Status::kWaiting);
}

TEST(JudgeClearing, AnArmedMotorIsNotYetCleared)
{
  const auto layout = layout_of({{.chain = 0, .motor = 0}});
  const auto t = telemetry_of({{0, {motor_in(DriveState::DRV_ENABLED)}}});
  EXPECT_EQ(judge_clearing(t, layout, kMaxAge, milliseconds{100}, kGrace).status,
    ArmingVerdict::Status::kWaiting);
}

TEST(JudgeClearing, WaitsForAChainThatIsNotReportingYet)
{
  const auto layout = layout_of({{.chain = 0, .motor = 0}, {.chain = 1, .motor = 0}});
  const auto t = telemetry_of({{0, {motor_in(DriveState::DRV_DISABLED)}}});
  const auto verdict = judge_clearing(t, layout, kMaxAge, milliseconds{100}, kGrace);
  EXPECT_EQ(verdict.status, ArmingVerdict::Status::kWaiting);
  EXPECT_EQ(verdict.joint, 1);
}

TEST(JudgeClearing, AFaultGetsAGraceForTheResetToLand)
{
  const auto layout = layout_of({{.chain = 0, .motor = 0}});
  const auto t = telemetry_of({{0,
        {motor_in(DriveState::DRV_FAULT, DriveFault::DRVF_OVERTORQUE)}}});
  EXPECT_EQ(judge_clearing(t, layout, kMaxAge, milliseconds{100}, kGrace).status,
    ArmingVerdict::Status::kWaiting);
}

TEST(JudgeClearing, AFaultThatSurvivesTheGraceFails)
{
  const auto layout = layout_of({{.chain = 0, .motor = 0}, {.chain = 0, .motor = 1}});
  const auto t = telemetry_of({
      {0, {motor_in(DriveState::DRV_DISABLED),
          motor_in(DriveState::DRV_FAULT, DriveFault::DRVF_MOTOR_REPORTED)}}});
  const auto verdict = judge_clearing(t, layout, kMaxAge, milliseconds{600}, kGrace);
  EXPECT_EQ(verdict.status, ArmingVerdict::Status::kFailed);
  EXPECT_EQ(verdict.joint, 1);
}

// ---------------------------------------------------------------------------
// judge_arming: every joint ENABLED and fresh
// ---------------------------------------------------------------------------

TEST(JudgeArming, ReachedOnceEveryJointIsEnabledWithFreshFeedback)
{
  const auto layout = layout_of(
    {{.chain = 0, .motor = 0}, {.chain = 0, .motor = 1}, {.chain = 1, .motor = 0}});
  const auto t = telemetry_of({
      {0, {motor_in(DriveState::DRV_ENABLED), motor_in(DriveState::DRV_ENABLED)}},
      {1, {motor_in(DriveState::DRV_ENABLED)}}});
  EXPECT_EQ(judge_arming(t, layout, kMaxAge).status, ArmingVerdict::Status::kReached);
}

TEST(JudgeArming, EnabledWithoutFreshFeedbackIsNotArmed)
{
  // A motor with no drive behind it can report ENABLED briefly before the CAN timeout faults it.
  const auto layout = layout_of({{.chain = 0, .motor = 0}, {.chain = 0, .motor = 1}});
  const auto t = telemetry_of({
      {0, {motor_in(DriveState::DRV_ENABLED), aged(motor_in(DriveState::DRV_ENABLED), 255)}}});
  const auto verdict = judge_arming(t, layout, kMaxAge);
  EXPECT_EQ(verdict.status, ArmingVerdict::Status::kWaiting);
  EXPECT_EQ(verdict.joint, 1);
}

TEST(JudgeArming, WaitsAndNamesTheJointThatIsStillDisabled)
{
  const auto layout = layout_of({{.chain = 0, .motor = 0}, {.chain = 0, .motor = 1}});
  const auto t = telemetry_of({
      {0, {motor_in(DriveState::DRV_ENABLED), motor_in(DriveState::DRV_DISABLED)}}});
  const auto verdict = judge_arming(t, layout, kMaxAge);
  EXPECT_EQ(verdict.status, ArmingVerdict::Status::kWaiting);
  EXPECT_EQ(verdict.joint, 1);
}

TEST(JudgeArming, AMotorStillEnablingIsNotYetArmed)
{
  const auto layout = layout_of({{.chain = 0, .motor = 0}});
  const auto t = telemetry_of({{0, {motor_in(DriveState::DRV_ENABLING)}}});
  EXPECT_EQ(judge_arming(t, layout, kMaxAge).status, ArmingVerdict::Status::kWaiting);
}

TEST(JudgeArming, WaitsForAChainThatIsNotReportingYet)
{
  const auto layout = layout_of({{.chain = 0, .motor = 0}, {.chain = 1, .motor = 0}});
  const auto t = telemetry_of({{0, {motor_in(DriveState::DRV_ENABLED)}}});
  const auto verdict = judge_arming(t, layout, kMaxAge);
  EXPECT_EQ(verdict.status, ArmingVerdict::Status::kWaiting);
  EXPECT_EQ(verdict.joint, 1);
}

TEST(JudgeArming, AnyFaultFailsAtOnceBecauseFromDisabledItIsThisArmingsOwn)
{
  const auto layout = layout_of({{.chain = 0, .motor = 0}, {.chain = 0, .motor = 1}});
  for (const DriveFault fault : {DriveFault::DRVF_WOUND, DriveFault::DRVF_CAN_TIMEOUT}) {
    const auto t = telemetry_of({
        {0, {motor_in(DriveState::DRV_ENABLED), motor_in(DriveState::DRV_FAULT, fault)}}});
    const auto verdict = judge_arming(t, layout, kMaxAge);
    EXPECT_EQ(verdict.status, ArmingVerdict::Status::kFailed);
    EXPECT_EQ(verdict.joint, 1);
  }
}

}  // namespace
