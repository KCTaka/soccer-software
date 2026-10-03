// Copyright 2026 UTRA-RoboSoccer
// Stm32SerialTransport against a fake master: the ActuatorTransport conformance checks that the
// MuJoCo transport also passes, and the fault handling that only a physical transport has.
//
// A fake master on a pseudo-terminal tests the lifecycle and the parser. It is not timing
// evidence and not the H1 emulator of ADR-007: see test/fake_master.hpp.

#include <gtest/gtest.h>

#include <cstdlib>
#include <new>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fake_master.hpp"
#include "humanoid_transport_stm32/stm32_serial_transport.hpp"

// --- Allocation counting, for the real-time claim -----------------------------------------------
// Counts allocations made by the calling thread only, so the fake master's and the reader thread's
// own allocations do not pollute the measurement of the thread that calls exchange().
namespace
{
thread_local std::size_t g_thread_allocations = 0;
}  // namespace

void * operator new(std::size_t size)
{
  ++g_thread_allocations;
  if (void * p = std::malloc(size)) {
    return p;
  }
  throw std::bad_alloc{};
}
void operator delete(void * p) noexcept {std::free(p);}
void operator delete(void * p, std::size_t) noexcept {std::free(p);}

namespace
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using humanoid::transport::CommandBatch;
using humanoid::transport::FeedbackBatch;
using humanoid::transport::JointManifest;
using humanoid::transport::SafetyManifest;
using humanoid::transport::TransportError;
using humanoid::transport_stm32::JointWiring;
using humanoid::transport_stm32::Stm32SerialTransport;
using humanoid::transport_stm32::testing::FakeMaster;
namespace wire = humanoid::transport_stm32::wire;

// Three joints on two slaves. Joint 1 is mirrored and offset, so any missed sign or offset shows.
constexpr std::uint8_t kJoints = 3;

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

class TransportTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    start_fake(default_fake_config());

    const char * names[kJoints] = {"hip", "knee", "ankle"};
    manifest.joint_count = kJoints;
    safety.joint_count = kJoints;
    for (std::uint8_t i = 0; i < kJoints; ++i) {
      std::strncpy(
        manifest.joints[i].name.data(), names[i], humanoid::transport::kMaxNameLength - 1);
      auto & e = safety.envelopes[i];
      e.position_min_rad = -1.5;
      e.position_max_rad = 1.5;
      e.velocity_max_rad_s = 20.0;
      e.torque_peak_nm = 30.0;
      e.stiffness_max_nm_rad = 100.0;
      e.damping_max_nm_s_rad = 5.0;
    }
    safety.feedback_max_age_us = 15000;

    params.serial_device = fake->device();
    params.handshake_timeout = 500ms;
    params.arming_timeout = 1500ms;
    params.release_timeout = 500ms;
    params.fault_grace = 150ms;
    params.wiring = {wiring(0, 0), wiring(0, 1, -1, 0.5), wiring(1, 0)};
  }

  void TearDown() override
  {
    transport.reset();
    fake.reset();
  }

  static FakeMaster::Config default_fake_config()
  {
    FakeMaster::Config config;
    config.chain_ids = {0, 1};
    config.motors_per_chain = {2, 1};
    return config;
  }

  void start_fake(const FakeMaster::Config & config)
  {
    transport.reset();
    fake = std::make_unique<FakeMaster>(config);
    fake->start();
    transport = std::make_unique<Stm32SerialTransport>();
    params.serial_device = fake->device();
  }

  bool configure() {return transport->configure_with(params, manifest, safety);}

  // Configured and activated, ready to exchange.
  void bring_up()
  {
    ASSERT_TRUE(configure());
    ASSERT_TRUE(transport->activate());
  }

  humanoid::transport::ExchangeResult exchange(
    std::chrono::milliseconds budget = 100ms)
  {
    command.sequence = ++sequence;
    command.joint_count = kJoints;
    return transport->exchange(command, feedback, std::chrono::steady_clock::now() + budget);
  }

  // Runs `count` exchanges and returns the last result.
  humanoid::transport::ExchangeResult exchange_many(int count)
  {
    humanoid::transport::ExchangeResult result;
    for (int i = 0; i < count; ++i) {
      result = exchange();
    }
    return result;
  }

  // One exchange, then whether the master reports that motor idle.
  bool exchange_and_check_idle(std::uint8_t chain, std::uint8_t motor)
  {
    return exchange().ok() && fake->state(chain, motor) == wire::Lifecycle::kIdle;
  }

  template<typename Predicate>
  bool eventually(Predicate predicate, std::chrono::milliseconds limit = 1000ms)
  {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < end) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return predicate();
  }

  std::unique_ptr<FakeMaster> fake;
  std::unique_ptr<Stm32SerialTransport> transport;
  Stm32SerialTransport::Parameters params;
  JointManifest manifest{};
  SafetyManifest safety{};
  CommandBatch command{};
  FeedbackBatch feedback{};
  humanoid::transport::CycleSequence sequence{0};
};

// ---------------------------------------------------------------------------
// Conformance: the checks the MuJoCo transport also passes
// ---------------------------------------------------------------------------

TEST_F(TransportTest, C1_ConfigureAcceptsValidManifests)
{
  EXPECT_TRUE(configure());
}

TEST_F(TransportTest, C2_ActivateSucceedsAfterConfigure)
{
  ASSERT_TRUE(configure());
  EXPECT_TRUE(transport->activate());
}

TEST_F(TransportTest, C3_ExchangeReturnsWithinTheDeadline)
{
  bring_up();
  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + 100ms;
  command.sequence = 1;
  command.joint_count = kJoints;
  const auto result = transport->exchange(command, feedback, deadline);
  EXPECT_TRUE(result.ok());
  EXPECT_LE(std::chrono::steady_clock::now(), deadline + 10ms);
}

TEST_F(TransportTest, C4_SequenceNumbersAdvanceAndEcho)
{
  bring_up();
  for (const std::uint64_t seq : {42U, 43U, 44U, 1000U}) {
    command.sequence = seq;
    command.joint_count = kJoints;
    ASSERT_TRUE(transport->exchange(
        command, feedback, std::chrono::steady_clock::now() + 100ms).ok());
    EXPECT_EQ(feedback.sequence, seq);
  }
}

TEST_F(TransportTest, C6_ACommandedTorqueMovesTheJointTheRightWay)
{
  bring_up();
  command.joints[0].effort_nm = 5.0;
  ASSERT_TRUE(exchange_many(5).ok());
  EXPECT_GT(feedback.joints[0].effort_nm, 4.9);

  // Joint 1 is mirrored: a positive joint torque is negative on the wire, and the feedback must
  // come back in joint terms, positive again.
  command.joints[1].effort_nm = 5.0;
  ASSERT_TRUE(exchange_many(5).ok());
  EXPECT_NEAR(feedback.joints[1].effort_nm, 5.0, 0.02);
  const auto wire_command = fake->last_command();
  ASSERT_TRUE(wire_command.has_value());
  EXPECT_LT(wire_command->chains[0].motors[1].tau_ff, 0);
}

TEST_F(TransportTest, C7_HealthReportsActiveOnlyWhileActive)
{
  ASSERT_TRUE(configure());
  EXPECT_FALSE(transport->health_snapshot().active);
  ASSERT_TRUE(transport->activate());
  EXPECT_TRUE(transport->health_snapshot().active);
  transport->deactivate();
  EXPECT_FALSE(transport->health_snapshot().active);
}

TEST_F(TransportTest, C8_ExchangeWhileInactiveFailsWithNotActive)
{
  ASSERT_TRUE(configure());
  EXPECT_EQ(exchange().error, TransportError::kNotActive);
}

TEST_F(TransportTest, CapabilitiesDescribeAFullTuplePhysicalTransport)
{
  ASSERT_TRUE(configure());
  const auto caps = transport->capabilities();
  EXPECT_STREQ(caps.implementation_name.data(), "Stm32SerialTransport");
  EXPECT_EQ(caps.transport_class, humanoid::transport::TransportClass::kPhysical);
  EXPECT_EQ(caps.tuple_completeness, humanoid::transport::TupleCompleteness::kFull);
  EXPECT_EQ(caps.joint_count, kJoints);
  EXPECT_TRUE(caps.supports_per_joint_disable);
  EXPECT_TRUE(caps.supports_availability_mask);
  EXPECT_TRUE(caps.provides_temperature);
  EXPECT_FALSE(caps.provides_bus_voltage);
  EXPECT_FALSE(caps.is_deterministic);
  // The master's own cycle, which humanoid_actuator_system checks against kControlPeriod.
  EXPECT_EQ(caps.nominal_cycle_period_us, 5000U);
}

// ---------------------------------------------------------------------------
// The command reaches the motors
// ---------------------------------------------------------------------------

TEST_F(TransportTest, TheFullTupleReachesTheMasterInJointAndWireUnits)
{
  bring_up();
  command.joints[1].position_rad = 0.3;
  command.joints[1].velocity_rad_s = 2.0;
  command.joints[1].effort_nm = 1.5;
  command.joints[1].stiffness_nm_rad = 40.0;
  command.joints[1].damping_nm_s_rad = 2.0;
  ASSERT_TRUE(exchange_many(5).ok());

  const auto wire_command = fake->last_command();
  ASSERT_TRUE(wire_command.has_value());
  EXPECT_EQ(wire_command->n_chains, 2);
  const auto & motor = wire_command->chains[0].motors[1];  // joint 1: chain 0, motor 1
  EXPECT_EQ(motor.mode_req, static_cast<std::uint8_t>(wire::ModeRequest::kMit));
  EXPECT_EQ(motor.pos, 2000);   // -1 * 0.3 + 0.5 = 0.2 rad
  EXPECT_EQ(motor.vel, -200);   // sign flips motion
  EXPECT_EQ(motor.tau_ff, -150);
  EXPECT_EQ(motor.kp, 400);     // gains keep their sign
  EXPECT_EQ(motor.kd, 200);
  EXPECT_NE(motor.flags & wire::kCmdFlagValid, 0);
  EXPECT_EQ(motor.flags & wire::kCmdFlagFaultReset, 0);  // running never resets a fault
}

TEST_F(TransportTest, FeedbackComesBackInJointUnits)
{
  bring_up();
  command.joints[1].position_rad = 0.3;
  command.joints[1].velocity_rad_s = 2.0;
  ASSERT_TRUE(exchange_many(5).ok());

  // The fake motor follows its command, so this is the whole round trip through both conversions.
  EXPECT_NEAR(feedback.joints[1].position_rad, 0.3, 1.0 / wire::kPosScale);
  EXPECT_NEAR(feedback.joints[1].velocity_rad_s, 2.0, 1.0 / wire::kVelScale);
  EXPECT_FLOAT_EQ(feedback.joints[1].temperature_c, 30.0F);
  EXPECT_TRUE(feedback.joints[1].fresh);
  EXPECT_EQ(feedback.joint_count, kJoints);
}

TEST_F(TransportTest, EveryJointIsFreshAndTheBatchCoversTheManifest)
{
  bring_up();
  const auto result = exchange_many(3);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.joints_reported, kJoints);
  for (std::uint8_t i = 0; i < kJoints; ++i) {
    EXPECT_TRUE(feedback.joints[i].fresh) << "joint " << static_cast<int>(i);
    EXPECT_TRUE(feedback.availability_mask.test(i));
  }
  EXPECT_GT(result.round_trip, std::chrono::nanoseconds::zero());
}

TEST_F(TransportTest, CommandSequenceNumbersOnTheWireAreDistinctAndNeverZero)
{
  bring_up();
  std::uint16_t previous = 0;
  for (int i = 0; i < 20; ++i) {
    ASSERT_TRUE(exchange().ok());
    const auto wire_command = fake->last_command();
    ASSERT_TRUE(wire_command.has_value());
    EXPECT_NE(wire_command->cmd_seq, 0);
    EXPECT_NE(wire_command->cmd_seq, previous);
    previous = wire_command->cmd_seq;
  }
}

// ---------------------------------------------------------------------------
// Configure
// ---------------------------------------------------------------------------

TEST_F(TransportTest, ConfigureCanBeRepeated)
{
  ASSERT_TRUE(configure());
  ASSERT_TRUE(configure());
  EXPECT_TRUE(transport->activate());
}

TEST_F(TransportTest, ConfigureFailsWhenTheDeviceCannotBeOpened)
{
  params.serial_device = "/dev/definitely-not-a-serial-port";
  EXPECT_FALSE(configure());
}

TEST_F(TransportTest, ConfigureFailsWhenTheMasterNeverAnswers)
{
  FakeMaster idle(default_fake_config());  // never started: a silent device
  params.serial_device = idle.device();
  params.handshake_timeout = 150ms;
  EXPECT_FALSE(configure());
}

TEST_F(TransportTest, ConfigureFailsAgainstAMasterOnAnotherProtocolVersion)
{
  auto config = default_fake_config();
  config.status_version = 7;
  start_fake(config);
  params.handshake_timeout = 300ms;
  EXPECT_FALSE(configure());
}

TEST_F(TransportTest, ConfigureFailsWhenPollAndTelemetryRatesDiffer)
{
  auto config = default_fake_config();
  config.poll_hz = 200;
  config.telemetry_hz = 100;
  start_fake(config);
  EXPECT_FALSE(configure());
}

TEST_F(TransportTest, TheReportedCycleIsTheMastersNotAConstantHere)
{
  auto config = default_fake_config();
  config.poll_hz = 400;
  config.telemetry_hz = 400;
  start_fake(config);
  ASSERT_TRUE(configure());
  // humanoid_actuator_system compares this with kControlPeriod and refuses a mismatch.
  EXPECT_EQ(transport->capabilities().nominal_cycle_period_us, 2500U);
}

TEST_F(TransportTest, ConfigureFailsWhenAWiredChainIsNotReporting)
{
  fake->drop_chain(1, true);
  EXPECT_FALSE(configure());
}

TEST_F(TransportTest, ConfigureFailsWhenAChainReportsFewerMotorsThanTheWiringNeeds)
{
  params.wiring[2] = wiring(1, 1);  // chain 1 has one motor
  EXPECT_FALSE(configure());
}

TEST_F(TransportTest, ConfigureFailsOnAWiringThatCountsDifferentlyFromTheManifest)
{
  params.wiring.pop_back();
  EXPECT_FALSE(configure());
}

TEST_F(TransportTest, ConfigureFailsWhenTwoJointsShareAMotor)
{
  params.wiring[2] = wiring(0, 0);
  EXPECT_FALSE(configure());
}

TEST_F(TransportTest, ConfigureFailsOnAnUnsetSignOrOffset)
{
  params.wiring[1].direction_sign = 0;
  EXPECT_FALSE(configure());
  params.wiring[1].direction_sign = -1;
  params.wiring[1].zero_offset_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(configure());
}

TEST_F(TransportTest, ConfigureFailsWhenAnEnvelopeCannotBeCarriedByTheWire)
{
  safety.envelopes[2].stiffness_max_nm_rad = 7000.0;
  EXPECT_FALSE(configure());
}

TEST_F(TransportTest, ConfigureFailsWhenTheSafetyManifestCoversOtherJoints)
{
  safety.joint_count = kJoints - 1;
  EXPECT_FALSE(configure());
}

TEST_F(TransportTest, ConfigureRejectsAFeedbackAgeTheWireCannotEnforce)
{
  safety.feedback_max_age_us = 0;
  EXPECT_FALSE(configure());
  // The wire saturates a drive's age at 255 ms and uses 255 for "never heard from it".
  safety.feedback_max_age_us = 255000;
  EXPECT_FALSE(configure());
  safety.feedback_max_age_us = 254000;
  EXPECT_TRUE(configure());
}

TEST_F(TransportTest, ActivateRefusesAnUnconfiguredTransport)
{
  EXPECT_FALSE(transport->activate());
  params.serial_device = "/dev/definitely-not-a-serial-port";
  EXPECT_FALSE(configure());
  EXPECT_FALSE(transport->activate());  // a failed configure leaves it unusable
}

// ---------------------------------------------------------------------------
// Activation and deactivation
// ---------------------------------------------------------------------------

TEST_F(TransportTest, ActivationArmsEveryMotorIntoHold)
{
  bring_up();
  EXPECT_EQ(fake->state(0, 0), wire::Lifecycle::kHold);
  EXPECT_EQ(fake->state(0, 1), wire::Lifecycle::kHold);
  EXPECT_EQ(fake->state(1, 0), wire::Lifecycle::kHold);
}

TEST_F(TransportTest, ActivationSurvivesASlaveGoingSilentWhileItArms)
{
  auto config = default_fake_config();
  config.arming_silence_cycles = 8;  // the firmware's ~40 ms blocking arm handshake
  start_fake(config);
  bring_up();
  EXPECT_EQ(fake->state(0, 1), wire::Lifecycle::kHold);
}

TEST_F(TransportTest, ActivationFailsAndLeavesNothingArmedWhenAShaftIsWound)
{
  fake->set_wound(0, 1, true);
  ASSERT_TRUE(configure());
  EXPECT_FALSE(transport->activate());

  EXPECT_FALSE(transport->health_snapshot().active);
  // The motors that did arm are released again.
  EXPECT_TRUE(eventually([&] {
      return fake->state(0, 0) == wire::Lifecycle::kIdle &&
             fake->state(1, 0) == wire::Lifecycle::kIdle;
    }));
  EXPECT_EQ(fake->cause(0, 1), wire::FaultCause::kNone);  // the release cleared the latched fault
}

TEST_F(TransportTest, ActivationFailsWhenAChainStopsAnsweringAndLeavesNothingArmed)
{
  params.arming_timeout = 300ms;
  ASSERT_TRUE(configure());
  fake->drop_chain(1, true);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(transport->activate());
  EXPECT_LT(std::chrono::steady_clock::now() - start, 2s);
  EXPECT_FALSE(transport->health_snapshot().active);

  // Nothing is left armed, including the motor on the chain that stopped answering: the release
  // keeps asking for IDLE even though it can never see the answer.
  EXPECT_TRUE(eventually([&] {
      return fake->state(0, 0) == wire::Lifecycle::kIdle &&
             fake->state(0, 1) == wire::Lifecycle::kIdle && fake->state(1,
        0) == wire::Lifecycle::kIdle;
    }));
}

TEST_F(TransportTest, ActivationClearsAFaultLatchedByAPreviousSession)
{
  fake->force_fault(0, 0, wire::FaultCause::kOvertorque);
  ASSERT_TRUE(configure());
  ASSERT_TRUE(transport->activate());  // the fault reset in the clearing request clears it
  EXPECT_EQ(fake->state(0, 0), wire::Lifecycle::kHold);
}

TEST_F(TransportTest, ArmingHoldsAFaultedMotorWhereItIsNotWhereItFaulted)
{
  // The leg was moved while the motor sat faulted. The slave does not poll a FAULT motor, so its
  // idea of the position is still the one from the moment of the fault.
  fake->force_fault(0, 0, wire::FaultCause::kOvertorque);
  fake->set_measured(0, 0, 0.8, 0.0, 0.0);
  ASSERT_TRUE(configure());
  ASSERT_TRUE(transport->activate());

  EXPECT_NEAR(fake->hold_position(0, 0), 0.8, 1e-3);
  EXPECT_NEAR(fake->shaft_position(0, 0), 0.8, 1e-3);  // arming did not step it to 0
}

TEST_F(TransportTest, AMotorItsSlaveNeverDiscoveredIsNeverArmed)
{
  // The drive was powered after its slave booted: it answers now, but the slave never rediscovers
  // it, and arming it past that check would step it to the zero position the slave assumes.
  params.arming_timeout = 400ms;
  fake->set_undiscovered(0, 1);
  fake->set_measured(0, 1, 0.6, 0.0, 0.0);
  ASSERT_TRUE(configure());

  EXPECT_FALSE(transport->activate());
  EXPECT_FALSE(transport->health_snapshot().active);
  EXPECT_EQ(fake->state(0, 1), wire::Lifecycle::kIdle);
  EXPECT_NEAR(fake->shaft_position(0, 1), 0.6, 1e-3);
  EXPECT_TRUE(eventually([&] {
      return fake->state(0, 0) == wire::Lifecycle::kIdle &&
             fake->state(1, 0) == wire::Lifecycle::kIdle;
    }));
}

TEST_F(TransportTest, AMotorWithNoDriveBehindItIsNeverReportedArmed)
{
  // The slave reports HOLD for 100 ms after arming a motor whose drive never answers.
  params.arming_timeout = 400ms;
  fake->set_undiscovered(1, 0);
  fake->set_drive_answers(1, 0, false);
  ASSERT_TRUE(configure());

  EXPECT_FALSE(transport->activate());
  EXPECT_FALSE(transport->health_snapshot().active);
  EXPECT_NE(fake->state(1, 0), wire::Lifecycle::kHold);
}

TEST_F(TransportTest, ActivationFailsWhenADriveStopsAnsweringBeforeItArms)
{
  params.arming_timeout = 400ms;
  ASSERT_TRUE(configure());
  fake->set_drive_answers(0, 1, false);
  // Until its silence outlasts the 15 ms feedback limit, the drive did just answer.
  std::this_thread::sleep_for(50ms);

  EXPECT_FALSE(transport->activate());
  EXPECT_NE(fake->state(0, 1), wire::Lifecycle::kHold);
  EXPECT_NE(fake->state(0, 1), wire::Lifecycle::kMit);
}

TEST_F(TransportTest, ActivationIsIdempotent)
{
  bring_up();
  EXPECT_TRUE(transport->activate());
  EXPECT_TRUE(exchange().ok());
}

TEST_F(TransportTest, DeactivationIdlesEveryMotorAndWaitsForTheMasterToSayIt)
{
  bring_up();
  ASSERT_TRUE(exchange_many(5).ok());
  EXPECT_EQ(fake->state(0, 0), wire::Lifecycle::kMit);

  transport->deactivate();
  EXPECT_EQ(fake->state(0, 0), wire::Lifecycle::kIdle);
  EXPECT_EQ(fake->state(0, 1), wire::Lifecycle::kIdle);
  EXPECT_EQ(fake->state(1, 0), wire::Lifecycle::kIdle);
  EXPECT_EQ(exchange().error, TransportError::kNotActive);
}

TEST_F(TransportTest, ADeactivatedTransportCanBeActivatedAgain)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  transport->deactivate();
  ASSERT_TRUE(transport->activate());
  EXPECT_TRUE(exchange_many(3).ok());
  EXPECT_EQ(fake->state(0, 0), wire::Lifecycle::kMit);
}

TEST_F(TransportTest, DestroyingAnActiveTransportIdlesTheMotors)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  transport.reset();
  EXPECT_EQ(fake->state(0, 0), wire::Lifecycle::kIdle);
}

// ---------------------------------------------------------------------------
// Failure handling on the real-time path
// ---------------------------------------------------------------------------

TEST_F(TransportTest, ASilentMasterTimesOutAtTheDeadlineAndNeverLooksLikeCurrentFeedback)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  const auto last_good = feedback.sequence;

  fake->set_silent(true);
  std::this_thread::sleep_for(30ms);
  const auto start = std::chrono::steady_clock::now();
  command.sequence = last_good + 1;
  command.joint_count = kJoints;
  const auto deadline = start + 30ms;
  const auto result = transport->exchange(command, feedback, deadline);
  const auto end = std::chrono::steady_clock::now();

  EXPECT_EQ(result.error, TransportError::kTimeout);
  EXPECT_LT(end - deadline, 20ms);
  // The caller judges freshness by sequence: it must still show the last good cycle.
  EXPECT_EQ(feedback.sequence, last_good);
}

TEST_F(TransportTest, ExchangeRecoversWhenTheMasterComesBack)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  fake->set_silent(true);
  EXPECT_EQ(exchange(30ms).error, TransportError::kTimeout);
  fake->set_silent(false);
  EXPECT_TRUE(exchange_many(3).ok());
}

TEST_F(TransportTest, GarbageOnTheLineIsSkippedAndCounted)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  fake->inject_bytes({0x01, 0x02, 0x03, 0xFF, 0x09, 0x00});
  ASSERT_TRUE(exchange_many(5).ok());
  EXPECT_GE(transport->health_snapshot().framing_errors, 1U);
}

TEST_F(TransportTest, ACorruptedFrameCostsOneCycleNotTheStream)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  fake->corrupt_next_telemetry();
  int failures = 0;
  for (int i = 0; i < 10; ++i) {
    if (!exchange().ok()) {
      ++failures;
    }
  }
  EXPECT_LE(failures, 1);
  EXPECT_GE(transport->health_snapshot().framing_errors, 1U);
}

TEST_F(TransportTest, ADuplicateFrameIsRejectedAndCounted)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  fake->repeat_last_telemetry();
  ASSERT_TRUE(exchange_many(5).ok());
  EXPECT_EQ(transport->health_snapshot().sequence_rejections, 1U);
}

TEST_F(TransportTest, AMasterResetIsAFaultUntilTheNextActivation)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  fake->restart();

  // The reset master boots disarmed. Whatever it reports now was not armed by this transport.
  EXPECT_TRUE(eventually([&] {return exchange().error == TransportError::kHardwareFault;}));
  EXPECT_EQ(exchange().error, TransportError::kHardwareFault);

  // And nothing more is sent to it: a command to a master that has just booted could be applied
  // to motors nobody has armed.
  const auto sent_before = fake->commands_received();
  EXPECT_EQ(exchange().error, TransportError::kHardwareFault);
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(fake->commands_received(), sent_before);

  transport->deactivate();
  ASSERT_TRUE(transport->activate());
  EXPECT_TRUE(exchange_many(3).ok());
}

TEST_F(TransportTest, AStalledHostLoopTripsTheMastersDeadManAndIsReportedAsAFault)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());

  // No exchanges for longer than the firmware's host-death limit of 12 cycles (60 ms).
  std::this_thread::sleep_for(150ms);
  ASSERT_TRUE(fake->host_lost());

  // The master damped the motors on its own, and says so. This transport must not paper over it
  // by simply carrying on, because the motors are no longer in the state it commanded.
  const auto result = exchange();
  EXPECT_EQ(result.error, TransportError::kHardwareFault);

  // Recovery is an explicit re-activation, whose clearing request carries the fault reset.
  transport->deactivate();
  ASSERT_TRUE(transport->activate());
  EXPECT_FALSE(fake->host_lost());
  EXPECT_TRUE(exchange_many(3).ok());
}

TEST_F(TransportTest, AnUnpluggedDeviceIsAHardwareFault)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  fake->unplug();
  EXPECT_TRUE(eventually([&] {return exchange(50ms).error == TransportError::kHardwareFault;}));
}

TEST_F(TransportTest, AChainThatStopsAnsweringLeavesItsJointsStaleAndUnavailable)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  fake->drop_chain(1, true);

  humanoid::transport::ExchangeResult result;
  ASSERT_TRUE(eventually([&] {
      result = exchange();
      return result.ok() && !feedback.joints[2].fresh;
    }));
  EXPECT_EQ(result.joints_reported, 2);
  EXPECT_TRUE(feedback.joints[0].fresh);
  EXPECT_FALSE(feedback.availability_mask.test(2));
  EXPECT_TRUE(feedback.availability_mask.test(0));
  EXPECT_GE(feedback.availability_epoch, 1U);
}

TEST_F(TransportTest, AllChainsSilentIsAnIncompleteBatch)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  fake->drop_chain(0, true);
  fake->drop_chain(1, true);
  EXPECT_TRUE(eventually([&] {return exchange().error == TransportError::kIncompleteBatch;}));
}

TEST_F(TransportTest, AStaleDriveIsNotFresh)
{
  bring_up();
  fake->set_fb_age(0, 1, 40);  // 40 ms since the drive last reported; the limit is 15 ms
  ASSERT_TRUE(eventually([&] {return exchange().ok() && !feedback.joints[1].fresh;}));
  EXPECT_TRUE(feedback.joints[0].fresh);
  EXPECT_EQ(exchange().joints_reported, 2);
}

// ---------------------------------------------------------------------------
// Isolation, faults, and what the transport refuses to do
// ---------------------------------------------------------------------------

TEST_F(TransportTest, JointDisableIdlesThatMotorAndNoOtherAndIsAcknowledgedInTheMask)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  const auto epoch_before = feedback.availability_epoch;

  EXPECT_TRUE(transport->request_joint_disable(1));
  ASSERT_TRUE(eventually([&] {return exchange_and_check_idle(0, 1);}));
  EXPECT_EQ(fake->state(0, 0), wire::Lifecycle::kMit);
  EXPECT_EQ(fake->state(1, 0), wire::Lifecycle::kMit);

  ASSERT_TRUE(eventually([&] {return exchange().ok() && !feedback.availability_mask.test(1);}));
  EXPECT_TRUE(feedback.availability_mask.test(0));
  EXPECT_GT(feedback.availability_epoch, epoch_before);  // the change is announced, not silent
}

TEST_F(TransportTest, AnIsolatedJointStaysIsolatedAndIsNeverRearmed)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  EXPECT_TRUE(transport->request_joint_disable(0));
  ASSERT_TRUE(eventually([&] {return exchange_and_check_idle(0, 0);}));
  ASSERT_TRUE(exchange_many(30).ok());
  EXPECT_EQ(fake->state(0, 0), wire::Lifecycle::kIdle);
}

TEST_F(TransportTest, DisableAllIdlesEveryMotor)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  EXPECT_TRUE(transport->request_all_disable());
  ASSERT_TRUE(eventually([&] {
      return exchange().ok() && fake->state(0, 0) == wire::Lifecycle::kIdle &&
             fake->state(0, 1) == wire::Lifecycle::kIdle && fake->state(1,
        0) == wire::Lifecycle::kIdle;
    }));
}

TEST_F(TransportTest, DisableRequestsFailForAJointThatDoesNotExistOrWhenInactive)
{
  ASSERT_TRUE(configure());
  EXPECT_FALSE(transport->request_joint_disable(0));  // not active
  EXPECT_FALSE(transport->request_all_disable());
  ASSERT_TRUE(transport->activate());
  EXPECT_FALSE(transport->request_joint_disable(kJoints));
  EXPECT_TRUE(transport->request_joint_disable(kJoints - 1));
}

TEST_F(TransportTest, AFaultedMotorIsReportedAndNeverRearmed)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  fake->force_fault(0, 0, wire::FaultCause::kOvertorque);

  ASSERT_TRUE(eventually([&] {
      return exchange().ok() &&
             (feedback.joints[0].fault_bits & humanoid::transport_stm32::kFaultBitFirmwareFault) !=
             0;
    }));
  const auto bits = feedback.joints[0].fault_bits;
  EXPECT_EQ(bits >> humanoid::transport_stm32::kFaultCauseShift,
    static_cast<unsigned>(wire::FaultCause::kOvertorque));
  EXPECT_FALSE(feedback.availability_mask.test(0));
  // The slave stops polling a FAULT motor, so its sample keeps saying "faulted" but ages out.
  EXPECT_TRUE(eventually([&] {return exchange().ok() && !feedback.joints[0].fresh;}));
  EXPECT_NE(feedback.joints[0].fault_bits & humanoid::transport_stm32::kFaultBitFirmwareFault, 0);

  ASSERT_TRUE(exchange_many(30).ok());
  EXPECT_EQ(fake->state(0, 0), wire::Lifecycle::kFault);  // 30 MIT requests did not rearm it
  EXPECT_EQ(fake->state(0, 1), wire::Lifecycle::kMit);    // and its neighbours carry on
}

TEST_F(TransportTest, ATupleTheWireCannotCarryIsReplacedByDampingNeverClamped)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  const auto last_good = feedback.sequence;

  command.joints[0].stiffness_nm_rad = 1.0e5;  // beyond the wire's Kp range
  command.sequence = last_good + 1;
  command.joint_count = kJoints;
  const auto result = transport->exchange(
    command, feedback, std::chrono::steady_clock::now() + 100ms);

  EXPECT_EQ(result.error, TransportError::kManifestMismatch);
  EXPECT_EQ(feedback.sequence, last_good);  // the cycle is counted as bad by the caller

  // The offending joint is damped on the slave's own gains; its neighbours are untouched.
  ASSERT_TRUE(eventually([&] {
      static_cast<void>(exchange());
      return fake->state(0, 0) == wire::Lifecycle::kDamped;
    }));
  EXPECT_EQ(fake->state(0, 1), wire::Lifecycle::kMit);
  const auto wire_command = fake->last_command();
  ASSERT_TRUE(wire_command.has_value());
  EXPECT_NE(wire_command->chains[0].motors[0].flags & wire::kCmdFlagUseConfigGains, 0);
}

TEST_F(TransportTest, ANonFiniteCommandIsNeverSentAsATuple)
{
  bring_up();
  command.joints[2].position_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(exchange().error, TransportError::kManifestMismatch);
  ASSERT_TRUE(eventually([&] {
      static_cast<void>(exchange());
      return fake->state(1, 0) == wire::Lifecycle::kDamped;
    }));
}

TEST_F(TransportTest, ABatchOfTheWrongSizeIsRejected)
{
  bring_up();
  command.sequence = 1;
  command.joint_count = kJoints + 1;
  EXPECT_EQ(
    transport->exchange(command, feedback, std::chrono::steady_clock::now() + 100ms).error,
    TransportError::kManifestMismatch);
}

// ---------------------------------------------------------------------------
// Real-time properties
// ---------------------------------------------------------------------------

TEST(AllocationCounter, SeesAnAllocationOnItsOwnThread)
{
  // The control for the test below: a counter that counted nothing would pass it vacuously.
  const std::size_t before = g_thread_allocations;
  auto * allocated = new std::vector<int>(100);
  EXPECT_GT(g_thread_allocations, before);
  delete allocated;
}

TEST_F(TransportTest, ExchangeAllocatesNothingOnTheCallingThread)
{
  bring_up();
  ASSERT_TRUE(exchange_many(5).ok());  // warm up

  const std::size_t before = g_thread_allocations;
  for (int i = 0; i < 50; ++i) {
    command.sequence = ++sequence;
    command.joint_count = kJoints;
    const auto result =
      transport->exchange(command, feedback, std::chrono::steady_clock::now() + 100ms);
    ASSERT_TRUE(result.ok());
  }
  EXPECT_EQ(g_thread_allocations - before, 0U);
}

TEST_F(TransportTest, ExchangeIsPacedByTheMastersCycleNotByTheHost)
{
  bring_up();
  ASSERT_TRUE(exchange_many(5).ok());

  const auto start = std::chrono::steady_clock::now();
  constexpr int kCycles = 100;
  for (int i = 0; i < kCycles; ++i) {
    ASSERT_TRUE(exchange().ok());
  }
  const auto elapsed = std::chrono::steady_clock::now() - start;

  // One master cycle per exchange, whatever the host does. A loose bound: SIL timing is never
  // timing evidence (ADR-007-05); this only shows the loop is locked to the master and neither
  // spinning nor stalling.
  const auto per_cycle = elapsed / kCycles;
  EXPECT_GT(per_cycle, 3ms);
  EXPECT_LT(per_cycle, 10ms);
}

}  // namespace
