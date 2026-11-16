// Copyright 2026 UTRA-RoboSoccer
// The ADR-001 transport conformance suite: the behaviour every ActuatorTransport owes the layers
// above it, run unchanged against each transport. ADR-001 requires it to "run unchanged against
// STM32, simulator, and test-double implementations", and ADR-007 makes a transport-specific
// exemption a defect, not a configuration. What only one transport can show, such as a fake
// master's fault injection or MuJoCo's physics, stays in that transport's own tests.
//
// A transport's package instantiates the suite in one test file, with a harness that builds the
// transport and configures it as its deployment does:
//
//   INSTANTIATE_TEST_SUITE_P(
//     Stm32, TransportConformance,
//     ::testing::Values(TransportUnderTest{"FakeMaster", [] {return std::make_unique<...>();}}),
//     TransportName{});
//
// This header defines tests and replaces the global operator new (allocation_counter.hpp), so
// include it in exactly one translation unit of a test binary.
#ifndef HUMANOID_TEST_SUPPORT__TRANSPORT_CONFORMANCE_HPP_
#define HUMANOID_TEST_SUPPORT__TRANSPORT_CONFORMANCE_HPP_

#include <gmock/gmock.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "humanoid_test_support/allocation_counter.hpp"
#include "humanoid_transport/actuator_transport.hpp"
#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/joint_manifest.hpp"
#include "humanoid_transport/timing.hpp"

namespace humanoid::test_support
{

/// One transport, set up for one test the way its deployment runs it: a fake master behind the
/// STM32 transport, a generated MJCF behind MuJoCo's.
class ConformanceHarness
{
public:
  virtual ~ConformanceHarness() = default;

  [[nodiscard]] virtual transport::ActuatorTransport & transport() = 0;

  /// configure() with the harness's manifests, through whatever path the deployment uses.
  [[nodiscard]] virtual bool configure() = 0;

  [[nodiscard]] virtual const transport::JointManifest & joint_manifest() const = 0;
};

/// A suite parameter: a name for the test output and a harness for each test.
struct TransportUnderTest
{
  std::string name;
  std::function<std::unique_ptr<ConformanceHarness>()> make;
};

inline std::ostream & operator<<(std::ostream & out, const TransportUnderTest & t)
{
  return out << t.name;
}

/// INSTANTIATE_TEST_SUITE_P's name generator: each instance is named after its transport.
struct TransportName
{
  template<typename ParamInfo>
  std::string operator()(const ParamInfo & info) const {return info.param.name;}
};

class TransportConformance : public ::testing::TestWithParam<TransportUnderTest>
{
protected:
  /// The cycles a commanded effort may take to show in the feedback: a physical transport's
  /// feedback trails its command by the master's pipeline.
  static constexpr int kSettleCycles = 5;
  /// Small enough for every actuator's torque range, large enough to read past quantisation.
  static constexpr double kEffortNm = 1.0;
  static constexpr double kEffortToleranceNm = 0.05;

  void SetUp() override
  {
    harness_ = GetParam().make();
    ASSERT_NE(harness_, nullptr);
    command_.joint_count = joint_count();
  }

  void TearDown() override {harness_.reset();}

  transport::ActuatorTransport & transport() {return harness_->transport();}
  std::uint8_t joint_count() const {return harness_->joint_manifest().joint_count;}

  /// Configured and activated, ready to exchange.
  void bring_up()
  {
    ASSERT_TRUE(harness_->configure());
    ASSERT_TRUE(transport().activate());
  }

  /// One exchange of command_ under the next sequence number, due `budget` from now.
  transport::ExchangeResult exchange(
    std::chrono::milliseconds budget = std::chrono::milliseconds{100})
  {
    command_.sequence = ++sequence_;
    return transport().exchange(command_, feedback_, std::chrono::steady_clock::now() + budget);
  }

  /// `count` exchanges; the last one's result.
  transport::ExchangeResult exchange_many(int count)
  {
    transport::ExchangeResult result;
    for (int i = 0; i < count; ++i) {
      result = exchange();
    }
    return result;
  }

  std::unique_ptr<ConformanceHarness> harness_;
  transport::CommandBatch command_{};
  transport::FeedbackBatch feedback_{};
  transport::CycleSequence sequence_{0U};
};

// --- Lifecycle ---------------------------------------------------------------------------------

TEST_P(TransportConformance, ConfigureAcceptsItsManifests)
{
  EXPECT_TRUE(harness_->configure());
}

TEST_P(TransportConformance, ConfigureCanBeRepeated)
{
  ASSERT_TRUE(harness_->configure());
  ASSERT_TRUE(harness_->configure());
  EXPECT_TRUE(transport().activate());
}

TEST_P(TransportConformance, ActivateRefusesAnUnconfiguredTransport)
{
  EXPECT_FALSE(transport().activate());
}

TEST_P(TransportConformance, HealthReportsActiveOnlyWhileActive)
{
  ASSERT_TRUE(harness_->configure());
  EXPECT_FALSE(transport().health_snapshot().active);
  ASSERT_TRUE(transport().activate());
  EXPECT_TRUE(transport().health_snapshot().active);
  transport().deactivate();
  EXPECT_FALSE(transport().health_snapshot().active);
}

TEST_P(TransportConformance, ADeactivatedTransportCanBeActivatedAgain)
{
  bring_up();
  ASSERT_TRUE(exchange_many(3).ok());
  transport().deactivate();
  ASSERT_TRUE(transport().activate());
  EXPECT_TRUE(exchange_many(3).ok());
}

// --- Capabilities ------------------------------------------------------------------------------

TEST_P(TransportConformance, CapabilitiesDescribeTheConfiguredTransport)
{
  bring_up();
  const auto caps = transport().capabilities();
  EXPECT_STRNE(caps.implementation_name.data(), "");
  EXPECT_EQ(caps.joint_count, joint_count());
  // humanoid_actuator_system refuses a transport whose cycle is not the control period.
  EXPECT_EQ(
    caps.nominal_cycle_period_us, static_cast<std::uint32_t>(transport::kControlPeriod.count()));
}

TEST_P(TransportConformance, ReportsAnImuExactlyWhenItSaysItHasOne)
{
  bring_up();
  ASSERT_TRUE(exchange().ok());
  const auto & imu = feedback_.imu;
  if (!transport().capabilities().provides_imu) {
    EXPECT_FALSE(imu.valid);
    return;
  }
  ASSERT_TRUE(imu.valid);
  const double norm = std::sqrt(
    imu.orientation_x * imu.orientation_x + imu.orientation_y * imu.orientation_y +
    imu.orientation_z * imu.orientation_z + imu.orientation_w * imu.orientation_w);
  EXPECT_NEAR(norm, 1.0, 1e-9);
}

// --- The exchange ------------------------------------------------------------------------------

TEST_P(TransportConformance, ExchangeWhileInactiveFailsWithNotActive)
{
  ASSERT_TRUE(harness_->configure());
  EXPECT_EQ(exchange().error, transport::TransportError::kNotActive);
}

TEST_P(TransportConformance, ExchangeReturnsByTheDeadline)
{
  bring_up();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{100};
  command_.sequence = ++sequence_;
  EXPECT_TRUE(transport().exchange(command_, feedback_, deadline).ok());
  // A loose margin: SIL timing is never timing evidence (ADR-007-05).
  EXPECT_LE(std::chrono::steady_clock::now(), deadline + std::chrono::milliseconds{10});
}

TEST_P(TransportConformance, FeedbackEchoesTheCommandsSequence)
{
  bring_up();
  for (const transport::CycleSequence sequence : {42U, 43U, 44U, 1000U}) {
    command_.sequence = sequence;
    ASSERT_TRUE(
      transport().exchange(
        command_, feedback_,
        std::chrono::steady_clock::now() + std::chrono::milliseconds{100}).ok());
    EXPECT_EQ(feedback_.sequence, sequence);
  }
}

TEST_P(TransportConformance, FeedbackCoversEveryJointOfTheManifest)
{
  bring_up();
  const auto result = exchange_many(3);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.joints_reported, joint_count());
  EXPECT_EQ(feedback_.joint_count, joint_count());
  for (std::uint8_t i = 0; i < joint_count(); ++i) {
    EXPECT_TRUE(feedback_.joints[i].fresh) << "joint " << +i;
  }
  EXPECT_GT(result.round_trip, std::chrono::nanoseconds::zero());
}

TEST_P(TransportConformance, ACommandedEffortComesBackInJointTermsWithItsSign)
{
  // Units, signs and joint order: a wire or model joint that is mirrored, swapped or scaled shows
  // here as a joint whose effort comes back wrong (ADR-007 gate 2, sign-flip detection).
  bring_up();
  for (const double effort : {kEffortNm, -kEffortNm}) {
    SCOPED_TRACE("commanded effort " + std::to_string(effort) + " N*m on every joint");
    for (std::uint8_t i = 0; i < joint_count(); ++i) {
      command_.joints[i] = transport::JointCommand{.effort_nm = effort};
    }
    ASSERT_TRUE(exchange_many(kSettleCycles).ok());
    for (std::uint8_t i = 0; i < joint_count(); ++i) {
      EXPECT_NEAR(feedback_.joints[i].effort_nm, effort, kEffortToleranceNm)
        << "joint " << +i << " (" << harness_->joint_manifest().joints[i].name.data() << ")";
    }
  }
}

TEST_P(TransportConformance, ABatchOfTheWrongSizeFailsTheCycle)
{
  bring_up();
  command_.joint_count = static_cast<std::uint8_t>(joint_count() + 1U);
  EXPECT_EQ(exchange().error, transport::TransportError::kManifestMismatch);
}

TEST_P(TransportConformance, ANonFiniteCommandFailsTheCycle)
{
  // In the integrated system the actuator layer stops a non-finite command first; this is the
  // transport's own backstop. Every field of the tuple, each on the last joint.
  bring_up();
  ASSERT_TRUE(exchange().ok());
  const std::uint8_t last = static_cast<std::uint8_t>(joint_count() - 1U);
  for (const auto & field : transport::kMitFields) {
    SCOPED_TRACE(std::string{field.name} + " = NaN");
    command_.joints[last] = transport::JointCommand{};
    command_.joints[last].*field.member = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(exchange().error, transport::TransportError::kManifestMismatch);
  }
}

// --- Real time ---------------------------------------------------------------------------------

TEST(AllocationCounter, SeesAnAllocationOnItsOwnThread)
{
  // The control for the test below: a counter that counted nothing would pass it vacuously.
  const std::size_t before = thread_allocations();
  auto * allocated = new std::vector<int>(100);
  EXPECT_GT(thread_allocations(), before);
  delete allocated;
}

TEST_P(TransportConformance, ExchangeAllocatesNothingOnTheCallingThread)
{
  bring_up();
  ASSERT_TRUE(exchange_many(5).ok());  // warm up
  const std::size_t before = thread_allocations();
  for (int i = 0; i < 50; ++i) {
    ASSERT_TRUE(exchange().ok());
  }
  EXPECT_EQ(thread_allocations() - before, 0U);
}

}  // namespace humanoid::test_support

#endif  // HUMANOID_TEST_SUPPORT__TRANSPORT_CONFORMANCE_HPP_
