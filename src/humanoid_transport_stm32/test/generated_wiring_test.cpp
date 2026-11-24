// Copyright 2026 UTRA-RoboSoccer
// The contract with model/generators/configs.py, once per robot it generates a Stm32SerialTransport
// joints map for: the generated wiring is one this transport accepts, for exactly the joints of
// the robot's generated safety manifest, and the wire carries every joint's envelope once mapped
// through its sign and offset. These read the files humanoid_description installs, the ones a
// launch hands the transport.
//
// A joint whose zero the generator leaves out has not been calibrated: the transport reads its
// offset as unset (NaN) and refuses to start until it is (the generated file says so). Such a
// wiring is checked for that refusal and for everything the offset does not decide.

#include <gmock/gmock.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "humanoid_test_support/generated_robots.hpp"
#include "humanoid_test_support/matchers.hpp"
#include "humanoid_transport/safety_manifest.hpp"
#include "humanoid_transport_stm32/joint_wiring.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

using humanoid::test_support::GeneratedRobot;
using humanoid::test_support::generated_robots_where;
using humanoid::test_support::HoldsError;
using humanoid::test_support::HoldsValue;
using humanoid::test_support::RobotName;
using humanoid::transport::JointEnvelope;
using humanoid::transport::kEnvelopeFields;
using humanoid::transport_stm32::check_envelope;
using humanoid::transport_stm32::JointWiring;
using humanoid::transport_stm32::make_layout;
using ::testing::HasSubstr;
using ::testing::UnorderedElementsAreArray;

/// One generated joint: its name and its wiring, the offset unset (NaN) if not yet calibrated.
struct WiredJoint
{
  std::string name;
  JointWiring wiring;

  [[nodiscard]] bool calibrated() const {return !std::isnan(wiring.zero_offset_rad);}
};

class GeneratedWiring : public ::testing::TestWithParam<GeneratedRobot>
{
protected:
  void SetUp() override
  {
    // The parameters of the transport's node, "stm32_serial", as configs.py writes them.
    const auto map = YAML::LoadFile(*GetParam().stm32_wiring())["stm32_serial"]["ros__parameters"]
      ["joints"];
    ASSERT_TRUE(map.IsMap()) << "no stm32_serial joints map";
    for (const auto & entry : map) {
      const auto & w = entry.second;
      const double offset = w["zero_offset_rad"] ?
        w["zero_offset_rad"].as<double>() : std::numeric_limits<double>::quiet_NaN();
      joints_.push_back(
        {entry.first.as<std::string>(),
          JointWiring{
            .chain = static_cast<std::uint8_t>(w["chain"].as<int>()),
            .motor = static_cast<std::uint8_t>(w["motor"].as<int>()),
            .direction_sign = static_cast<std::int8_t>(w["direction_sign"].as<int>()),
            .zero_offset_rad = offset}});
    }
    envelopes_ = YAML::LoadFile(GetParam().safety_manifest())["envelopes"];
    ASSERT_TRUE(envelopes_.IsMap()) << "no envelopes in " << GetParam().safety_manifest();
  }

  std::vector<WiredJoint> joints_;
  YAML::Node envelopes_;
};

TEST_P(GeneratedWiring, WiresExactlyTheJointsOfTheSafetyManifest)
{
  std::vector<std::string> wired;
  for (const auto & joint : joints_) {
    wired.push_back(joint.name);
  }
  std::vector<std::string> enveloped;
  for (const auto & entry : envelopes_) {
    enveloped.push_back(entry.first.as<std::string>());
  }
  EXPECT_THAT(wired, UnorderedElementsAreArray(enveloped));
}

TEST_P(GeneratedWiring, IsALayoutTheTransportAcceptsOnceCalibrated)
{
  std::vector<JointWiring> wiring;
  std::vector<std::string> names;
  std::optional<std::string> uncalibrated;
  for (const auto & joint : joints_) {
    wiring.push_back(joint.wiring);
    names.push_back(joint.name);
    if (!joint.calibrated() && !uncalibrated) {
      uncalibrated = joint.name;
    }
  }
  if (uncalibrated) {
    // Refused, and for that reason alone: with any finite zeros, the rest of the wiring is valid.
    EXPECT_THAT(
      make_layout(wiring, names),
      HoldsError(HasSubstr("'" + *uncalibrated + "': zero_offset_rad is not set")));
    for (auto & w : wiring) {
      w.zero_offset_rad = std::isnan(w.zero_offset_rad) ? 0.0 : w.zero_offset_rad;
    }
  }
  EXPECT_THAT(make_layout(wiring, names), HoldsValue());
}

TEST_P(GeneratedWiring, CarriesEveryJointsEnvelopeOverTheWire)
{
  for (const auto & joint : joints_) {
    SCOPED_TRACE(joint.name);
    const auto fields = envelopes_[joint.name];
    ASSERT_TRUE(fields.IsMap());
    JointEnvelope envelope;
    for (const auto & field : kEnvelopeFields) {
      envelope.*field.member = fields[std::string{field.name}].as<double>();
    }
    if (joint.calibrated()) {
      EXPECT_EQ(check_envelope(joint.wiring, envelope), std::nullopt);
    } else {
      // Where the position range lands on the wire depends on the zero. Every other range does
      // not, so check those, with the position range out of the way at the uncalibrated zero.
      envelope.position_min_rad = 0.0;
      envelope.position_max_rad = 0.0;
      auto at_zero = joint.wiring;
      at_zero.zero_offset_rad = 0.0;
      EXPECT_EQ(check_envelope(at_zero, envelope), std::nullopt);
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  EveryStm32Robot, GeneratedWiring,
  ::testing::ValuesIn(
    generated_robots_where(
      [](const GeneratedRobot & robot) {return robot.stm32_wiring().has_value();})),
  RobotName{});

}  // namespace
