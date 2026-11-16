// Copyright 2026 UTRA-RoboSoccer

#include <gmock/gmock.h>

#include <string>
#include <variant>
#include <vector>

#include "humanoid_actuator_system/safety_manifest_parser.hpp"
#include "humanoid_test_support/matchers.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

using humanoid::actuator_system::parse_safety_manifest;
using humanoid::test_support::HoldsError;
using humanoid::test_support::HoldsValue;
using humanoid::transport::kEnvelopeFields;
using humanoid::transport::SafetyManifest;
using ::testing::_;
using ::testing::HasSubstr;

const std::vector<std::string> kJoints{"left_knee_joint", "right_knee_joint"};

/// A valid manifest for kJoints. Field i of joint j holds 10 * j + i, so a value read into the
/// wrong member is detected.
YAML::Node valid_manifest()
{
  YAML::Node manifest;
  manifest["schema_version"] = std::string{humanoid::transport::kSafetyManifestSchemaVersion};
  manifest["source_digest_note"] = "test";
  manifest["joint_count"] = kJoints.size();
  manifest["max_consecutive_bad_cycles"] = 3;
  manifest["feedback_max_age_us"] = 15000;
  for (std::size_t j = 0U; j < kJoints.size(); ++j) {
    for (std::size_t i = 0U; i < kEnvelopeFields.size(); ++i) {
      manifest["envelopes"][kJoints[j]][std::string{kEnvelopeFields[i].name}] =
        static_cast<double>(10U * j + i);
    }
  }
  return manifest;
}

auto parse(const YAML::Node & manifest, const std::vector<std::string> & joints = kJoints)
{
  return parse_safety_manifest(manifest, joints);
}

TEST(SafetyManifestParser, ReadsEachKeyIntoItsOwnField)
{
  const auto result = parse(valid_manifest());
  ASSERT_THAT(result, HoldsValue());
  const auto & manifest = std::get<SafetyManifest>(result);
  EXPECT_EQ(manifest.joint_count, kJoints.size());
  EXPECT_EQ(manifest.max_consecutive_bad_cycles, 3U);
  EXPECT_EQ(manifest.feedback_max_age_us, 15000U);
  for (std::size_t j = 0U; j < kJoints.size(); ++j) {
    for (std::size_t i = 0U; i < kEnvelopeFields.size(); ++i) {
      EXPECT_EQ(manifest.envelopes[j].*kEnvelopeFields[i].member, static_cast<double>(10U * j + i))
        << kJoints[j] << "/" << kEnvelopeFields[i].name;
    }
  }
}

TEST(SafetyManifestParser, OrdersEnvelopesByTheDescriptionNotTheFile)
{
  const std::vector<std::string> reversed{kJoints.rbegin(), kJoints.rend()};
  const auto result = parse(valid_manifest(), reversed);
  ASSERT_THAT(result, HoldsValue());
  EXPECT_EQ(
    std::get<SafetyManifest>(result).envelopes[0].*kEnvelopeFields[0].member, 10.0);
}

TEST(SafetyManifestParser, RejectsAnotherSchemaVersion)
{
  auto manifest = valid_manifest();
  manifest["schema_version"] = "0.2.0";
  EXPECT_THAT(parse(manifest), HoldsError(HasSubstr("schema_version")));
}

TEST(SafetyManifestParser, RejectsAMissingEnvelopeKey)
{
  auto manifest = valid_manifest();
  manifest["envelopes"][kJoints[1]].remove("power_max_w");
  EXPECT_THAT(parse(manifest), HoldsError(HasSubstr("missing key 'power_max_w'")));
}

TEST(SafetyManifestParser, RejectsAnUnknownEnvelopeKey)
{
  // A misspelling is two errors at once: an unknown key and a missing one.
  auto manifest = valid_manifest();
  manifest["envelopes"][kJoints[0]]["power_max_W"] = 1.0;
  EXPECT_THAT(parse(manifest), HoldsError(HasSubstr("unexpected key 'power_max_W'")));
}

TEST(SafetyManifestParser, RejectsAnUnknownOrMissingTopLevelKey)
{
  auto extra = valid_manifest();
  extra["digest"] = "abc";
  EXPECT_THAT(parse(extra), HoldsError(HasSubstr("unexpected key 'digest'")));

  auto missing = valid_manifest();
  missing.remove("feedback_max_age_us");
  EXPECT_THAT(parse(missing), HoldsError(HasSubstr("missing key 'feedback_max_age_us'")));
}

TEST(SafetyManifestParser, RequiresExactlyTheDescribedJoints)
{
  auto extra = valid_manifest();
  extra["envelopes"]["elbow_joint"] = extra["envelopes"][kJoints[0]];
  EXPECT_THAT(parse(extra), HoldsError(HasSubstr("'elbow_joint'")));

  EXPECT_THAT(parse(valid_manifest(), {kJoints[0]}), HoldsError(HasSubstr("joint_count")));

  auto missing = valid_manifest();
  missing["envelopes"].remove(kJoints[1]);
  missing["joint_count"] = 2;
  EXPECT_THAT(parse(missing), HoldsError(HasSubstr("no envelope for joint")));
}

TEST(SafetyManifestParser, RejectsValuesOfTheWrongType)
{
  auto manifest = valid_manifest();
  manifest["envelopes"][kJoints[0]]["torque_peak_nm"] = "high";
  EXPECT_THAT(parse(manifest), HoldsError(_));

  auto out_of_range = valid_manifest();
  out_of_range["max_consecutive_bad_cycles"] = 300;
  EXPECT_THAT(parse(out_of_range), HoldsError(_));
}

}  // namespace
