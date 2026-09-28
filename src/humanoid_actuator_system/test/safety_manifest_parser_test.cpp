// Copyright 2026 UTRA-RoboSoccer

#include <gtest/gtest.h>

#include <string>
#include <variant>
#include <vector>

#include "humanoid_actuator_system/safety_manifest_parser.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

using humanoid::actuator_system::parse_safety_manifest;
using humanoid::transport::kEnvelopeFields;
using humanoid::transport::SafetyManifest;

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

std::string error_of(const YAML::Node & manifest, const std::vector<std::string> & joints = kJoints)
{
  const auto result = parse_safety_manifest(manifest, joints);
  const auto * error = std::get_if<std::string>(&result);
  return error ? *error : std::string{};
}

TEST(SafetyManifestParser, ReadsTheGeneratedManifest)
{
  // The contract with model/generators/emit.py: whatever it writes, this build must read.
  const YAML::Node manifest = YAML::LoadFile(HUMANOID_TEST_SAFETY_MANIFEST_PATH);
  std::vector<std::string> joints;
  for (const auto & entry : manifest["envelopes"]) {
    joints.push_back(entry.first.as<std::string>());
  }
  ASSERT_FALSE(joints.empty());
  EXPECT_EQ(error_of(manifest, joints), "");
}

TEST(SafetyManifestParser, ReadsEachKeyIntoItsOwnField)
{
  const auto result = parse_safety_manifest(valid_manifest(), kJoints);
  ASSERT_TRUE(std::holds_alternative<SafetyManifest>(result)) << error_of(valid_manifest());
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
  const auto result = parse_safety_manifest(valid_manifest(), reversed);
  ASSERT_TRUE(std::holds_alternative<SafetyManifest>(result));
  EXPECT_EQ(
    std::get<SafetyManifest>(result).envelopes[0].*kEnvelopeFields[0].member, 10.0);
}

TEST(SafetyManifestParser, RejectsAnotherSchemaVersion)
{
  auto manifest = valid_manifest();
  manifest["schema_version"] = "0.2.0";
  EXPECT_NE(error_of(manifest).find("schema_version"), std::string::npos);
}

TEST(SafetyManifestParser, RejectsAMissingEnvelopeKey)
{
  auto manifest = valid_manifest();
  manifest["envelopes"][kJoints[1]].remove("power_max_w");
  EXPECT_NE(error_of(manifest).find("missing key 'power_max_w'"), std::string::npos);
}

TEST(SafetyManifestParser, RejectsAnUnknownEnvelopeKey)
{
  // A misspelling is two errors at once: an unknown key and a missing one.
  auto manifest = valid_manifest();
  manifest["envelopes"][kJoints[0]]["power_max_W"] = 1.0;
  EXPECT_NE(error_of(manifest).find("unexpected key 'power_max_W'"), std::string::npos);
}

TEST(SafetyManifestParser, RejectsAnUnknownOrMissingTopLevelKey)
{
  auto extra = valid_manifest();
  extra["digest"] = "abc";
  EXPECT_NE(error_of(extra).find("unexpected key 'digest'"), std::string::npos);

  auto missing = valid_manifest();
  missing.remove("feedback_max_age_us");
  EXPECT_NE(error_of(missing).find("missing key 'feedback_max_age_us'"), std::string::npos);
}

TEST(SafetyManifestParser, RequiresExactlyTheDescribedJoints)
{
  auto extra = valid_manifest();
  extra["envelopes"]["elbow_joint"] = extra["envelopes"][kJoints[0]];
  EXPECT_NE(error_of(extra).find("'elbow_joint'"), std::string::npos);

  EXPECT_NE(error_of(valid_manifest(), {kJoints[0]}).find("joint_count"), std::string::npos);

  auto missing = valid_manifest();
  missing["envelopes"].remove(kJoints[1]);
  missing["joint_count"] = 2;
  EXPECT_NE(error_of(missing).find("no envelope for joint"), std::string::npos);
}

TEST(SafetyManifestParser, RejectsValuesOfTheWrongType)
{
  auto manifest = valid_manifest();
  manifest["envelopes"][kJoints[0]]["torque_peak_nm"] = "high";
  EXPECT_NE(error_of(manifest), "");

  auto out_of_range = valid_manifest();
  out_of_range["max_consecutive_bad_cycles"] = 300;
  EXPECT_NE(error_of(out_of_range), "");
}

}  // namespace
