// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_actuator_system/safety_manifest_parser.hpp"

#include <format>

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace humanoid::actuator_system
{

namespace
{
constexpr std::string_view kSchemaVersionKey = "schema_version";
constexpr std::string_view kSourceDigestNoteKey = "source_digest_note";
constexpr std::string_view kJointCountKey = "joint_count";
constexpr std::string_view kMaxBadCyclesKey = "max_consecutive_bad_cycles";
constexpr std::string_view kFeedbackMaxAgeKey = "feedback_max_age_us";
constexpr std::string_view kEnvelopesKey = "envelopes";
constexpr std::array<std::string_view, 6> kTopLevelKeys{
  kSchemaVersionKey, kSourceDigestNoteKey, kJointCountKey, kMaxBadCyclesKey, kFeedbackMaxAgeKey,
  kEnvelopesKey};

template<typename IsKnown>
std::optional<std::string> unexpected_key(const YAML::Node & map, IsKnown is_known);

YAML::Node at(const YAML::Node & map, std::string_view key);
}  // namespace

ManifestOrError parse_safety_manifest(
  const YAML::Node & manifest, std::span<const std::string> joint_names)
{
  if (joint_names.size() > transport::kMaxJoints) {
    return std::format("{} joints exceed kMaxJoints={}", joint_names.size(), transport::kMaxJoints);
  }
  try {
    if (!manifest.IsMap()) {
      return std::string{"the manifest is not a map"};
    }
    const auto is_top_level = [](std::string_view key) {
        return std::find(kTopLevelKeys.begin(), kTopLevelKeys.end(), key) != kTopLevelKeys.end();
      };
    if (const auto key = unexpected_key(manifest, is_top_level)) {
      return std::format("unexpected key '{}'", *key);
    }
    for (const auto key : kTopLevelKeys) {
      if (!at(manifest, key)) {
        return std::format("missing key '{}'", key);
      }
    }

    const auto version = at(manifest, kSchemaVersionKey).as<std::string>();
    if (version != transport::kSafetyManifestSchemaVersion) {
      return std::format(
        "schema_version '{}' is not '{}', the version this build reads", version,
        transport::kSafetyManifestSchemaVersion);
    }
    const auto joint_count = at(manifest, kJointCountKey).as<std::size_t>();
    if (joint_count != joint_names.size()) {
      return std::format(
        "joint_count is {}, but the robot description declares {} joints", joint_count,
        joint_names.size());
    }

    transport::SafetyManifest out{};
    out.joint_count = static_cast<std::uint8_t>(joint_names.size());
    out.max_consecutive_bad_cycles = at(manifest, kMaxBadCyclesKey).as<std::uint8_t>();
    out.feedback_max_age_us = at(manifest, kFeedbackMaxAgeKey).as<std::uint32_t>();

    const YAML::Node envelopes = at(manifest, kEnvelopesKey);
    if (!envelopes.IsMap()) {
      return std::string{"'envelopes' is not a map"};
    }
    const auto is_joint = [joint_names](std::string_view key) {
        return std::find(joint_names.begin(), joint_names.end(), key) != joint_names.end();
      };
    if (const auto key = unexpected_key(envelopes, is_joint)) {
      return std::format("envelope for '{}', which is not a declared joint", *key);
    }
    const auto is_field = [](std::string_view key) {
        return std::any_of(
          transport::kEnvelopeFields.begin(), transport::kEnvelopeFields.end(),
          [key](const auto & field) {return field.name == key;});
      };
    for (std::size_t i = 0U; i < joint_names.size(); ++i) {
      const YAML::Node envelope = at(envelopes, joint_names[i]);
      if (!envelope || !envelope.IsMap()) {
        return std::format("no envelope for joint '{}'", joint_names[i]);
      }
      if (const auto key = unexpected_key(envelope, is_field)) {
        return std::format("joint '{}': unexpected key '{}'", joint_names[i], *key);
      }
      for (const auto & field : transport::kEnvelopeFields) {
        const YAML::Node value = at(envelope, field.name);
        if (!value) {
          return std::format("joint '{}': missing key '{}'", joint_names[i], field.name);
        }
        out.envelopes[i].*field.member = value.as<double>();
      }
    }
    return out;
  } catch (const YAML::Exception & e) {
    return std::string{e.what()};
  }
}

namespace
{

/// The first key of `map` that `is_known` rejects, if any.
template<typename IsKnown>
std::optional<std::string> unexpected_key(const YAML::Node & map, IsKnown is_known)
{
  for (const auto & entry : map) {
    auto key = entry.first.as<std::string>();
    if (!is_known(key)) {
      return key;
    }
  }
  return std::nullopt;
}

/// Lookup that cannot insert: operator[] on a non-const Node would add the key.
YAML::Node at(const YAML::Node & map, std::string_view key)
{
  return map[std::string{key}];
}

}  // namespace

}  // namespace humanoid::actuator_system
