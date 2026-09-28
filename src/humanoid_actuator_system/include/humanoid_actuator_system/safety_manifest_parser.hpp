// Copyright 2026 UTRA-RoboSoccer
// Reads the ADR-002 safety manifest that model/generators/emit.py writes.
//
// Strict by design: every key is required, no other key is accepted, and the schema version must
// be the one this build reads. A misspelled key would otherwise leave a limit at its zero default,
// and a new one would be silently ignored.
#ifndef HUMANOID_ACTUATOR_SYSTEM__SAFETY_MANIFEST_PARSER_HPP_
#define HUMANOID_ACTUATOR_SYSTEM__SAFETY_MANIFEST_PARSER_HPP_

#include <span>  // NOLINT(build/include_order)
#include <string>
#include <variant>

#include "humanoid_transport/safety_manifest.hpp"
#include "yaml-cpp/yaml.h"

namespace humanoid::actuator_system
{

/// The manifest, or why it was rejected. (std::expected is C++23.)
using ManifestOrError = std::variant<transport::SafetyManifest, std::string>;

/// Parses `manifest` for `joint_names`, whose order sets the envelope order. The manifest must
/// cover exactly those joints. manifest_digest is left empty.
[[nodiscard]] ManifestOrError parse_safety_manifest(
  const YAML::Node & manifest, std::span<const std::string> joint_names);

}  // namespace humanoid::actuator_system

#endif  // HUMANOID_ACTUATOR_SYSTEM__SAFETY_MANIFEST_PARSER_HPP_
