// Copyright 2026 UTRA-RoboSoccer
// Manifests for tests: a JointManifest naming given joints, and a SafetyManifest giving every
// joint one envelope.
#ifndef HUMANOID_TEST_SUPPORT__MANIFESTS_HPP_
#define HUMANOID_TEST_SUPPORT__MANIFESTS_HPP_

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/joint_manifest.hpp"
#include "humanoid_transport/safety_manifest.hpp"

namespace humanoid::test_support
{

/// A JointManifest naming `names`, in order. A name or a count the manifest cannot hold fails the
/// test rather than being truncated.
inline transport::JointManifest joint_manifest(const std::vector<std::string> & names)
{
  transport::JointManifest manifest{};
  if (names.size() > transport::kMaxJoints) {
    ADD_FAILURE() << names.size() << " joints exceed kMaxJoints = " << transport::kMaxJoints;
    return manifest;
  }
  for (std::size_t i = 0; i < names.size(); ++i) {
    auto & name = manifest.joints[i].name;
    if (names[i].size() >= name.size()) {
      ADD_FAILURE() << "joint name '" << names[i] << "' does not fit a JointEntry";
    }
    std::copy_n(names[i].begin(), std::min(names[i].size(), name.size() - 1), name.begin());
  }
  manifest.joint_count = static_cast<std::uint8_t>(names.size());
  return manifest;
}

/// A SafetyManifest for `joint_count` joints, each with `envelope`.
inline transport::SafetyManifest safety_manifest(
  std::uint8_t joint_count, const transport::JointEnvelope & envelope = {},
  std::uint32_t feedback_max_age_us = 15000U)
{
  transport::SafetyManifest safety{};
  safety.joint_count = joint_count;
  safety.feedback_max_age_us = feedback_max_age_us;
  std::fill_n(safety.envelopes.begin(), joint_count, envelope);
  return safety;
}

}  // namespace humanoid::test_support

#endif  // HUMANOID_TEST_SUPPORT__MANIFESTS_HPP_
