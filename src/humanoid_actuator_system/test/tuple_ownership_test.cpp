// Copyright 2026 UTRA-RoboSoccer

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <string_view>
#include <vector>

#include "humanoid_actuator_system/tuple_ownership.hpp"

namespace
{

using humanoid::actuator_system::all_joints;
using humanoid::actuator_system::apply_claims;
using humanoid::actuator_system::CommandAuthority;
using humanoid::actuator_system::JointMask;
using humanoid::actuator_system::next_authority;
using humanoid::actuator_system::TupleClaims;
using humanoid::actuator_system::tuple_claims;
using humanoid::transport::kMitFields;

const std::vector<std::string> kJoints{"left_knee_joint", "right_knee_joint", "waist_yaw_joint"};

/// All five interface names of each listed joint, in the order a controller claims them.
std::vector<std::string> tuples(const std::vector<std::string> & joints)
{
  std::vector<std::string> names;
  for (const auto & joint : joints) {
    for (const auto & field : kMitFields) {
      names.push_back(joint + "/" + std::string{field.name});
    }
  }
  return names;
}

// --- tuple_claims: the prepare_command_mode_switch veto ---

TEST(TupleClaims, AcceptsEveryJointWhole)
{
  const auto claims = tuple_claims(tuples(kJoints), {}, kJoints);
  ASSERT_TRUE(claims);
  EXPECT_EQ(claims->start, all_joints(kJoints.size()));
  EXPECT_EQ(claims->stop, 0U);
}

TEST(TupleClaims, AcceptsSomeJointsWhole)
{
  // A controller that owns only the waist claims its tuple whole.
  const auto claims = tuple_claims(tuples({"waist_yaw_joint"}), {}, kJoints);
  ASSERT_TRUE(claims);
  EXPECT_EQ(claims->start, JointMask{1U} << 2U);
}

TEST(TupleClaims, RejectsPositionAlone)
{
  EXPECT_FALSE(tuple_claims({"left_knee_joint/position"}, {}, kJoints));
}

TEST(TupleClaims, RejectsOneFieldShortOfATuple)
{
  for (std::size_t missing = 0U; missing < kMitFields.size(); ++missing) {
    auto names = tuples(kJoints);
    names.erase(names.begin() + static_cast<std::ptrdiff_t>(missing));
    EXPECT_FALSE(tuple_claims(names, {}, kJoints)) << "missing " << kMitFields[missing].name;
  }
}

TEST(TupleClaims, RejectsPartialRelease)
{
  auto names = tuples(kJoints);
  names.pop_back();  // right-most joint keeps its damping
  EXPECT_FALSE(tuple_claims({}, names, kJoints));
}

TEST(TupleClaims, RejectsNamesThatAreNotOurs)
{
  auto names = tuples(kJoints);
  names.push_back("left_knee_joint/torque");
  EXPECT_FALSE(tuple_claims(names, {}, kJoints));
  EXPECT_FALSE(tuple_claims({"elbow_joint/position"}, {}, kJoints));
  EXPECT_FALSE(tuple_claims({"no_separator"}, {}, kJoints));
}

TEST(TupleClaims, MatchesWholeJointNamesNotPrefixes)
{
  // "left_knee" is a prefix of a real joint, not a joint.
  const std::vector<std::string> names{
    "left_knee/position", "left_knee/velocity", "left_knee/effort", "left_knee/stiffness",
    "left_knee/damping"};
  EXPECT_FALSE(tuple_claims(names, {}, kJoints));
}

TEST(TupleClaims, AcceptsAHandOverInOneSwitch)
{
  const auto claims = tuple_claims(tuples(kJoints), tuples(kJoints), kJoints);
  ASSERT_TRUE(claims);
  EXPECT_EQ(claims->start, all_joints(kJoints.size()));
  EXPECT_EQ(claims->stop, all_joints(kJoints.size()));
}

TEST(TupleClaims, CannotSeeASplitBetweenTwoControllersInOneRequest)
{
  // The lists are a union over controllers. Two that split every tuple between them look exactly
  // like one controller claiming it whole; the hardware cannot reject this. Recorded so that the
  // limit stays visible.
  const auto names = tuples(kJoints);
  // Controller A claims the first seven names, splitting right_knee_joint; B claims the rest.
  std::vector<std::string> requested(names.begin(), names.begin() + 7);
  requested.insert(requested.end(), names.begin() + 7, names.end());
  EXPECT_TRUE(tuple_claims(requested, {}, kJoints));
}

// --- apply_claims: what perform_command_mode_switch records ---

TEST(ApplyClaims, AHandOverLeavesEveryJointOwned)
{
  const JointMask all = all_joints(kJoints.size());
  EXPECT_EQ(apply_claims(all, TupleClaims{all, all}), all);
}

TEST(ApplyClaims, ReleaseClearsOnlyTheReleasedJoints)
{
  const JointMask all = all_joints(kJoints.size());
  EXPECT_EQ(apply_claims(all, TupleClaims{0U, JointMask{1U}}), all & ~JointMask{1U});
}

TEST(ApplyClaims, ControllersThatShareTheRobotByJointAddUp)
{
  const JointMask legs = apply_claims(0U, TupleClaims{0b011U, 0U});
  EXPECT_EQ(apply_claims(legs, TupleClaims{0b100U, 0U}), all_joints(kJoints.size()));
}

TEST(AllJoints, CoversExactlyTheManifest)
{
  EXPECT_EQ(all_joints(0U), 0U);
  EXPECT_EQ(all_joints(3U), 0b111U);
  EXPECT_EQ(all_joints(humanoid::transport::kMaxJoints), ~JointMask{0U});
}

// --- next_authority: whose command write() sends ---

TEST(NextAuthority, SendsNoControllerCommandUntilTheOwnerHasHadACycle)
{
  auto authority = CommandAuthority::kAwaitingClaim;
  authority = next_authority(authority, false);
  EXPECT_EQ(authority, CommandAuthority::kAwaitingClaim);
  authority = next_authority(authority, true);
  EXPECT_EQ(authority, CommandAuthority::kClaimed);
  authority = next_authority(authority, true);
  EXPECT_EQ(authority, CommandAuthority::kController);
}

TEST(NextAuthority, ReleaseWhileCommandingLatchesProtective)
{
  auto authority = next_authority(CommandAuthority::kController, false);
  EXPECT_EQ(authority, CommandAuthority::kProtective);
  // A new claim does not rearm: ADR-002 moves toward lower energy until an explicit rearm.
  authority = next_authority(authority, true);
  EXPECT_EQ(authority, CommandAuthority::kProtective);
}

TEST(NextAuthority, ReleaseBeforeCommandingIsNotCommandLoss)
{
  // A controller that fails activation is switched back out before it ever commanded.
  EXPECT_EQ(
    next_authority(CommandAuthority::kClaimed, false), CommandAuthority::kAwaitingClaim);
}

}  // namespace
