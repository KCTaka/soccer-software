// Copyright 2026 UTRA-RoboSoccer
// gmock matchers for the workspace's "a value, or why not" results: std::variant<T, std::string>,
// as make_layout(), drive_layout(), parse_safety_manifest() and resolve_gains() return. For the
// std::optional<std::string> "why not, if anything" results, use gmock's own
// Optional(HasSubstr(...)) and Eq(std::nullopt).
//
//   EXPECT_THAT(make_layout(joints), HoldsValue());
//   EXPECT_THAT(make_layout(joints), HoldsError(HasSubstr("already wired")));
//
// A failure prints the error the result holds, which EXPECT_TRUE(holds_alternative<...>) cannot.
#ifndef HUMANOID_TEST_SUPPORT__MATCHERS_HPP_
#define HUMANOID_TEST_SUPPORT__MATCHERS_HPP_

#include <gmock/gmock.h>

#include <string>
#include <variant>

namespace humanoid::test_support
{

/// The result holds a value, not an error.
MATCHER(HoldsValue, negation ? "holds an error" : "holds a value, not an error")
{
  if (const auto * error = std::get_if<std::string>(&arg)) {
    *result_listener << "holds the error \"" << *error << "\"";
    return false;
  }
  return true;
}

/// The result holds an error that `matcher` matches (a gmock matcher, or a string it equals).
MATCHER_P(
  HoldsError, matcher,
  std::string(negation ? "does not hold" : "holds") + " an error that " +
  ::testing::DescribeMatcher<std::string>(matcher))
{
  const auto * error = std::get_if<std::string>(&arg);
  if (error == nullptr) {
    *result_listener << "holds a value, not an error";
    return false;
  }
  *result_listener << "holds the error \"" << *error << "\"";
  return ::testing::ExplainMatchResult(matcher, *error, result_listener);
}

}  // namespace humanoid::test_support

#endif  // HUMANOID_TEST_SUPPORT__MATCHERS_HPP_
