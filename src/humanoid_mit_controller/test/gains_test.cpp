// Copyright 2026 UTRA-RoboSoccer

#include <gmock/gmock.h>

#include <limits>
#include <string>
#include <variant>
#include <vector>

#include "humanoid_mit_controller/gains.hpp"
#include "humanoid_test_support/matchers.hpp"

namespace
{

using humanoid::control::resolve_gains;
using humanoid::test_support::HoldsError;
using ::testing::_;
using ::testing::HasSubstr;
using Gains = std::vector<double>;
using humanoid::control::GainsOrError;

TEST(Gains, EmptyListGivesEveryJointTheFallback)
{
  EXPECT_EQ(resolve_gains("kp", {}, 20.0, 3), (GainsOrError{Gains(3, 20.0)}));
}

TEST(Gains, ListIsUsedAsGivenAndTheFallbackIgnored)
{
  const Gains given{1.0, 2.0, 4.0};
  EXPECT_EQ(resolve_gains("kd", given, 99.0, 3), (GainsOrError{given}));
}

TEST(Gains, ShortOrLongListIsRejected)
{
  EXPECT_THAT(resolve_gains("kp", {1.0, 2.0}, 0.0, 3), HoldsError(_));
  EXPECT_THAT(resolve_gains("kp", {1.0, 2.0, 3.0, 4.0}, 0.0, 3), HoldsError(_));
}

TEST(Gains, NegativeOrNonFiniteIsRejected)
{
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THAT(resolve_gains("kp", {1.0, -1.0}, 0.0, 2), HoldsError(_));
  EXPECT_THAT(resolve_gains("kp", {1.0, kNan}, 0.0, 2), HoldsError(_));
  EXPECT_THAT(resolve_gains("kp", {}, -5.0, 2), HoldsError(_));
}

TEST(Gains, ErrorNamesTheOffendingEntry)
{
  EXPECT_THAT(resolve_gains("kd", {1.0, -1.0}, 0.0, 2), HoldsError(HasSubstr("'kd'[1]")));
}

}  // namespace
