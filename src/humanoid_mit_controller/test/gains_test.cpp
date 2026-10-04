// Copyright 2026 UTRA-RoboSoccer

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <variant>
#include <vector>

#include "humanoid_mit_controller/gains.hpp"

namespace
{

using humanoid::control::resolve_gains;

TEST(Gains, EmptyListGivesEveryJointTheFallback)
{
  const auto gains = resolve_gains("kp", {}, 20.0, 3);
  ASSERT_TRUE(std::holds_alternative<std::vector<double>>(gains));
  EXPECT_EQ(std::get<std::vector<double>>(gains), (std::vector<double>{20.0, 20.0, 20.0}));
}

TEST(Gains, ListIsUsedAsGivenAndTheFallbackIgnored)
{
  const auto gains = resolve_gains("kd", {1.0, 2.0, 4.0}, 99.0, 3);
  ASSERT_TRUE(std::holds_alternative<std::vector<double>>(gains));
  EXPECT_EQ(std::get<std::vector<double>>(gains), (std::vector<double>{1.0, 2.0, 4.0}));
}

TEST(Gains, ShortOrLongListIsRejected)
{
  EXPECT_TRUE(std::holds_alternative<std::string>(resolve_gains("kp", {1.0, 2.0}, 0.0, 3)));
  EXPECT_TRUE(std::holds_alternative<std::string>(resolve_gains("kp", {1.0, 2.0, 3.0, 4.0}, 0.0,
      3)));
}

TEST(Gains, NegativeOrNonFiniteIsRejected)
{
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(std::holds_alternative<std::string>(resolve_gains("kp", {1.0, -1.0}, 0.0, 2)));
  EXPECT_TRUE(std::holds_alternative<std::string>(resolve_gains("kp", {1.0, kNan}, 0.0, 2)));
  EXPECT_TRUE(std::holds_alternative<std::string>(resolve_gains("kp", {}, -5.0, 2)));
}

TEST(Gains, ErrorNamesTheOffendingEntry)
{
  const auto gains = resolve_gains("kd", {1.0, -1.0}, 0.0, 2);
  ASSERT_TRUE(std::holds_alternative<std::string>(gains));
  EXPECT_NE(std::get<std::string>(gains).find("'kd'[1]"), std::string::npos);
}

}  // namespace
