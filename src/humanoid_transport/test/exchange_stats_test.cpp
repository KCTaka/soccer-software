// Copyright 2026 UTRA-RoboSoccer

#include <gtest/gtest.h>

#include <chrono>

#include "humanoid_transport/exchange_stats.hpp"

namespace
{

using humanoid::transport::ExchangeResult;
using humanoid::transport::ExchangeStats;
using humanoid::transport::TransportError;
using std::chrono::nanoseconds;

TEST(ExchangeStats, StartsZeroed)
{
  constexpr ExchangeStats stats{};
  EXPECT_EQ(stats.attempted, 0U);
  EXPECT_EQ(stats.failed, 0U);
  EXPECT_EQ(stats.deadline_misses, 0U);
  EXPECT_EQ(stats.last_sequence, 0U);
  EXPECT_EQ(stats.worst_exchange, nanoseconds{0});
}

TEST(ExchangeStats, CountsAttemptsFailuresAndMisses)
{
  ExchangeStats stats;
  const ExchangeResult ok{};
  const ExchangeResult failed{.error = TransportError::kTimeout};

  stats.record(1U, ok, nanoseconds{100}, false);
  stats.record(2U, failed, nanoseconds{100}, true);
  stats.record(3U, ok, nanoseconds{100}, true);

  EXPECT_EQ(stats.attempted, 3U);
  EXPECT_EQ(stats.failed, 1U);
  EXPECT_EQ(stats.deadline_misses, 2U);
  EXPECT_EQ(stats.last_sequence, 3U);
}

TEST(ExchangeStats, KeepsWorstExchangeIncludingFailures)
{
  ExchangeStats stats;
  stats.record(1U, ExchangeResult{}, nanoseconds{300}, false);
  stats.record(2U, ExchangeResult{.error = TransportError::kNotActive}, nanoseconds{900}, false);
  stats.record(3U, ExchangeResult{}, nanoseconds{500}, false);
  EXPECT_EQ(stats.worst_exchange, nanoseconds{900});
}

TEST(ExchangeStats, IsUsableInConstantExpressions)
{
  constexpr ExchangeStats stats = [] {
      ExchangeStats s;
      s.record(7U, ExchangeResult{.error = TransportError::kFraming}, nanoseconds{5}, true);
      return s;
    }();
  static_assert(stats.attempted == 1U && stats.failed == 1U && stats.deadline_misses == 1U);
  EXPECT_EQ(stats.last_sequence, 7U);
}

}  // namespace
