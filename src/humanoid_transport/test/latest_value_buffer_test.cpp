// Copyright 2026 UTRA-RoboSoccer
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <thread>

#include "humanoid_transport/latest_value_buffer.hpp"

namespace
{

using humanoid::transport::LatestValueBuffer;

// Every word carries the same sequence number, so a torn copy is detectable.
struct TestFrame
{
  bool valid{false};
  std::uint64_t words[64]{};
};

TestFrame make_frame(std::uint64_t sequence)
{
  TestFrame frame;
  frame.valid = true;
  for (auto & word : frame.words) {
    word = sequence;
  }
  return frame;
}

TEST(LatestValueBuffer, ReadsInvalidBeforeFirstPublish)
{
  LatestValueBuffer<TestFrame> buffer;
  EXPECT_FALSE(buffer.read().has_value());
}

TEST(LatestValueBuffer, ReadsMostRecentPublish)
{
  LatestValueBuffer<TestFrame> buffer;
  for (std::uint64_t s = 1U; s <= 5U; ++s) {
    buffer.publish(make_frame(s));
  }
  const auto out = buffer.read();
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->words[0], 5U);
}

// Repeated reads with no intervening publish must keep returning the latest
// frame, never swap an older (or never-written) slot back in.
TEST(LatestValueBuffer, RepeatedReadsAreStable)
{
  LatestValueBuffer<TestFrame> buffer;
  buffer.publish(make_frame(1U));
  buffer.publish(make_frame(2U));
  for (int i = 0; i < 4; ++i) {
    const auto out = buffer.read();
    ASSERT_TRUE(out.has_value()) << "read " << i;
    EXPECT_EQ(out->words[0], 2U) << "read " << i;
  }
  buffer.publish(make_frame(3U));
  for (int i = 0; i < 4; ++i) {
    const auto out = buffer.read();
    ASSERT_TRUE(out.has_value()) << "read " << i;
    EXPECT_EQ(out->words[0], 3U) << "read " << i;
  }
}

TEST(LatestValueBuffer, ResetDiscardsPublishedFrames)
{
  LatestValueBuffer<TestFrame> buffer;
  buffer.publish(make_frame(1U));
  ASSERT_TRUE(buffer.read().has_value());  // frame 1 now in the consumer slot
  buffer.publish(make_frame(2U));          // frame 2 pending in the middle slot
  buffer.reset();
  EXPECT_FALSE(buffer.read().has_value());
  EXPECT_FALSE(buffer.read().has_value());

  buffer.publish(make_frame(3U));
  const auto out = buffer.read();
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->words[0], 3U);
}

// Producer publishes as fast as it can while the consumer reads: every frame
// read must be whole, and sequences must never go backwards.
TEST(LatestValueBuffer, ConcurrentReadsAreNeverTornOrStale)
{
  LatestValueBuffer<TestFrame> buffer;
  std::atomic<bool> stop{false};
  std::thread producer([&buffer, &stop] {
      for (std::uint64_t s = 1U; !stop.load(std::memory_order_relaxed); ++s) {
        buffer.publish(make_frame(s));
      }
    });

  // On a loaded machine the producer may not have run yet, and a fixed number of reads could all
  // find nothing. Wait for its first publish, so the reads below overlap publishing.
  const auto give_up = std::chrono::steady_clock::now() + std::chrono::seconds{5};
  std::optional<TestFrame> first;
  while (!first && std::chrono::steady_clock::now() < give_up) {
    first = buffer.read();
    std::this_thread::yield();
  }
  if (!first) {
    stop.store(true, std::memory_order_relaxed);
    producer.join();
    FAIL() << "the producer published nothing within 5 s";
  }

  // At least 200000 reads, and on until the producer has been seen to advance, so the reads are
  // known to have overlapped publishing.
  std::uint64_t last = first->words[0];
  std::uint64_t torn = 0U;
  std::uint64_t regressions = 0U;
  const auto read_until = std::chrono::steady_clock::now() + std::chrono::seconds{5};
  for (int i = 0; i < 200000 ||
    (last == first->words[0] && std::chrono::steady_clock::now() < read_until); ++i)
  {
    const auto out = buffer.read();
    if (!out) {
      continue;
    }
    for (const auto word : out->words) {
      if (word != out->words[0]) {
        ++torn;
        break;
      }
    }
    if (out->words[0] < last) {
      ++regressions;
    }
    last = out->words[0];
  }
  stop.store(true, std::memory_order_relaxed);
  producer.join();

  EXPECT_EQ(torn, 0U);
  EXPECT_EQ(regressions, 0U);
  EXPECT_GT(last, first->words[0]);  // the producer kept publishing while it was read
}

}  // namespace
