// Copyright 2026 UTRA-RoboSoccer
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
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
  TestFrame out;
  EXPECT_FALSE(buffer.read(out));
}

TEST(LatestValueBuffer, ReadsMostRecentPublish)
{
  LatestValueBuffer<TestFrame> buffer;
  for (std::uint64_t s = 1U; s <= 5U; ++s) {
    buffer.publish(make_frame(s));
  }
  TestFrame out;
  ASSERT_TRUE(buffer.read(out));
  EXPECT_EQ(out.words[0], 5U);
}

// Repeated reads with no intervening publish must keep returning the latest
// frame, never swap an older (or never-written) slot back in.
TEST(LatestValueBuffer, RepeatedReadsAreStable)
{
  LatestValueBuffer<TestFrame> buffer;
  buffer.publish(make_frame(1U));
  buffer.publish(make_frame(2U));
  for (int i = 0; i < 4; ++i) {
    TestFrame out;
    ASSERT_TRUE(buffer.read(out)) << "read " << i;
    EXPECT_EQ(out.words[0], 2U) << "read " << i;
  }
  buffer.publish(make_frame(3U));
  for (int i = 0; i < 4; ++i) {
    TestFrame out;
    ASSERT_TRUE(buffer.read(out)) << "read " << i;
    EXPECT_EQ(out.words[0], 3U) << "read " << i;
  }
}

TEST(LatestValueBuffer, ResetDiscardsPublishedFrames)
{
  LatestValueBuffer<TestFrame> buffer;
  TestFrame out;
  buffer.publish(make_frame(1U));
  ASSERT_TRUE(buffer.read(out));   // frame 1 now in the consumer slot
  buffer.publish(make_frame(2U));  // frame 2 pending in the middle slot
  buffer.reset();
  EXPECT_FALSE(buffer.read(out));
  EXPECT_FALSE(buffer.read(out));

  buffer.publish(make_frame(3U));
  ASSERT_TRUE(buffer.read(out));
  EXPECT_EQ(out.words[0], 3U);
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

  std::uint64_t last = 0U;
  std::uint64_t torn = 0U;
  std::uint64_t regressions = 0U;
  for (int i = 0; i < 200000; ++i) {
    TestFrame out;
    if (!buffer.read(out)) {
      continue;
    }
    for (const auto word : out.words) {
      if (word != out.words[0]) {
        ++torn;
        break;
      }
    }
    if (out.words[0] < last) {
      ++regressions;
    }
    last = out.words[0];
  }
  stop.store(true, std::memory_order_relaxed);
  producer.join();

  EXPECT_EQ(torn, 0U);
  EXPECT_EQ(regressions, 0U);
  EXPECT_GT(last, 0U);
}

}  // namespace
