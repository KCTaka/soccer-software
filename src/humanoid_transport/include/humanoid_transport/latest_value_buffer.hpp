// Copyright 2026 UTRA-RoboSoccer
// Lock-free single-producer single-consumer latest-value buffer.
//
// Semantics: the consumer always reads the most recently published frame.
// A partially written frame is never observed. No allocation, no syscall,
// no mutex on either the publish or read path.
//
// FrameT must have a `bool valid` member: a default-constructed FrameT is
// expected to read as invalid until the first publish().
#ifndef HUMANOID_TRANSPORT__LATEST_VALUE_BUFFER_HPP_
#define HUMANOID_TRANSPORT__LATEST_VALUE_BUFFER_HPP_

#include <atomic>
#include <concepts>  // NOLINT(build/include_order)
#include <cstddef>

namespace humanoid::transport
{

template<typename FrameT>
concept LatestValueFrame = requires(const FrameT & frame) {
  {frame.valid}->std::convertible_to<bool>;
};  // NOLINT(readability/braces)

/// Double-buffered latest-value SPSC.
/// The producer writes to the back buffer, then atomically swaps the index.
/// The consumer reads whichever buffer the index points to.
/// Because there is exactly one producer and one consumer, and the swap is
/// a single atomic store, the consumer never observes a torn write.
template<LatestValueFrame FrameT>
class LatestValueBuffer
{
public:
  LatestValueBuffer() = default;

  /// Called by the producer. May be real-time or non-real-time depending on
  /// which side of a given buffer is designated the producer.
  void publish(const FrameT & frame) noexcept
  {
    const std::size_t back = 1U - index_.load(std::memory_order_relaxed);
    slots_[back] = frame;
    // Release: the frame write is visible before the index swap.
    index_.store(back, std::memory_order_release);
  }

  /// Called by the consumer. Never blocks, never allocates.
  /// Returns false if no valid frame has been published yet.
  [[nodiscard]] bool read(FrameT & out) const noexcept
  {
    // Acquire: see the frame that was published before the index swap.
    const std::size_t front = index_.load(std::memory_order_acquire);
    out = slots_[front];
    return out.valid;
  }

  void reset() noexcept
  {
    slots_[0] = FrameT{};
    slots_[1] = FrameT{};
    index_.store(0U, std::memory_order_relaxed);
  }

private:
  FrameT slots_[2]{};
  std::atomic<std::size_t> index_{0U};
};

}  // namespace humanoid::transport

#endif  // HUMANOID_TRANSPORT__LATEST_VALUE_BUFFER_HPP_
