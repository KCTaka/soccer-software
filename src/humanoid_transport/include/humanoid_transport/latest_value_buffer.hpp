// Copyright 2026 UTRA-RoboSoccer
// Lock-free single-producer single-consumer latest-value buffer.
//
// Semantics: the consumer always reads the most recently published frame.
// A partially written frame is never observed. No allocation, no syscall,
// no mutex on either the publish or read path.
//
// FrameT must have a `bool valid` member: a default-constructed FrameT is
// expected to read as invalid until the first publish(). FrameT must also be
// trivially copyable, so a slot copy is a plain memcpy that cannot allocate
// or throw.
#ifndef HUMANOID_TRANSPORT__LATEST_VALUE_BUFFER_HPP_
#define HUMANOID_TRANSPORT__LATEST_VALUE_BUFFER_HPP_

#include <atomic>
#include <concepts>  // NOLINT(build/include_order)
#include <cstdint>
#include <optional>
#include <type_traits>

namespace humanoid::transport
{

template<typename FrameT>
concept LatestValueFrame = requires(const FrameT & frame) {
  {frame.valid}->std::convertible_to<bool>;
  requires std::is_trivially_copyable_v<FrameT>;
};  // NOLINT(readability/braces)

/// Triple-buffered latest-value SPSC.
///
/// Three slots are partitioned between three owners at all times:
///   back_   - owned by the producer, the slot it writes next;
///   front_  - owned by the consumer, the slot it copies from;
///   middle_ - the hand-off slot, owned by neither.
/// publish() writes the back slot, then atomically exchanges it with the
/// middle slot and sets kFresh. read() exchanges its front slot with the
/// middle slot only when kFresh is set, so repeated reads keep returning the
/// latest frame instead of swapping an older one back in.
///
/// Because neither side ever touches a slot the other side owns, the consumer
/// never observes a torn write, however long a copy is preempted. (A double
/// buffer cannot give this guarantee: two publishes during one read cycle the
/// producer back onto the slot being read.)
template<LatestValueFrame FrameT>
class LatestValueBuffer
{
public:
  LatestValueBuffer() = default;

  /// Called by the producer only. May be real-time or non-real-time depending
  /// on which side of a given buffer is designated the producer.
  void publish(const FrameT & frame) noexcept
  {
    slots_[back_] = frame;
    // Release: the slot write is visible to the consumer that takes this
    // index. Acquire: the consumer has finished reading the slot we get back.
    const std::uint32_t previous =
      middle_.exchange(back_ | kFresh, std::memory_order_acq_rel);
    back_ = previous & kIndexMask;
  }

  /// Called by the consumer only. Never blocks, never allocates.
  /// Returns the latest frame, or nullopt if no valid frame has been
  /// published yet (or since reset()).
  ///
  /// Returns by value (C++ Core Guidelines F.20). A caller initialising a
  /// fresh object gets the slot copied straight into it: the prvalue is
  /// guaranteed-elided and, under the x86-64/AArch64 ABIs, the hidden return
  /// pointer *is* the caller's object. No frame is copied on the empty path.
  /// std::optional of a trivially copyable FrameT is itself trivially
  /// copyable, so this stays allocation-free and real-time safe.
  [[nodiscard]] std::optional<FrameT> read() noexcept
  {
    take_latest();
    if (!slots_[front_].valid) {
      return std::nullopt;
    }
    return slots_[front_];
  }

  /// Called by the consumer only. Discards every frame published so far, so
  /// read() returns nullopt until the next publish(). A publish() running
  /// concurrently with reset() may land either side of it.
  void reset() noexcept
  {
    take_latest();
    // The middle slot is not fresh, and the back slot is overwritten before
    // it is handed off, so the front slot is the only pre-reset frame that
    // read() could still return.
    slots_[front_] = FrameT{};
  }

private:
  static constexpr std::uint32_t kIndexMask = 0x3U;
  static constexpr std::uint32_t kFresh = 0x4U;

  static_assert(std::atomic<std::uint32_t>::is_always_lock_free);

  /// Consumer side: swap the fresh middle slot into front_, if there is one.
  void take_latest() noexcept
  {
    // Only the producer sets kFresh and only the consumer clears it, so a
    // relaxed check is safe; the exchange supplies the synchronisation.
    if ((middle_.load(std::memory_order_relaxed) & kFresh) != 0U) {
      const std::uint32_t previous =
        middle_.exchange(front_, std::memory_order_acq_rel);
      front_ = previous & kIndexMask;
    }
  }

  FrameT slots_[3]{};
  std::atomic<std::uint32_t> middle_{0U};
  std::uint32_t back_{1U};   // producer-owned
  std::uint32_t front_{2U};  // consumer-owned
};

}  // namespace humanoid::transport

#endif  // HUMANOID_TRANSPORT__LATEST_VALUE_BUFFER_HPP_
