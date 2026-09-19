// Copyright 2026 Your Organization Name
// Lock-free single-producer single-consumer reference buffer.
//
// Producer: non-real-time thread (trajectory player, Zenoh callback).
// Consumer: the 200 Hz update() loop.
//
// Semantics: latest-value. The consumer always reads the most recent
// complete reference. A partially written reference is never observed.
// No allocation, no syscall, no mutex on the read path.
#ifndef HUMANOID_MIT_CONTROLLER__REFERENCE_BUFFER_HPP_
#define HUMANOID_MIT_CONTROLLER__REFERENCE_BUFFER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>

#include "humanoid_transport/batch_types.hpp"

namespace humanoid::control
{

/// A single reference frame: what the controller should command this cycle.
/// Reuses the transport JointCommand layout for the five MIT fields.
struct ReferenceFrame
{
  transport::CycleSequence sequence{0U};
  transport::MonotonicStamp stamp{};
  std::uint8_t joint_count{0U};
  bool valid{false};
  std::array<transport::JointCommand, transport::kMaxJoints> joints{};
};

/// Double-buffered latest-value SPSC.
/// The producer writes to the back buffer, then atomically swaps the index.
/// The consumer reads whichever buffer the index points to.
/// Because there is exactly one producer and one consumer, and the swap is
/// a single atomic store, the consumer never observes a torn write.
class ReferenceBuffer
{
public:
  ReferenceBuffer() = default;

  /// Non-real-time. Called by the producer.
  void publish(const ReferenceFrame & frame) noexcept
  {
    const std::size_t back = 1U - index_.load(std::memory_order_relaxed);
    slots_[back] = frame;
    // Release: the frame write is visible before the index swap.
    index_.store(back, std::memory_order_release);
  }

  /// Real-time. Called inside update(). Never blocks, never allocates.
  /// Returns false if no valid reference has been published yet.
  [[nodiscard]] bool read(ReferenceFrame & out) const noexcept
  {
    // Acquire: see the frame that was published before the index swap.
    const std::size_t front = index_.load(std::memory_order_acquire);
    out = slots_[front];
    return out.valid;
  }

  /// Real-time. Returns the sequence of the latest published reference,
  /// or 0 if none.
  [[nodiscard]] transport::CycleSequence latest_sequence() const noexcept
  {
    const std::size_t front = index_.load(std::memory_order_acquire);
    return slots_[front].sequence;
  }

  void reset() noexcept
  {
    slots_[0] = ReferenceFrame{};
    slots_[1] = ReferenceFrame{};
    index_.store(0U, std::memory_order_relaxed);
  }

private:
  ReferenceFrame slots_[2]{};
  std::atomic<std::size_t> index_{0U};
};

}  // namespace humanoid::control

#endif  // HUMANOID_MIT_CONTROLLER__REFERENCE_BUFFER_HPP_
