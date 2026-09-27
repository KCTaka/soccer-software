// Copyright 2026 UTRA-RoboSoccer
// Reference buffer for the 200 Hz control loop.
//
// Producer: non-real-time thread (trajectory player, Zenoh callback).
// Consumer: the 200 Hz update() loop.
//
// The lock-free triple-buffer mechanics live in
// humanoid_transport::LatestValueBuffer, shared with other SPSC producer/
// consumer handoffs (e.g. humanoid_transport_mujoco's ground-truth buffer).
// This class adds the reference-specific convenience accessors.
#ifndef HUMANOID_MIT_CONTROLLER__REFERENCE_BUFFER_HPP_
#define HUMANOID_MIT_CONTROLLER__REFERENCE_BUFFER_HPP_

#include <chrono>
#include <cstdint>

#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/latest_value_buffer.hpp"

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

/// Thin wrapper over humanoid_transport::LatestValueBuffer<ReferenceFrame>.
/// Producer is non-real-time; publish()/read() are safe to call from the
/// real-time update() loop regardless.
class ReferenceBuffer
{
public:
  ReferenceBuffer() = default;

  /// Non-real-time. Called by the producer.
  void publish(const ReferenceFrame & frame) noexcept
  {
    buffer_.publish(frame);
  }

  /// Real-time. Called inside update(). Never blocks, never allocates.
  /// Returns false if no valid reference has been published yet.
  [[nodiscard]] bool read(ReferenceFrame & out) noexcept
  {
    return buffer_.read(out);
  }

  /// Real-time, consumer side only. Returns the sequence of the latest
  /// published reference, or 0 if none.
  [[nodiscard]] transport::CycleSequence latest_sequence() noexcept
  {
    ReferenceFrame frame;
    static_cast<void>(buffer_.read(frame));
    return frame.sequence;
  }

  /// Consumer side only. Discards any published reference, so read()
  /// returns false until the next publish().
  void reset() noexcept
  {
    buffer_.reset();
  }

private:
  transport::LatestValueBuffer<ReferenceFrame> buffer_;
};

}  // namespace humanoid::control

#endif  // HUMANOID_MIT_CONTROLLER__REFERENCE_BUFFER_HPP_
