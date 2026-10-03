// Copyright 2026 UTRA-RoboSoccer
// The link to the master STM32: one serial port, one reader thread that decodes what arrives and
// hands it to the real-time exchange().
//
// Threads. The reader thread is the only one that reads the port, and it is not real-time. The
// real-time thread writes commands with send() and receives telemetry with wait_for_newer(); the
// two meet only through a LatestValueBuffer and an eventfd, so neither side allocates, locks, or
// waits without a bound (ADR-001). The lifecycle thread (configure, activate, deactivate) uses the
// mutex-guarded waits below. send() has a single caller at a time: the lifecycle thread while the
// transport is not active, the real-time thread while it is.
#ifndef HUMANOID_TRANSPORT_STM32__MASTER_LINK_HPP_
#define HUMANOID_TRANSPORT_STM32__MASTER_LINK_HPP_

#include <span>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>

#include "humanoid_transport/latest_value_buffer.hpp"
#include "humanoid_transport_stm32/serial_port.hpp"
#include "humanoid_transport_stm32/wire_protocol.hpp"

namespace humanoid::transport_stm32
{

/// One telemetry frame and the host time it arrived.
struct TelemetryFrame
{
  bool valid{false};
  MonotonicStamp received{};
  wire::RobotTelemetry robot{};
};
static_assert(std::is_trivially_copyable_v<TelemetryFrame>);

class MasterLink
{
public:
  MasterLink() = default;
  ~MasterLink();

  MasterLink(const MasterLink &) = delete;
  MasterLink & operator=(const MasterLink &) = delete;
  MasterLink(MasterLink &&) = delete;
  MasterLink & operator=(MasterLink &&) = delete;

  // --- Non-real-time ---

  /// Opens `device` and starts the reader thread. Returns the error, or nothing.
  [[nodiscard]] std::optional<std::string> open(const std::string & device);

  /// Stops the reader thread and closes the port. Safe to call twice.
  void close() noexcept;

  /// The most recent MASTER_STATUS (20 Hz), waiting up to `timeout` for the first.
  [[nodiscard]] std::optional<wire::MasterStatus> wait_for_status(
    std::chrono::milliseconds timeout);

  /// Telemetry frames accepted since open().
  [[nodiscard]] std::uint64_t frame_count() const;

  /// The newest frame accepted after the `after`-th, waiting up to `timeout`.
  [[nodiscard]] std::optional<TelemetryFrame> wait_for_frame(
    std::uint64_t after, std::chrono::milliseconds timeout);

  // --- Real-time ---

  /// Writes one complete frame, waiting for buffer space only until `deadline`.
  [[nodiscard]] WriteStatus send(
    std::span<const std::uint8_t> frame, MonotonicStamp deadline) noexcept;

  /// The newest frame whose cycle_id is ahead of `after_cycle`, or nothing by `deadline`.
  /// Frames are consumed newest-first: a caller that fell behind skips the ones in between.
  [[nodiscard]] std::optional<TelemetryFrame> wait_for_newer(
    std::uint16_t after_cycle, MonotonicStamp deadline) noexcept;

  // --- Any thread ---

  struct Counters
  {
    /// Runs of bytes dropped while resynchronising, frames of another protocol version, and
    /// frames whose payload did not decode.
    std::uint64_t framing_errors{0};
    /// Telemetry frames repeating the cycle of the one before.
    std::uint64_t duplicate_frames{0};
  };
  [[nodiscard]] Counters counters() const noexcept;

  /// The master's cycle counter or clock ran backwards: it reset (brownout, watchdog, reflash).
  /// ADR-002: a reset STM32 boots disarmed and needs a fresh handshake, so the transport treats
  /// this as a fault until the next activation acknowledges it.
  [[nodiscard]] bool master_reset_seen() const noexcept
  {
    return master_reset_seen_.load(std::memory_order_acquire);
  }
  void acknowledge_master_reset() noexcept
  {
    master_reset_seen_.store(false, std::memory_order_release);
  }

  /// The port failed or went away (unplugged, hung up).
  [[nodiscard]] bool failed() const noexcept {return failed_.load(std::memory_order_acquire);}

  /// Protocol version of the most recent frame from a master on another version, or 0. Lets
  /// configure() say "the master speaks version 7" instead of just "no telemetry".
  [[nodiscard]] std::uint8_t foreign_version() const noexcept
  {
    return foreign_version_.load(std::memory_order_relaxed);
  }

private:
  void run();
  void handle_frame(const wire::FrameScanner::Frame & frame);
  void handle_telemetry(const wire::RobotTelemetry & robot);

  SerialPort port_;
  UniqueFd frame_wake_;  // eventfd: written per published frame, read by wait_for_newer()
  UniqueFd stop_wake_;   // eventfd: written to stop the reader thread
  std::thread reader_;

  // The real-time hand-off. Written by the reader thread, read by the real-time thread.
  transport::LatestValueBuffer<TelemetryFrame> rt_frames_;

  // The lifecycle hand-off, under mutex_.
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  std::optional<wire::MasterStatus> status_;
  std::optional<TelemetryFrame> latest_;
  std::uint64_t frame_count_{0};

  // Reader-thread-only: the byte-stream scanner and the previous frame, to classify the next as a
  // duplicate or a reset.
  wire::FrameScanner scanner_;
  bool have_previous_{false};
  std::uint16_t previous_cycle_{0};
  std::uint32_t previous_master_time_us_{0};
  std::uint64_t malformed_payloads_{0};

  std::atomic<std::uint64_t> framing_errors_{0};
  std::atomic<std::uint64_t> duplicate_frames_{0};
  std::atomic<bool> master_reset_seen_{false};
  std::atomic<bool> failed_{false};
  std::atomic<std::uint8_t> foreign_version_{0};
};

}  // namespace humanoid::transport_stm32

#endif  // HUMANOID_TRANSPORT_STM32__MASTER_LINK_HPP_
