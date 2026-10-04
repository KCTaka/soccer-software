// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_stm32/master_link.hpp"

#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <format>

#include <array>
#include <cerrno>
#include <cstring>

namespace humanoid::transport_stm32
{

namespace
{

// Writes one count to an eventfd, waking whoever polls it.
void signal(const UniqueFd & fd) noexcept;

// Clears an eventfd's counter so the next poll waits for a new signal.
void drain(const UniqueFd & fd) noexcept;

}  // namespace

MasterLink::~MasterLink()
{
  close();
}

// ===========================================================================
// Non-real-time
// ===========================================================================

std::optional<std::string> MasterLink::open(const std::string & device)
{
  close();

  frame_wake_.reset(::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC));
  stop_wake_.reset(::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC));
  if (!frame_wake_.valid() || !stop_wake_.valid()) {
    return std::format("cannot create an eventfd: {}", std::strerror(errno));
  }
  if (auto error = port_.open(device)) {
    return error;
  }

  // Nothing else is running yet, so this thread may act as the buffer's consumer.
  rt_frames_.reset();
  {
    const std::scoped_lock lock{mutex_};
    status_.reset();
    latest_.reset();
    frame_count_ = 0;
  }
  scanner_ = wire::FrameScanner{};
  have_previous_ = false;
  malformed_payloads_ = 0;
  framing_errors_.store(0, std::memory_order_relaxed);
  duplicate_frames_.store(0, std::memory_order_relaxed);
  master_reset_seen_.store(false, std::memory_order_relaxed);
  failed_.store(false, std::memory_order_relaxed);
  foreign_version_.store(0, std::memory_order_relaxed);

  reader_ = std::thread([this] {run();});
  return std::nullopt;
}

void MasterLink::close() noexcept
{
  if (reader_.joinable()) {
    signal(stop_wake_);
    reader_.join();
  }
  port_.close();
  frame_wake_.reset();
  stop_wake_.reset();
}

std::optional<wire::MasterStatus> MasterLink::wait_for_status(std::chrono::milliseconds timeout)
{
  std::unique_lock lock{mutex_};
  changed_.wait_for(lock, timeout, [this] {return status_.has_value() || failed();});
  return status_;
}

std::uint64_t MasterLink::frame_count() const
{
  const std::scoped_lock lock{mutex_};
  return frame_count_;
}

std::optional<TelemetryFrame> MasterLink::wait_for_frame(
  std::uint64_t after, std::chrono::milliseconds timeout)
{
  std::unique_lock lock{mutex_};
  changed_.wait_for(lock, timeout, [this, after] {return frame_count_ > after || failed();});
  if (frame_count_ > after) {
    return latest_;
  }
  return std::nullopt;
}

// ===========================================================================
// Real-time
// ===========================================================================

WriteStatus MasterLink::send(std::span<const std::uint8_t> frame, MonotonicStamp deadline) noexcept
{
  const WriteStatus status = port_.write_all(frame, deadline);
  if (status == WriteStatus::kError) {
    failed_.store(true, std::memory_order_release);
  }
  return status;
}

std::optional<TelemetryFrame> MasterLink::wait_for_newer(
  std::uint16_t after_cycle, MonotonicStamp deadline) noexcept
{
  for (;; ) {
    if (const auto frame = rt_frames_.read()) {
      // Signed 16-bit difference: correct across the cycle counter's wrap.
      if (static_cast<std::int16_t>(frame->robot.header.cycle_id - after_cycle) > 0) {
        return frame;
      }
    }
    if (failed()) {
      return std::nullopt;
    }

    const auto remaining = deadline - std::chrono::steady_clock::now();
    if (remaining <= std::chrono::nanoseconds::zero()) {
      return std::nullopt;
    }
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(remaining);
    timespec timeout{};
    timeout.tv_sec = static_cast<time_t>(ns.count() / 1'000'000'000);
    timeout.tv_nsec = static_cast<long>(ns.count() % 1'000'000'000);  // NOLINT(runtime/int)

    // Sleeps until the reader thread publishes a frame or the deadline passes, whichever is first.
    pollfd wake{frame_wake_.get(), POLLIN, 0};
    static_cast<void>(::ppoll(&wake, 1, &timeout, nullptr));
    drain(frame_wake_);
  }
}

// ===========================================================================
// Any thread
// ===========================================================================

MasterLink::Counters MasterLink::counters() const noexcept
{
  return {
    framing_errors_.load(std::memory_order_relaxed),
    duplicate_frames_.load(std::memory_order_relaxed)};
}

// ===========================================================================
// Reader thread
// ===========================================================================

void MasterLink::run()
{
  std::array<std::uint8_t, 512> chunk{};
  pollfd fds[2] = {{port_.fd(), POLLIN, 0}, {stop_wake_.get(), POLLIN, 0}};

  for (;; ) {
    fds[0].revents = 0;
    fds[1].revents = 0;
    if (::poll(fds, 2, -1) < 0) {
      if (errno == EINTR) {
        continue;
      }
      failed_.store(true, std::memory_order_release);
      break;
    }
    if ((fds[1].revents & POLLIN) != 0) {
      break;
    }

    // Drain everything waiting. POLLHUP can arrive together with the last bytes, so the read
    // result, not the poll flags, decides whether the port failed.
    bool port_failed = false;
    for (;; ) {
      const ReadResult read = port_.read_some(chunk);
      if (read.failed) {
        port_failed = true;
        break;
      }
      if (read.count == 0) {
        break;
      }

      std::span<const std::uint8_t> rest{chunk.data(), read.count};
      while (!rest.empty()) {
        const std::size_t accepted = scanner_.push(rest);
        rest = rest.subspan(accepted);
        while (const auto frame = scanner_.next()) {
          handle_frame(*frame);
        }
        if (accepted == 0) {
          break;
        }
      }
    }

    framing_errors_.store(
      scanner_.resync_events() + scanner_.foreign_version_frames() + malformed_payloads_,
      std::memory_order_relaxed);
    foreign_version_.store(scanner_.last_foreign_version(), std::memory_order_relaxed);

    if (port_failed) {
      failed_.store(true, std::memory_order_release);
      break;
    }
  }

  // Wake anyone waiting, so they see the failure instead of sleeping to their deadline.
  signal(frame_wake_);
  const std::scoped_lock lock{mutex_};
  changed_.notify_all();
}

void MasterLink::handle_frame(const wire::FrameScanner::Frame & frame)
{
  switch (frame.type) {
    case wire::MessageType::kMasterStatus:
      if (const auto status = wire::parse_master_status(frame.payload)) {
        const std::scoped_lock lock{mutex_};
        status_ = *status;
        changed_.notify_all();
      } else {
        ++malformed_payloads_;
      }
      break;
    case wire::MessageType::kRobotTele:
      if (const auto robot = wire::parse_robot_tele(frame.payload)) {
        handle_telemetry(*robot);
      } else {
        ++malformed_payloads_;
      }
      break;
    case wire::MessageType::kPing:
    case wire::MessageType::kSlaveStatus:
    case wire::MessageType::kRobotCmd:
      break;  // nothing here reads them
  }
}

void MasterLink::handle_telemetry(const wire::RobotTelemetry & robot)
{
  const std::uint16_t cycle = robot.header.cycle_id;
  const std::uint32_t master_time_us = robot.header.master_time_us;

  if (have_previous_) {
    // Signed differences are correct across the wrap of the 16-bit cycle and the 32-bit clock
    // (about 71 minutes). The USB stream preserves order, so a step backwards is not reordering:
    // it is the master having restarted.
    const auto cycle_step = static_cast<std::int16_t>(cycle - previous_cycle_);
    const auto time_step = static_cast<std::int32_t>(master_time_us - previous_master_time_us_);
    if (cycle_step < 0 || time_step < 0) {
      master_reset_seen_.store(true, std::memory_order_release);
    } else if (cycle_step == 0) {
      duplicate_frames_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
  }
  have_previous_ = true;
  previous_cycle_ = cycle;
  previous_master_time_us_ = master_time_us;

  TelemetryFrame frame;
  frame.valid = true;
  frame.received = std::chrono::steady_clock::now();
  frame.robot = robot;

  rt_frames_.publish(frame);
  {
    const std::scoped_lock lock{mutex_};
    latest_ = frame;
    ++frame_count_;
    changed_.notify_all();
  }
  signal(frame_wake_);
}

namespace
{

void signal(const UniqueFd & fd) noexcept
{
  std::uint64_t one = 1;
  for (;; ) {
    const ssize_t res = ::write(fd.get(), &one, sizeof(one));
    if (res == static_cast<ssize_t>(sizeof(one))) {
      return;
    }
    if (res < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      return;
    }
    return;
  }
}

void drain(const UniqueFd & fd) noexcept
{
  std::uint64_t count = 0;
  for (;; ) {
    const ssize_t res = ::read(fd.get(), &count, sizeof(count));
    if (res == static_cast<ssize_t>(sizeof(count))) {
      return;
    }
    if (res < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      return;
    }
    return;
  }
}

}  // namespace

}  // namespace humanoid::transport_stm32
