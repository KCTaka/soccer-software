// Copyright 2026 UTRA-RoboSoccer
// A non-blocking POSIX serial port: the USB CDC device of the master STM32.
//
// open() and close() are non-real-time. write_all() and read_some() are bounded and never allocate,
// so exchange() may call them (ADR-001).
#ifndef HUMANOID_TRANSPORT_STM32__SERIAL_PORT_HPP_
#define HUMANOID_TRANSPORT_STM32__SERIAL_PORT_HPP_

#include <span>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "humanoid_transport/batch_types.hpp"

namespace humanoid::transport_stm32
{

using transport::MonotonicStamp;

/// Owns a file descriptor and closes it. Movable, not copyable.
class UniqueFd
{
public:
  UniqueFd() = default;
  explicit UniqueFd(int fd) noexcept
  : fd_(fd) {}
  ~UniqueFd() {reset();}

  UniqueFd(const UniqueFd &) = delete;
  UniqueFd & operator=(const UniqueFd &) = delete;
  UniqueFd(UniqueFd && other) noexcept
  : fd_(other.release()) {}
  UniqueFd & operator=(UniqueFd && other) noexcept
  {
    if (this != &other) {
      reset(other.release());
    }
    return *this;
  }

  [[nodiscard]] int get() const noexcept {return fd_;}
  [[nodiscard]] bool valid() const noexcept {return fd_ >= 0;}
  int release() noexcept
  {
    const int fd = fd_;
    fd_ = -1;
    return fd;
  }
  void reset(int fd = -1) noexcept;

private:
  int fd_{-1};
};

/// Outcome of a bounded write.
enum class WriteStatus : std::uint8_t
{
  kOk,
  /// The deadline passed with bytes still unwritten. Part of a frame may have gone out; the
  /// master's resynchronising scanner discards it.
  kTimeout,
  kError,
};

struct ReadResult
{
  std::size_t count{0};
  /// The port is gone (unplugged, hung up) or failed. Zero bytes with failed == false just means
  /// nothing was waiting.
  bool failed{false};
};

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort() = default;
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;
  SerialPort(SerialPort &&) = delete;
  SerialPort & operator=(SerialPort &&) = delete;

  /// Opens `device` for exclusive raw access and discards anything already buffered, so a stale
  /// burst from before the open is not mistaken for live data. Returns the error, or nothing.
  [[nodiscard]] std::optional<std::string> open(const std::string & device);

  /// Releases the exclusive lock and closes. Safe to call twice.
  void close() noexcept;
  [[nodiscard]] bool is_open() const noexcept {return fd_.valid();}
  [[nodiscard]] int fd() const noexcept {return fd_.get();}

  /// Writes all of `bytes`, waiting for buffer space only until `deadline`.
  [[nodiscard]] WriteStatus write_all(
    std::span<const std::uint8_t> bytes, MonotonicStamp deadline) noexcept;

  /// Reads what is waiting, up to buffer.size(). Never blocks.
  [[nodiscard]] ReadResult read_some(std::span<std::uint8_t> buffer) noexcept;

private:
  UniqueFd fd_;
};

}  // namespace humanoid::transport_stm32

#endif  // HUMANOID_TRANSPORT_STM32__SERIAL_PORT_HPP_
