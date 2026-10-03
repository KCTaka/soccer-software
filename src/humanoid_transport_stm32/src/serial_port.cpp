// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_stm32/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <format>

#include <cerrno>
#include <cstring>

namespace humanoid::transport_stm32
{

namespace
{

// Waits until `fd` is writable or `deadline` passes; false on timeout or error.
bool wait_writable(int fd, MonotonicStamp deadline) noexcept;

}  // namespace

void UniqueFd::reset(int fd) noexcept
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
  fd_ = fd;
}

std::optional<std::string> SerialPort::open(const std::string & device)
{
  fd_.reset(::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC));
  if (!fd_.valid()) {
    return std::format("cannot open '{}': {}", device, std::strerror(errno));
  }

  // Exclusive, so ModemManager or a second process cannot open the port and inject bytes that
  // the master would parse as host commands.
  if (::ioctl(fd_.get(), TIOCEXCL) != 0) {
    const auto error = std::format("cannot take '{}' exclusively: {}", device,
        std::strerror(errno));
    fd_.reset();
    return error;
  }

  termios tty{};
  if (::tcgetattr(fd_.get(), &tty) != 0) {
    const auto error = std::format("'{}' is not a serial device: {}", device, std::strerror(errno));
    fd_.reset();
    return error;
  }
  ::cfmakeraw(&tty);
  // VMIN 1: a read returns as soon as one byte is waiting, and with O_NONBLOCK fails with EAGAIN
  // when none is. VMIN 0 would make an empty read return 0, which is also what end-of-file looks
  // like, so a quiet line would be mistaken for a vanished device.
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;
  // CDC ignores the line speed; set one so the termios is fully defined.
  ::cfsetspeed(&tty, B115200);
  if (::tcsetattr(fd_.get(), TCSANOW, &tty) != 0) {
    const auto error = std::format("cannot configure '{}': {}", device, std::strerror(errno));
    fd_.reset();
    return error;
  }

  ::tcflush(fd_.get(), TCIOFLUSH);
  return std::nullopt;
}

void SerialPort::close() noexcept
{
  if (fd_.valid()) {
    // The exclusive flag lives on the terminal, which outlives this descriptor if another process
    // (or a test's pseudo-terminal) still holds it open, and would refuse the next open().
    ::ioctl(fd_.get(), TIOCNXCL);
  }
  fd_.reset();
}

WriteStatus SerialPort::write_all(
  std::span<const std::uint8_t> bytes, MonotonicStamp deadline) noexcept
{
  std::size_t written = 0;
  while (written < bytes.size()) {
    const ssize_t n = ::write(fd_.get(), bytes.data() + written, bytes.size() - written);
    if (n > 0) {
      written += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      if (!wait_writable(fd_.get(), deadline)) {
        return WriteStatus::kTimeout;
      }
      continue;
    }
    return WriteStatus::kError;
  }
  return WriteStatus::kOk;
}

ReadResult SerialPort::read_some(std::span<std::uint8_t> buffer) noexcept
{
  for (;; ) {
    const ssize_t n = ::read(fd_.get(), buffer.data(), buffer.size());
    if (n > 0) {
      return {static_cast<std::size_t>(n), false};
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return {0, false};
    }
    // n == 0 is end of file on a character device: the other end went away. EIO is how a pty or
    // an unplugged CDC device reports it.
    return {0, true};
  }
}

namespace
{

bool wait_writable(int fd, MonotonicStamp deadline) noexcept
{
  const auto remaining = deadline - std::chrono::steady_clock::now();
  if (remaining <= std::chrono::nanoseconds::zero()) {
    return false;
  }
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(remaining);
  timespec timeout{};
  timeout.tv_sec = static_cast<time_t>(ns.count() / 1'000'000'000);
  timeout.tv_nsec = static_cast<long>(ns.count() % 1'000'000'000);  // NOLINT(runtime/int)

  pollfd pfd{fd, POLLOUT, 0};
  const int ready = ::ppoll(&pfd, 1, &timeout, nullptr);
  return ready > 0 && (pfd.revents & POLLOUT) != 0;
}

}  // namespace

}  // namespace humanoid::transport_stm32
