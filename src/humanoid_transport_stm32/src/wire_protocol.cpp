// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_transport_stm32/wire_protocol.hpp"

#include <numbers>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace humanoid::transport_stm32::wire
{

namespace
{

// File-local helpers. Declared here, defined at the end of this file, so the public functions
// read first.
bool is_known_type(std::uint16_t type) noexcept;
std::uint16_t frame_crc(const MsgHeader & header, std::span<const std::uint8_t> payload) noexcept;

constexpr std::uint16_t kCrcPoly = 0x1021U;

// Defined here, not at the bottom: a constant expression needs the function's definition.
constexpr std::array<std::uint16_t, 256> make_crc_table() noexcept
{
  std::array<std::uint16_t, 256> table{};
  for (std::size_t i = 0; i < table.size(); ++i) {
    auto crc = static_cast<std::uint16_t>(i << 8U);
    for (int bit = 0; bit < 8; ++bit) {
      crc = static_cast<std::uint16_t>(
        (crc & 0x8000U) != 0U ? ((crc << 1U) ^ kCrcPoly) : (crc << 1U));
    }
    table[i] = crc;
  }
  return table;
}

constexpr auto kCrcTable = make_crc_table();

}  // namespace

// ===========================================================================
// CRC and fixed point
// ===========================================================================

std::uint16_t crc16(std::span<const std::uint8_t> data, std::uint16_t crc) noexcept
{
  for (const std::uint8_t byte : data) {
    const auto index = static_cast<std::uint8_t>((crc >> 8U) ^ byte);
    crc = static_cast<std::uint16_t>((crc << 8U) ^ kCrcTable[index]);
  }
  return crc;
}

Encoded<std::int16_t> encode_i16(float value, float scale) noexcept
{
  if (!std::isfinite(value)) {
    return {0, true};
  }
  float scaled = value * scale;
  scaled = (scaled >= 0.0F) ? (scaled + 0.5F) : (scaled - 0.5F);
  if (scaled > 32767.0F) {
    return {32767, true};
  }
  if (scaled < -32768.0F) {
    return {-32768, true};
  }
  return {static_cast<std::int16_t>(scaled), false};
}

Encoded<std::uint16_t> encode_u16(float value, float scale) noexcept
{
  if (!std::isfinite(value)) {
    return {0, true};
  }
  const float scaled = value * scale + 0.5F;
  if (scaled > 65535.0F) {
    return {65535, true};
  }
  if (scaled < 0.0F) {
    return {0, true};
  }
  return {static_cast<std::uint16_t>(scaled), false};
}

double decode_i16(std::int16_t raw, float scale) noexcept
{
  return static_cast<double>(raw) / static_cast<double>(scale);
}

EncodedCommand encode_command(
  ModeRequest mode, const MitSetpoint & setpoint, std::uint8_t flags) noexcept
{
  const auto pos = encode_i16(setpoint.position_rad, kPosScale);
  const auto vel = encode_i16(setpoint.velocity_rad_s, kVelScale);
  const auto kp = encode_u16(setpoint.stiffness_nm_rad, kKpScale);
  const auto kd = encode_u16(setpoint.damping_nm_s_rad, kKdScale);
  const auto tau = encode_i16(setpoint.effort_nm, kTauScale);

  // The wire's position is the home-frame angle wrapped to +-pi. A target outside that range
  // would encode (the int16 reaches +-3.2767) but name a different angle than the one asked for.
  const bool in_home_frame =
    std::abs(static_cast<double>(setpoint.position_rad)) <= std::numbers::pi;

  EncodedCommand encoded{};
  encoded.command.mode_req = static_cast<std::uint8_t>(mode);
  encoded.command.pos = pos.raw;
  encoded.command.vel = vel.raw;
  encoded.command.kp = kp.raw;
  encoded.command.kd = kd.raw;
  encoded.command.tau_ff = tau.raw;
  encoded.command.flags = flags;
  encoded.representable =
    !(pos.saturated || vel.saturated || kp.saturated || kd.saturated || tau.saturated) &&
    in_home_frame;
  return encoded;
}

// ===========================================================================
// Framing
// ===========================================================================

std::optional<std::size_t> build_frame(
  MessageType type, std::uint16_t seq, NodeId src, NodeId dst, std::uint32_t ts_ms,
  std::span<const std::uint8_t> payload, std::span<std::uint8_t> out) noexcept
{
  const std::size_t total = kHeaderSize + payload.size();
  if (payload.size() > kMaxPayload || out.size() < total) {
    return std::nullopt;
  }

  MsgHeader header{};
  header.type = static_cast<std::uint16_t>(type);
  header.seq = seq;
  header.src = static_cast<std::uint8_t>(src);
  header.dst = static_cast<std::uint8_t>(dst);
  header.ts_ms = ts_ms;
  header.len = static_cast<std::uint16_t>(payload.size());
  header.ver_flags = kProtocolVersion;
  header.crc16 = frame_crc(header, payload);

  std::memcpy(out.data(), &header, kHeaderSize);
  if (!payload.empty()) {
    std::memcpy(out.data() + kHeaderSize, payload.data(), payload.size());
  }
  return total;
}

std::size_t FrameScanner::push(std::span<const std::uint8_t> bytes) noexcept
{
  if (begin_ == end_) {
    begin_ = 0;
    end_ = 0;
  } else if (begin_ > 0 && kCapacity - end_ < bytes.size()) {
    std::memmove(buffer_.data(), buffer_.data() + begin_, end_ - begin_);
    end_ -= begin_;
    begin_ = 0;
  }
  const std::size_t accepted = std::min(bytes.size(), kCapacity - end_);
  if (accepted > 0) {
    std::memcpy(buffer_.data() + end_, bytes.data(), accepted);
  }
  end_ += accepted;
  return accepted;
}

std::optional<FrameScanner::Frame> FrameScanner::next() noexcept
{
  while (end_ - begin_ >= kHeaderSize) {
    MsgHeader header;
    std::memcpy(&header, buffer_.data() + begin_, kHeaderSize);

    // Plausibility first, because it is cheap and a CRC over garbage is not. The version is
    // deliberately not part of it: a master on another protocol version must be recognisable as
    // such, rather than indistinguishable from line noise.
    if (!is_known_type(header.type) || header.len > kMaxPayload) {
      drop(1);
      continue;
    }

    const std::size_t total = kHeaderSize + header.len;
    if (end_ - begin_ < total) {
      return std::nullopt;
    }

    const std::span<const std::uint8_t> payload{buffer_.data() + begin_ + kHeaderSize, header.len};
    if (frame_crc(header, payload) != header.crc16) {
      drop(1);
      continue;
    }

    in_resync_ = false;
    begin_ += total;

    const auto version = static_cast<std::uint8_t>(header.ver_flags & 0xFFU);
    if (version != kProtocolVersion) {
      ++foreign_version_frames_;
      last_foreign_version_ = version;
      continue;
    }
    return Frame{static_cast<MessageType>(header.type), header.seq, header.ts_ms, payload};
  }
  return std::nullopt;
}

void FrameScanner::drop(std::size_t count) noexcept
{
  if (!in_resync_) {
    ++resync_events_;
    in_resync_ = true;
  }
  discarded_bytes_ += count;
  begin_ += count;
}

// ===========================================================================
// Payload decoding
// ===========================================================================

std::optional<RobotTelemetry> parse_robot_tele(std::span<const std::uint8_t> payload) noexcept
{
  if (payload.size() < sizeof(TeleRobotHeader)) {
    return std::nullopt;
  }

  RobotTelemetry telemetry{};
  std::memcpy(&telemetry.header, payload.data(), sizeof(TeleRobotHeader));

  const std::size_t chain_count = telemetry.header.n_chains;
  if (chain_count > kMaxChains ||
    payload.size() < sizeof(TeleRobotHeader) + chain_count * sizeof(TeleChain))
  {
    return std::nullopt;
  }

  for (std::size_t i = 0; i < chain_count; ++i) {
    std::memcpy(
      &telemetry.chains[i], payload.data() + sizeof(TeleRobotHeader) + i * sizeof(TeleChain),
      sizeof(TeleChain));
    if (telemetry.chains[i].n_motors > kMaxMotorsPerChain) {
      return std::nullopt;
    }
  }
  return telemetry;
}

std::optional<MasterStatus> parse_master_status(std::span<const std::uint8_t> payload) noexcept
{
  if (payload.size() < sizeof(MasterStatus)) {
    return std::nullopt;
  }
  MasterStatus status;
  std::memcpy(&status, payload.data(), sizeof(MasterStatus));
  return status;
}

const char * lifecycle_name(std::uint8_t state) noexcept
{
  switch (static_cast<Lifecycle>(state)) {
    case Lifecycle::kBoot: return "BOOT";
    case Lifecycle::kDiscovering: return "DISCOVERING";
    case Lifecycle::kIdle: return "IDLE";
    case Lifecycle::kHold: return "HOLD";
    case Lifecycle::kMit: return "MIT";
    case Lifecycle::kDamped: return "DAMPED";
    case Lifecycle::kToZero: return "TO_ZERO";
    case Lifecycle::kFault: return "FAULT";
  }
  return "UNKNOWN";
}

const char * cause_name(std::uint8_t cause) noexcept
{
  switch (static_cast<FaultCause>(cause)) {
    case FaultCause::kNone: return "NONE";
    case FaultCause::kOvertorque: return "OVERTORQUE";
    case FaultCause::kCanTimeout: return "CAN_TIMEOUT";
    case FaultCause::kWatchdog: return "WATCHDOG";
    case FaultCause::kMotorFault: return "MOTOR_FAULT";
    case FaultCause::kZeroTimeout: return "ZERO_TIMEOUT";
    case FaultCause::kNotEnabled: return "NOT_ENABLED";
    case FaultCause::kWound: return "WOUND";
    case FaultCause::kMasterLost: return "MASTER_LOST";
  }
  return "UNKNOWN";
}

const char * robot_state_name(std::uint8_t state) noexcept
{
  switch (static_cast<RobotState>(state)) {
    case RobotState::kInit: return "INIT";
    case RobotState::kReady: return "READY";
    case RobotState::kDegraded: return "DEGRADED";
    case RobotState::kHostLost: return "HOST_LOST";
  }
  return "UNKNOWN";
}

// ===========================================================================
// File-local helpers
// ===========================================================================

namespace
{

bool is_known_type(std::uint16_t type) noexcept
{
  switch (static_cast<MessageType>(type)) {
    case MessageType::kPing:
    case MessageType::kMasterStatus:
    case MessageType::kSlaveStatus:
    case MessageType::kRobotCmd:
    case MessageType::kRobotTele:
      return true;
  }
  return false;
}

// The CRC covers the header with its own crc field zeroed, then the payload.
std::uint16_t frame_crc(const MsgHeader & header, std::span<const std::uint8_t> payload) noexcept
{
  MsgHeader zeroed = header;
  zeroed.crc16 = 0;
  std::array<std::uint8_t, kHeaderSize> bytes{};
  std::memcpy(bytes.data(), &zeroed, kHeaderSize);
  return crc16(payload, crc16(bytes));
}

}  // namespace

}  // namespace humanoid::transport_stm32::wire
