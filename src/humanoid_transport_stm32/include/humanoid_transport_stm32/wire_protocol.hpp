// Copyright 2026 UTRA-RoboSoccer
// The Jetson <-> master STM32 wire protocol of soccer-firmware.
//
// Thin wrapper over the GENERATED header vendored in generated/robot_wire_gen.hpp (copied from a
// pinned soccer-firmware release by generated/vendor.sh; see generated/FIRMWARE_PIN). The layout --
// every struct, enum, flag and scale -- is defined once, in firmware's protocol.yaml, and emitted
// into that header; this file does NOT redeclare it. It aliases the generated types into this
// namespace and keeps only the hand-written host-side logic: CRC, fixed-point encode/decode,
// framing, the resync FrameScanner, payload decoders, and name helpers.
//
// kProtocolVersion and the layout come from the generated header, so a firmware bump is picked up
// by re-vendoring (vendor.sh), not by editing this file. wire_pin_test asserts the vendored
// schema hash equals the pin and that the firmware golden fixtures round-trip.
//
// Everything here is allocation-free and noexcept, so the real-time exchange() path may use it.
#ifndef HUMANOID_TRANSPORT_STM32__WIRE_PROTOCOL_HPP_
#define HUMANOID_TRANSPORT_STM32__WIRE_PROTOCOL_HPP_

#include <bit>
#include <span>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>

#include "robot_wire_gen.hpp"

namespace humanoid::transport_stm32::wire
{

// The structs are memcpy'd to and from the byte stream, so the host must share the MCU's byte
// order. Both supported hosts (x86-64, AArch64 Jetson) are little-endian.
static_assert(std::endian::native == std::endian::little, "the wire is little-endian");

// --- Layout, enums, flags and scales: aliased from the generated header -----------------------

inline constexpr std::uint8_t kProtocolVersion = robot_wire::PROTO_VERSION;
inline constexpr std::uint32_t kSchemaHash = robot_wire::PROTO_SCHEMA_HASH;

inline constexpr std::size_t kMaxChains = robot_wire::MAX_CHAINS;
inline constexpr std::size_t kMaxMotorsPerChain = robot_wire::MAX_MOTORS_PER_CHAIN;
inline constexpr std::size_t kMaxMotors = kMaxChains * kMaxMotorsPerChain;
inline constexpr std::size_t kHeaderSize = robot_wire::MSG_HEADER_SIZE;

using MessageType = robot_wire::MsgType;
using NodeId = robot_wire::NodeId;
/// Per-motor drive state reported in tele_motor_t.drive_state (v10 CiA-402-style lifecycle).
using DriveState = robot_wire::DriveState;
/// Per-motor drive request in cmd_motor_t.drive_req. The MIT tuple travels under kCmdFlagValid;
/// there is no "MIT request" -- an ENABLED motor simply applies every valid tuple.
using DriveReq = robot_wire::DriveReq;
/// Why a motor is, or last was, faulted. Latched at detection by the firmware.
using DriveFault = robot_wire::DriveFault;
using RobotState = robot_wire::RobotState;
/// Per-chain SPI link state, authored by the master in tele_chain_t.link.
using ChainLink = robot_wire::ChainLink;

using MsgHeader = robot_wire::MsgHeader;
using CmdMotor = robot_wire::cmd_motor_t;
using CmdChain = robot_wire::cmd_chain_t;
using CmdRobot = robot_wire::cmd_robot_t;
using TeleMotor = robot_wire::tele_motor_t;
using TeleChain = robot_wire::tele_chain_t;
using MasterStatus = robot_wire::MasterStatus;

// cmd_motor_t.flags
inline constexpr std::uint8_t kCmdFlagValid = robot_wire::CMD_FLAG_VALID;
inline constexpr std::uint8_t kCmdFlagUseConfigGains = robot_wire::CMD_FLAG_USE_CONFIG_GAINS;

// tele_motor_t.flags
inline constexpr std::uint8_t kTeleFlagRequestRejected = robot_wire::TELE_FLAG_REQUEST_REJECTED;

// Fixed-point scales (shared by every RobStride model).
inline constexpr float kPosScale = robot_wire::PROTO_POS_SCALE;  // rad   -> i16, home-frame +-pi
inline constexpr float kVelScale = robot_wire::PROTO_VEL_SCALE;  // rad/s -> i16
inline constexpr float kTauScale = robot_wire::PROTO_TAU_SCALE;  // N*m   -> i16
inline constexpr float kKpScale = robot_wire::PROTO_KP_SCALE;    // N*m/rad   -> u16
inline constexpr float kKdScale = robot_wire::PROTO_KD_SCALE;    // N*m*s/rad -> u16

/// Names for reports and error messages. An undefined value reads "UNKNOWN".
[[nodiscard]] const char * drive_state_name(std::uint8_t state) noexcept;
[[nodiscard]] const char * drive_fault_name(std::uint8_t fault) noexcept;
[[nodiscard]] const char * drive_req_name(std::uint8_t req) noexcept;
[[nodiscard]] const char * robot_state_name(std::uint8_t state) noexcept;
[[nodiscard]] const char * chain_link_name(std::uint8_t link) noexcept;

// --- Decoded telemetry ------------------------------------------------------------------------

/// A decoded tele_robot_t: the full fixed-size struct, chains at or beyond n_chains left zero.
/// The generated tele_robot_t is a flat packed POD (header fields then chains[kMaxChains]); the
/// head is its first robot_wire::offsetof(chains) bytes, followed by the chains actually sent.
using RobotTelemetry = robot_wire::tele_robot_t;
static_assert(std::is_trivially_copyable_v<RobotTelemetry>);

inline constexpr std::size_t kTeleHeadSize = offsetof(robot_wire::tele_robot_t, chains);
inline constexpr std::size_t kMaxPayload = sizeof(RobotTelemetry);  // = PROTO_MAX_PAYLOAD
inline constexpr std::size_t kRobotCmdFrameSize = kHeaderSize + sizeof(CmdRobot);

// --- CRC16-CCITT ------------------------------------------------------------------------------

inline constexpr std::uint16_t kCrcInit = robot_wire::PROTO_CRC_INIT;

/// CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection), as the firmware's proto_crc16.
[[nodiscard]] std::uint16_t crc16(
  std::span<const std::uint8_t> data, std::uint16_t crc = kCrcInit) noexcept;

// --- Fixed-point encoding -------------------------------------------------------------------
// Float arithmetic, as in the firmware: the same input must round to the same raw value on both
// sides, and a double computation can round a tie the other way.

template<typename Raw>
struct Encoded
{
  Raw raw;
  /// The value did not fit the wire type (or was not finite) and raw is the clamped result.
  bool saturated;
};

/// Round half away from zero, saturating at the int16 limits.
[[nodiscard]] Encoded<std::int16_t> encode_i16(float value, float scale) noexcept;

/// Gains are non-negative: a negative value is reported as saturated rather than silently zeroed.
[[nodiscard]] Encoded<std::uint16_t> encode_u16(float value, float scale) noexcept;

[[nodiscard]] double decode_i16(std::int16_t raw, float scale) noexcept;

/// One motor's MIT setpoint in SI units, as the five-field tuple of ADR-001.
struct MitSetpoint
{
  float position_rad{0.0F};
  float velocity_rad_s{0.0F};
  float stiffness_nm_rad{0.0F};
  float damping_nm_s_rad{0.0F};
  float effort_nm{0.0F};
};

struct EncodedCommand
{
  CmdMotor command;
  /// False when any field was not finite, did not fit the wire type, or put the position outside
  /// the home-frame range of +-pi. The encoded command is then NOT safe to send.
  bool representable;
};

/// Encode one motor's drive request plus MIT tuple. `flags` should include kCmdFlagValid for a
/// live tuple; in v10 the request byte (DriveReq) and the tuple are independent.
[[nodiscard]] EncodedCommand encode_command(
  DriveReq request, const MitSetpoint & setpoint, std::uint8_t flags) noexcept;

// --- Framing --------------------------------------------------------------------------------

/// Write one complete frame: header, payload, CRC. Returns the frame size, or nothing if `out` is
/// too small or the payload exceeds kMaxPayload.
[[nodiscard]] std::optional<std::size_t> build_frame(
  MessageType type, std::uint16_t seq, NodeId src, NodeId dst, std::uint32_t ts_ms,
  std::span<const std::uint8_t> payload, std::span<std::uint8_t> out) noexcept;

/// Incremental frame extractor for the byte stream. The stream has no delimiter and no magic
/// number, so a frame boundary is found the way the firmware finds it: a plausible header followed
/// by a body whose CRC checks out. Anything else drops ONE byte and retries, so a lost byte costs
/// one frame, not the stream.
class FrameScanner
{
public:
  struct Frame
  {
    MessageType type;
    std::uint16_t seq;
    std::uint32_t ts_ms;
    /// Points into the scanner's buffer; valid until the next push() or next().
    std::span<const std::uint8_t> payload;
  };

  /// Appends as many bytes as fit and returns that count. next() drains the buffer, so a caller
  /// that alternates push() and next() until it returns nothing never has bytes refused.
  [[nodiscard]] std::size_t push(std::span<const std::uint8_t> bytes) noexcept;

  /// The next complete, CRC-valid frame of this protocol version, or nothing if more bytes are
  /// needed. Frames of another protocol version are consumed and counted, never returned.
  [[nodiscard]] std::optional<Frame> next() noexcept;

  /// Bytes dropped while searching for a frame boundary.
  [[nodiscard]] std::uint64_t discarded_bytes() const noexcept {return discarded_bytes_;}

  /// Distinct runs of dropped bytes: one corrupted frame counts once, however long it was.
  [[nodiscard]] std::uint64_t resync_events() const noexcept {return resync_events_;}

  /// CRC-valid frames from a master that speaks another protocol version.
  [[nodiscard]] std::uint64_t foreign_version_frames() const noexcept
  {
    return foreign_version_frames_;
  }

  /// Version byte of the most recent such frame, for the error message; 0 if none was seen.
  [[nodiscard]] std::uint8_t last_foreign_version() const noexcept {return last_foreign_version_;}

private:
  static constexpr std::size_t kCapacity = 2048;
  static_assert(kCapacity >= 2 * (kHeaderSize + kMaxPayload));

  void drop(std::size_t count) noexcept;

  std::array<std::uint8_t, kCapacity> buffer_{};
  std::size_t begin_{0};
  std::size_t end_{0};
  bool in_resync_{false};
  std::uint64_t discarded_bytes_{0};
  std::uint64_t resync_events_{0};
  std::uint64_t foreign_version_frames_{0};
  std::uint8_t last_foreign_version_{0};
};

// --- Payload decoding -----------------------------------------------------------------------

/// Decode a tele_robot_t payload: the fixed head plus the n_chains chains actually transmitted.
/// Nothing if the payload is shorter than that, or declares more chains or motors than the wire
/// allows.
[[nodiscard]] std::optional<RobotTelemetry> parse_robot_tele(
  std::span<const std::uint8_t> payload) noexcept;

[[nodiscard]] std::optional<MasterStatus> parse_master_status(
  std::span<const std::uint8_t> payload) noexcept;

}  // namespace humanoid::transport_stm32::wire

#endif  // HUMANOID_TRANSPORT_STM32__WIRE_PROTOCOL_HPP_
