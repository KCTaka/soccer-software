// Copyright 2026 UTRA-RoboSoccer
// The Jetson <-> master STM32 wire protocol of soccer-firmware, PROTO_VERSION 8.
//
// Source of truth: firmware/common/include/protocol.h on soccer-firmware branch
// akp/single_motor_rework (commit e30d756). This file mirrors it byte for byte; the golden frames
// under test/golden are produced by compiling that header, so a layout mistake here fails a test
// instead of misreading a motor. When the firmware changes any struct it bumps PROTO_VERSION, and
// kProtocolVersion below must follow it in the same change.
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

namespace humanoid::transport_stm32::wire
{

// The structs below are memcpy'd to and from the byte stream, so the host must share the MCU's
// byte order. Both supported hosts (x86-64, AArch64 Jetson) are little-endian.
static_assert(std::endian::native == std::endian::little, "the wire is little-endian");

inline constexpr std::uint8_t kProtocolVersion = 8;

// Fixed WIRE maximums. They size the arrays in every frame; the live counts travel in n_chains and
// n_motors. They are distinct from the firmware's generated per-build NUM_SLAVES / N_MOTORS.
inline constexpr std::size_t kMaxChains = 4;
inline constexpr std::size_t kMaxMotorsPerChain = 5;
inline constexpr std::size_t kMaxMotors = kMaxChains * kMaxMotorsPerChain;

inline constexpr std::size_t kHeaderSize = 16;

enum class MessageType : std::uint16_t
{
  kPing = 0x01,
  kMasterStatus = 0x02,
  kSlaveStatus = 0x03,
  kRobotCmd = 0x08,
  kRobotTele = 0x09,
};

enum class NodeId : std::uint8_t
{
  kJetson = 1,
  kMaster = 2,
};

/// tele_motor_t.state: what one motor is doing. HOLD, MIT, DAMPED and TO_ZERO are "armed".
enum class Lifecycle : std::uint8_t
{
  kBoot = 0,
  kDiscovering = 1,
  kIdle = 2,
  kHold = 3,
  kMit = 4,
  kDamped = 5,
  kToZero = 6,
  kFault = 7,
};

/// cmd_motor_t.mode_req. kHold is the only request that arms a motor from kIdle.
enum class ModeRequest : std::uint8_t
{
  kIdle = 0,
  kHold = 1,
  kMit = 2,
  kDamped = 3,
  kToZero = 4,
};

/// tele_motor_t.cause: why a motor is, or last was, faulted. Latched by the firmware.
enum class FaultCause : std::uint8_t
{
  kNone = 0,
  kOvertorque = 1,
  kCanTimeout = 2,
  kWatchdog = 3,
  kMotorFault = 4,
  kZeroTimeout = 5,
  kNotEnabled = 6,
  kWound = 7,
  kMasterLost = 8,
};

enum class RobotState : std::uint8_t
{
  kInit = 0,
  kReady = 1,
  kDegraded = 2,
  /// The master's host-death watchdog tripped. Latched until a command carries kCmdFlagFaultReset.
  kHostLost = 3,
};

// cmd_motor_t.flags
inline constexpr std::uint8_t kCmdFlagValid = 1U << 0U;
inline constexpr std::uint8_t kCmdFlagUseConfigGains = 1U << 1U;
inline constexpr std::uint8_t kCmdFlagFaultReset = 1U << 2U;

// Fixed-point scales (shared by every RobStride model). Mirror protocol.h exactly.
inline constexpr float kPosScale = 10000.0F;  // rad   -> i16, home-frame +-pi
inline constexpr float kVelScale = 100.0F;    // rad/s -> i16
inline constexpr float kTauScale = 100.0F;    // N*m   -> i16
inline constexpr float kKpScale = 10.0F;      // N*m/rad     -> u16
inline constexpr float kKdScale = 100.0F;     // N*m*s/rad   -> u16

// --- Frame header and payload structs -------------------------------------------------------

struct [[gnu::packed]] MsgHeader
{
  std::uint16_t type;
  std::uint16_t seq;
  std::uint8_t src;
  std::uint8_t dst;
  std::uint32_t ts_ms;
  std::uint16_t len;
  std::uint16_t ver_flags;  // low byte: protocol version; high byte reserved, 0
  std::uint16_t crc16;
};
static_assert(sizeof(MsgHeader) == kHeaderSize);

struct [[gnu::packed]] CmdMotor
{
  std::uint8_t mode_req;
  std::int16_t pos;
  std::int16_t vel;
  std::uint16_t kp;
  std::uint16_t kd;
  std::int16_t tau_ff;
  std::uint8_t flags;
};
static_assert(sizeof(CmdMotor) == 12);

struct [[gnu::packed]] CmdChain
{
  std::uint8_t chain_id;
  std::uint8_t n_motors;
  std::array<CmdMotor, kMaxMotorsPerChain> motors;
};
static_assert(sizeof(CmdChain) == 62);

struct [[gnu::packed]] CmdRobot
{
  std::uint16_t cycle_id;  // the master cycle whose telemetry this command was computed from
  std::uint16_t cmd_seq;   // one per host command, never 0 (0 means "none applied")
  std::uint8_t n_chains;
  std::uint8_t reserved;
  std::array<CmdChain, kMaxChains> chains;
};
static_assert(sizeof(CmdRobot) == 254);

struct [[gnu::packed]] TeleMotor
{
  std::int16_t pos;
  std::int16_t vel;
  std::int16_t tau;
  std::uint8_t temp_c;
  std::uint8_t state;  // Lifecycle
  std::uint8_t cause;  // FaultCause
  std::uint8_t motor_mode;
  // Drive fault bits: b0 undervoltage, b1 driver, b2 overheat, b3 encoder, b4 stall, b5 uncal.
  std::uint8_t motor_fault;
  std::uint8_t flags;
  std::uint8_t fb_age_ms;  // ms since the drive last reported, saturating at 255
  std::uint32_t fault_word;
  std::uint16_t last_applied_seq;
  std::uint16_t reserved;
};
static_assert(sizeof(TeleMotor) == 21);

struct [[gnu::packed]] TeleChain
{
  std::uint8_t chain_id;
  std::uint8_t n_motors;
  std::uint8_t spi_seq_echo;
  std::uint8_t spi_resyncs;
  std::uint32_t slave_time_us;
  std::uint16_t cmd_crc_errors;
  std::uint16_t can_tx_errors;
  std::uint8_t spi_tx_arm_fails;
  std::array<TeleMotor, kMaxMotorsPerChain> motors;
};
static_assert(sizeof(TeleChain) == 118);

/// The fixed head of tele_robot_t. The master transmits this plus only the first n_chains chains.
struct [[gnu::packed]] TeleRobotHeader
{
  std::uint16_t cycle_id;
  std::uint32_t master_time_us;
  std::uint16_t last_cmd_seq_rx;
  std::uint16_t cmd_seq_active;
  std::uint16_t cmd_on_time;
  std::uint16_t cmd_late;
  std::uint16_t cmd_missing;
  std::uint16_t cmd_duplicate;
  std::uint8_t n_chains;
  std::uint8_t robot_state;  // RobotState
};
static_assert(sizeof(TeleRobotHeader) == 20);

/// A decoded tele_robot_t. Chains at or beyond header.n_chains are zero.
struct RobotTelemetry
{
  TeleRobotHeader header{};
  std::array<TeleChain, kMaxChains> chains{};
};
static_assert(std::is_trivially_copyable_v<RobotTelemetry>);

struct [[gnu::packed]] MasterStatus
{
  std::uint8_t robot_state;
  std::uint8_t slave_alive;
  std::uint32_t uptime_ms;
  std::uint32_t link_errors;
  std::uint32_t rx_frames;
  std::uint16_t master_poll_hz;
  std::uint16_t telemetry_hz;
  std::uint16_t slave_tick_hz;
  std::uint16_t host_cmd_hz;
  std::uint32_t rx_resyncs;
  std::uint32_t rx_discarded_bytes;
};
static_assert(sizeof(MasterStatus) == 30);

inline constexpr std::size_t kRobotCmdFrameSize = kHeaderSize + sizeof(CmdRobot);
inline constexpr std::size_t kMaxPayload = sizeof(TeleRobotHeader) + kMaxChains * sizeof(TeleChain);

// --- CRC16-CCITT ----------------------------------------------------------------------------

inline constexpr std::uint16_t kCrcInit = 0xFFFFU;

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

/// Encode one motor's request. `flags` should include kCmdFlagValid for a live request.
[[nodiscard]] EncodedCommand encode_command(
  ModeRequest mode, const MitSetpoint & setpoint, std::uint8_t flags) noexcept;

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
