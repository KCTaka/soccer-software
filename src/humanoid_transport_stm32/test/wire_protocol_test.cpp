// Copyright 2026 UTRA-RoboSoccer
// The wire codec against frames the firmware's own C code produced (test/golden), and the frame
// scanner's recovery behaviour.

#include <gtest/gtest.h>

#include <span>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <vector>

#include "humanoid_transport_stm32/wire_protocol.hpp"

#include "golden/golden_frames.hpp"

namespace
{

using namespace humanoid::transport_stm32::wire;  // NOLINT(build/namespaces)

template<typename Array>
std::span<const std::uint8_t> bytes_of(const Array & array)
{
  return {array.data(), array.size()};
}

// Feeds `bytes` to a scanner and collects the payload type of every frame it yields.
std::vector<MessageType> scan_all(FrameScanner & scanner, std::span<const std::uint8_t> bytes)
{
  std::vector<MessageType> types;
  while (!bytes.empty()) {
    const auto accepted = scanner.push(bytes);
    bytes = bytes.subspan(accepted);
    while (const auto frame = scanner.next()) {
      types.push_back(frame->type);
    }
  }
  return types;
}

// ---------------------------------------------------------------------------
// CRC and fixed point
// ---------------------------------------------------------------------------

TEST(WireCrc, MatchesTheStandardCheckValueAndTheFirmware)
{
  const std::array<std::uint8_t, 9> check{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(crc16(check), 0x29B1);
  EXPECT_EQ(crc16(check), humanoid::transport_stm32::golden::kCrcOfCheckString);
}

TEST(WireCrc, IsIncrementalAcrossASplit)
{
  const std::array<std::uint8_t, 9> check{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  const auto head = std::span<const std::uint8_t>{check}.first(4);
  const auto tail = std::span<const std::uint8_t>{check}.subspan(4);
  EXPECT_EQ(crc16(tail, crc16(head)), crc16(check));
}

TEST(WireFixedPoint, RoundsAndSaturatesExactlyAsTheFirmwareDoes)
{
  for (const auto & c : humanoid::transport_stm32::golden::kFixedPointCases) {
    EXPECT_EQ(encode_i16(c.x, c.scale).raw, c.raw) << "x=" << c.x << " scale=" << c.scale;
  }
  EXPECT_TRUE(encode_i16(4.0F, kPosScale).saturated);
  EXPECT_TRUE(encode_i16(-4.0F, kPosScale).saturated);
  EXPECT_FALSE(encode_i16(3.1416F, kPosScale).saturated);
}

TEST(WireFixedPoint, GainsUseTheUnsignedScales)
{
  EXPECT_EQ(encode_u16(15.0F, kKpScale).raw, humanoid::transport_stm32::golden::kKpOf15);
  EXPECT_EQ(encode_u16(1.0F, kKdScale).raw, humanoid::transport_stm32::golden::kKdOf1);
  EXPECT_FALSE(encode_u16(15.0F, kKpScale).saturated);
}

TEST(WireFixedPoint, NegativeGainAndNonFiniteValuesAreFlaggedNotPatched)
{
  // The firmware turns a negative gain into 0 silently. A negative gain is a defect upstream.
  EXPECT_TRUE(encode_u16(-1.0F, kKpScale).saturated);
  EXPECT_TRUE(encode_u16(7000.0F, kKpScale).saturated);
  EXPECT_TRUE(encode_i16(std::numeric_limits<float>::quiet_NaN(), kTauScale).saturated);
  EXPECT_TRUE(encode_i16(std::numeric_limits<float>::infinity(), kTauScale).saturated);
}

TEST(WireFixedPoint, DecodeInvertsEncodeWithinOneLsb)
{
  const auto encoded = encode_i16(1.2345F, kPosScale);
  EXPECT_NEAR(decode_i16(encoded.raw, kPosScale), 1.2345, 1.0 / kPosScale);
}

TEST(WireEncodeCommand, FullTupleMatchesTheGoldenMotorBytes)
{
  MitSetpoint setpoint;
  setpoint.position_rad = 0.5F;
  setpoint.velocity_rad_s = -1.25F;
  setpoint.stiffness_nm_rad = 15.0F;
  setpoint.damping_nm_s_rad = 1.0F;
  setpoint.effort_nm = 2.5F;

  const auto encoded = encode_command(ModeRequest::kMit, setpoint, kCmdFlagValid);
  EXPECT_TRUE(encoded.representable);
  EXPECT_EQ(encoded.command.mode_req, static_cast<std::uint8_t>(ModeRequest::kMit));
  EXPECT_EQ(encoded.command.pos, 5000);
  EXPECT_EQ(encoded.command.vel, -125);
  EXPECT_EQ(encoded.command.kp, 150);
  EXPECT_EQ(encoded.command.kd, 100);
  EXPECT_EQ(encoded.command.tau_ff, 250);
}

TEST(WireEncodeCommand, PositionOutsideTheHomeFrameIsNotRepresentable)
{
  MitSetpoint setpoint;
  setpoint.position_rad = 3.2F;  // fits the int16, but is not an angle in [-pi, pi]
  EXPECT_FALSE(encode_command(ModeRequest::kMit, setpoint, kCmdFlagValid).representable);

  setpoint.position_rad = 3.1F;
  EXPECT_TRUE(encode_command(ModeRequest::kMit, setpoint, kCmdFlagValid).representable);
}

TEST(WireEncodeCommand, AnyOutOfRangeFieldMakesTheWholeCommandUnrepresentable)
{
  MitSetpoint setpoint;
  setpoint.stiffness_nm_rad = 7000.0F;
  EXPECT_FALSE(encode_command(ModeRequest::kMit, setpoint, kCmdFlagValid).representable);

  setpoint = MitSetpoint{};
  setpoint.effort_nm = 400.0F;
  EXPECT_FALSE(encode_command(ModeRequest::kMit, setpoint, kCmdFlagValid).representable);

  setpoint = MitSetpoint{};
  setpoint.velocity_rad_s = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(encode_command(ModeRequest::kMit, setpoint, kCmdFlagValid).representable);
}

// ---------------------------------------------------------------------------
// Golden frames: bytes built by the firmware
// ---------------------------------------------------------------------------

TEST(WireGolden, HostCommandFrameIsByteIdenticalToTheFirmwaresBuilder)
{
  CmdRobot cmd{};
  cmd.cycle_id = 0x0102;
  cmd.cmd_seq = 0x0304;
  cmd.n_chains = 2;

  MitSetpoint mit;
  mit.position_rad = 0.5F;
  mit.velocity_rad_s = -1.25F;
  mit.stiffness_nm_rad = 15.0F;
  mit.damping_nm_s_rad = 1.0F;
  mit.effort_nm = 2.5F;

  cmd.chains[0].chain_id = 0;
  cmd.chains[0].n_motors = 2;
  cmd.chains[0].motors[0] = encode_command(ModeRequest::kMit, mit, kCmdFlagValid).command;
  cmd.chains[0].motors[1] =
    encode_command(ModeRequest::kHold, {}, kCmdFlagValid | kCmdFlagFaultReset).command;

  MitSetpoint damp;
  damp.damping_nm_s_rad = 2.0F;
  cmd.chains[1].chain_id = 1;
  cmd.chains[1].n_motors = 1;
  cmd.chains[1].motors[0] =
    encode_command(ModeRequest::kDamped, damp, kCmdFlagValid | kCmdFlagUseConfigGains).command;

  std::array<std::uint8_t, sizeof(CmdRobot)> payload{};
  std::memcpy(payload.data(), &cmd, sizeof(cmd));

  std::array<std::uint8_t, kRobotCmdFrameSize> frame{};
  const auto size = build_frame(
    MessageType::kRobotCmd, 0x1234, NodeId::kJetson, NodeId::kMaster, 0xA1B2C3D4, payload, frame);

  ASSERT_TRUE(size.has_value());
  EXPECT_EQ(*size, humanoid::transport_stm32::golden::kRobotCmdFrame.size());
  EXPECT_EQ(frame, humanoid::transport_stm32::golden::kRobotCmdFrame);
}

TEST(WireGolden, TelemetryFrameDecodesToTheValuesTheFirmwarePacked)
{
  FrameScanner scanner;
  const auto & bytes = humanoid::transport_stm32::golden::kRobotTeleFrame;
  ASSERT_EQ(scanner.push(bytes_of(bytes)), bytes.size());

  const auto frame = scanner.next();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->type, MessageType::kRobotTele);
  EXPECT_EQ(frame->seq, 0x0007);
  EXPECT_EQ(frame->ts_ms, 0x01020304U);

  const auto tele = parse_robot_tele(frame->payload);
  ASSERT_TRUE(tele.has_value());
  EXPECT_EQ(tele->header.cycle_id, 0xFFFE);
  EXPECT_EQ(tele->header.master_time_us, 0xDEADBEEFU);
  EXPECT_EQ(tele->header.last_cmd_seq_rx, 0x1111);
  EXPECT_EQ(tele->header.cmd_seq_active, 0x2222);
  EXPECT_EQ(tele->header.cmd_on_time, 5);
  EXPECT_EQ(tele->header.cmd_late, 6);
  EXPECT_EQ(tele->header.cmd_missing, 7);
  EXPECT_EQ(tele->header.cmd_duplicate, 8);
  EXPECT_EQ(tele->header.n_chains, 2);
  EXPECT_EQ(tele->header.robot_state, static_cast<std::uint8_t>(RobotState::kReady));

  const auto & chain0 = tele->chains[0];
  EXPECT_EQ(chain0.chain_id, 0);
  EXPECT_EQ(chain0.n_motors, 2);
  EXPECT_EQ(chain0.spi_seq_echo, 7);
  EXPECT_EQ(chain0.spi_resyncs, 1);
  EXPECT_EQ(chain0.slave_time_us, 123456789U);
  EXPECT_EQ(chain0.cmd_crc_errors, 2);
  EXPECT_EQ(chain0.can_tx_errors, 3);
  EXPECT_EQ(chain0.spi_tx_arm_fails, 4);

  const auto & m0 = chain0.motors[0];
  EXPECT_EQ(m0.pos, 12345);
  EXPECT_EQ(m0.vel, -250);
  EXPECT_EQ(m0.tau, 1234);
  EXPECT_EQ(m0.temp_c, 41);
  EXPECT_EQ(m0.state, static_cast<std::uint8_t>(Lifecycle::kMit));
  EXPECT_EQ(m0.cause, static_cast<std::uint8_t>(FaultCause::kNone));
  EXPECT_EQ(m0.motor_mode, 2);
  EXPECT_EQ(m0.motor_fault, 0x04);
  EXPECT_EQ(m0.flags, 0x08 | 0x20);
  EXPECT_EQ(m0.fb_age_ms, 3);
  EXPECT_EQ(m0.fault_word, 0x3022U);
  EXPECT_EQ(m0.last_applied_seq, 0xBEEF);

  const auto & m1 = chain0.motors[1];
  EXPECT_EQ(m1.pos, -31416);
  EXPECT_EQ(m1.vel, 32767);
  EXPECT_EQ(m1.tau, -32768);
  EXPECT_EQ(m1.temp_c, 255);
  EXPECT_EQ(m1.cause, static_cast<std::uint8_t>(FaultCause::kWatchdog));
  EXPECT_EQ(m1.fb_age_ms, 255);
  EXPECT_EQ(m1.fault_word, 0xFFFFFFFFU);

  const auto & chain1 = tele->chains[1];
  EXPECT_EQ(chain1.chain_id, 1);
  EXPECT_EQ(chain1.n_motors, 1);
  EXPECT_EQ(chain1.motors[0].state, static_cast<std::uint8_t>(Lifecycle::kFault));
  EXPECT_EQ(chain1.motors[0].cause, static_cast<std::uint8_t>(FaultCause::kOvertorque));

  // Only the transmitted prefix is decoded; the rest of the wire maximum stays zero.
  EXPECT_EQ(tele->chains[2].n_motors, 0);
}

TEST(WireGolden, MasterStatusDecodesTheConfiguredRates)
{
  FrameScanner scanner;
  ASSERT_GT(scanner.push(bytes_of(humanoid::transport_stm32::golden::kMasterStatusFrame)), 0U);
  const auto frame = scanner.next();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->type, MessageType::kMasterStatus);

  const auto status = parse_master_status(frame->payload);
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->robot_state, static_cast<std::uint8_t>(RobotState::kReady));
  EXPECT_EQ(status->slave_alive, 0x03);
  EXPECT_EQ(status->uptime_ms, 123456U);
  EXPECT_EQ(status->master_poll_hz, 200);
  EXPECT_EQ(status->telemetry_hz, 200);
  EXPECT_EQ(status->slave_tick_hz, 200);
  EXPECT_EQ(status->host_cmd_hz, 50);
}

// ---------------------------------------------------------------------------
// Payload decoding
// ---------------------------------------------------------------------------

TEST(WireParse, TelemetryShorterThanItsDeclaredChainsIsRejected)
{
  FrameScanner scanner;
  ASSERT_GT(scanner.push(bytes_of(humanoid::transport_stm32::golden::kRobotTeleFrame)), 0U);
  const auto frame = scanner.next();
  ASSERT_TRUE(frame.has_value());

  EXPECT_FALSE(parse_robot_tele(frame->payload.first(frame->payload.size() - 1)).has_value());
  EXPECT_FALSE(parse_robot_tele(frame->payload.first(sizeof(TeleRobotHeader) - 1)).has_value());
}

TEST(WireParse, TelemetryDeclaringMoreChainsOrMotorsThanTheWireAllowsIsRejected)
{
  std::array<std::uint8_t, kMaxPayload> payload{};
  TeleRobotHeader header{};
  header.n_chains = static_cast<std::uint8_t>(kMaxChains + 1);
  std::memcpy(payload.data(), &header, sizeof(header));
  EXPECT_FALSE(parse_robot_tele(payload).has_value());

  header.n_chains = 1;
  std::memcpy(payload.data(), &header, sizeof(header));
  TeleChain chain{};
  chain.n_motors = static_cast<std::uint8_t>(kMaxMotorsPerChain + 1);
  std::memcpy(payload.data() + sizeof(header), &chain, sizeof(chain));
  EXPECT_FALSE(parse_robot_tele(payload).has_value());
}

TEST(WireParse, MasterStatusShorterThanItsStructIsRejected)
{
  const std::array<std::uint8_t, sizeof(MasterStatus) - 1> shorter{};
  EXPECT_FALSE(parse_master_status(shorter).has_value());
}

// ---------------------------------------------------------------------------
// Frame scanner
// ---------------------------------------------------------------------------

TEST(FrameScanner, YieldsBackToBackFramesInOrder)
{
  std::vector<std::uint8_t> stream;
  const auto & tele = humanoid::transport_stm32::golden::kRobotTeleFrame;
  const auto & status = humanoid::transport_stm32::golden::kMasterStatusFrame;
  stream.insert(stream.end(), tele.begin(), tele.end());
  stream.insert(stream.end(), status.begin(), status.end());
  stream.insert(stream.end(), tele.begin(), tele.end());

  FrameScanner scanner;
  const auto types = scan_all(scanner, stream);
  ASSERT_EQ(types.size(), 3U);
  EXPECT_EQ(types[0], MessageType::kRobotTele);
  EXPECT_EQ(types[1], MessageType::kMasterStatus);
  EXPECT_EQ(types[2], MessageType::kRobotTele);
  EXPECT_EQ(scanner.discarded_bytes(), 0U);
  EXPECT_EQ(scanner.resync_events(), 0U);
}

TEST(FrameScanner, AssemblesAFrameDeliveredOneByteAtATime)
{
  FrameScanner scanner;
  std::size_t frames = 0;
  for (const std::uint8_t byte : humanoid::transport_stm32::golden::kRobotTeleFrame) {
    ASSERT_EQ(scanner.push({&byte, 1}), 1U);
    while (scanner.next()) {
      ++frames;
    }
  }
  EXPECT_EQ(frames, 1U);
  EXPECT_EQ(scanner.discarded_bytes(), 0U);
}

TEST(FrameScanner, ResyncsAfterLeadingGarbage)
{
  std::vector<std::uint8_t> stream{0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x02};
  const auto & tele = humanoid::transport_stm32::golden::kRobotTeleFrame;
  stream.insert(stream.end(), tele.begin(), tele.end());

  FrameScanner scanner;
  const auto types = scan_all(scanner, stream);
  ASSERT_EQ(types.size(), 1U);
  EXPECT_EQ(types[0], MessageType::kRobotTele);
  EXPECT_EQ(scanner.discarded_bytes(), 7U);
  EXPECT_EQ(scanner.resync_events(), 1U);
}

TEST(FrameScanner, ACorruptedFrameCostsOneFrameNotTheStream)
{
  const auto & tele = humanoid::transport_stm32::golden::kRobotTeleFrame;
  std::vector<std::uint8_t> stream(tele.begin(), tele.end());
  stream[kHeaderSize + 30] ^= 0x01;  // one flipped bit inside the first frame's payload
  stream.insert(stream.end(), tele.begin(), tele.end());

  FrameScanner scanner;
  const auto types = scan_all(scanner, stream);
  ASSERT_EQ(types.size(), 1U);  // the second frame survives
  EXPECT_GE(scanner.resync_events(), 1U);
  EXPECT_GE(scanner.discarded_bytes(), 1U);
}

TEST(FrameScanner, ALostByteCostsOneFrameNotTheStream)
{
  const auto & tele = humanoid::transport_stm32::golden::kRobotTeleFrame;
  std::vector<std::uint8_t> stream(tele.begin(), tele.end());
  stream.erase(stream.begin() + 40);
  stream.insert(stream.end(), tele.begin(), tele.end());

  FrameScanner scanner;
  EXPECT_EQ(scan_all(scanner, stream).size(), 1U);
}

TEST(FrameScanner, AnImplausibleLengthIsDroppedWithoutWaitingForItsBody)
{
  // A header claiming a payload larger than any frame of this protocol must not make the scanner
  // wait for bytes that will never come.
  MsgHeader header{};
  header.type = static_cast<std::uint16_t>(MessageType::kRobotTele);
  header.len = static_cast<std::uint16_t>(kMaxPayload + 1);
  header.ver_flags = kProtocolVersion;
  std::array<std::uint8_t, kHeaderSize> bytes{};
  std::memcpy(bytes.data(), &header, kHeaderSize);

  FrameScanner scanner;
  ASSERT_EQ(scanner.push(bytes), kHeaderSize);
  EXPECT_FALSE(scanner.next().has_value());
  EXPECT_GE(scanner.discarded_bytes(), 1U);

  // And the next real frame is still found.
  EXPECT_EQ(
    scan_all(scanner, bytes_of(humanoid::transport_stm32::golden::kMasterStatusFrame)).size(), 1U);
}

TEST(FrameScanner, FramesOfAnotherProtocolVersionAreCountedAndNeverReturned)
{
  std::array<std::uint8_t, 8> payload{1, 2, 3, 4, 5, 6, 7, 8};
  MsgHeader header{};
  header.type = static_cast<std::uint16_t>(MessageType::kMasterStatus);
  header.len = static_cast<std::uint16_t>(payload.size());
  header.ver_flags = 7;  // an older master
  std::array<std::uint8_t, kHeaderSize> head_bytes{};
  std::memcpy(head_bytes.data(), &header, kHeaderSize);
  header.crc16 = crc16(payload, crc16(head_bytes));

  std::vector<std::uint8_t> stream(kHeaderSize + payload.size());
  std::memcpy(stream.data(), &header, kHeaderSize);
  std::memcpy(stream.data() + kHeaderSize, payload.data(), payload.size());

  FrameScanner scanner;
  EXPECT_TRUE(scan_all(scanner, stream).empty());
  EXPECT_EQ(scanner.foreign_version_frames(), 1U);
  EXPECT_EQ(scanner.last_foreign_version(), 7);
  EXPECT_EQ(scanner.discarded_bytes(), 0U);  // recognised, not treated as noise
}

TEST(FrameScanner, NeverRefusesBytesWhenDrainedBetweenPushes)
{
  FrameScanner scanner;
  const auto & tele = humanoid::transport_stm32::golden::kRobotTeleFrame;
  std::size_t frames = 0;
  for (int i = 0; i < 100; ++i) {
    ASSERT_EQ(scanner.push(bytes_of(tele)), tele.size());
    while (scanner.next()) {
      ++frames;
    }
  }
  EXPECT_EQ(frames, 100U);
}

}  // namespace
