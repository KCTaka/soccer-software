// Copyright 2026 UTRA-RoboSoccer
// The host-side wire codec: CRC, fixed-point, the command encoder, framing, the resync FrameScanner
// and the telemetry/status decoders.
//
// The cross-repo wire oracle is generated/fixtures/*.bin (the firmware's own bytes, round-tripped
// in wire_pin_test). This file tests the HAND-WRITTEN host code the fixtures do not exercise:
// build_frame's framing, the FrameScanner's resynchronisation, and parse_robot_tele /
// parse_master_status. Frames are built at runtime with the (fixture-validated) codec, so the
// framing is exercised without testing the codec against itself.

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

// --- Canonical frames, built at runtime from the codec ---------------------------------------

// A two-chain telemetry frame (chain 0: two motors, chain 1: one), carrying distinctive field
// values so a decode mistake shows. Header seq 0x0007, ts 0x01020304.
RobotTelemetry sample_telemetry()
{
  RobotTelemetry tele{};
  tele.cycle_id = 0xFFFE;
  tele.master_time_us = 0xDEADBEEF;
  tele.last_cmd_seq_rx = 0x1111;
  tele.cmd_seq_active = 0x2222;
  tele.cmd_on_time = 5;
  tele.cmd_late = 6;
  tele.cmd_missing = 7;
  tele.cmd_duplicate = 8;
  tele.n_chains = 2;
  tele.robot_state = static_cast<std::uint8_t>(RobotState::ROBOT_READY);

  tele.chains[0].chain_id = 0;
  tele.chains[0].n_motors = 2;
  tele.chains[0].link = static_cast<std::uint8_t>(ChainLink::CHAIN_LINK_ALIVE);
  tele.chains[0].spi_seq_echo = 7;
  tele.chains[0].spi_resyncs = 1;
  tele.chains[0].slave_time_us = 123456789U;
  tele.chains[0].cmd_crc_errors = 2;
  tele.chains[0].can_tx_errors = 3;
  tele.chains[0].spi_tx_arm_fails = 4;

  auto & m0 = tele.chains[0].motors[0];
  m0.pos = 12345;
  m0.vel = -250;
  m0.tau = 1234;
  m0.temp_c = 41;
  m0.drive_state = static_cast<std::uint8_t>(DriveState::DRV_ENABLED);
  m0.drive_fault = static_cast<std::uint8_t>(DriveFault::DRVF_NONE);
  m0.rs_mode = 2;
  m0.rs_fault_bits = 0x04;
  m0.flags = 0x08;
  m0.fb_age_ms = 3;
  m0.rs_fault_word = 0x3022U;
  m0.last_applied_seq = 0xBEEF;

  auto & m1 = tele.chains[0].motors[1];
  m1.pos = -31416;
  m1.vel = 32767;
  m1.tau = -32768;
  m1.temp_c = 255;
  m1.drive_state = static_cast<std::uint8_t>(DriveState::DRV_FAULT);
  m1.drive_fault = static_cast<std::uint8_t>(DriveFault::DRVF_OVERTORQUE);
  m1.fb_age_ms = 255;
  m1.rs_fault_word = 0xFFFFFFFFU;

  tele.chains[1].chain_id = 1;
  tele.chains[1].n_motors = 1;
  tele.chains[1].link = static_cast<std::uint8_t>(ChainLink::CHAIN_LINK_ALIVE);
  tele.chains[1].motors[0].drive_state = static_cast<std::uint8_t>(DriveState::DRV_FAULT);
  tele.chains[1].motors[0].drive_fault = static_cast<std::uint8_t>(DriveFault::DRVF_MASTER_LOST);
  return tele;
}

// Serialises `tele` the way the master transmits it (fixed head + the n_chains chains actually
// present) and wraps it in a MSG_ROBOT_TELE frame.
std::vector<std::uint8_t> tele_frame(const RobotTelemetry & tele)
{
  const std::size_t payload_size = kTeleHeadSize + tele.n_chains * sizeof(TeleChain);
  std::vector<std::uint8_t> payload(payload_size);
  std::memcpy(payload.data(), &tele, payload_size);
  std::vector<std::uint8_t> frame(kHeaderSize + payload_size);
  const auto size = build_frame(
    MessageType::MSG_ROBOT_TELE, 0x0007, NodeId::NODE_MASTER, NodeId::NODE_JETSON, 0x01020304U,
    payload, frame);
  frame.resize(size.value_or(0));
  return frame;
}

MasterStatus sample_status()
{
  MasterStatus status{};
  status.schema_hash = kSchemaHash;
  status.slave_proto_mismatch = 0;
  status.uptime_ms = 123456U;
  status.link_errors = 9;
  status.rx_frames = 1000;
  status.master_poll_hz = 200;
  status.telemetry_hz = 200;
  status.slave_tick_hz = 200;
  status.host_cmd_hz = 50;
  return status;
}

std::vector<std::uint8_t> status_frame(const MasterStatus & status)
{
  std::array<std::uint8_t, sizeof(MasterStatus)> payload{};
  std::memcpy(payload.data(), &status, sizeof(status));
  std::vector<std::uint8_t> frame(kHeaderSize + sizeof(MasterStatus));
  const auto size = build_frame(
    MessageType::MSG_MASTER_STATUS, 0x0042, NodeId::NODE_MASTER, NodeId::NODE_JETSON, 5000U,
    payload, frame);
  frame.resize(size.value_or(0));
  return frame;
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

// ---------------------------------------------------------------------------
// encode_command: the drive request and the tuple are independent (v10)
// ---------------------------------------------------------------------------

TEST(WireEncodeCommand, CarriesTheRequestByteAndTheFullTuple)
{
  MitSetpoint setpoint;
  setpoint.position_rad = 0.5F;
  setpoint.velocity_rad_s = -1.25F;
  setpoint.stiffness_nm_rad = 15.0F;
  setpoint.damping_nm_s_rad = 1.0F;
  setpoint.effort_nm = 2.5F;

  const auto encoded = encode_command(DriveReq::DRV_REQ_ENABLE, setpoint, kCmdFlagValid);
  EXPECT_TRUE(encoded.representable);
  EXPECT_EQ(encoded.command.drive_req, static_cast<std::uint8_t>(DriveReq::DRV_REQ_ENABLE));
  EXPECT_NE(encoded.command.flags & kCmdFlagValid, 0);
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
  EXPECT_FALSE(encode_command(DriveReq::DRV_REQ_ENABLE, setpoint, kCmdFlagValid).representable);

  setpoint.position_rad = 3.1F;
  EXPECT_TRUE(encode_command(DriveReq::DRV_REQ_ENABLE, setpoint, kCmdFlagValid).representable);
}

TEST(WireEncodeCommand, AnyOutOfRangeFieldMakesTheWholeCommandUnrepresentable)
{
  MitSetpoint setpoint;
  setpoint.stiffness_nm_rad = 7000.0F;
  EXPECT_FALSE(encode_command(DriveReq::DRV_REQ_ENABLE, setpoint, kCmdFlagValid).representable);

  setpoint = MitSetpoint{};
  setpoint.effort_nm = 400.0F;
  EXPECT_FALSE(encode_command(DriveReq::DRV_REQ_ENABLE, setpoint, kCmdFlagValid).representable);

  setpoint = MitSetpoint{};
  setpoint.velocity_rad_s = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(encode_command(DriveReq::DRV_REQ_ENABLE, setpoint, kCmdFlagValid).representable);
}

// ---------------------------------------------------------------------------
// Decoding a frame the codec built
// ---------------------------------------------------------------------------

TEST(WireDecode, TelemetryRoundTripsThroughFramingAndTheDecoder)
{
  const auto frame = tele_frame(sample_telemetry());
  FrameScanner scanner;
  ASSERT_EQ(scanner.push(bytes_of(frame)), frame.size());

  const auto scanned = scanner.next();
  ASSERT_TRUE(scanned.has_value());
  EXPECT_EQ(scanned->type, MessageType::MSG_ROBOT_TELE);
  EXPECT_EQ(scanned->seq, 0x0007);
  EXPECT_EQ(scanned->ts_ms, 0x01020304U);

  const auto tele = parse_robot_tele(scanned->payload);
  ASSERT_TRUE(tele.has_value());
  EXPECT_EQ(tele->cycle_id, 0xFFFE);
  EXPECT_EQ(tele->master_time_us, 0xDEADBEEFU);
  EXPECT_EQ(tele->last_cmd_seq_rx, 0x1111);
  EXPECT_EQ(tele->cmd_seq_active, 0x2222);
  EXPECT_EQ(tele->cmd_on_time, 5);
  EXPECT_EQ(tele->cmd_duplicate, 8);
  EXPECT_EQ(tele->n_chains, 2);
  EXPECT_EQ(tele->robot_state, static_cast<std::uint8_t>(RobotState::ROBOT_READY));

  const auto & chain0 = tele->chains[0];
  EXPECT_EQ(chain0.chain_id, 0);
  EXPECT_EQ(chain0.n_motors, 2);
  EXPECT_EQ(chain0.link, static_cast<std::uint8_t>(ChainLink::CHAIN_LINK_ALIVE));
  EXPECT_EQ(chain0.slave_time_us, 123456789U);
  EXPECT_EQ(chain0.can_tx_errors, 3);

  const auto & m0 = chain0.motors[0];
  EXPECT_EQ(m0.pos, 12345);
  EXPECT_EQ(m0.vel, -250);
  EXPECT_EQ(m0.tau, 1234);
  EXPECT_EQ(m0.temp_c, 41);
  EXPECT_EQ(m0.drive_state, static_cast<std::uint8_t>(DriveState::DRV_ENABLED));
  EXPECT_EQ(m0.drive_fault, static_cast<std::uint8_t>(DriveFault::DRVF_NONE));
  EXPECT_EQ(m0.rs_mode, 2);
  EXPECT_EQ(m0.rs_fault_bits, 0x04);
  EXPECT_EQ(m0.fb_age_ms, 3);
  EXPECT_EQ(m0.rs_fault_word, 0x3022U);
  EXPECT_EQ(m0.last_applied_seq, 0xBEEF);

  const auto & m1 = chain0.motors[1];
  EXPECT_EQ(m1.pos, -31416);
  EXPECT_EQ(m1.vel, 32767);
  EXPECT_EQ(m1.tau, -32768);
  EXPECT_EQ(m1.drive_state, static_cast<std::uint8_t>(DriveState::DRV_FAULT));
  EXPECT_EQ(m1.drive_fault, static_cast<std::uint8_t>(DriveFault::DRVF_OVERTORQUE));
  EXPECT_EQ(m1.rs_fault_word, 0xFFFFFFFFU);

  const auto & chain1 = tele->chains[1];
  EXPECT_EQ(chain1.chain_id, 1);
  EXPECT_EQ(chain1.motors[0].drive_fault, static_cast<std::uint8_t>(DriveFault::DRVF_MASTER_LOST));

  // Only the transmitted prefix is decoded; chains beyond n_chains stay zero.
  EXPECT_EQ(tele->chains[2].n_motors, 0);
}

TEST(WireDecode, MasterStatusDecodesTheConfiguredRatesAndSchemaHash)
{
  const auto frame = status_frame(sample_status());
  FrameScanner scanner;
  ASSERT_GT(scanner.push(bytes_of(frame)), 0U);
  const auto scanned = scanner.next();
  ASSERT_TRUE(scanned.has_value());
  EXPECT_EQ(scanned->type, MessageType::MSG_MASTER_STATUS);

  const auto status = parse_master_status(scanned->payload);
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->schema_hash, kSchemaHash);
  EXPECT_EQ(status->slave_proto_mismatch, 0);
  EXPECT_EQ(status->uptime_ms, 123456U);
  EXPECT_EQ(status->master_poll_hz, 200);
  EXPECT_EQ(status->telemetry_hz, 200);
  EXPECT_EQ(status->slave_tick_hz, 200);
  EXPECT_EQ(status->host_cmd_hz, 50);
}

// ---------------------------------------------------------------------------
// Payload decoding: rejection of malformed telemetry
// ---------------------------------------------------------------------------

TEST(WireParse, TelemetryShorterThanItsDeclaredChainsIsRejected)
{
  const auto frame = tele_frame(sample_telemetry());
  FrameScanner scanner;
  ASSERT_GT(scanner.push(bytes_of(frame)), 0U);
  const auto scanned = scanner.next();
  ASSERT_TRUE(scanned.has_value());

  EXPECT_FALSE(parse_robot_tele(scanned->payload.first(scanned->payload.size() - 1)).has_value());
  EXPECT_FALSE(parse_robot_tele(scanned->payload.first(kTeleHeadSize - 1)).has_value());
}

TEST(WireParse, TelemetryDeclaringMoreChainsOrMotorsThanTheWireAllowsIsRejected)
{
  std::array<std::uint8_t, kMaxPayload> payload{};
  RobotTelemetry head{};
  head.n_chains = static_cast<std::uint8_t>(kMaxChains + 1);
  std::memcpy(payload.data(), &head, kTeleHeadSize);
  EXPECT_FALSE(parse_robot_tele(payload).has_value());

  head.n_chains = 1;
  std::memcpy(payload.data(), &head, kTeleHeadSize);
  TeleChain chain{};
  chain.n_motors = static_cast<std::uint8_t>(kMaxMotorsPerChain + 1);
  std::memcpy(payload.data() + kTeleHeadSize, &chain, sizeof(chain));
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
  const auto tele = tele_frame(sample_telemetry());
  const auto status = status_frame(sample_status());
  std::vector<std::uint8_t> stream;
  stream.insert(stream.end(), tele.begin(), tele.end());
  stream.insert(stream.end(), status.begin(), status.end());
  stream.insert(stream.end(), tele.begin(), tele.end());

  FrameScanner scanner;
  const auto types = scan_all(scanner, stream);
  ASSERT_EQ(types.size(), 3U);
  EXPECT_EQ(types[0], MessageType::MSG_ROBOT_TELE);
  EXPECT_EQ(types[1], MessageType::MSG_MASTER_STATUS);
  EXPECT_EQ(types[2], MessageType::MSG_ROBOT_TELE);
  EXPECT_EQ(scanner.discarded_bytes(), 0U);
  EXPECT_EQ(scanner.resync_events(), 0U);
}

TEST(FrameScanner, AssemblesAFrameDeliveredOneByteAtATime)
{
  const auto tele = tele_frame(sample_telemetry());
  FrameScanner scanner;
  std::size_t frames = 0;
  for (const std::uint8_t byte : tele) {
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
  const auto tele = tele_frame(sample_telemetry());
  stream.insert(stream.end(), tele.begin(), tele.end());

  FrameScanner scanner;
  const auto types = scan_all(scanner, stream);
  ASSERT_EQ(types.size(), 1U);
  EXPECT_EQ(types[0], MessageType::MSG_ROBOT_TELE);
  EXPECT_EQ(scanner.discarded_bytes(), 7U);
  EXPECT_EQ(scanner.resync_events(), 1U);
}

TEST(FrameScanner, ACorruptedFrameCostsOneFrameNotTheStream)
{
  const auto tele = tele_frame(sample_telemetry());
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
  const auto tele = tele_frame(sample_telemetry());
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
  header.type = static_cast<std::uint16_t>(MessageType::MSG_ROBOT_TELE);
  header.len = static_cast<std::uint16_t>(kMaxPayload + 1);
  header.ver_flags = kProtocolVersion;
  std::array<std::uint8_t, kHeaderSize> bytes{};
  std::memcpy(bytes.data(), &header, kHeaderSize);

  FrameScanner scanner;
  ASSERT_EQ(scanner.push(bytes), kHeaderSize);
  EXPECT_FALSE(scanner.next().has_value());
  EXPECT_GE(scanner.discarded_bytes(), 1U);

  // And the next real frame is still found.
  const auto status = status_frame(sample_status());
  EXPECT_EQ(scan_all(scanner, bytes_of(status)).size(), 1U);
}

TEST(FrameScanner, FramesOfAnotherProtocolVersionAreCountedAndNeverReturned)
{
  std::array<std::uint8_t, 8> payload{1, 2, 3, 4, 5, 6, 7, 8};
  MsgHeader header{};
  header.type = static_cast<std::uint16_t>(MessageType::MSG_MASTER_STATUS);
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
  const auto tele = tele_frame(sample_telemetry());
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
