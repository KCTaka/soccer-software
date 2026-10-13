// Copyright 2026 UTRA-RoboSoccer
// The vendored wire code (generated/) must match the pinned firmware release.
//
// generated/robot_wire_gen.hpp and generated/fixtures/ are copied from a tagged
// soccer-firmware release by generated/vendor.sh; generated/FIRMWARE_PIN records
// the tag and its schema hash. These tests make the pin self-checking:
//   - the vendored header's PROTO_SCHEMA_HASH equals the hash in FIRMWARE_PIN
//     (CMake reads the pin and passes it as WIRE_PINNED_SCHEMA_HASH), so a header
//     re-vendored without updating the pin — or vice versa — fails the build;
//   - the vendored golden fixtures decode to the known values and re-encode to
//     the same bytes, proving the header and the fixtures describe one layout
//     (the same guarantee soccer-firmware's test_cpp_wire enforces at the source).
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "robot_wire_gen.hpp"

namespace
{
std::vector<std::uint8_t> read_file(const std::string & path)
{
  std::vector<std::uint8_t> out;
  std::FILE * f = std::fopen(path.c_str(), "rb");
  if (f == nullptr) {return out;}
  std::array<std::uint8_t, 1024> buf{};
  std::size_t n = 0;
  while ((n = std::fread(buf.data(), 1, buf.size(), f)) > 0) {
    out.insert(out.end(), buf.begin(), buf.begin() + n);
  }
  std::fclose(f);
  return out;
}
}  // namespace

TEST(WirePin, SchemaHashMatchesPin)
{
  // WIRE_PINNED_SCHEMA_HASH is defined by CMake from generated/FIRMWARE_PIN.
  EXPECT_EQ(robot_wire::PROTO_SCHEMA_HASH, static_cast<std::uint32_t>(WIRE_PINNED_SCHEMA_HASH));
}

TEST(WirePin, CmdFixtureRoundTrips)
{
  const auto raw = read_file(std::string{WIRE_FIXTURE_DIR} + "/robot_cmd.bin");
  ASSERT_EQ(raw.size(), sizeof(robot_wire::cmd_robot_t));
  const auto c = robot_wire::from_bytes<robot_wire::cmd_robot_t>(raw.data());
  EXPECT_EQ(c.cycle_id, 7U);
  EXPECT_EQ(c.cmd_seq, 42U);
  EXPECT_EQ(c.n_chains, 1U);
  EXPECT_EQ(c.reserved, 0xDEU);  // master test sentinel, pinned in the fixture
  const robot_wire::cmd_motor_t m = c.chains[0].motors[0];
  EXPECT_EQ(m.drive_req, static_cast<std::uint8_t>(robot_wire::DriveReq::DRV_REQ_ENABLE));
  EXPECT_EQ(m.pos, 5000);
  EXPECT_EQ(m.flags, robot_wire::CMD_FLAG_VALID);
  const auto re = robot_wire::to_bytes(c);
  EXPECT_EQ(std::vector<std::uint8_t>(re.begin(), re.end()), raw);
}

TEST(WirePin, TeleFixtureRoundTrips)
{
  const auto raw = read_file(std::string{WIRE_FIXTURE_DIR} + "/robot_tele.bin");
  ASSERT_EQ(raw.size(), sizeof(robot_wire::tele_robot_t));
  const auto t = robot_wire::from_bytes<robot_wire::tele_robot_t>(raw.data());
  EXPECT_EQ(t.cycle_id, 9U);
  EXPECT_EQ(t.n_chains, 1U);
  EXPECT_EQ(
    t.robot_state, static_cast<std::uint8_t>(robot_wire::RobotState::ROBOT_READY));
  const robot_wire::tele_chain_t ch = t.chains[0];
  EXPECT_EQ(ch.link, static_cast<std::uint8_t>(robot_wire::ChainLink::CHAIN_LINK_ALIVE));
  const robot_wire::tele_motor_t mo = ch.motors[0];
  EXPECT_EQ(mo.drive_state, static_cast<std::uint8_t>(robot_wire::DriveState::DRV_ENABLED));
  EXPECT_EQ(mo.rs_fault_word, 0xDEADBEEFU);
  const auto re = robot_wire::to_bytes(t);
  EXPECT_EQ(std::vector<std::uint8_t>(re.begin(), re.end()), raw);
}
