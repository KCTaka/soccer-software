// Copyright 2026 UTRA-RoboSoccer
// The master logic the fake master emulates BY HAND rather than from the vendored drive SM: the
// version-gate enable refusal (spi_gate.h) and the robot_state reporting priority (spi_master.c /
// protocol_v9.md §robot_state). Nothing else covers these, so they are tested directly, on the
// deterministic manual clock so a cycle count means a cycle count regardless of CI load.

#include <gtest/gtest.h>

#include <cstdint>

#include "fake_master.hpp"
#include "humanoid_transport_stm32/wire_protocol.hpp"

namespace
{

using humanoid::transport_stm32::testing::FakeMaster;
namespace wire = humanoid::transport_stm32::wire;

// Two chains (ids 0 and 1), one motor each, on a manual clock.
FakeMaster::Config two_chains()
{
  FakeMaster::Config config;
  config.chain_ids = {0, 1};
  config.motors_per_chain = {1, 1};
  config.clock_mode = FakeMaster::ClockMode::kManual;
  return config;
}

// A command whose motor (chain c, motor 0) carries `req`; every other wired slot is DISABLE.
wire::CmdRobot command_for(std::uint8_t chain, wire::DriveReq req, std::uint16_t seq)
{
  wire::CmdRobot command{};
  command.cmd_seq = seq;
  command.n_chains = 2;
  for (std::uint8_t c = 0; c < 2; ++c) {
    command.chains[c].chain_id = c;
    command.chains[c].n_motors = 1;
    command.chains[c].motors[0].drive_req =
      static_cast<std::uint8_t>(wire::DriveReq::DRV_REQ_DISABLE);
  }
  command.chains[chain].motors[0].drive_req = static_cast<std::uint8_t>(req);
  return command;
}

// ---------------------------------------------------------------------------
// Version gate: a fresh ENABLE is refused robot-wide while any slave mismatches
// ---------------------------------------------------------------------------

TEST(VersionGate, EnableReachesTheMotorWhenNoSlaveMismatches)
{
  FakeMaster fake(two_chains());  // control for the test below
  fake.deliver_command(command_for(0, wire::DriveReq::DRV_REQ_ENABLE, 1));
  fake.advance(5);
  EXPECT_EQ(fake.state(0, 0), wire::DriveState::DRV_ENABLED);
}

TEST(VersionGate, AFreshEnableIsRefusedRobotWideWhileAnotherSlaveMismatches)
{
  FakeMaster fake(two_chains());
  fake.set_chain_mismatch(1, true);  // chain 1 fails the version gate
  fake.deliver_command(command_for(0, wire::DriveReq::DRV_REQ_ENABLE, 1));
  fake.advance(5);
  // The ENABLE for the not-yet-armed motor on the healthy chain was rewritten to DISABLE.
  EXPECT_EQ(fake.state(0, 0), wire::DriveState::DRV_DISABLED);
}

TEST(VersionGate, AnAlreadyArmedMotorIsLeftAloneWhenAnotherSlaveStartsMismatching)
{
  FakeMaster fake(two_chains());
  fake.deliver_command(command_for(0, wire::DriveReq::DRV_REQ_ENABLE, 1));
  fake.advance(5);
  ASSERT_EQ(fake.state(0, 0), wire::DriveState::DRV_ENABLED);

  fake.set_chain_mismatch(1, true);
  fake.deliver_command(command_for(0, wire::DriveReq::DRV_REQ_ENABLE, 2));
  fake.advance(5);
  EXPECT_EQ(fake.state(0, 0), wire::DriveState::DRV_ENABLED);  // the gate leaves armed motors alone
}

// ---------------------------------------------------------------------------
// robot_state reporting priority: HOST_LOST > PROTO_MISMATCH > INIT/READY/DEGRADED
// ---------------------------------------------------------------------------

TEST(RobotStatePriority, EveryChainAliveReportsReady)
{
  FakeMaster fake(two_chains());
  fake.advance(1);
  EXPECT_EQ(fake.robot_state(), wire::RobotState::ROBOT_READY);
}

TEST(RobotStatePriority, AMissingChainReportsDegraded)
{
  FakeMaster fake(two_chains());
  fake.drop_chain(1, true);
  fake.advance(1);
  EXPECT_EQ(fake.robot_state(), wire::RobotState::ROBOT_DEGRADED);
}

TEST(RobotStatePriority, NoChainAliveReportsInit)
{
  FakeMaster fake(two_chains());
  fake.drop_chain(0, true);
  fake.drop_chain(1, true);
  fake.advance(1);
  EXPECT_EQ(fake.robot_state(), wire::RobotState::ROBOT_INIT);
}

TEST(RobotStatePriority, ProtoMismatchOutranksDegraded)
{
  FakeMaster fake(two_chains());
  fake.set_chain_mismatch(1, true);  // a mismatched chain is also not alive (would be DEGRADED)
  fake.advance(1);
  EXPECT_EQ(fake.robot_state(), wire::RobotState::ROBOT_PROTO_MISMATCH);
}

TEST(RobotStatePriority, HostLostOutranksProtoMismatch)
{
  FakeMaster fake(two_chains());
  // Arm a motor so the host-death watchdog can trip, then mismatch the other chain.
  fake.deliver_command(command_for(0, wire::DriveReq::DRV_REQ_ENABLE, 1));
  fake.advance(5);
  ASSERT_EQ(fake.state(0, 0), wire::DriveState::DRV_ENABLED);

  fake.set_chain_mismatch(1, true);
  // No fresh command for well over HOST_LOST_CYCLES (5): the watchdog latches HOST_LINK_STOPPED.
  fake.advance(15);
  ASSERT_TRUE(fake.host_lost());
  EXPECT_EQ(fake.robot_state(), wire::RobotState::ROBOT_HOST_LOST);
}

}  // namespace
