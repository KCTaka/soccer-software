// Copyright 2026 UTRA-RoboSoccer
// The serial link against a fake master on a pseudo-terminal: framing, resynchronisation,
// duplicate and reset detection, and bounded waits.

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include "fake_master.hpp"
#include "humanoid_transport_stm32/master_link.hpp"
#include "humanoid_transport_stm32/wire_protocol.hpp"

namespace
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using humanoid::transport_stm32::MasterLink;
using humanoid::transport_stm32::WriteStatus;
using humanoid::transport_stm32::testing::FakeMaster;
namespace wire = humanoid::transport_stm32::wire;

class MasterLinkTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fake = std::make_unique<FakeMaster>(FakeMaster::Config{});
    fake->start();
  }

  void TearDown() override
  {
    link.close();
    fake.reset();
  }

  // Waits for `count` more telemetry frames.
  bool wait_frames(std::uint64_t count)
  {
    const auto target = link.frame_count() + count;
    return link.wait_for_frame(target - 1, 2s).has_value();
  }

  std::unique_ptr<FakeMaster> fake;
  MasterLink link;
};

TEST_F(MasterLinkTest, OpensAndReceivesStatusAndTelemetry)
{
  ASSERT_FALSE(link.open(fake->device()).has_value());

  const auto status = link.wait_for_status(1s);
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->master_poll_hz, 200);
  EXPECT_EQ(status->telemetry_hz, 200);

  const auto frame = link.wait_for_frame(0, 1s);
  ASSERT_TRUE(frame.has_value());
  EXPECT_TRUE(frame->valid);
  EXPECT_EQ(frame->robot.header.n_chains, 1);
  EXPECT_EQ(frame->robot.chains[0].n_motors, 2);

  EXPECT_EQ(link.counters().framing_errors, 0U);
  EXPECT_EQ(link.counters().duplicate_frames, 0U);
  EXPECT_FALSE(link.failed());
}

TEST_F(MasterLinkTest, ReportsADeviceThatCannotBeOpened)
{
  const auto error = link.open("/dev/definitely-not-a-serial-port");
  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("cannot open"), std::string::npos);
}

TEST_F(MasterLinkTest, ReportsAFileThatIsNotASerialDevice)
{
  const auto error = link.open("/dev/null");
  ASSERT_TRUE(error.has_value());
}

TEST_F(MasterLinkTest, DeliversACommandFrameToTheMaster)
{
  ASSERT_FALSE(link.open(fake->device()).has_value());
  ASSERT_TRUE(link.wait_for_frame(0, 1s).has_value());

  wire::CmdRobot command{};
  command.cmd_seq = 77;
  command.n_chains = 1;
  command.chains[0].chain_id = 0;
  command.chains[0].n_motors = 1;
  command.chains[0].motors[0].mode_req = static_cast<std::uint8_t>(wire::ModeRequest::kHold);
  command.chains[0].motors[0].flags = wire::kCmdFlagValid;

  std::array<std::uint8_t, sizeof(command)> payload{};
  std::memcpy(payload.data(), &command, sizeof(command));
  std::array<std::uint8_t, wire::kRobotCmdFrameSize> frame{};
  ASSERT_TRUE(wire::build_frame(
      wire::MessageType::kRobotCmd, 1, wire::NodeId::kJetson, wire::NodeId::kMaster, 0, payload,
      frame).has_value());

  ASSERT_EQ(
    link.send(frame, std::chrono::steady_clock::now() + 100ms), WriteStatus::kOk);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (fake->commands_received() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_EQ(fake->commands_received(), 1U);
  EXPECT_EQ(fake->last_command()->cmd_seq, 77);
}

TEST_F(MasterLinkTest, WaitForNewerReturnsEachFrameInOrder)
{
  ASSERT_FALSE(link.open(fake->device()).has_value());
  auto frame = link.wait_for_frame(0, 1s);
  ASSERT_TRUE(frame.has_value());

  std::uint16_t cycle = frame->robot.header.cycle_id;
  for (int i = 0; i < 20; ++i) {
    const auto next = link.wait_for_newer(cycle, std::chrono::steady_clock::now() + 200ms);
    ASSERT_TRUE(next.has_value()) << "frame " << i;
    EXPECT_GT(static_cast<std::int16_t>(next->robot.header.cycle_id - cycle), 0);
    cycle = next->robot.header.cycle_id;
  }
}

TEST_F(MasterLinkTest, WaitForNewerReturnsAtTheDeadlineWhenTheMasterIsSilent)
{
  ASSERT_FALSE(link.open(fake->device()).has_value());
  auto frame = link.wait_for_frame(0, 1s);
  ASSERT_TRUE(frame.has_value());
  fake->set_silent(true);
  std::this_thread::sleep_for(30ms);  // let anything in flight arrive
  const auto latest = link.wait_for_frame(link.frame_count() - 1, 1s);
  ASSERT_TRUE(latest.has_value());

  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + 30ms;
  const auto result = link.wait_for_newer(latest->robot.header.cycle_id, deadline);
  const auto end = std::chrono::steady_clock::now();

  EXPECT_FALSE(result.has_value());
  EXPECT_GE(end, deadline);
  EXPECT_LT(end - deadline, 20ms);  // returns at the deadline, not a poll interval later
}

TEST_F(MasterLinkTest, ResynchronisesAfterGarbageAndCountsIt)
{
  ASSERT_FALSE(link.open(fake->device()).has_value());
  ASSERT_TRUE(wait_frames(3));

  fake->inject_bytes({0x00, 0x13, 0x37, 0xFF, 0x08, 0x00, 0x01});
  const auto before = link.frame_count();
  ASSERT_TRUE(link.wait_for_frame(before + 2, 1s).has_value());  // the stream carries on

  EXPECT_GE(link.counters().framing_errors, 1U);
}

TEST_F(MasterLinkTest, ACorruptedFrameIsDroppedAndTheNextOneArrives)
{
  ASSERT_FALSE(link.open(fake->device()).has_value());
  ASSERT_TRUE(wait_frames(3));

  fake->corrupt_next_telemetry();
  const auto before = link.frame_count();
  ASSERT_TRUE(link.wait_for_frame(before + 3, 1s).has_value());

  EXPECT_GE(link.counters().framing_errors, 1U);
  EXPECT_FALSE(link.master_reset_seen());
}

TEST_F(MasterLinkTest, ADuplicateFrameIsCountedAndNotPublishedAgain)
{
  ASSERT_FALSE(link.open(fake->device()).has_value());
  ASSERT_TRUE(wait_frames(3));

  fake->repeat_last_telemetry();
  ASSERT_TRUE(wait_frames(3));

  EXPECT_EQ(link.counters().duplicate_frames, 1U);
  EXPECT_FALSE(link.master_reset_seen());  // a repeat is not a reset
}

TEST_F(MasterLinkTest, ARestartedMasterIsDetectedFromItsCycleCounterRunningBackwards)
{
  ASSERT_FALSE(link.open(fake->device()).has_value());
  ASSERT_TRUE(wait_frames(20));
  EXPECT_FALSE(link.master_reset_seen());

  fake->restart();
  ASSERT_TRUE(wait_frames(3));

  EXPECT_TRUE(link.master_reset_seen());
  link.acknowledge_master_reset();
  ASSERT_TRUE(wait_frames(3));
  EXPECT_FALSE(link.master_reset_seen());  // frames after the restart are accepted normally
}

TEST_F(MasterLinkTest, ReportsTheVersionOfAMasterOnAnotherProtocol)
{
  fake.reset();
  FakeMaster::Config config;
  config.status_version = 7;
  fake = std::make_unique<FakeMaster>(config);
  fake->start();

  ASSERT_FALSE(link.open(fake->device()).has_value());
  EXPECT_FALSE(link.wait_for_status(300ms).has_value());
  EXPECT_EQ(link.foreign_version(), 7);
  EXPECT_GE(link.counters().framing_errors, 1U);
}

TEST_F(MasterLinkTest, ReportsAnUnpluggedDevice)
{
  ASSERT_FALSE(link.open(fake->device()).has_value());
  ASSERT_TRUE(wait_frames(3));

  fake->unplug();
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!link.failed() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(link.failed());

  // Nothing newer than the last frame can arrive, and the wait must say so at once rather than
  // sleeping to its deadline.
  const auto latest = link.wait_for_frame(link.frame_count() - 1, 100ms);
  ASSERT_TRUE(latest.has_value());
  const auto start = std::chrono::steady_clock::now();
  const auto result =
    link.wait_for_newer(latest->robot.header.cycle_id, start + 1s);
  EXPECT_FALSE(result.has_value());
  EXPECT_LT(std::chrono::steady_clock::now() - start, 200ms);
}

TEST_F(MasterLinkTest, CloseStopsTheReaderPromptlyAndAllowsReopening)
{
  ASSERT_FALSE(link.open(fake->device()).has_value());
  ASSERT_TRUE(wait_frames(3));

  const auto start = std::chrono::steady_clock::now();
  link.close();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 200ms);

  ASSERT_FALSE(link.open(fake->device()).has_value());
  EXPECT_TRUE(link.wait_for_frame(0, 1s).has_value());
}

}  // namespace
