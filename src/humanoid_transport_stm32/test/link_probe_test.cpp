// Copyright 2026 UTRA-RoboSoccer
// The read-only probe, against the fake master.

#include <gtest/gtest.h>

#include <chrono>
#include <sstream>
#include <string>

#include "fake_master.hpp"
#include "humanoid_transport_stm32/link_probe.hpp"

namespace
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using humanoid::transport_stm32::probe_master;
using humanoid::transport_stm32::testing::FakeMaster;

TEST(LinkProbe, ReportsAHealthyMasterAndSendsNothing)
{
  FakeMaster::Config config;
  config.chain_ids = {0, 1};
  config.motors_per_chain = {2, 1};
  FakeMaster fake(config);
  fake.start();

  std::ostringstream out;
  EXPECT_TRUE(probe_master(fake.device(), 1500ms, out)) << out.str();

  const std::string report = out.str();
  EXPECT_NE(report.find("protocol version 8 (matches)"), std::string::npos) << report;
  EXPECT_NE(report.find("polls at 200 Hz, telemetry 200 Hz"), std::string::npos) << report;
  EXPECT_NE(report.find("= 200.0 Hz"), std::string::npos) << report;  // the fake's 5 ms cycle
  EXPECT_NE(report.find("chain 0: 2 motor(s)"), std::string::npos) << report;
  EXPECT_NE(report.find("chain 1: 1 motor(s)"), std::string::npos) << report;
  EXPECT_NE(report.find("IDLE"), std::string::npos) << report;
  EXPECT_NE(report.find("OK"), std::string::npos) << report;

  // The one guarantee that matters for first contact with a live robot.
  EXPECT_EQ(fake.commands_received(), 0U);
}

TEST(LinkProbe, NamesAMasterOnAnotherProtocolVersion)
{
  FakeMaster::Config config;
  config.status_version = 7;
  FakeMaster fake(config);
  fake.start();

  std::ostringstream out;
  EXPECT_FALSE(probe_master(fake.device(), 400ms, out));
  EXPECT_NE(out.str().find("speaks protocol version 7"), std::string::npos) << out.str();
}

TEST(LinkProbe, FailsClearlyOnAMissingDevice)
{
  std::ostringstream out;
  EXPECT_FALSE(probe_master("/dev/definitely-not-a-serial-port", 100ms, out));
  EXPECT_NE(out.str().find("FAIL"), std::string::npos);
}

TEST(LinkProbe, FailsClearlyWhenTheMasterIsSilent)
{
  FakeMaster fake(FakeMaster::Config{});  // never started
  std::ostringstream out;
  EXPECT_FALSE(probe_master(fake.device(), 200ms, out));
  EXPECT_NE(out.str().find("no MASTER_STATUS"), std::string::npos) << out.str();
}

}  // namespace
