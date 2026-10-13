// Copyright 2026 UTRA-RoboSoccer
// Guards the shim boundary. fw_drive_sm.hpp hands the fake master the firmware drive SM's decisions
// as plain uint8_t values, ASSUMED equal to the wire:: enum-class values. Both sides come from the
// same pinned firmware tag, so this passes; it exists to catch a future shim edit that maps a field
// to the wrong enum, or a firmware/schema divergence that the SCHEMA_HASH alone would not surface
// (the drive SM compiles against the C protocol.h, the host against the generated C++ header).

#include <gtest/gtest.h>

#include <cstdint>

#include "humanoid_transport_stm32/wire_protocol.hpp"

#include "fw_drive_sm.hpp"

namespace
{

namespace wire = humanoid::transport_stm32::wire;
namespace fw = humanoid::transport_stm32::testing::fw;

template<typename Enum>
std::uint8_t v(Enum e)
{
  return static_cast<std::uint8_t>(e);
}

TEST(FwBoundary, DriveStateValuesAgreeAcrossTheShim)
{
  const auto f = fw::enum_values();
  EXPECT_EQ(f.drv_boot, v(wire::DriveState::DRV_BOOT));
  EXPECT_EQ(f.drv_discovering, v(wire::DriveState::DRV_DISCOVERING));
  EXPECT_EQ(f.drv_absent, v(wire::DriveState::DRV_ABSENT));
  EXPECT_EQ(f.drv_disabled, v(wire::DriveState::DRV_DISABLED));
  EXPECT_EQ(f.drv_enabling, v(wire::DriveState::DRV_ENABLING));
  EXPECT_EQ(f.drv_enabled, v(wire::DriveState::DRV_ENABLED));
  EXPECT_EQ(f.drv_stopping, v(wire::DriveState::DRV_STOPPING));
  EXPECT_EQ(f.drv_clearing, v(wire::DriveState::DRV_CLEARING));
  EXPECT_EQ(f.drv_fault, v(wire::DriveState::DRV_FAULT));
}

TEST(FwBoundary, DriveFaultValuesAgreeAcrossTheShim)
{
  const auto f = fw::enum_values();
  EXPECT_EQ(f.drvf_none, v(wire::DriveFault::DRVF_NONE));
  EXPECT_EQ(f.drvf_can_timeout, v(wire::DriveFault::DRVF_CAN_TIMEOUT));
  EXPECT_EQ(f.drvf_master_lost, v(wire::DriveFault::DRVF_MASTER_LOST));
  EXPECT_EQ(f.drvf_enable_failed, v(wire::DriveFault::DRVF_ENABLE_FAILED));
  EXPECT_EQ(f.drvf_not_enabled, v(wire::DriveFault::DRVF_NOT_ENABLED));
  EXPECT_EQ(f.drvf_overtorque, v(wire::DriveFault::DRVF_OVERTORQUE));
  EXPECT_EQ(f.drvf_motor_reported, v(wire::DriveFault::DRVF_MOTOR_REPORTED));
  EXPECT_EQ(f.drvf_wound, v(wire::DriveFault::DRVF_WOUND));
}

TEST(FwBoundary, DriveReqAndTeleFlagAgreeAcrossTheShim)
{
  const auto f = fw::enum_values();
  EXPECT_EQ(f.req_disable, v(wire::DriveReq::DRV_REQ_DISABLE));
  EXPECT_EQ(f.req_enable, v(wire::DriveReq::DRV_REQ_ENABLE));
  EXPECT_EQ(f.req_stop, v(wire::DriveReq::DRV_REQ_STOP));
  EXPECT_EQ(f.req_reset, v(wire::DriveReq::DRV_REQ_RESET));
  EXPECT_EQ(f.tele_flag_request_rejected, wire::kTeleFlagRequestRejected);
}

}  // namespace
