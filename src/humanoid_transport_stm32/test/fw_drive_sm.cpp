// Copyright 2026 UTRA-RoboSoccer
// Isolated translation unit: the ONLY place the firmware protocol.h (and its macros) is included
// on the soccer-software side. It calls the vendored, pure firmware drive state machine and copies
// its decisions into the plain structs of fw_drive_sm.hpp. No wire_protocol.hpp here.
#include "fw_drive_sm.hpp"

#include "drive_sm.h"      // NOLINT(build/include_subdir) generated/firmware_sm, this TU only
#include "motor_config.h"  // NOLINT(build/include_subdir) firmware timing, vendored at the pin

namespace humanoid::transport_stm32::testing::fw
{

ReqDecision on_request(
  std::uint8_t state, std::uint8_t req, std::uint8_t wound, std::uint8_t fault_latched) noexcept
{
  const DriveReqDecision d = drive_sm_on_request(state, req, wound, fault_latched);
  return {d.next_state, d.rejected, d.send_disable, d.send_clear, d.begin_enable, d.begin_stop,
    d.abort_enable, d.latch_fault, d.clear_fault, d.reset_cmd_seq};
}

FaultDecision on_fault(
  std::uint8_t state, std::uint8_t fault, std::uint8_t fault_latched) noexcept
{
  const DriveFaultDecision d = drive_sm_on_fault(state, fault, fault_latched);
  return {d.next_state, d.latch_fault, d.send_disable, d.begin_stop, d.abort_enable};
}

TickDecision on_tick(std::uint8_t state, const TickInputs & in) noexcept
{
  DriveTickInputs fin{};
  fin.run_seen = in.run_seen;
  fin.enable_timeout = in.enable_timeout;
  fin.clear_reply = in.clear_reply;
  fin.clear_dirty = in.clear_dirty;
  fin.clear_timeout = in.clear_timeout;
  fin.stop_done = in.stop_done;
  fin.fault_latched = in.fault_latched;
  fin.fb_timeout = in.fb_timeout;
  fin.fb_fresh = in.fb_fresh;
  const DriveTickDecision d = drive_sm_on_tick(state, &fin);
  return {d.next_state, d.latch_fault, d.send_disable, d.enter_enabled};
}

Timing timing() noexcept
{
  return {
    MOTOR_CAN_FB_TIMEOUT_MS,
    MOTOR_ENABLE_TIMEOUT_MS,
    MOTOR_CLEAR_TIMEOUT_MS,
    MASTER_LOST_DAMP_MS,
    HOST_LOST_CYCLES,
  };
}

EnumValues enum_values() noexcept
{
  return {
    DRV_BOOT, DRV_DISCOVERING, DRV_ABSENT, DRV_DISABLED, DRV_ENABLING, DRV_ENABLED, DRV_STOPPING,
    DRV_CLEARING, DRV_FAULT,
    DRVF_NONE, DRVF_CAN_TIMEOUT, DRVF_MASTER_LOST, DRVF_ENABLE_FAILED, DRVF_NOT_ENABLED,
    DRVF_OVERTORQUE, DRVF_MOTOR_REPORTED, DRVF_WOUND,
    DRV_REQ_DISABLE, DRV_REQ_ENABLE, DRV_REQ_STOP, DRV_REQ_RESET,
    TELE_FLAG_REQUEST_REJECTED,
  };
}

}  // namespace humanoid::transport_stm32::testing::fw
