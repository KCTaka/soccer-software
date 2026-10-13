// Copyright 2026 UTRA-RoboSoccer
// A clean C++ wrapper around the firmware's pure drive state machine (generated/firmware_sm/
// drive_sm.h), so the fake master runs the REAL decision logic instead of a hand-written mirror.
//
// drive_sm.h includes the firmware protocol.h, whose macros (PROTO_VERSION, MAX_CHAINS, ...) would
// clobber the C++ wire wrapper if both landed in one translation unit. So the firmware header is
// included only in fw_drive_sm.cpp; this header exposes field-for-field copies of its decision
// structs (plain data -- the LOGIC stays in the vendored firmware code) in terms of std::uint8_t,
// which match the drive-state/request/fault values in wire:: (robot_wire enum class) by value.
#ifndef FW_DRIVE_SM_HPP_
#define FW_DRIVE_SM_HPP_

#include <cstdint>

namespace humanoid::transport_stm32::testing::fw
{

/// Mirror of DriveReqDecision. At most one send_* / begin_* is set.
struct ReqDecision
{
  std::uint8_t next_state;
  std::uint8_t rejected;
  std::uint8_t send_disable;
  std::uint8_t send_clear;
  std::uint8_t begin_enable;
  std::uint8_t begin_stop;
  std::uint8_t abort_enable;
  std::uint8_t latch_fault;
  std::uint8_t clear_fault;
  std::uint8_t reset_cmd_seq;
};

/// Mirror of DriveFaultDecision.
struct FaultDecision
{
  std::uint8_t next_state;
  std::uint8_t latch_fault;
  std::uint8_t send_disable;
  std::uint8_t begin_stop;
  std::uint8_t abort_enable;
};

/// Mirror of DriveTickInputs.
struct TickInputs
{
  std::uint8_t run_seen;
  std::uint8_t enable_timeout;
  std::uint8_t clear_reply;
  std::uint8_t clear_dirty;
  std::uint8_t clear_timeout;
  std::uint8_t stop_done;
  std::uint8_t fault_latched;
  std::uint8_t fb_timeout;
  std::uint8_t fb_fresh;
};

/// Mirror of DriveTickDecision.
struct TickDecision
{
  std::uint8_t next_state;
  std::uint8_t latch_fault;
  std::uint8_t send_disable;
  std::uint8_t enter_enabled;
};

/// The three firmware entry points (drive_sm_on_request / _on_fault / _on_tick), verbatim.
[[nodiscard]] ReqDecision on_request(
  std::uint8_t state, std::uint8_t req, std::uint8_t wound, std::uint8_t fault_latched) noexcept;
[[nodiscard]] FaultDecision on_fault(
  std::uint8_t state, std::uint8_t fault, std::uint8_t fault_latched) noexcept;
[[nodiscard]] TickDecision on_tick(std::uint8_t state, const TickInputs & in) noexcept;

/// The firmware timing constants (firmware motor_config.h, generated from the robot config and
/// vendored at the pin), so the fake master's durations are the firmware's, not hand-copied numbers
/// that drift. All in milliseconds except host_lost_cycles (master cycles).
struct Timing
{
  std::uint32_t can_fb_timeout_ms;    ///< MOTOR_CAN_FB_TIMEOUT_MS: DISABLED->ABSENT, armed CAN loss
  std::uint32_t enable_timeout_ms;    ///< MOTOR_ENABLE_TIMEOUT_MS: ENABLING -> ENABLE_FAILED
  std::uint32_t clear_timeout_ms;     ///< MOTOR_CLEAR_TIMEOUT_MS: CLEARING -> ABSENT
  std::uint32_t stop_ramp_ms;         ///< MASTER_LOST_DAMP_MS: STOPPING ramp dwell before DISABLED
  std::uint32_t host_lost_cycles;     ///< HOST_LOST_CYCLES: master cycles with no fresh command
};
[[nodiscard]] Timing timing() noexcept;

/// The numeric values the firmware gives DriveState / DriveFault / DriveReq and the one tele flag
/// the fake master sets. The shim passes these as plain uint8_t assumed equal to wire::'s enum
/// class values; fw_boundary_test compares each against wire:: so a future shim edit that maps one
/// wrong, or a firmware/schema divergence, is caught. Both sides come from the same pin.
struct EnumValues
{
  std::uint8_t drv_boot, drv_discovering, drv_absent, drv_disabled, drv_enabling, drv_enabled,
    drv_stopping, drv_clearing, drv_fault;
  std::uint8_t drvf_none, drvf_can_timeout, drvf_master_lost, drvf_enable_failed, drvf_not_enabled,
    drvf_overtorque, drvf_motor_reported, drvf_wound;
  std::uint8_t req_disable, req_enable, req_stop, req_reset;
  std::uint8_t tele_flag_request_rejected;
};
[[nodiscard]] EnumValues enum_values() noexcept;

}  // namespace humanoid::transport_stm32::testing::fw

#endif  // FW_DRIVE_SM_HPP_
