// Copyright 2026 UTRA-RoboSoccer
// A software stand-in for the master STM32, on a pseudo-terminal, for testing the host side.
//
// It speaks PROTO_VERSION 8 and reproduces the firmware behaviour the transport depends on: the
// per-motor mode state machine (HOLD is the only arming request; a faulted motor rejects armed
// requests until reset), the double-buffered command mailbox with its apply-at-next-cycle delay,
// the host-death watchdog (damped, then idle, latched until a fault reset), and a telemetry frame
// per cycle that omits a chain that is not answering.
//
// It also reproduces how the slave knows where a motor is (motor_runtime.c at soccer-firmware
// 218126a), flaws included, because the transport must not depend on them. The slave polls an
// IDLE or armed motor every tick and a FAULT motor never, so a faulted motor's reported position
// and feedback age go stale. HOLD captures the reported position. A motor the slave did not
// discover at boot is FAULT with cause NONE, and HOLD on it is rejected from IDLE. HOLD with a
// fault reset on a FAULT motor skips that check and the wound check, and arms at the stale
// position. An armed motor whose drive does not answer faults with CAN_TIMEOUT after 100 ms.
//
// What this is NOT: hardware-in-the-loop evidence. ADR-007 rejects a host-side loopback as the H1
// emulator because it leaves the USB stack, cdc_acm, and real timing untested. It tests the
// parser, the lifecycle, and the fault handling, and it says nothing about timing.
#ifndef FAKE_MASTER_HPP_
#define FAKE_MASTER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "humanoid_transport_stm32/serial_port.hpp"
#include "humanoid_transport_stm32/wire_protocol.hpp"

namespace humanoid::transport_stm32::testing
{

class FakeMaster
{
public:
  struct Config
  {
    /// Firmware chain ids that exist, and how many motors each carries.
    std::vector<std::uint8_t> chain_ids{0};
    std::vector<std::uint8_t> motors_per_chain{2};
    std::uint16_t poll_hz{200};
    std::uint16_t telemetry_hz{200};
    std::chrono::microseconds cycle{5000};
    /// Cycles a chain goes silent while one of its motors arms: the firmware's arm handshake
    /// blocks the slave for about 40 ms, during which the master gets no valid telemetry from it.
    int arming_silence_cycles{0};
    /// The protocol version byte stamped on MASTER_STATUS frames.
    std::uint8_t status_version{wire::kProtocolVersion};
  };

  explicit FakeMaster(Config config);
  ~FakeMaster();

  FakeMaster(const FakeMaster &) = delete;
  FakeMaster & operator=(const FakeMaster &) = delete;

  /// The pseudo-terminal's slave path, to give to the transport as its serial device.
  [[nodiscard]] const std::string & device() const noexcept {return device_;}

  /// Starts the 200 Hz cycle thread.
  void start();

  // --- Observation ---

  [[nodiscard]] std::uint64_t commands_received() const;
  [[nodiscard]] std::optional<wire::CmdRobot> last_command() const;
  [[nodiscard]] wire::Lifecycle state(std::uint8_t chain, std::uint8_t motor) const;
  [[nodiscard]] wire::FaultCause cause(std::uint8_t chain, std::uint8_t motor) const;
  [[nodiscard]] bool host_lost() const;
  /// Where the motor is holding, rad on the wire: the position HOLD captured.
  [[nodiscard]] double hold_position(std::uint8_t chain, std::uint8_t motor) const;
  /// Where the shaft physically is, rad on the wire. An armed motor drives it to its target.
  [[nodiscard]] double shaft_position(std::uint8_t chain, std::uint8_t motor) const;

  // --- Scenario and fault injection ---

  void set_silent(bool silent);
  void drop_chain(std::uint8_t chain, bool dropped);
  /// Raw bytes written to the host before the next telemetry frame.
  void inject_bytes(std::vector<std::uint8_t> bytes);
  /// Sends the most recent telemetry frame a second time.
  void repeat_last_telemetry();
  /// Flips one payload byte of the next telemetry frame, so its CRC fails.
  void corrupt_next_telemetry();
  /// A master reset: cycle counter and clock restart, every motor is disarmed.
  void restart();
  void set_wound(std::uint8_t chain, std::uint8_t motor, bool wound);
  void set_fb_age(std::uint8_t chain, std::uint8_t motor, std::uint8_t age_ms);
  void force_fault(std::uint8_t chain, std::uint8_t motor, wire::FaultCause cause);
  /// Moves the shaft and sets what the drive measures, in SI units. The slave sees it only while
  /// it polls the motor. An armed motor's shaft follows its target instead.
  void set_measured(
    std::uint8_t chain, std::uint8_t motor, double pos_rad, double vel_rad_s, double tau_nm);
  /// The slave did not find this motor at boot: FAULT, cause NONE, never polled until reset, and
  /// never armable from IDLE.
  void set_undiscovered(std::uint8_t chain, std::uint8_t motor);
  /// Whether the motor's drive answers on CAN (powered and on the bus).
  void set_drive_answers(std::uint8_t chain, std::uint8_t motor, bool answers);
  /// Closes the pseudo-terminal, as an unplugged USB cable would.
  void unplug();

private:
  struct Motor
  {
    wire::Lifecycle state{wire::Lifecycle::kIdle};
    wire::FaultCause cause{wire::FaultCause::kNone};
    /// What the slave last heard from the drive: what telemetry reports and what HOLD captures.
    std::int16_t pos{0};
    std::int16_t vel{0};
    std::int16_t tau{0};
    /// Where the shaft really is. Copied into `pos` whenever the slave polls the motor.
    std::int16_t shaft{0};
    std::int16_t hold{0};
    std::uint16_t last_applied{0};
    /// The feedback age while the slave polls the motor and the drive answers.
    std::uint8_t fb_age{2};
    /// Milliseconds since the drive last answered, while it does not.
    std::uint32_t unanswered_ms{0};
    /// Milliseconds since the motor armed, for the CAN timeout's grace.
    std::uint32_t armed_ms{0};
    std::uint8_t motor_fault{0};
    std::uint8_t flags{0};
    bool wound{false};
    bool discovered{true};
    bool drive_answers{true};
  };

  struct Chain
  {
    std::uint8_t id{0};
    std::vector<Motor> motors;
    bool dropped{false};
    int silent_cycles{0};
  };

  enum class HostLink : std::uint8_t {kOk, kDamped, kIdle};

  void run();
  void cycle();
  void read_host();
  void apply_requests();
  void apply_request(Chain & chain, Motor & motor, const wire::CmdMotor & request);
  void poll_drives();
  void step_host_watchdog(bool fresh, bool fault_reset);
  void emit_telemetry();
  void emit_status();
  void write_host(const std::vector<std::uint8_t> & bytes);
  Motor & motor_at(std::uint8_t chain, std::uint8_t motor);
  const Motor & motor_at(std::uint8_t chain, std::uint8_t motor) const;
  [[nodiscard]] bool any_armed() const;

  Config config_;
  std::string device_;
  UniqueFd master_fd_;
  UniqueFd slave_fd_;  // held open so the master never sees a hang-up, and to set raw mode

  mutable std::mutex mutex_;
  std::thread thread_;
  std::atomic<bool> stop_{false};

  std::vector<Chain> chains_;
  wire::FrameScanner scanner_;
  std::optional<wire::CmdRobot> pending_;
  std::optional<wire::CmdRobot> active_;
  std::uint64_t commands_received_{0};
  std::uint16_t cmd_seq_active_{0};
  std::uint16_t cmd_seq_rx_{0};
  std::uint32_t cycles_since_fresh_{0};
  HostLink host_link_{HostLink::kOk};
  std::uint32_t damped_cycles_{0};

  std::uint16_t cycle_id_{0};
  std::uint32_t master_time_us_{0};
  std::uint16_t tx_seq_{0};
  int status_divider_{0};

  bool silent_{false};
  bool corrupt_next_{false};
  bool repeat_last_{false};
  std::vector<std::uint8_t> injected_;
  std::vector<std::uint8_t> last_telemetry_frame_;
};

}  // namespace humanoid::transport_stm32::testing

#endif  // FAKE_MASTER_HPP_
