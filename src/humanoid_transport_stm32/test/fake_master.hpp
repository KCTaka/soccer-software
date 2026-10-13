// Copyright 2026 UTRA-RoboSoccer
// A software stand-in for the master STM32, on a pseudo-terminal, for testing the host side.
//
// It speaks protocol v10 and runs the firmware's REAL decision logic: the per-motor drive state
// machine is the vendored, pure drive_sm.h (via fw_drive_sm.hpp), and the host-death dead-man is
// the vendored host_watchdog.h. The emulator is therefore not a hand-written mirror that can drift
// from the firmware; it contributes only what the master gets from hardware and the wire:
//   - simulated drives: rs_mode moves Reset->Run a few cycles after ENABLE (unless told it never
//     reaches Run); an ENABLED motor's shaft tracks its last valid tuple; feedback age.
//   - presence: a motor whose drive does not answer ages out to ABSENT; a chain can be LOST or
//     MISMATCH, which the master reports in tele_chain_t.link.
//   - wire identity: MasterStatus with a settable schema_hash; robot_state in the telemetry frame.
//   - fault injection (P8.8): CAN timeout, overtorque, motor-reported bits, master loss, and an
//     enable that never reaches Run.
// The cycle runs on its own 200 Hz clock, so the host-death watchdog trips on the master's
// schedule.
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

#include "fw_drive_sm.hpp"

namespace humanoid::transport_stm32::testing
{

class FakeMaster
{
public:
  /// How the 200 Hz cycle is driven. kRealTime runs a wall-clock thread (for the transport
  /// integration tests and SIL); kManual runs no thread and the test calls advance() per cycle, so
  /// cycle-count timing (the host-death trip, ENABLE/CLEAR timeouts, the stop ramp) is
  /// deterministic on a loaded CI runner instead of racing the wall clock.
  enum class ClockMode : std::uint8_t
  {
    kRealTime,
    kManual,
  };

  struct Config
  {
    /// Firmware chain ids that exist, and how many motors each carries.
    std::vector<std::uint8_t> chain_ids{0};
    std::vector<std::uint8_t> motors_per_chain{2};
    std::uint16_t poll_hz{200};
    std::uint16_t telemetry_hz{200};
    std::chrono::microseconds cycle{5000};
    ClockMode clock_mode{ClockMode::kRealTime};
    /// Cycles a chain goes silent while one of its motors enables: the firmware's enable handshake
    /// can leave the master without valid telemetry from that chain for a few cycles.
    int arming_silence_cycles{0};
    /// The protocol version byte stamped on MASTER_STATUS frames.
    std::uint8_t status_version{wire::kProtocolVersion};
    /// The schema hash reported in MasterStatus; a host that pins a different one must refuse.
    std::uint32_t schema_hash{wire::kSchemaHash};
  };

  explicit FakeMaster(Config config);
  ~FakeMaster();

  FakeMaster(const FakeMaster &) = delete;
  FakeMaster & operator=(const FakeMaster &) = delete;

  /// The pseudo-terminal's slave path, to give to the transport as its serial device.
  [[nodiscard]] const std::string & device() const noexcept {return device_;}

  /// Starts the 200 Hz cycle thread (kRealTime). A no-op in kManual mode: drive it with advance().
  void start();

  /// Runs `cycles` master cycles synchronously, for a kManual clock. Each cycle reads the host,
  /// advances the drives, and emits telemetry exactly as the real-time thread would. Undefined for
  /// a kRealTime master (the thread already owns the cycle).
  void advance(int cycles = 1);

  /// Hands a decoded command to the mailbox as if it had arrived framed over the wire, for a
  /// kManual test that does not run the transport. Applied on the next advance(), like a wire one.
  void deliver_command(const wire::CmdRobot & command);

  // --- Observation ---

  [[nodiscard]] std::uint64_t commands_received() const;
  [[nodiscard]] std::optional<wire::CmdRobot> last_command() const;
  [[nodiscard]] wire::DriveState state(std::uint8_t chain, std::uint8_t motor) const;
  [[nodiscard]] wire::DriveFault fault(std::uint8_t chain, std::uint8_t motor) const;
  /// The robot_state the master stamped on its most recent telemetry frame.
  [[nodiscard]] wire::RobotState robot_state() const;
  [[nodiscard]] bool host_lost() const;
  /// Where the motor is holding, rad on the wire: the position the last valid tuple set.
  [[nodiscard]] double hold_position(std::uint8_t chain, std::uint8_t motor) const;
  /// Where the shaft physically is, rad on the wire. An ENABLED motor drives it to its target.
  [[nodiscard]] double shaft_position(std::uint8_t chain, std::uint8_t motor) const;

  // --- Scenario and fault injection ---

  void set_silent(bool silent);
  void drop_chain(std::uint8_t chain, bool dropped);
  /// The chain answers but with the wrong proto/schema: the master reports link == MISMATCH.
  void set_chain_mismatch(std::uint8_t chain, bool mismatch);
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
  /// Inject a detected fault, routed through the firmware drive SM (drive_sm_on_fault).
  void force_fault(std::uint8_t chain, std::uint8_t motor, wire::DriveFault fault);
  /// Moves the shaft and sets what the drive measures, in SI units. The slave sees it only while
  /// it polls the motor. An ENABLED motor's shaft follows its target instead.
  void set_measured(
    std::uint8_t chain, std::uint8_t motor, double pos_rad, double vel_rad_s, double tau_nm);
  /// The slave did not find this motor at boot: it never answers, so it ages to ABSENT and an
  /// ENABLE is refused (ABSENT rejects it).
  void set_undiscovered(std::uint8_t chain, std::uint8_t motor);
  /// Whether the motor's drive answers on CAN (powered and on the bus).
  void set_drive_answers(std::uint8_t chain, std::uint8_t motor, bool answers);
  /// The drive answers but never reports rs_mode == Run, so ENABLING times out to ENABLE_FAILED.
  void set_enable_never_runs(std::uint8_t chain, std::uint8_t motor, bool never);
  /// Closes the pseudo-terminal, as an unplugged USB cable would.
  void unplug();

private:
  struct Motor
  {
    std::uint8_t drive_state{static_cast<std::uint8_t>(wire::DriveState::DRV_DISABLED)};
    std::uint8_t drive_fault{static_cast<std::uint8_t>(wire::DriveFault::DRVF_NONE)};
    std::uint8_t rs_mode{0};  // RobStride Type-2: 0 reset, 2 run
    /// What the slave last heard from the drive: what telemetry reports and what a tuple captures.
    std::int16_t pos{0};
    std::int16_t vel{0};
    std::int16_t tau{0};
    /// Where the shaft really is. Copied into `pos` whenever the slave polls the motor.
    std::int16_t shaft{0};
    /// The latest commanded hold/target (an ENABLED motor drives the shaft here).
    std::int16_t hold{0};
    bool have_tuple{false};
    std::uint16_t last_applied{0};
    /// The feedback age while the slave polls the motor and the drive answers.
    std::uint8_t fb_age{2};
    /// Milliseconds since the drive last answered, while it does not.
    std::uint32_t unanswered_ms{0};
    /// Milliseconds in the current transient state (ENABLING / CLEARING / STOPPING).
    std::uint32_t state_ms{0};
    std::uint8_t motor_fault{0};
    std::uint8_t flags{0};
    bool wound{false};
    bool drive_answers{true};
    bool enable_never_runs{false};
  };

  struct Chain
  {
    std::uint8_t id{0};
    std::vector<Motor> motors;
    bool dropped{false};
    bool mismatch{false};
    int silent_cycles{0};
  };

  void run();
  void cycle();
  void read_host();
  void apply_requests();
  void apply_request(Chain & chain, Motor & motor, const wire::CmdMotor & request);
  void poll_drives();
  void emit_telemetry();
  void emit_status();
  void write_host(const std::vector<std::uint8_t> & bytes);
  Motor & motor_at(std::uint8_t chain, std::uint8_t motor);
  const Motor & motor_at(std::uint8_t chain, std::uint8_t motor) const;
  [[nodiscard]] bool any_armed() const;
  [[nodiscard]] bool any_mismatch() const;
  void latch_fault(Motor & motor, std::uint8_t fault);

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
  std::uint8_t host_link_{0};  // firmware HostLinkAction: 0 = OK, 1 = STOPPED (host lost)

  std::uint16_t cycle_id_{0};
  std::uint32_t master_time_us_{0};
  std::uint16_t tx_seq_{0};
  int status_divider_{0};
  wire::RobotState last_robot_state_{wire::RobotState::ROBOT_INIT};

  bool silent_{false};
  bool corrupt_next_{false};
  bool repeat_last_{false};
  std::vector<std::uint8_t> injected_;
  std::vector<std::uint8_t> last_telemetry_frame_;
};

}  // namespace humanoid::transport_stm32::testing

#endif  // FAKE_MASTER_HPP_
