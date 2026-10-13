// Copyright 2026 UTRA-RoboSoccer

#include "fake_master.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "host_watchdog.h"  // NOLINT(build/include_subdir) generated/firmware_sm, macro-free

namespace humanoid::transport_stm32::testing
{

namespace
{

// All firmware timing (enable/clear/stop-ramp durations, the host-death cycle count) comes from the
// vendored firmware motor_config.h through the shim, not hand-copied numbers that drift.
const fw::Timing kTiming = fw::timing();
// How fast the SIMULATED drive reports rs_mode == Run after an ENABLE. This is the fake's own
// hardware model, not a firmware constant; it only has to be below enable_timeout_ms.
constexpr std::uint32_t kEnableRunMs = 10;
// The wire saturates fb_age_ms (a u8) at 255 ms, which also means "never heard from".
constexpr std::uint32_t kWireAgeSaturationMs = 255;

// The master's version gate (firmware spi_gate.h spi_gate_filter_req): while any slave fails the
// gate, a fresh ENABLE for a not-yet-armed motor is rewritten to DISABLE robot-wide; armed motors
// are left alone. Hand-emulated here because it is master logic, not the vendored drive SM.
std::uint8_t gate_filter_req(std::uint8_t drive_req, std::uint8_t drive_state) noexcept
{
  if (drive_req != static_cast<std::uint8_t>(wire::DriveReq::DRV_REQ_ENABLE)) {
    return drive_req;
  }
  const bool armed =
    drive_state == static_cast<std::uint8_t>(wire::DriveState::DRV_ENABLING) ||
    drive_state == static_cast<std::uint8_t>(wire::DriveState::DRV_ENABLED) ||
    drive_state == static_cast<std::uint8_t>(wire::DriveState::DRV_STOPPING);
  return armed ? drive_req : static_cast<std::uint8_t>(wire::DriveReq::DRV_REQ_DISABLE);
}

constexpr std::uint8_t kReqDisable = static_cast<std::uint8_t>(wire::DriveReq::DRV_REQ_DISABLE);
constexpr std::uint8_t kReqEnable = static_cast<std::uint8_t>(wire::DriveReq::DRV_REQ_ENABLE);
constexpr std::uint8_t kReqStop = static_cast<std::uint8_t>(wire::DriveReq::DRV_REQ_STOP);
constexpr std::uint8_t kReqReset = static_cast<std::uint8_t>(wire::DriveReq::DRV_REQ_RESET);

constexpr std::uint8_t kStEnabling = static_cast<std::uint8_t>(wire::DriveState::DRV_ENABLING);
constexpr std::uint8_t kStEnabled = static_cast<std::uint8_t>(wire::DriveState::DRV_ENABLED);
constexpr std::uint8_t kStStopping = static_cast<std::uint8_t>(wire::DriveState::DRV_STOPPING);
constexpr std::uint8_t kStClearing = static_cast<std::uint8_t>(wire::DriveState::DRV_CLEARING);
constexpr std::uint8_t kStDisabled = static_cast<std::uint8_t>(wire::DriveState::DRV_DISABLED);
constexpr std::uint8_t kStAbsent = static_cast<std::uint8_t>(wire::DriveState::DRV_ABSENT);

bool is_armed_state(std::uint8_t state) noexcept
{
  return state == kStEnabling || state == kStEnabled || state == kStStopping;
}

bool request_carries_reset(const wire::CmdRobot & command) noexcept
{
  const std::size_t chain_count = std::min<std::size_t>(command.n_chains, wire::kMaxChains);
  for (std::size_t c = 0; c < chain_count; ++c) {
    const std::size_t motors =
      std::min<std::size_t>(command.chains[c].n_motors, wire::kMaxMotorsPerChain);
    for (std::size_t m = 0; m < motors; ++m) {
      if (command.chains[c].motors[m].drive_req == kReqReset) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

FakeMaster::FakeMaster(Config config)
: config_(std::move(config))
{
  const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
  if (master < 0 || ::grantpt(master) != 0 || ::unlockpt(master) != 0) {
    throw std::runtime_error("FakeMaster: cannot create a pseudo-terminal");
  }
  master_fd_.reset(master);
  ::fcntl(master, F_SETFL, ::fcntl(master, F_GETFL) | O_NONBLOCK);

  std::array<char, 128> name{};
  if (::ptsname_r(master, name.data(), name.size()) != 0) {
    throw std::runtime_error("FakeMaster: no pseudo-terminal slave name");
  }
  device_ = name.data();

  slave_fd_.reset(::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC));
  termios tty{};
  if (!slave_fd_.valid() || ::tcgetattr(slave_fd_.get(), &tty) != 0) {
    throw std::runtime_error("FakeMaster: cannot open the pseudo-terminal slave");
  }
  ::cfmakeraw(&tty);
  ::tcsetattr(slave_fd_.get(), TCSANOW, &tty);

  for (std::size_t i = 0; i < config_.chain_ids.size(); ++i) {
    Chain chain;
    chain.id = config_.chain_ids[i];
    chain.motors.resize(config_.motors_per_chain.at(i));
    chains_.push_back(std::move(chain));
  }
}

FakeMaster::~FakeMaster()
{
  stop_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void FakeMaster::start()
{
  if (config_.clock_mode == ClockMode::kManual) {
    return;  // the test owns the clock; advance() runs each cycle.
  }
  thread_ = std::thread([this] {run();});
}

void FakeMaster::advance(int cycles)
{
  for (int i = 0; i < cycles; ++i) {
    cycle();
  }
}

void FakeMaster::deliver_command(const wire::CmdRobot & command)
{
  const std::scoped_lock lock{mutex_};
  pending_ = command;
  cmd_seq_rx_ = command.cmd_seq;
  ++commands_received_;
}

// ===========================================================================
// Observation
// ===========================================================================

std::uint64_t FakeMaster::commands_received() const
{
  const std::scoped_lock lock{mutex_};
  return commands_received_;
}

std::optional<wire::CmdRobot> FakeMaster::last_command() const
{
  const std::scoped_lock lock{mutex_};
  return active_ ? active_ : pending_;
}

wire::DriveState FakeMaster::state(std::uint8_t chain, std::uint8_t motor) const
{
  const std::scoped_lock lock{mutex_};
  return static_cast<wire::DriveState>(motor_at(chain, motor).drive_state);
}

wire::DriveFault FakeMaster::fault(std::uint8_t chain, std::uint8_t motor) const
{
  const std::scoped_lock lock{mutex_};
  return static_cast<wire::DriveFault>(motor_at(chain, motor).drive_fault);
}

wire::RobotState FakeMaster::robot_state() const
{
  const std::scoped_lock lock{mutex_};
  return last_robot_state_;
}

bool FakeMaster::host_lost() const
{
  const std::scoped_lock lock{mutex_};
  return host_link_ != static_cast<std::uint8_t>(HOST_LINK_OK);
}

double FakeMaster::hold_position(std::uint8_t chain, std::uint8_t motor) const
{
  const std::scoped_lock lock{mutex_};
  return wire::decode_i16(motor_at(chain, motor).hold, wire::kPosScale);
}

double FakeMaster::shaft_position(std::uint8_t chain, std::uint8_t motor) const
{
  const std::scoped_lock lock{mutex_};
  return wire::decode_i16(motor_at(chain, motor).shaft, wire::kPosScale);
}

// ===========================================================================
// Scenario and fault injection
// ===========================================================================

void FakeMaster::set_silent(bool silent)
{
  const std::scoped_lock lock{mutex_};
  silent_ = silent;
}

void FakeMaster::drop_chain(std::uint8_t chain, bool dropped)
{
  const std::scoped_lock lock{mutex_};
  for (auto & c : chains_) {
    if (c.id == chain) {
      c.dropped = dropped;
    }
  }
}

void FakeMaster::set_chain_mismatch(std::uint8_t chain, bool mismatch)
{
  const std::scoped_lock lock{mutex_};
  for (auto & c : chains_) {
    if (c.id == chain) {
      c.mismatch = mismatch;
    }
  }
}

void FakeMaster::inject_bytes(std::vector<std::uint8_t> bytes)
{
  const std::scoped_lock lock{mutex_};
  injected_.insert(injected_.end(), bytes.begin(), bytes.end());
}

void FakeMaster::repeat_last_telemetry()
{
  const std::scoped_lock lock{mutex_};
  repeat_last_ = true;
}

void FakeMaster::corrupt_next_telemetry()
{
  const std::scoped_lock lock{mutex_};
  corrupt_next_ = true;
}

void FakeMaster::restart()
{
  const std::scoped_lock lock{mutex_};
  cycle_id_ = 0;
  master_time_us_ = 0;
  pending_.reset();
  active_.reset();
  cmd_seq_active_ = 0;
  cmd_seq_rx_ = 0;
  cycles_since_fresh_ = 0;
  host_link_ = static_cast<std::uint8_t>(HOST_LINK_OK);
  for (auto & chain : chains_) {
    chain.silent_cycles = 0;
    for (auto & motor : chain.motors) {
      motor = Motor{};
    }
  }
}

void FakeMaster::set_wound(std::uint8_t chain, std::uint8_t motor, bool wound)
{
  const std::scoped_lock lock{mutex_};
  motor_at(chain, motor).wound = wound;
}

void FakeMaster::set_fb_age(std::uint8_t chain, std::uint8_t motor, std::uint8_t age_ms)
{
  const std::scoped_lock lock{mutex_};
  motor_at(chain, motor).fb_age = age_ms;
}

void FakeMaster::force_fault(std::uint8_t chain, std::uint8_t motor, wire::DriveFault fault)
{
  const std::scoped_lock lock{mutex_};
  Motor & m = motor_at(chain, motor);
  // Route through the firmware's fault reaction so the state transition (ramp vs immediate) is the
  // real one, not an assumption.
  const auto d = fw::on_fault(
    m.drive_state, static_cast<std::uint8_t>(fault),
    static_cast<std::uint8_t>(m.drive_fault !=
    static_cast<std::uint8_t>(wire::DriveFault::DRVF_NONE)));
  if (d.latch_fault != static_cast<std::uint8_t>(wire::DriveFault::DRVF_NONE)) {
    latch_fault(m, d.latch_fault);
  }
  if (d.begin_stop) {
    m.state_ms = 0;
  }
  if (static_cast<std::uint8_t>(fault) ==
    static_cast<std::uint8_t>(wire::DriveFault::DRVF_MOTOR_REPORTED))
  {
    m.motor_fault = 0x20;  // a vendor fault bit, so a subsequent CLEARING reads dirty
  }
  m.drive_state = d.next_state;
}

void FakeMaster::set_measured(
  std::uint8_t chain, std::uint8_t motor, double pos_rad, double vel_rad_s, double tau_nm)
{
  const std::scoped_lock lock{mutex_};
  auto & m = motor_at(chain, motor);
  m.shaft = wire::encode_i16(static_cast<float>(pos_rad), wire::kPosScale).raw;
  m.vel = wire::encode_i16(static_cast<float>(vel_rad_s), wire::kVelScale).raw;
  m.tau = wire::encode_i16(static_cast<float>(tau_nm), wire::kTauScale).raw;
}

void FakeMaster::set_undiscovered(std::uint8_t chain, std::uint8_t motor)
{
  const std::scoped_lock lock{mutex_};
  auto & m = motor_at(chain, motor);
  m.drive_answers = false;
  m.drive_state = kStAbsent;  // never found at boot: ABSENT, and ENABLE is refused from ABSENT
  m.pos = 0;
  m.vel = 0;
  m.tau = 0;
  m.unanswered_ms = kWireAgeSaturationMs;
}

void FakeMaster::set_drive_answers(std::uint8_t chain, std::uint8_t motor, bool answers)
{
  const std::scoped_lock lock{mutex_};
  motor_at(chain, motor).drive_answers = answers;
}

void FakeMaster::set_discovering(std::uint8_t chain, std::uint8_t motor, bool discovering)
{
  const std::scoped_lock lock{mutex_};
  auto & m = motor_at(chain, motor);
  // DISCOVERING is not advanced by drive_sm_on_tick (a boot state the discovery routine owns), so
  // it stays put until cleared -- exactly a slave still coming up.
  m.drive_state = static_cast<std::uint8_t>(
    discovering ? wire::DriveState::DRV_DISCOVERING : wire::DriveState::DRV_DISABLED);
}

void FakeMaster::set_enable_never_runs(std::uint8_t chain, std::uint8_t motor, bool never)
{
  const std::scoped_lock lock{mutex_};
  motor_at(chain, motor).enable_never_runs = never;
}

void FakeMaster::unplug()
{
  const std::scoped_lock lock{mutex_};
  master_fd_.reset();
  slave_fd_.reset();
}

// ===========================================================================
// The cycle thread
// ===========================================================================

void FakeMaster::run()
{
  auto next = std::chrono::steady_clock::now();
  while (!stop_.load()) {
    next += config_.cycle;
    std::this_thread::sleep_until(next);
    if (std::chrono::steady_clock::now() > next + 20 * config_.cycle) {
      next = std::chrono::steady_clock::now();
    }
    cycle();
  }
}

void FakeMaster::cycle()
{
  const std::scoped_lock lock{mutex_};
  read_host();

  ++cycle_id_;
  master_time_us_ += static_cast<std::uint32_t>(config_.cycle.count());

  // The mailbox swap: a command received during one cycle takes effect at the next.
  bool reset_seen = false;
  if (pending_) {
    active_ = pending_;
    cmd_seq_active_ = pending_->cmd_seq;
    reset_seen = request_carries_reset(*pending_);
    pending_.reset();
    cycles_since_fresh_ = 0;
  } else {
    ++cycles_since_fresh_;
  }

  // The host-death dead-man is the vendored firmware watchdog, verbatim.
  HostWatchdog w{static_cast<HostLinkAction>(host_link_)};
  host_watchdog_step(
    &w, cycles_since_fresh_, static_cast<std::uint8_t>(any_armed()),
    static_cast<std::uint8_t>(reset_seen), kTiming.host_lost_cycles);
  host_link_ = static_cast<std::uint8_t>(w.state);

  apply_requests();
  poll_drives();
  emit_telemetry();
  if (++status_divider_ >= 10) {
    status_divider_ = 0;
    emit_status();
  }
}

void FakeMaster::read_host()
{
  if (!master_fd_.valid()) {
    return;
  }
  std::array<std::uint8_t, 1024> buffer{};
  for (;; ) {
    const ssize_t n = ::read(master_fd_.get(), buffer.data(), buffer.size());
    if (n <= 0) {
      break;
    }
    std::span<const std::uint8_t> rest{buffer.data(), static_cast<std::size_t>(n)};
    while (!rest.empty()) {
      const std::size_t accepted = scanner_.push(rest);
      rest = rest.subspan(accepted);
      while (const auto frame = scanner_.next()) {
        if (frame->type == wire::MessageType::MSG_ROBOT_CMD &&
          frame->payload.size() >= sizeof(wire::CmdRobot))
        {
          wire::CmdRobot command;
          std::memcpy(&command, frame->payload.data(), sizeof(command));
          pending_ = command;
          cmd_seq_rx_ = command.cmd_seq;
          ++commands_received_;
        }
      }
      if (accepted == 0) {
        break;
      }
    }
  }
}

void FakeMaster::apply_requests()
{
  if (!active_) {
    return;
  }
  const bool host_lost = host_link_ != static_cast<std::uint8_t>(HOST_LINK_OK);
  const bool gate = any_mismatch();  // version gate open while any slave fails it
  const std::size_t chain_count = std::min<std::size_t>(active_->n_chains, wire::kMaxChains);
  for (std::size_t c = 0; c < chain_count; ++c) {
    const wire::CmdChain & requested = active_->chains[c];
    for (auto & chain : chains_) {
      if (chain.id != requested.chain_id) {
        continue;
      }
      // A slave that fails the version gate is polled with NOPs, never commanded: motors freeze.
      if (chain.mismatch) {
        continue;
      }
      const std::size_t slots =
        std::min<std::size_t>({requested.n_motors, chain.motors.size(), wire::kMaxMotorsPerChain});
      for (std::size_t m = 0; m < slots; ++m) {
        wire::CmdMotor request = requested.motors[m];
        // Host lost: the master forces STOP into every slot (host_watchdog.h), latched until a
        // RESET-bearing command recovers the link.
        if (host_lost) {
          request.drive_req = kReqStop;
          request.flags = 0;
        } else if (gate) {
          // While any slave mismatches, a fresh ENABLE for a not-yet-armed motor is refused
          // robot-wide (spi_gate_filter_req), so nothing arms against a half-known robot.
          request.drive_req = gate_filter_req(request.drive_req, chain.motors[m].drive_state);
        }
        apply_request(chain, chain.motors[m], request);
      }
    }
  }
}

// One motor's drive request, decided by the real firmware drive SM (drive_sm_on_request).
void FakeMaster::apply_request(Chain & /*chain*/, Motor & motor, const wire::CmdMotor & request)
{
  const std::uint8_t fault_latched = static_cast<std::uint8_t>(
    motor.drive_fault != static_cast<std::uint8_t>(wire::DriveFault::DRVF_NONE));
  const auto d = fw::on_request(
    motor.drive_state, request.drive_req, static_cast<std::uint8_t>(motor.wound), fault_latched);

  motor.flags = d.rejected ? wire::kTeleFlagRequestRejected : 0;
  if (d.clear_fault) {
    motor.drive_fault = static_cast<std::uint8_t>(wire::DriveFault::DRVF_NONE);
  }
  if (d.latch_fault != static_cast<std::uint8_t>(wire::DriveFault::DRVF_NONE)) {
    latch_fault(motor, d.latch_fault);
  }
  if (d.begin_enable) {
    motor.state_ms = 0;
    motor.rs_mode = 0;
    motor.have_tuple = false;
  }
  if (d.send_clear || d.begin_stop) {
    motor.state_ms = 0;
  }
  if (d.reset_cmd_seq) {
    motor.last_applied = 0;
  }
  motor.drive_state = d.next_state;

  // The MIT tuple is independent of the request (v10). A VALID tuple latches while ENABLING and is
  // applied on the ENABLING->ENABLED edge; an ENABLED motor executes it immediately.
  const bool valid = (request.flags & wire::kCmdFlagValid) != 0;
  if (valid && (motor.drive_state == kStEnabled || motor.drive_state == kStEnabling)) {
    motor.have_tuple = true;
    motor.hold = request.pos;
    if (motor.drive_state == kStEnabled) {
      motor.pos = request.pos;
      motor.vel = request.vel;
      motor.tau = request.tau_ff;
      motor.shaft = request.pos;
      motor.last_applied = cmd_seq_active_;
    }
  }
}

// The slave's per-tick CAN service: advance the transient states (drive_sm_on_tick), poll the
// drives, and detect a CAN timeout on an armed motor whose drive went silent (drive_sm_on_fault).
void FakeMaster::poll_drives()
{
  const auto cycle_ms = static_cast<std::uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(config_.cycle).count());
  for (auto & chain : chains_) {
    for (auto & motor : chain.motors) {
      const std::uint8_t state = motor.drive_state;
      const bool pollable = state == kStDisabled || is_armed_state(state) || state == kStClearing;

      // The drive's reply (while polled and answering) refreshes what the slave knows.
      if (pollable && motor.drive_answers) {
        if (motor.drive_state == kStEnabled && motor.have_tuple) {
          motor.shaft = motor.hold;  // an ENABLED motor drives the shaft to its target
        }
        motor.pos = motor.shaft;
        motor.unanswered_ms = 0;
      } else {
        motor.unanswered_ms = std::min(motor.unanswered_ms + cycle_ms, kWireAgeSaturationMs);
      }

      // A CAN timeout on an armed motor is a detected fault (comms-broken -> immediate FAULT).
      if (is_armed_state(state) && !motor.drive_answers &&
        motor.unanswered_ms >= kTiming.can_fb_timeout_ms)
      {
        const auto fd = fw::on_fault(
          state, static_cast<std::uint8_t>(wire::DriveFault::DRVF_CAN_TIMEOUT),
          static_cast<std::uint8_t>(
            motor.drive_fault != static_cast<std::uint8_t>(wire::DriveFault::DRVF_NONE)));
        if (fd.latch_fault != static_cast<std::uint8_t>(wire::DriveFault::DRVF_NONE)) {
          latch_fault(motor, fd.latch_fault);
        }
        motor.drive_state = fd.next_state;
        continue;
      }

      if (state == kStEnabling && motor.drive_answers && !motor.enable_never_runs &&
        motor.state_ms >= kEnableRunMs)
      {
        motor.rs_mode = 2;  // Run
      }

      fw::TickInputs in{};
      in.run_seen = static_cast<std::uint8_t>(motor.rs_mode == 2 && state == kStEnabling);
      in.enable_timeout = static_cast<std::uint8_t>(
        state == kStEnabling && motor.state_ms >= kTiming.enable_timeout_ms);
      in.clear_reply = static_cast<std::uint8_t>(
        state == kStClearing && motor.drive_answers && motor.state_ms >= cycle_ms);
      in.clear_dirty = static_cast<std::uint8_t>(in.clear_reply && motor.motor_fault != 0);
      in.clear_timeout = static_cast<std::uint8_t>(
        state == kStClearing && !motor.drive_answers && motor.state_ms >= kTiming.clear_timeout_ms);
      in.stop_done = static_cast<std::uint8_t>(
        state == kStStopping && motor.state_ms >= kTiming.stop_ramp_ms);
      in.fault_latched = static_cast<std::uint8_t>(
        motor.drive_fault != static_cast<std::uint8_t>(wire::DriveFault::DRVF_NONE));
      in.fb_timeout = static_cast<std::uint8_t>(
        state == kStDisabled && !motor.drive_answers &&
        motor.unanswered_ms >= kTiming.can_fb_timeout_ms);
      in.fb_fresh = static_cast<std::uint8_t>(state == kStAbsent && motor.drive_answers);

      const auto td = fw::on_tick(state, in);
      if (td.latch_fault != static_cast<std::uint8_t>(wire::DriveFault::DRVF_NONE)) {
        latch_fault(motor, td.latch_fault);
      }
      if (td.enter_enabled) {
        motor.rs_mode = 2;
        if (motor.have_tuple) {
          motor.shaft = motor.hold;
        }
      }
      if (td.next_state == kStDisabled && state != kStDisabled) {
        motor.rs_mode = 0;  // a stop/clear that reached DISABLED is no longer running
      }
      motor.drive_state = td.next_state;

      if (is_armed_state(motor.drive_state) || motor.drive_state == kStClearing) {
        motor.state_ms += cycle_ms;
      }
    }
  }
}

void FakeMaster::emit_telemetry()
{
  if (!master_fd_.valid()) {
    return;
  }
  if (!injected_.empty()) {
    write_host(injected_);
    injected_.clear();
  }
  if (repeat_last_ && !last_telemetry_frame_.empty()) {
    write_host(last_telemetry_frame_);
    repeat_last_ = false;
  }
  if (silent_) {
    return;
  }

  // Every configured chain is emitted at its slot with a link byte (v10); a chain that is not
  // ALIVE carries no motor data.
  std::vector<std::uint8_t> payload(wire::kTeleHeadSize);
  std::uint8_t emitted = 0;
  std::uint8_t alive_count = 0;
  for (auto & chain : chains_) {
    wire::TeleChain tele{};
    tele.chain_id = chain.id;
    bool alive = true;
    if (chain.silent_cycles > 0) {
      --chain.silent_cycles;
      alive = false;
    }
    if (chain.dropped) {alive = false;}
    if (chain.mismatch) {
      tele.link = static_cast<std::uint8_t>(wire::ChainLink::CHAIN_LINK_MISMATCH);
      alive = false;
    } else {
      tele.link = static_cast<std::uint8_t>(
        alive ? wire::ChainLink::CHAIN_LINK_ALIVE : wire::ChainLink::CHAIN_LINK_LOST);
    }
    if (!alive) {
      tele.n_motors = 0;
    } else {
      ++alive_count;
      tele.n_motors = static_cast<std::uint8_t>(chain.motors.size());
      tele.slave_time_us = master_time_us_;
      for (std::size_t m = 0; m < chain.motors.size(); ++m) {
        const Motor & motor = chain.motors[m];
        wire::TeleMotor & out = tele.motors[m];
        out.pos = motor.pos;
        out.vel = motor.vel;
        out.tau = motor.tau;
        out.temp_c = 30;
        out.drive_state = motor.drive_state;
        out.drive_fault = motor.drive_fault;
        out.rs_mode = motor.rs_mode;
        out.rs_fault_bits = motor.motor_fault;
        out.flags = motor.flags;
        out.fb_age_ms = static_cast<std::uint8_t>(
          std::min<std::uint32_t>(motor.fb_age + motor.unanswered_ms, kWireAgeSaturationMs));
        out.last_applied_seq = motor.last_applied;
      }
    }
    const auto * bytes = reinterpret_cast<const std::uint8_t *>(&tele);
    payload.insert(payload.end(), bytes, bytes + sizeof(tele));
    ++emitted;
  }

  wire::RobotTelemetry header{};
  header.cycle_id = cycle_id_;
  header.master_time_us = master_time_us_;
  header.last_cmd_seq_rx = cmd_seq_rx_;
  header.cmd_seq_active = cmd_seq_active_;
  header.n_chains = emitted;
  // Reporting priority (firmware spi_master.c / protocol_v9.md §robot_state): HOST_LOST >
  // PROTO_MISMATCH > the computed connectivity state (no chain alive = INIT, every chain alive =
  // READY, otherwise DEGRADED). Connectivity, not drive state: an all-DISABLED robot is READY.
  const bool all_alive = alive_count == chains_.size();
  const wire::RobotState rs =
    host_link_ != static_cast<std::uint8_t>(HOST_LINK_OK) ? wire::RobotState::ROBOT_HOST_LOST :
    any_mismatch() ? wire::RobotState::ROBOT_PROTO_MISMATCH :
    alive_count == 0 ? wire::RobotState::ROBOT_INIT :
    all_alive ? wire::RobotState::ROBOT_READY : wire::RobotState::ROBOT_DEGRADED;
  header.robot_state = static_cast<std::uint8_t>(rs);
  last_robot_state_ = rs;
  std::memcpy(payload.data(), &header, wire::kTeleHeadSize);

  std::vector<std::uint8_t> frame(wire::kHeaderSize + payload.size());
  const auto size = wire::build_frame(
    wire::MessageType::MSG_ROBOT_TELE, tx_seq_++, wire::NodeId::NODE_MASTER,
    wire::NodeId::NODE_JETSON, master_time_us_ / 1000U, payload, frame);
  frame.resize(size.value_or(0));
  last_telemetry_frame_ = frame;

  if (corrupt_next_) {
    frame[wire::kHeaderSize + 25] ^= 0x01;
    corrupt_next_ = false;
  }
  write_host(frame);
}

void FakeMaster::emit_status()
{
  if (!master_fd_.valid()) {
    return;
  }
  wire::MasterStatus status{};
  status.schema_hash = config_.schema_hash;
  status.slave_proto_mismatch = 0;
  for (std::size_t c = 0; c < chains_.size() && c < 8; ++c) {
    if (chains_[c].mismatch) {
      status.slave_proto_mismatch =
        static_cast<std::uint8_t>(status.slave_proto_mismatch | (1U << c));
    }
  }
  status.uptime_ms = master_time_us_ / 1000U;
  status.master_poll_hz = config_.poll_hz;
  status.telemetry_hz = config_.telemetry_hz;
  status.slave_tick_hz = 200;
  status.host_cmd_hz = 50;
  std::array<std::uint8_t, sizeof(status)> payload{};
  std::memcpy(payload.data(), &status, sizeof(status));

  std::vector<std::uint8_t> frame(wire::kHeaderSize + payload.size());
  const auto size = wire::build_frame(
    wire::MessageType::MSG_MASTER_STATUS, tx_seq_++, wire::NodeId::NODE_MASTER,
    wire::NodeId::NODE_JETSON, master_time_us_ / 1000U, payload, frame);
  frame.resize(size.value_or(0));

  if (config_.status_version != wire::kProtocolVersion && frame.size() >= wire::kHeaderSize) {
    // Re-stamp the version byte and recompute the CRC, as a master on another version would send.
    frame[12] = config_.status_version;
    frame[14] = 0;
    frame[15] = 0;
    const std::uint16_t crc = wire::crc16(frame);
    frame[14] = static_cast<std::uint8_t>(crc & 0xFFU);
    frame[15] = static_cast<std::uint8_t>(crc >> 8U);
  }
  write_host(frame);
}

void FakeMaster::write_host(const std::vector<std::uint8_t> & bytes)
{
  if (!master_fd_.valid() || bytes.empty()) {
    return;
  }
  if (::write(master_fd_.get(), bytes.data(), bytes.size()) < 0) {
    return;
  }
}

FakeMaster::Motor & FakeMaster::motor_at(std::uint8_t chain, std::uint8_t motor)
{
  for (auto & c : chains_) {
    if (c.id == chain) {
      return c.motors.at(motor);
    }
  }
  throw std::out_of_range("FakeMaster: no such chain");
}

const FakeMaster::Motor & FakeMaster::motor_at(std::uint8_t chain, std::uint8_t motor) const
{
  for (const auto & c : chains_) {
    if (c.id == chain) {
      return c.motors.at(motor);
    }
  }
  throw std::out_of_range("FakeMaster: no such chain");
}

bool FakeMaster::any_armed() const
{
  for (const auto & chain : chains_) {
    for (const auto & motor : chain.motors) {
      if (is_armed_state(motor.drive_state)) {
        return true;
      }
    }
  }
  return false;
}

bool FakeMaster::any_mismatch() const
{
  for (const auto & chain : chains_) {
    if (chain.mismatch) {
      return true;
    }
  }
  return false;
}

void FakeMaster::latch_fault(Motor & motor, std::uint8_t fault)
{
  // First fault wins, as in the firmware: only latch into a clean motor.
  if (motor.drive_fault == static_cast<std::uint8_t>(wire::DriveFault::DRVF_NONE)) {
    motor.drive_fault = fault;
  }
}

}  // namespace humanoid::transport_stm32::testing
