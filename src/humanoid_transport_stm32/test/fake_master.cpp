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

namespace humanoid::transport_stm32::testing
{

namespace
{

// The firmware's host-death trigger and dwell, in master cycles (system_config.h).
constexpr std::uint32_t kHostLostCycles = 12;
constexpr std::uint32_t kHostLostDampCycles = 60;

bool is_armed(wire::Lifecycle state) noexcept;
bool request_carries_fault_reset(const wire::CmdRobot & command) noexcept;

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

  // Opening the slave here, in raw mode, means no byte the fake writes is mangled by line
  // discipline processing before the transport opens it, and the master never sees a hang-up when
  // the transport closes its end.
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
  thread_ = std::thread([this] {run();});
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

wire::Lifecycle FakeMaster::state(std::uint8_t chain, std::uint8_t motor) const
{
  const std::scoped_lock lock{mutex_};
  return motor_at(chain, motor).state;
}

wire::FaultCause FakeMaster::cause(std::uint8_t chain, std::uint8_t motor) const
{
  const std::scoped_lock lock{mutex_};
  return motor_at(chain, motor).cause;
}

bool FakeMaster::host_lost() const
{
  const std::scoped_lock lock{mutex_};
  return host_link_ != HostLink::kOk;
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
  host_link_ = HostLink::kOk;
  damped_cycles_ = 0;
  for (auto & chain : chains_) {
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

void FakeMaster::force_fault(std::uint8_t chain, std::uint8_t motor, wire::FaultCause cause)
{
  const std::scoped_lock lock{mutex_};
  auto & m = motor_at(chain, motor);
  m.state = wire::Lifecycle::kFault;
  m.cause = cause;
}

void FakeMaster::set_measured(
  std::uint8_t chain, std::uint8_t motor, double pos_rad, double vel_rad_s, double tau_nm)
{
  const std::scoped_lock lock{mutex_};
  auto & m = motor_at(chain, motor);
  m.pos = wire::encode_i16(static_cast<float>(pos_rad), wire::kPosScale).raw;
  m.vel = wire::encode_i16(static_cast<float>(vel_rad_s), wire::kVelScale).raw;
  m.tau = wire::encode_i16(static_cast<float>(tau_nm), wire::kTauScale).raw;
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
    // After a long stall, do not fire a burst of back-to-back cycles to catch up.
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
  bool fresh = false;
  bool fault_reset = false;
  if (pending_) {
    active_ = pending_;
    cmd_seq_active_ = pending_->cmd_seq;
    fault_reset = request_carries_fault_reset(*pending_);
    pending_.reset();
    cycles_since_fresh_ = 0;
    fresh = true;
  } else {
    ++cycles_since_fresh_;
  }

  step_host_watchdog(fresh, fault_reset);
  apply_requests();
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
        if (frame->type == wire::MessageType::kRobotCmd &&
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

// The host-death dead-man (host_watchdog.h): armed and silent for too long, the master drives the
// slaves DAMPED and then IDLE, latched until a fresh command carries a fault reset.
void FakeMaster::step_host_watchdog(bool /*fresh*/, bool fault_reset)
{
  if (fault_reset) {
    host_link_ = HostLink::kOk;
    damped_cycles_ = 0;
  }
  switch (host_link_) {
    case HostLink::kOk:
      if (any_armed() && cycles_since_fresh_ >= kHostLostCycles) {
        host_link_ = HostLink::kDamped;
        damped_cycles_ = 0;
      }
      break;
    case HostLink::kDamped:
      if (++damped_cycles_ >= kHostLostDampCycles) {
        host_link_ = HostLink::kIdle;
      }
      break;
    case HostLink::kIdle:
      break;
  }
}

void FakeMaster::apply_requests()
{
  if (!active_) {
    return;
  }
  const std::size_t chain_count = std::min<std::size_t>(active_->n_chains, wire::kMaxChains);
  for (std::size_t c = 0; c < chain_count; ++c) {
    const wire::CmdChain & requested = active_->chains[c];
    for (auto & chain : chains_) {
      if (chain.id != requested.chain_id) {
        continue;
      }
      const std::size_t slots =
        std::min<std::size_t>({requested.n_motors, chain.motors.size(), wire::kMaxMotorsPerChain});
      for (std::size_t m = 0; m < slots; ++m) {
        wire::CmdMotor request = requested.motors[m];
        // While the master has lost the host, it overrides what it forwards to the slaves.
        if (host_link_ == HostLink::kDamped && is_armed(chain.motors[m].state)) {
          request.mode_req = static_cast<std::uint8_t>(wire::ModeRequest::kDamped);
        } else if (host_link_ == HostLink::kIdle) {
          request.mode_req = static_cast<std::uint8_t>(wire::ModeRequest::kIdle);
        }
        apply_request(chain, chain.motors[m], request);
      }
    }
  }
}

// The slave's mode state machine (mode_sm.c), level-triggered: applied every cycle.
void FakeMaster::apply_request(Chain & chain, Motor & motor, const wire::CmdMotor & request)
{
  if ((request.flags & wire::kCmdFlagValid) == 0) {
    return;
  }
  motor.flags = 0;
  const auto mode = static_cast<wire::ModeRequest>(request.mode_req);
  const bool fault_reset = (request.flags & wire::kCmdFlagFaultReset) != 0;

  if (motor.state == wire::Lifecycle::kFault) {
    if (mode != wire::ModeRequest::kIdle && !fault_reset) {
      motor.flags |= 0x01;  // TELE_FLAG_REQUEST_REJECTED: faults latch
      return;
    }
    motor.state = wire::Lifecycle::kIdle;
    motor.cause = wire::FaultCause::kNone;
    motor.motor_fault = 0;
  }

  if (!is_armed(motor.state)) {
    if (mode == wire::ModeRequest::kIdle) {
      return;
    }
    if (mode != wire::ModeRequest::kHold) {
      motor.flags |= 0x01;  // from IDLE only HOLD arms
      return;
    }
    if (motor.wound) {
      motor.state = wire::Lifecycle::kFault;
      motor.cause = wire::FaultCause::kWound;
      return;
    }
    motor.state = wire::Lifecycle::kHold;
    motor.last_applied = 0;
    chain.silent_cycles = config_.arming_silence_cycles;
    return;
  }

  switch (mode) {
    case wire::ModeRequest::kIdle:
      motor.state = wire::Lifecycle::kIdle;
      motor.last_applied = 0;
      break;
    case wire::ModeRequest::kHold:
      motor.state = wire::Lifecycle::kHold;
      break;
    case wire::ModeRequest::kMit:
      motor.state = wire::Lifecycle::kMit;
      // The motor follows its command, which makes sign and unit conversion observable.
      motor.pos = request.pos;
      motor.vel = request.vel;
      motor.tau = request.tau_ff;
      motor.last_applied = cmd_seq_active_;
      break;
    case wire::ModeRequest::kDamped:
      motor.state = wire::Lifecycle::kDamped;
      break;
    case wire::ModeRequest::kToZero:
      motor.state = wire::Lifecycle::kToZero;
      break;
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

  std::vector<std::uint8_t> payload(sizeof(wire::TeleRobotHeader));
  std::uint8_t emitted = 0;
  for (auto & chain : chains_) {
    if (chain.silent_cycles > 0) {
      --chain.silent_cycles;  // the slave is blocked: the master gets nothing valid from it
      continue;
    }
    if (chain.dropped) {
      continue;
    }
    wire::TeleChain tele{};
    tele.chain_id = chain.id;
    tele.n_motors = static_cast<std::uint8_t>(chain.motors.size());
    tele.slave_time_us = master_time_us_;
    for (std::size_t m = 0; m < chain.motors.size(); ++m) {
      const Motor & motor = chain.motors[m];
      wire::TeleMotor & out = tele.motors[m];
      out.pos = motor.pos;
      out.vel = motor.vel;
      out.tau = motor.tau;
      out.temp_c = 30;
      out.state = static_cast<std::uint8_t>(motor.state);
      out.cause = static_cast<std::uint8_t>(motor.cause);
      out.motor_mode = is_armed(motor.state) ? 2 : 0;
      out.motor_fault = motor.motor_fault;
      out.flags = motor.flags;
      out.fb_age_ms = motor.fb_age;
      out.last_applied_seq = motor.last_applied;
    }
    const auto * bytes = reinterpret_cast<const std::uint8_t *>(&tele);
    payload.insert(payload.end(), bytes, bytes + sizeof(tele));
    ++emitted;
  }

  wire::TeleRobotHeader header{};
  header.cycle_id = cycle_id_;
  header.master_time_us = master_time_us_;
  header.last_cmd_seq_rx = cmd_seq_rx_;
  header.cmd_seq_active = cmd_seq_active_;
  header.n_chains = emitted;
  header.robot_state = static_cast<std::uint8_t>(
    host_link_ != HostLink::kOk ? wire::RobotState::kHostLost : wire::RobotState::kReady);
  std::memcpy(payload.data(), &header, sizeof(header));

  std::vector<std::uint8_t> frame(wire::kHeaderSize + payload.size());
  const auto size = wire::build_frame(
    wire::MessageType::kRobotTele, tx_seq_++, wire::NodeId::kMaster, wire::NodeId::kJetson,
    master_time_us_ / 1000U, payload, frame);
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
  status.robot_state = static_cast<std::uint8_t>(wire::RobotState::kReady);
  status.uptime_ms = master_time_us_ / 1000U;
  status.master_poll_hz = config_.poll_hz;
  status.telemetry_hz = config_.telemetry_hz;
  status.slave_tick_hz = 200;
  status.host_cmd_hz = 50;
  std::array<std::uint8_t, sizeof(status)> payload{};
  std::memcpy(payload.data(), &status, sizeof(status));

  std::vector<std::uint8_t> frame(wire::kHeaderSize + payload.size());
  const auto size = wire::build_frame(
    wire::MessageType::kMasterStatus, tx_seq_++, wire::NodeId::kMaster, wire::NodeId::kJetson,
    master_time_us_ / 1000U, payload, frame);
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
  // Non-blocking: a full buffer drops the frame, as the master's USB TX ring would.
  static_cast<void>(::write(master_fd_.get(), bytes.data(), bytes.size()));
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
      if (is_armed(motor.state)) {
        return true;
      }
    }
  }
  return false;
}

namespace
{

bool is_armed(wire::Lifecycle state) noexcept
{
  return state == wire::Lifecycle::kHold || state == wire::Lifecycle::kMit ||
         state == wire::Lifecycle::kDamped || state == wire::Lifecycle::kToZero;
}

bool request_carries_fault_reset(const wire::CmdRobot & command) noexcept
{
  const std::size_t chain_count = std::min<std::size_t>(command.n_chains, wire::kMaxChains);
  for (std::size_t c = 0; c < chain_count; ++c) {
    const std::size_t motors =
      std::min<std::size_t>(command.chains[c].n_motors, wire::kMaxMotorsPerChain);
    for (std::size_t m = 0; m < motors; ++m) {
      if ((command.chains[c].motors[m].flags & wire::kCmdFlagFaultReset) != 0) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

}  // namespace humanoid::transport_stm32::testing
