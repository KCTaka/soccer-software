// The stable hardware-abstraction seam of ADR-001, and the simulation seam of ADR-007.
//
// Replacing the physical actuator path -- dual-STM32 USB serial today, native CAN FD or EtherCAT
// later -- must be a configuration change, not a rewrite. Everything above this interface is
// digest-identical between production, SIL, HIL, fault injection, and replay.
#ifndef HUMANOID_TRANSPORT__ACTUATOR_TRANSPORT_HPP_
#define HUMANOID_TRANSPORT__ACTUATOR_TRANSPORT_HPP_

#include "humanoid_transport/batch_types.hpp"
#include "humanoid_transport/joint_manifest.hpp"
#include "humanoid_transport/safety_manifest.hpp"
#include "humanoid_transport/transport_capabilities.hpp"

namespace humanoid::transport
{

class ActuatorTransport
{
public:
  virtual ~ActuatorTransport() = default;

  ActuatorTransport(const ActuatorTransport &) = delete;
  ActuatorTransport & operator=(const ActuatorTransport &) = delete;
  ActuatorTransport(ActuatorTransport &&) = delete;
  ActuatorTransport & operator=(ActuatorTransport &&) = delete;

  // --- Non-real-time. Allocation, I/O, and blocking are permitted here and only here. ---

  [[nodiscard]] virtual bool configure(
    const JointManifest & joints, const SafetyManifest & safety) = 0;

  [[nodiscard]] virtual bool activate() = 0;

  virtual void deactivate() = 0;

  [[nodiscard]] virtual TransportCapabilities capabilities() const = 0;

  // --- Real-time. No allocation, no logging, no filesystem, no unbounded wait (ADR-001). ---

  /// Exchange one cycle. Must return by `deadline` whether or not the hardware answered;
  /// a transport that waits indefinitely converts a bus fault into a missed control cycle.
  [[nodiscard]] virtual ExchangeResult exchange(
    const CommandBatch & command, FeedbackBatch & feedback, MonotonicStamp deadline) noexcept = 0;

  /// Best-effort. ADR-002: an unverified isolation must escalate to shared power removal.
  [[nodiscard]] virtual bool request_joint_disable(JointIndex joint) noexcept = 0;

  [[nodiscard]] virtual bool request_all_disable() noexcept = 0;

  [[nodiscard]] virtual HealthSnapshot health_snapshot() const noexcept = 0;

protected:
  ActuatorTransport() = default;
};

}  // namespace humanoid::transport

#endif  // HUMANOID_TRANSPORT__ACTUATOR_TRANSPORT_HPP_
