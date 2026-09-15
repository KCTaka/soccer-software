// What a transport can do, reported without naming a bus. No USB, SPI, CAN, EtherCAT, packet
// layout, or topology concept may appear above the transport boundary (ADR-001).
#ifndef HUMANOID_TRANSPORT__TRANSPORT_CAPABILITIES_HPP_
#define HUMANOID_TRANSPORT__TRANSPORT_CAPABILITIES_HPP_

#include <array>
#include <cstdint>

#include "humanoid_transport/joint_manifest.hpp"

namespace humanoid::transport
{

enum class TransportClass : std::uint8_t
{
  kPhysical = 0,
  kSimulated,
  kReplay,
  kFaultInjection
};

enum class TupleCompleteness : std::uint8_t
{
    kFull = 0, 
    kPositionVelocityOnly
};

struct TransportCapabilities
{
  std::array<char, kMaxNameLength> implementation_name{};
  TransportClass transport_class{TransportClass::kPhysical};
  std::uint8_t joint_count{0U};
  std::uint32_t nominal_cycle_period_us{0U};
  std::uint32_t worst_case_exchange_us{0U};
  bool supports_per_joint_disable{false};
  bool supports_availability_mask{false};
  bool provides_temperature{false};
  bool provides_bus_voltage{false};
  /// True only for kReplay. ADR-007 forbids claiming a timing property from a non-physical class.
  bool is_deterministic{false};

  /// Which subset of the MIT tuple this transport delivers.
  TupleCompleteness tuple_completeness{TupleCompleteness::kFull};
};

}  // namespace humanoid::transport

#endif  // HUMANOID_TRANSPORT__TRANSPORT_CAPABILITIES_HPP_
