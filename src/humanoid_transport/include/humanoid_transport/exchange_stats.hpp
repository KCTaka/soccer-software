// Copyright 2026 UTRA-RoboSoccer
// Per-exchange accounting kept by the single caller of ActuatorTransport::exchange().
//
// Why the caller and not the transport: everything here is derivable from the
// call itself (sequence sent, ExchangeResult, elapsed time, deadline), so it is
// transport-agnostic and must be implemented once, not re-implemented -- and
// allowed to drift -- in every transport. Transports report only what they
// alone can observe, in HealthSnapshot.
//
// This is the "cycle duration, deadline overruns, and transport counters" data
// that ADR-008-11 makes first-class fields of the in-cycle telemetry record.
#ifndef HUMANOID_TRANSPORT__EXCHANGE_STATS_HPP_
#define HUMANOID_TRANSPORT__EXCHANGE_STATS_HPP_

#include <chrono>
#include <cstdint>
#include <type_traits>

#include "humanoid_transport/batch_types.hpp"

namespace humanoid::transport
{

/// Plain value, trivially copyable: the owner's real-time thread updates it in
/// place, and a record or snapshot is a straight copy. It is not synchronised;
/// a reader on another thread needs a hand-off (e.g. LatestValueBuffer), not
/// direct access.
struct ExchangeStats
{
  /// Sequence of the most recent exchange attempted. Correlation key (ADR-008-02).
  CycleSequence last_sequence{0U};
  std::uint64_t attempted{0U};
  std::uint64_t failed{0U};
  std::uint64_t deadline_misses{0U};
  /// Longest exchange() call measured by the caller, including a failed one.
  std::chrono::nanoseconds worst_exchange{0};

  /// Real-time. Accounts for one exchange() call.
  constexpr void record(
    CycleSequence sequence, const ExchangeResult & result,
    std::chrono::nanoseconds elapsed, bool deadline_missed) noexcept
  {
    last_sequence = sequence;
    ++attempted;
    if (!result.ok()) {
      ++failed;
    }
    if (deadline_missed) {
      ++deadline_misses;
    }
    if (elapsed > worst_exchange) {
      worst_exchange = elapsed;
    }
  }
};
static_assert(std::is_trivially_copyable_v<ExchangeStats>);

}  // namespace humanoid::transport

#endif  // HUMANOID_TRANSPORT__EXCHANGE_STATS_HPP_
