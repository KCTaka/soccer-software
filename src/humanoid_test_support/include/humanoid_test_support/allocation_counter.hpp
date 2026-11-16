// Copyright 2026 UTRA-RoboSoccer
// Counts the allocations the calling thread makes, for the real-time claim that exchange() never
// allocates (ADR-001). Only the calling thread is counted, so a transport's reader thread or a
// fake master's own allocations do not pollute the measurement.
//
// This header REPLACES the global operator new and delete, so include it in exactly one
// translation unit of a test binary (transport_conformance.hpp does). It sees C++ allocations
// only: memory a C library takes with malloc, MuJoCo's for one, passes unseen.
#ifndef HUMANOID_TEST_SUPPORT__ALLOCATION_COUNTER_HPP_
#define HUMANOID_TEST_SUPPORT__ALLOCATION_COUNTER_HPP_

#include <cstddef>
#include <cstdlib>
#include <new>

namespace humanoid::test_support
{

inline thread_local std::size_t g_thread_allocations = 0;

/// operator new calls the calling thread has made so far.
inline std::size_t thread_allocations() noexcept {return g_thread_allocations;}

}  // namespace humanoid::test_support

// noinline: inlined into a caller, GCC pairs the malloc/free inside these with the caller's
// new/delete and reports -Wmismatched-new-delete on what is a matched pair.
[[gnu::noinline]] void * operator new(std::size_t size)
{
  ++humanoid::test_support::g_thread_allocations;
  if (void * p = std::malloc(size)) {
    return p;
  }
  throw std::bad_alloc{};
}
[[gnu::noinline]] void operator delete(void * p) noexcept {std::free(p);}
[[gnu::noinline]] void operator delete(void * p, std::size_t) noexcept {std::free(p);}

#endif  // HUMANOID_TEST_SUPPORT__ALLOCATION_COUNTER_HPP_
