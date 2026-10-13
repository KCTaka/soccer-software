// Copyright 2026 UTRA-RoboSoccer
// Codec oracle for wire_protocol_test: a few hand-verified constants the firmware's wire layout
// does NOT determine -- the CRC-16 check value and the fixed-point rounding/saturation of a handful
// of (value, scale) pairs. These are pure arithmetic, identical across protocol versions, so they
// are written out by hand rather than generated from a firmware build.
//
// The firmware-authored BYTE frames that used to live here (robot_cmd / robot_tele / master_status)
// are gone: the cross-repo wire oracle is now generated/fixtures/*.bin, emitted by the firmware's
// own gen_protocol.py at the pinned tag and round-tripped in wire_pin_test. The framing and scanner
// tests build their own frames at runtime with the (fixture-validated) codec, which exercises the
// framing behaviour without testing the codec against itself.
#ifndef GOLDEN__GOLDEN_FRAMES_HPP_
#define GOLDEN__GOLDEN_FRAMES_HPP_

#include <array>
#include <cstdint>

namespace humanoid::transport_stm32::golden
{

/// CRC-16-CCITT-FALSE over the standard "123456789" check vector.
inline constexpr std::uint16_t kCrcOfCheckString = 0x29B1;

/// (value, scale) -> the raw int16 the firmware's encoder produces: round half away from zero,
/// saturating at the int16 limits. Scale is carried explicitly so the case is independent of which
/// field (pos/vel/tau) it stands for.
struct FixedPointCase
{
  float x;
  float scale;
  std::int16_t raw;
};
inline constexpr std::array<FixedPointCase, 9> kFixedPointCases{{
  {0.500000f, 10000.0f, 5000},
  {-1.250000f, 100.0f, -125},
  {0.000050f, 10000.0f, 1},
  {-0.000050f, 10000.0f, -1},
  {3.141600f, 10000.0f, 31416},
  {4.000000f, 10000.0f, 32767},
  {-4.000000f, 10000.0f, -32768},
  {327.674988f, 100.0f, 32767},
  {0.000000f, 100.0f, 0},
}};

/// Gains use the unsigned scales: Kp 15 N*m/rad and Kd 1 N*m*s/rad at the firmware's scales.
inline constexpr std::uint16_t kKpOf15 = 150;
inline constexpr std::uint16_t kKdOf1 = 100;

}  // namespace humanoid::transport_stm32::golden

#endif  // GOLDEN__GOLDEN_FRAMES_HPP_
