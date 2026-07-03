// soccerbot ↔ Master wire protocol — HOST (C++) side.
//
// DRAFT of the recommended Jetson↔Master contract documented in
// docs/architecture/jetson_master_protocol.md. It mirrors the firmware's
// soccer-firmware/firmware/common/include/protocol.h framing (16-byte header,
// CRC16-CCITT, COBS) but adds the high-rate, all-joints MIT command/telemetry
// frames that the firmware contract still leaves as a stub.
//
// IMPORTANT: this header is a hand-written placeholder. Per recommendation #4
// (single source of truth), these structs MUST eventually be GENERATED — shared
// byte-for-byte with the firmware and the Python host — not maintained by hand.
#ifndef SOCCER_HARDWARE__WIRE_PROTOCOL_HPP_
#define SOCCER_HARDWARE__WIRE_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace soccer_hardware::wire
{

// ── protocol version (refuse to arm on mismatch — recommendation #5) ──
constexpr uint16_t PROTO_VERSION = 1;

// ── node ids ──
enum class Node : uint8_t { JETSON = 1, MASTER = 2, SLAVE_0 = 3 };

// ── message types ──
enum class Msg : uint16_t {
  PING = 0x01,
  MASTER_STATUS = 0x02,
  MOTOR_STATE = 0x04,   // Master → Jetson, all joints + body sensors (data plane)
  CONTROL_REQ = 0x05,   // Jetson → Master, lifecycle (control plane)
  CONTROL_RESP = 0x06,
  MOTOR_CMD = 0x07,     // Jetson → Master, all joints MIT (data plane)
};

// ── 16-byte framed header (matches firmware protocol.h) ──
#pragma pack(push, 1)
struct Header {
  uint16_t type;
  uint16_t seq;
  uint8_t src;
  uint8_t dst;
  uint32_t ts_ms;
  uint16_t len;    // payload bytes
  uint16_t flags;  // bit 0..: reserved; high byte carries proto_version on PING
  uint16_t crc16;  // CRC16-CCITT over header(crc=0)+payload
};
static_assert(sizeof(Header) == 16, "wire header must be 16 bytes");

// ── data-plane command: one per joint, all joints per frame ──
struct JointMitCmd {
  float q_des;   // rad
  float qd_des;  // rad/s
  float kp;      // impedance stiffness
  float kd;      // impedance damping
  float tau_ff;  // N·m feed-forward
};
static_assert(sizeof(JointMitCmd) == 20, "JointMitCmd must be 20 bytes");

// ── data-plane telemetry: one per joint ──
struct JointState {
  float q;                // rad
  float qd;               // rad/s
  float tau;              // N·m (current-sense)
  int8_t temp_c;          // °C
  uint8_t state;          // MotorLifecycle
  uint16_t fault_flags;   // Robostride fault bits
  uint16_t last_cmd_seq;  // echoes the command seq → RTT / drop detection
};
static_assert(sizeof(JointState) == 18, "JointState must be 18 bytes");

// ── body sensors, appended once per telemetry frame (IMU on the Master MCU) ──
struct BodyState {
  float quat[4];          // w, x, y, z
  float gyro[3];          // rad/s
  float accel[3];         // m/s²
  uint16_t contact_bits;  // per-foot contact, bit i = foot i
};
#pragma pack(pop)

// ── CRC16-CCITT (poly 0x1021, init 0xFFFF) — identical to the firmware ──
inline uint16_t crc16_ccitt(const uint8_t * data, size_t len, uint16_t crc = 0xFFFF)
{
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

// ── COBS (Consistent Overhead Byte Stuffing); frames end with a 0x00 delimiter ──
inline std::vector<uint8_t> cobs_encode(const uint8_t * data, size_t len)
{
  std::vector<uint8_t> out;
  out.reserve(len + len / 254 + 2);
  size_t code_idx = out.size();
  out.push_back(0);  // placeholder for the first code byte
  uint8_t code = 1;
  for (size_t i = 0; i < len; ++i) {
    if (data[i] == 0) {
      out[code_idx] = code;
      code_idx = out.size();
      out.push_back(0);
      code = 1;
    } else {
      out.push_back(data[i]);
      if (++code == 0xFF) {
        out[code_idx] = code;
        code_idx = out.size();
        out.push_back(0);
        code = 1;
      }
    }
  }
  out[code_idx] = code;
  return out;
}

// Decode one COBS block (without the trailing delimiter). Returns false on a
// malformed block.
inline bool cobs_decode(const uint8_t * data, size_t len, std::vector<uint8_t> & out)
{
  out.clear();
  size_t i = 0;
  while (i < len) {
    uint8_t code = data[i++];
    if (code == 0) {
      return false;  // unexpected delimiter inside the block
    }
    for (uint8_t j = 1; j < code; ++j) {
      if (i >= len) {
        return false;
      }
      out.push_back(data[i++]);
    }
    if (code != 0xFF && i < len) {
      out.push_back(0);
    }
  }
  return true;
}

}  // namespace soccer_hardware::wire

#endif  // SOCCER_HARDWARE__WIRE_PROTOCOL_HPP_
