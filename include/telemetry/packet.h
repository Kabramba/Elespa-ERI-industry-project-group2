// packet.h - the common envelope every telemetry packet travels in.
//
// The transmission medium is deliberately out of scope: these are byte layouts,
// not a protocol stack. Whatever carries them later - UART, Wi-Fi, LoRa, a shared
// memory region on an SBC - sees the same bytes.
//
// BYTE ORDER is little-endian, native on both the ESP32-S3 (Xtensa LE) and any
// x86/ARM host that consumes these, so encoding is a memcpy rather than a
// field-by-field swap. A big-endian consumer must byte-swap.
//
// EVERY PAYLOAD CARRIES A FIELD MASK. A telemetry link that drops a reading must
// not present it as a zero - that is how "the discharge MOSFET is off" gets
// confused with "that frame never arrived". Consumers check the bit before
// trusting the field.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace telemetry {

constexpr uint16_t kMagic         = 0xE15A;  // "ELSA"
constexpr uint8_t  kSchemaVersion = 1;

// Wire identifiers. Values are frozen once published - append, never renumber.
enum class PacketType : uint8_t {
  kIdentity = 0x01,  // static, on change or once per minute
  kEngine   = 0x10,  // ECU, engine-related, fast channels summarised
  kVehicle  = 0x11,  // ECU, not engine-related
  kBattery  = 0x20,  // BMS, vendor-independent
  kMotor    = 0x30,  // RESERVED - motor controller, future scope
  kIotGroup = 0x40,  // RESERVED - partner group's sensor set
};

// Which subsystem produced the packet, so a consumer can tell two batteries apart.
enum class SourceId : uint8_t {
  kUnknown  = 0,
  kEcu      = 1,
  kBatteryA = 2,
  kBatteryB = 3,
  kMotor    = 4,
  kIotGroup = 5,
};

enum HeaderFlag : uint8_t {
  kFlagPartial   = 1 << 0,  // some fields missing this cycle; check the field mask
  kFlagDegraded  = 1 << 1,  // link quality below threshold, values may be stale
  kFlagSimulated = 1 << 2,  // not from real hardware - never set in production
};

// 16 bytes, no padding.
struct PacketHeader {
  uint16_t magic;
  uint8_t  schemaVersion;
  uint8_t  type;          // PacketType
  uint16_t payloadBytes;  // excludes this header
  uint16_t sequence;      // per source, wraps
  uint32_t uptimeMs;      // producer clock; no wall clock on the device
  uint8_t  source;        // SourceId
  uint8_t  flags;         // HeaderFlag bits
  uint16_t crc;           // CRC-16/MODBUS over header (crc zeroed) then payload
};
static_assert(sizeof(PacketHeader) == 16, "header must stay 16 bytes");

constexpr size_t kHeaderBytes = sizeof(PacketHeader);

// Serialise header + payload into `out`. Returns bytes written, 0 if it will not
// fit. The CRC is computed with its own field zeroed, so a verifier can recompute
// over the buffer in place.
size_t encode(PacketType type, SourceId source, uint16_t sequence, uint32_t uptimeMs,
              uint8_t flags, const void *payload, uint16_t payloadBytes, uint8_t *out,
              size_t outMax);

// Typed convenience wrapper.
template <typename T>
size_t encode(PacketType type, SourceId source, uint16_t sequence, uint32_t uptimeMs,
              uint8_t flags, const T &payload, uint8_t *out, size_t outMax) {
  return encode(type, source, sequence, uptimeMs, flags, &payload,
                static_cast<uint16_t>(sizeof(T)), out, outMax);
}

// Reads the header without validating the payload.
bool peekHeader(const uint8_t *buf, size_t length, PacketHeader &out);

// Full check: magic, schema version, declared length against the buffer, and CRC.
bool verify(const uint8_t *buf, size_t length);

// Pointer to the payload inside a verified buffer, or nullptr.
const void *payloadOf(const uint8_t *buf, size_t length);

const char *toString(PacketType t);
const char *toString(SourceId s);

}  // namespace telemetry
