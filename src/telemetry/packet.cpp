#include "telemetry/packet.h"

#include <string.h>

#include "crc16.h"

namespace telemetry {

size_t encode(PacketType type, SourceId source, uint16_t sequence, uint32_t uptimeMs,
              uint8_t flags, const void *payload, uint16_t payloadBytes, uint8_t *out,
              size_t outMax) {
  if (out == nullptr) return 0;
  const size_t total = kHeaderBytes + payloadBytes;
  if (total > outMax) return 0;
  if (payloadBytes > 0 && payload == nullptr) return 0;

  PacketHeader h{};
  h.magic         = kMagic;
  h.schemaVersion = kSchemaVersion;
  h.type          = static_cast<uint8_t>(type);
  h.payloadBytes  = payloadBytes;
  h.sequence      = sequence;
  h.uptimeMs      = uptimeMs;
  h.source        = static_cast<uint8_t>(source);
  h.flags         = flags;
  h.crc           = 0;  // zeroed while the CRC is computed over the whole buffer

  memcpy(out, &h, kHeaderBytes);
  if (payloadBytes) memcpy(out + kHeaderBytes, payload, payloadBytes);

  const uint16_t crc = util::crc16Modbus(out, total);
  memcpy(out + offsetof(PacketHeader, crc), &crc, sizeof(crc));
  return total;
}

bool peekHeader(const uint8_t *buf, size_t length, PacketHeader &out) {
  if (buf == nullptr || length < kHeaderBytes) return false;
  memcpy(&out, buf, kHeaderBytes);
  return out.magic == kMagic;
}

bool verify(const uint8_t *buf, size_t length) {
  PacketHeader h;
  if (!peekHeader(buf, length, h)) return false;
  if (h.schemaVersion != kSchemaVersion) return false;
  if (kHeaderBytes + static_cast<size_t>(h.payloadBytes) > length) return false;

  // Recompute exactly as encode() did: the header with its CRC field zeroed,
  // then the payload. The update form spans both without copying the payload.
  uint8_t header[kHeaderBytes];
  memcpy(header, buf, kHeaderBytes);
  memset(header + offsetof(PacketHeader, crc), 0, sizeof(h.crc));

  uint16_t crc = util::crc16ModbusUpdate(util::kCrcInit, header, kHeaderBytes);
  crc = util::crc16ModbusUpdate(crc, buf + kHeaderBytes, h.payloadBytes);
  return crc == h.crc;
}

const void *payloadOf(const uint8_t *buf, size_t length) {
  if (!verify(buf, length)) return nullptr;
  PacketHeader h;
  peekHeader(buf, length, h);
  return (h.payloadBytes == 0) ? nullptr : (buf + kHeaderBytes);
}

const char *toString(PacketType t) {
  switch (t) {
    case PacketType::kIdentity:    return "identity";
    case PacketType::kEngine:      return "engine";
    case PacketType::kMotor:       return "motor";
    case PacketType::kDynamics:    return "dynamics";
    case PacketType::kNavigation:  return "navigation";
    case PacketType::kBattery:     return "battery";
    case PacketType::kStatusEvent: return "status-event";
  }
  return "?";
}

const char *toString(SourceId s) {
  switch (s) {
    case SourceId::kUnknown:      return "unknown";
    case SourceId::kEcu:          return "ecu";
    case SourceId::kBatteryA:     return "battery-a";
    case SourceId::kBatteryB:     return "battery-b";
    case SourceId::kMotorDriver:  return "motor-driver";
    case SourceId::kSensorGroup1: return "sensor-group1";
    case SourceId::kFusion:       return "fusion";
  }
  return "?";
}

}  // namespace telemetry
