// crc16.h - CRC-16/MODBUS, shared.
//
// Poly 0xA001 (reflected 0x8005), init 0xFFFF, no final XOR. Check value for the
// ASCII string "123456789" is 0x4B37.
//
// Used by two unrelated layers - the JBD CAN payload checksum and the telemetry
// packet trailer - so it lives here rather than being written twice. The update
// form exists so a CRC can span two buffers without copying either.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace util {

constexpr uint16_t kCrcInit = 0xFFFF;

uint16_t crc16ModbusUpdate(uint16_t crc, const uint8_t *data, size_t length);

inline uint16_t crc16Modbus(const uint8_t *data, size_t length) {
  return crc16ModbusUpdate(kCrcInit, data, length);
}

}  // namespace util
