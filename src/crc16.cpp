#include "crc16.h"

namespace util {

uint16_t crc16ModbusUpdate(uint16_t crc, const uint8_t *data, size_t length) {
  if (data == nullptr) return crc;
  while (length--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i)
      crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001)
                      : static_cast<uint16_t>(crc >> 1);
  }
  return crc;
}

}  // namespace util
