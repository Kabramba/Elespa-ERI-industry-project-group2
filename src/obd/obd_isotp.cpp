#include "obd/obd_isotp.h"

#include <string.h>

namespace obd {

bool isAllowedSid(uint8_t sid) {
  for (size_t i = 0; i < kAllowedSidCount; ++i)
    if (kAllowedSids[i] == sid) return true;
  return false;
}

Pci pciOf(const uint8_t *d, uint8_t dlc) {
  if (d == nullptr || dlc == 0) return Pci::kInvalid;
  switch (d[0] >> 4) {
    case 0x0: return Pci::kSingle;
    case 0x1: return Pci::kFirst;
    case 0x2: return Pci::kConsecutive;
    case 0x3: return Pci::kFlowControl;
    default:  return Pci::kInvalid;
  }
}

uint8_t  singleFrameLength(const uint8_t *d) { return static_cast<uint8_t>(d[0] & 0x0F); }
uint16_t firstFrameLength(const uint8_t *d) {
  return static_cast<uint16_t>((static_cast<uint16_t>(d[0] & 0x0F) << 8) | d[1]);
}
uint8_t consecutiveIndex(const uint8_t *d) { return static_cast<uint8_t>(d[0] & 0x0F); }

bool isPermittedRequestFrame(const uint8_t *d, uint8_t dlc) {
  if (d == nullptr || dlc != 8) return false;  // all OBD frames are DLC 8

  const Pci pci = pciOf(d, dlc);

  // Flow Control is permitted: it carries no service, only pacing.
  if (pci == Pci::kFlowControl) return true;

  // Only Single Frames otherwise. A multi-frame request would mean writing a
  // payload into the ECU, which nothing here has any business doing.
  if (pci != Pci::kSingle) return false;

  const uint8_t len = singleFrameLength(d);
  if (len < 1 || len > 7) return false;

  return isAllowedSid(d[1]);
}

size_t buildSingleFrame(uint8_t sid, const uint8_t *args, uint8_t argCount, uint8_t *out8) {
  memset(out8, 0x00, 8);  // pad with zeros, as in the reference's examples
  if (argCount > 6) argCount = 6;
  out8[0] = static_cast<uint8_t>(1 + argCount);  // PCI: single frame, length
  out8[1] = sid;
  for (uint8_t i = 0; i < argCount; ++i) out8[2 + i] = args[i];
  return 8;
}

size_t buildFlowControl(uint8_t *out8, uint8_t blockSize, uint8_t stMin) {
  memset(out8, 0x00, 8);
  out8[0] = 0x30;       // Continue To Send
  out8[1] = blockSize;  // 0 = send everything without further flow control
  out8[2] = stMin;      // 0 = as fast as you like
  return 8;
}

}  // namespace obd
