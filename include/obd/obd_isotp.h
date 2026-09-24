// obd_isotp.h - ISO 15765-2 framing and the transmit guard.
//
// Pure: no Arduino, no TWAI. Every rule the Activa reference calls
// non-negotiable is enforced here as code, and tested on the host.
//
// ISO-TP puts a Protocol Control Information nibble in byte 0:
//   0x0  Single Frame      0L data...        (L = 1..7)
//   0x1  First Frame       1L LL data(6)     (12-bit total length)
//   0x2  Consecutive Frame 2N data(7)        (N wraps 0..F)
//   0x3  Flow Control      30 BS STmin
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace obd {

enum class Pci : uint8_t { kSingle, kFirst, kConsecutive, kFlowControl, kInvalid };

// Diagnostic services this firmware may request. Read-only services only:
// 0x01 live data, 0x02 freeze frame, 0x03/0x07/0x0A DTCs, 0x09 vehicle info.
//
// Everything else is refused, explicitly including 0x04 and 0x14 (clear DTCs),
// 0x10 (session control), 0x11 (ECU reset), 0x27 (security access), 0x2E, 0x2F,
// 0x31, 0x34-0x37 and 0x3D. Those either change ECU state or move data into it.
constexpr uint8_t kAllowedSids[] = {0x01, 0x02, 0x03, 0x07, 0x09, 0x0A};
constexpr size_t  kAllowedSidCount = sizeof(kAllowedSids) / sizeof(kAllowedSids[0]);

bool isAllowedSid(uint8_t sid);

Pci      pciOf(const uint8_t *d, uint8_t dlc);
uint8_t  singleFrameLength(const uint8_t *d);   // payload bytes in a Single Frame
uint16_t firstFrameLength(const uint8_t *d);    // total bytes the ECU will send
uint8_t  consecutiveIndex(const uint8_t *d);    // sequence nibble

// THE GATE. A frame may leave this node only if it is a Single Frame carrying
// an allowed SID, or a Flow Control frame. Multi-frame requests, anything with
// a disallowed SID, and malformed PCI are all refused.
bool isPermittedRequestFrame(const uint8_t *d, uint8_t dlc);

// Builders. Each writes exactly 8 bytes, zero padded, and returns 8.
size_t buildSingleFrame(uint8_t sid, const uint8_t *args, uint8_t argCount, uint8_t *out8);
size_t buildFlowControl(uint8_t *out8, uint8_t blockSize = 0, uint8_t stMin = 0);

}  // namespace obd
