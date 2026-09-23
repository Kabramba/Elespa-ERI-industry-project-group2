#include "jbd_uart_link.h"

namespace jbd {

bool JbdUartLink::begin() {
  if (cfg_.port == nullptr) return false;
  cfg_.port->begin(cfg_.baud, SERIAL_8N1, cfg_.rxPin, cfg_.txPin);
  return true;
}

bool JbdUartLink::readByte(uint8_t &b, uint32_t deadline) {
  while (static_cast<int32_t>(deadline - millis()) > 0) {
    const int c = cfg_.port->read();
    if (c >= 0) {
      b = static_cast<uint8_t>(c);
      return true;
    }
    delay(1);
  }
  return false;
}

// Hunts for the 0xDD start byte, then pulls the header and exactly as many
// payload bytes as the length field promises. Returns bytes buffered, 0 on
// timeout.
size_t JbdUartLink::receive(uint32_t deadline) {
  size_t  n = 0;
  uint8_t b = 0;

  for (;;) {
    if (!readByte(b, deadline)) return 0;
    if (b == kStart) break;
  }
  rxBuf_[n++] = b;

  while (n < 4) {
    if (!readByte(b, deadline)) return 0;
    rxBuf_[n++] = b;
  }

  const size_t total = 4u + rxBuf_[3] + 3u;
  if (total > sizeof(rxBuf_)) return 0;
  while (n < total) {
    if (!readByte(b, deadline)) return 0;
    rxBuf_[n++] = b;
  }
  return n;
}

bool JbdUartLink::readRegister(uint8_t reg, Frame &f) {
  for (uint8_t attempt = 0; attempt < cfg_.attempts; ++attempt) {
    ++stats_.attempts;

    uint8_t req[kRequestBytes];
    buildReadRequest(reg, req);
    while (cfg_.port->available()) cfg_.port->read();  // drop stale bytes
    cfg_.port->write(req, kRequestBytes);

    const size_t n = receive(millis() + cfg_.timeoutMs);
    if (n == 0) {
      ++stats_.failures;
      continue;
    }

    const ParseResult r = parseResponse(rxBuf_, n, reg, f);
    if (r == ParseResult::kOk) {
      ++stats_.successes;
      return true;
    }
    if (r == ParseResult::kBadChecksum) ++stats_.crcErrors;
    ++stats_.failures;
    delay(50);
  }
  return false;
}

bool JbdUartLink::poll(BmsSnapshot &out) {
  Frame       f;
  BmsSnapshot s = {};

  if (!readRegister(kRegBasic, f)) return false;
  if (!decodeBasic(f, s)) return false;

  // Cell voltages are a separate register; a miss here still leaves a usable
  // snapshot, so it is not fatal.
  if (readRegister(kRegCells, f)) decodeCells(f, s);

  s.uptimeMs = millis();
  computeDerived(s);
  out = s;
  return true;
}

}  // namespace jbd
