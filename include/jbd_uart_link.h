// jbd_uart_link.h - JBD protocol V4 over UART1. The dependable link: 100 %
// reply rate in every session so far.
#pragma once

#include <Arduino.h>

#include "bms_link.h"
#include "jbd_protocol.h"

namespace jbd {

class JbdUartLink final : public IBmsLink {
 public:
  struct Config {
    int             rxPin    = 18;      // ESP32 RX  <- BMS UART-TXD
    int             txPin    = 17;      // ESP32 TX  -> BMS UART-RXD
    uint32_t        baud     = 9600;    // datasheet s4.10.1
    uint32_t        timeoutMs = 500;
    uint8_t         attempts = 3;       // the first request after idle can drop
    HardwareSerial *port     = &Serial1;
  };

  // Config{} cannot be a default argument here: default member initializers of a
  // nested class are not complete until the enclosing class is. Two constructors
  // instead - the defaulted one picks up Config's initializers via cfg_.
  JbdUartLink() = default;
  explicit JbdUartLink(const Config &cfg) : cfg_(cfg) {}

  const char *name() const override { return "UART"; }
  bool begin() override;
  bool poll(BmsSnapshot &out) override;
  void stats(LinkStats &out) const override { out = stats_; }

 private:
  bool   readByte(uint8_t &b, uint32_t deadline);
  size_t receive(uint32_t deadline);
  bool   readRegister(uint8_t reg, Frame &f);

  Config    cfg_;
  LinkStats stats_ = {};
  uint8_t   rxBuf_[kMaxFrameBytes] = {};
};

}  // namespace jbd
