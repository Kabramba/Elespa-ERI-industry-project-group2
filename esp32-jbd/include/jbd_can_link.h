// jbd_can_link.h - JBD CAN protocol over the ESP32 TWAI peripheral.
//
// Query based: the BMS never broadcasts. We send a REMOTE frame, it answers
// with a DATA frame of the same id.
//
// PINS: TX = GPIO5, RX = GPIO4. This is the opposite of what context.md states
// and was established by measurement (see safe-save-copy-esp32/can_pin_probe).
//
// TRANSMISSION MODE: see JBD_CAN_HARDWARE_RETRANSMIT in app_config.h. Both
// modes are fully implemented; the toggle picks one.
#pragma once

#include <Arduino.h>
#include "driver/twai.h"

#include "app_config.h"
#include "bms_link.h"
#include "jbd_protocol.h"

namespace jbd {

class JbdCanLink final : public IBmsLink {
 public:
  struct Config {
    int txPin = 5;  // -> transceiver D / TXD / CTX
    int rxPin = 4;  // <- transceiver R / RXD / CRX

    // Master toggle. Defaults from app_config.h; overridable per instance.
    bool hardwareRetransmit = (JBD_CAN_HARDWARE_RETRANSMIT != 0);

    // Software retries when single shot. The measured per-attempt reply rate
    // has been as low as 5 %, so retries carry the link.
    uint8_t retries = 4;

    // Software retries when the controller is already retransmitting. These
    // exist only for the second failure mode - frame ACKed, no reply - so a
    // smaller number is right; more would just stack duplicates.
    uint8_t retriesHardware = 2;

    // Hardware mode only. A frame nothing ever ACKs is retried forever, so this
    // is the bounded escape: above it the peripheral is stopped and restarted.
    // 0 disables. Bus-off is 256.
    //
    // Healthy operation on this bus was measured oscillating up to TEC 169, so
    // 192 sat close enough to trip on an ordinary transient. 224 leaves normal
    // behaviour alone while still catching a genuine runaway.
    uint16_t hardwareRecoverTec = 224;

    uint32_t replyTimeoutMs = 100;
  };

  JbdCanLink() = default;
  explicit JbdCanLink(const Config &cfg) : cfg_(cfg) {}

  const char *name() const override { return "CAN"; }
  bool begin() override;
  bool poll(BmsSnapshot &out) override;
  void stats(LinkStats &out) const override;

  bool        hardwareRetransmit() const { return cfg_.hardwareRetransmit; }
  const char *modeName() const { return cfg_.hardwareRetransmit ? "hardware-retransmit" : "single-shot"; }

  // True after the last poll saw the controller in BUS_OFF.
  bool wasBusOff() const { return busOff_; }
  // True if the last request never got an ACK (single shot only - the alert
  // that reports this does not fire when the controller is retransmitting).
  bool lastTxUnacked() const { return lastTxUnacked_; }

 private:
  bool request(uint32_t id, uint8_t dlc, twai_message_t &reply);
  void maintain();

  Config    cfg_;
  LinkStats stats_          = {};
  bool      up_             = false;
  bool      busOff_         = false;
  bool      lastTxUnacked_  = false;
};

}  // namespace jbd
