/*
 * twai_selftest.ino
 * -----------------------------------------------------------------------------
 * Proves the ESP32-S3 TWAI (CAN) controller, GPIO routing and CAN transceiver
 * work BEFORE the BMS is attached. No other CAN node is needed.
 *
 * Mode: TWAI_MODE_NO_ACK + self-reception request. The controller transmits a
 * frame and receives its own frame back; no ACK from another node is required.
 *
 *   Step A (no transceiver): jumper wire GPIO TWAI_TX_PIN <-> GPIO TWAI_RX_PIN.
 *   Step B (transceiver)   : TX -> transceiver TXD/CTX, RX <- transceiver RXD/CRX,
 *                            3V3 + GND to the module, CANH/CANL left open
 *                            (module's onboard 120 ohm stays fitted).
 *
 * PASS: every transmitted frame comes back with identical ID and payload,
 *       TEC/REC stay 0, state stays RUNNING.
 */

#include <Arduino.h>
#include <string.h>
#include "driver/twai.h"

#ifndef TWAI_TX_PIN
#define TWAI_TX_PIN 4
#endif
#ifndef TWAI_RX_PIN
#define TWAI_RX_PIN 5
#endif

static uint32_t sent = 0, echoed = 0, mismatched = 0, lost = 0, txErr = 0;

static const char *stateStr(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default:                    return "?";
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(200);
  Serial.printf("\n# twai_selftest  TX=GPIO%d  RX=GPIO%d  500 kbit/s  NO_ACK + self-rx\n",
                TWAI_TX_PIN, TWAI_RX_PIN);

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN,
                                                        (gpio_num_t)TWAI_RX_PIN,
                                                        TWAI_MODE_NO_ACK);
  g.rx_queue_len = 16;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t e = twai_driver_install(&g, &t, &f);
  if (e == ESP_OK) e = twai_start();
  if (e != ESP_OK) {
    Serial.printf("# TWAI init failed: %s\n", esp_err_to_name(e));
    for (;;) delay(1000);
  }
}

void loop() {
  twai_message_t tx = {};
  tx.identifier = 0x123;
  tx.self = 1;                         // self-reception request
  tx.data_length_code = 8;
  for (uint8_t i = 0; i < 8; i++) tx.data[i] = uint8_t((sent >> ((i & 3) * 8)) ^ (0xA0 + i));

  const esp_err_t e = twai_transmit(&tx, pdMS_TO_TICKS(100));
  sent++;
  if (e != ESP_OK) {
    txErr++;
    Serial.printf("# twai_transmit: %s\n", esp_err_to_name(e));
  }

  twai_message_t rx;
  if (twai_receive(&rx, pdMS_TO_TICKS(100)) == ESP_OK) {
    const bool same = rx.identifier == tx.identifier && rx.data_length_code == 8 &&
                      memcmp(rx.data, tx.data, 8) == 0;
    if (same) echoed++; else mismatched++;
  } else {
    lost++;
  }

  twai_status_info_t st;
  twai_get_status_info(&st);

  if (sent % 10 == 0) {
    const bool pass = (echoed == sent) && st.tx_error_counter == 0 && st.state == TWAI_STATE_RUNNING;
    Serial.printf("sent %lu  echoed %lu  mismatch %lu  lost %lu  txerr %lu | %s TEC %lu REC %lu buserr %lu -> %s\n",
                  (unsigned long)sent, (unsigned long)echoed, (unsigned long)mismatched,
                  (unsigned long)lost, (unsigned long)txErr, stateStr(st.state),
                  (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter,
                  (unsigned long)st.bus_error_count, pass ? "PASS" : "FAIL");
  }

  if (st.state == TWAI_STATE_BUS_OFF) {
    Serial.printf("# BUS_OFF: TX not reaching RX. Check jumper / transceiver 3V3+GND / "
                  "TXD-RXD orientation / standby pin (Rs or S must be LOW)\n");
    twai_initiate_recovery();
    delay(500);
    twai_start();
  }
  delay(100);
}
