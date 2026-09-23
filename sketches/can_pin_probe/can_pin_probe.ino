/*
 * can_pin_probe.ino
 * -----------------------------------------------------------------------------
 * Separates two failure causes that both look like "Remote frame NOT ACKed":
 *   (a) the transceiver path ESP32-TX -> transceiver -> bus -> transceiver -> ESP32-RX
 *       is broken (unpowered, mis-wired, TX/RX crossed), versus
 *   (b) the transceiver is fine and the BMS simply is not answering on CAN.
 *
 * Method: TWAI_MODE_NO_ACK + self-reception request. The controller transmits and
 * receives its own frame; no second node is needed to ACK. If the frame comes back,
 * the whole analogue path works. Tries BOTH pin orders in one run.
 *
 * Read-only safety: transmits ONLY remote frames with ID 0x104, which is inside the
 * permitted 0x100-0x110 range and is the same request jbd_can_reader already sends.
 * No data frame is ever transmitted.
 */

#include <Arduino.h>
#include "driver/twai.h"

static constexpr uint32_t PROBE_ID     = 0x104;   // permitted read ID (cell/NTC count)
static constexpr int      FRAMES       = 20;
static constexpr uint32_t ECHO_WAIT_MS = 60;

static const char *stateStr(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default:                    return "?";
  }
}

// Returns the number of self-received echoes.
static int probe(int txPin, int rxPin) {
  Serial.printf("\n=== probe  TX=GPIO%d  RX=GPIO%d ===\n", txPin, rxPin);

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)txPin,
                                                        (gpio_num_t)rxPin,
                                                        TWAI_MODE_NO_ACK);
  g.rx_queue_len = 16;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t e = twai_driver_install(&g, &t, &f);
  if (e != ESP_OK) { Serial.printf("  install failed: %s\n", esp_err_to_name(e)); return -1; }
  if ((e = twai_start()) != ESP_OK) {
    Serial.printf("  start failed: %s\n", esp_err_to_name(e));
    twai_driver_uninstall();
    return -1;
  }

  int sent = 0, echoed = 0, lost = 0, foreign = 0;
  for (int i = 0; i < FRAMES; i++) {
    twai_message_t tx = {};
    tx.identifier       = PROBE_ID;
    tx.rtr              = 1;      // remote frame only - never a data frame
    tx.self             = 1;      // self-reception request
    tx.data_length_code = 4;
    if (twai_transmit(&tx, pdMS_TO_TICKS(50)) == ESP_OK) sent++;

    bool got = false;
    const uint32_t deadline = millis() + ECHO_WAIT_MS;
    while ((int32_t)(deadline - millis()) > 0) {
      twai_message_t rx;
      if (twai_receive(&rx, pdMS_TO_TICKS(10)) != ESP_OK) continue;
      if (rx.identifier == PROBE_ID && rx.rtr) { echoed++; got = true; break; }
      foreign++;                                   // anything else is a real reply
      Serial.printf("  !! FOREIGN FRAME  %03lX%s [%u]",
                    (unsigned long)rx.identifier, rx.rtr ? " RTR" : "", rx.data_length_code);
      if (!rx.rtr) for (uint8_t k = 0; k < rx.data_length_code && k < 8; k++)
        Serial.printf(" %02X", rx.data[k]);
      Serial.printf("   <- the BMS answered!\n");
    }
    if (!got) lost++;
    delay(20);
  }

  twai_status_info_t st;
  twai_get_status_info(&st);
  Serial.printf("  sent %d  echoed %d  lost %d  foreign %d | %s TEC %lu REC %lu buserr %lu txfail %lu\n",
                sent, echoed, lost, foreign, stateStr(st.state),
                (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter,
                (unsigned long)st.bus_error_count, (unsigned long)st.tx_failed_count);
  Serial.printf("  -> transceiver path %s\n", echoed > 0 ? "WORKS" : "NOT proven");

  twai_stop();
  twai_driver_uninstall();
  return echoed;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(200);
  Serial.printf("\n# can_pin_probe - remote-frame self-reception on both pin orders\n");
  Serial.printf("# 500 kbit/s, NO_ACK + self-rx, ID 0x%03X RTR only (no data frames)\n", PROBE_ID);

  const int a = probe(4, 5);
  const int b = probe(5, 4);

  Serial.printf("\n===================== VERDICT =====================\n");
  if (a <= 0 && b <= 0) {
    Serial.printf("Neither pin order echoed.\n"
                  "  -> The TX->transceiver->RX path is NOT working.\n"
                  "     Check: module VCC on 3V3 and GND on GND (measure 3.3 V across them),\n"
                  "     the Rs/S/STB standby pin must be LOW, and that GPIO4/GPIO5 really\n"
                  "     land on the module's two logic pins.\n");
  } else if (a > b) {
    Serial.printf("TX=GPIO4 RX=GPIO5 is correct (%d echoes vs %d).\n"
                  "  -> Transceiver is ALIVE. Silence is coming from the BMS.\n", a, b);
  } else {
    Serial.printf("TX=GPIO5 RX=GPIO4 is correct (%d echoes vs %d).\n"
                  "  -> Transceiver is ALIVE. Silence is coming from the BMS.\n", b, a);
  }
  Serial.printf("===================================================\n");
}

void loop() { delay(1000); }
