/*
 * can_normal_probe.ino
 * -----------------------------------------------------------------------------
 * can_pin_probe proved (in TWAI_MODE_NO_ACK) that the BMS receives our remote
 * frames and answers them. But in TWAI_MODE_NORMAL the controller reports the
 * request as un-ACKed. NO_ACK self-reception can be satisfied internally, so it
 * does not by itself prove the RX pin reads the bus - however the BMS data frames
 * we caught there DID arrive through RX, so both directions work at least sometimes.
 *
 * This probe answers one question: in NORMAL mode, how many remote-frame requests
 * actually get an answer? It sends the request repeatedly with a generous receive
 * window and logs every frame that arrives, ACK or no ACK.
 *
 * Single-shot is kept ON: with retransmission enabled the controller sits in a
 * permanent error-passive retry loop and floods the bus.
 *
 * Read-only: remote frames with ID 0x104 only. No data frame is ever sent.
 */

#include <Arduino.h>
#include "driver/twai.h"

#define TWAI_TX_PIN 5          // proven by can_pin_probe
#define TWAI_RX_PIN 4

static constexpr uint32_t PROBE_ID = 0x104;
static constexpr int      ROUNDS   = 30;
static constexpr uint32_t WAIT_MS  = 100;

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
  Serial.printf("\n# can_normal_probe  TX=GPIO%d RX=GPIO%d  NORMAL mode  500 kbit/s\n",
                TWAI_TX_PIN, TWAI_RX_PIN);
  Serial.printf("# %d x remote frame 0x%03X, %lu ms receive window each\n",
                ROUNDS, PROBE_ID, (unsigned long)WAIT_MS);

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN,
                                                        (gpio_num_t)TWAI_RX_PIN,
                                                        TWAI_MODE_NORMAL);
  g.rx_queue_len   = 32;
  g.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t e = twai_driver_install(&g, &t, &f);
  if (e == ESP_OK) e = twai_start();
  if (e != ESP_OK) { Serial.printf("# init failed: %s\n", esp_err_to_name(e)); return; }

  int txOk = 0, acked = 0, txFailed = 0, replies = 0;

  for (int i = 0; i < ROUNDS; i++) {
    uint32_t a = 0;
    twai_read_alerts(&a, 0);                       // clear stale

    twai_message_t q = {};
    q.identifier = PROBE_ID;
    q.rtr = 1;
    q.ss  = 1;                                     // no retransmit storm
    q.data_length_code = 4;
    if (twai_transmit(&q, pdMS_TO_TICKS(50)) == ESP_OK) txOk++;

    a = 0;
    twai_read_alerts(&a, pdMS_TO_TICKS(20));
    if (a & TWAI_ALERT_TX_SUCCESS) acked++;
    if (a & TWAI_ALERT_TX_FAILED)  txFailed++;

    const uint32_t deadline = millis() + WAIT_MS;
    while ((int32_t)(deadline - millis()) > 0) {
      twai_message_t m;
      if (twai_receive(&m, pdMS_TO_TICKS(10)) != ESP_OK) continue;
      if (m.rtr) continue;                         // our own echo, ignore
      replies++;
      Serial.printf("  [%02d] REPLY %03lX [%u]", i, (unsigned long)m.identifier,
                    m.data_length_code);
      for (uint8_t k = 0; k < m.data_length_code && k < 8; k++) Serial.printf(" %02X", m.data[k]);
      Serial.printf("\n");
    }
    delay(30);
  }

  twai_status_info_t st;
  twai_get_status_info(&st);
  Serial.printf("\n===================== RESULT =====================\n");
  Serial.printf("rounds %d | tx queued %d | ACKed %d | TX_FAILED %d | REPLIES %d\n",
                ROUNDS, txOk, acked, txFailed, replies);
  Serial.printf("bus %s  TEC %lu  REC %lu  buserr %lu  txfail %lu  rxmiss %lu\n",
                stateStr(st.state), (unsigned long)st.tx_error_counter,
                (unsigned long)st.rx_error_counter, (unsigned long)st.bus_error_count,
                (unsigned long)st.tx_failed_count, (unsigned long)st.rx_missed_count);
  if (replies > 0)
    Serial.printf("-> NORMAL mode DOES get answers (%d/%d = %d%%). The link works but is lossy.\n",
                  replies, ROUNDS, replies * 100 / ROUNDS);
  else
    Serial.printf("-> No answers in NORMAL mode at all.\n");
  Serial.printf("==================================================\n");
}

void loop() { delay(1000); }
