/*
 * can_noack_impact.ino
 * -----------------------------------------------------------------------------
 * Does running a TWAI_MODE_NO_ACK probe against a live BMS break the link for
 * subsequent NORMAL-mode traffic?
 *
 * Rationale: in NO_ACK mode this node never acknowledges frames it receives. The
 * BMS therefore sees an ACK error on every reply it sends, retransmits, and walks
 * its own transmit error counter up - potentially to error-passive or bus-off.
 * That would make NORMAL mode fail immediately afterwards, which is exactly what
 * was observed in the first session and wrongly attributed to the BMS sleeping.
 *
 *   A  NORMAL  baseline
 *   B  NO_ACK  burst (the suspected poison)
 *   C  NORMAL  immediately after B
 *   D  NORMAL  after 25 s of bus idle (recovery check)
 *
 * If C collapses and D recovers, NO_ACK probing is the culprit.
 * Read-only: remote frames, id 0x104 only. No data frames, UART never used.
 */

#include <Arduino.h>
#include "driver/twai.h"

#define TWAI_TX_PIN 5
#define TWAI_RX_PIN 4

static constexpr uint32_t ID = 0x104;
static constexpr int      N  = 15;

static bool bringUp(twai_mode_t mode) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN,
                                                        (gpio_num_t)TWAI_RX_PIN, mode);
  g.rx_queue_len   = 32;
  g.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  return twai_start() == ESP_OK;
}
static void tearDown() { twai_stop(); twai_driver_uninstall(); }

// Returns replies received. `self` adds a self-reception request (NO_ACK probing).
static int burst(const char *label, twai_mode_t mode, int n, bool self) {
  if (!bringUp(mode)) { Serial.printf("%-28s install FAILED\n", label); return -1; }
  int acked = 0, replies = 0;
  for (int i = 0; i < n; i++) {
    uint32_t a = 0; twai_read_alerts(&a, 0);
    twai_message_t q = {};
    q.identifier = ID; q.rtr = 1; q.ss = 1; q.data_length_code = 4;
    if (self) q.self = 1;
    twai_transmit(&q, pdMS_TO_TICKS(50));
    a = 0; twai_read_alerts(&a, pdMS_TO_TICKS(15));
    if (a & TWAI_ALERT_TX_SUCCESS) acked++;
    const uint32_t dl = millis() + 100;
    while ((int32_t)(dl - millis()) > 0) {
      twai_message_t m;
      if (twai_receive(&m, pdMS_TO_TICKS(5)) != ESP_OK) continue;
      if (m.rtr || m.identifier != ID) continue;
      replies++; break;
    }
    delay(40);
  }
  twai_status_info_t st; twai_get_status_info(&st);
  Serial.printf("%-28s replies %2d/%-2d (%3d%%)  ACKed %2d  TEC %3lu  buserr %lu\n",
                label, replies, n, replies * 100 / n, acked,
                (unsigned long)st.tx_error_counter, (unsigned long)st.bus_error_count);
  tearDown();
  return replies;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(200);
  Serial.printf("\n# can_noack_impact  TX=GPIO%d RX=GPIO%d\n\n", TWAI_TX_PIN, TWAI_RX_PIN);

  const int a = burst("A NORMAL baseline",        TWAI_MODE_NORMAL, N, false);
  delay(500);
  const int b = burst("B NO_ACK burst (poison)",  TWAI_MODE_NO_ACK, 20, true);
  delay(200);
  const int c = burst("C NORMAL right after",     TWAI_MODE_NORMAL, N, false);
  Serial.printf("   ... 25 s bus idle ...\n");
  delay(25000);
  const int d = burst("D NORMAL after idle",      TWAI_MODE_NORMAL, N, false);

  Serial.printf("\n===================== VERDICT =====================\n");
  if (a > 0 && c == 0 && d > 0)
    Serial.printf("CONFIRMED: the NO_ACK burst kills NORMAL mode (A=%d, C=%d), and it\n"
                  "recovers after an idle period (D=%d). Do NOT run NO_ACK probes\n"
                  "against a live BMS - it drives the BMS's own error counters up.\n", a, c, d);
  else if (a > 0 && c > 0)
    Serial.printf("NOT reproduced: NORMAL still works after NO_ACK (A=%d, C=%d, D=%d).\n"
                  "The original failure had some other cause.\n", a, c, d);
  else
    Serial.printf("Inconclusive: A=%d B=%d C=%d D=%d\n", a, b, c, d);
  Serial.printf("===================================================\n");
}

void loop() { delay(1000); }
