/*
 * can_timing_probe.ino
 * -----------------------------------------------------------------------------
 * NORMAL mode fails to see the BMS's ACK bit while NO_ACK mode gets real replies.
 * That is the signature of excessive loop delay (TX -> transceiver -> bus ->
 * transceiver -> RX) relative to where the controller samples the ACK slot.
 *
 * A late sample point, a slower time quantum, or triple sampling can all absorb
 * extra loop delay. This probe walks several 500 kbit/s bit timings in NORMAL mode
 * and reports which - if any - gets the request ACKed and answered.
 *
 * TWAI clock is 80 MHz. bit time = brp * (1 + tseg_1 + tseg_2) quanta.
 * Constraints on ESP32-S3: tseg_1 1..16, tseg_2 1..8, sjw 1..4.
 *
 * Read-only: remote frames with ID 0x104 only, single-shot. No data frames.
 */

#include <Arduino.h>
#include "driver/twai.h"

#define TWAI_TX_PIN 5
#define TWAI_RX_PIN 4

static constexpr uint32_t PROBE_ID = 0x104;
static constexpr int      ROUNDS   = 10;

struct Cfg { const char *name; uint32_t brp; uint8_t t1, t2, sjw; bool triple; };

static const Cfg CFGS[] = {
  { "stock 500k      brp8  t1=15 t2=4",  8, 15, 4, 3, false },
  { "late sample     brp8  t1=16 t2=3",  8, 16, 3, 3, false },
  { "stock + triple  brp8  t1=15 t2=4",  8, 15, 4, 3, true  },
  { "16tq late       brp10 t1=13 t2=2", 10, 13, 2, 2, false },
  { "10tq late       brp16 t1=8  t2=1", 16,  8, 1, 1, false },
  { "10tq + triple   brp16 t1=7  t2=2", 16,  7, 2, 2, true  },
};

static int runCfg(const Cfg &c) {
  const uint32_t tq     = 1u + c.t1 + c.t2;
  const uint32_t bitrate = 80000000UL / (c.brp * tq);
  const uint32_t sample  = (1u + c.t1) * 100u / tq;
  Serial.printf("\n--- %s | %lu tq, %lu bit/s, sample %lu%%%s ---\n",
                c.name, (unsigned long)tq, (unsigned long)bitrate,
                (unsigned long)sample, c.triple ? ", triple" : "");

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN,
                                                        (gpio_num_t)TWAI_RX_PIN,
                                                        TWAI_MODE_NORMAL);
  g.rx_queue_len   = 16;
  g.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED;

  twai_timing_config_t t = {};
  t.brp = c.brp; t.tseg_1 = c.t1; t.tseg_2 = c.t2; t.sjw = c.sjw; t.triple_sampling = c.triple;
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t e = twai_driver_install(&g, &t, &f);
  if (e != ESP_OK) { Serial.printf("  install failed: %s\n", esp_err_to_name(e)); return -1; }
  if ((e = twai_start()) != ESP_OK) {
    Serial.printf("  start failed: %s\n", esp_err_to_name(e));
    twai_driver_uninstall(); return -1;
  }

  int acked = 0, failed = 0, replies = 0;
  for (int i = 0; i < ROUNDS; i++) {
    uint32_t a = 0; twai_read_alerts(&a, 0);
    twai_message_t q = {};
    q.identifier = PROBE_ID; q.rtr = 1; q.ss = 1; q.data_length_code = 4;
    twai_transmit(&q, pdMS_TO_TICKS(50));
    a = 0; twai_read_alerts(&a, pdMS_TO_TICKS(20));
    if (a & TWAI_ALERT_TX_SUCCESS) acked++;
    if (a & TWAI_ALERT_TX_FAILED)  failed++;

    const uint32_t deadline = millis() + 80;
    while ((int32_t)(deadline - millis()) > 0) {
      twai_message_t m;
      if (twai_receive(&m, pdMS_TO_TICKS(10)) != ESP_OK) continue;
      if (m.rtr) continue;
      replies++;
      Serial.printf("    REPLY %03lX [%u]", (unsigned long)m.identifier, m.data_length_code);
      for (uint8_t k = 0; k < m.data_length_code && k < 8; k++) Serial.printf(" %02X", m.data[k]);
      Serial.printf("\n");
    }
    delay(20);
  }

  twai_status_info_t st; twai_get_status_info(&st);
  Serial.printf("  ACKed %d/%d   TX_FAILED %d   replies %d   TEC %lu buserr %lu  %s\n",
                acked, ROUNDS, failed, replies, (unsigned long)st.tx_error_counter,
                (unsigned long)st.bus_error_count,
                (acked > 0 || replies > 0) ? "<<< WORKS" : "");
  twai_stop(); twai_driver_uninstall();
  return (acked > 0 || replies > 0) ? replies + acked : 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(200);
  Serial.printf("\n# can_timing_probe  TX=GPIO%d RX=GPIO%d  NORMAL mode\n", TWAI_TX_PIN, TWAI_RX_PIN);
  Serial.printf("# walking bit timings to absorb transceiver loop delay\n");

  int best = -1; int bestScore = 0;
  for (size_t i = 0; i < sizeof(CFGS) / sizeof(CFGS[0]); i++) {
    const int score = runCfg(CFGS[i]);
    if (score > bestScore) { bestScore = score; best = (int)i; }
  }

  Serial.printf("\n===================== VERDICT =====================\n");
  if (best < 0)
    Serial.printf("No bit timing got an ACK. The fault is analogue, not timing.\n"
                  "  -> Tie the transceiver's Rs / S / STB pin DIRECTLY to GND (high-speed\n"
                  "     mode). A slope-control resistor there is the usual cause.\n"
                  "  -> Then check termination across CANH/CANL.\n");
  else
    Serial.printf("BEST: %s  (score %d)\n", CFGS[best].name, bestScore);
  Serial.printf("===================================================\n");
}

void loop() { delay(1000); }
