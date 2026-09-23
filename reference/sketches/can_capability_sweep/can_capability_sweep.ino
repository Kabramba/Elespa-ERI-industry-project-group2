/*
 * can_capability_sweep.ino
 * -----------------------------------------------------------------------------
 * Two questions, one run:
 *
 *  PART 1  What does the BMS actually expose on CAN? Enumerates EVERY id from
 *          0x100 to 0x110 inclusive, prints the raw frame and a decode, and says
 *          which ids answer and which stay silent.
 *
 *  PART 2  Does CAN traffic alone keep the BMS awake, or is a UART poll required?
 *          After one UART wake, UART is never touched again: 0x104 is requested
 *          every 2 s for 5 minutes and the hit rate is reported per 15 s bucket.
 *          The datasheet puts the sleep threshold near 5 min [DS s4.8], so a decay
 *          to zero inside this window answers it. Finishes by proving on UART that
 *          the BMS is merely asleep, then re-querying CAN to show it revives.
 *
 * Read-only: UART register 0x03 only, CAN remote frames 0x100-0x110 only.
 */

#include <Arduino.h>
#include <string.h>
#include "driver/twai.h"

#define BMS_RX_PIN  18
#define BMS_TX_PIN  17
#define TWAI_TX_PIN 5
#define TWAI_RX_PIN 4

static constexpr uint8_t  RETRIES      = 5;
static constexpr uint32_t ENDURANCE_MS = 300000;   // 5 minutes, CAN only
static constexpr uint32_t BUCKET_MS    = 15000;

HardwareSerial &BmsSerial = Serial1;

static const char *const PROT_NAMES[13] = {
  "CELL_OV","CELL_UV","PACK_OV","PACK_UV","CHG_OT","CHG_UT","DSG_OT",
  "DSG_UT","CHG_OC","DSG_OC","SHORT","AFE_ERR","MOS_SW_LOCK"
};

static inline uint16_t be16(const uint8_t *p) { return (uint16_t(p[0]) << 8) | p[1]; }

static uint16_t crc16Modbus(const uint8_t *p, uint8_t n) {
  uint16_t c = 0xFFFF;
  while (n--) { c ^= *p++; for (uint8_t i = 0; i < 8; i++) c = (c & 1) ? uint16_t((c >> 1) ^ 0xA001) : uint16_t(c >> 1); }
  return c;
}
static bool crcOk(const twai_message_t &m) {
  if (m.data_length_code < 3) return true;
  const uint8_t n = m.data_length_code - 2;
  return crc16Modbus(m.data, n) == ((uint16_t(m.data[n]) << 8) | m.data[n + 1]);
}

// ---- UART (wake + ground truth) ----
static uint16_t jbdChecksum(uint8_t first, uint8_t len, const uint8_t *d) {
  uint16_t c = 0; c -= first; c -= len;
  for (uint16_t i = 0; i < len; i++) c -= d[i];
  return c;
}
static bool readByte(uint8_t &b, uint32_t dl) {
  while ((int32_t)(dl - millis()) > 0) { const int c = BmsSerial.read(); if (c >= 0) { b = c; return true; } delay(1); }
  return false;
}
static int uartRead(uint8_t reg, uint8_t *out) {
  const uint16_t c = jbdChecksum(reg, 0, nullptr);
  const uint8_t req[7] = {0xDD,0xA5,reg,0x00,uint8_t(c>>8),uint8_t(c&0xFF),0x77};
  while (BmsSerial.available()) BmsSerial.read();
  BmsSerial.write(req, sizeof(req));
  const uint32_t dl = millis() + 500;
  uint8_t buf[300]; uint16_t n = 0, b;
  uint8_t t;
  for (;;) { if (!readByte(t, dl)) return -1; if (t == 0xDD) break; }
  buf[n++] = t;
  while (n < 4) { if (!readByte(t, dl)) return -1; buf[n++] = t; }
  const uint16_t total = 4 + buf[3] + 3;
  while (n < total) { if (!readByte(t, dl)) return -1; buf[n++] = t; }
  if (buf[1] != reg || buf[2] != 0x00) return -1;
  b = (uint16_t(buf[4+buf[3]]) << 8) | buf[5+buf[3]];
  if (jbdChecksum(buf[2], buf[3], buf+4) != b) return -1;
  memcpy(out, buf+4, buf[3]);
  return buf[3];
}

// ---- CAN ----
static bool canGet(uint32_t id, uint8_t dlc, twai_message_t &reply, uint8_t tries = RETRIES) {
  for (uint8_t a = 0; a < tries; a++) {
    uint32_t al = 0; twai_read_alerts(&al, 0);
    twai_message_t q = {};
    q.identifier = id; q.rtr = 1; q.ss = 1; q.data_length_code = dlc;
    if (twai_transmit(&q, pdMS_TO_TICKS(50)) != ESP_OK) { delay(5); continue; }
    al = 0; twai_read_alerts(&al, pdMS_TO_TICKS(15));
    const uint32_t dl = millis() + 100;
    while ((int32_t)(dl - millis()) > 0) {
      twai_message_t m;
      if (twai_receive(&m, pdMS_TO_TICKS(5)) != ESP_OK) continue;
      if (m.rtr || m.identifier != id) continue;
      if (!crcOk(m)) continue;
      reply = m; return true;
    }
    twai_status_info_t st; twai_get_status_info(&st);
    if (st.state == TWAI_STATE_BUS_OFF) { twai_initiate_recovery(); delay(60); twai_start(); }
    delay(8);
  }
  return false;
}

static void decodeAndPrint(uint32_t id, const twai_message_t &m) {
  const uint8_t *d = m.data;
  Serial.printf("  0x%03lX [%u]", (unsigned long)id, m.data_length_code);
  for (uint8_t i = 0; i < m.data_length_code; i++) Serial.printf(" %02X", d[i]);
  Serial.printf("   -> ");
  switch (id) {
    case 0x100:
      Serial.printf("pack %.2f V | current %+.2f A | remaining %.2f Ah",
                    be16(d)*0.01f, int16_t(be16(d+2))*0.01f, be16(d+4)*0.01f); break;
    case 0x101:
      Serial.printf("full capacity %.2f Ah | cycles %u | RSOC %u %%",
                    be16(d)*0.01f, be16(d+2), be16(d+4)); break;
    case 0x102: {
      const uint16_t bl = be16(d), bh = be16(d+2), pr = be16(d+4);
      Serial.printf("balance 1-16 0x%04X | 17-32 0x%04X | protection 0x%04X", bl, bh, pr);
      for (uint8_t i = 0; i < 13; i++) if (pr & (1u<<i)) Serial.printf(" %s", PROT_NAMES[i]);
      break;
    }
    case 0x103: {
      const uint16_t mos = be16(d), dt = be16(d+2), sw = be16(d+4);
      Serial.printf("MOS CHG %s DSG %s | mfg %04u-%02u-%02u | sw 0x%04X",
                    (mos&1)?"ON":"off", (mos&2)?"ON":"off",
                    2000+(dt>>9), (dt>>5)&0x0F, dt&0x1F, sw); break;
    }
    case 0x104: Serial.printf("cell count %u | NTC count %u", d[0], d[1]); break;
    case 0x105: case 0x106: {
      const uint8_t base = (id == 0x105) ? 1 : 4;
      for (uint8_t i = 0; i < 3; i++)
        Serial.printf("T%u=%.1fC ", base+i, (int(be16(d+2*i))-2731)*0.1f);
      break;
    }
    default:
      if (id >= 0x107 && id <= 0x110) {
        const uint8_t base = uint8_t((id-0x107)*3);
        for (uint8_t i = 0; i < 3; i++) Serial.printf("C%02u=%u mV  ", base+i+1, be16(d+2*i));
      } else Serial.printf("(no documented decode)");
      break;
  }
  Serial.printf("\n");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(200);
  Serial.printf("\n# can_capability_sweep\n");

  BmsSerial.begin(9600, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN,
                                                        (gpio_num_t)TWAI_RX_PIN, TWAI_MODE_NORMAL);
  g.rx_queue_len = 32;
  g.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g,&t,&f) == ESP_OK) twai_start();

  uint8_t d[260];
  Serial.printf("\n### UART wake: ");
  const int n = uartRead(0x03, d);
  if (n >= 23) Serial.printf("OK - pack %.2f V, %u cells, %u NTC\n", be16(d)*0.01f, d[21], d[22]);
  else         Serial.printf("FAILED (%d)\n", n);

  // ---------------- PART 1: full id enumeration ----------------
  Serial.printf("\n=========== PART 1: every id 0x100-0x110 ===========\n");
  uint32_t answered = 0, silent = 0;
  char silentList[128] = ""; 
  for (uint32_t id = 0x100; id <= 0x110; id++) {
    twai_message_t m;
    const uint8_t dlc = (id == 0x104) ? 4 : 8;
    if (canGet(id, dlc, m)) { decodeAndPrint(id, m); answered++; }
    else {
      Serial.printf("  0x%03lX  -- no reply --\n", (unsigned long)id);
      silent++;
      char tmp[8]; snprintf(tmp, sizeof(tmp), "%03lX ", (unsigned long)id);
      strncat(silentList, tmp, sizeof(silentList)-strlen(silentList)-1);
    }
  }
  Serial.printf("\n%lu id(s) answered, %lu silent: %s\n",
                (unsigned long)answered, (unsigned long)silent, silent ? silentList : "(none)");

  // ---------------- PART 2: sleep endurance, CAN only ----------------
  Serial.printf("\n=========== PART 2: CAN-only endurance, %lu s ===========\n",
                (unsigned long)(ENDURANCE_MS/1000));
  Serial.printf("UART is NOT touched again until the end.\n");
  Serial.printf("%8s %10s %8s %8s\n", "t(s)", "attempts", "wins", "rate");

  const uint32_t t0 = millis();
  uint32_t bucketStart = millis(), att = 0, win = 0;
  while (millis() - t0 < ENDURANCE_MS) {
    twai_message_t m;
    att++;
    if (canGet(0x104, 4, m, 3)) win++;
    if (millis() - bucketStart >= BUCKET_MS) {
      Serial.printf("%8lu %10lu %8lu %7lu%%\n", (unsigned long)((millis()-t0)/1000),
                    (unsigned long)att, (unsigned long)win,
                    att ? (unsigned long)(win*100/att) : 0);
      bucketStart = millis(); att = 0; win = 0;
    }
    delay(2000);
  }

  Serial.printf("\n### After %lu s with no UART:\n", (unsigned long)(ENDURANCE_MS/1000));
  twai_message_t m;
  Serial.printf("  CAN 0x104 : %s\n", canGet(0x104, 4, m) ? "still answering" : "SILENT");
  const int n2 = uartRead(0x03, d);
  Serial.printf("  UART 0x03 : %s\n", n2 >= 23 ? "answers (so the BMS is alive, just asleep)" : "no reply");
  Serial.printf("  CAN 0x104 immediately after the UART poke : %s\n",
                canGet(0x104, 4, m) ? "ANSWERS -> UART wake is what revives it" : "still silent");
  Serial.printf("\n# done\n");
}

void loop() { delay(1000); }
