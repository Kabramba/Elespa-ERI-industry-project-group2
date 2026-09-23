/*
 * jbd_dual_reader.ino
 * -----------------------------------------------------------------------------
 * Runs the UART link and the CAN link together on the same BMS.
 *
 * Purpose: the CAN transceiver and both jumpers are proven good (rx_pin_check),
 * and the BMS answered a CAN remote frame once - but only while the UART reader
 * had recently been polling it. The BMS lowers its sampling rate after ~5 min with
 * no communication [DS: JBD-SP24S004 s4.8]. This sketch polls UART first (which
 * wakes the BMS and gives ground truth) and then immediately queries CAN, so CAN
 * is always exercised against a BMS that is known to be awake this millisecond.
 *
 * Pins: UART1 RX=18 TX=17 @9600 8N1 | TWAI TX=5 RX=4 @500 kbit/s
 * Read-only: UART register 0x03 only; CAN remote frames in 0x100-0x110 only.
 */

#include <Arduino.h>
#include <string.h>
#include "driver/twai.h"

#define BMS_RX_PIN  18
#define BMS_TX_PIN  17
#define TWAI_TX_PIN 5
#define TWAI_RX_PIN 4

static constexpr uint8_t  REG_BASIC = 0x03;
static constexpr uint8_t  CAN_RETRIES = 4;   // ~65 % per attempt -> >98 % over 4

HardwareSerial &BmsSerial = Serial1;

static const char *stateStr(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED: return "STOPPED";
    case TWAI_STATE_RUNNING: return "RUNNING";
    case TWAI_STATE_BUS_OFF: return "BUS_OFF";
    default:                 return "RECOVERING";
  }
}

static inline uint16_t be16(const uint8_t *p) { return (uint16_t(p[0]) << 8) | p[1]; }

static uint16_t jbdChecksum(uint8_t first, uint8_t len, const uint8_t *d) {
  uint16_t c = 0;
  c -= first; c -= len;
  for (uint16_t i = 0; i < len; i++) c -= d[i];
  return c;
}

static uint8_t rxBuf[300];
static uint16_t rxLen;

static bool readByte(uint8_t &b, uint32_t deadline) {
  while ((int32_t)(deadline - millis()) > 0) {
    const int c = BmsSerial.read();
    if (c >= 0) { b = uint8_t(c); return true; }
    delay(1);
  }
  return false;
}

// Read register `reg`; returns payload length or -1.
static int uartRead(uint8_t reg, uint8_t *out) {
  const uint16_t c = jbdChecksum(reg, 0, nullptr);
  const uint8_t req[7] = {0xDD, 0xA5, reg, 0x00, uint8_t(c >> 8), uint8_t(c & 0xFF), 0x77};
  while (BmsSerial.available()) BmsSerial.read();
  BmsSerial.write(req, sizeof(req));

  const uint32_t deadline = millis() + 500;
  rxLen = 0;
  uint8_t b;
  for (;;) {
    if (!readByte(b, deadline)) return -1;
    if (b == 0xDD) break;
  }
  rxBuf[rxLen++] = b;
  while (rxLen < 4) { if (!readByte(b, deadline)) return -1; rxBuf[rxLen++] = b; }
  const uint16_t total = 4 + rxBuf[3] + 3;
  while (rxLen < total) { if (!readByte(b, deadline)) return -1; rxBuf[rxLen++] = b; }
  if (rxBuf[1] != reg || rxBuf[2] != 0x00) return -1;
  const uint16_t got = (uint16_t(rxBuf[4 + rxBuf[3]]) << 8) | rxBuf[5 + rxBuf[3]];
  if (jbdChecksum(rxBuf[2], rxBuf[3], rxBuf + 4) != got) return -1;
  memcpy(out, rxBuf + 4, rxBuf[3]);
  return rxBuf[3];
}

static uint16_t crc16Modbus(const uint8_t *p, uint8_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) { crc ^= *p++; for (uint8_t i = 0; i < 8; i++) crc = (crc & 1) ? uint16_t((crc >> 1) ^ 0xA001) : uint16_t(crc >> 1); }
  return crc;
}

// CRC-16/MODBUS, big-endian, over all bytes before the last two. Verified against
// the live 0x104 frame (11 05 -> E3 CD).
static bool canCrcOk(const twai_message_t &m) {
  if (m.data_length_code < 3) return true;
  const uint8_t n = m.data_length_code - 2;
  return crc16Modbus(m.data, n) == ((uint16_t(m.data[n]) << 8) | m.data[n + 1]);
}

static uint32_t canAttempts = 0, canWins = 0, canCrcBad = 0;

// One remote-frame request, retried. Returns true and fills `reply` on success.
static bool canGet(uint32_t id, uint8_t dlc, twai_message_t &reply) {
  for (uint8_t attempt = 0; attempt < CAN_RETRIES; attempt++) {
    canAttempts++;
    uint32_t a = 0;
    twai_read_alerts(&a, 0);

    twai_message_t q = {};
    q.identifier = id; q.rtr = 1; q.ss = 1; q.data_length_code = dlc;
    if (twai_transmit(&q, pdMS_TO_TICKS(50)) != ESP_OK) { delay(5); continue; }
    a = 0;
    twai_read_alerts(&a, pdMS_TO_TICKS(15));

    const uint32_t deadline = millis() + 100;
    while ((int32_t)(deadline - millis()) > 0) {
      twai_message_t m;
      if (twai_receive(&m, pdMS_TO_TICKS(5)) != ESP_OK) continue;
      if (m.rtr || m.identifier != id) continue;
      if (!canCrcOk(m)) { canCrcBad++; continue; }
      reply = m;
      canWins++;
      return true;
    }
    twai_status_info_t st;
    twai_get_status_info(&st);
    if (st.state == TWAI_STATE_BUS_OFF) { twai_initiate_recovery(); delay(60); twai_start(); }
    delay(8);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(200);
  Serial.printf("\n# jbd_dual_reader  UART1 RX=%d TX=%d @9600 | TWAI TX=%d RX=%d @500k\n",
                BMS_RX_PIN, BMS_TX_PIN, TWAI_TX_PIN, TWAI_RX_PIN);

  BmsSerial.begin(9600, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN,
                                                        (gpio_num_t)TWAI_RX_PIN,
                                                        TWAI_MODE_NORMAL);
  g.rx_queue_len   = 32;
  g.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t e = twai_driver_install(&g, &t, &f);
  if (e == ESP_OK) e = twai_start();
  if (e != ESP_OK) { Serial.printf("# TWAI init failed: %s\n", esp_err_to_name(e)); }
}

void loop() {
  static uint32_t cycle = 0;
  uint8_t d[260];

  Serial.printf("\n======== cycle %lu   t=%lu ms ========\n",
                (unsigned long)++cycle, (unsigned long)millis());

  // ---- UART: wakes the BMS and provides ground truth ----
  const int n = uartRead(REG_BASIC, d);
  if (n < 23) {
    Serial.printf("UART: no valid reply (%d). Skipping CAN this cycle.\n", n);
    delay(1000);
    return;
  }
  const float    uPackV = be16(d) * 0.01f;
  const float    uCurA  = int16_t(be16(d + 2)) * 0.01f;
  const float    uRemAh = be16(d + 4) * 0.01f;
  const uint8_t  uSoc   = d[19];
  const uint8_t  uCells = d[21];
  const uint8_t  uNtc   = d[22];

  uint8_t c[260];
  const int cn = uartRead(0x04, c);
  uint16_t uCellMv[32] = {0};
  const uint8_t uCellN = (cn > 0) ? uint8_t(cn / 2) : 0;
  for (uint8_t i = 0; i < uCellN && i < 32; i++) uCellMv[i] = be16(c + 2 * i);

  // ---- CAN: BMS is provably awake right now ----
  twai_message_t m;
  float    cPackV = -1, cCurA = 0, cRemAh = 0;
  uint16_t cSoc = 0;
  uint8_t  cCells = 0, cNtc = 0;
  uint16_t cCellMv[30] = {0};
  bool     gotPack = false, gotCap = false, gotCfg = false;

  if (canGet(0x104, 4, m)) { cCells = m.data[0]; cNtc = m.data[1]; gotCfg = true; }
  if (canGet(0x100, 8, m)) {
    cPackV = be16(m.data) * 0.01f;
    cCurA  = int16_t(be16(m.data + 2)) * 0.01f;
    cRemAh = be16(m.data + 4) * 0.01f;
    gotPack = true;
  }
  if (canGet(0x101, 8, m)) { cSoc = be16(m.data + 4); gotCap = true; }

  const uint8_t frames = uint8_t((cCells + 2) / 3);
  uint8_t cellFramesOk = 0;
  for (uint8_t i = 0; i < frames && (0x107 + i) <= 0x110; i++) {
    if (!canGet(0x107 + i, 8, m)) continue;
    cellFramesOk++;
    const uint8_t base = uint8_t(i * 3);
    for (uint8_t k = 0; k < 3; k++)
      if (base + k < 30 && 2 * k + 1 < m.data_length_code - 2)
        cCellMv[base + k] = be16(m.data + 2 * k);
  }

  // ---- side by side ----
  Serial.printf("%-14s %12s %12s %10s\n", "field", "UART", "CAN", "delta");
  Serial.printf("%-14s %12.2f %12s %10s\n", "pack V", uPackV,
                gotPack ? String(cPackV, 2).c_str() : "-",
                gotPack ? String(cPackV - uPackV, 2).c_str() : "-");
  Serial.printf("%-14s %12.2f %12s %10s\n", "current A", uCurA,
                gotPack ? String(cCurA, 2).c_str() : "-",
                gotPack ? String(cCurA - uCurA, 2).c_str() : "-");
  Serial.printf("%-14s %12.2f %12s %10s\n", "remaining Ah", uRemAh,
                gotPack ? String(cRemAh, 2).c_str() : "-",
                gotPack ? String(cRemAh - uRemAh, 2).c_str() : "-");
  Serial.printf("%-14s %12u %12s %10s\n", "SoC %", uSoc,
                gotCap ? String(cSoc).c_str() : "-",
                gotCap ? String(int(cSoc) - int(uSoc)).c_str() : "-");
  Serial.printf("%-14s %12u %12s %10s\n", "cell count", uCells,
                gotCfg ? String(cCells).c_str() : "-", (gotCfg && cCells == uCells) ? "match" : "-");
  Serial.printf("%-14s %12u %12s %10s\n", "NTC count", uNtc,
                gotCfg ? String(cNtc).c_str() : "-", (gotCfg && cNtc == uNtc) ? "match" : "-");

  if (cellFramesOk) {
    uint8_t same = 0, cmp = 0; int worst = 0;
    for (uint8_t i = 0; i < uCellN && i < 30; i++) {
      if (!cCellMv[i] && !uCellMv[i]) continue;
      if (!cCellMv[i]) continue;
      cmp++;
      const int dmv = int(cCellMv[i]) - int(uCellMv[i]);
      if (dmv == 0) same++;
      if (abs(dmv) > abs(worst)) worst = dmv;
    }
    Serial.printf("cells        : %u/%u CAN frames, %u compared, %u identical, worst delta %d mV\n",
                  cellFramesOk, frames, cmp, same, worst);
    Serial.printf("  CAN  ");
    for (uint8_t i = 0; i < uCellN && i < 30; i++) Serial.printf(" %u", cCellMv[i]);
    Serial.printf("\n  UART ");
    for (uint8_t i = 0; i < uCellN && i < 30; i++) Serial.printf(" %u", uCellMv[i]);
    Serial.printf("\n");
  } else {
    Serial.printf("cells        : no CAN cell frames this cycle\n");
  }

  twai_status_info_t st;
  twai_get_status_info(&st);
  Serial.printf("CAN link     : %lu/%lu frames won (%lu%%), crc-bad %lu | bus %s TEC %lu buserr %lu\n",
                (unsigned long)canWins, (unsigned long)canAttempts,
                canAttempts ? (unsigned long)(canWins * 100 / canAttempts) : 0,
                (unsigned long)canCrcBad, stateStr(st.state),
                (unsigned long)st.tx_error_counter, (unsigned long)st.bus_error_count);

  if (st.state == TWAI_STATE_BUS_OFF) { twai_initiate_recovery(); delay(100); twai_start(); }
  delay(1500);
}
