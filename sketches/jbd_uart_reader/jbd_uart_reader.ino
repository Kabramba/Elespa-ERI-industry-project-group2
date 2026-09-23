/*
 * jbd_uart_reader.ino
 * -----------------------------------------------------------------------------
 * Read-only telemetry from the yellow pack's JBD (Jiabaida) BMS over UART.
 *
 * BMS    : JBD SP24S007 V1.1 (label) - 8-24S LiFePO4, 100 A, UART/RS485/CAN
 * Link   : non-isolated UART, 9600 baud, 8N1       [DS: JBD-SP24S004 s4.10.1 p.11]
 * Ports  : J10 = GND, RXD, TXD, VDD(NC)            [DS: JBD-SP24S004 s7 p.14-15]
 *          J8  = GND, RXD, TXD, VDD(11-15 V)  <- never connect pin 4 to the ESP32
 * Proto  : JBD protocol V4 frame  DD | A5 | reg | len | data | chk_hi chk_lo | 77
 *          chk = 0x10000 - sum(bytes from reg/status through end of data)
 *
 * This sketch only ever sends the three READ requests (0x03, 0x04, 0x05).
 * It contains no write (0x5A) frame builder by design.
 *
 * Wiring
 *   BMS UART-GND  ->  ESP32 GND        (B- side ground; NOT the Anderson negative)
 *   BMS UART-TXD  ->  ESP32 BMS_RX_PIN (1 kOhm series resistor recommended)
 *   BMS UART-RXD  <-  ESP32 BMS_TX_PIN
 *   BMS UART-VDD  ->  not connected
 *   Logic level: measure TXD idle vs GND first. ~3.3 V -> direct. ~5 V -> level-shift.
 */

#include <Arduino.h>
#include <string.h>

// ---- Link configuration (ESP32-S3) ------------------------------------------
// GPIO18/17 are the ESP32-S3 UART1 default pins (U1RXD/U1TXD) and are free on
// DevKitC-1 style boards. Avoid 0,3,45,46 (strap), 19/20 (USB), 26-37 (flash/PSRAM),
// 43/44 (UART0 console).
#ifndef BMS_RX_PIN
#define BMS_RX_PIN 18   // ESP32 RX  <- BMS UART-TXD
#endif
#ifndef BMS_TX_PIN
#define BMS_TX_PIN 17   // ESP32 TX  -> BMS UART-RXD
#endif

static constexpr uint32_t BMS_BAUD        = 9600;
static constexpr uint32_t POLL_PERIOD_MS  = 1000;
static constexpr uint32_t RESP_TIMEOUT_MS = 500;   // 24-cell 0x04 reply = 55 B ~ 57 ms on the wire
static constexpr uint8_t  MAX_ATTEMPTS    = 3;     // first request after BMS sleep can be dropped
static constexpr bool     PRINT_RAW       = true;  // hex-dump every request and response

HardwareSerial &BmsSerial = Serial1;

// ---- Protocol constants -----------------------------------------------------
static constexpr uint8_t JBD_START = 0xDD;
static constexpr uint8_t JBD_READ  = 0xA5;
static constexpr uint8_t JBD_END   = 0x77;
static constexpr uint8_t REG_BASIC = 0x03;   // basic info and status
static constexpr uint8_t REG_CELLS = 0x04;   // cell voltages
static constexpr uint8_t REG_HWVER = 0x05;   // hardware version string

struct JbdFrame {
  uint8_t reg;
  uint8_t status;   // 0x00 ok, 0x80 error
  uint8_t len;
  uint8_t data[255];
};

struct JbdBasic {
  float    packV, currentA, remainAh, nominalAh;
  uint16_t cycles, prodYear;
  uint8_t  prodMonth, prodDay;
  uint32_t balanceBits;      // bit n = cell n+1 balancing
  uint16_t protection;
  uint8_t  swVersion, rsocPct, fet, nCells, nNtc;
  float    ntcC[8];
};

struct JbdCells {
  uint8_t  n;
  uint16_t mv[32];
};

enum JbdResult : int8_t {
  JBD_OK = 0, JBD_TIMEOUT = -1, JBD_BAD_LEN = -2, JBD_BAD_END = -3,
  JBD_BAD_CHK = -4, JBD_BAD_REG = -5, JBD_BMS_ERR = -6
};

static const char *jbdResultStr(JbdResult r) {
  switch (r) {
    case JBD_OK:      return "ok";
    case JBD_TIMEOUT: return "timeout (check TX/RX swap, GND, logic level, BMS awake)";
    case JBD_BAD_LEN: return "bad length";
    case JBD_BAD_END: return "missing 0x77 end byte (baud or framing)";
    case JBD_BAD_CHK: return "checksum mismatch (noise, level or baud)";
    case JBD_BAD_REG: return "reply for a different register";
    case JBD_BMS_ERR: return "BMS returned error status 0x80";
  }
  return "?";
}

static inline uint16_t be16(const uint8_t *p) { return (uint16_t(p[0]) << 8) | p[1]; }

// 0x10000 - sum(first, len, data[0..len-1]), truncated to 16 bits
static uint16_t jbdChecksum(uint8_t first, uint8_t len, const uint8_t *data) {
  uint16_t c = 0;
  c -= first;
  c -= len;
  for (uint16_t i = 0; i < len; i++) c -= data[i];
  return c;
}

static void hexDump(const char *tag, const uint8_t *p, size_t n) {
  Serial.printf("%s", tag);
  for (size_t i = 0; i < n; i++) Serial.printf(" %02X", p[i]);
  Serial.printf("\n");
}

// ---- Frame parsing (pure; shared by UART path and self-test) -----------------
static JbdResult jbdParse(const uint8_t *b, size_t n, uint8_t expectReg, JbdFrame &f) {
  if (n < 7 || b[0] != JBD_START) return JBD_BAD_LEN;
  f.reg = b[1];
  f.status = b[2];
  f.len = b[3];
  if (n != size_t(f.len) + 7) return JBD_BAD_LEN;
  memcpy(f.data, b + 4, f.len);
  const uint16_t rxChk = (uint16_t(b[4 + f.len]) << 8) | b[5 + f.len];
  if (b[6 + f.len] != JBD_END) return JBD_BAD_END;
  if (jbdChecksum(f.status, f.len, f.data) != rxChk) return JBD_BAD_CHK;
  if (f.reg != expectReg) return JBD_BAD_REG;
  if (f.status != 0x00) return JBD_BMS_ERR;
  return JBD_OK;
}

// ---- UART transport ---------------------------------------------------------
static uint8_t  rxBuf[4 + 255 + 3];
static uint16_t rxLen  = 0;
static uint16_t rxJunk = 0;   // bytes discarded before 0xDD

static void jbdSendRead(uint8_t reg) {
  const uint16_t c = jbdChecksum(reg, 0, nullptr);
  const uint8_t f[7] = {JBD_START, JBD_READ, reg, 0x00, uint8_t(c >> 8), uint8_t(c & 0xFF), JBD_END};
  while (BmsSerial.available()) BmsSerial.read();   // drop stale bytes
  BmsSerial.write(f, sizeof(f));
  if (PRINT_RAW) hexDump(">>", f, sizeof(f));
}

static bool readByte(uint8_t &b, uint32_t deadline) {
  while ((int32_t)(deadline - millis()) > 0) {
    const int c = BmsSerial.read();
    if (c >= 0) { b = uint8_t(c); return true; }
    delay(1);
  }
  return false;
}

static JbdResult jbdReceive(uint8_t expectReg, JbdFrame &f) {
  const uint32_t deadline = millis() + RESP_TIMEOUT_MS;
  rxLen = 0;
  rxJunk = 0;
  uint8_t b;
  for (;;) {                                        // hunt for start byte
    if (!readByte(b, deadline)) return JBD_TIMEOUT;
    if (b == JBD_START) break;
    rxJunk++;
  }
  rxBuf[rxLen++] = b;
  while (rxLen < 4) {                               // reg, status, len
    if (!readByte(b, deadline)) return JBD_TIMEOUT;
    rxBuf[rxLen++] = b;
  }
  const uint16_t total = 4 + rxBuf[3] + 3;          // + data + chk(2) + end
  while (rxLen < total) {
    if (!readByte(b, deadline)) return JBD_TIMEOUT;
    rxBuf[rxLen++] = b;
  }
  return jbdParse(rxBuf, rxLen, expectReg, f);
}

static JbdResult jbdQuery(uint8_t reg, JbdFrame &f) {
  JbdResult r = JBD_TIMEOUT;
  for (uint8_t attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
    jbdSendRead(reg);
    r = jbdReceive(reg, f);
    if (PRINT_RAW && rxLen) hexDump("<<", rxBuf, rxLen);
    if (PRINT_RAW && rxJunk) Serial.printf("# %u junk byte(s) before 0xDD\n", rxJunk);
    if (r == JBD_OK) return r;
    Serial.printf("# reg 0x%02X attempt %u/%u: %s\n", reg, attempt, MAX_ATTEMPTS, jbdResultStr(r));
    delay(100);
  }
  return r;
}

// ---- Decoders ---------------------------------------------------------------
// Register 0x03 layout, big-endian [REF: JBD protocol V4]
static bool jbdDecodeBasic(const JbdFrame &f, JbdBasic &o) {
  if (f.len < 23) return false;
  const uint8_t *d = f.data;
  o.packV      = be16(d + 0) * 0.01f;              // 10 mV
  o.currentA   = int16_t(be16(d + 2)) * 0.01f;     // 10 mA, + = charge, - = discharge
  o.remainAh   = be16(d + 4) * 0.01f;              // 10 mAh
  o.nominalAh  = be16(d + 6) * 0.01f;              // 10 mAh
  o.cycles     = be16(d + 8);
  const uint16_t date = be16(d + 10);              // (Y-2000)<<9 | M<<5 | D
  o.prodYear   = 2000 + (date >> 9);
  o.prodMonth  = (date >> 5) & 0x0F;
  o.prodDay    = date & 0x1F;
  o.balanceBits = uint32_t(be16(d + 12)) | (uint32_t(be16(d + 14)) << 16);
  o.protection = be16(d + 16);
  o.swVersion  = d[18];
  o.rsocPct    = d[19];
  o.fet        = d[20];                            // bit0 CHG MOS, bit1 DSG MOS
  o.nCells     = d[21];
  uint8_t n    = d[22];
  if (n > 8) n = 8;
  if (23 + 2 * n > f.len) n = (f.len - 23) / 2;
  for (uint8_t i = 0; i < n; i++)                  // 0.1 K, offset 2731
    o.ntcC[i] = (int(be16(d + 23 + 2 * i)) - 2731) * 0.1f;
  o.nNtc = n;
  return true;
}

static bool jbdDecodeCells(const JbdFrame &f, JbdCells &o) {
  o.n = f.len / 2;
  if (o.n > 32) o.n = 32;
  for (uint8_t i = 0; i < o.n; i++) o.mv[i] = be16(f.data + 2 * i);
  return o.n > 0;
}

// Protection status bits [REF: JBD protocol V4]
static const char *const PROT_NAMES[13] = {
  "CELL_OV", "CELL_UV", "PACK_OV", "PACK_UV", "CHG_OT", "CHG_UT", "DSG_OT",
  "DSG_UT", "CHG_OC", "DSG_OC", "SHORT", "AFE_ERR", "MOS_SW_LOCK"
};

// ---- Output -----------------------------------------------------------------
static void printBasic(const JbdBasic &b) {
  Serial.printf("Pack    %.2f V   %+.2f A   %+.1f W\n", b.packV, b.currentA, b.packV * b.currentA);
  Serial.printf("SoC     %u %%   %.2f / %.2f Ah   cycles %u\n",
                b.rsocPct, b.remainAh, b.nominalAh, b.cycles);
  Serial.printf("MOS     CHG %s   DSG %s\n", (b.fet & 1) ? "ON" : "off", (b.fet & 2) ? "ON" : "off");
  Serial.printf("Temps  ");
  for (uint8_t i = 0; i < b.nNtc; i++) Serial.printf(" T%u=%.1fC", i + 1, b.ntcC[i]);
  Serial.printf("\nProt    0x%04X", b.protection);
  for (uint8_t i = 0; i < 13; i++) if (b.protection & (1u << i)) Serial.printf(" %s", PROT_NAMES[i]);
  Serial.printf("\nBalance 0x%08lX\n", (unsigned long)b.balanceBits);
  Serial.printf("Config  %u cells, %u NTC, sw 0x%02X, mfg %04u-%02u-%02u\n",
                b.nCells, b.nNtc, b.swVersion, b.prodYear, b.prodMonth, b.prodDay);
}

static void printCells(const JbdCells &c, float packV) {
  uint16_t lo = 0xFFFF, hi = 0;
  uint32_t sum = 0;
  for (uint8_t i = 0; i < c.n; i++) {
    Serial.printf("%sC%02u=%u", (i % 8) ? "  " : (i ? "\n        " : "Cells   "), i + 1, c.mv[i]);
    if (c.mv[i] < lo) lo = c.mv[i];
    if (c.mv[i] > hi) hi = c.mv[i];
    sum += c.mv[i];
  }
  Serial.printf("\n        min %u  max %u  delta %u mV   sum %.2f V (pack %.2f V)\n",
                lo, hi, hi - lo, sum / 1000.0f, packV);
}

// ---- Self-test: documented example frame from JBD protocol V4 ---------------
// Expected: 58.88 V, 0.00 A, 7.20/10.00 Ah, 72 %, 15 cells, 2 NTC = 20.3 / 21.5 C
static const uint8_t SELFTEST_FRAME[] = {
  0xDD, 0x03, 0x00, 0x1B,
  0x17, 0x00, 0x00, 0x00, 0x02, 0xD0, 0x03, 0xE8, 0x00, 0x00, 0x20, 0x78,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x48, 0x03, 0x0F, 0x02, 0x0B,
  0x76, 0x0B, 0x82,
  0xFB, 0xFF, 0x77
};

static bool runSelfTest() {
  JbdFrame f;
  JbdBasic b;
  const JbdResult r = jbdParse(SELFTEST_FRAME, sizeof(SELFTEST_FRAME), REG_BASIC, f);
  const uint16_t reqChk = jbdChecksum(REG_BASIC, 0, nullptr);
  const bool ok = (r == JBD_OK) && jbdDecodeBasic(f, b) && reqChk == 0xFFFD &&
                  b.nCells == 15 && b.rsocPct == 72 && b.nNtc == 2 &&
                  b.packV > 58.87f && b.packV < 58.89f &&
                  b.ntcC[0] > 20.25f && b.ntcC[0] < 20.35f;
  Serial.printf("# self-test %s\n", ok ? "PASS" : "FAIL");
  return ok;
}

// ---- Arduino entry points ---------------------------------------------------
static bool     haveHwVer = false;
static uint32_t lastPoll  = 0;

static void tryHwVersion() {
  JbdFrame f;
  if (jbdQuery(REG_HWVER, f) == JBD_OK) {
    Serial.printf("# BMS hardware: %.*s\n", f.len, (const char *)f.data);
    haveHwVer = true;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);   // native-USB CDC: wait for host
  delay(200);
  Serial.printf("\n# jbd_uart_reader - JBD BMS UART reader (read-only)\n");
  runSelfTest();
  BmsSerial.begin(BMS_BAUD, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
  Serial.printf("# UART1 %lu 8N1  RX=GPIO%d  TX=GPIO%d\n",
                (unsigned long)BMS_BAUD, BMS_RX_PIN, BMS_TX_PIN);
  tryHwVersion();
}

void loop() {
  if (millis() - lastPoll < POLL_PERIOD_MS) return;
  lastPoll = millis();

  JbdFrame f;
  JbdBasic basic;
  JbdCells cells;

  Serial.printf("\n---- t=%lu ms ----\n", (unsigned long)millis());
  if (jbdQuery(REG_BASIC, f) == JBD_OK && jbdDecodeBasic(f, basic)) {
    printBasic(basic);
    if (!haveHwVer) tryHwVersion();
    if (jbdQuery(REG_CELLS, f) == JBD_OK && jbdDecodeCells(f, cells)) printCells(cells, basic.packV);
  }
}
