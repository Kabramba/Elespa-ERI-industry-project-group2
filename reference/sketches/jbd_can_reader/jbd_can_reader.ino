/*
 * jbd_can_reader.ino
 * -----------------------------------------------------------------------------
 * Read-only telemetry from the JBD SP24S007 BMS over CAN (connector J6).
 *
 * JBD "Protection board CANBUS communication protocol":
 *   - 500 kbit/s, 11-bit standard IDs (CAN 2.0B controller)
 *   - QUERY-BASED: the BMS never broadcasts. The host sends a REMOTE frame with
 *     the wanted ID; the BMS answers with a DATA frame of the same ID.
 *     Data frames sent to the BMS are ignored by it.
 *   - Payload big-endian; last 2 bytes = CRC-16 (poly 0xA001, init 0xFFFF,
 *     i.e. CRC-16/MODBUS) over the preceding bytes. The document does not state
 *     the CRC byte order explicitly, so both orders are checked and reported.
 *
 * This sketch transmits ONLY remote frames for IDs 0x100-0x110. It never sends
 * a data frame. Single-shot transmission is used so an un-ACKed request is not
 * retransmitted endlessly.
 *
 * IMPORTANT: the BMS only answers CAN while it is AWAKE. On its own this sketch
 * will get nothing but 'Remote frame NOT ACKed' once the BMS has idled for a few
 * minutes. Use sketches/jbd_dual_reader, which polls UART first to wake it.
 * Pins below are the measured-correct ones: TX=GPIO5, RX=GPIO4.
 *
 * Wiring (ESP32-S3 + 3.3 V transceiver, e.g. SN65HVD230 module):
 *   GPIO TWAI_TX_PIN -> transceiver TXD/CTX
 *   GPIO TWAI_RX_PIN <- transceiver RXD/CRX
 *   3V3, GND         -> transceiver
 *   transceiver CANH -> BMS J6 pin 2 (CAN-High)
 *   transceiver CANL -> BMS J6 pin 1 (CAN-Low)
 *   ESP32 GND        -> BMS UART-GND (J10/J8 pin 1) - keep from the UART phase
 */

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "driver/twai.h"

#ifndef TWAI_TX_PIN
#define TWAI_TX_PIN 5
#endif
#ifndef TWAI_RX_PIN
#define TWAI_RX_PIN 4
#endif

// Declared up here: the Arduino preprocessor injects prototypes for the functions
// below, and those signatures name these enums.
enum CrcKind : uint8_t { CRC_BE, CRC_LE, CRC_ZERO, CRC_BAD, CRC_NA };
enum ReqResult : uint8_t { REQ_OK, REQ_NO_ACK, REQ_ACKED_NO_REPLY, REQ_TX_STUCK, REQ_TX_ERR };

static constexpr uint32_t POLL_PERIOD_MS   = 2000;
static constexpr uint32_t REPLY_TIMEOUT_MS = 150;
static constexpr uint32_t LISTEN_FIRST_MS  = 2000;   // detect any broadcasting firmware first
static constexpr bool     PRINT_RAW        = true;

// ---- JBD CAN identifiers ------------------------------------------------------
static constexpr uint32_t ID_PACK       = 0x100;  // pack V (10 mV), current (10 mA, s16), remaining (10 mAh)
static constexpr uint32_t ID_CAPACITY   = 0x101;  // full cap (10 mAh), cycles, RSOC (%)
static constexpr uint32_t ID_BAL_PROT   = 0x102;  // balance cells 1-16, balance 17-32, protection flags
static constexpr uint32_t ID_MOS_DATE   = 0x103;  // MOS status, production date, sw version
static constexpr uint32_t ID_CONFIG     = 0x104;  // cell count, NTC count  (DLC 4)
static constexpr uint32_t ID_NTC_1_3    = 0x105;  // NTC1..3, 0.1 K
static constexpr uint32_t ID_NTC_4_6    = 0x106;  // NTC4..6, 0.1 K
static constexpr uint32_t ID_CELL_FIRST = 0x107;  // 3 cells per frame, mV
static constexpr uint32_t ID_CELL_LAST  = 0x110;  // cells 28..30

static const char *const PROT_NAMES[13] = {
  "CELL_OV", "CELL_UV", "PACK_OV", "PACK_UV", "CHG_OT", "CHG_UT", "DSG_OT",
  "DSG_UT", "CHG_OC", "DSG_OC", "SHORT", "AFE_ERR", "MOS_SW_LOCK"
};

// ---- Helpers ------------------------------------------------------------------
static inline uint16_t be16(const uint8_t *p) { return (uint16_t(p[0]) << 8) | p[1]; }

static uint16_t crc16Modbus(const uint8_t *p, uint8_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= *p++;
    for (uint8_t i = 0; i < 8; i++) crc = (crc & 1) ? uint16_t((crc >> 1) ^ 0xA001) : uint16_t(crc >> 1);
  }
  return crc;
}

static const char *crcStr(CrcKind k) {
  switch (k) {
    case CRC_BE:   return "crc=ok(BE)";
    case CRC_LE:   return "crc=ok(LE)";
    case CRC_ZERO: return "crc=absent(0000)";
    case CRC_BAD:  return "crc=MISMATCH";
    default:       return "crc=n/a";
  }
}

static CrcKind checkCrc(const twai_message_t &m) {
  if (m.data_length_code < 3) return CRC_NA;
  const uint8_t n = m.data_length_code - 2;
  const uint16_t c  = crc16Modbus(m.data, n);
  const uint16_t be = (uint16_t(m.data[n]) << 8) | m.data[n + 1];
  const uint16_t le = (uint16_t(m.data[n + 1]) << 8) | m.data[n];
  if (c == be) return CRC_BE;
  if (c == le) return CRC_LE;
  if (be == 0) return CRC_ZERO;
  return CRC_BAD;
}

static void logFrame(const char *tag, const twai_message_t &m, const char *suffix = "") {
  if (m.extd) Serial.printf("%s %08lX", tag, (unsigned long)m.identifier);
  else        Serial.printf("%s %03lX", tag, (unsigned long)m.identifier);
  Serial.printf("%s [%u]", m.rtr ? " RTR" : "", m.data_length_code);
  if (!m.rtr) for (uint8_t i = 0; i < m.data_length_code && i < 8; i++) Serial.printf(" %02X", m.data[i]);
  Serial.printf(" %s\n", suffix);
}

static const char *stateStr(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default:                    return "?";
  }
}

// ---- Decoded state ------------------------------------------------------------
struct JbdCan {
  bool     haveConfig = false;
  uint8_t  nCells = 0, nNtc = 0;
  float    packV = NAN, currentA = NAN, remainAh = NAN, fullAh = NAN;
  uint16_t cycles = 0, rsoc = 0, balLo = 0, balHi = 0, prot = 0, mos = 0, date = 0, sw = 0;
  float    ntcC[6] = {NAN, NAN, NAN, NAN, NAN, NAN};
  uint16_t cellMv[30] = {0};
};
static JbdCan S;

static void decode(const twai_message_t &m) {
  if (m.data_length_code < 2) return;
  const uint8_t *d = m.data;
  const uint8_t payload = m.data_length_code - 2;   // bytes before the CRC
  const uint32_t id = m.identifier;
  switch (id) {
    case ID_PACK:
      if (payload < 6) return;
      S.packV    = be16(d) * 0.01f;
      S.currentA = int16_t(be16(d + 2)) * 0.01f;      // + charge, - discharge
      S.remainAh = be16(d + 4) * 0.01f;
      break;
    case ID_CAPACITY:
      if (payload < 6) return;
      S.fullAh = be16(d) * 0.01f;
      S.cycles = be16(d + 2);
      S.rsoc   = be16(d + 4);
      break;
    case ID_BAL_PROT:
      if (payload < 6) return;
      S.balLo = be16(d);
      S.balHi = be16(d + 2);
      S.prot  = be16(d + 4);
      break;
    case ID_MOS_DATE:
      if (payload < 6) return;
      S.mos  = be16(d);                                // bit0 CHG MOS, bit1 DSG MOS
      S.date = be16(d + 2);                            // (Y-2000)<<9 | M<<5 | D
      S.sw   = be16(d + 4);
      break;
    case ID_CONFIG:
      S.nCells = d[0];
      S.nNtc   = d[1];
      S.haveConfig = true;
      break;
    case ID_NTC_1_3:
    case ID_NTC_4_6: {
      const uint8_t base = (id == ID_NTC_1_3) ? 0 : 3;
      for (uint8_t i = 0; i < 3; i++)
        if (2 * i + 1 < payload) S.ntcC[base + i] = (int(be16(d + 2 * i)) - 2731) * 0.1f;
      break;
    }
    default:
      if (id >= ID_CELL_FIRST && id <= ID_CELL_LAST) {
        const uint8_t base = uint8_t((id - ID_CELL_FIRST) * 3);
        for (uint8_t i = 0; i < 3; i++)
          if (base + i < 30 && 2 * i + 1 < payload) S.cellMv[base + i] = be16(d + 2 * i);
      }
      break;
  }
}

// ---- Transport ----------------------------------------------------------------

static ReqResult request(uint32_t id, uint8_t dlc, twai_message_t &reply) {
  uint32_t alerts = 0;
  twai_read_alerts(&alerts, 0);                     // clear stale alerts

  twai_message_t q = {};
  q.identifier = id;
  q.rtr = 1;                                        // remote frame = read request
  // Single shot: can_pin_probe showed that allowing retransmission puts the
  // controller into a permanent error-passive retry loop on this bus.
  q.ss = 1;
  q.data_length_code = dlc;
  const esp_err_t e = twai_transmit(&q, pdMS_TO_TICKS(20));
  if (e != ESP_OK) {
    Serial.printf("# TX 0x%03lX: %s\n", (unsigned long)id, esp_err_to_name(e));
    return REQ_TX_ERR;
  }

  alerts = 0;
  twai_read_alerts(&alerts, pdMS_TO_TICKS(20));
  const bool acked  = alerts & TWAI_ALERT_TX_SUCCESS;
  const bool failed = alerts & TWAI_ALERT_TX_FAILED;

  const uint32_t deadline = millis() + REPLY_TIMEOUT_MS;
  while ((int32_t)(deadline - millis()) > 0) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(10)) != ESP_OK) continue;
    if (!m.extd && !m.rtr && m.identifier == id) { reply = m; return REQ_OK; }
    logFrame("RX (other)", m);
  }
  if (failed) return REQ_NO_ACK;
  if (acked)  return REQ_ACKED_NO_REPLY;
  return REQ_TX_STUCK;
}

static void diagnose(ReqResult r) {
  switch (r) {
    case REQ_NO_ACK:
      Serial.printf("# Remote frame NOT ACKed: no CAN node is listening.\n"
                    "#   -> swap CAN-H/CAN-L, confirm 500 kbit/s, wake the BMS (run the UART\n"
                    "#      reader or apply a load), check ground reference and termination.\n");
      break;
    case REQ_ACKED_NO_REPLY:
      Serial.printf("# Frame ACKed but no reply: the BMS CAN controller is alive.\n"
                    "#   -> BMS CAN address may be non-zero (datasheet default CAN_ID_0),\n"
                    "#      or this firmware uses a different CAN protocol. Check JBDTools.\n");
      break;
    case REQ_TX_STUCK:
      Serial.printf("# TX never completed: bus stuck/dominant or transceiver not driving.\n"
                    "#   -> transceiver 3V3/GND, TXD/RXD orientation, standby pin LOW.\n");
      break;
    default: break;
  }
}

static void printState() {
  Serial.printf("Pack    %.2f V   %+.2f A   %+.1f W   remaining %.2f Ah\n",
                S.packV, S.currentA, S.packV * S.currentA, S.remainAh);
  Serial.printf("SoC     %u %%   full %.2f Ah   cycles %u\n", S.rsoc, S.fullAh, S.cycles);
  Serial.printf("MOS     CHG %s   DSG %s   sw 0x%04X   mfg %04u-%02u-%02u\n",
                (S.mos & 1) ? "ON" : "off", (S.mos & 2) ? "ON" : "off", S.sw,
                2000 + (S.date >> 9), (S.date >> 5) & 0x0F, S.date & 0x1F);
  Serial.printf("Prot    0x%04X", S.prot);
  for (uint8_t i = 0; i < 13; i++) if (S.prot & (1u << i)) Serial.printf(" %s", PROT_NAMES[i]);
  Serial.printf("\nBalance 0x%04X%04X\nTemps  ", S.balHi, S.balLo);
  for (uint8_t i = 0; i < S.nNtc && i < 6; i++) Serial.printf(" T%u=%.1fC", i + 1, S.ntcC[i]);
  Serial.printf("\n");
  uint16_t lo = 0xFFFF, hi = 0;
  uint32_t sum = 0;
  const uint8_t n = S.nCells > 30 ? 30 : S.nCells;
  for (uint8_t i = 0; i < n; i++) {
    Serial.printf("%sC%02u=%u", (i % 8) ? "  " : (i ? "\n        " : "Cells   "), i + 1, S.cellMv[i]);
    if (S.cellMv[i] < lo) lo = S.cellMv[i];
    if (S.cellMv[i] > hi) hi = S.cellMv[i];
    sum += S.cellMv[i];
  }
  if (n) Serial.printf("\n        min %u  max %u  delta %u mV   sum %.2f V\n", lo, hi, hi - lo, sum / 1000.0f);
}

static void printBusStatus() {
  twai_status_info_t st;
  twai_get_status_info(&st);
  Serial.printf("# bus %s  TEC %lu  REC %lu  txfail %lu  buserr %lu  rxmiss %lu\n",
                stateStr(st.state), (unsigned long)st.tx_error_counter,
                (unsigned long)st.rx_error_counter, (unsigned long)st.tx_failed_count,
                (unsigned long)st.bus_error_count, (unsigned long)st.rx_missed_count);
}

static void recoverIfBusOff() {
  twai_status_info_t st;
  twai_get_status_info(&st);
  if (st.state == TWAI_STATE_BUS_OFF) {
    Serial.printf("# BUS_OFF -> initiating recovery\n");
    twai_initiate_recovery();
    delay(100);
  }
  twai_get_status_info(&st);
  if (st.state == TWAI_STATE_STOPPED) twai_start();
}

// Request one ID; returns true on a valid reply
static bool poll(uint32_t id, uint8_t dlc, ReqResult *outResult = nullptr) {
  twai_message_t m;
  const ReqResult r = request(id, dlc, m);
  if (outResult) *outResult = r;
  if (r != REQ_OK) {
    Serial.printf("# no reply for 0x%03lX\n", (unsigned long)id);
    return false;
  }
  const CrcKind k = checkCrc(m);
  if (PRINT_RAW) logFrame("RX", m, crcStr(k));
  decode(m);
  return true;
}

static bool crcSelfTest() {
  const uint8_t v[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  return crc16Modbus(v, sizeof(v)) == 0x4B37;         // CRC-16/MODBUS check value
}

// ---- Arduino entry points -----------------------------------------------------
static uint32_t lastPoll = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(200);
  Serial.printf("\n# jbd_can_reader  TX=GPIO%d  RX=GPIO%d  500 kbit/s  (remote-frame queries only)\n",
                TWAI_TX_PIN, TWAI_RX_PIN);
  Serial.printf("# CRC-16/MODBUS self-test %s\n", crcSelfTest() ? "PASS" : "FAIL");

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN,
                                                        (gpio_num_t)TWAI_RX_PIN,
                                                        TWAI_MODE_NORMAL);
  g.rx_queue_len = 32;
  g.tx_queue_len = 4;
  g.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR |
                     TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                     TWAI_ALERT_RX_QUEUE_FULL;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t e = twai_driver_install(&g, &t, &f);
  if (e == ESP_OK) e = twai_start();
  if (e != ESP_OK) {
    Serial.printf("# TWAI init failed: %s\n", esp_err_to_name(e));
    for (;;) delay(1000);
  }

  Serial.printf("# listening %lu ms for unsolicited frames...\n", (unsigned long)LISTEN_FIRST_MS);
  uint32_t seen = 0;
  const uint32_t until = millis() + LISTEN_FIRST_MS;
  while ((int32_t)(until - millis()) > 0) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(20)) == ESP_OK) { logFrame("RX (broadcast)", m); seen++; }
  }
  Serial.printf("# %lu unsolicited frame(s) (0 is expected: JBD is query-based)\n", (unsigned long)seen);
}

void loop() {
  if (millis() - lastPoll < POLL_PERIOD_MS) return;
  lastPoll = millis();
  recoverIfBusOff();
  Serial.printf("\n---- t=%lu ms ----\n", (unsigned long)millis());

  ReqResult r;
  if (!poll(ID_CONFIG, 4, &r)) {                     // first request doubles as a link check
    diagnose(r);
    printBusStatus();
    return;
  }
  poll(ID_PACK, 8);
  poll(ID_CAPACITY, 8);
  poll(ID_BAL_PROT, 8);
  poll(ID_MOS_DATE, 8);
  poll(ID_NTC_1_3, 8);
  if (S.nNtc > 3) poll(ID_NTC_4_6, 8);
  const uint8_t cellFrames = uint8_t((S.nCells + 2) / 3);
  for (uint8_t i = 0; i < cellFrames && ID_CELL_FIRST + i <= ID_CELL_LAST; i++) poll(ID_CELL_FIRST + i, 8);

  Serial.printf("Config  %u cells, %u NTC\n", S.nCells, S.nNtc);
  printState();
  printBusStatus();
}
