// main_activa_obd.cpp - actively ask the Activa's ECU for data.
//
// Listening found silence at every bitrate with no bus errors, which means the
// DLC is diagnostic-only: it says nothing until something asks. This asks.
//
// SAFETY ENVELOPE - from docs/activa-can.md, enforced in code, not convention:
//   * Every transmit goes through txGuarded(). Nothing else may call
//     twai_transmit().
//   * The CAN id must pass can::mayTransmitId() for the active profile:
//     0x7DF / 0x7E0-0x7E7 for 11-bit, 0x18DB33F1 / 0x18DAxxF1 for 29-bit.
//   * The frame must pass obd::isPermittedRequestFrame(): a Single Frame with a
//     read-only SID (01, 02, 03, 07, 09, 0A) or a Flow Control frame. Clear
//     DTCs, session control, ECU reset and security access cannot be encoded.
//   * >= kMinRequestGapMs between transmits, under the 20 requests/s ceiling.
//   * Bus-off or error-passive while transmitting latches TX off until reboot.
//
// Single-shot transmission is used deliberately: TWAI_ALERT_TX_FAILED then
// reports, per frame, whether ANYTHING on the bus acknowledged. That one bit
// separates "wrong bitrate" from "right bitrate, ECU not answering".
#include <Arduino.h>

#include "app_config.h"
#include "can/can_profile_registry.h"
#include "can/twai_profile.h"
#include "obd/obd_isotp.h"
#include "status_led.h"

namespace {

constexpr int      kTxPin           = 5;
constexpr int      kRxPin           = 4;
constexpr uint32_t kMinRequestGapMs = 60;   // ~16/s, under the 20/s ceiling
constexpr uint32_t kReplyWindowMs   = 300;  // P2 is 50 ms; margin for a slow ECU
constexpr uint32_t kBitrates[]      = {500000, 250000, 1000000, 125000};
constexpr size_t   kBitrateCount    = sizeof(kBitrates) / sizeof(kBitrates[0]);

bool                 g_txLatchedOff = false;
uint32_t             g_lastTxMs     = 0;
can::CanBusProfile   g_profile;      // the profile currently driving the bus
bool                 g_haveProfile = false;

// ---------------------------------------------------------------- the gate
// The single point through which any frame may leave this node.
bool txGuarded(uint32_t id, const uint8_t *d, uint8_t dlc, bool *acked) {
  if (acked) *acked = false;
  if (g_txLatchedOff) {
    Serial.printf("   !! TX latched off after a bus fault; not transmitting\n");
    return false;
  }
  if (!g_haveProfile) return false;

  if (!can::mayTransmitId(g_profile, id)) {
    Serial.printf("   !! REFUSED id %lX: not on this profile's allowlist\n",
                  static_cast<unsigned long>(id));
    return false;
  }
  if (!obd::isPermittedRequestFrame(d, dlc)) {
    Serial.printf("   !! REFUSED frame: not a permitted read request\n");
    return false;
  }

  while (millis() - g_lastTxMs < kMinRequestGapMs) delay(2);  // rate limit

  uint32_t alerts = 0;
  twai_read_alerts(&alerts, 0);

  twai_message_t m = {};
  m.identifier       = id;
  m.extd             = (g_profile.bus.frameFormat == can::FrameFormat::kExtended29) ? 1 : 0;
  m.ss               = 1;  // single shot: report this frame's own fate
  m.data_length_code = dlc;
  memcpy(m.data, d, dlc);

  const esp_err_t e = twai_transmit(&m, pdMS_TO_TICKS(50));
  g_lastTxMs        = millis();
  if (e != ESP_OK) {
    Serial.printf("   !! twai_transmit: %s\n", esp_err_to_name(e));
    return false;
  }

  alerts = 0;
  twai_read_alerts(&alerts, pdMS_TO_TICKS(30));
  if (acked) *acked = (alerts & TWAI_ALERT_TX_SUCCESS) != 0;

  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_BUS_OFF) {
    Serial.printf("   !! BUS-OFF while transmitting - latching TX off until reboot\n");
    g_txLatchedOff = true;
    status::show_error_state53();
  }
  return true;
}

void hexDump(const char *tag, uint32_t id, const uint8_t *d, uint8_t dlc) {
  Serial.printf("   %s %0*lX#", tag, (g_profile.bus.frameFormat == can::FrameFormat::kExtended29) ? 8 : 3,
                static_cast<unsigned long>(id));
  for (uint8_t i = 0; i < dlc; ++i) Serial.printf("%02X", d[i]);
  Serial.printf("\n");
}

// Collects a complete ISO-TP response. Returns payload length, 0 on timeout.
size_t receiveIsoTp(uint8_t *out, size_t outMax, uint32_t *fromId) {
  const uint32_t deadline = millis() + kReplyWindowMs;
  size_t         got = 0, expected = 0;
  bool           multi = false;

  while (static_cast<int32_t>(deadline - millis()) > 0) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(10)) != ESP_OK) continue;
    if (m.rtr) continue;
    if (!can::respondsOnId(g_profile, m.identifier)) continue;
    if (fromId) *fromId = m.identifier;
    hexDump("RX", m.identifier, m.data, m.data_length_code);

    switch (obd::pciOf(m.data, m.data_length_code)) {
      case obd::Pci::kSingle: {
        const uint8_t n = obd::singleFrameLength(m.data);
        for (uint8_t i = 0; i < n && got < outMax; ++i) out[got++] = m.data[1 + i];
        return got;
      }
      case obd::Pci::kFirst: {
        expected = obd::firstFrameLength(m.data);
        multi    = true;
        for (uint8_t i = 0; i < 6 && got < outMax; ++i) out[got++] = m.data[2 + i];
        // The ECU waits for Flow Control before sending the rest. It goes to the
        // ECU's PHYSICAL request id, not the functional one we asked on.
        uint8_t fc[8];
        obd::buildFlowControl(fc);
        const uint32_t fcId =
            (g_profile.bus.frameFormat == can::FrameFormat::kExtended29)
                ? (0x18DA00F1u | ((m.identifier & 0xFFu) << 8))
                : (m.identifier - 8u);
        bool acked = false;
        hexDump("TX", fcId, fc, 8);
        txGuarded(fcId, fc, 8, &acked);
        break;
      }
      case obd::Pci::kConsecutive: {
        for (uint8_t i = 0; i < 7 && got < outMax; ++i) out[got++] = m.data[1 + i];
        if (multi && got >= expected) return expected < outMax ? expected : got;
        break;
      }
      default:
        break;
    }
  }
  return got;
}

// Send one request and collect the answer. Returns payload length, 0 if none.
size_t ask(uint8_t sid, const uint8_t *args, uint8_t argCount, uint8_t *out, size_t outMax,
           bool *acked) {
  uint8_t frame[8];
  obd::buildSingleFrame(sid, args, argCount, frame);
  hexDump("TX", g_profile.protocol.requestId, frame, 8);
  if (!txGuarded(g_profile.protocol.requestId, frame, 8, acked)) return 0;
  uint32_t from = 0;
  return receiveIsoTp(out, outMax, &from);
}

bool openBus(const char *profileKey, uint32_t bitrate) {
  const can::CanBusProfile *base = can::findProfile(profileKey);
  if (base == nullptr) return false;
  g_profile        = *base;
  g_profile.bus.txPin = kTxPin;
  g_profile.bus.rxPin = kRxPin;
  g_profile.bus.bitrate = bitrate;
  // Single shot during discovery: hardware retransmission would hide whether an
  // individual frame was acknowledged, which is the signal we need most.
  g_profile.bus.hardwareRetransmit = false;
  const char *why = nullptr;
  if (!can::startFromProfile(g_profile, &why)) {
    Serial.printf("   !! bus would not start: %s\n", why ? why : "?");
    g_haveProfile = false;
    return false;
  }
  g_haveProfile = true;
  return true;
}

void closeBus() {
  can::stopBus();
  g_haveProfile = false;
}

// -------------------------------------------------------------- decoding
void decodeService01(uint8_t pid, const uint8_t *d, size_t n) {
  if (n < 2) return;
  const uint8_t A = (n > 2) ? d[2] : 0;
  const uint8_t B = (n > 3) ? d[3] : 0;
  switch (pid) {
    case 0x05: Serial.printf("      engine temp    : %d C\n", A - 40); break;
    case 0x0C: Serial.printf("      RPM            : %.1f\n", (256.0f * A + B) / 4.0f); break;
    case 0x0D: Serial.printf("      speed          : %u km/h\n", A); break;
    case 0x0F: Serial.printf("      intake air     : %d C\n", A - 40); break;
    case 0x11: Serial.printf("      throttle       : %.1f %%\n", A * 100.0f / 255.0f); break;
    case 0x0B: Serial.printf("      MAP            : %u kPa\n", A); break;
    case 0x04: Serial.printf("      engine load    : %.1f %%\n", A * 100.0f / 255.0f); break;
    case 0x42: Serial.printf("      module voltage : %.3f V\n", (256.0f * A + B) / 1000.0f); break;
    case 0x2F: Serial.printf("      fuel level     : %.1f %%\n", A * 100.0f / 255.0f); break;
    default:   break;  // unknown scaling: the raw bytes are already logged
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(300);
  status::begin(STATUS_LED_BRIGHTNESS_PERCENT);
  status::show_boot();

  Serial.printf("\n###########################################################\n");
  Serial.printf("# Activa OBD-II requester                                 #\n");
  Serial.printf("# Read-only services only: 01 02 03 07 09 0A.             #\n");
  Serial.printf("# Clear-DTC, session control, ECU reset and security      #\n");
  Serial.printf("# access cannot be encoded by this firmware.              #\n");
  Serial.printf("# Scooter must be stationary. Engine off.                 #\n");
  Serial.printf("###########################################################\n");
}

void loop() {
  static bool found = false;
  static char foundKey[16] = {0};
  static uint32_t foundBitrate = 0;

  if (!found) {
    Serial.printf("\n=========== discovery: asking for Service 01 PID 00 ===========\n");
    for (size_t b = 0; b < kBitrateCount && !found; ++b) {
      for (const char *key : {"activa-obd11", "activa-obd29"}) {
        Serial.printf("\n--- %lu bit/s, %s ---\n", static_cast<unsigned long>(kBitrates[b]), key);
        if (!openBus(key, kBitrates[b])) continue;

        const uint8_t pid = 0x00;
        uint8_t       resp[64];
        bool          acked = false;
        const size_t  n     = ask(0x01, &pid, 1, resp, sizeof(resp), &acked);

        twai_status_info_t st;
        twai_get_status_info(&st);
        Serial.printf("   ACKed by something: %-3s | TEC %lu buserr %lu | reply bytes %u\n",
                      acked ? "YES" : "no", static_cast<unsigned long>(st.tx_error_counter),
                      static_cast<unsigned long>(st.bus_error_count), static_cast<unsigned>(n));

        if (n >= 2 && resp[0] == 0x41) {
          Serial.printf("\n   *** ECU ANSWERED *** %lu bit/s, %s\n",
                        static_cast<unsigned long>(kBitrates[b]), key);
          found        = true;
          foundBitrate = kBitrates[b];
          strncpy(foundKey, key, sizeof(foundKey) - 1);
          status::show_links_up();
          break;
        }
        closeBus();
        if (g_txLatchedOff) { Serial.printf("\n!! TX latched off. Halting.\n"); return; }
      }
    }
    if (!found) {
      Serial.printf("\n>>> No answer at any bitrate or addressing.\n");
      Serial.printf("    If 'ACKed by something' was NO everywhere, nothing is receiving us:\n");
      Serial.printf("    check CAN-H/CAN-L are not swapped, and that GND is shared.\n");
      Serial.printf("    If it was YES somewhere, the bus is right but the ECU declined.\n");
      status::show_error_state52();
      delay(3000);
      return;
    }
  }

  // ---- connected: pull what we can ----
  Serial.printf("\n=========== live data  (%lu bit/s, %s) ===========\n",
                static_cast<unsigned long>(foundBitrate), foundKey);

  uint8_t resp[128];
  uint8_t pid;
  bool    acked;

  pid = 0x00;
  size_t n = ask(0x01, &pid, 1, resp, sizeof(resp), &acked);
  if (n >= 6) {
    const uint32_t bitmap = (uint32_t(resp[2]) << 24) | (uint32_t(resp[3]) << 16) |
                            (uint32_t(resp[4]) << 8) | uint32_t(resp[5]);
    Serial.printf("   supported PIDs 01-20: %08lX\n", static_cast<unsigned long>(bitmap));
    Serial.printf("   ->");
    for (uint8_t i = 0; i < 32; ++i)
      if (bitmap & (1u << (31 - i))) Serial.printf(" %02X", i + 1);
    Serial.printf("\n");

    for (uint8_t i = 0; i < 32; ++i) {
      if (!(bitmap & (1u << (31 - i)))) continue;
      const uint8_t p = static_cast<uint8_t>(i + 1);
      if (p == 0x20) continue;
      uint8_t r[32];
      const size_t rn = ask(0x01, &p, 1, r, sizeof(r), &acked);
      if (rn >= 2 && r[0] == 0x41) decodeService01(p, r, rn);
    }
  }

  // Service 09 PID 02 is the VIN, and is multi-frame - it exercises Flow Control.
  pid = 0x02;
  n   = ask(0x09, &pid, 1, resp, sizeof(resp), &acked);
  if (n > 3 && resp[0] == 0x49) {
    Serial.printf("   VIN / ident   : ");
    for (size_t i = 3; i < n; ++i)
      Serial.printf("%c", (resp[i] >= 32 && resp[i] < 127) ? resp[i] : '.');
    Serial.printf("\n");
  }

  status::tick();
  delay(2000);
}
