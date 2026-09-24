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
#include "obd/vehicle_types.h"
#include "status_led.h"
#include "telemetry.h"

namespace {

constexpr int      kTxPin           = 5;
constexpr int      kRxPin           = 4;
constexpr uint32_t kMinRequestGapMs = 60;   // ~16/s, under the 20/s ceiling
constexpr uint32_t kReplyWindowMs   = 300;   // P2 is 50 ms; margin for a slow ECU
constexpr uint32_t kPendingWindowMs = 5000;  // after NRC 0x78, per the reference
constexpr uint32_t kBitrates[]      = {500000, 250000, 1000000, 125000};
// Raw frame logging is for bring-up. Off by default so the serial monitor
// shows readings rather than hex.
constexpr bool     kVerboseFrames   = false;
constexpr size_t   kBitrateCount    = sizeof(kBitrates) / sizeof(kBitrates[0]);

bool                 g_txLatchedOff = false;
uint32_t             g_lastTxMs     = 0;
can::CanBusProfile   g_profile;      // the profile currently driving the bus
bool                 g_haveProfile = false;

// Service 09 identity is static for the life of the vehicle, so it is read once
// and copied into every snapshot rather than re-requested each sweep.
char     g_vin[20]     = {0};
char     g_calId[20]   = {0};
char     g_ecuName[24] = {0};
uint32_t g_cvn         = 0;

void copyAscii(char *dst, size_t dstMax, const uint8_t *src, size_t n, size_t skip) {
  size_t o = 0;
  for (size_t i = skip; i < n && o + 1 < dstMax; ++i)
    if (src[i] >= 32 && src[i] < 127) dst[o++] = static_cast<char>(src[i]);
  dst[o] = '\0';
}

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
  if (!kVerboseFrames) return;
  Serial.printf("   %s %0*lX#", tag, (g_profile.bus.frameFormat == can::FrameFormat::kExtended29) ? 8 : 3,
                static_cast<unsigned long>(id));
  for (uint8_t i = 0; i < dlc; ++i) Serial.printf("%02X", d[i]);
  Serial.printf("\n");
}

// Collects a complete ISO-TP response. Returns payload length, 0 on timeout.
size_t receiveIsoTp(uint8_t *out, size_t outMax, uint32_t *fromId, uint8_t expectSid) {
  uint32_t deadline = millis() + kReplyWindowMs;
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

        // 7F <sid> 78 is responsePending: the ECU has the request and needs
        // more time. Returning here would be wrong twice over - we would miss
        // the answer, AND it would arrive during the NEXT request and be
        // mistaken for its reply, desynchronising everything after it.
        if (n >= 3 && m.data[1] == 0x7F && m.data[3] == 0x78) {
          Serial.printf("      (responsePending, waiting up to %lu ms)\n",
                        static_cast<unsigned long>(kPendingWindowMs));
          deadline = millis() + kPendingWindowMs;
          continue;
        }

        // Ignore a positive reply to some earlier request that arrived late.
        if (expectSid != 0 && m.data[1] != 0x7F && m.data[1] != expectSid) {
          Serial.printf("      (stale reply for SID %02X, ignoring)\n", m.data[1] - 0x40);
          continue;
        }

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
  // Drop anything still queued from a previous exchange, so a late reply
  // cannot be read as the answer to this request.
  twai_message_t stale;
  while (twai_receive(&stale, 0) == ESP_OK) {
  }

  uint8_t frame[8];
  obd::buildSingleFrame(sid, args, argCount, frame);
  hexDump("TX", g_profile.protocol.requestId, frame, 8);
  if (!txGuarded(g_profile.protocol.requestId, frame, 8, acked)) return 0;
  uint32_t from = 0;
  return receiveIsoTp(out, outMax, &from, static_cast<uint8_t>(sid + 0x40));
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
// Formulas from docs/activa-can.md s6.6. A,B,C,D are the bytes after the PID.
void decodeService01(uint8_t pid, const uint8_t *d, size_t n) {
  const uint8_t A = (n > 2) ? d[2] : 0;
  const uint8_t B = (n > 3) ? d[3] : 0;
  const uint8_t C = (n > 4) ? d[4] : 0;
  const uint8_t D = (n > 5) ? d[5] : 0;
  switch (pid) {
    case 0x01:
      Serial.printf("      MIL %s, %u stored DTC(s)\n", (A & 0x80) ? "ON" : "off", A & 0x7F);
      break;
    case 0x03: Serial.printf("      fuel system    : 0x%02X 0x%02X\n", A, B); break;
    case 0x04: Serial.printf("      engine load    : %.1f %%\n", A * 100.0f / 255.0f); break;
    case 0x05: Serial.printf("      engine temp    : %d C\n", A - 40); break;
    case 0x06: Serial.printf("      short fuel trim: %+.1f %%\n", A * 100.0f / 128.0f - 100.0f); break;
    case 0x07: Serial.printf("      long fuel trim : %+.1f %%\n", A * 100.0f / 128.0f - 100.0f); break;
    case 0x0B: Serial.printf("      MAP            : %u kPa\n", A); break;
    case 0x0C: Serial.printf("      RPM            : %.1f\n", (256.0f * A + B) / 4.0f); break;
    case 0x0D: Serial.printf("      speed          : %u km/h\n", A); break;
    case 0x0E: Serial.printf("      timing advance : %+.1f deg BTDC\n", A / 2.0f - 64.0f); break;
    case 0x0F: Serial.printf("      intake air     : %d C\n", A - 40); break;
    case 0x10: Serial.printf("      MAF            : %.2f g/s\n", (256.0f * A + B) / 100.0f); break;
    case 0x11: Serial.printf("      throttle       : %.1f %%\n", A * 100.0f / 255.0f); break;
    case 0x1C: Serial.printf("      OBD standard   : 0x%02X\n", A); break;
    case 0x1F: Serial.printf("      run time       : %u s\n", 256u * A + B); break;
    case 0x21: Serial.printf("      dist. w/ MIL   : %u km\n", 256u * A + B); break;
    case 0x2F: Serial.printf("      fuel level     : %.1f %%\n", A * 100.0f / 255.0f); break;
    case 0x31: Serial.printf("      dist. cleared  : %u km\n", 256u * A + B); break;
    case 0x33: Serial.printf("      barometric     : %u kPa\n", A); break;
    case 0x42: Serial.printf("      module voltage : %.3f V\n", (256.0f * A + B) / 1000.0f); break;
    case 0x45: Serial.printf("      rel. throttle  : %.1f %%\n", A * 100.0f / 255.0f); break;
    case 0x46: Serial.printf("      ambient temp   : %d C\n", A - 40); break;
    case 0x5C: Serial.printf("      oil temp       : %d C\n", A - 40); break;
    case 0xA6:
      Serial.printf("      odometer       : %.1f km\n",
                    ((static_cast<uint32_t>(A) << 24) | (static_cast<uint32_t>(B) << 16) |
                     (static_cast<uint32_t>(C) << 8) | D) / 10.0f);
      break;
    default:
      // Unknown scaling. The raw bytes are already in the RX log above; the
      // reference is explicit that guessing a formula is worse than not.
      Serial.printf("      (raw, no known scaling)\n");
      break;
  }
}

// docs/activa-can.md s6.7: bits 15-14 pick the system letter, the remaining
// 14 bits are the four digits.
//
// The positive SID must be verified before anything is decoded. Without that
// check a negative response (7F <SID> <NRC>) or an empty reply gets parsed as
// if it were DTC data, which is exactly how this printed ten imaginary codes
// while PID 01 was reporting none.
void printDtcCodes(const char *label, uint8_t expectSid, const uint8_t *d, size_t n) {
  Serial.printf("   %-14s: ", label);
  if (n == 0) { Serial.printf("no response\n"); return; }

  Serial.printf("[raw");
  for (size_t i = 0; i < n && i < 16; ++i) Serial.printf(" %02X", d[i]);
  Serial.printf("] ");

  if (d[0] == 0x7F) {
    Serial.printf("negative response, NRC 0x%02X\n", (n > 2) ? d[2] : 0);
    return;
  }
  if (d[0] != expectSid) {
    Serial.printf("unexpected SID 0x%02X (wanted 0x%02X)\n", d[0], expectSid);
    return;
  }
  if (n < 2) { Serial.printf("empty\n"); return; }

  const uint8_t count = d[1];
  if (count == 0) { Serial.printf("none\n"); return; }
  Serial.printf("%u code(s)  ", count);
  size_t shown = 0;
  for (size_t i = 2; i + 1 < n && shown < count; i += 2) {
    const uint16_t raw = static_cast<uint16_t>((d[i] << 8) | d[i + 1]);
    if (raw == 0) continue;
    const char sys = "PCBU"[(raw >> 14) & 0x03];
    Serial.printf("%c%01X%01X%01X%01X ", sys, (raw >> 12) & 0x03, (raw >> 8) & 0x0F,
                  (raw >> 4) & 0x0F, raw & 0x0F);
    ++shown;
  }
  Serial.printf("\n");
}

void printAscii(const char *label, const uint8_t *d, size_t n, size_t skip) {
  if (n <= skip) { Serial.printf("   %-14s: no response\n", label); return; }
  Serial.printf("   %-14s: ", label);
  for (size_t i = skip; i < n; ++i)
    Serial.printf("%c", (d[i] >= 32 && d[i] < 127) ? d[i] : '.');
  Serial.printf("\n");
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
  static bool     found        = false;
  static char     foundKey[16] = {0};
  static uint32_t foundBitrate = 0;
  static bool     inventoried  = false;

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
      status::show_error_state52();
      delay(3000);
      return;
    }
  }

  uint8_t resp[128];
  uint8_t pid;
  bool    acked;

  // ---------------- one-time inventory: everything static about this ECU ----
  if (!inventoried) {
    inventoried = true;
    Serial.printf("\n########## ECU INVENTORY (%lu bit/s, %s) ##########\n",
                  static_cast<unsigned long>(foundBitrate), foundKey);

    // Service 09: vehicle information.
    Serial.printf("\n--- Service 09: vehicle information ---\n");
    struct S09 { uint8_t pid; const char *label; size_t skip; bool ascii; };
    const S09 kS09[] = {
        {0x02, "VIN", 3, true}, {0x04, "Calibration ID", 3, true},
        {0x06, "CVN", 3, false}, {0x0A, "ECU name", 3, true},
    };
    for (const S09 &q : kS09) {
      const size_t n = ask(0x09, &q.pid, 1, resp, sizeof(resp), &acked);
      Serial.printf("   %-14s: ", q.label);
      if (n == 0) { Serial.printf("no response\n"); continue; }
      Serial.printf("[raw");
      for (size_t i = 0; i < n && i < 24; ++i) Serial.printf(" %02X", resp[i]);
      Serial.printf("] ");
      if (resp[0] == 0x7F) {
        Serial.printf("negative response, NRC 0x%02X\n", (n > 2) ? resp[2] : 0);
        continue;
      }
      if (resp[0] != 0x49) { Serial.printf("unexpected SID 0x%02X\n", resp[0]); continue; }
      if (q.ascii) {
        for (size_t i = q.skip; i < n; ++i)
          Serial.printf("%c", (resp[i] >= 32 && resp[i] < 127) ? resp[i] : '.');
        switch (q.pid) {
          case 0x02: copyAscii(g_vin, sizeof(g_vin), resp, n, q.skip); break;
          case 0x04: copyAscii(g_calId, sizeof(g_calId), resp, n, q.skip); break;
          case 0x0A: copyAscii(g_ecuName, sizeof(g_ecuName), resp, n, q.skip); break;
          default: break;
        }
      } else {
        for (size_t i = q.skip; i < n; ++i) Serial.printf("%02X", resp[i]);
        if (q.pid == 0x06 && n >= q.skip + 4)
          g_cvn = (static_cast<uint32_t>(resp[q.skip]) << 24) |
                  (static_cast<uint32_t>(resp[q.skip + 1]) << 16) |
                  (static_cast<uint32_t>(resp[q.skip + 2]) << 8) | resp[q.skip + 3];
      }
      Serial.printf("\n");
    }

    // Diagnostic trouble codes: stored, pending, permanent.
    Serial.printf("\n--- DTCs ---\n");
    size_t n = ask(0x03, nullptr, 0, resp, sizeof(resp), &acked);
    printDtcCodes("stored (03)", 0x43, resp, n);
    n = ask(0x07, nullptr, 0, resp, sizeof(resp), &acked);
    printDtcCodes("pending (07)", 0x47, resp, n);
    n = ask(0x0A, nullptr, 0, resp, sizeof(resp), &acked);
    printDtcCodes("permanent (0A)", 0x4A, resp, n);
  }

  // ---------------- every supported Service 01 PID, every page -------------
  static obd::VehicleSnapshot snap;
  obd::reset(snap);
  snap.bitrate            = foundBitrate;
  snap.extendedAddressing = (g_profile.bus.frameFormat == can::FrameFormat::kExtended29);
  strncpy(snap.vin, g_vin, sizeof(snap.vin) - 1);
  strncpy(snap.calibrationId, g_calId, sizeof(snap.calibrationId) - 1);
  strncpy(snap.ecuName, g_ecuName, sizeof(snap.ecuName) - 1);
  snap.cvn = g_cvn;

  // PIDs 0x00, 0x20, 0x40 ... each return a bitmap for the next 32. Bit 0 of
  // byte D says whether the following page exists, so walk while it is set.
  uint8_t supported[0x100] = {0};
  size_t  supportedCount   = 0;
  for (uint16_t page = 0x00; page <= 0xC0; page += 0x20) {
    const uint8_t p = static_cast<uint8_t>(page);
    const size_t  n = ask(0x01, &p, 1, resp, sizeof(resp), &acked);
    if (n < 6 || resp[0] != 0x41) break;
    const uint32_t bitmap = (static_cast<uint32_t>(resp[2]) << 24) |
                            (static_cast<uint32_t>(resp[3]) << 16) |
                            (static_cast<uint32_t>(resp[4]) << 8) | resp[5];
    Serial.printf("   supported %02X-%02X : %08lX  ->", page + 1, page + 0x20,
                  static_cast<unsigned long>(bitmap));
    for (uint8_t i = 0; i < 32; ++i) {
      if (!(bitmap & (1u << (31 - i)))) continue;
      const uint8_t q = static_cast<uint8_t>(page + i + 1);
      Serial.printf(" %02X", q);
      if (supportedCount < sizeof(supported)) supported[supportedCount++] = q;
    }
    Serial.printf("\n");
    if ((bitmap & 1u) == 0) break;  // no further page
  }
  Serial.printf("   %u PID(s) supported in total\n", static_cast<unsigned>(supportedCount));

  for (size_t i = 0; i < supportedCount; ++i) {
    const uint8_t q = supported[i];
    if ((q & 0x1F) == 0) continue;  // the page markers themselves
    uint8_t      r[64];
    const size_t rn = ask(0x01, &q, 1, r, sizeof(r), &acked);
    if (rn >= 2 && r[0] == 0x41) {
      obd::addSample(snap, q, (rn > 2) ? &r[2] : nullptr, static_cast<uint8_t>(rn - 2));
      if (kVerboseFrames) {
        Serial.printf("   PID %02X ->", q);
        for (size_t k = 2; k < rn; ++k) Serial.printf(" %02X", r[k]);
        Serial.printf("\n");
        decodeService01(q, r, rn);
      }
    }
    status::tick();
  }

  obd::decodeAll(snap);
  snap.uptimeMs = millis();
  snap.valid    = true;
  app::vehicleTelemetryProducer().publish(snap);

  // Read it back through the const handle - the same path any other consumer
  // would use. This is a reader, not the producer's own copy.
  const app::VehicleRing   &ring = app::vehicleTelemetry();
  const obd::VehicleSnapshot *v  = ring.latestRef();
  if (v == nullptr) { delay(3000); return; }

  Serial.printf("\n======== VEHICLE  t=%lu ms ========\n", static_cast<unsigned long>(v->uptimeMs));
  Serial.printf("  VIN %s   ECU %s   cal %s   CVN %08lX\n", v->vin, v->ecuName, v->calibrationId,
                static_cast<unsigned long>(v->cvn));
  Serial.printf("  link %lu bit/s %s   %u PIDs sampled\n", static_cast<unsigned long>(v->bitrate),
                v->extendedAddressing ? "29-bit" : "11-bit", v->sampleCount);
  Serial.printf("  MIL %s   stored DTCs %u\n", v->milOn ? "ON" : "off", v->storedDtcCount);

  // A field is printed only if its PID actually answered; a missing PID is not
  // the same as a reading of zero.
  struct Row { uint8_t pid; const char *label; const float *value; const char *unit; };
  const Row rows[] = {
      {0x0C, "RPM", &v->rpm, ""},          {0x0D, "speed", &v->speedKph, "km/h"},
      {0x04, "engine load", &v->engineLoadPct, "%"},
      {0x11, "throttle", &v->throttlePct, "%"},
      {0x0B, "MAP", &v->mapKpa, "kPa"},    {0x0F, "intake air", &v->intakeAirC, "C"},
      {0x5C, "oil temp", &v->oilTempC, "C"},
      {0x0E, "timing adv", &v->timingAdvanceDeg, "deg"},
      {0x06, "short trim", &v->shortFuelTrimPct, "%"},
  };
  for (const Row &r : rows)
    if (obd::hasPid(*v, r.pid)) Serial.printf("  %-12s %8.1f %s\n", r.label, *r.value, r.unit);
  if (v->distanceWithMilKm != 0xFFFF)
    Serial.printf("  %-12s %8u km\n", "dist w/ MIL", v->distanceWithMilKm);

  Serial.printf("  buffer: %u/%u held, %lu published, %lu discarded\n",
                static_cast<unsigned>(ring.count()), static_cast<unsigned>(ring.capacity()),
                static_cast<unsigned long>(ring.publishedCount()),
                static_cast<unsigned long>(ring.overwrittenCount()));

  status::tick();
  delay(3000);
}
