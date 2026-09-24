// main_activa_obd.cpp - ask the Activa's ECU for data and put it on the wire.
//
// This application EMITS, it does not format. Every CAN frame is printed as one
// machine-readable line and tools/activa_dashboard.py does the decoding and the
// display. Two line shapes, nothing else:
//
//   # KEY value...                          metadata
//   R (12.345678) 18DAF110#03410C1A2C       a frame, 'T' transmitted, 'R' received
//
// Strip the direction character and it is candump text, so a capture pastes
// straight into SavvyCAN or python-can.
//
// SAFETY ENVELOPE - from docs/activa-can.md, enforced in code, not convention:
//   * Every transmit goes through txGuarded(). Nothing else calls twai_transmit().
//   * The id must pass can::mayTransmitId() for the active profile.
//   * The frame must pass obd::isPermittedRequestFrame(): a Single Frame with a
//     read-only SID (01 02 03 07 09 0A) or Flow Control. Clear-DTC, session
//     control, ECU reset and security access cannot be encoded.
//   * >= kMinRequestGapMs between transmits, under the 20 requests/s ceiling.
//   * Bus-off while transmitting latches TX off until reboot.
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
constexpr uint32_t kMinRequestGapMs = 60;    // ~16/s, under the 20/s ceiling
constexpr uint32_t kReplyWindowMs   = 300;   // P2 is 50 ms; margin for a slow ECU
constexpr uint32_t kPendingWindowMs = 5000;  // after NRC 0x78, per the reference
constexpr uint32_t kBitrates[]      = {500000, 250000, 1000000, 125000};
constexpr size_t   kBitrateCount    = sizeof(kBitrates) / sizeof(kBitrates[0]);

// RPM every iteration, the rest rotating: RPM lands near 8 Hz and the others
// around 2.7 Hz, which is enough to resolve a throttle blip.
constexpr uint8_t  kHotPid         = 0x0C;
constexpr uint8_t  kRotating[]     = {0x11, 0x0B, 0x0E, 0x04};
constexpr size_t   kRotatingCount  = sizeof(kRotating) / sizeof(kRotating[0]);
constexpr uint32_t kFullSweepEvery = 120;
// Metadata is re-announced on a short cycle, not just once at boot. A dashboard
// started while the board is already running opens the port mid-stream - and
// pyserial flushes the input buffer on open - so a one-shot banner is missed
// and the client waits forever for a mode line that already went past.
constexpr uint32_t kMetaEvery = 16;

bool               g_txLatchedOff = false;
uint32_t           g_lastTxMs     = 0;
can::CanBusProfile g_profile;
bool               g_haveProfile = false;

void logFrame(char dir, uint32_t id, const uint8_t *d, uint8_t dlc, bool extd) {
  const uint64_t us = static_cast<uint64_t>(micros());
  Serial.printf("%c (%lu.%06lu) ", dir, static_cast<unsigned long>(us / 1000000ULL),
                static_cast<unsigned long>(us % 1000000ULL));
  Serial.printf(extd ? "%08lX#" : "%03lX#", static_cast<unsigned long>(id));
  for (uint8_t i = 0; i < dlc && i < 8; ++i) Serial.printf("%02X", d[i]);
  Serial.printf("\n");
}

// ---------------------------------------------------------------- the gate
bool txGuarded(uint32_t id, const uint8_t *d, uint8_t dlc, bool *acked) {
  if (acked) *acked = false;
  if (g_txLatchedOff || !g_haveProfile) return false;

  if (!can::mayTransmitId(g_profile, id)) {
    Serial.printf("# REFUSED id=%lX not-on-allowlist\n", static_cast<unsigned long>(id));
    return false;
  }
  if (!obd::isPermittedRequestFrame(d, dlc)) {
    Serial.printf("# REFUSED frame not-a-permitted-read\n");
    return false;
  }

  while (millis() - g_lastTxMs < kMinRequestGapMs) delay(2);

  uint32_t alerts = 0;
  twai_read_alerts(&alerts, 0);

  twai_message_t m   = {};
  m.identifier       = id;
  m.extd             = (g_profile.bus.frameFormat == can::FrameFormat::kExtended29) ? 1 : 0;
  m.ss               = 1;  // single shot: this frame's own fate is reported
  m.data_length_code = dlc;
  memcpy(m.data, d, dlc);

  const esp_err_t e = twai_transmit(&m, pdMS_TO_TICKS(50));
  g_lastTxMs        = millis();
  if (e != ESP_OK) {
    Serial.printf("# TXERR %s\n", esp_err_to_name(e));
    return false;
  }
  logFrame('T', id, d, dlc, m.extd != 0);

  alerts = 0;
  twai_read_alerts(&alerts, pdMS_TO_TICKS(30));
  if (acked) *acked = (alerts & TWAI_ALERT_TX_SUCCESS) != 0;

  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_BUS_OFF) {
    Serial.printf("# BUSOFF latching-tx-off\n");
    g_txLatchedOff = true;
    status::show_error_state53();
  }
  return true;
}

// Collects one complete ISO-TP response. Returns payload length, 0 on timeout.
size_t receiveIsoTp(uint8_t *out, size_t outMax, uint8_t expectSid) {
  uint32_t deadline = millis() + kReplyWindowMs;
  size_t   got = 0, expected = 0;
  bool     multi = false;

  while (static_cast<int32_t>(deadline - millis()) > 0) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(10)) != ESP_OK) continue;
    if (m.rtr) continue;
    if (!can::respondsOnId(g_profile, m.identifier)) continue;
    logFrame('R', m.identifier, m.data, m.data_length_code, m.extd != 0);

    switch (obd::pciOf(m.data, m.data_length_code)) {
      case obd::Pci::kSingle: {
        const uint8_t n = obd::singleFrameLength(m.data);
        // 7F <sid> 78 is responsePending. Returning here would lose the answer
        // AND let it arrive during the next request, desynchronising everything
        // after it.
        if (n >= 3 && m.data[1] == 0x7F && m.data[3] == 0x78) {
          deadline = millis() + kPendingWindowMs;
          continue;
        }
        if (expectSid != 0 && m.data[1] != 0x7F && m.data[1] != expectSid) continue;
        for (uint8_t i = 0; i < n && got < outMax; ++i) out[got++] = m.data[1 + i];
        return got;
      }
      case obd::Pci::kFirst: {
        expected = obd::firstFrameLength(m.data);
        multi    = true;
        for (uint8_t i = 0; i < 6 && got < outMax; ++i) out[got++] = m.data[2 + i];
        uint8_t fc[8];
        obd::buildFlowControl(fc);
        // Flow Control goes to the ECU's PHYSICAL request id, not the
        // functional one the request went out on.
        const uint32_t fcId = (g_profile.bus.frameFormat == can::FrameFormat::kExtended29)
                                  ? (0x18DA00F1u | ((m.identifier & 0xFFu) << 8))
                                  : (m.identifier - 8u);
        bool ack = false;
        txGuarded(fcId, fc, 8, &ack);
        break;
      }
      case obd::Pci::kConsecutive: {
        for (uint8_t i = 0; i < 7 && got < outMax; ++i) out[got++] = m.data[1 + i];
        if (multi && got >= expected) return (expected < outMax) ? expected : got;
        break;
      }
      default:
        break;
    }
  }
  return got;
}

size_t ask(uint8_t sid, const uint8_t *args, uint8_t argCount, uint8_t *out, size_t outMax,
           bool *acked) {
  twai_message_t stale;  // drop anything left from a previous exchange
  while (twai_receive(&stale, 0) == ESP_OK) {
  }
  uint8_t frame[8];
  obd::buildSingleFrame(sid, args, argCount, frame);
  if (!txGuarded(g_profile.protocol.requestId, frame, 8, acked)) return 0;
  return receiveIsoTp(out, outMax, static_cast<uint8_t>(sid + 0x40));
}

bool openBus(const char *profileKey, uint32_t bitrate) {
  const can::CanBusProfile *base = can::findProfile(profileKey);
  if (base == nullptr) return false;
  g_profile                        = *base;
  g_profile.bus.txPin              = kTxPin;
  g_profile.bus.rxPin              = kRxPin;
  g_profile.bus.bitrate            = bitrate;
  g_profile.bus.hardwareRetransmit = false;  // per-frame ACK visibility
  const char *why                  = nullptr;
  if (!can::startFromProfile(g_profile, &why)) {
    Serial.printf("# BUSFAIL %s\n", why ? why : "unknown");
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

char     g_vin[20] = {0}, g_calId[20] = {0}, g_ecuName[24] = {0};
uint32_t g_cvn = 0;

void copyAscii(char *dst, size_t dstMax, const uint8_t *src, size_t n, size_t skip) {
  size_t o = 0;
  for (size_t i = skip; i < n && o + 1 < dstMax; ++i)
    if (src[i] >= 32 && src[i] < 127) dst[o++] = static_cast<char>(src[i]);
  dst[o] = '\0';
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(300);
  status::begin(STATUS_LED_BRIGHTNESS_PERCENT);
  status::show_boot();
  // The dashboard looks for this line and refuses to run without it.
  Serial.printf("\n# MODE activa-obd\n");
  Serial.printf("# SAFETY read-only-services-01-02-03-07-09-0A-only\n");
}

void loop() {
  static bool     found = false, inventoried = false, snapInit = false;
  static uint32_t foundBitrate = 0, cycle = 0;
  static obd::VehicleSnapshot snap;

  uint8_t resp[128];
  uint8_t pid;
  bool    acked;

  if (!snapInit) {
    obd::reset(snap);
    snapInit = true;
  }

  // ---------------- discovery ----------------
  if (!found) {
    for (size_t b = 0; b < kBitrateCount && !found; ++b) {
      for (const char *key : {"activa-obd11", "activa-obd29"}) {
        Serial.printf("# TRY %lu %s\n", static_cast<unsigned long>(kBitrates[b]), key);
        if (!openBus(key, kBitrates[b])) continue;
        pid                = 0x00;
        const size_t n     = ask(0x01, &pid, 1, resp, sizeof(resp), &acked);
        Serial.printf("# TRYRESULT acked=%d bytes=%u\n", acked ? 1 : 0,
                      static_cast<unsigned>(n));
        if (n >= 2 && resp[0] == 0x41) {
          found        = true;
          foundBitrate = kBitrates[b];
          Serial.printf("# FOUND %lu %s\n", static_cast<unsigned long>(kBitrates[b]), key);
          status::show_links_up();
          break;
        }
        closeBus();
        if (g_txLatchedOff) return;
      }
    }
    if (!found) {
      Serial.printf("# NOANSWER all-bitrates-and-addressing\n");
      status::show_error_state52();
      delay(3000);
      return;
    }
  }

  // ---------------- one-time inventory ----------------
  if (!inventoried) {
    inventoried = true;
    struct S09 { uint8_t pid; char *dst; size_t max; };
    const S09 kS09[] = {{0x02, g_vin, sizeof(g_vin)},
                        {0x04, g_calId, sizeof(g_calId)},
                        {0x0A, g_ecuName, sizeof(g_ecuName)}};
    for (const S09 &q : kS09) {
      const size_t n = ask(0x09, &q.pid, 1, resp, sizeof(resp), &acked);
      if (n > 3 && resp[0] == 0x49) copyAscii(q.dst, q.max, resp, n, 3);
    }
    pid            = 0x06;
    const size_t n = ask(0x09, &pid, 1, resp, sizeof(resp), &acked);
    if (n >= 7 && resp[0] == 0x49)
      g_cvn = (static_cast<uint32_t>(resp[3]) << 24) | (static_cast<uint32_t>(resp[4]) << 16) |
              (static_cast<uint32_t>(resp[5]) << 8) | resp[6];

    // DTC reads. The frames are logged; the dashboard decodes them.
    ask(0x03, nullptr, 0, resp, sizeof(resp), &acked);
    if (resp[0] == 0x43) snap.storedDtcCount = resp[1];
    ask(0x07, nullptr, 0, resp, sizeof(resp), &acked);
    if (resp[0] == 0x47) snap.pendingDtcCount = resp[1];
    ask(0x0A, nullptr, 0, resp, sizeof(resp), &acked);
  }

  // ---------------- sampling ----------------
  const bool fullSweep = (cycle % kFullSweepEvery) == 0;
  ++cycle;

  snap.bitrate            = foundBitrate;
  snap.extendedAddressing = (g_profile.bus.frameFormat == can::FrameFormat::kExtended29);
  strncpy(snap.vin, g_vin, sizeof(snap.vin) - 1);
  strncpy(snap.calibrationId, g_calId, sizeof(snap.calibrationId) - 1);
  strncpy(snap.ecuName, g_ecuName, sizeof(snap.ecuName) - 1);
  snap.cvn = g_cvn;

  if (fullSweep) {
    uint8_t supported[0x100] = {0};
    size_t  supportedCount   = 0;
    for (uint16_t page = 0x00; page <= 0xC0; page += 0x20) {
      const uint8_t p = static_cast<uint8_t>(page);
      const size_t  n = ask(0x01, &p, 1, resp, sizeof(resp), &acked);
      if (n < 6 || resp[0] != 0x41) break;
      const uint32_t bitmap = (static_cast<uint32_t>(resp[2]) << 24) |
                              (static_cast<uint32_t>(resp[3]) << 16) |
                              (static_cast<uint32_t>(resp[4]) << 8) | resp[5];
      for (uint8_t i = 0; i < 32; ++i)
        if ((bitmap & (1u << (31 - i))) && supportedCount < sizeof(supported))
          supported[supportedCount++] = static_cast<uint8_t>(page + i + 1);
      if ((bitmap & 1u) == 0) break;
    }
    for (size_t i = 0; i < supportedCount; ++i) {
      const uint8_t q = supported[i];
      if ((q & 0x1F) == 0) continue;  // page markers
      uint8_t      r[64];
      const size_t rn = ask(0x01, &q, 1, r, sizeof(r), &acked);
      if (rn >= 2 && r[0] == 0x41)
        obd::addSample(snap, q, (rn > 2) ? &r[2] : nullptr, static_cast<uint8_t>(rn - 2));
      status::tick();
    }
  } else {
    uint8_t r[16];
    uint8_t q  = kHotPid;
    size_t  rn = ask(0x01, &q, 1, r, sizeof(r), &acked);
    if (rn >= 2 && r[0] == 0x41)
      obd::addSample(snap, q, (rn > 2) ? &r[2] : nullptr, static_cast<uint8_t>(rn - 2));
    q  = kRotating[cycle % kRotatingCount];
    rn = ask(0x01, &q, 1, r, sizeof(r), &acked);
    if (rn >= 2 && r[0] == 0x41)
      obd::addSample(snap, q, (rn > 2) ? &r[2] : nullptr, static_cast<uint8_t>(rn - 2));
  }

  obd::decodeAll(snap);
  snap.uptimeMs = millis();
  snap.valid    = true;
  app::vehicleTelemetryProducer().publish(snap);

  if ((cycle % kMetaEvery) == 0) {
    const app::VehicleRing &ring = app::vehicleTelemetry();
    Serial.printf("# MODE activa-obd\n");
    Serial.printf("# LINK %lu %s\n", static_cast<unsigned long>(snap.bitrate),
                  snap.extendedAddressing ? "29bit" : "11bit");
    Serial.printf("# VIN %s\n# ECU %s\n# CAL %s\n# CVN %08lX\n", snap.vin, snap.ecuName,
                  snap.calibrationId, static_cast<unsigned long>(snap.cvn));
    Serial.printf("# DTC mil=%u stored=%u pending=%u\n", snap.milOn ? 1 : 0,
                  snap.storedDtcCount, snap.pendingDtcCount);
    Serial.printf("# PIDS");
    for (uint8_t i = 0; i < snap.sampleCount && i < obd::kMaxPidSamples; ++i)
      Serial.printf(" %02X", snap.samples[i].pid);
    Serial.printf("\n# RING %u/%u pub=%lu drop=%lu\n", static_cast<unsigned>(ring.count()),
                  static_cast<unsigned>(ring.capacity()),
                  static_cast<unsigned long>(ring.publishedCount()),
                  static_cast<unsigned long>(ring.overwrittenCount()));
  }
  status::tick();
}
