// main_activa_probe.cpp - find the Activa's CAN bitrate, safely.
//
// THIS APPLICATION NEVER TRANSMITS. Every bus it opens is TWAI_MODE_LISTEN_ONLY,
// in which the controller does not drive the bus at all - not even an ACK bit.
// It is the only safe way onto a bus belonging to a vehicle you ride.
//
// Method (docs/activa-can.md s5): for each candidate bitrate, listen for a few
// seconds and record valid frames, receive errors and bus errors.
//   correct rate  -> valid frames, ~0 bus errors
//   wrong rate    -> rising bus errors and REC, no valid frames
//   nothing, anywhere, with no errors -> the bus is silent and only answers
//                                        diagnostic requests
//
// Because we never ACK, the ECU may retransmit the same frame repeatedly. That
// is expected and still proves the bitrate.
//
// Changing bitrate on ESP32 TWAI requires stop -> uninstall -> reinstall, which
// is what startFromProfile()/stopBus() do.
#include <Arduino.h>

#include "app_config.h"
#include "can/active_profile.h"
#include "can/can_profile_registry.h"
#include "can/twai_profile.h"
#include "status_led.h"

namespace {

// Probe order from the reference: ISO 15765-4 default first.
const char *const kCandidates[] = {"sniff-500k", "sniff-250k", "sniff-1m", "sniff-125k"};
constexpr size_t  kCandidateCount = sizeof(kCandidates) / sizeof(kCandidates[0]);

constexpr int      kTxPin      = 5;   // module D  (unused: we never transmit)
constexpr int      kRxPin      = 4;   // module R
constexpr uint32_t kListenMs   = 4000;
constexpr size_t   kLogPerRate = 12;  // cap the dump so one chatty rate cannot flood

struct Result {
  const char *key;
  uint32_t    bitrate;
  uint32_t    frames;
  uint32_t    busErrors;
  uint32_t    rec;
  bool        started;
};

// candump text format, so captures paste straight into SavvyCAN or python-can:
//   (1234.567890) can0 7E8#064100BE3FA81300
void logFrame(const twai_message_t &m) {
  const uint64_t us = static_cast<uint64_t>(micros());
  Serial.printf("(%lu.%06lu) can0 ", static_cast<unsigned long>(us / 1000000ULL),
                static_cast<unsigned long>(us % 1000000ULL));
  if (m.extd)
    Serial.printf("%08lX#", static_cast<unsigned long>(m.identifier));
  else
    Serial.printf("%03lX#", static_cast<unsigned long>(m.identifier));
  if (m.rtr)
    Serial.printf("R");
  else
    for (uint8_t i = 0; i < m.data_length_code && i < 8; ++i) Serial.printf("%02X", m.data[i]);
  Serial.printf("\n");
}

Result listenAt(const char *key) {
  Result r{key, 0, 0, 0, 0, false};

  const can::CanBusProfile *base = can::findProfile(key);
  if (base == nullptr) {
    Serial.printf("!! no profile '%s'\n", key);
    return r;
  }

  // The sniffer profiles ship without pins so they are not tied to one board.
  can::CanBusProfile p = *base;
  p.bus.txPin          = kTxPin;
  p.bus.rxPin          = kRxPin;
  r.bitrate            = p.bus.bitrate;

  // Refuse to continue if anything could transmit. This should be impossible -
  // the profile is listen-only twice over - but the check costs nothing and the
  // consequence of being wrong is frames on somebody's engine ECU bus.
  if (can::allowsTransmit(p) || p.bus.mode != can::BusMode::kListenOnly) {
    Serial.printf("!! REFUSING '%s': not listen-only\n", key);
    return r;
  }

  const char *why = nullptr;
  if (!can::startFromProfile(p, &why)) {
    Serial.printf("!! %s: bus would not start (%s)\n", key, why ? why : "?");
    return r;
  }
  r.started = true;

  Serial.printf("\n--- %-11s %7lu bit/s, listening %lu ms (listen-only) ---\n", key,
                static_cast<unsigned long>(p.bus.bitrate),
                static_cast<unsigned long>(kListenMs));

  size_t         logged   = 0;
  const uint32_t deadline = millis() + kListenMs;
  while (static_cast<int32_t>(deadline - millis()) > 0) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(20)) != ESP_OK) continue;
    ++r.frames;
    if (logged < kLogPerRate) {
      logFrame(m);
      ++logged;
    } else if (logged == kLogPerRate) {
      Serial.printf("   ... further frames not logged at this rate\n");
      ++logged;
    }
    status::show_can_ok();
    status::tick();
  }

  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK) {
    r.busErrors = st.bus_error_count;
    r.rec       = st.rx_error_counter;
  }
  can::stopBus();

  Serial.printf("    frames %lu   bus errors %lu   REC %lu\n",
                static_cast<unsigned long>(r.frames), static_cast<unsigned long>(r.busErrors),
                static_cast<unsigned long>(r.rec));
  return r;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(300);

  status::begin(STATUS_LED_BRIGHTNESS_PERCENT);
  status::show_boot();

  Serial.printf("\n");
  Serial.printf("###########################################################\n");
  Serial.printf("# Activa CAN bitrate probe - LISTEN ONLY                  #\n");
  Serial.printf("# This firmware never transmits and never ACKs.           #\n");
  Serial.printf("###########################################################\n");
  Serial.printf("# TX=GPIO%d (idle) RX=GPIO%d   %u candidate rates\n", kTxPin, kRxPin,
                static_cast<unsigned>(kCandidateCount));
  Serial.printf("# Wiring: DLC Yellow/Black -> CANH, Yellow/Red -> CANL,\n");
  Serial.printf("#         DLC Green -> transceiver GND AND ESP32 GND.\n");
  Serial.printf("# Laptop must be on BATTERY, charger unplugged.\n");
  Serial.printf("# Turn the ignition ON now; the scan repeats continuously.\n");
}

void loop() {
  static uint32_t pass = 0;
  Serial.printf("\n================ pass %lu ================\n",
                static_cast<unsigned long>(++pass));

  Result results[kCandidateCount];
  for (size_t i = 0; i < kCandidateCount; ++i) {
    results[i] = listenAt(kCandidates[i]);
    status::tick();
  }

  Serial.printf("\n---------------- summary ----------------\n");
  Serial.printf("%-11s %9s %8s %10s %6s\n", "rate", "bit/s", "frames", "bus errors", "REC");
  int  best      = -1;
  bool anyFrames = false, anyErrors = false;
  for (size_t i = 0; i < kCandidateCount; ++i) {
    const Result &r = results[i];
    Serial.printf("%-11s %9lu %8lu %10lu %6lu%s\n", r.key, static_cast<unsigned long>(r.bitrate),
                  static_cast<unsigned long>(r.frames), static_cast<unsigned long>(r.busErrors),
                  static_cast<unsigned long>(r.rec), r.started ? "" : "   (did not start)");
    if (r.frames) anyFrames = true;
    if (r.busErrors) anyErrors = true;
    if (r.frames && (best < 0 || r.frames > results[best].frames)) best = static_cast<int>(i);
  }

  Serial.printf("\n");
  if (best >= 0) {
    Serial.printf(">>> LIKELY BITRATE: %lu bit/s  (%s) - %lu frames, %lu bus errors\n",
                  static_cast<unsigned long>(results[best].bitrate), results[best].key,
                  static_cast<unsigned long>(results[best].frames),
                  static_cast<unsigned long>(results[best].busErrors));
    Serial.printf("    Record it in docs/activa-can.md and set the activa-* profiles to it.\n");
    status::show_links_up();
  } else if (anyErrors) {
    Serial.printf(">>> Bus errors at every rate but no valid frames.\n");
    Serial.printf("    Most likely CAN-H/CAN-L are swapped - swap the two yellow wires.\n");
    Serial.printf("    Also confirm the transceiver's Rs pin is tied to GND.\n");
    status::show_error_state52();
  } else if (!anyFrames) {
    Serial.printf(">>> Silence at every rate, and no bus errors either.\n");
    Serial.printf("    Either the ignition is OFF, the ground is not shared, or this DLC\n");
    Serial.printf("    is diagnostic-only and stays quiet until a request is sent - which\n");
    Serial.printf("    is common. Next step is an ISO-TP request, not more listening.\n");
    status::show_link_degraded();
  }
  status::tick();
  delay(2000);
}
