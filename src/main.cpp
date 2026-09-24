// main.cpp - wiring only.
//
// Constructs the links, the sinks and the status LED, then runs the poll loop.
// It contains no protocol knowledge and no formatting: adding a transport means
// adding an IBmsLink, adding a consumer means adding an IBmsSink, and neither
// touches this file beyond one array entry.
#include <Arduino.h>

#include "app_config.h"
#include "can/active_profile.h"
#include "can/can_profile_registry.h"
#include "jbd_can_link.h"
#include "jbd_uart_link.h"
#include "ring_sink.h"
#include "serial_sink.h"
#include "status_led.h"
#include "summary_consumer.h"
#include "telemetry.h"

namespace {

constexpr uint32_t kPollPeriodMs = 2000;
constexpr uint32_t kSummaryEvery = 5;  // cycles

jbd::JbdUartLink g_uart;
jbd::JbdCanLink  g_can;

app::RingSink   g_ringSink{app::telemetryProducer()};
app::SerialSink g_serialSink;

// Both links feed the same ring; BmsSnapshot::source says which produced each
// entry, so a consumer can filter or cross-check.
jbd::IBmsLink *g_links[] = {&g_uart, &g_can};
jbd::IBmsSink *g_sinks[] = {&g_ringSink, &g_serialSink};

uint32_t g_lastPoll = 0;
uint32_t g_cycle    = 0;

// Held LED claims. Each is taken when its condition appears and released when
// it clears, so the indicator always reflects the present state rather than the
// last thing that happened.
status::Token g_tokIdle       = status::kNoToken;
status::Token g_tokProtection = status::kNoToken;
status::Token g_tokCanFault   = status::kNoToken;
status::Token g_tokRingFull   = status::kNoToken;

// Take the claim on a rising edge, drop it on a falling one.
void holdWhile(bool condition, status::Token &tok, status::Token (*show)()) {
  if (condition && tok == status::kNoToken) {
    tok = show();
  } else if (!condition && tok != status::kNoToken) {
    status::release(tok);
    tok = status::kNoToken;
  }
}

// Every profile is compiled in unconditionally, so this is the whole catalogue
// the firmware could switch to at runtime - a DIP switch, a serial command, a
// UI. Printing it at boot makes that visible rather than implied.
void dumpProfiles() {
  Serial.printf("\n# CAN profiles compiled in: %u\n", static_cast<unsigned>(can::profileCount()));
  Serial.printf("#  %-3s %-15s %-9s %-10s %-19s %-18s %-11s %s\n",
                "idx", "key", "bitrate", "format", "interaction", "access", "provenance", "tx");
  for (size_t i = 0; i < can::profileCount(); ++i) {
    const can::CanBusProfile *p = can::profileAt(i);
    if (p == nullptr) continue;
    Serial.printf("#  %-3u %-15s %-9lu %-10s %-19s %-18s %-11s %u rule(s)%s\n",
                  static_cast<unsigned>(i), p->device.key,
                  static_cast<unsigned long>(p->bus.bitrate),
                  can::toString(p->bus.frameFormat), can::toString(p->protocol.interaction),
                  can::toString(p->access), can::toString(p->device.provenance),
                  p->txAllow.count, p->bus.expectsExternalTermination ? "  [needs 120R]" : "");
  }

  const char *badKey = nullptr, *reason = nullptr;
  const size_t bad = can::validateAll(&badKey, &reason);
  if (bad)
    Serial.printf("#  !! %u profile(s) INVALID, first: %s - %s\n", static_cast<unsigned>(bad),
                  badKey ? badKey : "?", reason ? reason : "?");
  else
    Serial.printf("#  all profiles validate\n");

  const can::CanBusProfile &a = can::active();
  can::BitTiming t;
  Serial.printf("# active profile : %s (%s)%s\n", a.device.key, a.device.displayName,
                can::activeWasChosen() ? "" : "  [default, nothing selected it]");
  if (can::resolveTiming(a, t))
    Serial.printf("#   timing        : brp %lu tseg1 %u tseg2 %u sjw %u -> %lu bit/s @ %u%%\n",
                  static_cast<unsigned long>(t.brp), t.tseg1, t.tseg2, t.sjw,
                  static_cast<unsigned long>(t.actualBitrate), t.actualSamplePointPercent);
  Serial.printf("#   may transmit  : %s\n", can::allowsTransmit(a) ? "yes" : "NO (passive)");
}

void reportLink(const jbd::IBmsLink *link) {
  jbd::LinkStats st;
  link->stats(st);
  Serial.printf("       link %s: %lu/%lu ok (%lu%%), crc-bad %lu", link->name(),
                static_cast<unsigned long>(st.successes), static_cast<unsigned long>(st.attempts),
                st.attempts ? static_cast<unsigned long>(st.successes * 100 / st.attempts) : 0UL,
                static_cast<unsigned long>(st.crcErrors));
  if (st.busErrors || st.txErrorCounter)
    Serial.printf(", TEC %lu, buserr %lu", static_cast<unsigned long>(st.txErrorCounter),
                  static_cast<unsigned long>(st.busErrors));
  Serial.printf("\n");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(200);

  // First thing: take control of the WS2812. Left alone it holds whatever it
  // latched at power-on, at full brightness.
  status::begin(STATUS_LED_BRIGHTNESS_PERCENT);
  status::show_boot();

  Serial.printf("\n# esp32-jbd telemetry bridge\n");
  Serial.printf("# CAN mode      : %s\n", g_can.modeName());
  Serial.printf("# LED brightness: %u %%\n", status::led().brightness());

  bool allUp = true;
  for (jbd::IBmsLink *link : g_links) {
    const bool ok = link->begin();
    allUp         = allUp && ok;
    Serial.printf("# link %-4s : %s\n", link->name(), ok ? "up" : "FAILED");
  }
  for (jbd::IBmsSink *sink : g_sinks) Serial.printf("# sink %s\n", sink->name());
  Serial.printf("# telemetry ring: %u slots, RAM only, cleared on boot\n",
                static_cast<unsigned>(app::telemetry().capacity()));
  dumpProfiles();

  if (!allUp)
    status::show_link_init_failed();
  else
    status::show_links_up();

  g_tokIdle = status::show_idle();
}

void loop() {
  // Every iteration: the LED's blink phase and claim expiry are driven here.
  status::tick();

  if (millis() - g_lastPoll < kPollPeriodMs) {
    delay(2);
    return;
  }
  g_lastPoll = millis();
  ++g_cycle;

  Serial.printf("\n======== cycle %lu  t=%lu ms ========\n",
                static_cast<unsigned long>(g_cycle), static_cast<unsigned long>(millis()));

  bool sawProtection = false;

  for (jbd::IBmsLink *link : g_links) {
    const bool isCan = (link == &g_can);
    isCan ? status::show_can_poll() : status::show_uart_poll();
    status::tick();

    jbd::BmsSnapshot s;
    if (!link->poll(s)) {
      Serial.printf("[%s] no reading this cycle\n", link->name());
      reportLink(link);
      if (!isCan) status::show_error_state50();  // UART timeout
      continue;
    }

    isCan ? status::show_can_ok() : status::show_uart_ok();
    for (jbd::IBmsSink *sink : g_sinks) sink->onSnapshot(s);
    reportLink(link);

    // Only trust the protection flags if that frame group actually arrived.
    if (jbd::hasFields(s, jbd::kFieldProtect) && s.protection != 0) sawProtection = true;
  }

  // CAN health: bus-off outranks a plain no-ACK, and both outrank routine blinks.
  holdWhile(g_can.wasBusOff(), g_tokCanFault, &status::show_error_state53);
  if (!g_can.wasBusOff())
    holdWhile(g_can.lastTxUnacked(), g_tokCanFault, &status::show_error_state52);

  holdWhile(sawProtection, g_tokProtection, &status::show_error_state58);
  holdWhile(app::telemetry().overwrittenCount() > 0, g_tokRingFull, &status::show_error_state56);

  if ((g_cycle % kSummaryEvery) == 0) app::printTelemetrySummary();
  status::tick();
}
