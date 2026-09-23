#include "summary_consumer.h"

#include <Arduino.h>

#include "telemetry.h"

namespace app {

void printTelemetrySummary() {
  // Read-only handle. There is no publish() reachable from here.
  const TelemetryRing &ring = telemetry();

  Serial.printf("\n--- telemetry buffer: %u/%u held, %lu published, %lu discarded ---\n",
                static_cast<unsigned>(ring.count()), static_cast<unsigned>(ring.capacity()),
                static_cast<unsigned long>(ring.publishedCount()),
                static_cast<unsigned long>(ring.overwrittenCount()));

  // Read path 1: zero-copy. Valid until two further publishes, so take what we
  // need now and do not keep the pointer.
  if (const jbd::BmsSnapshot *now = ring.latestRef()) {
    Serial.printf("  newest (zero-copy) : %s  %.2f V  %u %%  at t=%lu ms\n",
                  jbd::sourceName(now->source), now->packVolts, now->socPercent,
                  static_cast<unsigned long>(now->uptimeMs));
  }

  // Read path 2: copy out. Always coherent, safe to hold for as long as we like.
  jbd::BmsSnapshot recent[4];
  const size_t     n = ring.history(recent, 4);
  Serial.printf("  last %u samples    :", static_cast<unsigned>(n));
  for (size_t i = 0; i < n; ++i)
    Serial.printf(" %s/%.2fV", jbd::sourceName(recent[i].source), recent[i].packVolts);
  Serial.printf("\n");
}

}  // namespace app
