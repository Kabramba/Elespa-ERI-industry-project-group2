// summary_consumer.h - a worked example of reading the telemetry ring.
//
// It receives nothing and is wired to nothing: it just reaches for the shared
// buffer through the read-only accessor. This is the shape any downstream
// module takes - a display driver, an uplink, a protection watchdog.
#pragma once

namespace app {
void printTelemetrySummary();
}
