// telemetry.h - the single shared telemetry buffer.
//
// This is the seam between "code that talks to the BMS" and "code that does
// something with the numbers". The producer side is handed out separately from
// the consumer side so that a consumer literally cannot publish: it only ever
// sees a const reference.
#pragma once

#include <stddef.h>

#include "bms_ring.h"

namespace app {

constexpr size_t kHistoryDepth = 16;
using TelemetryRing = jbd::BmsRing<kHistoryDepth>;

// For anything that reads. Every method reachable through this is const.
const TelemetryRing &telemetry();

// For the poll loop, and nothing else.
TelemetryRing &telemetryProducer();

}  // namespace app
