// output.h - the one place an application gets a Publisher from.
//
// Every app needs the same chain - publisher, queue, COBS, UART - and wiring it
// by hand in each main() is how three apps end up with three subtly different
// transports. app::telemetryOutput() builds it once, according to the toggles
// in app_config.h, and hands back the only two things an application needs:
// somewhere to publish, and a pump to call from the loop.
//
// WHY A QUEUE IS IN THE PATH AT ALL on a single-threaded Arduino loop: a
// publish must not block the acquisition loop. Writing 324 bytes at 115200
// baud takes about 28 ms, and an OBD cycle that stalls that long misses its
// reply window and starts losing PIDs. Publishing enqueues; pump() drains at
// whatever rate the UART can take.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "app_config.h"
#include "telemetry/packet_queue.h"
#include "telemetry/publisher.h"

namespace app {

constexpr size_t kTelemetryQueueDepth = TELEMETRY_QUEUE_DEPTH;
using TelemetryQueue                  = telemetry::PacketQueue<kTelemetryQueueDepth>;

// Bring the output chain up. Call once from setup(), before publishing.
void beginTelemetryOutput();

// Somewhere to publish. Valid only after beginTelemetryOutput().
telemetry::Publisher &telemetryPublisher();

// Move queued packets onto the wire. Call from loop() as often as convenient;
// it writes at most `maxPackets` per call so it cannot monopolise the loop.
// Returns how many it actually wrote.
size_t pumpTelemetryOutput(size_t maxPackets = 2);

// The queue, for diagnostics - depth, drops, totals.
const TelemetryQueue &telemetryQueue();

// How many packets have been handed to the transport, and how many the
// transport refused. Worth printing periodically: a rising refusal count is
// the signal that the link cannot keep up with the publish rate.
uint32_t telemetryFramesWritten();
uint32_t telemetryWriteFailures();

}  // namespace app
