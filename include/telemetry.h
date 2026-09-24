// telemetry.h - the single shared telemetry buffer.
//
// This is the seam between "code that talks to the BMS" and "code that does
// something with the numbers". The producer side is handed out separately from
// the consumer side so that a consumer literally cannot publish: it only ever
// sees a const reference.
#pragma once

#include <stddef.h>

#include "bms_types.h"
#include "obd/vehicle_types.h"
#include "snapshot_ring.h"

namespace app {

// Battery telemetry. Separate buffer from the vehicle's - they have different
// shapes, different rates, and a stall in one must not displace the other.
constexpr size_t kHistoryDepth = 16;
using TelemetryRing = util::SnapshotRing<jbd::BmsSnapshot, kHistoryDepth>;

const TelemetryRing &telemetry();          // consumers: every method is const
TelemetryRing       &telemetryProducer();  // the poll loop, and nothing else

// Vehicle (OBD-II) telemetry. Its own buffer, shallower because each snapshot
// is a whole PID sweep rather than a single reading.
constexpr size_t kVehicleHistoryDepth = 8;
using VehicleRing = util::SnapshotRing<obd::VehicleSnapshot, kVehicleHistoryDepth>;

const VehicleRing &vehicleTelemetry();
VehicleRing       &vehicleTelemetryProducer();

}  // namespace app
