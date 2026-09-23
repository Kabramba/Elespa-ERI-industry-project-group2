#include "telemetry.h"

namespace app {
namespace {
// Namespace-scope object: lives in RAM (.bss/.data), reinitialised by the C
// runtime on every boot, never written to flash, NVS or EEPROM. Deliberately
// not RTC_DATA_ATTR / RTC_NOINIT_ATTR, which would survive a reset.
TelemetryRing g_ring;
}  // namespace

const TelemetryRing &telemetry() { return g_ring; }
TelemetryRing       &telemetryProducer() { return g_ring; }

}  // namespace app
