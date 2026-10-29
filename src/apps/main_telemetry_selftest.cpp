// main_telemetry_selftest.cpp - the packet pipeline, end to end, on a board
// with nothing attached to it.
//
// WHAT THIS IS FOR. Every other app needs hardware on the other end: a BMS, or
// a vehicle with its ignition on. That makes the transport hard to bring up,
// because a silent link could be the CAN wiring, the ECU, the protocol, or the
// packet layer, and there is no way to tell. This app removes all of it and
// exercises only what the schema owns: fill a payload, publish it, encode,
// frame, queue, UART. If tools/telemetry_decode.py reads these packets, the
// pipeline is proven and any later silence is upstream of it.
//
// EVERY PACKET IS FLAGGED kFlagSimulated. The values below are synthetic -
// plausible, deliberately moving so a decoder visibly comes alive, and not
// measured from anything. That flag is the difference between a test fixture
// and a fabricated reading, and it is set on every packet this app emits.
#include <Arduino.h>

#include <math.h>

#include "app_config.h"
#include "status_led.h"
#include "telemetry/mappers.h"
#include "telemetry/output.h"
#include "telemetry/packets.h"
#include "telemetry/publisher.h"

namespace {

using namespace telemetry;

// With packets on the USB port the stream is binary, so nothing human-readable
// may share it. On a separate UART the USB port is free for diagnostics.
constexpr bool kVerbose = (TELEMETRY_OUTPUT_UART1 != 0);

constexpr uint32_t kFastPeriodMs     = 125;   // 8 Hz, the real ECU sweep rate
constexpr uint32_t kPacketPeriodMs   = 1000;  // the 1 Hz group
constexpr uint32_t kIdentityPeriodMs = 15000; // static data, as a keepalive

// A gentle wander, so a decoder shows moving numbers rather than constants.
float wander(float centre, float amplitude, float periodMs, uint32_t nowMs) {
  return centre + amplitude * sinf(2.0f * 3.14159265f * static_cast<float>(nowMs) / periodMs);
}

// ---- synthetic sources ----------------------------------------------------

// Shaped like a real sweep so the same EngineAccumulator the live app uses is
// the thing under test here, not a simplified stand-in.
obd::VehicleSnapshot fakeSweep(uint32_t nowMs) {
  obd::VehicleSnapshot v = {};
  obd::reset(v);
  v.valid              = true;
  v.uptimeMs           = nowMs;
  v.bitrate            = 500000;
  v.extendedAddressing = true;

  v.rpm              = wander(1500.0f, 120.0f, 4000.0f, nowMs);
  v.mapKpa           = wander(70.0f, 20.0f, 4000.0f, nowMs);
  v.throttlePct      = wander(14.0f, 5.0f, 6000.0f, nowMs);
  v.timingAdvanceDeg = wander(9.0f, 1.5f, 5000.0f, nowMs);
  v.engineLoadPct    = wander(34.0f, 3.0f, 4500.0f, nowMs);
  v.intakeAirC       = 31.0f;
  v.oilTempC         = wander(78.0f, 4.0f, 30000.0f, nowMs);
  v.shortFuelTrimPct = 1.6f;

  const uint8_t fuelSys[2] = {0x02, 0x00};  // closed loop
  obd::addSample(v, 0x03, fuelSys, 2);
  const uint8_t standard = 0x25;            // what this ECU really answers
  obd::addSample(v, 0x1C, &standard, 1);
  return v;
}

obd::VehicleSnapshot fakeIdentity(uint32_t nowMs) {
  obd::VehicleSnapshot v = fakeSweep(nowMs);
  strncpy(v.vin, "SIMULATED-NO-VIN", sizeof(v.vin) - 1);
  strncpy(v.ecuName, "SIMULATED ECM", sizeof(v.ecuName) - 1);
  strncpy(v.calibrationId, "SIM0000000", sizeof(v.calibrationId) - 1);
  v.cvn = 0x5A5A5A5A;
  return v;
}

jbd::BmsSnapshot fakePack(uint32_t nowMs) {
  jbd::BmsSnapshot s = {};
  s.valid    = true;
  s.uptimeMs = nowMs;
  s.source   = jbd::Source::kUart;
  s.fields   = jbd::kFieldPack | jbd::kFieldCapacity | jbd::kFieldProtect |
               jbd::kFieldMos | jbd::kFieldConfig | jbd::kFieldTemps | jbd::kFieldCells;

  s.cellCount = 16;
  s.ntcCount  = 5;
  s.packVolts = wander(53.4f, 0.4f, 20000.0f, nowMs);
  s.currentAmps = wander(-2.5f, 1.8f, 7000.0f, nowMs);
  s.remainingAh = 42.0f;
  s.fullAh      = 100.0f;
  s.cycles      = 12;
  s.socPercent  = 42;

  s.chargeMosOn    = true;
  s.dischargeMosOn = true;

  for (uint8_t i = 0; i < 16; ++i) {
    s.cellsReadMask |= (1u << i);
    s.cellMilliVolts[i] =
        static_cast<uint16_t>(3335.0f + wander(0.0f, 6.0f, 9000.0f + i * 250.0f, nowMs));
  }
  for (uint8_t i = 0; i < 5; ++i) s.ntcCelsius[i] = wander(26.0f + i, 0.8f, 25000.0f, nowMs);

  jbd::computeDerived(s);
  return s;
}

// Group 1's board and the Votol are not interfaced yet, so these two are filled
// with shape-only values purely to prove the packet types encode and decode.
// Nothing here is measured, and the simulated flag says so on the wire.
DynamicsTelemetry fakeDynamics(uint32_t nowMs) {
  DynamicsTelemetry d = {};
  RunningStat        ax, ay, az;
  ax.reset(nowMs);
  ay.reset(nowMs);
  az.reset(nowMs);
  for (int i = 0; i < 40; ++i) {  // 200 Hz for a 200 ms slice
    const uint32_t t = nowMs + static_cast<uint32_t>(i * 5);
    ax.push(wander(0.02f, 0.15f, 800.0f, t), t);
    ay.push(wander(-0.01f, 0.10f, 1100.0f, t), t);
    az.push(wander(1.00f, 0.08f, 950.0f, t), t);
  }
  d.accelX = ax.snapshot();
  d.accelY = ay.snapshot();
  d.accelZ = az.snapshot();

  // The remaining six channels have no source yet. Empty is the honest value:
  // samples == 0 and NaN, so a consumer cannot read them as a level attitude.
  d.pitchDeg      = emptyStats();
  d.rollDeg       = emptyStats();
  d.tiltDeg       = emptyStats();
  d.wheelSpeedKph = emptyStats();
  d.longAccelMss  = emptyStats();
  d.jerkMsss      = emptyStats();

  d.windowMs = 200;
  d.fields   = kDynAccel;
  return d;
}

MotorTelemetry fakeMotor(uint32_t nowMs) {
  MotorTelemetry m = {};
  RunningStat    rpm, amps;
  rpm.reset(nowMs);
  amps.reset(nowMs);
  for (int i = 0; i < 8; ++i) {
    const uint32_t t = nowMs + static_cast<uint32_t>(i * 20);
    rpm.push(wander(820.0f, 60.0f, 6000.0f, t), t);
    amps.push(wander(12.0f, 4.0f, 5000.0f, t), t);
  }
  m.shaftRpm        = rpm.snapshot();
  m.currentAmps     = amps.snapshot();
  m.torqueNm        = emptyStats();  // no source; inferred downstream if at all
  m.throttlePercent = emptyStats();
  m.busVolts        = 60.4f;
  m.windowMs        = kPacketPeriodMs;
  m.state           = static_cast<uint8_t>(MotorState::kRun);
  m.linkKind        = static_cast<uint8_t>(MotorLink::kUnknown);  // not established
  m.fields          = kMotRpm | kMotCurrent | kMotBusVolts;
  return m;
}

// ---- state ----------------------------------------------------------------

EngineAccumulator g_engine;
StatusTracker     g_status;

uint32_t g_lastFast     = 0;
uint32_t g_lastPacket   = 0;
uint32_t g_lastIdentity = 0;
uint32_t g_published    = 0;

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(300);

  status::begin(STATUS_LED_BRIGHTNESS_PERCENT);
  status::show_boot();

  app::beginTelemetryOutput();
  g_engine.reset(millis());

  if (kVerbose) {
    Serial.printf("\n# MODE telemetry-selftest\n");
    Serial.printf("# SCHEMA v%u  seven packet types\n", telemetry::kSchemaVersion);
    Serial.printf("# OUTPUT uart1 tx=%d %d baud\n", TELEMETRY_UART_TX_PIN, TELEMETRY_UART_BAUD);
    Serial.printf("# WARNING every packet is SIMULATED, nothing here is measured\n");
  }
}

void loop() {
  const uint32_t now = millis();

  // The fast path: fold a sweep into the statistics, as the live app does.
  if (now - g_lastFast >= kFastPeriodMs) {
    g_lastFast = now;
    g_engine.noteRequest();
    g_engine.noteReply();
    g_engine.push(fakeSweep(now), now);
  }

  if (now - g_lastPacket >= kPacketPeriodMs) {
    g_lastPacket = now;
    telemetry::Publisher &pub = app::telemetryPublisher();

    // Simulated, always. These numbers came from a sine wave, not a vehicle.
    constexpr uint8_t kFlags = telemetry::kFlagSimulated;

    EngineTelemetry e = {};
    if (g_engine.flush(e, now)) {
      pub.publish(PacketType::kEngine, SourceId::kEcu, e, kFlags);
      ++g_published;
    }

    const jbd::BmsSnapshot pack = fakePack(now);
    BatteryTelemetry       b    = {};
    toBatteryTelemetry(pack, b);
    pub.publish(PacketType::kBattery, SourceId::kBatteryA, b, kFlags);
    ++g_published;

    const DynamicsTelemetry d = fakeDynamics(now);
    pub.publish(PacketType::kDynamics, SourceId::kSensorGroup1, d, kFlags);
    ++g_published;

    const MotorTelemetry m = fakeMotor(now);
    pub.publish(PacketType::kMotor, SourceId::kMotorDriver, m, kFlags);
    ++g_published;

    // Navigation has no source at all yet; send the empty shape once a second
    // so a consumer sees the type exists and that every field is absent.
    NavigationTelemetry nav = {};
    nav.gpsSpeedKph         = emptyStats();
    nav.ecuSpeedKph         = emptyStats();
    pub.publish(PacketType::kNavigation, SourceId::kSensorGroup1, nav, kFlags);
    ++g_published;

    // On change only, which is the point of the type.
    StatusEventTelemetry st = {};
    if (g_status.update(pack, st)) {
      pub.publish(PacketType::kStatusEvent, SourceId::kBatteryA, st, kFlags);
      ++g_published;
    }

    status::show_ring_published();  // a brief flash per publish cycle
  }

  if (now - g_lastIdentity >= kIdentityPeriodMs) {
    g_lastIdentity = now;
    IdentityTelemetry id = {};
    toIdentityTelemetry(fakeIdentity(now), id);
    id.batteryVendor = static_cast<uint8_t>(BatteryVendor::kJbd);
    id.fields |= kIdBattery;
    app::telemetryPublisher().publish(PacketType::kIdentity, SourceId::kFusion, id,
                                      telemetry::kFlagSimulated);
    ++g_published;

    if (kVerbose) {
      Serial.printf("# STATS published=%lu frames=%lu queued=%u dropped=%lu failed=%lu\n",
                    static_cast<unsigned long>(g_published),
                    static_cast<unsigned long>(app::telemetryFramesWritten()),
                    static_cast<unsigned>(app::telemetryQueue().count()),
                    static_cast<unsigned long>(app::telemetryQueue().droppedCount()),
                    static_cast<unsigned long>(app::telemetryWriteFailures()));
    }
  }

  // Drain the queue onto the wire.
  app::pumpTelemetryOutput(3);
  delay(5);
}
