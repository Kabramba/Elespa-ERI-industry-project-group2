// make_golden_capture.cpp - writes a framed packet stream from the real C++
// structs, so the Python decoder can be checked against them.
//
// tools/telemetry_decode.py restates the schema a second time, in struct format
// strings. Two independent descriptions of the same bytes will eventually
// disagree, and the failure mode is silent: every field after the mistake is
// read from the wrong offset and still looks like a plausible number. This
// generator closes that loop. It emits known values; the decoder must report
// those exact values back.
//
// Build and run (from the repo root):
//     c++ -std=c++17 -I include -o /tmp/golden tools/make_golden_capture.cpp \
//         src/crc16.cpp src/telemetry/packet.cpp src/telemetry/framing.cpp \
//         src/telemetry/publisher.cpp src/telemetry/stat_block.cpp
//     /tmp/golden > test/fixtures/golden_capture.bin
//
// tools/check_decoder.py does both steps and verifies the result.
#include <stdio.h>
#include <string.h>

#include "telemetry/framing.h"
#include "telemetry/packets.h"
#include "telemetry/publisher.h"

using namespace telemetry;

namespace {

// Writes the framed bytes to stdout.
class StdoutWriter final : public IByteWriter {
 public:
  size_t write(const uint8_t *data, size_t length) override {
    return fwrite(data, 1, length, stdout);
  }
};

uint32_t g_clock = 1000;
uint32_t clockFn() { return g_clock; }

// A StatBlock with values a decoder can be checked against exactly.
StatBlock stat(float lo, float hi, float mean, float sd, float last, float slope,
               uint32_t window, uint16_t samples) {
  StatBlock s   = {};
  s.min         = lo;
  s.max         = hi;
  s.mean        = mean;
  s.stddev      = sd;
  s.last        = last;
  s.slopePerSec = slope;
  s.windowMs    = window;
  s.samples     = samples;
  return s;
}

}  // namespace

int main() {
  StdoutWriter  out;
  CobsFrameSink sink(out);
  Publisher     pub(sink, clockFn);

  // ---- identity ----------------------------------------------------------
  IdentityTelemetry id = {};
  strcpy(id.vin, "GOLDENVIN00000001");
  strcpy(id.ecuName, "ECM -EngineControl");
  strcpy(id.calibrationId, "0105411002");
  strcpy(id.motorModel, "EM-30S");
  id.cvn                   = 0xDEADBEEF;
  id.ecuBitrate            = 500000;
  id.batteryBitrate        = 500000;
  id.motorBitrate          = 0;
  id.ecuExtendedAddressing = 1;
  id.obdStandardRaw        = 0x25;
  id.batteryVendor         = static_cast<uint8_t>(BatteryVendor::kJbd);
  id.fields                = kIdVehicle | kIdEcu | kIdBattery;
  pub.publish(PacketType::kIdentity, SourceId::kFusion, id);

  // ---- engine ------------------------------------------------------------
  g_clock = 2000;
  EngineTelemetry e        = {};
  e.rpm                    = stat(1480, 1520, 1502, 2.5f, 1510, 1.5f, 1000, 8);
  e.throttlePercent        = stat(13, 15, 14, 0.5f, 14.5f, 0.1f, 1000, 8);
  e.mapKpa                 = stat(51, 92, 70.5f, 9.5f, 71, -2.0f, 1000, 8);
  e.timingAdvanceDeg       = stat(8, 9.5f, 8.75f, 0.4f, 9, 0.2f, 1000, 8);
  e.loadPercent            = stat(33, 35, 34, 0.6f, 34.5f, 0.05f, 1000, 8);
  e.shortFuelTrimPercent   = 1.5625f;
  e.windowMs               = 1000;
  e.intakeAirDeciC         = 310;   // 31.0 C
  e.oilTempDeciC           = 785;   // 78.5 C
  e.requests               = 20;
  e.replies                = 17;
  e.fuelSystemStatus[0]    = 0x02;
  e.fuelSystemStatus[1]    = 0x00;
  e.crcErrors              = 3;
  e.fields                 = kEngRpm | kEngThrottle | kEngMap | kEngTiming | kEngLoad |
                             kEngIntakeTemp | kEngOilTemp | kEngFuelTrim | kEngFuelSystem;
  pub.publish(PacketType::kEngine, SourceId::kEcu, e);

  // ---- motor -------------------------------------------------------------
  g_clock = 3000;
  MotorTelemetry m        = {};
  m.shaftRpm              = stat(800, 840, 820, 12, 835, 5, 1000, 8);
  m.currentAmps           = stat(8, 16, 12, 2.5f, 13, 1, 1000, 8);
  m.torqueNm              = emptyStats();
  m.throttlePercent       = emptyStats();
  m.busVolts              = 60.4f;
  m.windowMs              = 1000;
  m.faultFlags            = 0x00001010;
  m.controllerTempDeciC   = 425;   // 42.5 C
  m.motorTempDeciC        = 511;   // 51.1 C
  m.state                 = static_cast<uint8_t>(MotorState::kRun);
  m.linkKind              = static_cast<uint8_t>(MotorLink::kCan);
  m.fields                = kMotRpm | kMotCurrent | kMotBusVolts | kMotCtrlTemp | kMotMotorTemp;
  pub.publish(PacketType::kMotor, SourceId::kMotorDriver, m);

  // ---- dynamics ----------------------------------------------------------
  g_clock = 4000;
  DynamicsTelemetry d = {};
  d.accelX            = stat(-0.2f, 0.25f, 0.02f, 0.09f, 0.05f, 0.1f, 200, 40);
  d.accelY            = stat(-0.15f, 0.12f, -0.01f, 0.07f, 0.0f, 0.0f, 200, 40);
  d.accelZ            = stat(0.92f, 1.08f, 1.0f, 0.05f, 1.01f, 0.0f, 200, 40);
  d.pitchDeg          = stat(-3, 3, 0.5f, 1.2f, 0.8f, 0.1f, 200, 40);
  d.rollDeg           = stat(-5, 5, 1.0f, 2.0f, 1.5f, 0.2f, 200, 40);
  d.tiltDeg           = emptyStats();
  d.wheelSpeedKph     = stat(0, 32, 18.5f, 8.0f, 22, 3.0f, 200, 40);
  d.longAccelMss      = emptyStats();
  d.jerkMsss          = emptyStats();
  d.wheelRpm          = 310.5f;
  d.odometerM         = 1234.5f;
  d.wheelCircumM      = 1.256f;
  d.windowMs          = 200;
  d.rollRisk          = 0;
  d.fields            = kDynAccel | kDynAttitude | kDynWheel;
  pub.publish(PacketType::kDynamics, SourceId::kSensorGroup1, d);

  // ---- navigation --------------------------------------------------------
  g_clock = 5000;
  NavigationTelemetry n = {};
  n.gpsSpeedKph         = stat(0, 34, 19.0f, 8.5f, 24, 2.0f, 1000, 10);
  n.ecuSpeedKph         = stat(0, 33, 18.0f, 8.0f, 23, 2.0f, 1000, 10);
  // Pune, to 1e-7 degrees. Chosen because an exactly representable value would
  // hide the very float error the scaled integer exists to prevent.
  n.latitudeE7          = 184567891;
  n.longitudeE7         = 738765432;
  n.gpsAltitudeM        = 560.25f;
  n.baroAltitudeM       = 558.75f;
  n.pressureHpa         = 948.6f;
  n.ambientTempC        = 29.5f;
  n.headingDeg          = 137.5f;
  n.verticalInclinePct  = 2.5f;
  n.gpsTotalDistanceKm  = 12.345f;
  n.utcSeconds          = 1762000000u;
  n.ecuRunTimeSec       = 3600;
  n.ecuDistanceWithMilKm    = 0;
  n.ecuDistanceSinceClearKm = 4521;
  n.satelliteCount      = 9;
  n.fixQuality          = 1;
  n.fields = kNavFix | kNavPosition | kNavGpsSpeed | kNavEcuSpeed | kNavHeading |
             kNavGpsAlt | kNavBaro | kNavIncline | kNavOdometry | kNavUtc;
  pub.publish(PacketType::kNavigation, SourceId::kSensorGroup1, n);

  // ---- battery -----------------------------------------------------------
  g_clock = 6000;
  BatteryTelemetry b = {};
  b.currentAmps      = stat(-4.2f, -1.1f, -2.5f, 0.8f, -2.2f, 0.3f, 1000, 10);
  b.packVolts        = 53.4f;
  b.remainingAh      = 42.0f;
  b.fullAh           = 100.0f;
  b.cycleCount       = 12;
  b.socPercent       = 42;
  b.sohPercent       = 0;  // the JBD does not report it
  b.cellCount        = 17; // the bench pack: 17 configured, 16 carried
  b.vendor           = static_cast<uint8_t>(BatteryVendor::kJbd);
  b.chemistry        = static_cast<uint8_t>(CellChemistry::kLiFePO4);

  for (uint8_t i = 0; i < 16; ++i) {
    b.cellMilliVolts[i] = static_cast<uint16_t>(3330 + i);
  }
  // Channel 10 is the unpopulated tap: present in the array as 0, absent in the
  // mask. The decoder must report it as absent rather than as 0 mV.
  b.cellMilliVolts[9] = 0;
  for (uint8_t i = 0; i < 16; ++i) {
    if (i != 9) b.cellPresentMask |= (1u << i);
  }
  b.liveCellCount = 15;
  b.cellMinMv     = 3330;
  b.cellMaxMv     = 3345;
  b.cellSpreadMv  = 15;
  b.cellMinIndex  = 0;
  b.cellMaxIndex  = 15;
  b.balancingMask = 0x5;

  for (uint8_t i = 0; i < 5; ++i) b.tempDeciC[i] = static_cast<int16_t>(250 + i * 10);
  b.tempPresentMask = 0x1F;
  b.tempCount       = 5;
  b.tempMinDeciC    = 250;
  b.tempMaxDeciC    = 290;

  b.protectionFlags = kProtCellUv;
  b.mosfetState     = kMosCharge;  // discharge latched off, as on the bench
  b.fields          = kBatPack | kBatCurrent | kBatCapacity | kBatCells | kBatTemps |
                      kBatProtection | kBatMosfet | kBatBalance;
  pub.publish(PacketType::kBattery, SourceId::kBatteryA, b);

  // ---- status event ------------------------------------------------------
  g_clock = 7000;
  StatusEventTelemetry st = {};
  st.ecuMonitorStatus     = 0x00070000;
  st.ecuMonitorThisCycle  = 0x00000001;
  st.motorFaultFlags      = 0x00001010;
  st.dtc[0]               = 0x0145;  // P0145
  st.dtc[1]               = 0xC34D;  // U034D
  st.dtcListed            = 2;
  st.storedDtcCount       = 2;
  st.pendingDtcCount      = 1;
  st.milOn                = 1;
  st.obdStandardRaw       = 0x25;
  st.batteryProtectionFlags = kProtCellUv;
  st.batteryMosfetState   = kMosCharge;
  st.motorState           = static_cast<uint8_t>(MotorState::kFault);
  st.proximityMm          = 450;
  st.presenceDetected     = 1;
  st.rollRisk             = 0;
  const uint8_t uid[7]    = {0x04, 0x7A, 0x2B, 0x19, 0xC3, 0x5E, 0x80};
  memcpy(st.rfidUid, uid, sizeof(uid));
  st.rfidUidLength  = 7;
  st.rfidAuthorised = 1;
  st.eventMask      = kEvDtcAppeared | kEvMilChanged | kEvBatteryFault | kEvRfidPresented;
  st.fields         = kStEcuDtc | kStEcuMonitors | kStBattery | kStMotor | kStRfid | kStProximity;
  pub.publish(PacketType::kStatusEvent, SourceId::kFusion, st);

  fflush(stdout);
  fprintf(stderr, "wrote %u packets, %u refused\n",
          static_cast<unsigned>(pub.publishedCount()),
          static_cast<unsigned>(pub.refusedCount()));
  return pub.publishedCount() == 7 ? 0 : 1;
}
