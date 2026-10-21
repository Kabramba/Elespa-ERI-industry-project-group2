// packets.h - the telemetry payload schemas.
//
// Seven groups covering four sources: the Activa engine ECU, a BMS (JBD or
// Daly), the Votol motor driver, and the partner group's sensor board.
//
// GROUPING RULE: a packet holds fields that change at a similar rate, because
// that is what sets how often it must be sent. Mixing a 200 Hz accelerometer
// with a VIN forces the VIN to be retransmitted thousands of times for nothing,
// or the accelerometer to arrive useless and stale.
//
//   kIdentity     static   on change, or once a minute as a keepalive
//   kEngine       1 Hz     ICE channels; fast ones arrive as StatBlocks
//   kMotor        1 Hz     electric drive; same treatment
//   kDynamics     1 Hz     IMU and wheel - the highest-rate source by far
//   kNavigation   1 Hz     position, altitude, odometry; three speed sources
//   kBattery      1 Hz     pack state, vendor-independent
//   kStatusEvent  on change  faults, DTCs, RFID, presence
//
// EVERY PAYLOAD HAS A `fields` MASK. A dropped reading must never be presented
// as a zero - on a lossy link that is how "discharge MOSFET off" becomes
// indistinguishable from "that frame never arrived". Check the bit first.
//
// Structs are naturally aligned, not packed: the ESP32-S3 pays for unaligned
// access, and explicit padding plus a static_assert catches accidental drift
// better than a pragma hides it.
#pragma once

#include <stdint.h>

#include "telemetry/stat_block.h"

namespace telemetry {

constexpr uint8_t kMaxCells   = 16;  // both target packs are 16S
constexpr uint8_t kMaxTemps   = 8;
constexpr uint8_t kMaxDtcs    = 8;
constexpr uint8_t kMaxUidBytes = 10;  // MIFARE UIDs are 4, 7 or 10 bytes

// ============================== identity ==================================
enum IdentityField : uint16_t {
  kIdVehicle = 1u << 0,
  kIdEcu     = 1u << 1,
  kIdBattery = 1u << 2,
  kIdMotor   = 1u << 3,
  kIdSensors = 1u << 4,
};

struct IdentityTelemetry {
  char     vin[20];            // ME4XXXXXXXXXXXXXX
  char     ecuName[24];        // "ECM -EngineControl"
  char     calibrationId[20];  // "0105411002"
  char     motorModel[16];     // "EM-30S"
  uint32_t cvn;                // calibration verification number
  uint32_t ecuBitrate;         // measured, not assumed
  uint32_t batteryBitrate;
  uint32_t motorBitrate;       // 0 until the Votol interface is established
  uint16_t fields;
  uint8_t  ecuExtendedAddressing;  // 1 = 29-bit
  uint8_t  obdStandardRaw;         // PID 1C verbatim; 0x25 is outside ISO 15031-5
  uint8_t  batteryVendor;          // BatteryVendor
  uint8_t  batteryCellCount;
  uint8_t  motorLinkKind;          // MotorLink
  uint8_t  sensorBoardPresent;
  uint8_t  reserved[8];
};
static_assert(sizeof(IdentityTelemetry) == 112, "identity layout changed");

enum class MotorLink : uint8_t { kUnknown = 0, kUart = 1, kCan = 2 };

// =============================== engine ===================================
// Source: Activa ECU over OBD-II. See docs/ECU_schema.md.
enum EngineField : uint16_t {
  kEngRpm        = 1u << 0,
  kEngThrottle   = 1u << 1,
  kEngMap        = 1u << 2,
  kEngTiming     = 1u << 3,
  kEngLoad       = 1u << 4,
  kEngIntakeTemp = 1u << 5,
  kEngOilTemp    = 1u << 6,
  kEngFuelTrim   = 1u << 7,
  kEngFuelSystem = 1u << 8,
};

struct EngineTelemetry {
  // Fast channels: summarised over the window rather than sent sample by sample.
  StatBlock rpm;               // PID 0C, acquired ~8 Hz
  StatBlock throttlePercent;   // PID 11
  StatBlock mapKpa;            // PID 0B - manifold vacuum, the liveness check
  StatBlock timingAdvanceDeg;  // PID 0E
  StatBlock loadPercent;       // PID 04

  float    shortFuelTrimPercent;  // PID 06, slow enough for a scalar
  uint32_t windowMs;
  int16_t  intakeAirDeciC;        // PID 0F
  int16_t  oilTempDeciC;          // PID 5C - air-cooled, replaces coolant
  uint16_t requests;              // link quality over the window
  uint16_t replies;
  uint16_t fields;
  uint8_t  fuelSystemStatus[2];   // PID 03, one enum per bank
  uint8_t  crcErrors;
  uint8_t  reserved;
};
static_assert(sizeof(EngineTelemetry) == 184, "engine layout changed");

// =============================== motor ====================================
// Source: Votol EM-30S. See docs/VOTOL_EM30S_study.md - the interface itself is
// not yet established, so this is a target shape rather than a measured one.
enum MotorField : uint16_t {
  kMotRpm        = 1u << 0,
  kMotCurrent    = 1u << 1,
  kMotTorque     = 1u << 2,
  kMotBusVolts   = 1u << 3,
  kMotCtrlTemp   = 1u << 4,
  kMotMotorTemp  = 1u << 5,
  kMotThrottle   = 1u << 6,
};

// Votol controller-state enum, from the EM100 reverse engineering.
enum class MotorState : uint8_t {
  kIdle = 0, kInit = 1, kStart = 2, kRun = 3,
  kStop = 4, kBrake = 5, kWait = 6, kFault = 7,
};

struct MotorTelemetry {
  StatBlock shaftRpm;       // B16-B17 in the Votol CAN map
  StatBlock currentAmps;    // B9-B10, divide by 10
  StatBlock torqueNm;       // no direct source; inferred from current
  StatBlock throttlePercent;

  float    busVolts;        // B7-B8, divide by 10
  uint32_t windowMs;
  uint32_t faultFlags;      // B12-B15, 28 documented conditions
  int16_t  controllerTempDeciC;  // B18, offset -50
  int16_t  motorTempDeciC;       // B19, offset -50
  uint16_t fields;
  uint8_t  state;           // MotorState, B23
  uint8_t  linkKind;        // MotorLink
  uint8_t  reserved[4];
};
static_assert(sizeof(MotorTelemetry) == 152, "motor layout changed");

// ============================= dynamics ===================================
// Source: partner group's MPU6050 and KY-003 hall wheel sensor.
//
// This is the fastest data in the system - an MPU6050 samples in the hundreds
// of hertz and jerk is a second derivative, so raw transmission is out of the
// question. Nine StatBlocks is 54 features per window, and it is the packet
// that most justifies the statistics layer existing at all.
enum DynamicsField : uint16_t {
  kDynAccel     = 1u << 0,
  kDynAttitude  = 1u << 1,  // pitch, roll, tilt
  kDynWheel     = 1u << 2,  // speed, rpm, distance
  kDynLongAccel = 1u << 3,
  kDynJerk      = 1u << 4,
  kDynRollRisk  = 1u << 5,
};

struct DynamicsTelemetry {
  StatBlock accelX;          // g, raw/16384
  StatBlock accelY;
  StatBlock accelZ;
  StatBlock pitchDeg;        // atan2(-ax, hypot(ay, az))
  StatBlock rollDeg;         // atan2(ay, az)
  StatBlock tiltDeg;         // acos(az / |a|)
  StatBlock wheelSpeedKph;   // hall pulses x circumference
  StatBlock longAccelMss;    // d(speed)/dt
  StatBlock jerkMsss;        // d(accel)/dt - noisiest channel in the system

  float    wheelRpm;         // latest
  float    odometerM;        // cumulative since boot
  float    wheelCircumM;     // configuration, echoed for traceability
  uint32_t windowMs;
  uint16_t fields;
  uint8_t  rollRisk;         // |roll| over threshold
  uint8_t  reserved;
};
static_assert(sizeof(DynamicsTelemetry) == 308, "dynamics layout changed");

// ============================ navigation ==================================
// Source: NEO-6M GPS, BMP280 barometric, and the ECU's own road speed and
// distance counters. Deliberately together: three INDEPENDENT speed sources
// (GPS, wheel hall, ECU PID 0D) and two altitude sources (GPS, barometric) in
// one packet let a consumer cross-check rather than trust one sensor.
enum NavigationField : uint16_t {
  kNavFix        = 1u << 0,
  kNavPosition   = 1u << 1,
  kNavGpsSpeed   = 1u << 2,
  kNavEcuSpeed   = 1u << 3,
  kNavHeading    = 1u << 4,
  kNavGpsAlt     = 1u << 5,
  kNavBaro       = 1u << 6,
  kNavIncline    = 1u << 7,
  kNavOdometry   = 1u << 8,
  kNavUtc        = 1u << 9,
};

struct NavigationTelemetry {
  StatBlock gpsSpeedKph;      // NEO-6M, knots x 1.852
  StatBlock ecuSpeedKph;      // PID 0D - independent of the GPS

  // Scaled integer degrees, 1e-7 per LSB: about 11 mm, and EXACT. A float
  // carries ~7 significant digits, which at 2-3 digits of whole degrees leaves
  // metres of error - the classic GPS-in-float bug.
  int32_t  latitudeE7;
  int32_t  longitudeE7;

  float    gpsAltitudeM;
  float    baroAltitudeM;     // 44330 x (1 - (p/1013.25)^0.1903)
  float    pressureHpa;
  float    ambientTempC;      // BMP280
  float    headingDeg;
  float    verticalInclinePct;
  float    gpsTotalDistanceKm;  // haversine accumulated
  uint32_t utcSeconds;          // seconds since the Unix epoch, 0 if no fix
  uint32_t ecuRunTimeSec;       // PID 1F
  uint16_t ecuDistanceWithMilKm;      // PID 21
  uint16_t ecuDistanceSinceClearKm;   // PID 31
  uint16_t fields;
  uint8_t  satelliteCount;
  uint8_t  fixQuality;        // 0 none, 1 GPS, 2 DGPS
  uint8_t  reserved[12];
};
static_assert(sizeof(NavigationTelemetry) == 128, "navigation layout changed");

// ============================== battery ===================================
// Vendor independent on purpose: the JBD and the Daly differ in bitrate, frame
// format and addressing, but both describe a 16S pack with per-cell voltages,
// temperatures and protection flags. Those differences belong in the link layer.
enum class BatteryVendor : uint8_t { kUnknown = 0, kJbd = 1, kDaly = 2 };
enum class CellChemistry : uint8_t { kUnknown = 0, kLiFePO4 = 1, kNmc = 2, kLto = 3 };

enum BatteryField : uint16_t {
  kBatPack       = 1u << 0,
  kBatCurrent    = 1u << 1,
  kBatCapacity   = 1u << 2,
  kBatCells      = 1u << 3,
  kBatTemps      = 1u << 4,
  kBatProtection = 1u << 5,
  kBatMosfet     = 1u << 6,
  kBatBalance    = 1u << 7,
};

enum BatteryProtection : uint16_t {
  kProtCellOv   = 1u << 0,  kProtCellUv  = 1u << 1,
  kProtPackOv   = 1u << 2,  kProtPackUv  = 1u << 3,
  kProtChgOt    = 1u << 4,  kProtChgUt   = 1u << 5,
  kProtDsgOt    = 1u << 6,  kProtDsgUt   = 1u << 7,
  kProtChgOc    = 1u << 8,  kProtDsgOc   = 1u << 9,
  kProtShort    = 1u << 10, kProtAfeError = 1u << 11,
  kProtMosLock  = 1u << 12,
};

enum MosfetState : uint8_t { kMosCharge = 1u << 0, kMosDischarge = 1u << 1 };

struct BatteryTelemetry {
  StatBlock currentAmps;  // positive = charging; fast, so summarised

  float    packVolts;
  float    remainingAh;
  float    fullAh;
  uint32_t cellPresentMask;  // bit i set = channel i was read AND non-zero
  uint32_t balancingMask;
  uint16_t cellMilliVolts[kMaxCells];
  int16_t  tempDeciC[kMaxTemps];
  uint16_t cellMinMv;
  uint16_t cellMaxMv;
  uint16_t cellSpreadMv;  // the single most useful pack-health number
  int16_t  tempMinDeciC;
  int16_t  tempMaxDeciC;
  uint16_t cycleCount;
  uint16_t protectionFlags;
  uint16_t fields;
  uint8_t  vendor;          // BatteryVendor
  uint8_t  chemistry;       // CellChemistry
  uint8_t  cellCount;       // channels the BMS is configured for
  uint8_t  liveCellCount;   // of those, how many carry voltage
  uint8_t  tempCount;
  uint8_t  tempPresentMask;
  uint8_t  cellMinIndex;
  uint8_t  cellMaxIndex;
  uint8_t  socPercent;
  uint8_t  sohPercent;      // 0 when the BMS does not report it
  uint8_t  mosfetState;     // MosfetState bits
  uint8_t  reserved;
};
static_assert(sizeof(BatteryTelemetry) == 128, "battery layout changed");

// =========================== status and events ============================
// Everything discrete, from all four sources. Sent ON CHANGE rather than on a
// clock: a fault that appears and clears between two 1 Hz samples would
// otherwise never be seen, and a steady state costs nothing to not resend.
enum StatusField : uint16_t {
  kStEcuDtc      = 1u << 0,
  kStEcuMonitors = 1u << 1,
  kStBattery     = 1u << 2,
  kStMotor       = 1u << 3,
  kStRfid        = 1u << 4,
  kStProximity   = 1u << 5,
  kStRollRisk    = 1u << 6,
};

// Which conditions changed since the previous status packet. A consumer that
// only wants edges reads this and ignores the rest.
enum EventFlag : uint16_t {
  kEvDtcAppeared     = 1u << 0,
  kEvDtcCleared      = 1u << 1,
  kEvMilChanged      = 1u << 2,
  kEvBatteryFault    = 1u << 3,
  kEvMosfetChanged   = 1u << 4,
  kEvMotorFault      = 1u << 5,
  kEvMotorStateChange = 1u << 6,
  kEvRfidPresented   = 1u << 7,
  kEvRfidRejected    = 1u << 8,
  kEvPresenceChanged = 1u << 9,
  kEvRollRiskRaised  = 1u << 10,
};

struct StatusEventTelemetry {
  uint32_t ecuMonitorStatus;      // PID 01 raw
  uint32_t ecuMonitorThisCycle;   // PID 41 raw
  uint32_t motorFaultFlags;       // Votol B12-B15
  uint16_t dtc[kMaxDtcs];         // services 03 and 07, packed 16-bit codes
  uint16_t batteryProtectionFlags;
  uint16_t proximityMm;           // VL53L0X
  uint16_t eventMask;             // EventFlag bits - what changed
  uint16_t fields;
  uint8_t  rfidUid[kMaxUidBytes];
  uint8_t  rfidUidLength;         // 0, 4, 7 or 10
  uint8_t  rfidAuthorised;
  uint8_t  dtcListed;
  uint8_t  storedDtcCount;
  uint8_t  pendingDtcCount;
  uint8_t  milOn;
  uint8_t  obdStandardRaw;
  uint8_t  batteryMosfetState;
  uint8_t  motorState;            // MotorState
  uint8_t  presenceDetected;
  uint8_t  rollRisk;
  uint8_t  reserved[5];
};
static_assert(sizeof(StatusEventTelemetry) == 64, "status layout changed");

}  // namespace telemetry
