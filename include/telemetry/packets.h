// packets.h - the telemetry payload schemas.
//
// GROUPING RULE: a packet holds fields that change at a similar rate, because
// that is what sets how often the packet must be sent. Mixing a 10 Hz signal
// with a once-per-minute one forces the slow field to be retransmitted 600 times
// for nothing, or the fast one to be delivered stale.
//
//   kIdentity   static      on change, or once a minute as a keepalive
//   kEngine     ~1 Hz       fast channels arrive as StatBlocks, not raw samples
//   kVehicle    ~1 Hz       road-side state, slower and coarser than engine
//   kBattery    ~1 Hz       pack state; current is fast, so it is a StatBlock
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

constexpr uint8_t kMaxCells = 16;  // both target packs are 16S
constexpr uint8_t kMaxTemps = 8;
constexpr uint8_t kMaxDtcs  = 8;

// ============================== identity ==================================
enum IdentityField : uint16_t {
  kIdVehicle = 1u << 0,
  kIdEcu     = 1u << 1,
  kIdBattery = 1u << 2,
};

struct IdentityTelemetry {
  char     vin[20];            // ME4XXXXXXXXXXXXXX - 17 chars plus terminator
  char     ecuName[24];        // "ECM -EngineControl"
  char     calibrationId[20];  // "0105411002"
  uint32_t cvn;                // calibration verification number
  uint32_t busBitrate;         // measured, not assumed
  uint16_t fields;
  uint8_t  extendedAddressing;  // 1 = 29-bit, 0 = 11-bit
  uint8_t  obdStandardRaw;      // PID 1C verbatim; 0x25 is outside ISO 15031-5
  uint8_t  batteryVendor;       // BatteryVendor
  uint8_t  batteryCellCount;
  uint8_t  reserved[2];
};
static_assert(sizeof(IdentityTelemetry) == 80, "identity layout changed");

// =============================== engine ===================================
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
  StatBlock rpm;               // PID 0C, polled ~8 Hz
  StatBlock throttlePercent;   // PID 11
  StatBlock mapKpa;            // PID 0B - manifold vacuum, the liveness check
  StatBlock timingAdvanceDeg;  // PID 0E
  StatBlock loadPercent;       // PID 04

  float    shortFuelTrimPercent;  // PID 06, slow enough for a scalar
  uint32_t windowMs;              // span the StatBlocks cover
  int16_t  intakeAirDeciC;        // PID 0F, 0.1 degC
  int16_t  oilTempDeciC;          // PID 5C - air-cooled, so this replaces coolant
  uint16_t requests;              // link quality over the window
  uint16_t replies;
  uint16_t fields;
  uint8_t  fuelSystemStatus[2];   // PID 03, one enum per bank
  uint8_t  crcErrors;
  uint8_t  reserved;
};
static_assert(sizeof(EngineTelemetry) == 184, "engine layout changed");

// =============================== vehicle ==================================
enum VehicleField : uint16_t {
  kVehSpeed       = 1u << 0,
  kVehRunTime     = 1u << 1,
  kVehDistanceMil = 1u << 2,
  kVehDistClear   = 1u << 3,
  kVehOdometer    = 1u << 4,
  kVehDtc         = 1u << 5,
  kVehMonitors    = 1u << 6,
};

struct VehicleTelemetry {
  StatBlock speedKph;  // PID 0D - slow at rest, fast on the road

  float    odometerKm;              // PID A6; NOT supported by this ECU
  uint32_t runTimeSec;              // PID 1F
  uint32_t monitorStatus;           // PID 01 raw, decode per docs/ECU_schema.md
  uint32_t monitorThisCycle;        // PID 41 raw
  uint16_t distanceWithMilKm;       // PID 21
  uint16_t distanceSinceClearedKm;  // PID 31
  uint16_t dtc[kMaxDtcs];           // services 03 and 07, packed 16-bit codes
  uint16_t fields;
  uint8_t  milOn;
  uint8_t  storedDtcCount;
  uint8_t  pendingDtcCount;
  uint8_t  dtcListed;      // how many of dtc[] are populated
  uint8_t  obdStandardRaw;  // PID 1C verbatim
  uint8_t  reserved[5];     // pads to 80 and leaves room to grow
};
static_assert(sizeof(VehicleTelemetry) == 80, "vehicle layout changed");

// =============================== battery ==================================
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

// Same bit order on both vendors; the link layer translates if it must.
enum BatteryProtection : uint16_t {
  kProtCellOv    = 1u << 0,
  kProtCellUv    = 1u << 1,
  kProtPackOv    = 1u << 2,
  kProtPackUv    = 1u << 3,
  kProtChgOt     = 1u << 4,
  kProtChgUt     = 1u << 5,
  kProtDsgOt     = 1u << 6,
  kProtDsgUt     = 1u << 7,
  kProtChgOc     = 1u << 8,
  kProtDsgOc     = 1u << 9,
  kProtShort     = 1u << 10,
  kProtAfeError  = 1u << 11,
  kProtMosLock   = 1u << 12,
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

// ===================== reserved, not yet implemented ======================
// Declared so the type numbering and the consumer feature list are stable
// before the hardware exists. Adding fields to these is not a breaking change;
// renumbering PacketType would be.
struct MotorTelemetry {
  StatBlock shaftRpm;
  StatBlock currentAmps;
  StatBlock torqueNm;
  float    busVolts;
  int16_t  controllerTempDeciC;
  int16_t  motorTempDeciC;
  uint16_t faultFlags;
  uint16_t fields;
  uint8_t  state;
  uint8_t  reserved[3];
};

struct IotGroupTelemetry {
  StatBlock channel[4];  // partner group's sensor set, shape to be agreed
  uint32_t  channelMask;
  uint16_t  fields;
  uint8_t   channelCount;
  uint8_t   reserved;
};

}  // namespace telemetry
