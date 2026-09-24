// can_bus_profile.h - everything needed to bring up a CAN bus for one device,
// plus the metadata describing what is on the other end.
//
// A profile is pure data with pure helpers. It knows nothing about TWAI or
// Arduino, so it compiles and tests on the host; `twai_profile.h` turns one
// into controller registers.
//
// The field that actually generalises is Interaction. Devices differ far more
// in HOW you get data out of them than in bitrate or pin assignment:
//
//   JBD BMS    remote-frame query, reply on the same 11-bit id
//   Daly BMS   data-frame request, reply on a different 29-bit id
//   OBD-II     ISO-TP segmented transport, flow control, 0x7DF -> 0x7E8
//
// A profile carrying only bitrate and an id list handles the first and breaks
// on the third, so the transport belongs in the profile.
#pragma once

#include <stddef.h>   // size_t, for kMaxTxRules
#include <stdint.h>

#include "can/can_timing.h"

namespace can {

// ---------------------------------------------------------------- bus layer

enum class BusMode : uint8_t {
  kNormal,      // transmits and acknowledges
  kListenOnly,  // never drives the bus at all - the only safe way onto a bus
                // you do not own, e.g. a running vehicle
  kNoAck,       // transmits without requiring an ACK; bench diagnostics only
};

enum class FrameFormat : uint8_t { kStandard11, kExtended29, kMixed };

// --------------------------------------------------------- protocol layer

enum class Interaction : uint8_t {
  kRemoteFrameQuery,  // RTR out, data frame back on the same id
  kDataFrameRequest,  // data frame out, reply on a different id
  kBroadcastListen,   // device transmits unprompted; host only listens
  kIsoTp,             // ISO 15765-2 segmented transport (OBD-II, UDS)
};

enum class ByteOrder : uint8_t { kBigEndian, kLittleEndian };

enum class CrcScheme : uint8_t {
  kNone,
  kModbusBigEndian,     // CRC-16/MODBUS in the trailing 2 bytes, big-endian
  kModbusLittleEndian,
  kSum8,                // 8-bit additive checksum in the trailing byte
};

// ------------------------------------------------------------ safety

// What this firmware may put on the bus. Defence in depth: a profile for
// somebody else's vehicle bus must not be able to transmit by accident.
enum class AccessPolicy : uint8_t {
  kListenOnly,       // transmit nothing, ever
  kReadOnlyQueries,  // only the read requests the profile declares
  kReadWrite,        // writes allowed - must be opted into deliberately
};

// ------------------------------------------------- transmit permissions

// One permitted transmit pattern. An id is allowed when it falls in
// [first, last] AND, if mask is non-zero, agrees with `first` on the masked
// bits. The mask exists for patterns a plain range cannot express - OBD-II's
// 29-bit physical requests are 0x18DAxxF1, where the low byte is fixed.
struct TxRule {
  uint32_t first = 0;
  uint32_t last  = 0;
  uint32_t mask  = 0;  // 0 = range only
};

constexpr size_t kMaxTxRules = 4;

// Fail closed: an empty allowlist permits nothing, whatever the access policy
// says. A profile that may transmit must state exactly what.
struct TxAllowlist {
  TxRule  rules[kMaxTxRules] = {};
  uint8_t count              = 0;
};

// ------------------------------------------------------------ metadata

enum class DeviceClass : uint8_t {
  kGeneric, kBatteryBms, kVehicleEcu, kInverter, kCharger, kMotorController,
};

// How much to trust the numbers in this profile.
enum class Provenance : uint8_t {
  kMeasured,    // confirmed against the physical device
  kDatasheet,   // from the manufacturer's documentation, not yet confirmed
  kVendorDoc,   // from a third-party protocol write-up
  kUnverified,  // guessed or inherited; do not rely on it
};

// --------------------------------------------------------------- the parts

struct BusConfig {
  uint32_t bitrate = 500000;
  int8_t   txPin   = -1;  // -1 = must be supplied by the caller
  int8_t   rxPin   = -1;

  BusMode     mode        = BusMode::kNormal;
  FrameFormat frameFormat = FrameFormat::kStandard11;

  // Per profile, not global. Hardware retransmission is right for a bus whose
  // ACK is intermittently missed; a busy vehicle bus may want otherwise.
  bool hardwareRetransmit = true;

  uint8_t  samplePointPercent = 80;
  uint16_t rxQueueLen         = 32;
  uint16_t txQueueLen         = 4;

  // Acceptance filter. The default pair accepts everything.
  uint32_t acceptCode = 0;
  uint32_t acceptMask = 0xFFFFFFFF;

  // Bring-up metadata, not a register: does the bus rely on termination the
  // transceiver module does not provide?
  bool expectsExternalTermination = false;
};

struct ProtocolConfig {
  Interaction interaction = Interaction::kRemoteFrameQuery;
  ByteOrder   byteOrder   = ByteOrder::kBigEndian;
  CrcScheme   crc         = CrcScheme::kNone;

  // Inclusive id range the device answers on. For kRemoteFrameQuery these are
  // also the ids to query.
  uint32_t idFirst = 0;
  uint32_t idLast  = 0;

  // For kDataFrameRequest and kIsoTp, where requests and replies live.
  uint32_t requestId  = 0;
  uint32_t responseId = 0;

  uint16_t replyTimeoutMs = 100;
  uint8_t  retries        = 4;
  uint16_t pollPeriodMs   = 1000;
};

struct DeviceInfo {
  const char *key;          // stable lookup key, e.g. "jbd-sp24s"
  const char *displayName;
  const char *manufacturer;
  const char *model;
  const char *protocolName;
  DeviceClass deviceClass = DeviceClass::kGeneric;
  Provenance  provenance  = Provenance::kUnverified;
  const char *notes       = "";
};

struct CanBusProfile {
  DeviceInfo     device;
  BusConfig      bus;
  ProtocolConfig protocol;
  AccessPolicy   access = AccessPolicy::kListenOnly;  // safe default
  TxAllowlist    txAllow = {};                        // empty = transmit nothing
};

// --------------------------------------------------------------- utilities

const char *toString(BusMode v);
const char *toString(FrameFormat v);
const char *toString(Interaction v);
const char *toString(CrcScheme v);
const char *toString(AccessPolicy v);
const char *toString(DeviceClass v);
const char *toString(Provenance v);

// May this profile transmit at all?
bool allowsTransmit(const CanBusProfile &p);

// May this profile transmit THIS id? Every transmit path must go through here.
// Returns false when the profile cannot transmit at all, when the allowlist is
// empty, and when the id matches no rule - three independent ways to say no.
bool mayTransmitId(const CanBusProfile &p, uint32_t id);

// Is `id` one the device is expected to answer on?
bool respondsOnId(const CanBusProfile &p, uint32_t id);

// Does `id` fit the profile's declared frame format?
bool idFitsFormat(const CanBusProfile &p, uint32_t id);

// Solve the profile's bitrate and sample point into register values.
bool resolveTiming(const CanBusProfile &p, BitTiming &out);

// Internal consistency check. On failure `reason` points at a static string
// saying which rule was broken. Call this before bringing a bus up.
bool validate(const CanBusProfile &p, const char **reason = nullptr);

}  // namespace can
