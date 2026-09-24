#include "can/can_bus_profile.h"

namespace can {

const char *toString(BusMode v) {
  switch (v) {
    case BusMode::kNormal:     return "normal";
    case BusMode::kListenOnly: return "listen-only";
    case BusMode::kNoAck:      return "no-ack";
  }
  return "?";
}

const char *toString(FrameFormat v) {
  switch (v) {
    case FrameFormat::kStandard11: return "std-11bit";
    case FrameFormat::kExtended29: return "ext-29bit";
    case FrameFormat::kMixed:      return "mixed";
  }
  return "?";
}

const char *toString(Interaction v) {
  switch (v) {
    case Interaction::kRemoteFrameQuery: return "remote-frame-query";
    case Interaction::kDataFrameRequest: return "data-frame-request";
    case Interaction::kBroadcastListen:  return "broadcast-listen";
    case Interaction::kIsoTp:            return "iso-tp";
  }
  return "?";
}

const char *toString(CrcScheme v) {
  switch (v) {
    case CrcScheme::kNone:                return "none";
    case CrcScheme::kModbusBigEndian:     return "crc16-modbus-be";
    case CrcScheme::kModbusLittleEndian:  return "crc16-modbus-le";
    case CrcScheme::kSum8:                return "sum8";
  }
  return "?";
}

const char *toString(AccessPolicy v) {
  switch (v) {
    case AccessPolicy::kListenOnly:      return "listen-only";
    case AccessPolicy::kReadOnlyQueries: return "read-only-queries";
    case AccessPolicy::kReadWrite:       return "read-write";
  }
  return "?";
}

const char *toString(DeviceClass v) {
  switch (v) {
    case DeviceClass::kGeneric:         return "generic";
    case DeviceClass::kBatteryBms:      return "battery-bms";
    case DeviceClass::kVehicleEcu:      return "vehicle-ecu";
    case DeviceClass::kInverter:        return "inverter";
    case DeviceClass::kCharger:         return "charger";
    case DeviceClass::kMotorController: return "motor-controller";
  }
  return "?";
}

const char *toString(Provenance v) {
  switch (v) {
    case Provenance::kMeasured:   return "measured";
    case Provenance::kDatasheet:  return "datasheet";
    case Provenance::kVendorDoc:  return "vendor-doc";
    case Provenance::kUnverified: return "UNVERIFIED";
  }
  return "?";
}

bool allowsTransmit(const CanBusProfile &p) {
  // Two independent gates, and both must agree. The access policy is intent;
  // the bus mode is what the controller will physically do.
  if (p.access == AccessPolicy::kListenOnly) return false;
  if (p.bus.mode == BusMode::kListenOnly) return false;
  return true;
}

bool mayTransmitId(const CanBusProfile &p, uint32_t id) {
  if (!allowsTransmit(p)) return false;
  if (p.txAllow.count == 0) return false;  // fail closed
  if (!idFitsFormat(p, id)) return false;
  for (uint8_t i = 0; i < p.txAllow.count && i < kMaxTxRules; ++i) {
    const TxRule &r = p.txAllow.rules[i];
    if (id < r.first || id > r.last) continue;
    if (r.mask != 0 && (id & r.mask) != (r.first & r.mask)) continue;
    return true;
  }
  return false;
}

bool idFitsFormat(const CanBusProfile &p, uint32_t id) {
  switch (p.bus.frameFormat) {
    case FrameFormat::kStandard11: return id <= 0x7FFu;
    case FrameFormat::kExtended29: return id <= 0x1FFFFFFFu;
    case FrameFormat::kMixed:      return id <= 0x1FFFFFFFu;
  }
  return false;
}

bool respondsOnId(const CanBusProfile &p, uint32_t id) {
  if (!idFitsFormat(p, id)) return false;
  if (p.protocol.idFirst <= p.protocol.idLast && p.protocol.idLast != 0)
    if (id >= p.protocol.idFirst && id <= p.protocol.idLast) return true;
  return p.protocol.responseId != 0 && id == p.protocol.responseId;
}

bool resolveTiming(const CanBusProfile &p, BitTiming &out) {
  return computeBitTiming(p.bus.bitrate, p.bus.samplePointPercent, out);
}

bool validate(const CanBusProfile &p, const char **reason) {
  const char *why = nullptr;

  if (p.device.key == nullptr || p.device.key[0] == '\0') {
    why = "device.key is empty";
  } else if (p.bus.txPin < 0 || p.bus.rxPin < 0) {
    why = "bus pins are unset";
  } else if (p.bus.txPin == p.bus.rxPin) {
    why = "bus TX and RX are the same pin";
  } else {
    BitTiming t;
    if (!resolveTiming(p, t)) {
      why = "bitrate is not reachable with legal register values";
    } else if (p.protocol.idFirst > p.protocol.idLast) {
      why = "protocol id range is inverted";
    } else if (p.protocol.idLast != 0 && !idFitsFormat(p, p.protocol.idLast)) {
      why = "protocol id range does not fit the declared frame format";
    } else if (p.protocol.requestId != 0 && !idFitsFormat(p, p.protocol.requestId)) {
      why = "requestId does not fit the declared frame format";
    } else if (p.protocol.interaction == Interaction::kBroadcastListen && allowsTransmit(p)) {
      // A listen-only protocol that is permitted to transmit is a contradiction
      // and would put unexpected frames on somebody else's bus.
      why = "broadcast-listen protocol must not be allowed to transmit";
    } else if (p.protocol.interaction == Interaction::kIsoTp &&
               (p.protocol.requestId == 0 || p.protocol.responseId == 0)) {
      why = "iso-tp needs both requestId and responseId";
    } else if (p.protocol.interaction == Interaction::kDataFrameRequest &&
               p.protocol.requestId == 0) {
      why = "data-frame-request needs a requestId";
    } else if (p.protocol.interaction == Interaction::kRemoteFrameQuery &&
               p.protocol.idLast == 0) {
      why = "remote-frame-query needs an id range to poll";
    } else if (p.protocol.retries == 0) {
      why = "retries must be at least 1";
    } else if (allowsTransmit(p) && p.txAllow.count == 0) {
      // Being permitted to transmit without declaring what is the bug this
      // whole mechanism exists to prevent.
      why = "profile may transmit but its TX allowlist is empty";
    } else if (!allowsTransmit(p) && p.txAllow.count != 0) {
      why = "passive profile declares TX rules, which is contradictory";
    }
  }

  if (reason != nullptr) *reason = why;
  return why == nullptr;
}

}  // namespace can
