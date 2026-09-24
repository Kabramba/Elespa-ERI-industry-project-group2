#include "can/can_profile_registry.h"

#include <string.h>

namespace can {
namespace {

// ===========================================================================
// THE TABLE
// ===========================================================================
//
// Every field here is either measured on the physical device or a deliberate,
// documented default. Provenance says which - do not add an entry claiming
// kMeasured for numbers that came from a forum post.
//
constexpr CanBusProfile kProfiles[] = {

    // =================================================================
    // JBD SP24S007 BMS - every value confirmed on the bench.
    // =================================================================
    {
        /* device */ {
            "jbd-sp24s", "JBD SP24S007 BMS", "Jiabaida (JBD)", "SP24S007L24S100A",
            "JBD protection-board CAN", DeviceClass::kBatteryBms, Provenance::kMeasured,
            "17 channels configured on a 16S pack; channel 10 reads 0 mV and latches "
            "CELL_UV. TX=GPIO5/RX=GPIO4 is the opposite of the vendor notes and was "
            "established by probing.",
        },
        /* bus */ {
            500000, 5, 4, BusMode::kNormal, FrameFormat::kStandard11,
            // 86-87 % of frames won versus 0-27 % single-shot, TEC stable 114-169.
            /* hwRetx */ true, /* sample */ 80, 32, 4, 0, 0xFFFFFFFF,
            /* extTerm */ false,
        },
        /* protocol */ {
            Interaction::kRemoteFrameQuery, ByteOrder::kBigEndian,
            CrcScheme::kModbusBigEndian,  // confirmed: payload 11 05 -> E3 CD
            0x100, 0x110, 0, 0, 100, 2, 2000,
        },
        /* access  */ AccessPolicy::kReadOnlyQueries,
        /* txAllow */ {{{0x100, 0x110, 0}}, 1},
    },

    // =================================================================
    // Honda Activa 6G (BS6, 2024) engine ECU, 6-pin red Honda DLC.
    //
    // MEASURED (docs/activa-can.md): CAN-H = Yellow/Black, CAN-L = Yellow/Red,
    // GND = Green. Idle 2.4-2.5 V both lines with ignition ON. CAN-H<->CAN-L
    // reads 124 ohm with all nodes connected, i.e. the vehicle carries only ONE
    // terminator - the DLC is the open end and the tester supplies the second.
    // The SN65HVD230 module's onboard 120 ohm therefore STAYS FITTED.
    //
    // NOT MEASURED: the bitrate, and whether the bus carries broadcast traffic
    // at all. Provenance is kUnverified for exactly that reason - the bitrate is
    // the field that decides whether a bus comes up, and it is still a guess.
    // Confirm it with the activa-listen profiles before using either OBD entry.
    //
    // Never connect DLC Brown (SCS - grounding it puts the ECU into service
    // mode) or Orange (K-line at ~12 V, would destroy a GPIO).
    // =================================================================
    {
        /* device */ {
            "activa-listen", "Honda Activa 6G ECU (listen-only)", "Honda",
            "Activa 6G BS6 2024", "passive observation", DeviceClass::kVehicleEcu,
            Provenance::kUnverified,
            "START HERE. Transmits nothing and ACKs nothing, so it cannot disturb a "
            "running vehicle. 500 kbit/s is the ISO 15765-4 default and the first "
            "candidate; if it yields only bus errors try sniff-250k / -1m / -125k. "
            "If a rate that should work gives only error frames, swap the two yellow "
            "wires - CAN polarity was deduced from ISO 15031-3, not confirmed.",
        },
        /* bus */ {
            500000, 5, 4, BusMode::kListenOnly, FrameFormat::kMixed,
            false, 80, 64, 1, 0, 0xFFFFFFFF,
            /* extTerm */ true,  // module's 120 ohm must stay fitted
        },
        /* protocol */ {
            Interaction::kBroadcastListen, ByteOrder::kBigEndian, CrcScheme::kNone,
            0, 0, 0, 0, 0, 1, 0,
        },
        /* access  */ AccessPolicy::kListenOnly,
        /* txAllow */ {},
    },
    {
        /* device */ {
            "activa-obd11", "Honda Activa 6G ECU (OBD-II, 11-bit)", "Honda",
            "Activa 6G BS6 2024", "ISO 15765-4 / OBD-II", DeviceClass::kVehicleEcu,
            Provenance::kUnverified,
            "Try 11-bit before 29-bit. Functional request 0x7DF, physical 0x7E0, "
            "responses 0x7E8-0x7EF. Flow Control goes to the response id minus 8. "
            "P2 = 50 ms, extend to 5000 ms on NRC 0x78 (responsePending). One "
            "outstanding request at a time, <= 20 requests/s. Stationary on the "
            "centre stand only.",
        },
        /* bus */ {
            500000, 5, 4, BusMode::kNormal, FrameFormat::kStandard11,
            true, 80, 32, 4, 0, 0xFFFFFFFF, true,
        },
        /* protocol */ {
            Interaction::kIsoTp, ByteOrder::kBigEndian, CrcScheme::kNone,
            /* response range */ 0x7E8, 0x7EF,
            /* requestId      */ 0x7DF,
            /* responseId     */ 0x7E8,
            /* timeout ms     */ 100,
            /* retries        */ 3,
            /* poll ms        */ 100,  // 10/s, half the 20/s ceiling
        },
        /* access  */ AccessPolicy::kReadOnlyQueries,
        // Functional request, plus the physical request range. Nothing else.
        /* txAllow */ {{{0x7DF, 0x7DF, 0}, {0x7E0, 0x7E7, 0}}, 2},
    },
    {
        /* device */ {
            "activa-obd29", "Honda Activa 6G ECU (OBD-II, 29-bit)", "Honda",
            "Activa 6G BS6 2024", "ISO 15765-4 / OBD-II extended",
            DeviceClass::kVehicleEcu, Provenance::kUnverified,
            "Fallback if 11-bit does not answer. Functional 0x18DB33F1, physical "
            "0x18DA10F1, responses 0x18DAF1xx. Same timing and rate limits as "
            "activa-obd11.",
        },
        /* bus */ {
            500000, 5, 4, BusMode::kNormal, FrameFormat::kExtended29,
            true, 80, 32, 4, 0, 0xFFFFFFFF, true,
        },
        /* protocol */ {
            Interaction::kIsoTp, ByteOrder::kBigEndian, CrcScheme::kNone,
            0x18DAF100, 0x18DAF1FF,
            /* requestId  */ 0x18DB33F1,
            /* responseId */ 0x18DAF110,
            100, 3, 100,
        },
        /* access  */ AccessPolicy::kReadOnlyQueries,
        // 0x18DAxxF1: the mask pins the fixed halves so only a physical request
        // to some ECU address matches - a plain range would let far more through.
        /* txAllow */ {{{0x18DB33F1, 0x18DB33F1, 0},
                        {0x18DA00F1, 0x18DAFFF1, 0xFFFF00FF}}, 2},
    },

    // =================================================================
    // Generic listen-only sniffers, one per candidate bitrate. These transmit
    // NOTHING - not even an ACK - which makes them the only safe way onto a bus
    // you do not own. Use them to find an unknown device's bitrate before
    // writing a profile for it. Pins are supplied by the caller.
    // =================================================================
    {
        {"sniff-500k", "Generic 500 kbit/s sniffer", "-", "-", "passive observation",
         DeviceClass::kGeneric, Provenance::kMeasured,
         "ISO 15765-4 default; try this first."},
        {500000, -1, -1, BusMode::kListenOnly, FrameFormat::kMixed,
         false, 80, 64, 1, 0, 0xFFFFFFFF, false},
        {Interaction::kBroadcastListen, ByteOrder::kBigEndian, CrcScheme::kNone,
         0, 0, 0, 0, 0, 1, 0},
        AccessPolicy::kListenOnly,
        {},
    },
    {
        {"sniff-250k", "Generic 250 kbit/s sniffer", "-", "-", "passive observation",
         DeviceClass::kGeneric, Provenance::kMeasured,
         "Second candidate; also the J1939 rate."},
        {250000, -1, -1, BusMode::kListenOnly, FrameFormat::kMixed,
         false, 80, 64, 1, 0, 0xFFFFFFFF, false},
        {Interaction::kBroadcastListen, ByteOrder::kBigEndian, CrcScheme::kNone,
         0, 0, 0, 0, 0, 1, 0},
        AccessPolicy::kListenOnly,
        {},
    },
    {
        {"sniff-1m", "Generic 1 Mbit/s sniffer", "-", "-", "passive observation",
         DeviceClass::kGeneric, Provenance::kMeasured, "Third candidate."},
        {1000000, -1, -1, BusMode::kListenOnly, FrameFormat::kMixed,
         false, 80, 64, 1, 0, 0xFFFFFFFF, false},
        {Interaction::kBroadcastListen, ByteOrder::kBigEndian, CrcScheme::kNone,
         0, 0, 0, 0, 0, 1, 0},
        AccessPolicy::kListenOnly,
        {},
    },
    {
        {"sniff-125k", "Generic 125 kbit/s sniffer", "-", "-", "passive observation",
         DeviceClass::kGeneric, Provenance::kMeasured, "Fourth candidate."},
        {125000, -1, -1, BusMode::kListenOnly, FrameFormat::kMixed,
         false, 80, 64, 1, 0, 0xFFFFFFFF, false},
        {Interaction::kBroadcastListen, ByteOrder::kBigEndian, CrcScheme::kNone,
         0, 0, 0, 0, 0, 1, 0},
        AccessPolicy::kListenOnly,
        {},
    },

    // =================================================================
    // Daly BMS - still absent, and not for want of a slot. It needs the same
    // treatment the Activa got: a reference doc with the real bitrate, frame
    // format (expected 29-bit) and request/response ids. Guessing those would
    // put invented numbers behind a kMeasured-looking entry.
    // =================================================================
};

constexpr size_t kCount = sizeof(kProfiles) / sizeof(kProfiles[0]);

}  // namespace

size_t profileCount() { return kCount; }

const CanBusProfile *profileAt(size_t index) {
  return (index < kCount) ? &kProfiles[index] : nullptr;
}

const CanBusProfile *findProfile(const char *key) {
  if (key == nullptr) return nullptr;
  for (size_t i = 0; i < kCount; ++i)
    if (strcmp(kProfiles[i].device.key, key) == 0) return &kProfiles[i];
  return nullptr;
}

const CanBusProfile *findByClass(DeviceClass cls, size_t nth) {
  size_t seen = 0;
  for (size_t i = 0; i < kCount; ++i) {
    if (kProfiles[i].device.deviceClass != cls) continue;
    if (seen++ == nth) return &kProfiles[i];
  }
  return nullptr;
}

size_t validateAll(const char **firstBadKey, const char **firstReason) {
  size_t bad = 0;
  for (size_t i = 0; i < kCount; ++i) {
    const char *why = nullptr;
    // Sniffer profiles intentionally ship without pins; the caller supplies
    // them. Validate a copy with placeholder pins so the rest is still checked.
    CanBusProfile p = kProfiles[i];
    if (p.bus.txPin < 0 && p.bus.rxPin < 0) {
      p.bus.txPin = 5;
      p.bus.rxPin = 4;
    }
    if (!validate(p, &why)) {
      if (bad == 0) {
        if (firstBadKey) *firstBadKey = kProfiles[i].device.key;
        if (firstReason) *firstReason = why;
      }
      ++bad;
    }
  }
  return bad;
}

}  // namespace can
