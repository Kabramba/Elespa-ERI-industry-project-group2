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

    // -----------------------------------------------------------------
    // JBD SP24S007 BMS. Every value below was confirmed on the bench; see
    // docs/SESSION_NOTES.md for the measurements behind each one.
    // -----------------------------------------------------------------
    {
        /* device */ {
            "jbd-sp24s",
            "JBD SP24S007 BMS",
            "Jiabaida (JBD)",
            "SP24S007L24S100A",
            "JBD protection-board CAN",
            DeviceClass::kBatteryBms,
            Provenance::kMeasured,
            "17 channels configured on a 16S pack; channel 10 reads 0 mV and "
            "latches CELL_UV. TX=GPIO5/RX=GPIO4 is the opposite of the vendor "
            "notes and was established by probing.",
        },
        /* bus */ {
            /* bitrate */ 500000,
            /* txPin   */ 5,
            /* rxPin   */ 4,
            /* mode    */ BusMode::kNormal,
            /* format  */ FrameFormat::kStandard11,
            // Measured 86-87 % of frames won versus 0-27 % single-shot on this
            // bus, with TEC stable at 114-169 and no bus-off.
            /* hwRetx  */ true,
            /* sample  */ 80,
            /* rxQueue */ 32,
            /* txQueue */ 4,
            /* code    */ 0,
            /* mask    */ 0xFFFFFFFF,
            /* extTerm */ false,
        },
        /* protocol */ {
            Interaction::kRemoteFrameQuery,
            ByteOrder::kBigEndian,
            CrcScheme::kModbusBigEndian,  // confirmed: payload 11 05 -> E3 CD
            /* idFirst */ 0x100,
            /* idLast  */ 0x110,
            /* reqId   */ 0,
            /* respId  */ 0,
            /* timeout */ 100,
            /* retries */ 2,
            /* poll    */ 2000,
        },
        /* access */ AccessPolicy::kReadOnlyQueries,
    },

    // -----------------------------------------------------------------
    // Generic listen-only sniffers. These transmit NOTHING - not even an ACK -
    // so they are the safe way to approach a bus whose parameters are unknown,
    // including one you do not own. Use them to discover a device's bitrate and
    // id map before writing a profile for it.
    // -----------------------------------------------------------------
    {
        /* device */ {
            "sniff-500k",
            "Generic 500 kbit/s sniffer",
            "-", "-", "passive observation",
            DeviceClass::kGeneric,
            Provenance::kMeasured,
            "Listen-only. Drives nothing, ACKs nothing, so it cannot disturb a "
            "live bus. Pins must be supplied by the caller.",
        },
        /* bus */ {
            500000, -1, -1, BusMode::kListenOnly, FrameFormat::kMixed,
            false, 80, 64, 1, 0, 0xFFFFFFFF, false,
        },
        /* protocol */ {
            Interaction::kBroadcastListen, ByteOrder::kBigEndian, CrcScheme::kNone,
            0, 0, 0, 0, 0, 1, 0,
        },
        /* access */ AccessPolicy::kListenOnly,
    },
    {
        /* device */ {
            "sniff-250k",
            "Generic 250 kbit/s sniffer",
            "-", "-", "passive observation",
            DeviceClass::kGeneric,
            Provenance::kMeasured,
            "As sniff-500k. 250 kbit/s is the other rate worth trying first on "
            "an unknown vehicle or industrial bus.",
        },
        /* bus */ {
            250000, -1, -1, BusMode::kListenOnly, FrameFormat::kMixed,
            false, 80, 64, 1, 0, 0xFFFFFFFF, false,
        },
        /* protocol */ {
            Interaction::kBroadcastListen, ByteOrder::kBigEndian, CrcScheme::kNone,
            0, 0, 0, 0, 0, 1, 0,
        },
        /* access */ AccessPolicy::kListenOnly,
    },

    // -----------------------------------------------------------------
    // NOT YET ADDED - deliberately.
    //
    //   Daly BMS        Interaction::kDataFrameRequest, FrameFormat::kExtended29.
    //                   Needs the real request/response ids and bitrate.
    //   Activa OBD-II   Interaction::kIsoTp, requestId 0x7DF / responseId 0x7E8
    //                   on ISO 15765-4. Needs an ISO-TP transport underneath
    //                   before a profile is useful, and must start from
    //                   AccessPolicy::kListenOnly on a vehicle that is running.
    //
    // Both are left out rather than guessed. Confirm their parameters with
    // sniff-250k / sniff-500k first, then add entries with honest provenance.
    // -----------------------------------------------------------------
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
