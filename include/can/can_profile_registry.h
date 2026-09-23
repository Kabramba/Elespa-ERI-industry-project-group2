// can_profile_registry.h - every known bus profile, in one place.
//
// The table is a static constexpr array in can_profile_registry.cpp: profiles
// describe fixed hardware facts, so they cost no RAM, need no persistence, and
// are all visible in a single file. Adding a device means adding one entry.
#pragma once

#include <stddef.h>

#include "can/can_bus_profile.h"

namespace can {

size_t               profileCount();
const CanBusProfile *profileAt(size_t index);            // nullptr past the end
const CanBusProfile *findProfile(const char *key);       // nullptr if unknown
const CanBusProfile *findByClass(DeviceClass cls, size_t nth = 0);

// Validates every entry. Returns the count that failed and, if given, the key
// and reason of the first. Worth calling once at boot: a malformed profile is a
// build-time mistake that should not wait to surface as a dead bus.
size_t validateAll(const char **firstBadKey = nullptr, const char **firstReason = nullptr);

}  // namespace can
