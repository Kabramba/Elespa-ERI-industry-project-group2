// active_profile.h - which profile the CAN bus is currently configured for.
//
// Selection is a RUNTIME choice, not a build-time one: every profile in the
// registry is always compiled in, so a DIP switch, a serial command or a UI can
// change devices without reflashing. Nothing here is conditionally compiled.
#pragma once

#include <stddef.h>

#include "can/can_bus_profile.h"

namespace can {

// The profile in force. Never null - defaults to the first registry entry.
const CanBusProfile &active();

// Select by key or index. False (and no change) if it does not exist.
bool setActive(const char *key);
bool setActiveIndex(size_t index);

// True once something has selected deliberately, rather than defaulting.
bool activeWasChosen();

}  // namespace can
