// twai_profile.h - the only file that turns a CanBusProfile into ESP32 TWAI
// registers. Everything else in can/ stays hardware-free and host-testable.
#pragma once

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)

#include "driver/twai.h"

#include "can/can_bus_profile.h"

namespace can {

// Build the three driver structs from a profile. False if the profile is
// invalid or its bitrate is unreachable; `reason` then points at a static
// string. Pins must already be set - a sniffer profile ships without them.
bool buildTwaiConfigs(const CanBusProfile &p, twai_general_config_t &general,
                      twai_timing_config_t &timing, twai_filter_config_t &filter,
                      const char **reason = nullptr);

// Install and start the driver for this profile.
bool startFromProfile(const CanBusProfile &p, const char **reason = nullptr);

// Stop and uninstall, so another profile can take the peripheral.
bool stopBus();

// Acceptance filter matching exactly one 11-bit id. The TWAI filter registers
// are bit-shifted rather than plain id comparisons, which is easy to get wrong
// by hand.
twai_filter_config_t standardIdFilter(uint32_t id);

}  // namespace can

#endif
