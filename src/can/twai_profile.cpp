#include "can/twai_profile.h"

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)

namespace can {
namespace {

twai_mode_t toTwaiMode(BusMode m) {
  switch (m) {
    case BusMode::kNormal:     return TWAI_MODE_NORMAL;
    case BusMode::kListenOnly: return TWAI_MODE_LISTEN_ONLY;
    case BusMode::kNoAck:      return TWAI_MODE_NO_ACK;
  }
  return TWAI_MODE_LISTEN_ONLY;  // safest fallback: drives nothing
}

}  // namespace

twai_filter_config_t standardIdFilter(uint32_t id) {
  // Single-filter mode places an 11-bit id in the top bits; mask bits that are
  // SET are "don't care", so an exact match masks everything else in.
  twai_filter_config_t f;
  f.acceptance_code = (id & 0x7FFu) << 21;
  f.acceptance_mask = ~(0x7FFu << 21);
  f.single_filter   = true;
  return f;
}

bool buildTwaiConfigs(const CanBusProfile &p, twai_general_config_t &general,
                      twai_timing_config_t &timing, twai_filter_config_t &filter,
                      const char **reason) {
  const char *why = nullptr;
  if (!validate(p, &why)) {
    if (reason) *reason = why;
    return false;
  }

  BitTiming t;
  if (!resolveTiming(p, t)) {
    if (reason) *reason = "bitrate not reachable with legal register values";
    return false;
  }

  general = TWAI_GENERAL_CONFIG_DEFAULT(static_cast<gpio_num_t>(p.bus.txPin),
                                        static_cast<gpio_num_t>(p.bus.rxPin),
                                        toTwaiMode(p.bus.mode));
  general.rx_queue_len = p.bus.rxQueueLen;
  general.tx_queue_len = p.bus.txQueueLen;
  general.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ARB_LOST |
                           TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF |
                           TWAI_ALERT_BUS_RECOVERED;

  timing.brp             = t.brp;
  timing.tseg_1          = t.tseg1;
  timing.tseg_2          = t.tseg2;
  timing.sjw             = t.sjw;
  timing.triple_sampling = t.tripleSampling;

  filter.acceptance_code = p.bus.acceptCode;
  filter.acceptance_mask = p.bus.acceptMask;
  filter.single_filter   = true;

  if (reason) *reason = nullptr;
  return true;
}

bool startFromProfile(const CanBusProfile &p, const char **reason) {
  twai_general_config_t g;
  twai_timing_config_t  t;
  twai_filter_config_t  f;
  if (!buildTwaiConfigs(p, g, t, f, reason)) return false;

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    if (reason) *reason = "twai_driver_install failed";
    return false;
  }
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    if (reason) *reason = "twai_start failed";
    return false;
  }
  return true;
}

bool stopBus() {
  twai_stop();
  return twai_driver_uninstall() == ESP_OK;
}

}  // namespace can

#endif
