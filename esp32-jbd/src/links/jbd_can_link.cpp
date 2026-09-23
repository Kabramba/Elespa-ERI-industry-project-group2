#include "jbd_can_link.h"

namespace jbd {

bool JbdCanLink::begin() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(cfg_.txPin), static_cast<gpio_num_t>(cfg_.rxPin), TWAI_MODE_NORMAL);
  g.rx_queue_len = 32;
  g.tx_queue_len = 4;

  // TWAI_ALERT_TX_FAILED only fires for single-shot transmissions. It is
  // enabled either way so the bit means the same thing in both modes; it simply
  // never arrives when the controller is retransmitting. ARB_LOST is the useful
  // one in hardware mode, where a lost arbitration is retried silently.
  g.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ARB_LOST |
                     TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF |
                     TWAI_ALERT_BUS_RECOVERED;

  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }
  up_ = true;
  return true;
}

// Bus-off recovery, plus the hardware-mode escape hatch.
void JbdCanLink::maintain() {
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) return;

  busOff_ = (st.state == TWAI_STATE_BUS_OFF);
  if (busOff_) {
    twai_initiate_recovery();
    delay(60);
    if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_STOPPED) twai_start();
    return;
  }

  // Single shot self-limits: a frame is attempted once, TEC rises by 8, done.
  // Hardware retransmission does not - the controller keeps retrying the same
  // frame, so a bus that never ACKs walks TEC straight to bus-off. Cycling the
  // peripheral is the only way to abandon a frame already in flight; ESP-IDF
  // exposes no abort for it.
  if (cfg_.hardwareRetransmit && cfg_.hardwareRecoverTec > 0 &&
      st.tx_error_counter >= cfg_.hardwareRecoverTec) {
    twai_clear_transmit_queue();
    twai_stop();
    twai_start();
  }

  if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_STOPPED) twai_start();
}

bool JbdCanLink::request(uint32_t id, uint8_t dlc, twai_message_t &reply) {
  const uint8_t tries = cfg_.hardwareRetransmit ? cfg_.retriesHardware : cfg_.retries;
  lastTxUnacked_      = false;

  for (uint8_t attempt = 0; attempt < tries; ++attempt) {
    ++stats_.attempts;

    uint32_t alerts = 0;
    twai_read_alerts(&alerts, 0);  // clear stale

    twai_message_t q = {};
    q.identifier     = id;
    q.rtr            = 1;  // remote frame = read request; never a data frame

    // THE toggle. ss = 1 puts the frame on the wire exactly once and reports
    // the outcome per frame. ss = 0 hands retrying to the controller, which is
    // how CAN is normally run - at the cost of that per-frame visibility.
    q.ss               = cfg_.hardwareRetransmit ? 0 : 1;
    q.data_length_code = dlc;

    if (twai_transmit(&q, pdMS_TO_TICKS(50)) != ESP_OK) {
      // In hardware mode this usually means the queue is still backed up behind
      // a frame the controller is grinding on.
      ++stats_.failures;
      if (cfg_.hardwareRetransmit) twai_clear_transmit_queue();
      delay(5);
      continue;
    }

    alerts = 0;
    twai_read_alerts(&alerts, pdMS_TO_TICKS(15));
    if (!cfg_.hardwareRetransmit && (alerts & TWAI_ALERT_TX_FAILED)) lastTxUnacked_ = true;

    const uint32_t deadline = millis() + cfg_.replyTimeoutMs;
    while (static_cast<int32_t>(deadline - millis()) > 0) {
      twai_message_t m;
      if (twai_receive(&m, pdMS_TO_TICKS(5)) != ESP_OK) continue;
      if (m.rtr || m.identifier != id) continue;
      if (!canCrcOk(m.data, m.data_length_code)) {
        ++stats_.crcErrors;
        continue;
      }
      reply = m;
      ++stats_.successes;
      return true;
    }

    ++stats_.failures;
    // Hardware mode: the controller is still retrying this request and will sit
    // on the bus until it lands. Drop what is queued so the next id can go out.
    // This cannot cancel the frame already being transmitted.
    if (cfg_.hardwareRetransmit) twai_clear_transmit_queue();

    maintain();
    delay(8);
  }
  return false;
}

bool JbdCanLink::poll(BmsSnapshot &out) {
  if (!up_) return false;
  maintain();

  BmsSnapshot    s = {};
  twai_message_t m;

  // 0x104 first: it carries the cell and NTC counts that size everything else,
  // and doubles as a link check.
  if (!request(kCanConfig, 4, m)) return false;
  decodeCanFrame(kCanConfig, m.data, m.data_length_code, s);

  if (!request(kCanPack, 8, m)) return false;
  decodeCanFrame(kCanPack, m.data, m.data_length_code, s);

  // The rest are best effort: a dropped frame leaves its group unflagged in
  // s.fields rather than failing the whole poll.
  if (request(kCanCapacity, 8, m)) decodeCanFrame(kCanCapacity, m.data, m.data_length_code, s);
  if (request(kCanBalProt, 8, m))  decodeCanFrame(kCanBalProt, m.data, m.data_length_code, s);
  if (request(kCanMosDate, 8, m))  decodeCanFrame(kCanMosDate, m.data, m.data_length_code, s);
  if (request(kCanNtc1, 8, m))     decodeCanFrame(kCanNtc1, m.data, m.data_length_code, s);
  if (s.ntcCount > 3 && request(kCanNtc4, 8, m))
    decodeCanFrame(kCanNtc4, m.data, m.data_length_code, s);

  const uint8_t frames = canCellFrameCount(s.cellCount);
  for (uint8_t i = 0; i < frames && (kCanCellFirst + i) <= kCanCellLast; ++i) {
    const uint32_t id = kCanCellFirst + i;
    if (request(id, 8, m)) decodeCanFrame(id, m.data, m.data_length_code, s);
  }

  s.source   = Source::kCan;
  s.valid    = true;
  s.uptimeMs = millis();
  computeDerived(s);
  out = s;
  return true;
}

void JbdCanLink::stats(LinkStats &out) const {
  // NOTE: in hardware-retransmit mode `failures` counts reply timeouts, not
  // un-ACKed frames - the controller hides those. The two modes' failure counts
  // are not directly comparable.
  out = stats_;
  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK) {
    out.busErrors      = st.bus_error_count;
    out.txErrorCounter = st.tx_error_counter;
  }
}

}  // namespace jbd
