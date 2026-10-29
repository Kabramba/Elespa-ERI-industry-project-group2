#include "telemetry/output.h"

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>

#include "telemetry/stream_writer.h"

namespace app {
namespace {

// Namespace-scope, so all of this is in .bss: no allocation, nothing that
// survives a reset, exactly like the snapshot rings.
TelemetryQueue                             g_queue;
telemetry::QueueSink<kTelemetryQueueDepth> g_queueSink(g_queue);

uint32_t clockMs() { return millis(); }
telemetry::Publisher g_publisher(g_queueSink, clockMs);

#if TELEMETRY_OUTPUT == TELEMETRY_OUT_UART1_COBS
// Its own UART, so the USB port stays free for human-readable output.
HardwareSerial              g_uart(1);
telemetry::StreamByteWriter g_writer(g_uart);
telemetry::CobsFrameSink    g_sink(g_writer);
#elif TELEMETRY_OUTPUT == TELEMETRY_OUT_USB_HEX
// Text lines, so the existing line-oriented output can share the port.
telemetry::StreamByteWriter g_writer(Serial);
telemetry::HexLinePacketSink g_sink(g_writer);
#else
// Binary frames on the USB port. Nothing else may print there.
telemetry::StreamByteWriter g_writer(Serial);
telemetry::CobsFrameSink    g_sink(g_writer);
#endif

}  // namespace

void beginTelemetryOutput() {
#if TELEMETRY_OUTPUT == TELEMETRY_OUT_UART1_COBS
  g_uart.begin(TELEMETRY_UART_BAUD, SERIAL_8N1, TELEMETRY_UART_RX_PIN, TELEMETRY_UART_TX_PIN);
#endif
  g_queue.clear();
}

telemetry::Publisher &telemetryPublisher() { return g_publisher; }

const TelemetryQueue &telemetryQueue() { return g_queue; }

uint32_t telemetryFramesWritten() {
#if TELEMETRY_OUTPUT == TELEMETRY_OUT_USB_HEX
  return g_sink.linesWritten();
#else
  return g_sink.framesWritten();
#endif
}

uint32_t telemetryWriteFailures() { return g_sink.shortWrites(); }

const char *telemetryOutputName() {
#if TELEMETRY_OUTPUT == TELEMETRY_OUT_UART1_COBS
  return "uart1-cobs";
#elif TELEMETRY_OUTPUT == TELEMETRY_OUT_USB_HEX
  return "usb-hex";
#else
  return "usb-cobs";
#endif
}

bool telemetryOutputIsBinaryOnUsb() {
  return TELEMETRY_OUTPUT == TELEMETRY_OUT_USB_COBS;
}

size_t pumpTelemetryOutput(size_t maxPackets) {
  size_t  wrote = 0;
  uint8_t packet[telemetry::kMaxPacketBytes];

  for (size_t i = 0; i < maxPackets; ++i) {
    const size_t n = g_queue.pop(packet, sizeof(packet));
    if (n == 0) break;
    if (!g_sink.onPacket(packet, n)) break;  // transport backed up; try later
    ++wrote;
  }
  return wrote;
}

}  // namespace app

#endif
