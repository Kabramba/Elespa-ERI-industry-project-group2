#include "telemetry/output.h"

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>

#include "telemetry/stream_writer.h"

namespace app {
namespace {

// Namespace-scope, so all of this is in .bss: no allocation, nothing that
// survives a reset, exactly like the snapshot rings.
TelemetryQueue                g_queue;
telemetry::QueueSink<kTelemetryQueueDepth> g_queueSink(g_queue);

uint32_t clockMs() { return millis(); }
telemetry::Publisher g_publisher(g_queueSink, clockMs);

#if TELEMETRY_OUTPUT_UART1
// Its own UART, so the USB port stays free for human-readable output.
HardwareSerial       g_uart(1);
telemetry::StreamByteWriter g_writer(g_uart);
#else
// Shares the USB port. The port then carries binary frames.
telemetry::StreamByteWriter g_writer(Serial);
#endif

telemetry::CobsFrameSink g_cobs(g_writer);

}  // namespace

void beginTelemetryOutput() {
#if TELEMETRY_OUTPUT_UART1
  g_uart.begin(TELEMETRY_UART_BAUD, SERIAL_8N1, TELEMETRY_UART_RX_PIN, TELEMETRY_UART_TX_PIN);
#endif
  g_queue.clear();
}

telemetry::Publisher &telemetryPublisher() { return g_publisher; }

const TelemetryQueue &telemetryQueue() { return g_queue; }

uint32_t telemetryFramesWritten() { return g_cobs.framesWritten(); }
uint32_t telemetryWriteFailures() { return g_cobs.shortWrites(); }

size_t pumpTelemetryOutput(size_t maxPackets) {
  size_t  wrote = 0;
  uint8_t packet[telemetry::kMaxPacketBytes];

  for (size_t i = 0; i < maxPackets; ++i) {
    const size_t n = g_queue.pop(packet, sizeof(packet));
    if (n == 0) break;
    if (!g_cobs.onPacket(packet, n)) break;  // transport backed up; try later
    ++wrote;
  }
  return wrote;
}

}  // namespace app

#endif
