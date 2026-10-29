// app_config.h - master compile-time toggles.
//
// Every value here can be overridden from platformio.ini with -D, e.g.
//     build_flags = -D JBD_CAN_HARDWARE_RETRANSMIT=1
// so you can flip behaviour per environment without editing source.
#pragma once

// ===========================================================================
// CAN transmission mode
// ===========================================================================
//
//   0 - SINGLE SHOT (default). Each remote frame is put on the wire exactly
//       once. If it is not ACKed the controller gives up, TWAI_ALERT_TX_FAILED
//       fires, and JbdCanLink retries in software.
//
//   1 - HARDWARE RETRANSMISSION. The conventional way to run CAN: the
//       controller retries on its own until the frame is ACKed or the node
//       reaches bus-off.
//
// What actually changes when this is 1 - all of it handled in JbdCanLink:
//
//   twai_message_t::ss      1 -> 0. This is the flag that switches the mode.
//
//   TWAI_ALERT_TX_FAILED    STOPS FIRING. ESP-IDF raises it only for single
//                           shot transmissions, so per-frame failure reporting
//                           is gone. Health must be read from TEC, bus_error_count
//                           and tx_failed_count instead.
//
//   software retries        Drop from `retries` to `retriesHardware`. Retrying
//                           in software while the controller is also retrying
//                           just queues duplicates. The remaining retries exist
//                           only for the other failure mode: the frame WAS
//                           ACKed but the BMS never answered.
//
//   TX queue                A frame that is never ACKed blocks the queue while
//                           the controller retries it. After a reply timeout we
//                           call twai_clear_transmit_queue() so the next request
//                           can go out. NOTE: this drops queued frames but
//                           cannot abort the one already in flight - ESP-IDF has
//                           no such API.
//
//   bus-off exposure        Higher. Endless retries drive TEC up quickly. Above
//                           `hardwareRecoverTec` the link stops and restarts the
//                           peripheral as a bounded escape hatch.
//
//   stats meaning           stats.failures counts reply timeouts, not un-ACKed
//                           frames. Comparing the two modes' numbers directly is
//                           not meaningful.
//
// MEASURED 2026-09-23 on the current (degraded) bench bus, same firmware, same
// session, only this flag differing:
//
//     single shot            0 - 27 % of frames won,  TEC 32
//     hardware retransmit    86 - 87 % sustained,     TEC 114-169, no bus-off
//
// Over 70 s / 506 attempts the hardware-retransmit TEC oscillated in a stable
// equilibrium - rising on failure, falling on success, exactly as the CAN error
// counters are meant to behave - and never approached bus-off at 256. The ACK
// on this bus is missed intermittently rather than never, so retrying rides
// straight through it while single shot throws the frame away.
//
// So 1 is the default: it is both the conventional way to run CAN and, here,
// measurably the better one. Build env `esp32s3_canss` flips it to 0 when you
// want per-frame visibility for diagnosis - that is what single shot is for.
#ifndef JBD_CAN_HARDWARE_RETRANSMIT
#define JBD_CAN_HARDWARE_RETRANSMIT 1
#endif

// ===========================================================================
// Status LED
// ===========================================================================
// The DevKitC-1's WS2812 is uncomfortably bright at full scale, and the Arduino
// core's own default (RGB_BRIGHTNESS 64) is still 25 %. Percent of full scale.
#ifndef STATUS_LED_BRIGHTNESS_PERCENT
#define STATUS_LED_BRIGHTNESS_PERCENT 8
#endif

// Set to 0 to compile the LED out entirely (the API stays, it just does nothing).
#ifndef STATUS_LED_ENABLED
#define STATUS_LED_ENABLED 1
#endif

// ===========================================================================
// Telemetry packet output
// ===========================================================================
//
// Where encoded packets go. The schema itself is transport-agnostic - see
// telemetry/packet.h - so this only picks the pipe.
//
//   0  USB-COBS. Binary frames on the same port as the monitor. Needs no
//      wiring, which is what makes a bare-board bring-up possible, but the
//      port then carries binary and no text may share it.
//
//   1  UART1-COBS. Binary frames on their own pins. THIS IS THE DELIVERABLE:
//      packets leave on a dedicated link and the USB port stays free for
//      human-readable diagnostics.
//
//   2  USB-HEX. One text line per packet, "T <hex>". Costs twice the bytes and
//      is the only mode that COEXISTS with the text protocols the live apps
//      already emit, so the dashboard keeps working and packets are visible on
//      the single USB cable. Use it for bring-up on the vehicle.
#define TELEMETRY_OUT_USB_COBS   0
#define TELEMETRY_OUT_UART1_COBS 1
#define TELEMETRY_OUT_USB_HEX    2

#ifndef TELEMETRY_OUTPUT
#define TELEMETRY_OUTPUT TELEMETRY_OUT_USB_COBS
#endif

// GPIO15/16 are free on the DevKitC-1: 17/18 are the JBD UART, 4/5 the CAN
// transceiver, 43/44 the USB bridge, 48 the status LED.
#ifndef TELEMETRY_UART_TX_PIN
#define TELEMETRY_UART_TX_PIN 15
#endif
#ifndef TELEMETRY_UART_RX_PIN
#define TELEMETRY_UART_RX_PIN 16
#endif

// 115200 keeps a 324-byte packet under 30 ms on the wire. The largest packet at
// 1 Hz uses about 2.3 % of the link, so there is ample headroom for the whole
// seven-packet set plus the partner group's 200 Hz channels arriving as
// StatBlocks. In hex mode double it: still under 5 %.
#ifndef TELEMETRY_UART_BAUD
#define TELEMETRY_UART_BAUD 115200
#endif

// Depth of the producer-to-transmitter queue, in packets. Eight costs about
// 2.6 kB of SRAM.
#ifndef TELEMETRY_QUEUE_DEPTH
#define TELEMETRY_QUEUE_DEPTH 8
#endif
