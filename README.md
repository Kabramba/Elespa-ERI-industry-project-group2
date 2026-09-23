# esp32-jbd

ESP32-S3 telemetry bridge for a JBD (Jiabaida) BMS. Reads the pack over UART
and CAN, cross-checks the two, and publishes each reading into a RAM-only ring
buffer that any other part of the firmware can read.

Known-good Arduino sketches this was derived from live in
[`../safe-save-copy-esp32/`](../safe-save-copy-esp32/). They are frozen; this is
where new work happens.

---

## Getting set up in VS Code

1. Install the **PlatformIO IDE** extension (`platformio.platformio-ide`) from
   the marketplace.
2. Open **this folder** (`esp32-jbd`), not its parent. PlatformIO keys off
   `platformio.ini` at the root.
3. The toolchain is already downloaded under `~/.platformio`, so the extension
   will not re-fetch ~500 MB.

You then get build/upload/monitor buttons in the status bar, ESP-IDF
IntelliSense (hover `twai_transmit`, F12 into the driver), and the test panel.
Your existing `ms-vscode.cpptools` install is what PlatformIO drives for
IntelliSense.

A project-local CLI also exists at `.venv/bin/pio` if you prefer the terminal —
same project files, same builds, and it is what these commands use.

```bash
.venv/bin/pio run                 # build
.venv/bin/pio run -t upload       # flash
.venv/bin/pio device monitor      # serial monitor
.venv/bin/pio test -e native      # run the tests on THIS machine, no hardware
.venv/bin/pio run -t compiledb    # refresh compile_commands.json for clangd/nvim
```

## Layout

```
include/                 public headers
  app_config.h           MASTER TOGGLES - CAN mode, LED brightness
  bms_types.h            BmsSnapshot - the shared data model
  bms_link.h             IBmsLink    - a source of telemetry
  bms_sink.h             IBmsSink    - a consumer of telemetry
  bms_ring.h             the IPC ring buffer (header-only template)
  platform_lock.h        the only file that knows about FreeRTOS
  jbd_protocol.h         pure encode/decode
  jbd_uart_link.h        UART transport
  jbd_can_link.h         CAN transport
  telemetry.h            the shared buffer's accessors
  serial_sink.h          human-readable output
  ring_sink.h            publishes into the ring
  status_led.h           prioritised WS2812 status indicator
  summary_consumer.h     worked example of reading the ring

src/
  bms_types.cpp          derived statistics, naming
  jbd_protocol.cpp       pure decode        <- no Arduino, host-testable
  telemetry.cpp          owns the one ring instance
  status_led.cpp         LED arbitration + the named states
  links/                 jbd_uart_link.cpp, jbd_can_link.cpp
  sinks/                 serial_sink.cpp
  consumers/             summary_consumer.cpp
  main.cpp               wiring only

test/
  test_protocol/         decode tests, fixtures captured off the real pack
  test_ring/             ring overflow, ordering, both read paths
  test_status_led/       LED priority, expiry, blink, token safety
```

All three test suites run on the host: **29 tests in about a second, no board.**

## How it fits together

`IBmsLink::poll()` produces a `BmsSnapshot`. `IBmsSink::onSnapshot()` consumes
one. `main.cpp` does nothing but connect them:

```
JbdUartLink ─┐                    ┌─ RingSink ──> BmsRing<16> ──> any consumer
             ├─> BmsSnapshot ────>┤
JbdCanLink  ─┘                    └─ SerialSink
```

Adding a transport means one more `IBmsLink`. Adding a consumer means one more
`IBmsSink`. Neither touches the protocol layer, and `main.cpp` changes by one
array entry.

## The telemetry ring

`app::telemetry()` returns a `const TelemetryRing&` — every method reachable
through it is const, so a consumer cannot publish. `app::telemetryProducer()`
hands out the writable reference and only the poll loop should call it.

**Storage.** The ring is a namespace-scope object: plain RAM (`.bss`/`.data`),
reinitialised by the C runtime on every boot, never written to flash, NVS or
EEPROM. It is deliberately *not* `RTC_DATA_ATTR`/`RTC_NOINIT_ATTR`, which would
survive a reset. Power cycle and the history is gone.

**Overflow.** Fixed capacity, newest wins. Publishing into a full ring discards
the oldest entry and increments `overwrittenCount()`, so a consumer can tell it
fell behind rather than silently missing samples.

**Two read paths:**

```cpp
// Copy out. Always coherent, safe to hold indefinitely. Use this by default.
jbd::BmsSnapshot s;
if (app::telemetry().latest(s)) { /* ... */ }

jbd::BmsSnapshot recent[4];
size_t n = app::telemetry().history(recent, 4);   // oldest-first

// Zero copy. Double-buffered with an atomic index swap, so the pointee is
// stable - but only until TWO further publishes. Read promptly, never stash it.
if (const jbd::BmsSnapshot *now = app::telemetry().latestRef()) { /* ... */ }
```

See `src/consumers/summary_consumer.cpp` for both in use.

## Field validity — read this before trusting a value

CAN on this bench is lossy. A dropped reply leaves its fields at zero, which is
indistinguishable from a genuine zero unless you check. Every snapshot carries
a `fields` bitmask and a `cellsReadMask`:

```cpp
if (jbd::hasFields(s, jbd::kFieldMos)) {
  // safe: 0x103 actually arrived
  use(s.dischargeMosOn);
}
```

Without that check you will eventually read "discharge MOSFET off" from a frame
that never arrived. `computeDerived()` honours `cellsReadMask`, so a partial
cell read reports `cellsRead < cellCount` and its sum/min/max cover only the
channels actually received.

## CAN transmission mode

`JBD_CAN_HARDWARE_RETRANSMIT` in `app_config.h` is the master toggle. Switch
environments in the VS Code status bar rather than editing source:

| Environment | Mode | Use it for |
|---|---|---|
| **`esp32s3`** (default) | hardware retransmission | moving data |
| `esp32s3_canss` | single shot | diagnosing a sick bus |

Measured on the bench bus, same firmware, same session, only the flag differing:

| Mode | Frames won | TEC |
|---|---|---|
| single shot | 0 – 27 % | 32 |
| **hardware retransmit** | **86 – 87 % sustained** | 114 – 169, stable, no bus-off |

Over 70 s / 506 attempts the TEC oscillated in equilibrium - rising on failure,
falling on success, as CAN's counters are designed to - and never approached
bus-off at 256. The ACK here is missed *intermittently* rather than never, so
retrying rides through it while single shot discards the frame.

**What the toggle actually changes**, all inside `JbdCanLink`:

| | single shot | hardware retransmit |
|---|---|---|
| `twai_message_t::ss` | 1 | 0 |
| `TWAI_ALERT_TX_FAILED` | fires per frame | **never fires** |
| software retries | `retries` (4) | `retriesHardware` (2) - only for *no reply*, not *no ACK* |
| TX queue after a timeout | nothing to do | `twai_clear_transmit_queue()`, or a stuck frame blocks the next request |
| bus-off exposure | self-limiting | needs `hardwareRecoverTec` (224) as a bounded escape |
| `stats.failures` means | un-ACKed frames | reply timeouts only - **not comparable between modes** |

Single shot exists because it is the only mode that reports the fate of an
individual frame. That visibility is what identified the swapped TX/RX pins and
the ACK-error signature in the first place; keep it for diagnosis.

There is one limitation worth knowing: `twai_clear_transmit_queue()` drops
*queued* frames but cannot abort the one already in flight. ESP-IDF has no such
API, which is why the TEC threshold cycles the peripheral instead.

## Status LED

The DevKitC-1's WS2812 (GPIO48) is a single shared resource that several
unrelated parts of the firmware want. Unmanaged, the last writer wins and a
routine activity blink paints over a bus-off alarm. `status_led.h` makes every
request a **claim with a priority**; the highest-priority live claim shows.

```cpp
status::begin();                       // blanks the LED, sets brightness
status::show_error_state53();          // CAN bus-off - held until released
status::Token t = status::show_idle(); // held
status::release(t);
status::tick();                        // call every loop() - blink + expiry
```

- **Priority** `kIdle < kInfo < kActivity < kWarning < kError < kCritical`.
  Ties go to the most recent claim. A full claim table never evicts something
  that outranks the arrival.
- **Holds vs flashes.** `holdMs == 0` lasts until released; anything else
  expires on its own, which makes momentary activity blinks fire-and-forget.
- **Mutual exclusion.** `claim()`/`release()` are safe from any task on either
  core. `tick()` is single-task (the main loop): it owns RMT, and the core's
  `neopixelWrite()` is not reentrant. The lock is released *before* the LED is
  written - driving RMT with interrupts masked is asking for trouble.
- **Tokens carry a generation**, so a stale token cannot free a slot somebody
  else has since reused.

**Brightness** is `STATUS_LED_BRIGHTNESS_PERCENT`, default **8 %**. The Arduino
core's own default is `RGB_BRIGHTNESS 64` - still 25 % and genuinely painful at
a desk. `STATUS_LED_ENABLED=0` compiles the LED out while leaving the API.

> Nothing in the firmware previously touched the LED, which is *why* it was
> blinding: a WS2812 latches whatever it saw at power-on and holds it. The first
> thing `status::begin()` does is write black.

Named states are in `status_led.h` - lifecycle, momentary activity, link health,
pack state, and numbered faults `show_error_state50()` through `59()`, each with
a distinct colour and blink rate so a fault is identifiable across a room.

## Hardware

| Item | Value |
|---|---|
| Board | ESP32-S3 QFN56 rev v0.2, **16 MB flash, 8 MB PSRAM**, MAC `dc:b4:d9:38:26:68` |
| Port | `/dev/ttyACM0` (CH343 bridge, `1a86:55d3`) |
| UART | GPIO18 = RX ← BMS TXD · GPIO17 = TX → BMS RXD · 9600 8N1 · connector J8, **VDD unconnected** |
| CAN | **GPIO5 = TX → transceiver D** · **GPIO4 = RX ← transceiver R** · 500 kbit/s |
| BMS | JBD `SP24S007L24S100A`, 17 channels configured, 5 NTC |

> **CAN pin order is the opposite of what `context.md` says.** Established by
> measurement. Do not "fix" it back.

**Board definition caveat.** PlatformIO only ships `esp32-s3-devkitc-1` as the
N8 variant (8 MB, no PSRAM); the real module is 16 MB with 8 MB PSRAM.
`board_upload.flash_size = 16MB` in `platformio.ini` covers flashing. PSRAM is
left disabled because nothing here needs it — enabling it requires the correct
`memory_type` for this exact module, so verify before turning it on.

## Status

| Link | Reliability |
|---|---|
| UART | **100 %** — every poll answered, all 17 channels |
| CAN | **86–87 %** with hardware retransmission, 0 CRC errors |

Both links now return complete, matching snapshots: all 17 cells, MOS state,
protection flags, temperatures and manufacture date, with no missing field
groups.

Switching to hardware retransmission recovered the link, but it did not *fix*
the bus — it rides over a fault that is still there. The underlying ACK loss is
real and got worse across days (~55 % single-shot on 09-21 → 0–27 % by 09-23),
so the physical layer still deserves attention. Two things have never been
measured:

1. **Termination** — CANH–CANL at J6, everything unplugged. ~120 Ω means the
   BMS terminates internally; open means the transceiver module's 120 Ω is
   carrying the bus alone.
2. **Slope control** — if the SN65HVD230 module's Rs/S/STB pin reaches GND
   through a resistor instead of directly, edges are slowed.

Also worth reseating the CANH/CANL flying leads; the decline across days with a
static setup points at a connector.

## Pack note

Channel 10 reads exactly 0 mV and the other 16 sum to the reported pack voltage,
so the pack is **16S** while the BMS is configured for **17**. The BMS reads that
0.000 V as undervoltage, latches `CELL_UV`, and holds the discharge MOSFET off.
Fix is cell count 17 → 16 in JBDTools — a write, deliberately not performed here.
