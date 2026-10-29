# Elespa ERI — CAN interfacing

ESP32-S3 firmware for talking to CAN devices. Each device is described by a
**bus profile** carrying everything needed to bring its bus up plus metadata
about what is on the other end; the JBD BMS is the first worked example, with a
Daly BMS and a 2024 Activa's OBD-II port to follow.

Telemetry from any link lands in a RAM-only ring buffer that the rest of the
firmware reads through a const handle, and is published as one of **seven
versioned packet types** — see [Telemetry packet schema](#telemetry-packet-schema).

Grew out of the JBD bring-up; the frozen, hardware-validated Arduino sketches
from that phase are preserved verbatim in
[`reference/safe-save-copy-esp32/`](reference/safe-save-copy-esp32/).

---

## Getting set up in VS Code

1. Install the **PlatformIO IDE** extension (`platformio.platformio-ide`) from
   the marketplace.
2. Open **this folder** — `platformio.ini` sits at the repo root.
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
  can/
    can_timing.h         bitrate -> register arithmetic (pure)
    can_bus_profile.h    the profile structure + validation + TX allowlist
    can_profile_registry.h  lookup into the static table
    active_profile.h     which profile this build is running
    twai_profile.h       profile -> ESP32 TWAI registers (the only HW file)
  obd/
    obd_isotp.h          ISO-TP framing + THE REQUEST ALLOWLIST
    vehicle_types.h      VehicleSnapshot - decoded PIDs plus raw samples
  telemetry/
    packets.h            THE PUBLISHED SCHEMA - seven payload structs
    packet.h             the 16-byte envelope: magic, version, seq, CRC
    stat_block.h         StatBlock + Welford running statistics
    framing.h            COBS + the resynchronising stream reader
    publisher.h          THE API - publish(), the sinks, the fan-out
    packet_queue.h       the producer-to-transmitter hop
    mappers.h            acquisition types -> schema payloads
    output.h             the assembled output chain, per app_config.h
    byte_writer.h        IByteWriter, the seam that makes it host-testable
    stream_writer.h      the Arduino adapter (the only untestable part)
    battery_packet_sink.h  an IBmsSink that publishes packets
  bms_types.h            BmsSnapshot - the battery data model
  bms_link.h             IBmsLink    - a source of telemetry
  bms_sink.h             IBmsSink    - a consumer of telemetry
  snapshot_ring.h        util::SnapshotRing<T,N>, the IPC ring (header-only)
  platform_lock.h        the only file that knows about FreeRTOS
  crc16.h                CRC-16/MODBUS, shared by JBD framing and the envelope
  jbd_protocol.h         pure encode/decode
  jbd_uart_link.h        UART transport
  jbd_can_link.h         CAN transport
  telemetry.h            the two rings' accessors
  serial_sink.h          human-readable output
  ring_sink.h            publishes into the ring
  status_led.h           prioritised WS2812 status indicator
  summary_consumer.h     worked example of reading the ring

src/
  apps/                  one main() each, selected by build_src_filter
    main_jbd.cpp           JBD BMS over UART + CAN
    main_activa_probe.cpp  LISTEN ONLY bitrate probe
    main_activa_obd.cpp    OBD-II reader, machine-readable output
    main_telemetry_selftest.cpp  the packet pipeline, no hardware needed
  bms_types.cpp          derived statistics, naming
  jbd_protocol.cpp       pure decode        <- no Arduino, host-testable
  crc16.cpp              pure, with a continuation form for split buffers
  telemetry.cpp          owns the ring instances
  status_led.cpp         LED arbitration + the named states
  can/                   timing, profile utilities, THE PROFILE TABLE
  obd/                   ISO-TP framing, PID decode
  telemetry/             envelope encode/verify, running statistics
  links/                 jbd_uart_link.cpp, jbd_can_link.cpp
  sinks/                 serial_sink.cpp
  consumers/             summary_consumer.cpp

test/
  test_protocol/         decode tests, fixtures captured off the real pack
  test_ring/             ring overflow, ordering, both read paths
  test_status_led/       LED priority, expiry, blink, token safety
  test_can_profile/      timing solver, validation rules, registry
  test_obd/              ISO-TP reassembly, the request allowlist
  test_telemetry/        envelope round-trip, CRC rejection, statistics
  test_framing/          COBS, the 254-byte boundary, resynchronisation
  test_publisher/        sequencing, queue overflow, the whole path
  test_mappers/          acquisition -> schema, absent vs zero, edges

tools/
  telemetry_decode.py    the reference host consumer
  check_decoder.py       C++ <-> Python contract test
  make_golden_capture.cpp  writes the fixture from the real structs
  activa_dashboard.py    the web dashboard
```

Plus `docs/` (bring-up notes) and `reference/` (frozen sketches, superseded
files). `tools/serial_read.py` still does bounded non-interactive captures.

All nine suites run on the host: **113 tests in about four seconds, no board.**

## How it fits together

`IBmsLink::poll()` produces a `BmsSnapshot`. `IBmsSink::onSnapshot()` consumes
one. The application in `src/apps/` does nothing but connect them:

```
JbdUartLink ─┐                    ┌─ RingSink ──> SnapshotRing<16> ─> any consumer
             ├─> BmsSnapshot ────>┤
JbdCanLink  ─┘                    └─ SerialSink
```

Adding a transport means one more `IBmsLink`. Adding a consumer means one more
`IBmsSink`. Neither touches the protocol layer, and the application changes by
one array entry.

## Telemetry packet schema

The project's deliverable. Four heterogeneous sources — the Activa engine ECU,
a BMS (JBD or Daly), the Votol EM-30S motor driver, and the partner group's
sensor board — are normalised into **seven packet types**, grouped by *how fast
their fields change*, because that is what sets how often each must be sent.

| Type | Value | Rate | Payload bytes |
|---|---|---|---|
| `kIdentity` | `0x01` | on change / 60 s keepalive | 112 |
| `kEngine` | `0x10` | 1 Hz | 184 |
| `kMotor` | `0x11` | 1 Hz | 152 |
| `kDynamics` | `0x12` | 1 Hz | 308 |
| `kNavigation` | `0x13` | 1 Hz | 128 |
| `kBattery` | `0x20` | 1 Hz | 128 |
| `kStatusEvent` | `0x30` | on change | 64 |

Every packet travels behind a 16-byte envelope — magic `0xE15A`, schema
version, type, length, per-source sequence, producer uptime, source id, flags,
CRC-16/MODBUS. **The transport is deliberately out of scope:** these are byte
layouts, so UART, Wi-Fi, LoRa or shared memory all carry the same bytes.

```cpp
uint8_t buf[telemetry::kHeaderBytes + sizeof(telemetry::BatteryTelemetry)];
size_t n = telemetry::encode(telemetry::PacketType::kBattery,
                             telemetry::SourceId::kBatteryA,
                             seq++, millis(), 0, pkt, buf, sizeof buf);
// on the consumer
if (telemetry::verify(buf, n)) { /* ... */ }
```

Three rules the schema does not bend on:

- **Wire values are frozen once published.** Append, never renumber.
- **Every payload carries a `fields` mask.** A dropped reading is never
  presented as a zero.
- **Fast channels arrive as `StatBlock`s,** not bare samples: min, max, mean,
  stddev, last, slope, window, count — 32 bytes that describe a second of a
  200 Hz accelerometer without shipping 200 samples. Statistics are accumulated
  with Welford's algorithm, and an absent channel reports `NaN` with
  `samples == 0` so it can never be mistaken for a plausible zero.

### Getting a packet onto a wire

A magic number cannot delimit a binary stream — a payload will eventually
contain `5A E1` by chance — so frames are **COBS**-encoded, which removes every
zero byte and makes `0x00` an unambiguous delimiter. Overhead is 3 bytes on the
largest packet (0.9 %), and resynchronisation after corruption costs at most one
frame *by construction*.

Three output modes, set with `-D TELEMETRY_OUTPUT=` per environment:

| Mode | Wire | Use |
|---|---|---|
| `TELEMETRY_OUT_UART1_COBS` | binary on GPIO15 | **the deliverable** — own link, USB left free |
| `TELEMETRY_OUT_USB_COBS` | binary on USB | bare-board bring-up, no text may share the port |
| `TELEMETRY_OUT_USB_HEX` | `T <hex>` lines on USB | on the vehicle — the only mode that coexists with the dashboard |

### The API

```cpp
app::beginTelemetryOutput();                      // once, in setup()

telemetry::BatteryTelemetry b = {};
telemetry::toBatteryTelemetry(snapshot, b);       // pure, host-tested mapper
app::telemetryPublisher().publish(
    telemetry::PacketType::kBattery,
    telemetry::SourceId::kBatteryA, b);

app::pumpTelemetryOutput(2);                      // every loop() iteration
```

No application builds a header by hand. `Publisher` owns the per-source
sequence counters, the clock and the CRC; `publish()` only enqueues, because a
324-byte write at 115200 baud takes 28 ms and an OBD cycle that stalls that long
starts losing PIDs.

### Proving it with nothing attached

Every other app needs a BMS or a running vehicle on the other end, which makes
the transport hard to bring up: a silent link could be the wiring, the ECU, the
protocol or the packet layer.

```bash
pio run -e esp32s3_selftest -t upload     # safe on any bus: never opens a CAN controller
tools/telemetry_decode.py /dev/ttyACM0
```

```
  17.044  engine     ecu            #16  [simulated] rpm 1502 (min 1480 max 1520 sd 2.50 n=8)  map 70.50 kPa
  17.044  battery    battery_a      #17  [simulated] 53.40 V  -2.20 A  soc 42%  spread 15 mV  cells 15/17
  17.046  dynamics   sensor_group_1 #33  [simulated] ax 0.020 az 1.000 g  n=40  window 200 ms
```

Measured on a bare board: **101 packets, 0 framing errors, 0 CRC rejects, 0
sequence gaps** over 20 s. Every packet carries `kFlagSimulated` — the values
come from a sine wave, not a vehicle.

### Reading them on a host

```bash
tools/telemetry_decode.py /dev/ttyACM0 --json      # one object per line
tools/telemetry_decode.py /dev/ttyACM0 --types battery
tools/check_decoder.py                             # C++ <-> Python contract test
```

`tools/telemetry_decode.py` states the schema a second time, in Python. Two
descriptions of the same bytes drift silently, so `check_decoder.py` generates a
capture from the **real C++ structs** and asserts the decoder reports those
exact values back. It caught two real bugs on its first run.

Full field-by-field reference, the wire format, the API and the rate budget:
**[`docs/TELEMETRY_SCHEMA.md`](docs/TELEMETRY_SCHEMA.md)**. The 17 ECU values
and their units are in [`docs/ECU_schema.md`](docs/ECU_schema.md).

## The telemetry rings

There are **two independent buffers**, because the battery and the vehicle
produce different shapes of data at different rates and a stall in one must not
displace the other:

| Accessor | Payload | Depth |
|---|---|---|
| `app::telemetry()` | `jbd::BmsSnapshot` | 16 |
| `app::vehicleTelemetry()` | `obd::VehicleSnapshot` | 8 |

Both are `util::SnapshotRing<T, N>` — the same container, generic over payload.
`platform_lock.h` and `snapshot_ring.h` live in `namespace util` because neither
is specific to a battery.

A `VehicleSnapshot` keeps **raw PID samples alongside the decoded fields**. An
ECU's field set is not fixed — which PIDs exist varies by vehicle — so nothing
the ECU said is discarded just because we lack a formula for it. Use
`obd::hasPid()` before trusting a decoded value: a PID that never answered is
not the same as a reading of zero.

### Access, storage and overflow

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

## Live dashboard

```bash
./dash esp          # resolve the board by its USB id, whatever ttyACMn it landed on
./dash 0            # /dev/ttyACM0
./dash ACM1         # /dev/ttyACM1
./dash esp -l run.log
```

It opens a browser at `http://127.0.0.1:8477/`. `./dash` is a thin wrapper over
`tools/activa_dashboard.py`; the port stays an explicit argument, it just saves
typing, and with no arguments it lists the ports that exist right now.
`--http-port N` moves the web port, `--no-browser` suppresses the auto-open,
`-l FILE` appends the raw stream to a log. Ctrl-C stops it.

**Reflashing while the dashboard is open:** click **release port**. The
dashboard holds the serial port exclusively, so `pio run -t upload` otherwise
fails with *"multiple access on port"* — Linux does not lock a tty, so esptool
and the dashboard both open it and race for the bootloader handshake. The button
drops the port for 25 seconds and reconnects by itself; no restart needed.

The page shows a card per PID with the live value, its rate of change over a
one-second window, and a chart of recent history, plus the raw serial stream and
link statistics. Charts are drawn on a canvas by hand and nothing is fetched
from a CDN, so it works with no internet.

Dependencies live in the project venv:

```bash
.venv/bin/pip install -r tools/requirements.txt
```

Division of labour: **the firmware emits frames, the dashboard decodes them.**
`esp32s3_activa_obd` prints only two line shapes —

```
# KEY value...                       metadata
R (12.345678) 18DAF110#03410C1A2C    a frame; T transmitted, R received
```

— and Python does ISO-TP reassembly, PID decoding, deltas, sparklines and the
raw view. Strip the direction character and the frame lines are candump text,
so a capture pastes straight into SavvyCAN or python-can.

It requires `# MODE activa-obd` and says so plainly rather than silently
misreading another application's output.

## CAN bus profiles

A `CanBusProfile` is everything needed to bring up one device's bus, plus who is
on the other end. Pure data with pure helpers — no TWAI, no Arduino — so all of
it tests on the host. `can/twai_profile.h` is the only file that touches
hardware.

```cpp
const can::CanBusProfile *p = can::findProfile("jbd-sp24s");
const char *why = nullptr;
if (can::startFromProfile(*p, &why)) { /* bus is up */ }
```

The field that actually generalises is **`Interaction`**. Devices differ far
more in *how* you get data out of them than in bitrate or pins:

| Interaction | Meaning | Device |
|---|---|---|
| `kRemoteFrameQuery` | RTR out, data frame back on the same id | JBD BMS |
| `kDataFrameRequest` | data frame out, reply on a different id | Daly BMS |
| `kBroadcastListen` | device transmits unprompted; host only listens | sniffing |
| `kIsoTp` | ISO 15765-2 segmented transport with flow control | OBD-II / UDS |

A profile carrying only bitrate and an id list handles the first and breaks on
the last, because ISO-TP needs a segmentation layer underneath. Hence the
transport lives in the profile.

**Bit timing is solved, not hard-coded.** `computeBitTiming()` derives
`brp`/`tseg1`/`tseg2`/`sjw` for any exact bitrate and sample point, so a profile
can name 250 k or 800 k without anyone writing another
`TWAI_TIMING_CONFIG_*`. It refuses inexact bitrates rather than rounding — an
approximate rate desynchronises a bus instead of degrading. The tests pin it
against ESP-IDF's own canonical 125 k/250 k/500 k/1 M configs.

**Two independent safety gates.** `AccessPolicy` is intent, `BusMode` is what
the controller physically does, and `allowsTransmit()` requires both. A
`kBroadcastListen` profile that is permitted to transmit fails validation
outright — that combination would put unexpected frames on somebody else's bus.

**`Provenance`** records how much to trust each entry: `kMeasured`,
`kDatasheet`, `kVendorDoc`, `kUnverified`. Do not mark something measured
because a forum post said so.

### The table

All profiles live in one `constexpr` array in
[`src/can/can_profile_registry.cpp`](src/can/can_profile_registry.cpp) — no RAM,
no persistence, everything visible in one file. Adding a device is one entry.

| Key | Device | Interaction | Provenance |
|---|---|---|---|
| `jbd-sp24s` | JBD SP24S007 BMS | remote-frame query | **measured** |
| `activa-listen` | Honda Activa 6G ECU | broadcast listen | unverified |
| `activa-obd11` | Honda Activa 6G ECU | ISO-TP, 11-bit | unverified |
| `activa-obd29` | Honda Activa 6G ECU | ISO-TP, 29-bit | unverified |
| `sniff-500k` / `-250k` / `-1m` / `-125k` | any | broadcast listen | measured |

**Every profile is compiled in unconditionally.** Nothing is `#ifdef`'d out, so
a DIP switch, a serial command or a UI can switch devices at runtime without
reflashing — see `can/active_profile.h`. The whole table costs about 5 KB of a
6.5 MB app partition.

The listen-only entries transmit **nothing at all** — not even an ACK — which
makes them the only safe way onto a bus you do not own. `validateAll()` checks
every entry and runs at boot.

### Honda Activa 6G — read this before connecting

Measured facts are in [`docs/activa-can.md`](docs/activa-can.md). The short
version:

- **CAN-H = Yellow/Black, CAN-L = Yellow/Red, GND = Green** on the 6-pin red
  DLC. Polarity was deduced from ISO 15031-3, not confirmed — if a bitrate that
  should work yields only error frames, swap the two yellow wires.
- **Never connect Brown** (SCS — grounding it puts the ECU into service mode)
  **or Orange** (K-line at ~12 V, would destroy a GPIO).
- CAN-H↔CAN-L measures **124 Ω** with all nodes connected, so the vehicle has
  only one terminator and the DLC is the open end. **The SN65HVD230's onboard
  120 Ω stays fitted** — the profiles flag this as `expectsExternalTermination`.
- The **bitrate is still unknown**, which is exactly why all three Activa
  profiles are marked `kUnverified` rather than `kMeasured`. Start with
  `activa-listen`, then the other sniffers in order 250 k → 1 M → 125 k.

The transmit rules from that document are enforced in data, not convention.
`activa-obd11` permits `0x7DF` and `0x7E0–0x7E7` and nothing else;
`activa-obd29` uses a mask so only `0x18DAxxF1` and `0x18DB33F1` pass. Every
transmit path must go through `mayTransmitId()`, which fails closed three
separate ways: no transmit permission, empty allowlist, or no matching rule.

### Not built yet

- **ISO-TP transport.** The Activa profiles describe it; nothing speaks it. A
  segmentation layer (First/Consecutive/Flow Control) is needed before either
  OBD profile can do anything.
- **Bitrate probe.** Listen at each candidate rate, count valid frames versus
  bus errors. Safe to build — it never transmits.
- **Daly BMS.** Absent rather than guessed: it needs a reference document like
  the Activa's, with the real bitrate, frame format and request/response ids.

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
