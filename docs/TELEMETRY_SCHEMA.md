# Telemetry schema v2

The project's spine is **acquisition → preprocessing → telemetry packets**.
Everything downstream — the SBC, the models, whatever comes after — consumes
packets and nothing else. This document is the contract.

Transmission medium is out of scope. These are byte layouts; what carries them
later (UART, Wi-Fi, LoRa, shared memory) changes nothing here.

Code: `include/telemetry/` · `src/telemetry/` · tests in `test/test_telemetry/`

---

## 1. Four sources, seven packets

| Source | Device | Interface | Status |
|---|---|---|---|
| Engine ECU | Honda Activa 6G | OBD-II / ISO-TP over CAN, 500 k, 29-bit | **live** |
| Battery | JBD SP24S007 · Daly DL-R24K | CAN + UART · CAN 250 k, 29-bit | **JBD live**, Daly pending |
| Motor driver | Votol EM-30S | UART or CAN — not yet established | pending |
| Sensor board | partner group 1 | their board, format TBA | simulated data pending |

## 2. Design rules

**Group by rate, not by subject.** A packet holds fields that change at a
similar speed, because that is what sets how often it must be sent. Mixing a
200 Hz accelerometer with a VIN forces the VIN to be retransmitted thousands of
times for nothing, or the accelerometer to arrive useless and stale.

**Fast signals are summarised, never streamed.** An MPU6050 samples in the
hundreds of hertz and jerk is a *second* derivative — raw transmission is out of
the question, and a consumer wants features rather than a waveform. Each fast
channel is accumulated on the device and sent as a `StatBlock` once per window.

**A missing reading is never a zero.** Every payload carries a `fields` bitmask,
and an absent `StatBlock` has `samples == 0` with NaN statistics. On a lossy link
this is the difference between *"the discharge MOSFET is off"* and *"that frame
never arrived"* — a distinction that already cost us a debugging session.

**Discrete state is event-driven, not sampled.** A fault that appears and clears
between two 1 Hz samples would never be seen. `kStatusEvent` is sent on change
and carries an `eventMask` saying what moved.

**Numbers are frozen once published.** `PacketType` values and struct layouts are
append-only. Every struct has a `static_assert` on its size so accidental drift
fails the build rather than the field.

---

## 3. Complete data inventory

Everything the four sources produce. **Status** says whether it exists today.

### 3.1 Engine ECU — 15 data PIDs

| Parameter | PID | Unit | Treatment | Packet | Status |
|---|---|---|---|---|---|
| Engine speed | `0C` | rpm | StatBlock | Engine | **live** |
| Throttle position | `11` | % | StatBlock | Engine | **live** |
| Manifold absolute pressure | `0B` | kPa | StatBlock | Engine | **live** |
| Ignition timing advance | `0E` | ° BTDC | StatBlock | Engine | **live** |
| Calculated engine load | `04` | % | StatBlock | Engine | **live** |
| Intake air temperature | `0F` | 0.1 °C | scalar | Engine | **live** |
| Engine oil temperature | `5C` | 0.1 °C | scalar | Engine | **live** |
| Short-term fuel trim | `06` | % | scalar | Engine | **live** |
| Fuel system status ×2 | `03` | enum | scalar | Engine | **live** |
| Road speed | `0D` | km/h | StatBlock | Navigation | **live** |
| Run time since start | `1F` | s | scalar | Navigation | **live** |
| Distance with MIL on | `21` | km | scalar | Navigation | **live** |
| Distance since cleared | `31` | km | scalar | Navigation | ECU-dependent |
| MIL + stored DTC count | `01` | flags | scalar | StatusEvent | **live** |
| Readiness monitors ×2 | `01`,`41` | bitfield | raw u32 | StatusEvent | **live** |
| DTC list | `03`,`07` | code | array | StatusEvent | **live** |
| OBD standard | `1C` | raw | scalar | StatusEvent | **live** (raw) |
| VIN, ECU name, cal ID, CVN | svc 09 | text | — | Identity | **live** |

Not available on this ECU: coolant temp `05` (air-cooled), **fuel level `2F`**,
control-module voltage `42`, **odometer `A6`**, MAF `10`. See `docs/ECU_schema.md`.

### 3.2 Battery — JBD today, Daly planned

| Parameter | Unit | Treatment | Packet | Status |
|---|---|---|---|---|
| Pack voltage | V | scalar | Battery | **live** |
| Pack current | A, + = charge | StatBlock | Battery | **live** |
| Remaining / full capacity | Ah | scalar ×2 | Battery | **live** |
| State of charge | % | scalar | Battery | **live** |
| State of health | % | scalar | Battery | vendor-dependent |
| Cycle count | count | scalar | Battery | **live** |
| Per-cell voltage ×16 | mV | array | Battery | **live** |
| Cell present mask | bitfield | u32 | Battery | **live** |
| Cell min / max / spread + indices | mV | scalar ×5 | Battery | **live** |
| Balancing mask | bitfield | u32 | Battery | **live** |
| Temperatures ×8 + min/max | 0.1 °C | array | Battery | **live** |
| Protection flags ×13 | bitfield | u16 | StatusEvent | **live** |
| MOSFET state | bitfield | u8 | StatusEvent | **live** |

### 3.3 Motor driver — Votol EM-30S

Field map from the EM100 reverse engineering; see `docs/VOTOL_EM30S_study.md`.

| Parameter | Votol bytes | Unit | Treatment | Packet | Status |
|---|---|---|---|---|---|
| Shaft speed | B16–B17 | rpm | StatBlock | Motor | pending |
| Phase / battery current | B9–B10 ÷10 | A | StatBlock | Motor | pending |
| Torque | — | N·m | StatBlock | Motor | **inferred from current** |
| Throttle | unconfirmed | % | StatBlock | Motor | pending |
| Bus voltage | B7–B8 ÷10 | V | scalar | Motor | pending |
| Controller temperature | B18 −50 | 0.1 °C | scalar | Motor | pending |
| Motor temperature | B19 −50 | 0.1 °C | scalar | Motor | pending |
| Fault flags (28 conditions) | B12–B15 | u32 | — | StatusEvent | pending |
| Controller state | B23 | enum | — | StatusEvent | pending |

### 3.4 Sensor board — partner group 1

| Sensor | Parameter | Unit | Treatment | Packet |
|---|---|---|---|---|
| **KY-003** hall | Wheel speed | km/h | StatBlock | Dynamics |
| | Wheel RPM | rpm | scalar | Dynamics |
| | Longitudinal acceleration | m/s² | StatBlock | Dynamics |
| | **Jerk** | m/s³ | StatBlock | Dynamics |
| | Cumulative distance | m | scalar | Dynamics |
| | Wheel circumference | m | config echo | Dynamics |
| **MPU6050** | Acceleration X / Y / Z | g | StatBlock ×3 | Dynamics |
| | Pitch | ° | StatBlock | Dynamics |
| | Roll | ° | StatBlock | Dynamics |
| | Tilt | ° | StatBlock | Dynamics |
| | Roll risk | bool | flag | Dynamics + StatusEvent |
| **BMP280** | Barometric pressure | hPa | scalar | Navigation |
| | Ambient temperature | °C | scalar | Navigation |
| | Barometric altitude | m | scalar | Navigation |
| | Vertical incline gradient | % | scalar | Navigation |
| **VL53L0X** | Proximity distance | mm | scalar | StatusEvent |
| | Presence flag | bool | flag | StatusEvent |
| **RC522** | Card UID | 4/7/10 B | bytes | StatusEvent |
| | Authorisation result | bool | flag | StatusEvent |
| **NEO-6M** | Latitude / longitude | 1e-7 ° | int32 ×2 | Navigation |
| | GPS speed | km/h | StatBlock | Navigation |
| | Heading | ° | scalar | Navigation |
| | GPS altitude | m | scalar | Navigation |
| | UTC time / date | epoch s | u32 | Navigation |
| | Total distance (haversine) | km | scalar | Navigation |
| | Satellite count, fix quality | count/enum | scalar ×2 | Navigation |

**Jerk deserves a note.** It is the derivative of acceleration, which is itself
the derivative of speed — two differentiations of a pulse-counted signal. It is
the noisiest channel in the system and the one where a raw sample is least
meaningful. Transmitting its mean and standard deviation over a window is not a
compromise; it is the only honest representation.

---

## 4. The envelope

16 bytes, little-endian (native on both the ESP32-S3 and any x86/ARM consumer).

| Offset | Field | Type | Meaning |
|---|---|---|---|
| 0 | `magic` | u16 | `0xE15A` |
| 2 | `schemaVersion` | u8 | **`2`** — a mismatch is rejected, not guessed at |
| 3 | `type` | u8 | `PacketType` |
| 4 | `payloadBytes` | u16 | excludes this header |
| 6 | `sequence` | u16 | per source, wraps; gaps reveal loss |
| 8 | `uptimeMs` | u32 | producer clock — there is no wall clock on the device |
| 12 | `source` | u8 | `SourceId` |
| 13 | `flags` | u8 | `partial` · `degraded` · `simulated` |
| 14 | `crc` | u16 | CRC-16/MODBUS over header (crc zeroed) then payload |

`simulated` matters immediately: group 1 is sending **simulated data first**, and
that bit is how a consumer tells it from a measurement. Never set it in
production.

`SourceId`: `kEcu` · `kBatteryA` (JBD) · `kBatteryB` (Daly) · `kMotorDriver` ·
`kSensorGroup1` · `kFusion` (a packet combining more than one source).

## 5. StatBlock — the preprocessing output

32 bytes. One per fast channel per window.

| Field | Type | Meaning |
|---|---|---|
| `min`, `max` | f32 | extremes in the window; NaN when `samples == 0` |
| `mean` | f32 | running mean |
| `stddev` | f32 | **sample** standard deviation; 0 when `samples < 2` |
| `last` | f32 | most recent raw sample, for a live readout |
| `slopePerSec` | f32 | `(last − first) / window` — a crude trend term |
| `windowMs` | u32 | time actually spanned, not the nominal period |
| `samples` | u16 | **0 means the channel was not read** |

Computed with **Welford's online algorithm**: one pass, constant memory, stable
where a naive sum of squares fails. That matters concretely — engine RPM idles
near 1500 with a spread of ones, and in `float` the naive form loses the variance
entirely at that ratio. There is a regression test for exactly that case.

**Six features per channel.** The system carries **24 StatBlock channels**, so
**144 statistical features** plus scalars reach a consumer every second.

---

## 6. The seven packets

| Type | Value | Rate | Payload | On wire | StatBlocks |
|---|---|---|---|---|---|
| `kIdentity` | `0x01` | on change / 60 s | 112 B | 128 B | — |
| `kEngine` | `0x10` | 1 Hz | 184 B | 200 B | 5 |
| `kMotor` | `0x11` | 1 Hz | 152 B | 168 B | 4 |
| `kDynamics` | `0x12` | 1 Hz | 308 B | 324 B | **9** |
| `kNavigation` | `0x13` | 1 Hz | 128 B | 144 B | 2 |
| `kBattery` | `0x20` | 1 Hz | 128 B | 144 B | 1 |
| `kStatusEvent` | `0x30` | on change | 64 B | 80 B | — |

**The five periodic packets total 980 B/s.** Even a 9600-baud link carries that.

### 6.1 `kIdentity` — who is on the bus

VIN, ECU name, calibration ID, CVN, motor model, per-link bitrates, battery
vendor and cell count, OBD standard raw byte, motor link kind. Static for a
session; sent on change and as a once-a-minute keepalive so a consumer that
joins late is not left guessing.

### 6.2 `kEngine` — ICE powertrain

Five StatBlocks (RPM, throttle, MAP, timing advance, load) plus intake air
temperature, oil temperature, short-term fuel trim and fuel system status, with
request/reply counts so a consumer can see link quality rather than infer it.

### 6.3 `kMotor` — electric powertrain

Four StatBlocks (shaft RPM, current, torque, throttle) plus bus voltage,
controller and motor temperature, fault flags, controller state. Kept **separate
from `kEngine`** deliberately: the two are different propulsion systems and a
given platform normally has one, not both.

### 6.4 `kDynamics` — the fastest source in the system

Nine StatBlocks: acceleration X/Y/Z, pitch, roll, tilt, wheel speed,
longitudinal acceleration, jerk. Plus wheel RPM, odometer and the configured
circumference, echoed so a consumer can check the scaling that produced the rest.

This is the packet that most justifies the statistics layer. At a few hundred
hertz across nine channels, raw transmission would be ~5 kB/s of data a model
would immediately reduce to the same six numbers per channel we send.

### 6.5 `kNavigation` — where, how high, how fast

Position as **scaled integer degrees, 1e-7 per LSB** — about 11 mm, and exact.
A `float` carries ~7 significant digits, which at two or three digits of whole
degrees leaves metres of error; this is the classic GPS-in-float bug and there is
a test asserting the integer form survives bit-exact.

Deliberately combined: **three independent speed sources** (GPS, wheel hall, ECU
PID `0D`) and **two independent altitude sources** (GPS, barometric) in one
packet, so a consumer can cross-check rather than trust one sensor. Plus heading,
pressure, ambient temperature, incline gradient, UTC, satellite count, fix
quality, and the ECU's own distance counters.

### 6.6 `kBattery` — vendor-independent pack state

One StatBlock (current) plus pack voltage, capacities, SoC/SoH, cycles, 16 cell
voltages with a present mask, min/max/spread with indices, balancing mask, eight
temperatures with min/max.

`cellPresentMask` earns its place on the real pack: **17 configured channels,
16 carrying voltage, channel 10 reading exactly 0 mV**. Without the mask that
dead channel drags the minimum and spread to nonsense.

### 6.7 `kStatusEvent` — everything discrete, on change

ECU monitors and DTC list, battery protection flags and MOSFET state, motor
fault flags and controller state, RFID UID and authorisation, proximity and
presence, roll risk — plus an **`eventMask`** naming what changed since the last
status packet, so a consumer wanting only edges reads one field.

---

## 7. Rate budget

| Signal class | Native rate | Acquired at | Sent as |
|---|---|---|---|
| MPU6050 accel / attitude | 100–1000 Hz | sensor board | StatBlock, 1 Hz |
| Wheel speed, accel, jerk | pulse rate | sensor board | StatBlock, 1 Hz |
| Engine RPM | continuous | **~8 Hz** | StatBlock, 1 Hz |
| Engine throttle, MAP, timing, load | continuous | **~2.7 Hz** | StatBlock, 1 Hz |
| Motor RPM, current | continuous | ~3.3 Hz if CAN | StatBlock, 1 Hz |
| Battery pack and cells | ~1 Hz | 1 Hz | scalar + StatBlock |
| GPS | 1 Hz | 1 Hz | scalar + StatBlock |
| Barometric | ~1 Hz | 1 Hz | scalar |
| Identity, DTCs, RFID | static / event | on change | on change |

**The acquisition rate is the real ceiling, not the link.** Engine PIDs are
bounded by ISO 15765-4's one-outstanding-request rule and the ≤ 20 requests/s
cap in `docs/activa-can.md`; a 60 ms gap gives ~16 requests/s. Sending RPM faster
than 8 Hz would transmit interpolation, not measurement — which is the whole
argument for statistics over raw samples.

## 8. Using a packet

```c
if (!telemetry::verify(buf, n)) return;          // magic, version, length, CRC

telemetry::PacketHeader h;
telemetry::peekHeader(buf, n, h);
if (h.flags & telemetry::kFlagSimulated) mark_as_simulated();

if (h.type == (uint8_t)telemetry::PacketType::kDynamics) {
    const auto *d = (const telemetry::DynamicsTelemetry *)telemetry::payloadOf(buf, n);
    if ((d->fields & telemetry::kDynJerk) && d->jerkMsss.samples > 0) {
        use(d->jerkMsss.mean, d->jerkMsss.stddev, d->jerkMsss.max);
    }
}
```

Both guards matter. `fields` says the producer intended to populate the channel;
`samples > 0` says it actually got data. They disagree when a poll cycle fails
partway, which on a lossy CAN link is routine.

## 9. Changing this schema

- **Adding** a field inside existing reserved bytes, or a new `PacketType`: safe.
  Every payload is padded to a round size with spare bytes for exactly this.
- **Reordering or resizing** an existing field, or renumbering a `PacketType`:
  breaking. Increment `kSchemaVersion` — consumers reject an unknown version
  rather than misparsing it.

**v1 → v2** merged the former `kVehicle` into `kNavigation` and `kStatusEvent`,
promoted `kMotor` from reserved, and added `kDynamics` and `kNavigation` for the
sensor board. v1 was never consumed externally.

## 10. Test coverage

`test/test_telemetry/` — 15 tests, host only, no hardware:

- Welford against a hand-computed dataset (mean 5, σ 2.13809)
- numerical stability at a large mean with small spread, the RPM case
- empty window reports NaN and `samples == 0`, never a plausible zero
- single sample has zero spread and zero slope, not NaN
- encode/verify round trip preserving every header field
- single-bit corruption detected in header **and** payload
- truncated, header-only and undersized buffers rejected
- unknown schema version rejected
- field mask distinguishes *absent* from a genuine zero reading
- GPS position survives bit-exact as scaled integer, and the float form does not
- Dynamics carries nine summarised channels
- StatusEvent edge mask and variable-length RFID UID
- **all seven packet types round-trip**, so none is accidentally unencodable

## 11. Open items

| Item | Owner | Blocks |
|---|---|---|
| Group 1's wire format for the sensor board | group 1 | how `kDynamics` / `kNavigation` get filled |
| Votol EM-30S: UART or CAN? | us | `kMotor` entirely |
| Daly CAN parameters | us | second `kBatteryB` source |
| Torque source, or accept current-derived | us | `MotorTelemetry.torqueNm` |
| Whether `simulated` data is marked at source | group 1 | trust of early datasets |
| Scope: Elespa's document reportedly excludes "CAN bus communication" and "battery management" | **confirm with Elespa** | framing of the deliverable |
