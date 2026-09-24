# Telemetry schema

The project's spine is **acquisition → preprocessing → telemetry packets**.
Everything downstream — the SBC, the models, whatever comes after — consumes
packets and nothing else. This document is the contract.

Transmission medium is deliberately out of scope. These are byte layouts; what
carries them later (UART, Wi-Fi, LoRa, shared memory) changes nothing here.

Code: `include/telemetry/` · `src/telemetry/` · tests in `test/test_telemetry/`

---

## 1. Design rules

**Group by rate, not by subject.** A packet holds fields that change at a similar
speed, because that is what sets how often it must be sent. Mixing a 10 Hz signal
with a once-per-minute one either retransmits the slow field 600 times for nothing
or delivers the fast one stale.

**Fast signals are summarised, never streamed.** Engine RPM moves faster than any
sane telemetry link should carry sample by sample, and a consumer wants features
rather than a waveform. Each fast channel is accumulated on the device and sent as
a `StatBlock` once per window.

**A missing reading is never a zero.** Every payload carries a `fields` bitmask,
and an absent `StatBlock` has `samples == 0` with NaN statistics. On a lossy link
this is the difference between *"the discharge MOSFET is off"* and *"that frame
never arrived"* — a distinction that already cost us a debugging session.

**Numbers are frozen once published.** `PacketType` values and struct layouts are
append-only. Every struct has a `static_assert` on its size so accidental drift
fails the build rather than the field.

## 2. The envelope

16 bytes, little-endian (native on both the ESP32-S3 and any x86/ARM consumer).

| Offset | Field | Type | Meaning |
|---|---|---|---|
| 0 | `magic` | u16 | `0xE15A` |
| 2 | `schemaVersion` | u8 | `1` — a mismatch is rejected, not guessed at |
| 3 | `type` | u8 | `PacketType` |
| 4 | `payloadBytes` | u16 | excludes this header |
| 6 | `sequence` | u16 | per source, wraps; gaps reveal loss |
| 8 | `uptimeMs` | u32 | producer clock — there is no wall clock on the device |
| 12 | `source` | u8 | `SourceId`, so two batteries are distinguishable |
| 13 | `flags` | u8 | `partial` · `degraded` · `simulated` |
| 14 | `crc` | u16 | CRC-16/MODBUS over header (crc zeroed) then payload |

`simulated` is never set in production — it exists so synthetic data can never be
mistaken for a measurement.

## 3. StatBlock — the preprocessing output

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

Computed with **Welford's online algorithm**: one pass, constant memory, and
numerically stable where a naive sum of squares fails. That matters here — RPM
idles near 1500 with a spread of ones, and in `float` the naive form loses the
variance entirely at that ratio. There is a regression test for exactly that case.

Six features per channel, so five fast engine channels yield **30 features** per
window plus the window length.

## 4. Packet types

| Type | Value | Rate | Payload | On wire |
|---|---|---|---|---|
| `kIdentity` | `0x01` | on change / 1 min | 80 B | 96 B |
| `kEngine` | `0x10` | ~1 Hz | 184 B | 200 B |
| `kVehicle` | `0x11` | ~1 Hz | 80 B | 96 B |
| `kBattery` | `0x20` | ~1 Hz | 128 B | 144 B |
| `kMotor` | `0x30` | reserved | 112 B | — |
| `kIotGroup` | `0x40` | reserved | 136 B | — |

At 1 Hz the three live packets total **440 B/s**, which fits any plausible link.

---

## 5. Feature list

What a consumer can expect. **Status** says whether it exists today.

### 5.1 Engine (`kEngine`) — source: Activa ECU

| Feature | PID | Unit | Form | Status |
|---|---|---|---|---|
| Engine speed | `0C` | rpm | StatBlock ×6 | **Live** |
| Throttle position | `11` | % | StatBlock ×6 | **Live** |
| Manifold absolute pressure | `0B` | kPa | StatBlock ×6 | **Live** |
| Ignition timing advance | `0E` | ° BTDC | StatBlock ×6 | **Live** |
| Calculated engine load | `04` | % | StatBlock ×6 | **Live** |
| Intake air temperature | `0F` | 0.1 °C | scalar | **Live** |
| Engine oil temperature | `5C` | 0.1 °C | scalar | **Live** |
| Short-term fuel trim | `06` | % | scalar | **Live** |
| Fuel system status ×2 banks | `03` | enum | scalar | **Live** |
| Link quality (requests/replies) | — | count | scalar | **Live** |

**35 numeric features per window.** The engine is air-cooled, so there is no
coolant temperature (`05`) — oil temperature is the thermal channel.

### 5.2 Vehicle (`kVehicle`) — source: Activa ECU

| Feature | PID | Unit | Form | Status |
|---|---|---|---|---|
| Road speed | `0D` | km/h | StatBlock ×6 | **Live** |
| Run time since start | `1F` | s | scalar | **Live** |
| Distance with MIL on | `21` | km | scalar | **Live** |
| Distance since codes cleared | `31` | km | scalar | ECU-dependent |
| MIL state, stored/pending DTC counts | `01` | flags | scalar | **Live** |
| DTC list | `03`/`07` | code | up to 8 | **Live** |
| Readiness monitors, since clear | `01` | bitfield | raw u32 | **Live** |
| Readiness monitors, this cycle | `41` | bitfield | raw u32 | **Live** |
| OBD standard | `1C` | raw | scalar | **Live** (raw, see below) |
| Odometer | `A6` | km | scalar | **Not available** |

Two honest caveats for downstream: there is **no odometer and no fuel level** over
standard OBD on this ECU, and PID `1C` returns `0x25` (37), which is **outside the
published ISO 15031-5 enumeration** — the raw byte is forwarded rather than
decoded. Both are documented in `docs/ECU_schema.md`.

### 5.3 Battery (`kBattery`) — source: JBD today, Daly planned

Vendor-independent by design. The two BMSes differ in bitrate, frame format and
addressing; none of that reaches this packet.

| Feature | Unit | Form | Status |
|---|---|---|---|
| Pack voltage | V | scalar | **Live (JBD)** |
| Pack current | A, + = charge | StatBlock ×6 | **Live (JBD)** |
| Remaining / full capacity | Ah | scalar ×2 | **Live (JBD)** |
| State of charge | % | scalar | **Live (JBD)** |
| State of health | % | scalar | vendor-dependent |
| Cycle count | count | scalar | **Live (JBD)** |
| Per-cell voltage ×16 | mV | array | **Live (JBD)** |
| Cell present mask | bitfield | u32 | **Live (JBD)** |
| Cell min / max / spread + indices | mV | scalar ×5 | **Live (JBD)** |
| Balancing mask | bitfield | u32 | **Live (JBD)** |
| Temperatures ×8 | 0.1 °C | array | **Live (JBD)** |
| Temperature min / max | 0.1 °C | scalar ×2 | **Live (JBD)** |
| Protection flags ×13 | bitfield | u16 | **Live (JBD)** |
| MOSFET state (charge / discharge) | bitfield | u8 | **Live (JBD)** |

**~55 features.** `cellSpreadMv` is the single most useful pack-health number and
is computed on the device so a consumer need not reduce 16 cells itself.

`cellPresentMask` earns its place: the measured pack has 17 configured channels
but only 16 carrying voltage, and channel 10 reads exactly 0 mV. Without the mask
that dead channel drags the minimum and spread to nonsense.

### 5.4 Motor (`kMotor`) — reserved

Shaft speed, current, torque (StatBlocks), bus voltage, controller and motor
temperature, fault flags, state. **No hardware yet** — declared so the type
numbering and this feature list are stable before it arrives.

### 5.5 IoT Group 1 (`kIotGroup`) — reserved

Four generic `StatBlock` channels plus a channel mask. **Shape to be agreed with
the partner group** — the placeholder exists so integration does not require
renumbering anything.

---

## 6. Rate budget

| Signal class | Native rate | Acquired at | Sent as |
|---|---|---|---|
| Engine RPM | continuous | ~8 Hz | StatBlock, 1 Hz |
| Throttle, MAP, timing, load | continuous | ~2.7 Hz | StatBlock, 1 Hz |
| Engine/vehicle slow channels | seconds | ~0.06 Hz (full sweep) | scalar, 1 Hz |
| Battery pack and cells | ~1 Hz | 1 Hz | scalar + StatBlock, 1 Hz |
| Identity | static | once | on change |

Acquisition rates are bounded by ISO 15765-4's one-outstanding-request rule and
the ≤ 20 requests/s ceiling in `docs/activa-can.md`. A 60 ms request gap gives
~16 requests/s, inside it.

The **acquisition rate is the real ceiling**, not the link. Sending RPM faster than
8 Hz would transmit interpolation, not measurement — which is the argument for
statistics over raw samples.

## 7. Using a packet

```c
if (!telemetry::verify(buf, n)) return;                 // magic, version, length, CRC

telemetry::PacketHeader h;
telemetry::peekHeader(buf, n, h);
if (h.type != (uint8_t)telemetry::PacketType::kEngine) return;

const auto *e = (const telemetry::EngineTelemetry *)telemetry::payloadOf(buf, n);

if (e->fields & telemetry::kEngRpm && e->rpm.samples > 0) {
    use(e->rpm.mean, e->rpm.stddev, e->rpm.slopePerSec);
}
```

Both guards matter. `fields` says the producer intended to populate the channel;
`samples > 0` says it actually got data. They can disagree when a poll cycle
fails partway.

## 8. Changing this schema

- **Adding** a field to the end of a payload, or a new `PacketType`: safe.
  Bump the struct's `static_assert` and note it here.
- **Reordering or resizing** an existing field, or renumbering a `PacketType`:
  breaking. Increment `kSchemaVersion` — consumers reject an unknown version
  rather than misparsing it.

## 9. Test coverage

`test/test_telemetry/` — 11 tests, host only, no hardware:

- Welford against a hand-computed dataset (mean 5, σ 2.13809)
- numerical stability at a large mean with small spread, the RPM case
- empty window reports NaN and `samples == 0`, never a plausible zero
- single sample has zero spread and zero slope, not NaN
- encode/verify round trip preserving every header field
- single-bit corruption detected in header **and** payload
- truncated, header-only and undersized buffers rejected
- unknown schema version rejected
- field mask distinguishes *absent* from a genuine zero reading
