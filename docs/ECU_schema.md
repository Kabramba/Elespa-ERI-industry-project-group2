# Honda Activa 6G ECU — data schema

Every value the engine ECU exposes, what it means, how it is scaled, and the raw
bytes we actually observed.

**Vehicle** Honda Activa 6G, BS-VI, 2024 · VIN `ME4XXXXXXXXXXXXXX`
**ECU** `ECM -EngineControl` · calibration `0105411002` · CVN `925A91A6`
**Link** 500 kbit/s CAN, 29-bit extended addressing, ISO 15765-4
**Request** `0x18DB33F1` (functional) → **response** `0x18DAF110`

All readings below were captured with the engine **off, ignition on**, except
where a running-engine value is given for contrast.

---

## 1. How a value is addressed

A PID is **not** a CAN ID. OBD-II uses one CAN ID for every request and puts the
selector inside the payload:

```
18DB33F1 # 02 01 0C 00 00 00 00 00
└──────┘   │  │  └── PID 0x0C  — which measurement
 CAN ID    │  └───── Service 0x01 — live data
           └──────── ISO-TP single frame, 2 bytes follow
```

This is the opposite of the JBD BMS, which uses a different CAN ID per reading
(`0x100` = pack voltage, `0x104` = cell count). The difference is captured in the
`Interaction` field of `can::CanBusProfile`.

## 2. Supported-PID discovery

PIDs `0x00`, `0x20`, `0x40` … each return a 32-bit bitmap covering the next 32
PIDs. Bit 0 of the last byte says whether a further page exists.

| Page | Bitmap | PIDs reported |
|---|---|---|
| `0x00` | `F43E8011` | 01 02 03 04 06 0B 0C 0D 0E 0F 11 1C 20 |
| `0x20` | `80000001` | 21 40 |
| `0x40` | `80000010` | 41 5C |

Bit 0 of the `0x40` page bitmap is clear, so **there is no page beyond `0x60`** and
the walk terminates correctly. **17 PIDs total**, of which `0x20` and `0x40` are
page markers rather than measurements — so **15 carry data**.

---

## 3. The 15 data PIDs

### Fast-changing — these need statistics, not raw transmission

| PID | Parameter | Bytes | Formula | Unit | Observed |
|---|---|---|---|---|---|
| `0C` | **Engine RPM** | 2 | `(256·A + B) / 4` | rpm | `00 00` → 0 (off) · running ≈ 1400–1560 |
| `11` | **Throttle position** | 1 | `A · 100 / 255` | % | `1A` → 10.2 % (closed-throttle rest) |
| `0B` | **Intake manifold absolute pressure** | 1 | `A` | kPa | `5D` → 93 (≈ ambient, engine off) · running 51–92 |
| `0E` | **Ignition timing advance** | 1 | `A / 2 − 64` | ° BTDC | `80` → 0.0 · running 8.0–9.5 |
| `04` | **Calculated engine load** | 1 | `A · 100 / 255` | % | `00` → 0.0 · running 33–35 |

RPM at 4 counts per rev gives 0.25 rpm resolution. MAP oscillating 51–92 kPa while
running is manifold vacuum pulsing against a single cylinder's intake strokes —
sampled asynchronously, so successive reads catch different points in the cycle.

### Slow-changing — a scalar per packet is enough

| PID | Parameter | Bytes | Formula | Unit | Observed |
|---|---|---|---|---|---|
| `0F` | **Intake air temperature** | 1 | `A − 40` | °C | `59` → 49 · also seen 56 while hot |
| `5C` | **Engine oil temperature** | 1 | `A − 40` | °C | `56` → 46 · 80 after a run |
| `06` | **Short-term fuel trim, bank 1** | 1 | `A · 100 / 128 − 100` | % | `80` → 0.0 |
| `0D` | **Vehicle speed** | 1 | `A` | km/h | `00` → 0 |

The Activa is **air-cooled**, so PID `05` (coolant temperature) is *not supported* —
`5C` oil temperature is the thermal equivalent and the one to log.

### State and status

| PID | Parameter | Bytes | Meaning | Observed |
|---|---|---|---|---|
| `01` | **Monitor status since DTCs cleared** | 4 | see §4 | `00 04 20 00` |
| `41` | **Monitor status this drive cycle** | 4 | same layout, byte A always 0 | `00 04 20 20` |
| `03` | **Fuel system status** | 2 | one enum per bank, see §5 | `00 00` → motor off |
| `02` | **DTC that caused the freeze frame** | 2 | encoded as a DTC; `0000` = none | `00 00` |
| `21` | **Distance travelled with MIL on** | 2 | `256·A + B` | km | `00 0B` → 11 km |
| `1C` | **OBD standard conformance** | 1 | enum, see §6 | `25` → **37, undefined** |

---

## 4. PID `01` / `41` — monitor status

Four bytes. `01` is since-codes-cleared; `41` is this-drive-cycle with byte A
forced to zero.

| Byte | Bits | Meaning |
|---|---|---|
| A | 7 | MIL (check-engine lamp) on |
| A | 6–0 | count of confirmed emissions DTCs |
| B | 3 | engine type: 0 = spark ignition, 1 = compression |
| B | 2–0 | test **availability**: bit0 misfire, bit1 fuel system, bit2 components |
| B | 6–4 | test **incompleteness**, same order |
| C | 7–0 | spark-specific test availability |
| D | 7–0 | spark-specific test incompleteness |

Spark-ignition C/D bit order: 0 EGR · 1 oxygen-sensor heater · 2 oxygen sensor ·
3 A/C refrigerant · 4 secondary air · 5 evaporative system · 6 heated catalyst ·
7 catalyst.

**Our reading decoded.**

`01` → `00 04 20 00`
- A `00` — MIL **off**, **0 confirmed DTCs**
- B `04` — bit 3 clear → **spark ignition** (correct for a petrol scooter);
  bit 2 set → **components monitor supported**; bits 6–4 clear → **complete**
- C `20` — bit 5 → **evaporative-system monitor supported**
- D `00` — **complete**

`41` → `00 04 20 20`
- Same supported set, but D bit 5 is now **set** → the evaporative-system monitor
  is **incomplete for this drive cycle**. That is the expected difference between
  the two PIDs and is a useful confirmation the decode is right.

So this ECU supports exactly **two** readiness monitors: components and
evaporative system. A car would typically report many more.

## 5. PID `03` — fuel system status

One enumerated byte per bank; the Activa populates bank 1 only.

| Value | Meaning |
|---|---|
| 0 | motor off |
| 1 | open loop — insufficient engine temperature |
| 2 | **closed loop** — using oxygen-sensor feedback |
| 4 | open loop — engine load, or fuel cut on deceleration |
| 8 | open loop — system failure |
| 16 | closed loop, but an oxygen-sensor fault is present |

Observed `00 00` with the engine off, which is correct. **Worth re-reading with
the engine warm** — a transition to `2` confirms closed-loop fuelling, which is
one of the more diagnostically interesting signals on the vehicle.

## 6. PID `1C` — OBD standard, and an anomaly

Returns `0x25` = **37**.

The ISO 15031-5 / SAE J1979 enumeration defines values **1–35**, with 31 = India
OBD I (IOBD I) and 32 = India OBD II (IOBD II); 36–250 are reserved. **37 is not
a published assignment.**

Treat this as a manufacturer or region-specific code and **log the raw value**
rather than mapping it. Given the vehicle is a BS-VI two-wheeler type-approved
under AIS-137, a value adjacent to the IOBD range is plausible, but that is an
inference, not a decode.

---

## 7. What is *not* available

Confirmed absent from the supported bitmaps — each is a real finding, not a gap
in our implementation:

| PID | Parameter | Consequence |
|---|---|---|
| `05` | Coolant temperature | air-cooled engine; use `5C` oil temperature |
| `2F` | Fuel level | **no fuel gauge over OBD** |
| `42` | Control-module voltage | no battery voltage from the ECU — take it from the BMS |
| `A6` | Odometer | **no odometer over standard OBD**; only `21` distance-with-MIL |
| `10` | Mass air flow | speed-density fuelling via MAP instead |
| `0A` | Fuel pressure | — |
| `46` | Ambient temperature | only intake air `0F` |
| `14`–`1B` | Oxygen sensor voltages | not individually exposed |

## 8. Service 09 — static identity

| PID | Field | Value |
|---|---|---|
| `02` | VIN | `ME4XXXXXXXXXXXXXX` |
| `04` | Calibration ID | `0105411002` |
| `06` | Calibration verification number | `925A91A6` |
| `0A` | ECU name | `ECM -EngineControl` |

The VIN arrives as a multi-frame ISO-TP transfer: First Frame → our Flow Control
`30 00 00` → two Consecutive Frames. `ME4` is the India / Honda Motorcycle &
Scooter India world manufacturer identifier.

The CVN request answers `7F 09 78` first — *responsePending*. A client that gives
up on that negative response reads the eventual answer as the reply to its **next**
request, desynchronising everything afterwards. Wait up to 5000 ms, per ISO 15765-4.

## 9. Diagnostic trouble codes

| Service | Meaning | Response | Observed |
|---|---|---|---|
| `03` | stored (confirmed) | `43 <count> <2 bytes per DTC>` | `43 00` — none |
| `07` | pending | `47 <count> …` | `47 00` — none |
| `0A` | permanent | `4A <count> …` | no response — **not supported** |

A DTC is two bytes: bits 15–14 select the system letter (00 P, 01 C, 10 B, 11 U),
bits 13–12 the first digit, and the remaining three nibbles the rest — so `01 33`
is `P0133`.

**Always verify the positive-response SID before decoding.** Reading byte 1 as a
count without checking byte 0 is `0x43` will happily parse a negative response or
a late reply as fault codes; during bring-up that produced ten imaginary DTCs
while PID `01` was correctly reporting none.

## 10. Sampling rates in use

Bounded by ISO 15765-4's one-outstanding-request rule and the ≤ 20 requests/s
ceiling in `docs/activa-can.md`.

| Signal | Rate | Why |
|---|---|---|
| RPM (`0C`) | ~8 Hz | polled every cycle; the fastest-moving signal |
| throttle, MAP, timing, load | ~2.7 Hz | rotated one per cycle |
| all 15 PIDs | every ~18 s | full sweep refreshes the slow values |
| Service 09 identity | once | static for the life of the vehicle |

A request gap of 60 ms gives ~16 requests/s, inside the ceiling.

---

## References

- ISO 15031-5 / SAE J1979 — service and PID definitions, monitor bit layouts
- ISO 15765-2 — ISO-TP segmentation and flow control
- ISO 15765-4 — addressing, P2 timing, NRC 0x78 responsePending
- AIS-137 (ARAI) — BS-VI OBD mandate for Indian two-wheelers
- `docs/activa-can.md` — measured DLC pinout, bus electrical characteristics,
  and the transmit safety envelope
