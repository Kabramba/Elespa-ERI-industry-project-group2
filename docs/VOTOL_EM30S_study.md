# Votol EM-30S motor controller — interface study

Pre-bring-up study. **Nothing here has been measured on our unit yet** — every
figure is marked with where it came from, and the plan in §7 exists to replace
inference with measurement.

---

## 1. The two datasheets disagree

The two sheets supplied are both titled EM-30S and give **materially different
numbers**:

| Specification | Sheet A ("MOTOR ECU-EM 30s") | Sheet B ("MOTOR ECU-EM-30S") | Vendor listings |
|---|---|---|---|
| Rated current @ 48–60 V | **35 A** | **25 A** | 35 A max DC |
| Boost current | not listed | **40 A** | 50 A |
| Peak current | 120 A | 120 A | — |
| Max power @ 48–60 V | **2100 W** | **2400 W** | — |
| Max efficiency | **≤ 89 %** | **≥ 93 %** | — |
| Voltage range | 48–84 V | 48–108 V columns (96/108 blank) | 48–72 V |

Two of these are not rounding differences:

- **Rated current 35 A vs 25 A** — a 40 % difference in continuous rating.
- **Efficiency `≤89 %` vs `≥93 %`** — these are opposite bounds. One is an upper
  limit, the other a lower limit. At least one sheet is wrong, most likely a
  mis-set inequality on sheet A.

Sheet B carries mechanical drawings (120 × 124 mm footprint, 51 mm tall) and
columns for 96 V and 108 V variants, so it is probably the **newer revision**.

> **Treat the label on the physical unit as authoritative**, and derive the real
> continuous rating by measurement. For the thermal and current headroom in the
> telemetry schema, the conservative figure — **25 A continuous, 120 A peak** —
> is the one to design against.

A practical reading: continuous output is around **1.0–1.5 kW**, with
**2.1–2.4 kW** available in short bursts. Vendor listings describe it as suited
to **500–1000 W motors**, which is consistent with the conservative sheet.

## 2. Which interface does an EM-30S actually have?

**This is the first question to settle, and it decides the whole approach.**

Community sources state the split plainly: the **"S" series uses UART**, the
**"SP" series uses CAN**. If that holds, an EM-**30S** is **serial, not CAN** —
and the CAN work we have already built does not apply to it.

| | UART path | CAN path |
|---|---|---|
| Applies to | EM-xxS (likely ours) | EM-xxSP, and EM100 as documented |
| Bitrate | 9600 baud reported | 250 kbit/s |
| Levels | TTL (3.3/5 V) | differential, needs a transceiver |
| Our readiness | spare UARTs on the ESP32-S3 | SN65HVD230 already wired and proven |

**Do not assume.** Resolve it by looking at the connectors and by listening, per
§7. It is entirely possible the unit has both populated, or neither exposed on
the harness supplied.

## 3. If it is CAN — the protocol is already documented

Reverse-engineered for the **EM100** by
[zexry619/votol-esp32-can-bus](https://github.com/zexry619/votol-esp32-can-bus),
which uses an ESP32 and an **SN65HVD230** — the same transceiver we have on the
bench.

**250 kbit/s, 11-bit standard identifiers.**

| ID | Direction | Content |
|---|---|---|
| `0x3FF` | display → controller | poll; 2 frames back to back, every 300 ms |
| `0x3FE` | controller → display | live data; **3 frames form one 24-byte record** |

The three response frames are concatenated into one `B0`–`B23` stream:

| Field | Bytes | Scaling | Unit |
|---|---|---|---|
| Battery voltage | B7–B8 | ÷ 10 | V |
| Battery current | B9–B10 | ÷ 10 | A |
| RPM | B16–B17 | ×1 | rpm |
| Controller temperature | B18 | −50 offset | °C |
| Motor temperature | B19 | −50 offset | °C |
| Error code | B12–B15 | bitfield | u32, 28 conditions |
| Controller state | B23 | enum | see below |

Controller state: `0` idle · `1` init · `2` start · `3` run · `4` stop ·
`5` brake · `6` wait · `7` fault.

### This one is a broadcaster, and that changes everything

The JBD answers only remote frames. The Activa answers only diagnostic requests.
**The Votol appears to transmit `0x3FE` unprompted every 300 ms.** If so it is
the first of our three devices that can be read **completely passively** —
`Interaction::kBroadcastListen` in our profile model, which until now has had no
real user.

That is also the safest possible first contact: our existing **`sniff-250k`**
profile is listen-only, transmits nothing, and is exactly the right tool.

Caveat: this map is from an **EM100**, and the field offsets are marked
unconfirmed for gear/status bits even in that source. Verify against our unit
before trusting any byte.

## 4. If it is UART

Less well documented publicly. What is reported:

- **9600 baud**, TTL levels — one source using a logic analyser reported a much
  lower rate, so **measure rather than assume**.
- The vendor configuration software is Windows-only and talks over a
  **USB-to-TTL adapter** supplied with the controller.
- The protocol is request/response, driven by that software.

**The reliable way in is to watch the vendor tool.** Put a logic analyser or a
passive TTL tap between the adapter and the controller, run the Windows app, and
capture a session. That gives request frames, reply frames and checksum in one
go — the same approach that cracked the JBD UART protocol.

## 5. The motor — direct answers

### Three phase terminals

Three thick wires, conventionally **yellow / green / blue** (U / V / W). They
carry the full phase current, so they are the heaviest conductors on the unit.

Any **three-phase BLDC or PMSM** will do electrically. Match it to the
controller rather than the other way round:

| Parameter | Target |
|---|---|
| Nominal voltage | **60 V** (your pack; 48–72 V is the controller's stated range) |
| Continuous power | **1000–1500 W** |
| Peak power | up to ~2.4 kW, briefly |
| Continuous current | ≤ 25–35 A, per §1 |
| Type | hub motor or mid-drive, hall-sensored |

A typical 60 V 1000–1500 W e-scooter hub motor is the natural match, and is what
the EM-30S is sold for.

### Hall sensors — yes, 5 wires

**Yes.** The EM-30S is specified for **hall-sensored** operation, and the hall
connector is the standard **5-wire** arrangement:

| Wire | Function | Typical colour |
|---|---|---|
| 1 | +5 V supply to the hall ICs | red |
| 2 | Ground | black |
| 3 | Hall A | yellow |
| 4 | Hall B | green |
| 5 | Hall C | blue |

Many motors present **6 wires** — the sixth is a **motor temperature sensor**
(often a KTY84 or a thermistor), which the controller can use for thermal
rollback. If yours has it, connect it: motor temperature is `B19` in the CAN map
and a field in our telemetry schema.

**Phase and hall order must match each other**, not merely be connected. There
are 36 possible combinations of three phase wires and three hall wires and only
a few work. If the motor stutters, cogs, runs rough or draws heavy current at
standstill, that is a phase/hall mismatch — use the controller's **hall
self-learn / angle-learn** routine in the vendor software rather than trying
combinations by hand.

## 6. Hazards specific to this device

This is the first thing in the project that can deliver kilowatts.

**Capacitor inrush.** The controller has a large bulk capacitor bank. Connecting
a 60 V pack directly produces a substantial spark that pits and eventually welds
connector contacts, and stresses the BMS. **Use a pre-charge resistor** — the
same technique as the LiPo jump-start: bridge through ~100 Ω for a second or two,
then make the full connection. Your anti-spark connector habit from robotics
applies directly.

**Fusing.** At 120 A peak, an unfused fault is a fire. Fuse the pack side.

**Never run a hall-sensored motor with phases mis-wired under load.** It will
draw very large current at near-zero torque.

**Spin-up clearance.** If a wheel is attached, the same rules as the Activa:
secured, nothing near it, short bursts.

**Ground reference.** If the ESP32 shares ground with the controller, that ground
now carries motor-return noise. Keep the signal ground connection short and star
it at one point; expect a far dirtier electrical environment than either the BMS
or the ECU presented.

## 7. Bring-up plan — first communication, nothing else

Priority is a byte from the controller, by any route.

**Step 1 — identify the connectors.** Photograph the harness. The EM-30S has
three connectors (visible on both sheets): one large and two small. Look for a
twisted pair (CAN) and for a 3–4 pin group with a TTL look to it. Record every
wire colour and position before anything is plugged in.

**Step 2 — power it on a bench supply, not the pack.** Set a current limit of
1–2 A. The controller will draw only its logic current with no motor attached,
and a current limit turns a wiring mistake into a sulk rather than a bang.
**A motor is not needed to establish communication** — this is the fastest path
to data and it removes every mechanical hazard.

**Step 3 — listen for CAN, passively.** We already have the tool:

```bash
.venv/bin/pio run -e esp32s3_activa_probe -t upload
```

It sweeps 500 k / 250 k / 1 M / 125 k in `TWAI_MODE_LISTEN_ONLY` and transmits
nothing. If the controller broadcasts, **250 k should light up with frames and
no bus errors**, and `0x3FE` should appear at roughly 3.3 Hz.

- Frames at 250 k → it has CAN; go to §3.
- Bus errors at every rate → CAN lines present but something is wrong; check
  CAN-H/CAN-L polarity and termination.
- Silence everywhere with no errors → **no CAN on this unit**; go to §4.

**Step 4 — if silent, find the UART.** With the controller powered and idle,
probe the small-connector pins against ground with a scope or logic analyser.
A UART TX idles **high** at logic level and bursts when the vendor tool talks.
Then capture a session with the Windows tool in the loop.

**Step 5 — only then** attach a motor, and only with the pre-charge and fusing
of §6 in place.

## 8. Where this lands in the project

A `votol-em30s` entry goes into `src/can/can_profile_registry.cpp` **once the
interface is known** — not before. Deliberately absent for the same reason the
Daly is: a profile asserting 250 kbit/s on a controller that turns out to be
UART-only would be an invented fact wearing a measured-looking badge.

The telemetry side is already prepared. `telemetry::MotorTelemetry`
(`include/telemetry/packets.h`, type `0x30`) reserves shaft RPM, current and
torque as `StatBlock`s, plus bus voltage, controller and motor temperature,
fault flags and state. The EM100 field map in §3 maps onto it almost one to one:

| Votol field | Telemetry field |
|---|---|
| RPM, B16–B17 | `shaftRpm` (StatBlock) |
| Battery current, B9–B10 | `currentAmps` (StatBlock) |
| Battery voltage, B7–B8 | `busVolts` |
| Controller temp, B18 | `controllerTempDeciC` |
| Motor temp, B19 | `motorTempDeciC` |
| Error code, B12–B15 | `faultFlags` |
| Controller state, B23 | `state` |

Torque has no source in that map and may have to be inferred from current.

## 9. Open questions

| Question | How to settle it |
|---|---|
| CAN or UART on this unit? | §7 steps 3–4 |
| Real continuous current rating | unit label, then measure under load |
| Does `0x3FE` broadcast, or need the `0x3FF` poll? | listen-only first; only then consider polling |
| Field offsets valid for EM-30S? | cross-check against the vendor display |
| Hall connector pinout and whether a temp sensor pin exists | continuity and voltage check, controller powered |
| Does it expose throttle position or gear over the link? | the EM100 map flags these as unconfirmed |

---

## Sources

- Supplied datasheets, two revisions — conflicting, see §1
- [zexry619/votol-esp32-can-bus](https://github.com/zexry619/votol-esp32-can-bus) —
  EM100 CAN identifiers, field offsets and controller-state enum
- [Endless Sphere — Votol serial communication protocol](https://endless-sphere.com/sphere/threads/votol-serial-communication-protocol.112970/) —
  UART vs CAN split across S and SP series; 9600 baud reports
- [Endless Sphere — Votol EM100 CANbus protocols](https://endless-sphere.com/sphere/threads/votol-em100-canbus-protocols.114159/)
- [VOTOL EM30S user manuals](https://manuals.plus/category/votol) — voltage range,
  current ratings, hall-sensor operation
