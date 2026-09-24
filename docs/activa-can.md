# Honda Activa ECU CAN — Vehicle Interface Reference

Scope: everything known about the **CAN bus on Kabir's Honda Activa** and how the
ESP32-S3 + SN65HVD230 node is wired to it. This is one data source inside a larger project;
it says nothing about project structure. Treat the "Measured" sections as verified facts —
don't second-guess the pin map. Treat "Unknown" items as things to *measure*, never assume.

---

## 1. Vehicle
- **Honda Activa 6G (BS6), purchased March 2024** → BS6 Phase-2 / OBD-2 era, PGM-FI
  fuel-injected, single engine ECU.
- Diagnostic port: **6-pin red Honda DLC**, all 6 cavities populated. Accessed by removing the
  battery hood; probe wires (header pins) are inserted alongside the mated connector. All
  vehicle CAN nodes (ECU, instrument cluster, etc.) remain connected.

## 2. DLC pin map — MEASURED (DMM, ref. battery negative)

| Wire colour | Ign OFF | Ign ON | Signal | ISO 15031-3 pos. |
|---|---|---|---|---|
| **Green** | continuity to GND | 0 V | **GND** | A |
| **Yellow / Black stripe** | 0L to GND | ~2.4–2.5 V | **CAN-H** | B |
| **Brown** | ~920 Ω to GND | 4.7 V | SCS (service check, 5 V pull-up) | C |
| **Orange** | ~680 Ω to GND | 11.24 V | K-line (pulled up to V_bat) | D |
| **Yellow / Red stripe** | 0L to GND | ~2.4–2.5 V | **CAN-L** | E |
| **Black** | 0 V (beeps to GND via switched loads — not a short) | 12.06 V | +12 V, **ignition-switched** | F |

Physical layout as viewed by Kabir (mirror image of the standard face view):
```
 top:     Brown     Yellow/Black   Green
 bottom:  Black     Yellow/Red     Orange
```
CAN polarity is deduced from the ISO 15031-3 layout (GND/SCS/K-line/+12V all matched it by
measurement). Final confirmation = clean traffic. **If only bus errors/error frames appear at a
bitrate that otherwise should work, swap the two yellow wires.**

**Never connect** Brown (grounding SCS puts the ECU in service-check/blink-code mode) or
Orange (~12 V, would destroy a GPIO) to anything. Black only via a proper automotive buck + TVS.

## 3. Bus electrical characteristics — MEASURED
- Idle (ignition ON): CAN-H ≈ CAN-L ≈ 2.4–2.5 V → recessive, transceivers powered.
  Ignition OFF: lines ~0 V (bus unpowered, ECU transceiver high-Z).
- **CAN-H ↔ CAN-L = 124 Ω** (battery disconnected, all nodes plugged in) → **only one
  terminator on the vehicle side** (presumably in the ECU). The DLC is the open end of the bus;
  the tester is expected to provide the second terminator.
- **Therefore the SN65HVD230 module's onboard 120 Ω terminator stays FITTED.** Expected bus
  resistance with the node attached: ~60 Ω (to be confirmed).

## 4. Node wiring
```
DLC Yellow/Black (CAN-H) ───── SN65HVD230 CANH
DLC Yellow/Red   (CAN-L) ───── SN65HVD230 CANL
DLC Green        (GND)   ──┬── SN65HVD230 GND
                           └── ESP32-S3 GND        (common ground is mandatory)
ESP32-S3 3V3 ───────────────── SN65HVD230 VCC
ESP32-S3 TWAI TX GPIO ──────── SN65HVD230 D (TXD)
ESP32-S3 TWAI RX GPIO ──────── SN65HVD230 R (RXD)
SN65HVD230 Rs ──────────────── GND (high-speed slope mode)
```
- Transceiver: **SN65HVD230** (Kabir sometimes writes "SN65HVD320" — same part, typo).
- TWAI GPIOs: whatever the existing project already uses. If choosing fresh, avoid strapping
  pins 0/3/45/46, USB 19/20, flash/PSRAM 26–32 (33–37 on octal-PSRAM modules).
- CAN-H/CAN-L leads twisted, kept short (< ~30 cm) from DLC to module.
- Bench power: ESP32 over USB from a laptop **on battery** (no charger) to avoid a ground loop
  between mains earth and vehicle chassis.

## 5. Unknown — determine by measurement, in this order
1. **Bitrate.** Expect 500 kbit/s (ISO 15765-4 default); else 250k, then 1M, 125k.
2. **Is there broadcast traffic** (ECU ↔ cluster) or is the bus silent until a diagnostic
   request is sent? Diagnostic-only CAN on a DLC is commonly silent.
3. **Addressing:** 11-bit (0x7DF / 0x7E8) vs 29-bit (0x18DB33F1 / 0x18DAF1xx).
4. Which OBD Service 01 PIDs are supported; whether Service 09 (VIN, ECU name) and
   03/07/0A (DTCs) respond.
5. (Later, only with Kabir's approval) whether UDS 0x22 DIDs are readable in the default session.

### Bitrate probe method (listen-only — never transmits, never ACKs)
For each candidate rate: start TWAI in **LISTEN_ONLY**, accept-all filter, listen ~3 s, record
frames received, RX error counter, bus-error count, state.
- Correct rate → valid frames, ~0 bus errors.
- Wrong rate → rising bus errors/REC, no valid frames.
- No frames and no errors at any rate → bus is silent → move to an active diagnostic request.
- Note: in listen-only the ESP32 doesn't ACK; if nothing else ACKs the ECU's frames you'll see
  the same frame repeated (ACK-error retransmission). Still proves the bitrate.
- Changing mode or bitrate on ESP32 TWAI requires stop → uninstall → reinstall of the driver.

## 6. Diagnostic protocol reference (OBD-II over CAN, ISO 15765-4)

### 6.1 Addressing
| | 11-bit | 29-bit |
|---|---|---|
| Functional request | `0x7DF` | `0x18DB33F1` |
| Physical request to ECU | `0x7E0` | `0x18DA10F1` |
| ECU response | `0x7E8` (–`0x7EF`) | `0x18DAF110` (`0x18DAF1xx`) |
Try 11-bit first (3 retries, 100 ms), then 29-bit. Flow Control goes to the ECU's physical
request ID (11-bit: response ID − 8, e.g. 0x7E8 → 0x7E0).

### 6.2 Framing (ISO-TP, ISO 15765-2)
All frames DLC = 8, pad unused bytes (0xAA or 0x00).
| PCI nibble | Frame | Layout |
|---|---|---|
| 0x0 | Single | `0L data…` (L = 1–7) |
| 0x1 | First | `1L LL data(6)` (12-bit length) |
| 0x2 | Consecutive | `2N data(7)` (N = 1..F, wraps to 0) |
| 0x3 | Flow Control | tester sends `30 00 00` after a First Frame |

### 6.3 Timing
One outstanding request at a time. P2 = 50 ms response timeout; on NRC 0x78
(responsePending) extend to 5000 ms. Negative response = `7F <SID> <NRC>`.

### 6.4 Services
| SID | Purpose | +ve SID |
|---|---|---|
| 0x01 | Live data (PIDs) | 0x41 |
| 0x02 | Freeze frame | 0x42 |
| 0x03 / 0x07 / 0x0A | Stored / pending / permanent DTCs | 0x43 / 0x47 / 0x4A |
| 0x09 | Vehicle info: 0x02 VIN, 0x04 CalID, 0x06 CVN, 0x0A ECU name | 0x49 |

First probe: `7DF 02 01 00 00 00 00 00 00` → e.g. `7E8 06 41 00 AA BB CC DD xx`.

### 6.5 Supported-PID discovery
PIDs 0x00, 0x20, 0x40, … each return a 32-bit bitmap for the next 32 PIDs (MSB of byte A =
base+1). Continue while bit 0 of byte D is set. Poll only supported PIDs.

### 6.6 Service 01 decode (A,B,C,D = bytes after the PID)
| PID | Parameter | Formula | Unit |
|---|---|---|---|
| 01 | MIL + DTC count | A bit7 = MIL, A & 0x7F = count | — |
| 04 | Engine load | A·100/255 | % |
| 05 | Engine/coolant temp (air-cooled: engine temp sensor) | A − 40 | °C |
| 06/07 | STFT / LTFT | A·100/128 − 100 | % |
| 0B | MAP | A | kPa |
| 0C | RPM | (256A + B)/4 | rpm |
| 0D | Speed | A | km/h |
| 0E | Ignition timing advance | A/2 − 64 | ° BTDC |
| 0F | Intake air temp | A − 40 | °C |
| 11 | Throttle position | A·100/255 | % |
| 14–1B | O2 sensor | A/200 V, B·100/128 − 100 % | — |
| 1C | OBD standard | enum | — |
| 1F | Run time since start | 256A + B | s |
| 21 | Distance with MIL on | 256A + B | km |
| 2F | Fuel level | A·100/255 | % |
| 31 | Distance since codes cleared | 256A + B | km |
| 33 | Barometric pressure | A | kPa |
| 42 | Control module voltage | (256A + B)/1000 | V |
| 45 | Relative throttle | A·100/255 | % |
| 46 | Ambient temp | A − 40 | °C |
| 5C | Oil temp | A − 40 | °C |
| A6 | Odometer | (A·2²⁴ + B·2¹⁶ + C·2⁸ + D)/10 | km |
Unknown supported PIDs: log raw, don't guess scaling.

### 6.7 DTC decode
2 bytes each. Bits 15–14: 00 P, 01 C, 10 B, 11 U; remaining 14 bits → 4 hex digits.
`01 33` → P0133.

## 7. Transmit safety rules for this bus — NON-NEGOTIABLE
This is the engine ECU of a vehicle Kabir rides.
- **Allowed TX only:** to `0x7DF`, `0x7E0–0x7E7`, `0x18DB33F1`, `0x18DAxxF1`, carrying ISO-TP
  Single Frames with SID ∈ {0x01, 0x02, 0x03, 0x07, 0x09, 0x0A}, plus Flow Control `30 xx xx`.
  Route all Activa-bus TX through one guarded function that enforces this.
- **Never send:** 0x04/0x14 (clear DTCs), 0x10 (non-default session), 0x11 (ECU reset),
  0x27 (security access), 0x28, 0x2E, 0x2F, 0x31, 0x34–0x37, 0x3D, 0x85, 0x08 — and never
  replay/inject captured broadcast frames.
- UDS read-only (0x22, 0x19 subfn 01/02/0A, 0x3E 00) only if Kabir explicitly enables it, behind
  a build flag defaulting OFF, rate-limited ≥ 20 ms between requests.
- ≤ ~20 diagnostic requests/s total. On bus-off or error-passive while transmitting: stop all
  Activa TX until reboot and log it.
- Start listen-only. Transmitting tests only with the scooter stationary on the centre stand.

## 8. Logging convention
Log raw frames in candump text format so captures work in SavvyCAN/python-can:
`(1234.567890) can0 7E8#064100BE3FA81300` (8 hex digits for 29-bit IDs).
Record confirmed findings (bitrate, addressing, supported PIDs, VIN/ECU name, broadcast IDs)
back into this file under a "Confirmed" section.

## 9. Out of scope for now
K-line (Orange) — present on the DLC, would need an L9637D-class transceiver and Honda's
proprietary 10400-baud protocol. Only revisit if CAN data proves insufficient.
