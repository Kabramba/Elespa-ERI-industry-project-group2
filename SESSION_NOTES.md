# Session notes — 2026-09-21

## Host toolchain (verified working)

| Item | Value |
|---|---|
| arduino-cli | 1.5.0 (`/usr/bin/arduino-cli`, already on PATH) |
| esp32 core | **3.3.11** — has `driver/twai.h`, no downgrade needed |
| esptool | 5.3.1 (`~/.arduino15/packages/esp32/tools/esptool_py/5.3.1/esptool`) |
| pyserial | 3.5 |
| user groups | includes `dialout`; `/dev/ttyACM0` is `crw-rw-rw-` |
| ModemManager | active, but uploads are clean — left running |
| brltty | installed, but the port is stable — left installed |

## Board identified

| Item | Value |
|---|---|
| USB bridge | `1a86:55d3` QinHeng CH343 → **UART/COM port** |
| Device | `/dev/ttyACM0` |
| **FQBN** | **`esp32:esp32:esp32s3`** (no `CDCOnBoot=cdc` — this is the bridge port, not native USB) |
| Chip | ESP32-S3 (QFN56) **rev v0.2** |
| Features | Wi-Fi, BT 5 LE, dual core + LP core, 240 MHz |
| PSRAM | 8 MB embedded (AP_3v3) |
| Flash | **16 MB**, eFuse: quad (4 data lines), 3.3 V |
| MAC | `dc:b4:d9:38:26:68` |
| Auto-reset | **works over RTS** — no BOOT/RST button dance needed for upload |

Built with stock FQBN defaults (4 MB flash layout on the 16 MB part — harmless, the
sketch is 21 % of a 1.25 MB app partition; PSRAM left disabled, unused by these sketches).

## Build fixes applied (compile errors, not logic changes)

Both are the same root cause: **the Arduino preprocessor injects function prototypes
at the first function definition**, so any type named in a signature must be declared
above that point. Host-side compilation with mocked Arduino headers never sees this,
which is why the sketches passed there and failed here.

1. `jbd_uart_reader.ino` — hoisted `struct JbdBasic` and `struct JbdCells` up to sit
   beside `struct JbdFrame`.
2. `jbd_can_reader.ino` — hoisted `enum CrcKind` and `enum ReqResult` above the
   `// ---- Helpers ----` block.

No protocol, decode or pin logic was touched.

## Compile status — all three PASS

| Sketch | Flash | RAM |
|---|---|---|
| `jbd_uart_reader` | 276 653 B (21 %) | 21 864 B (6 %) |
| `twai_selftest`   | 280 157 B (21 %) | 21 616 B (6 %) |
| `jbd_can_reader`  | 283 541 B (21 %) | 21 728 B (6 %) |

## Flash + run result — `jbd_uart_reader`, no BMS attached

Upload wrote 276 800 B, **hash verified**, hard reset via RTS. Captured over serial:

```
# jbd_uart_reader - JBD BMS UART reader (read-only)
# self-test PASS                      <- JBD V4 parser decodes the reference frame correctly
# UART1 9600 8N1  RX=GPIO18  TX=GPIO17
>> DD A5 05 00 FF FB 77               <- valid read request, correct checksum
# reg 0x05 attempt 1/3: timeout       <- expected: nothing wired to the BMS yet
```

Poll loop runs at the intended ~1 s cadence. **Phase 0 (toolchain + board) is done.**

## Working commands

```bash
cd "/home/kabramba/Desktop/Elespa Industry Project/JBD Interfacing"
PORT=/dev/ttyACM0
FQBN=esp32:esp32:esp32s3

arduino-cli compile --fqbn "$FQBN" sketches/jbd_uart_reader
arduino-cli upload  -p "$PORT" --fqbn "$FQBN" sketches/jbd_uart_reader
python3 tools/serial_read.py "$PORT" 20 115200 --reset
```

## Next

Phase 1 wiring on BMS **J10** (see context.md §3.1 DMM pre-check first).

---

# Phase 1 — UART link: **WORKING** ✅

Wired to the connector silkscreened UART (= **J8**), VDD left disconnected.
TXD measured 3.3 V logic, so direct connection, no level shifter.

```
# BMS hardware: SP24S007L24S100A     <- matches the board label, link confirmed
```

First request after reset timed out, second succeeded — the documented wake-from-sleep
behaviour (context.md §3.5), not a fault. Output then repeats cleanly every 1 s.

## Live readings (stable across every poll)

| Field | Value |
|---|---|
| Pack voltage | **57.12 V** |
| Current | 0.00 A (idle) |
| Nominal capacity | **45.00 Ah** |
| Remaining / SoC | 0.00 Ah / **0 %** (not trustworthy — see below) |
| Cycles | 0 |
| MOS | CHG **ON**, DSG **off** |
| Protection | **0x0002 = CELL_UV** (latched) |
| Cell channels | **17 configured** |
| NTC | 5 × (28.5 / 27.3 / 27.5 / 27.7 / 28.0 °C) |
| Software ver | 0x60 |
| Manufactured | 2025-10-19 |

## Cell data — the significant finding

```
C01=3574  C02=3569  C03=3572  C04=3571  C05=3573  C06=3572  C07=3573  C08=3568
C09=3567  C10=0     C11=3568  C12=3569  C13=3571  C14=3568  C15=3571  C16=3568
C17=3573
```

- 17 channels are enabled; **16 carry real voltage**, channel **10 reads exactly 0 mV**.
- The 16 live cells sum to **57.127 V** vs a reported pack of **57.120 V** — **7 mV apart
  (0.012 %)**. So the pack really is **16S**, and channel 10 is a bridged/unused tap,
  not a measurement glitch.
- Live cells span 3567–3574 mV — only **7 mV spread**, a well-balanced pack.
- ~3.57 V/cell × 16 at 45 Ah is **NMC**, not LiFePO4.

### Root cause chain

1. BMS cell count is set to **17**; the pack is **16S** with taps 10/11 tied together.
2. Channel 10 therefore reads 0.000 V → far below any UV threshold.
3. BMS latches **CELL_UV** and switches the **discharge MOSFET off**.
4. With DSG off, the Anderson no longer sees the full pack — which explains
   **context.md's "29.1 V at the Anderson"** (≈ half of 57.12 V through the open
   MOSFET / balance network into a high-impedance DMM). The pack was never 29.1 V.
5. SoC 0 % / remaining 0.00 Ah is the BMS's confused latched state, not a flat pack.

**Fix = set cell count 17 → 16 in JBDTools.** That is a BMS *write* and is out of scope
for this read-only session. Reported, not performed.

### Discrepancy vs context.md §1.2

context.md describes the yellow pack as 8–24S **LiFePO4** at **29.1 V**. The hardware
string `SP24S007L24S100A` confirms this *is* the JBD SP24S007 board, but the pack behind
it is **16S ~3.57 V/cell, 45 Ah, 57.12 V** — i.e. NMC. The "LiFePO4" came from the board
datasheet's supported chemistry, not the pack. §1.2 should be corrected.

## Phase 1 gate (context.md §3.4)

| Criterion | Result |
|---|---|
| Pack V matches DMM within 0.2 V | ⚠️ reads 57.12 V vs 29.1 V logged at the Anderson — explained by DSG off, needs a re-measure at B−/B+ |
| Cell sum within 1 % of pack V | ✅ **0.012 %** |
| Output repeats every second | ✅ |

Recorded for Phase 3 cross-check: **17 channels, 5 NTC, 57.12 V, SoC 0 %, 45.00 Ah**.

---

# Phase 2/3 — CAN link: **WORKING** ✅ (cross-validated against UART)

## The two faults, and how each was found

### Fault 1 — pin order was backwards in the sketch

`sketches/can_pin_probe` transmits a remote frame with self-reception in
`TWAI_MODE_NO_ACK` and tries **both** pin orders in one run:

```
=== probe  TX=GPIO4  RX=GPIO5 ===   sent 1   echoed 0  | BUS_OFF TEC 128
=== probe  TX=GPIO5  RX=GPIO4 ===   sent 20  echoed 20 | RUNNING TEC 0
  !! FOREIGN FRAME  104 [4] 11 05 E3 CD   <- the BMS answered!
```

**Correct pinout is `TWAI_TX_PIN 5`, `TWAI_RX_PIN 4`** — the opposite of the sketch
defaults, and it matches how the board was actually wired. `context.md` §1.1 and §5.3
should be corrected. Both sketches updated.

### Fault 2 — the BMS ignores CAN while asleep

With the pins fixed, CAN still failed: **30/30 TX_FAILED, 0 ACKs, 0 replies**. Ruled out
in order:

| Test | Result |
|---|---|
| `can_timing_probe` — 6 bit timings, sample points 80→90 %, triple sampling | All identical: TEC 80, buserr 10, **0 ACKs**. Not a timing problem. |
| `rx_pin_check` — GPIO4 sampled against an internal pulldown | **100 % HIGH** → transceiver powered and actively driving RX. Not a wiring/power problem. |
| `rx_pin_check` — GPIO5 against pulldown | Follows the pulldown → correctly high-Z into the module's driver input. |
| Allowing retransmission (`q.ss = 0`) | **Much worse** — permanent error-passive retry loop, buserr ~8000/s. Reverted to single-shot. |

The tell: NO_ACK mode got real replies, NORMAL mode got none. In NORMAL mode the
controller, failing to see the ACK, raises an **active error flag** (6 dominant bits)
that destroys its own frame before the BMS can answer. In NO_ACK mode it skips that
check, the frame survives, and the BMS replies.

The reason it could not see the ACK: **the BMS had gone to sleep** (datasheet §4.8,
~5 min without communication). The one earlier success happened while the UART reader
had been polling. `sketches/jbd_dual_reader` polls UART first — which wakes the BMS —
then queries CAN in the same cycle:

```
UART OK in 81 ms: pack 57.12 V, 17 cells, 5 NTC, SoC 0%  <- BMS IS AWAKE
  CAN REPLY 104 [4] 11 05 E3 CD
CAN 0x104 x6 -> ACKed 4, replies 4 | bus RUNNING TEC 127
```

## CRC confirmed

JBD CAN payloads end in **CRC-16/MODBUS (poly 0xA001, init 0xFFFF), big-endian**.
Verified on live data: payload `11 05` → `0xE3CD`, frame carried `E3 CD`. The
ambiguity noted in `context.md` §5.1 is resolved — **BE**. Across the whole run,
**0 CRC failures** on every frame received.

## Cross-validation — UART vs CAN, same BMS, same second

```
field                  UART          CAN      delta
pack V                57.12        57.12       0.00
current A              0.00         0.00       0.00
remaining Ah           0.00         0.00       0.00
SoC %                     0            0          0
cell count               17           17      match
NTC count                 5            5      match
cells : 6/6 CAN frames, 16 compared, 14 identical, worst delta -1 mV
```

**Phase 3 gate (context.md §5.4) is met**: cell count, NTC count and pack voltage are
identical across both links, and cell voltages agree to within 1 mV (the 1 mV cases are
just the two links sampling microseconds apart).

## Link quality

| Link | Reliability |
|---|---|
| UART | **100 %** — every poll answered in ~80 ms |
| CAN | **~55 % per attempt**, 0 CRC errors. With `CAN_RETRIES 4` most frames land; a full 9-frame sweep usually completes. TEC oscillates 79–132, buserr climbs steadily. |

Every CAN frame that *arrives* is valid — the loss is entirely missed ACK slots, not
corruption. Worth improving but not blocking. Most likely remaining causes, in order:

1. **Termination.** Never measured (context.md §5.2 pre-check was skipped). Check
   CANH–CANL resistance at J6 with everything unplugged: ~120 Ω = BMS terminates
   internally; open = the module's 120 Ω is carrying the bus alone.
2. **Slope control.** If the SN65HVD230 module's **Rs / S / STB** pin goes to GND
   through a resistor rather than directly, edges are slowed. Tie it straight to GND
   for high-speed mode.
3. Untwisted / long CANH-CANL stub.

## Sketches in this folder

| Sketch | Purpose |
|---|---|
| `jbd_uart_reader` | Phase 1. UART telemetry, 100 % reliable. Pins 18/17. |
| `twai_selftest` | Phase 2 bench loopback. **Sends data frames — never run it with J6 connected.** |
| `jbd_can_reader` | Phase 3 CAN-only. Needs the BMS already awake. Pins TX=5 RX=4. |
| `can_pin_probe` | Finds the correct TWAI pin order; separates transceiver faults from BMS silence. |
| `can_normal_probe` | Measures NORMAL-mode reply rate. |
| `can_timing_probe` | Walks 6 bit timings to rule timing in or out. |
| `rx_pin_check` | Proves whether the transceiver actually drives the RX pin. Transmits nothing. |
| **`jbd_dual_reader`** | **The working deliverable.** UART wake + ground truth, then a full CAN sweep with retries, printed side by side. |

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 sketches/jbd_dual_reader
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3 sketches/jbd_dual_reader
python3 tools/serial_read.py /dev/ttyACM0 25 115200 --reset
```

---

# Retest — full CAN capability + sleep behaviour

## Every id 0x100–0x110 works. Nothing is unsupported.

The first sweep showed 0x103, 0x108, 0x10B, 0x10E silent after 5 retries. `can_id_stats`
re-measured each id with **25 independent single attempts, no retries**:

```
id     note                     hits    rate  last payload
0x100  control - pack           8/25     32%  16 50 00 00 00 00  crc ok
0x103  SUSPECT - MOS/date/sw    8/25     32%  00 01 33 53 00 60  crc ok
0x104  control - config         7/25     28%  11 05  crc ok
0x107  control - cells 1-3      8/25     32%  0D F6 0D F0 0D F3  crc ok
0x108  SUSPECT - cells 4-6      7/25     28%  0D F3 0D F4 0D F4  crc ok
0x109  control - cells 7-9      8/25     32%  0D F5 0D F0 0D EE  crc ok
0x10B  SUSPECT - cells 13-15    7/25     28%  0D F2 0D F0 0D F3  crc ok
0x10C  control - cells 16-17    8/25     32%  0D F0 0D F5  crc ok
0x10E  SUSPECT - cells 22-24    8/25     32%  (empty frame, DLC 0)
```

**Suspects and controls are statistically identical.** The earlier silence was chance:
the per-attempt hit rate is ~30 %, so 5 retries leaves ~17 % miss probability per id,
and across 17 ids ~3 misses are expected. 4 were seen.

Correction to the earlier note: the ~55–70 % figure was *with* retries. **Per attempt it
is ~30 %.** Retries are mandatory, not optional.

0x10E returning **DLC 0** is correct behaviour — cells 22–24 do not exist on a 17-channel
BMS, so the BMS answers with an empty frame rather than staying silent. Same for
0x10D/0x10F/0x110.

## What CAN exposes (all cross-checked against UART)

| ID | DLC | Content | Live value |
|---|---|---|---|
| 0x100 | 8 | pack V, current, remaining Ah | 57.12 V, +0.00 A, 0.00 Ah |
| 0x101 | 8 | full capacity, cycles, RSOC | 45.00 Ah, 0 cycles, 0 % |
| 0x102 | 8 | balance 1–16, balance 17–32, protection | 0x0000, 0x0000, **0x0002 CELL_UV** |
| 0x103 | 8 | MOS status, production date, sw version | CHG ON / DSG off, 2025-10-19, sw 0x0060 |
| 0x104 | 4 | cell count, NTC count | 17, 5 |
| 0x105 | 8 | NTC 1–3 | 30.0 / 27.3 / 27.6 °C |
| 0x106 | 6 | NTC 4–5 (short frame: only 5 NTCs) | 27.5 / 28.0 °C |
| 0x107–0x10C | 8 / 6 | cell voltages, 3 per frame | matches UART to ≤1 mV |
| 0x10D–0x110 | 0 | out of range for 17 cells | empty frames |

`0x103` decoded: `00 01 33 53 00 60` → MOS 0x0001 (CHG ON, DSG off), date 0x3353 =
2025-10-19, sw 0x0060. **Identical to UART register 0x03.** CAN is not a reduced
view — everything UART reports is reachable over CAN.

Decoder note: 0x106 and 0x10C come back **short** (DLC 6) when the last slot is unused.
Frames must be decoded using `DLC − 2` payload bytes, not a fixed 6. The first sweep
printed `T6=1497.0C` and `C18=27846 mV` because it read the CRC bytes as data.
`can_id_stats` prints `DLC − 2` correctly.

## Sleep: earlier conclusion was WRONG

`can_capability_sweep` PART 2 ran **CAN only, no UART at all, for 5 minutes**:

```
    t(s)   attempts     wins     rate
      16          9        7      77%
     150          8        6      75%
     285          8        5      62%
### After 300 s with no UART:
  CAN 0x104 : still answering
```

**No decay.** CAN traffic on its own keeps the BMS awake indefinitely — a CAN request
counts as "communication" for the datasheet §4.8 wake logic. So the UART poll in
`jbd_dual_reader` is *not* required to sustain CAN.

What is still untested: whether CAN alone can **cold-wake** a BMS that has already gone
to sleep. That needs 5+ minutes of total silence first. The earlier all-fail runs are
consistent with a sleeping BMS that CAN could not rouse, but that was never isolated —
the pin order was also wrong at the time, so two faults overlapped.
