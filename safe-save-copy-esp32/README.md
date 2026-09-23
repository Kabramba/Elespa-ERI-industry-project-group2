# safe-save-copy-esp32 — frozen, validated snapshot

**Frozen 2026-09-23.** Everything here was compiled, flashed to the real
ESP32-S3 and run against the real JBD BMS. Nothing in this folder is a draft.

Treat it as read-only. New work goes in the PlatformIO project; come back here
if that ever needs a known-good reference.

---

## Validation run — 2026-09-23

All ten sketches compile clean on esp32 core 3.3.11:

```
jbd_uart_reader        OK   276653 bytes      can_timing_probe       OK   281553 bytes
twai_selftest          OK   280157 bytes      can_id_stats           OK   280941 bytes
jbd_can_reader         OK   283541 bytes      can_capability_sweep   OK   283113 bytes
jbd_dual_reader        OK   284009 bytes      rx_pin_check           OK   279205 bytes
can_pin_probe          OK   281545 bytes      can_normal_probe       OK   280425 bytes
```

Live results against the pack:

| Link | Result |
|---|---|
| UART | **100 %** — every poll answered, ~80 ms. Pack 57.10 V, 17 channels, 5 NTC, full cell data. |
| CAN | **22–24 % frames won**, 0 CRC errors. Degraded from ~55 % on 2026-09-21 with no code change. |
| Transceiver | Healthy — GPIO4 held HIGH against an internal pulldown, GPIO5 correctly high-Z. |

Pack drifted 57.12 V → 57.10 V over two idle days, cells tracking down together.
Normal self-discharge, and a good sign the numbers are real.

## Cold-wake question: RESOLVED

After **two days of total silence**, CAN-only NORMAL mode got 10/30 replies with
UART never touched. **The BMS does not need a UART wake.** The session-1 conclusion
that sleep caused the CAN failure was **wrong** — the real cause was the swapped
TX/RX pins. `jbd_dual_reader`'s UART poll is useful for ground truth, not required.

The follow-up hypothesis — that NO_ACK probing poisons the BMS — was **not
reproduced** (`can_noack_impact`): every phase scored 0, including the baseline
that had worked minutes earlier. The link is **bursty**: `can_normal_probe` hit on
rounds 4–10 and 27–29 and missed the rest. That is an intermittent physical fault,
not a protocol one.

## Hardware configuration (measured, not assumed)

| Item | Value |
|---|---|
| Board | ESP32-S3 QFN56 rev v0.2, 16 MB flash, 8 MB PSRAM, MAC `dc:b4:d9:38:26:68` |
| USB bridge | `1a86:55d3` CH343 → `/dev/ttyACM0` |
| FQBN | `esp32:esp32:esp32s3` — **no** `CDCOnBoot=cdc` (bridge is on UART0, not native USB) |
| BMS | JBD `SP24S007L24S100A`, 17 channels configured, 5 NTC, mfg 2025-10-19 |
| UART | GPIO18 = RX ← BMS TXD · GPIO17 = TX → BMS RXD · 9600 8N1 · connector J8, **VDD unconnected** |
| CAN | **GPIO5 = TX → transceiver D** · **GPIO4 = RX ← transceiver R** · 500 kbit/s |
| CAN CRC | CRC-16/MODBUS (0xA001, init 0xFFFF), **big-endian**, over `DLC − 2` bytes |

> The CAN pin order is the **opposite** of what `context.md` §1.1/§5.3 states.
> Measured and proven by `can_pin_probe`. context.md is wrong on this.

## Sketch manifest

### Production
| Sketch | Purpose |
|---|---|
| `jbd_uart_reader` | UART telemetry. 100 % reliable. The dependable one. |
| `jbd_can_reader` | CAN-only telemetry. Pins already corrected to TX=5 RX=4. |
| `jbd_dual_reader` | **Reference implementation.** UART + CAN in one loop, printed side by side with deltas. |

### Diagnostics — each isolates exactly one variable
| Sketch | Answers |
|---|---|
| `can_pin_probe` | Which TWAI pin order is correct; transceiver fault vs BMS silence. |
| `rx_pin_check` | Is the transceiver actually driving the RX pin? **Transmits nothing.** |
| `can_timing_probe` | Is it bit timing? Walks 6 configs, 80–90 % sample points. |
| `can_normal_probe` | NORMAL-mode reply rate over N rounds. |
| `can_id_stats` | Is an ID unsupported, or just unlucky? Per-ID rates, no retries. |
| `can_capability_sweep` | Enumerates every ID 0x100–0x110 + a 5-minute sleep endurance test. |
| `can_noack_impact` | Does NO_ACK probing break subsequent NORMAL mode? (Not reproduced.) |
| `twai_selftest` | Transceiver loopback. ⚠️ **Sends DATA frames — never run with J6 connected.** |

## Build and flash

```bash
PORT=/dev/ttyACM0
FQBN=esp32:esp32:esp32s3
arduino-cli compile --fqbn "$FQBN" sketches/jbd_dual_reader
arduino-cli upload  -p "$PORT" --fqbn "$FQBN" sketches/jbd_dual_reader
python3 tools/serial_read.py "$PORT" 25 115200 --reset
```

`serial_read.py` exits after N seconds — that bounded exit is the point;
`arduino-cli monitor` is interactive and hangs a script.

## Gotchas worth carrying forward

1. **Arduino injects function prototypes above your type declarations.** Any struct
   or enum named in a signature must be declared before the first function
   definition. This broke all three original sketches and compiles fine under plain
   g++, so host-side mock builds never catch it. **PlatformIO does not do this** —
   one good reason to move.
2. **`q.ss = 1` (single shot) is load-bearing.** Without it the controller
   retransmits into a permanent error-passive loop (buserr ~8000/s) and
   `TWAI_ALERT_TX_FAILED` never fires, so you lose all per-frame visibility.
3. **Short frames are normal.** 0x106 and the last cell frame come back with
   DLC 6 when the final slot is unused. Decode `DLC − 2` payload bytes, never a
   fixed 6, or you will read CRC bytes as data.
4. **Out-of-range cell IDs answer with DLC 0**, they do not stay silent.
5. **`twai_transmit()` returning ESP_OK means queued, not transmitted.**

## Known open issue

CAN reliability fell from ~55 % to ~24 % across two idle days with no code change,
and every failure is an ACK error (TEC steps by exactly 8, zero CRC failures). The
digital side is proven good, so the suspect is the differential side. Never measured:

1. **Termination.** CANH–CANL at J6, everything unplugged: ~120 Ω = BMS terminates
   internally; open = the module's 120 Ω is carrying the bus alone.
2. **Slope control.** If the SN65HVD230 module's Rs/S/STB pin reaches GND through a
   resistor rather than directly, edges are slowed. Tie it straight to GND.
3. Connector seating on the CANH/CANL flying leads, and stub length.

**UART is unaffected and remains 100 %.**

## Pack finding (unrelated to interfacing)

Channel 10 reads exactly 0 mV. The other 16 cells sum to within ~7 mV of the
reported pack voltage, so the pack is **16S** and the BMS is configured for **17**.
The BMS reads that 0.000 V as undervoltage, latches `CELL_UV`, and holds the
**discharge MOSFET off** — which is why the Anderson measured 29.1 V instead of 57 V.

To confirm physically: an N-cell string needs **N+1** sense wires. 17 wires → 16S
(no cell on that channel). 18 wires → 17S (a genuinely dead cell).

Fix is cell count 17 → 16 in JBDTools — a **write**, deliberately not performed.
