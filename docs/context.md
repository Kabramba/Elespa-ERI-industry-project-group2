# context.md — ESP32-S3 ↔ JBD BMS telemetry (UART first, then CAN)

**Goal:** get live telemetry out of the yellow pack's JBD BMS into an ESP32-S3 DevKit that is attached to this computer over USB.

- **Order:** UART link first, then CAN.
- **Toolchain:** Arduino core for ESP32, driven through `arduino-cli`.

**Files in this folder:**

```
context.md                                  <- this file
sketches/jbd_uart_reader/jbd_uart_reader.ino  Phase 1: UART reader (9600 8N1, JBD protocol V4)
sketches/twai_selftest/twai_selftest.ino      Phase 2: CAN controller + transceiver loopback test, no BMS
sketches/jbd_can_reader/jbd_can_reader.ino    Phase 3: CAN reader (500 kbit/s, remote-frame queries)
tools/serial_read.py                          non-interactive serial capture
```

All three sketches have been compile- and logic-tested on a host with mocked Arduino/TWAI layers and a simulated BMS. **They have not yet run on the real ESP32-S3 or the real BMS.**

---

## 0. Rules for this session

1. **The agent cannot see or touch hardware.** Every wiring step, multimeter check or button press is done by the user.
   - Tell the user exactly what to do: which pins, the expected reading, and when to stop.
   - Wait for the result before moving on.
2. **Read-only BMS access.** The only frames that may ever be sent are:
   - UART: `DD A5 03 00 FF FD 77`, `DD A5 04 00 FF FC 77`, `DD A5 05 00 FF FB 77`.
   - CAN: **remote frames** (RTR) with 11-bit IDs `0x100`–`0x110`.
   - Never send a JBD write frame (second byte `0x5A`); these change BMS parameters or switch the MOSFETs.
   - Never send CAN data frames to the BMS.
3. **Gating.**
   - Do not start Phase 3 until Phase 1 shows valid data **and** Phase 2 prints `PASS`.
   - Phase 1's readings are the ground truth that Phase 3 is checked against.
4. **Scope.** Do not modify the sketches' protocol logic without a reason tied to observed output. Pin changes go through the `#define`s at the top of each sketch.

---

## 1. Hardware on the bench

### 1.1 ESP32-S3 DevKit ("DevKit V1", DevKitC-1 style board)

**Pin plan used by all sketches:**

| Function | ESP32-S3 GPIO | Connects to |
|---|---|---|
| UART1 RX | **18** | BMS UART-TXD |
| UART1 TX | **17** | BMS UART-RXD |
| TWAI TX | **4** | CAN transceiver TXD / CTX |
| TWAI RX | **5** | CAN transceiver RXD / CRX |
| GND | GND | BMS UART-GND and transceiver GND |
| 3V3 | 3V3 | CAN transceiver VCC (3.3 V module) |

**GPIOs to keep free on the S3:**

| GPIO | Why |
|---|---|
| 0, 3, 45, 46 | Strapping pins |
| 19, 20 | Native USB D−/D+ |
| 43, 44 | UART0 console via the USB-UART bridge |
| 26–32 | SPI flash |
| 33–37 | Octal PSRAM/flash on N8R8/N16R8 modules |

### 1.2 Yellow pack BMS

- **Board:** JBD (Jiabaida) SP24S007 V1.1, 8–24S LiFePO4, 100 A, one port for charge and discharge.
- **Label:** "Com: UART RS485 CAN".
- **Datasheet:** the SP24S004 sheet (same 8–24S family) provides the pinout. The board's connector silkscreen matches its J5–J8 row.
- **Topology:** the BMS switches the negative side. **B−** is the battery negative; the Anderson negative is **C−/P−**.
- **Connectors:** all are HY2.0 (2.0 mm pitch, PH2.0-compatible). The user needs HY2.0/PH2.0 pigtails.

| Connector | Pin 1 | Pin 2 | Pin 3 | Pin 4 | Notes |
|---|---|---|---|---|---|
| **J10** (top row, 2nd from left) — use this for UART | UART-GND | UART-RXD | UART-TXD | VDD = NC | Non-isolated |
| J8 (bottom row, labelled "UART") | UART-GND | UART-RXD | UART-TXD | **VDD 11–15 V** | Non-isolated; may already have a lead plugged in. **Pin 4 must never reach the ESP32** |
| **J6** (bottom row, labelled "RS485 CAN") — CAN | CAN-L | CAN-H | RS485-B | RS485-A | No GND pin on this connector |

**Link parameters:**

- **UART:** 9600 baud, 8N1 (datasheet §4.10.1).
- **CAN:** 500 kbit/s default (datasheet §4.10.2).
- **Sleep:** after ~5 min with no current, communication or balancing, the BMS lowers its sampling rate. Communication, the switch input, or charge/discharge current wakes it (datasheet §4.8).

**Pack voltage:** 29.1 V at the Anderson.

- The board's "24S" is its maximum string count, not the pack's.
- The real cell count comes back in UART register 0x03 or CAN ID 0x104.

### 1.3 Black pack (NOT in this session)

- **Pack:** Peakamp 60 V 45 Ah, 16S NMC, with a Daly DL-R24K-FTJ BMS (UART/RS485/CAN 2.0B/Bluetooth). Measured 52.7 V.
- **Connectors:** Anderson for power, plus a 5-pin GX16 circular connector.
  - The pack integrates a GPS/IoT tracker; the supplied TFT100 datasheet specifies a 10–97 V supply.
  - So the GX16 very likely carries pack voltage alongside CAN.
- **Do not wire the GX16 until it has been pinned out.**

---

## 2. Host setup (Ubuntu)

### 2.1 Install arduino-cli and the ESP32 core

```bash
command -v arduino-cli || (curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin sh)
export PATH="$HOME/.local/bin:$PATH"
arduino-cli config init 2>/dev/null || true
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli core list          # record the esp32 core version in the session notes
```

The sketches use the ESP-IDF `driver/twai.h` API, which is present in Arduino-ESP32 core **3.x**. If a compile fails because `driver/twai.h` is missing (a future core that dropped the legacy driver):

1. List available versions with `arduino-cli core search esp32 --all`.
2. Install the newest 3.x, e.g. `arduino-cli core install esp32:esp32@3.3.0`.

### 2.2 Serial port permissions and known Ubuntu issues

```bash
groups | grep -qw dialout || { sudo usermod -aG dialout "$USER"; echo "log out/in required"; }
sudo apt-get install -y python3-serial      # for tools/serial_read.py
```

- **CH340/CH341 bridge ports disappear right after plugging in:** `brltty` is claiming them. Fix with `sudo apt remove brltty`.
- **Upload is flaky on /dev/ttyACM\*:** ModemManager may be probing the port. Stop it for the session with `sudo systemctl stop ModemManager`.

### 2.3 Identify the board, port and FQBN

```bash
arduino-cli board list
lsusb | grep -Ei '303a|10c4|1a86'
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

| `lsusb` VID:PID | Which USB-C port the cable is in | Linux device | FQBN |
|---|---|---|---|
| `303a:1001` (Espressif) | Native **USB** port (USB-Serial/JTAG) | /dev/ttyACM\* | `esp32:esp32:esp32s3:CDCOnBoot=cdc` |
| `10c4:ea60` (CP210x) | **UART/COM** port (bridge) | /dev/ttyUSB\* | `esp32:esp32:esp32s3` |
| `1a86:55d3` (CH343) | **UART/COM** port (bridge) | /dev/ttyACM\* | `esp32:esp32:esp32s3` |
| `1a86:7523` (CH340) | **UART/COM** port (bridge) | /dev/ttyUSB\* | `esp32:esp32:esp32s3` |

`CDCOnBoot=cdc` routes `Serial` to the native USB port. Without it, output on a native-USB connection goes to UART0 (GPIO43/44) and nothing appears on the PC.

**Optional chip info** (flash size, PSRAM):

```bash
ESPTOOL=$(find ~/.arduino15/packages/esp32/tools/esptool_py -type f -name 'esptool' | head -1)
$ESPTOOL --port "$PORT" chip_id
$ESPTOOL --port "$PORT" flash_id
```

### 2.4 Build, flash, capture

```bash
PORT=/dev/ttyACM0                  # from 2.3
FQBN=esp32:esp32:esp32s3           # from 2.3
SK=sketches/jbd_uart_reader        # sketch folder

arduino-cli compile --fqbn "$FQBN" "$SK"
arduino-cli upload  -p "$PORT" --fqbn "$FQBN" "$SK"
python3 tools/serial_read.py "$PORT" 20 115200 --reset
```

- `serial_read.py` returns after N seconds, so the agent is never stuck in an interactive monitor.
- An interactive alternative for the user is `arduino-cli monitor -p "$PORT" --config baudrate=115200`.
- **If upload fails with "Failed to connect":** ask the user to hold **BOOT**, tap **RST**, release **BOOT**, then retry the upload, then tap **RST** again to run.

---

## 3. Phase 1 — UART link (`sketches/jbd_uart_reader`)

### 3.1 User pre-check before wiring (DMM, DC V)

Black probe on the BMS **B−** lug. Red probe on each pin of **J10**:

| Reading | Pin |
|---|---|
| 0 V | UART-GND |
| steady ≈ 3.3 V | UART-TXD idling high → 3.3 V logic → direct connection OK |
| steady ≈ 5 V | UART-TXD at 5 V logic → **stop**; needs a BSS138-type level shifter on both data lines |
| 11–15 V | That is VDD: wrong connector (J8) or wrong pin → stop |
| remaining pin | UART-RXD |

If every signal pin reads about 0 V, the BMS is asleep. Ask the user to put a small load on the Anderson briefly, then re-measure.

### 3.2 Wiring

| BMS J10 | ESP32-S3 |
|---|---|
| pin 1 UART-GND | GND |
| pin 2 UART-RXD | GPIO17 (TX) |
| pin 3 UART-TXD | GPIO18 (RX), via a 1 kΩ series resistor |
| pin 4 (NC) | not connected |

**Grounding:** the ESP32's GND goes to UART-GND (= B−) **only**. Never also connect it to the Anderson negative (C−): that would create a path around the BMS discharge MOSFETs.

### 3.3 Run

```bash
arduino-cli compile --fqbn "$FQBN" sketches/jbd_uart_reader && \
arduino-cli upload -p "$PORT" --fqbn "$FQBN" sketches/jbd_uart_reader && \
python3 tools/serial_read.py "$PORT" 20 115200 --reset
```

### 3.4 Expected output

Values below are illustrative:

```
# self-test PASS
# UART1 9600 8N1  RX=GPIO18  TX=GPIO17
>> DD A5 05 00 FF FB 77
<< DD 05 00 ...  77
# BMS hardware: <ASCII model string>
---- t=... ms ----
>> DD A5 03 00 FF FD 77
<< DD 03 00 ...  77
Pack    29.10 V   -0.00 A ...
SoC     .. %   .. / .. Ah   cycles ..
MOS     CHG ON   DSG ON
Temps   T1=..C ...
Prot    0x0000
Config  N cells, M NTC, ...
Cells   C01=....  ...
        min ...  max ...  delta .. mV   sum ..V (pack ..V)
```

**Phase 1 is done when:**

- Pack voltage matches the user's DMM reading at the Anderson within ~0.2 V.
- The sum of cell voltages is within ~1 % of pack voltage.
- The same output repeats every second.

**Record** the cell count, NTC count, pack voltage and SoC. Phase 3 is validated against them.

### 3.5 Troubleshooting

| Symptom | Likely cause → action |
|---|---|
| `self-test FAIL` | Parser broken; do not continue |
| `timeout` on every attempt, no `<<` bytes | TX/RX swapped (swap GPIO17/18 wires); GND missing; BMS asleep |
| `<<` bytes present but `checksum mismatch` / `missing 0x77` | Logic-level or baud problem; recheck §3.1 |
| First attempt times out, second succeeds | Normal: the BMS was waking from sleep |
| `BMS returned error status 0x80` | Should not happen for reads 0x03/0x04/0x05; capture the raw frame and report it |

---

## 4. Phase 2 — CAN controller self-test (`sketches/twai_selftest`)

**Purpose:** prove the ESP32-S3 TWAI peripheral and the transceiver work before the BMS is involved.

- **Mode:** `TWAI_MODE_NO_ACK` with self-reception requests.
- **Bitrate:** 500 kbit/s.
- **Other nodes:** none needed.

**Transceiver:**

- Use a **3.3 V CAN transceiver module**: SN65HVD230 / "VP230" / "CJMCU-230".
- **Ask the user which module they have before wiring.**
- **TJA1050 module (5 V):** VCC from the DevKit 5V pin; GPIO4 drives TXD directly; put a 1 kΩ / 2 kΩ divider on the module's RXD before GPIO5.
- **MCP2515 SPI boards:** not supported by these sketches.

**Step A — no transceiver:** jumper wire GPIO4 ↔ GPIO5.
**Step B — with transceiver:**

- GPIO4 → TXD/CTX, GPIO5 ← RXD/CRX, 3V3 and GND to the module.
- Leave CANH/CANL unconnected; the module's onboard 120 Ω stays fitted.
- The standby pin (Rs / S / STB) must be LOW; most modules tie it low already.

```bash
arduino-cli compile --fqbn "$FQBN" sketches/twai_selftest && \
arduino-cli upload -p "$PORT" --fqbn "$FQBN" sketches/twai_selftest && \
python3 tools/serial_read.py "$PORT" 15 115200 --reset
```

**Expected output**, every 10 frames:

```
sent 10  echoed 10  mismatch 0  lost 0  txerr 0 | RUNNING TEC 0 REC 0 buserr 0 -> PASS
```

**Failure patterns:**

- `lost` climbing plus `BUS_OFF`: TX is not reaching RX. Check the jumper, the TXD/RXD orientation, transceiver power, and the standby pin.
- Step A passes but Step B fails: the transceiver wiring or its supply is at fault.

---

## 5. Phase 3 — CAN link (`sketches/jbd_can_reader`)

### 5.1 How JBD CAN works

This comes from JBD's "Protection board CANBUS communication protocol".

- **Link:** 500 kbit/s, **11-bit standard IDs**.
- **Query-based:** the BMS **never broadcasts**. The host sends a **remote frame** with the wanted ID, and the BMS replies with a **data frame** of the same ID.
- **Data frames** sent to the BMS are ignored.
- **Consequence:** listen-only sniffing shows nothing, and the ESP32 must run in normal mode, both to transmit the requests and to ACK the replies.
- **Byte order:** payload is big-endian.
- **CRC:** the last 2 bytes are CRC-16/MODBUS (poly 0xA001, init 0xFFFF) over the preceding bytes. The document doesn't state the CRC byte order, so the sketch checks both and prints `crc=ok(BE)`, `crc=ok(LE)`, `crc=absent(0000)` or `crc=MISMATCH`.
  - It decodes regardless, because the CAN link layer already has its own CRC-15.

| ID | DLC | Content |
|---|---|---|
| 0x100 | 8 | Pack voltage (u16, 10 mV) · current (s16, 10 mA, + = charge) · remaining capacity (u16, 10 mAh) |
| 0x101 | 8 | Full capacity (10 mAh) · cycle count · RSOC (%) |
| 0x102 | 8 | Balance bits cells 1–16 · balance bits cells 17–32 · protection flags |
| 0x103 | 8 | MOS status (bit0 CHG, bit1 DSG) · production date `(Y-2000)<<9 \| M<<5 \| D` · software version |
| 0x104 | 4 | Cell count (u8) · NTC count (u8) |
| 0x105 / 0x106 | 8 | NTC1–3 / NTC4–6, 0.1 K, °C = (raw − 2731)/10 |
| 0x107 … 0x110 | 8 | Cell voltages, 3 per frame, mV (0x107 = cells 1–3 … 0x110 = cells 28–30) |

**Protection flags**, bit 0 → 12: cell OV, cell UV, pack OV, pack UV, charge OT, charge UT, discharge OT, discharge UT, charge OC, discharge OC, short circuit, AFE error, software MOS lock.

### 5.2 User pre-checks on J6 (BMS awake, nothing connected to J6)

1. **Grounding.** DC V, black probe on B−, red on J6 pin 1 and then pin 2.
   - ≈ 2.5 V each: the CAN side is non-isolated and needs the ESP32 GND ↔ B− link. **Keep the J10 pin 1 GND wire from Phase 1 connected.**
   - ≈ 0 V or drifting: the CAN side is isolated; the GND link is harmless either way.
2. **Termination.** Only after step 1 shows both pins below 5 V. Ohms between J6 pin 1 and pin 2.
   - ≈ 120 Ω: the BMS terminates internally; together with the module's 120 Ω the bus sees ≈ 60 Ω. Correct.
   - Open or ≫ 1 kΩ: no internal termination; the module's 120 Ω alone is fine for a lead under ~1 m.

### 5.3 Wiring

Keep Phase 1's UART wiring in place: its GND is the reference, and the UART reader can be used to wake the BMS.

| Transceiver | Connects to |
|---|---|
| TXD / CTX | ESP32 GPIO4 |
| RXD / CRX | ESP32 GPIO5 |
| VCC | ESP32 3V3 |
| GND | ESP32 GND |
| CANH | BMS **J6 pin 2** (CAN-H) |
| CANL | BMS **J6 pin 1** (CAN-L) |

Use a short twisted pair for CANH/CANL.

### 5.4 Run

```bash
arduino-cli compile --fqbn "$FQBN" sketches/jbd_can_reader && \
arduino-cli upload -p "$PORT" --fqbn "$FQBN" sketches/jbd_can_reader && \
python3 tools/serial_read.py "$PORT" 25 115200 --reset
```

**What the sketch does:**

1. Listens for 2 s and reports any unsolicited frames; 0 is expected.
2. Every 2 s it queries 0x104 first, which doubles as a link check.
3. Then it queries 0x100–0x103 and 0x105, plus 0x106 if there are more than 3 NTCs.
4. Then it queries ⌈cells/3⌉ cell frames starting at 0x107.
5. Remote frames are sent **single-shot**, so an un-ACKed request is not retransmitted endlessly.

**Phase 3 is done when:**

- Cell count, NTC count and pack voltage equal the Phase 1 UART values.
- SoC and cell voltages agree within a few mV between the two links.
- `# bus RUNNING  TEC 0` holds across cycles.

### 5.5 Troubleshooting

The sketch prints one of three diagnoses after a failed 0x104 request:

| Diagnosis line | Meaning → action |
|---|---|
| `Remote frame NOT ACKed` | No CAN node on the bus. → Swap CAN-H/CAN-L at J6; confirm the BMS is awake (run `jbd_uart_reader` once, or apply a load); check termination (§5.2); confirm the transceiver works (Phase 2 Step B) |
| `Frame ACKed but no reply` | The BMS CAN controller is alive but isn't answering these IDs. → The BMS CAN address may be non-zero (datasheet default `CAN_ID_0`) or the firmware uses another CAN protocol. Report to the user; check with JBDTools/app. **Do not start sending data frames to probe it** |
| `TX never completed` | Bus stuck dominant, or the transceiver isn't driving. → Transceiver power, standby pin, TXD/RXD swapped |
| `crc=MISMATCH` on every frame, but values match UART | CRC variant differs from the document; values are still trustworthy (CAN has its own CRC-15). Note it and move on |
| `BUS_OFF` | Wiring fault or bitrate mismatch; the sketch auto-recovers. Fix the wiring |

---

## 6. Reference

### 6.1 JBD UART protocol (V4)

- **Request:** `DD A5 <reg> 00 <chk_hi> <chk_lo> 77`.
- **Response:** `DD <reg> <status 00|80> <len> <data…> <chk_hi> <chk_lo> 77`.
- **Checksum:** `0x10000 − Σ(bytes from reg/status through end of data)`.

**Register 0x03** (big-endian):

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | Pack V, 10 mV |
| 2 | 2 | Current, s16, 10 mA |
| 4 | 2 | Remaining capacity, 10 mAh |
| 6 | 2 | Nominal capacity, 10 mAh |
| 8 | 2 | Cycle count |
| 10 | 2 | Date |
| 12 | 4 | Balance bits |
| 16 | 2 | Protection flags (same bit map as CAN) |
| 18 | 1 | Software version |
| 19 | 1 | RSOC % |
| 20 | 1 | FET status (bit0 CHG, bit1 DSG) |
| 21 | 1 | Cell count |
| 22 | 1 | NTC count N |
| 23 | 2·N | NTC temperatures, 0.1 K, offset 2731 |

**Register 0x04:** mV per cell, 2 bytes each.
**Register 0x05:** ASCII hardware string.

### 6.2 Sources

- **JBD-SP24S004 datasheet**, supplied by the user: pinout p.14–15, comms p.11, sleep p.9.
- **JBD CAN protocol PDF**, attached in https://forum.arduino.cc/t/esp32-with-mcp2515-not-receiving-bms-canbus-data-id-0x100/1391590
- **JBD UART protocol V4**, as implemented and tested by https://github.com/syssi/esphome-jbd-bms
