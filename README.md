# Elespa ERI Industry Project, Group 2

Data acquisition from heterogeneous vehicle subsystems.

This repository contains ESP32-S3 firmware that reads telemetry from four
unrelated vehicle subsystems, each with its own protocol, bitrate and data
model, and normalises everything into a single versioned packet format for
downstream use.

The packet schema is the project deliverable. Other groups consume the packets,
so the schema is specified separately in
[`docs/TELEMETRY_SCHEMA.md`](docs/TELEMETRY_SCHEMA.md).

## Subsystems

| Source | Device | Interface | Status |
|---|---|---|---|
| Engine ECU | Honda Activa 6G | OBD-II over CAN, 500 kbit/s, 29-bit | Working |
| Battery | JBD SP24S007L24S100A | UART 9600 8N1, and CAN 500 kbit/s | Working |
| Battery | Daly BMS | CAN | Not implemented |
| Motor driver | Votol EM-30S | UART or CAN, undetermined | Not implemented |
| Sensor board | Group 1 hardware | Not yet defined | Not implemented |


## Hardware

| Item | Value |
|---|---|
| Board | ESP32-S3-DevKitC-1, 16 MB flash, 8 MB PSRAM |
| USB bridge | CH343, appears as `/dev/ttyACM0` |
| CAN transceiver | SN65HVD230 |
| CAN wiring | GPIO5 = TX to transceiver D, GPIO4 = RX from transceiver R |
| BMS UART | GPIO18 = RX from BMS TXD, GPIO17 = TX to BMS RXD, VDD unconnected |
| Telemetry output | GPIO15 = TX, 115200 8N1 |

## Building

Requires [PlatformIO](https://platformio.org/). The host-side tools need a few
Python packages:

```bash
pip install platformio
pip install -r tools/requirements.txt
```

```bash
pio run -e esp32s3_selftest              # build
pio run -e esp32s3_selftest -t upload    # build and flash
pio test -e native                       # run the test suite on the host
```

Five build environments are available. Each selects one application; the
environment is the only thing that decides what the board will transmit.

| Environment | Application | Bus behaviour |
|---|---|---|
| `esp32s3_selftest` | Packet pipeline self-test | Does not open a CAN controller |
| `esp32s3_activa_probe` | Bitrate discovery | Listen only, never transmits |
| `esp32s3_activa_obd` | OBD-II reader | Transmits read-only requests |
| `esp32s3` | JBD BMS reader | Transmits BMS queries |
| `esp32s3_canss` | JBD BMS reader | As above, CAN single-shot for diagnosis |

`esp32s3_activa_obd` is the default, so an accidental upload cannot put BMS
frames onto a vehicle bus.

## Running

The self-test requires no peripherals. It publishes synthetic packets of all
seven types and never opens a CAN controller, so it is safe with anything
connected.

```bash
pio run -e esp32s3_selftest -t upload
tools/telemetry_decode.py /dev/ttyACM0
```

```
engine     ecu             #1798  [simulated] rpm 1402 (min 1380 max 1448 sd 24.85 n=8)  map 53.63 kPa  load 32.30%  replies 8/8 (100%)
battery    battery_a       #1799  [simulated] 53.74 V  -3.53 A  soc 42%  spread 11 mV  cells 16/16  mos c=Y d=Y
dynamics   sensor_group_1  #3596  [simulated] ax 0.099 az 1.069 g  n=40  window 200 ms
```

Every packet from this application carries the `simulated` flag. The values are
generated on the device and are not measurements.

A web dashboard is available for the OBD-II application:

```bash
./dash esp
```

## Tests

```bash
pio test -e native      # 135 tests, ten suites, no hardware required
tools/check_decoder.py  # verifies the C++ and Python definitions agree
```

`tools/check_decoder.py` generates a byte-level capture from the C++ structures
and checks that the Python decoder reads back the same values. The capture is
committed at `test/fixtures/golden_capture.bin` and can be used to validate a
consumer written in any language.

## Repository structure

```
include/          public headers
  can/            bus profiles, bit timing, transmit allowlist
  obd/            ISO-TP framing, PID decoding
  telemetry/      packet schema, framing, publish API
src/
  apps/           one main() per build environment
  links/          UART and CAN transports
  telemetry/      encoding, statistics, mapping
test/             host test suites
tools/            host decoder, dashboard, conformance checker
docs/             specifications and bring-up notes
reference/        frozen hardware-validated sketches
```

## Documentation

| Document | Contents |
|---|---|
| [`docs/TELEMETRY_SCHEMA.md`](docs/TELEMETRY_SCHEMA.md) | Packet schema, wire format, publish API |
| [`docs/ECU_schema.md`](docs/ECU_schema.md) | The 17 ECU parameters, with formulas and observed values |
| [`docs/activa-can.md`](docs/activa-can.md) | Vehicle CAN bus, OBD-II connector, access rules |
| [`docs/VOTOL_EM30S_study.md`](docs/VOTOL_EM30S_study.md) | Motor driver interface study |
| [`docs/context.md`](docs/context.md) | Project background and bring-up history |

## Safety

Vehicle access is read-only, enforced in code.

- Only OBD-II services 01, 02, 03, 07, 09 and 0A can be encoded. Clear-DTC,
  ECU reset, session control and security access are not representable.
- Every transmitted frame is checked against a per-profile identifier
  allowlist before it reaches the controller.
- No write command is ever sent to the BMS.

When the board is connected to a vehicle diagnostic connector, the host computer
must run on battery. Mains earth and vehicle chassis are separate references and
connecting both creates a ground loop through the CAN transceiver.

