#!/usr/bin/env python3
"""Cross-language contract test for the telemetry schema.

include/telemetry/ describes the packet layouts in C++; tools/telemetry_decode.py
describes them a second time in struct format strings. Two descriptions of the
same bytes drift, and the failure is silent - a wrong offset shifts every field
after it and they all still look like plausible numbers.

This rebuilds the fixture from the real C++ structs and asserts the decoder
reports the exact values that were written. Run it after touching either side:

    tools/check_decoder.py             # rebuild and check
    tools/check_decoder.py --no-build  # check the committed fixture only
"""

import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FIXTURE = os.path.join(ROOT, "test", "fixtures", "golden_capture.bin")

SOURCES = [
    "tools/make_golden_capture.cpp",
    "src/crc16.cpp",
    "src/telemetry/packet.cpp",
    "src/telemetry/framing.cpp",
    "src/telemetry/publisher.cpp",
    "src/telemetry/stat_block.cpp",
]

sys.path.insert(0, HERE)
from telemetry_decode import cobs_decode, decode_packet  # noqa: E402

failures = []


def check(label, got, want, tol=None):
    if tol is not None and isinstance(got, (int, float)) and isinstance(want, (int, float)):
        ok = abs(got - want) <= tol
    else:
        ok = got == want
    if not ok:
        failures.append(f"{label}: got {got!r}, expected {want!r}")
    return ok


def build(out_path: str) -> bool:
    cmd = ["c++", "-std=c++17", "-I", "include", "-o", out_path] + SOURCES
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stderr, file=sys.stderr)
        return False
    with open(FIXTURE, "wb") as fh:
        g = subprocess.run([out_path], cwd=ROOT, stdout=fh, stderr=subprocess.PIPE, text=True)
    if g.returncode != 0:
        print(g.stderr, file=sys.stderr)
        return False
    return True


def read_packets(path: str) -> list:
    data = open(path, "rb").read()
    packets = []
    for frame in data.split(b"\x00"):
        if not frame:
            continue
        body = cobs_decode(frame)
        if body is None:
            failures.append("a frame in the fixture is not valid COBS")
            continue
        pkt = decode_packet(body)
        if pkt is None:
            failures.append("a frame in the fixture failed header/CRC verification")
            continue
        if "error" in pkt:
            failures.append(pkt["error"])
            continue
        packets.append(pkt)
    return packets


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-build", action="store_true", help="check the committed fixture only")
    args = ap.parse_args()

    tmp = os.environ.get("TMPDIR", "/tmp")
    if not args.no_build:
        exe = os.path.join(tmp, "elespa_golden")
        if not build(exe):
            print("FAIL: could not build the golden generator", file=sys.stderr)
            return 1

    packets = read_packets(FIXTURE)
    check("packet count", len(packets), 7)
    if len(packets) != 7:
        for f in failures:
            print("  " + f, file=sys.stderr)
        return 1

    by_type = {p["type"]: p for p in packets}
    check("the seven types", sorted(by_type), sorted(
        ["identity", "engine", "motor", "dynamics", "navigation", "battery", "status_event"]))

    # Every packet must carry the right envelope.
    for p in packets:
        check(f"{p['type']} schema", p["schema"], 2)
        check(f"{p['type']} payload size", p["payload_bytes"], {
            "identity": 112, "engine": 184, "motor": 152, "dynamics": 308,
            "navigation": 128, "battery": 128, "status_event": 64,
        }[p["type"]])

    # ---- identity ----------------------------------------------------------
    i = by_type["identity"]["payload"]
    check("identity.vin", i["vin"], "GOLDENVIN00000001")
    check("identity.ecu_name", i["ecu_name"], "ECM -EngineControl")
    check("identity.calibration_id", i["calibration_id"], "0105411002")
    check("identity.motor_model", i["motor_model"], "EM-30S")
    check("identity.cvn", i["cvn"], "0xDEADBEEF")
    check("identity.ecu_bitrate", i["ecu_bitrate"], 500000)
    check("identity.29_bit", i["ecu_29_bit_addressing"], True)
    check("identity.obd_standard_raw", i["obd_standard_raw"], 0x25)
    check("identity.obd note", i["obd_standard_note"], "outside ISO 15031-5")
    check("identity.battery_vendor", i["battery_vendor"], "jbd")
    check("identity.motor_bitrate absent", i["motor_bitrate"], None)

    # ---- engine ------------------------------------------------------------
    e = by_type["engine"]["payload"]
    check("engine.rpm.min", e["rpm"]["min"], 1480.0, 0.01)
    check("engine.rpm.max", e["rpm"]["max"], 1520.0, 0.01)
    check("engine.rpm.mean", e["rpm"]["mean"], 1502.0, 0.01)
    check("engine.rpm.stddev", e["rpm"]["stddev"], 2.5, 0.01)
    check("engine.rpm.samples", e["rpm"]["samples"], 8)
    check("engine.map.mean", e["map_kpa"]["mean"], 70.5, 0.01)
    check("engine.intake_air_c", e["intake_air_c"], 31.0, 0.01)
    check("engine.oil_temp_c", e["oil_temp_c"], 78.5, 0.01)
    check("engine.requests", e["requests"], 20)
    check("engine.replies", e["replies"], 17)
    check("engine.reply_rate", e["reply_rate"], 0.85, 0.001)
    check("engine.crc_errors", e["crc_errors"], 3)
    check("engine.fuel_system", e["fuel_system_status"], [2, 0])
    check("engine.fields", sorted(e["fields"]), sorted(
        ["rpm", "throttle", "map", "timing", "load", "intake_temp", "oil_temp",
         "fuel_trim", "fuel_system"]))

    # ---- motor -------------------------------------------------------------
    m = by_type["motor"]["payload"]
    check("motor.rpm.mean", m["shaft_rpm"]["mean"], 820.0, 0.01)
    check("motor.current.mean", m["current_amps"]["mean"], 12.0, 0.01)
    check("motor.bus_volts", m["bus_volts"], 60.4, 0.01)
    check("motor.controller_temp_c", m["controller_temp_c"], 42.5, 0.01)
    check("motor.motor_temp_c", m["motor_temp_c"], 51.1, 0.01)
    check("motor.state", m["state"], "run")
    check("motor.link", m["link"], "can")
    check("motor.fault_flags", m["fault_flags"], "0x00001010")
    # An absent channel must report no samples, never a zero.
    check("motor.torque absent", m["torque_nm"], {"samples": 0})

    # ---- dynamics ----------------------------------------------------------
    d = by_type["dynamics"]["payload"]
    check("dynamics.accel_z.mean", d["accel_z"]["mean"], 1.0, 0.001)
    check("dynamics.accel_x.samples", d["accel_x"]["samples"], 40)
    check("dynamics.wheel_speed.mean", d["wheel_speed_kph"]["mean"], 18.5, 0.01)
    check("dynamics.wheel_rpm", d["wheel_rpm"], 310.5, 0.01)
    check("dynamics.odometer_m", d["odometer_m"], 1234.5, 0.01)
    check("dynamics.circumference", d["wheel_circumference_m"], 1.256, 0.001)
    check("dynamics.jerk absent", d["jerk_msss"], {"samples": 0})
    check("dynamics.tilt absent", d["tilt_deg"], {"samples": 0})

    # ---- navigation --------------------------------------------------------
    n = by_type["navigation"]["payload"]
    # The whole point of scaled integers: this must be exact to 1e-7, which a
    # float latitude cannot manage.
    check("navigation.latitude", n["latitude"], 18.4567891, 1e-9)
    check("navigation.longitude", n["longitude"], 73.8765432, 1e-9)
    check("navigation.gps_altitude_m", n["gps_altitude_m"], 560.25, 0.01)
    check("navigation.baro_altitude_m", n["baro_altitude_m"], 558.75, 0.01)
    check("navigation.pressure_hpa", n["pressure_hpa"], 948.6, 0.01)
    check("navigation.heading_deg", n["heading_deg"], 137.5, 0.01)
    check("navigation.utc_seconds", n["utc_seconds"], 1762000000)
    check("navigation.ecu_run_time_s", n["ecu_run_time_s"], 3600)
    check("navigation.distance_since_clear", n["ecu_distance_since_clear_km"], 4521)
    check("navigation.satellites", n["satellites"], 9)
    check("navigation.fix_quality", n["fix_quality"], 1)

    # ---- battery -----------------------------------------------------------
    b = by_type["battery"]["payload"]
    check("battery.pack_volts", b["pack_volts"], 53.4, 0.01)
    check("battery.current.mean", b["current_amps"]["mean"], -2.5, 0.01)
    check("battery.soc", b["soc_percent"], 42)
    check("battery.soh absent", b["soh_percent"], None)
    check("battery.cycle_count", b["cycle_count"], 12)
    check("battery.configured", b["cell_count_configured"], 17)
    check("battery.live", b["live_cell_count"], 15)
    # Channel 10 holds 0 mV and its presence bit is clear, so it must be
    # reported as absent - not as a cell sitting at zero volts.
    # Only channel 10 is listed: it IS carried and reads 0 mV with its presence
    # bit clear. Channel 17 is not in the array at all, which the truncation
    # warning below reports instead - a different fact, reported differently.
    check("battery.channel 10 absent", b["cells_absent"], [10])
    check("battery.channel 10 not in cells", 10 in b["cells_mv"], False)
    check("battery.channel 1 mv", b["cells_mv"].get(1), 3330)
    check("battery.channel 16 mv", b["cells_mv"].get(16), 3345)
    check("battery.cells reported", len(b["cells_mv"]), 15)
    check("battery.spread", b["cell_spread_mv"], 15)
    check("battery.min channel", b["cell_min_channel"], 1)
    check("battery.max channel", b["cell_max_channel"], 16)
    check("battery.temps", len(b["temps_c"]), 5)
    check("battery.temp 1", b["temps_c"].get(1), 25.0)
    check("battery.temp 5", b["temps_c"].get(5), 29.0)
    check("battery.temp_min_c", b["temp_min_c"], 25.0)
    check("battery.temp_max_c", b["temp_max_c"], 29.0)
    check("battery.balancing", b["balancing"], [1, 3])
    check("battery.protection", b["protection"], ["cell_uv"])
    check("battery.mosfet charge", b["mosfet_charge"], True)
    check("battery.mosfet discharge", b["mosfet_discharge"], False)
    check("battery.vendor", b["vendor"], "jbd")
    check("battery.chemistry", b["chemistry"], "lifepo4")
    # 17 configured channels into a 16-wide schema must be flagged, not hidden.
    check("battery.truncation warning", "warning" in b, True)

    # ---- status event ------------------------------------------------------
    s = by_type["status_event"]["payload"]
    check("status.dtcs", s["dtcs"], ["P0145", "U034D"])
    check("status.stored", s["stored_dtc_count"], 2)
    check("status.pending", s["pending_dtc_count"], 1)
    check("status.mil", s["mil_on"], True)
    check("status.monitor", s["ecu_monitor_status"], "0x00070000")
    check("status.battery_protection", s["battery_protection"], ["cell_uv"])
    check("status.motor_state", s["motor_state"], "fault")
    check("status.rfid_uid", s["rfid_uid"], "047a2b19c35e80")
    check("status.rfid_authorised", s["rfid_authorised"], True)
    check("status.proximity_mm", s["proximity_mm"], 450)
    check("status.presence", s["presence_detected"], True)
    check("status.events", sorted(s["events"]), sorted(
        ["dtc_appeared", "mil_changed", "battery_fault", "rfid_presented"]))

    if failures:
        print(f"FAIL: {len(failures)} mismatch(es) between the C++ structs and the decoder:",
              file=sys.stderr)
        for f in failures:
            print("  " + f, file=sys.stderr)
        return 1

    print("OK: the decoder agrees with the C++ structs on all 7 packet types")
    return 0


if __name__ == "__main__":
    sys.exit(main())
