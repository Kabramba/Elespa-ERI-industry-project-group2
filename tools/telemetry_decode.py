#!/usr/bin/env python3
"""Decode the Elespa ERI telemetry packet stream.

This is the reference consumer. It is deliberately dependency-free apart from
pyserial, and it is deliberately a mirror of include/telemetry/ rather than a
clever generic parser: the struct format strings below are the schema, written
out a second time in a different language. If the firmware and this file
disagree, one of them is wrong, and a size mismatch says so immediately instead
of silently shifting every field.

    tools/telemetry_decode.py /dev/ttyACM0            # live from a board
    tools/telemetry_decode.py capture.bin --file      # a saved byte stream
    tools/telemetry_decode.py /dev/ttyACM0 --json     # one JSON object a line
    tools/telemetry_decode.py /dev/ttyACM0 --raw capture.bin   # tee the bytes

Schema version 2. Little-endian, which is native on the ESP32-S3 and on any
x86/ARM host, so '<' here and a memcpy there describe the same bytes.
"""

import argparse
import json
import math
import struct
import sys
import time

SCHEMA_VERSION = 2
MAGIC = 0xE15A
DELIMITER = 0x00

# ---------------------------------------------------------------------------
# CRC-16/MODBUS, the same function as include/crc16.h
# ---------------------------------------------------------------------------


def crc16_modbus(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


# ---------------------------------------------------------------------------
# COBS
# ---------------------------------------------------------------------------


def cobs_decode(frame: bytes) -> bytes | None:
    """Inverse of cobsEncode(). None if the body is not valid COBS."""
    out = bytearray()
    i = 0
    n = len(frame)
    while i < n:
        code = frame[i]
        i += 1
        if code == 0:
            return None  # a zero cannot occur inside a body
        run = code - 1
        if i + run > n:
            return None  # the run claims more bytes than the frame holds
        out += frame[i : i + run]
        i += run
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


# ---------------------------------------------------------------------------
# The envelope
# ---------------------------------------------------------------------------

HEADER = struct.Struct("<HBBHHIBBH")
assert HEADER.size == 16, "header must stay 16 bytes"

PACKET_TYPES = {
    0x01: "identity",
    0x10: "engine",
    0x11: "motor",
    0x12: "dynamics",
    0x13: "navigation",
    0x20: "battery",
    0x30: "status_event",
}

SOURCES = {
    0: "unknown",
    1: "ecu",
    2: "battery_a",
    3: "battery_b",
    4: "motor_driver",
    5: "sensor_group_1",
    6: "fusion",
}

FLAGS = {1: "partial", 2: "degraded", 4: "simulated"}


def decode_header(buf: bytes) -> dict | None:
    if len(buf) < HEADER.size:
        return None
    magic, ver, ptype, plen, seq, uptime, source, flags, crc = HEADER.unpack_from(buf)
    if magic != MAGIC or ver != SCHEMA_VERSION:
        return None
    if HEADER.size + plen != len(buf):
        return None

    # The CRC is computed with its own field zeroed, so it can be recomputed
    # over the buffer in place - exactly what src/telemetry/packet.cpp does.
    blanked = bytearray(buf)
    blanked[14:16] = b"\x00\x00"
    if crc16_modbus(bytes(blanked)) != crc:
        return None

    return {
        "type": PACKET_TYPES.get(ptype, f"unknown(0x{ptype:02X})"),
        "type_id": ptype,
        "schema": ver,
        "payload_bytes": plen,
        "seq": seq,
        "uptime_ms": uptime,
        "source": SOURCES.get(source, f"unknown({source})"),
        "flags": [name for bit, name in FLAGS.items() if flags & bit],
    }


# ---------------------------------------------------------------------------
# StatBlock - 32 bytes, the unit of every fast channel
# ---------------------------------------------------------------------------

STATBLOCK = struct.Struct("<6fIHH")
assert STATBLOCK.size == 32, "StatBlock must stay 32 bytes"


def read_stat(buf: bytes, off: int) -> tuple[dict, int]:
    lo, hi, mean, sd, last, slope, window, samples, _ = STATBLOCK.unpack_from(buf, off)
    if samples == 0:
        # NaN with no samples is how the schema says "this channel was absent".
        # Reporting it as None keeps a consumer from plotting it as zero.
        return ({"samples": 0}, off + STATBLOCK.size)
    return (
        {
            "min": lo,
            "max": hi,
            "mean": mean,
            "stddev": sd,
            "last": last,
            "slope_per_s": slope,
            "window_ms": window,
            "samples": samples,
        },
        off + STATBLOCK.size,
    )


def bits(value: int, names: dict) -> list:
    return [n for b, n in names.items() if value & b]


# ---------------------------------------------------------------------------
# Payloads. One function per packet type, each asserting its own size.
# ---------------------------------------------------------------------------

ENGINE_FIELDS = {
    1: "rpm", 2: "throttle", 4: "map", 8: "timing", 16: "load",
    32: "intake_temp", 64: "oil_temp", 128: "fuel_trim", 256: "fuel_system",
}


def decode_engine(p: bytes) -> dict:
    off = 0
    out = {}
    for name in ("rpm", "throttle_percent", "map_kpa", "timing_advance_deg", "load_percent"):
        out[name], off = read_stat(p, off)
    trim, window, intake, oil, req, rep, fields, fs0, fs1, crcerr, _ = struct.unpack_from(
        "<fIhhHHHBBBB", p, off
    )
    out.update(
        short_fuel_trim_percent=trim,
        window_ms=window,
        intake_air_c=intake / 10.0,
        oil_temp_c=oil / 10.0,
        requests=req,
        replies=rep,
        reply_rate=(rep / req) if req else None,
        fuel_system_status=[fs0, fs1],
        crc_errors=crcerr,
        fields=bits(fields, ENGINE_FIELDS),
    )
    return out


MOTOR_FIELDS = {
    1: "rpm", 2: "current", 4: "torque", 8: "bus_volts",
    16: "ctrl_temp", 32: "motor_temp", 64: "throttle",
}
MOTOR_STATES = {0: "idle", 1: "init", 2: "start", 3: "run",
                4: "stop", 5: "brake", 6: "wait", 7: "fault"}
MOTOR_LINKS = {0: "unknown", 1: "uart", 2: "can"}


def decode_motor(p: bytes) -> dict:
    off = 0
    out = {}
    for name in ("shaft_rpm", "current_amps", "torque_nm", "throttle_percent"):
        out[name], off = read_stat(p, off)
    volts, window, faults, ctemp, mtemp, fields, state, link = struct.unpack_from(
        "<fIIhhHBB", p, off
    )
    out.update(
        bus_volts=volts,
        window_ms=window,
        fault_flags=f"0x{faults:08X}",
        controller_temp_c=ctemp / 10.0,
        motor_temp_c=mtemp / 10.0,
        state=MOTOR_STATES.get(state, state),
        link=MOTOR_LINKS.get(link, link),
        fields=bits(fields, MOTOR_FIELDS),
    )
    return out


DYNAMICS_FIELDS = {
    1: "accel", 2: "attitude", 4: "wheel", 8: "long_accel", 16: "jerk", 32: "roll_risk",
}


def decode_dynamics(p: bytes) -> dict:
    off = 0
    out = {}
    for name in ("accel_x", "accel_y", "accel_z", "pitch_deg", "roll_deg", "tilt_deg",
                 "wheel_speed_kph", "long_accel_mss", "jerk_msss"):
        out[name], off = read_stat(p, off)
    rpm, odo, circ, window, fields, risk, _ = struct.unpack_from("<fffIHBB", p, off)
    out.update(
        wheel_rpm=rpm,
        odometer_m=odo,
        wheel_circumference_m=circ,
        window_ms=window,
        roll_risk=bool(risk),
        fields=bits(fields, DYNAMICS_FIELDS),
    )
    return out


NAV_FIELDS = {
    1: "fix", 2: "position", 4: "gps_speed", 8: "ecu_speed", 16: "heading",
    32: "gps_alt", 64: "baro", 128: "incline", 256: "odometry", 512: "utc",
}


def decode_navigation(p: bytes) -> dict:
    off = 0
    out = {}
    out["gps_speed_kph"], off = read_stat(p, off)
    out["ecu_speed_kph"], off = read_stat(p, off)
    (lat, lon, galt, balt, hpa, amb, head, incl, dist, utc, runtime,
     dmil, dclear, fields, sats, fix) = struct.unpack_from("<iifffffffIIHHHBB", p, off)
    out.update(
        # 1e-7 degrees per LSB: exact, and about 11 mm. Dividing here is the
        # only place the value becomes a float, after any comparison is done.
        latitude=lat / 1e7,
        longitude=lon / 1e7,
        gps_altitude_m=galt,
        baro_altitude_m=balt,
        pressure_hpa=hpa,
        ambient_temp_c=amb,
        heading_deg=head,
        vertical_incline_pct=incl,
        gps_total_distance_km=dist,
        utc_seconds=utc,
        ecu_run_time_s=runtime,
        ecu_distance_with_mil_km=dmil,
        ecu_distance_since_clear_km=dclear,
        satellites=sats,
        fix_quality=fix,
        fields=bits(fields, NAV_FIELDS),
    )
    return out


BATTERY_FIELDS = {
    1: "pack", 2: "current", 4: "capacity", 8: "cells",
    16: "temps", 32: "protection", 64: "mosfet", 128: "balance",
}
PROTECTION = {
    1: "cell_ov", 2: "cell_uv", 4: "pack_ov", 8: "pack_uv",
    16: "chg_ot", 32: "chg_ut", 64: "dsg_ot", 128: "dsg_ut",
    256: "chg_oc", 512: "dsg_oc", 1024: "short", 2048: "afe_error", 4096: "mos_lock",
}
VENDORS = {0: "unknown", 1: "jbd", 2: "daly"}
CHEMISTRY = {0: "unknown", 1: "lifepo4", 2: "nmc", 3: "lto"}


def decode_battery(p: bytes) -> dict:
    out = {}
    out["current_amps"], off = read_stat(p, 0)
    volts, remain, full, present, balancing = struct.unpack_from("<fffII", p, off)
    off += 20
    cells = list(struct.unpack_from("<16H", p, off))
    off += 32
    temps = list(struct.unpack_from("<8h", p, off))
    off += 16
    (cmin, cmax, cspread, tmin, tmax, cycles, prot, fields, vendor, chem, ccount,
     live, tcount, tmask, cminix, cmaxix, soc, soh, mos, _) = struct.unpack_from(
        "<HHHhhHHH12B", p, off
    )

    # A channel is only reported if its presence bit is set. This is the whole
    # reason the mask exists: a dropped frame leaves 0 mV behind, and 0 mV is
    # indistinguishable from a real reading unless the bit is checked.
    out.update(
        pack_volts=volts,
        remaining_ah=remain,
        full_ah=full,
        soc_percent=soc,
        soh_percent=soh or None,
        cycle_count=cycles,
        cell_count_configured=ccount,
        live_cell_count=live,
        cells_mv={i + 1: cells[i] for i in range(16) if present & (1 << i)},
        cells_absent=[i + 1 for i in range(min(ccount, 16)) if not present & (1 << i)],
        cell_min_mv=cmin or None,
        cell_max_mv=cmax or None,
        cell_spread_mv=cspread if cmin else None,
        cell_min_channel=cminix + 1 if cmin else None,
        cell_max_channel=cmaxix + 1 if cmin else None,
        temps_c={i + 1: temps[i] / 10.0 for i in range(8) if tmask & (1 << i)},
        temp_min_c=tmin / 10.0 if tmask else None,
        temp_max_c=tmax / 10.0 if tmask else None,
        balancing=[i + 1 for i in range(32) if balancing & (1 << i)],
        protection=bits(prot, PROTECTION),
        mosfet_charge=bool(mos & 1),
        mosfet_discharge=bool(mos & 2),
        vendor=VENDORS.get(vendor, vendor),
        chemistry=CHEMISTRY.get(chem, chem),
        fields=bits(fields, BATTERY_FIELDS),
    )
    if ccount > 16:
        out["warning"] = (
            f"BMS configured for {ccount} channels; the schema carries 16"
        )
    return out


IDENTITY_FIELDS = {1: "vehicle", 2: "ecu", 4: "battery", 8: "motor", 16: "sensors"}


def decode_identity(p: bytes) -> dict:
    vin, ecu, cal, motor = struct.unpack_from("<20s24s20s16s", p, 0)
    off = 80
    (cvn, ecub, batb, motb, fields, ext, std, vendor, bcells, mlink,
     sensors) = struct.unpack_from("<IIIIHBBBBBB", p, off)

    def text(b: bytes) -> str:
        return b.split(b"\x00", 1)[0].decode("ascii", "replace")

    return {
        "vin": text(vin),
        "ecu_name": text(ecu),
        "calibration_id": text(cal),
        "motor_model": text(motor),
        "cvn": f"0x{cvn:08X}",
        "ecu_bitrate": ecub,
        "battery_bitrate": batb,
        "motor_bitrate": motb or None,
        "ecu_29_bit_addressing": bool(ext),
        # 0x25 is outside ISO 15031-5, whose highest defined value is 35. The
        # raw byte is carried rather than mapped to a standard it does not match.
        "obd_standard_raw": std,
        "obd_standard_note": "outside ISO 15031-5" if std > 35 else None,
        "battery_vendor": VENDORS.get(vendor, vendor),
        "battery_cell_count": bcells or None,
        "motor_link": MOTOR_LINKS.get(mlink, mlink),
        "sensor_board_present": bool(sensors),
        "fields": bits(fields, IDENTITY_FIELDS),
    }


STATUS_FIELDS = {
    1: "ecu_dtc", 2: "ecu_monitors", 4: "battery", 8: "motor",
    16: "rfid", 32: "proximity", 64: "roll_risk",
}
EVENTS = {
    1: "dtc_appeared", 2: "dtc_cleared", 4: "mil_changed", 8: "battery_fault",
    16: "mosfet_changed", 32: "motor_fault", 64: "motor_state_change",
    128: "rfid_presented", 256: "rfid_rejected", 512: "presence_changed",
    1024: "roll_risk_raised",
}


def dtc_text(code: int) -> str:
    """Packed 16-bit DTC to the printed form, per ISO 15031-5."""
    letter = "PCBU"[(code >> 14) & 0x3]
    return f"{letter}{(code >> 12) & 0x3}{(code >> 8) & 0xF:X}{(code >> 4) & 0xF:X}{code & 0xF:X}"


def decode_status(p: bytes) -> dict:
    mon, moncycle, mfault = struct.unpack_from("<III", p, 0)
    dtcs = list(struct.unpack_from("<8H", p, 12))
    (bprot, prox, events, fields) = struct.unpack_from("<HHHH", p, 28)
    uid = p[36:46]
    (uidlen, auth, listed, stored, pending, mil, std, bmos, mstate, presence,
     risk) = struct.unpack_from("<11B", p, 46)

    return {
        "ecu_monitor_status": f"0x{mon:08X}",
        "ecu_monitor_this_cycle": f"0x{moncycle:08X}",
        "motor_fault_flags": f"0x{mfault:08X}",
        "dtcs": [dtc_text(d) for d in dtcs[:listed]],
        "stored_dtc_count": stored,
        "pending_dtc_count": pending,
        "mil_on": bool(mil),
        "battery_protection": bits(bprot, PROTECTION),
        "battery_mosfet_charge": bool(bmos & 1),
        "battery_mosfet_discharge": bool(bmos & 2),
        "motor_state": MOTOR_STATES.get(mstate, mstate),
        "rfid_uid": uid[:uidlen].hex() if uidlen else None,
        "rfid_authorised": bool(auth) if uidlen else None,
        "proximity_mm": prox or None,
        "presence_detected": bool(presence),
        "roll_risk": bool(risk),
        "obd_standard_raw": std,
        "events": bits(events, EVENTS),
        "fields": bits(fields, STATUS_FIELDS),
    }


DECODERS = {
    "identity": (decode_identity, 112),
    "engine": (decode_engine, 184),
    "motor": (decode_motor, 152),
    "dynamics": (decode_dynamics, 308),
    "navigation": (decode_navigation, 128),
    "battery": (decode_battery, 128),
    "status_event": (decode_status, 64),
}


def decode_packet(buf: bytes) -> dict | None:
    head = decode_header(buf)
    if head is None:
        return None
    entry = DECODERS.get(head["type"])
    if entry is None:
        head["payload"] = {"undecoded_hex": buf[HEADER.size :].hex()}
        return head
    fn, expect = entry
    payload = buf[HEADER.size :]
    if len(payload) != expect:
        # A size disagreement means the firmware and this file are built from
        # different schema revisions. Say so rather than unpacking garbage.
        head["error"] = f"{head['type']} payload is {len(payload)} bytes, expected {expect}"
        return head
    head["payload"] = fn(payload)
    return head


# ---------------------------------------------------------------------------
# Presentation
# ---------------------------------------------------------------------------


def fmt(value, places=2) -> str:
    if isinstance(value, float):
        if math.isnan(value):
            return "nan"
        return f"{value:.{places}f}"
    return str(value)


def summarise(pkt: dict) -> str:
    p = pkt.get("payload", {})
    t = pkt["type"]
    sim = " SIM" if "simulated" in pkt["flags"] else ""

    if t == "engine":
        rpm = p.get("rpm", {})
        rate = p.get("reply_rate")
        return (
            f"rpm {fmt(rpm.get('mean'), 0)} (min {fmt(rpm.get('min'), 0)} "
            f"max {fmt(rpm.get('max'), 0)} sd {fmt(rpm.get('stddev'))} n={rpm.get('samples')})  "
            f"map {fmt(p.get('map_kpa', {}).get('mean'))} kPa  "
            f"load {fmt(p.get('load_percent', {}).get('mean'))}%  "
            f"replies {p.get('replies')}/{p.get('requests')}"
            + (f" ({rate:.0%})" if rate is not None else "")
        )
    if t == "battery":
        return (
            f"{fmt(p.get('pack_volts'))} V  {fmt(p.get('current_amps', {}).get('last'))} A  "
            f"soc {p.get('soc_percent')}%  "
            f"spread {p.get('cell_spread_mv')} mV  "
            f"cells {p.get('live_cell_count')}/{p.get('cell_count_configured')}  "
            f"mos c={'Y' if p.get('mosfet_charge') else 'n'} "
            f"d={'Y' if p.get('mosfet_discharge') else 'n'}"
            + (f"  PROT {','.join(p['protection'])}" if p.get("protection") else "")
        )
    if t == "dynamics":
        ax, az = p.get("accel_x", {}), p.get("accel_z", {})
        return (
            f"ax {fmt(ax.get('mean'), 3)} az {fmt(az.get('mean'), 3)} g  "
            f"n={ax.get('samples')}  window {p.get('window_ms')} ms"
        )
    if t == "motor":
        return (
            f"rpm {fmt(p.get('shaft_rpm', {}).get('mean'), 0)}  "
            f"{fmt(p.get('current_amps', {}).get('mean'))} A  "
            f"{fmt(p.get('bus_volts'))} V  state {p.get('state')}  link {p.get('link')}"
        )
    if t == "navigation":
        if not p.get("fields"):
            return "no fix, no source yet (every field absent)"
        return f"{fmt(p.get('latitude'), 6)},{fmt(p.get('longitude'), 6)}  sats {p.get('satellites')}"
    if t == "identity":
        return (
            f"vin {p.get('vin') or '-'}  ecu {p.get('ecu_name') or '-'}  "
            f"{p.get('ecu_bitrate')} bit/s  obd 0x{p.get('obd_standard_raw', 0):02X}"
        )
    if t == "status_event":
        parts = []
        if p.get("events"):
            parts.append("events " + ",".join(p["events"]))
        if p.get("dtcs"):
            parts.append("dtc " + ",".join(p["dtcs"]))
        if p.get("battery_protection"):
            parts.append("prot " + ",".join(p["battery_protection"]))
        parts.append(f"mil {'Y' if p.get('mil_on') else 'n'}")
        return "  ".join(parts)
    return sim


# ---------------------------------------------------------------------------
# Byte sources
# ---------------------------------------------------------------------------


def stream_from_file(path: str):
    with open(path, "rb") as fh:
        while True:
            chunk = fh.read(4096)
            if not chunk:
                return
            yield chunk


def stream_from_serial(port: str, baud: int):
    try:
        import serial  # type: ignore
    except ImportError:
        sys.exit("pyserial is not installed: pip install pyserial, or use --file")
    with serial.Serial(port, baud, timeout=0.2) as ser:
        while True:
            waiting = ser.in_waiting or 1
            chunk = ser.read(waiting)
            if chunk:
                yield chunk


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", help="serial port, or a file with --file")
    ap.add_argument("--file", action="store_true", help="read a saved byte stream")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--json", action="store_true", help="one JSON object per line")
    ap.add_argument("--raw", metavar="PATH", help="also write the raw bytes here")
    ap.add_argument("--count", type=int, default=0, help="stop after N packets")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after N seconds")
    ap.add_argument("--types", help="comma-separated list to show, e.g. battery,engine")
    args = ap.parse_args()

    wanted = set(args.types.split(",")) if args.types else None
    source = stream_from_file(args.source) if args.file else stream_from_serial(
        args.source, args.baud
    )
    tee = open(args.raw, "wb") if args.raw else None

    buf = bytearray()
    stats = {"accepted": 0, "framing": 0, "crc": 0, "bytes": 0}
    seen_seq: dict[str, int] = {}
    gaps = 0
    started = time.time()

    if not args.json:
        print(f"# schema v{SCHEMA_VERSION}  waiting for frames on {args.source}")

    try:
        for chunk in source:
            stats["bytes"] += len(chunk)
            if tee:
                tee.write(chunk)
                tee.flush()

            for byte in chunk:
                if byte != DELIMITER:
                    if len(buf) < 4096:
                        buf.append(byte)
                    continue

                frame, buf = bytes(buf), bytearray()
                if not frame:
                    continue

                body = cobs_decode(frame)
                if body is None:
                    stats["framing"] += 1
                    continue

                pkt = decode_packet(body)
                if pkt is None:
                    stats["crc"] += 1
                    continue

                stats["accepted"] += 1

                # Sequence numbers are per SOURCE, not per source-and-type:
                # two packet types from one source share the counter, so keying
                # this on the type would report a gap on every packet. A real
                # gap means loss between the board and here, which is worth
                # knowing separately from a CRC failure - one is loss, one is
                # corruption.
                key = pkt["source"]
                prev = seen_seq.get(key)
                if prev is not None and pkt["seq"] != (prev + 1) & 0xFFFF:
                    gaps += 1
                    pkt["gap_after_seq"] = prev
                seen_seq[key] = pkt["seq"]

                if wanted and pkt["type"] not in wanted:
                    continue

                if args.json:
                    print(json.dumps(pkt, default=str), flush=True)
                else:
                    flags = ("[" + ",".join(pkt["flags"]) + "]") if pkt["flags"] else ""
                    gap = "  << GAP" if "gap_after_seq" in pkt else ""
                    print(
                        f"{pkt['uptime_ms']/1000:9.3f}  {pkt['type']:<13}"
                        f"{pkt['source']:<15} #{pkt['seq']:<5} {flags:<12}"
                        f"{summarise(pkt)}{gap}",
                        flush=True,
                    )
                    if "error" in pkt:
                        print(f"           ERROR {pkt['error']}", flush=True)

                if args.count and stats["accepted"] >= args.count:
                    raise KeyboardInterrupt
            if args.seconds and time.time() - started > args.seconds:
                raise KeyboardInterrupt
    except KeyboardInterrupt:
        pass
    finally:
        if tee:
            tee.close()

    elapsed = time.time() - started
    print(
        f"\n# {stats['accepted']} packets, {stats['framing']} framing errors, "
        f"{stats['crc']} CRC/header rejects, {gaps} sequence gaps, "
        f"{stats['bytes']} bytes in {elapsed:.1f} s",
        file=sys.stderr,
    )
    return 0 if stats["accepted"] else 1


if __name__ == "__main__":
    sys.exit(main())
