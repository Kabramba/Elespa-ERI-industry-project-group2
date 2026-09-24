#!/usr/bin/env python3
"""Live terminal dashboard for the Activa ECU.

    ./tools/activa_dashboard.py /dev/ttyACM1

The firmware emits frames; this does the decoding and the display. It reads the
'T'/'R' lines the esp32s3_activa_obd application prints, reassembles ISO-TP,
decodes OBD-II Service 01 PIDs, and shows live values, their rate of change,
sparkline history and the raw stream.

Only the activa-obd application is supported - the dashboard says so plainly
rather than silently misreading some other firmware's output.

Keys:  q quit   r reset history   p pause raw pane
"""
import argparse
import curses
import re
import sys
import threading
import time
from collections import deque

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  sudo apt install python3-serial")

# --------------------------------------------------------------------------
# OBD-II Service 01 decode. Formulas from docs/activa-can.md s6.6.
# A, B, C, D are the bytes after the PID. Anything absent here is still shown
# raw - guessing a scaling is worse than admitting we do not have one.
# --------------------------------------------------------------------------
PIDS = {
    0x04: ("Engine load",    "%",    lambda b: b[0] * 100 / 255),
    0x05: ("Engine temp",    "C",    lambda b: b[0] - 40),
    0x06: ("Short fuel trim","%",    lambda b: b[0] * 100 / 128 - 100),
    0x07: ("Long fuel trim", "%",    lambda b: b[0] * 100 / 128 - 100),
    0x0B: ("MAP",            "kPa",  lambda b: b[0]),
    0x0C: ("RPM",            "rpm",  lambda b: (256 * b[0] + b[1]) / 4),
    0x0D: ("Speed",          "km/h", lambda b: b[0]),
    0x0E: ("Timing advance", "deg",  lambda b: b[0] / 2 - 64),
    0x0F: ("Intake air",     "C",    lambda b: b[0] - 40),
    0x10: ("MAF",            "g/s",  lambda b: (256 * b[0] + b[1]) / 100),
    0x11: ("Throttle",       "%",    lambda b: b[0] * 100 / 255),
    0x1F: ("Run time",       "s",    lambda b: 256 * b[0] + b[1]),
    0x21: ("Dist w/ MIL",    "km",   lambda b: 256 * b[0] + b[1]),
    0x2F: ("Fuel level",     "%",    lambda b: b[0] * 100 / 255),
    0x31: ("Dist cleared",   "km",   lambda b: 256 * b[0] + b[1]),
    0x33: ("Barometric",     "kPa",  lambda b: b[0]),
    0x42: ("Module voltage", "V",    lambda b: (256 * b[0] + b[1]) / 1000),
    0x45: ("Rel throttle",   "%",    lambda b: b[0] * 100 / 255),
    0x46: ("Ambient temp",   "C",    lambda b: b[0] - 40),
    0x5C: ("Oil temp",       "C",    lambda b: b[0] - 40),
    0xA6: ("Odometer",       "km",
           lambda b: ((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]) / 10),
}

SPARK = "▁▂▃▄▅▆▇█"
FRAME_RE = re.compile(r"^([TR])\s+\(([\d.]+)\)\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)$")

HISTORY = 240          # samples kept per PID, for sparklines and deltas
DELTA_WINDOW_S = 1.0   # rate of change is measured over this window


class Decoder:
    """Parses firmware lines into decoded PID values and metadata."""

    def __init__(self):
        self.values = {}        # pid -> (value, unit, name, monotonic_time)
        self.history = {}       # pid -> deque[(t, value)]
        self.raw_pids = {}      # pid -> raw bytes, for PIDs we cannot scale
        self.meta = {}          # MODE / LINK / VIN / ...
        self.rx = self.tx = self.parse_errors = 0
        self._rx_times = deque(maxlen=200)
        self._asm = {}          # id -> partial multi-frame payload

    # ---- ISO-TP reassembly -------------------------------------------
    def _payload(self, can_id, data):
        """Returns a complete ISO-TP payload, or None if still assembling."""
        if not data:
            return None
        pci = data[0] >> 4
        if pci == 0x0:                       # single frame
            return data[1:1 + (data[0] & 0x0F)]
        if pci == 0x1:                       # first frame
            total = ((data[0] & 0x0F) << 8) | data[1]
            self._asm[can_id] = (total, bytearray(data[2:]))
            return None
        if pci == 0x2:                       # consecutive frame
            entry = self._asm.get(can_id)
            if entry is None:
                return None
            total, buf = entry
            buf.extend(data[1:])
            if len(buf) >= total:
                del self._asm[can_id]
                return bytes(buf[:total])
        return None

    def feed(self, line):
        line = line.strip()
        if not line:
            return
        if line.startswith("#"):
            parts = line[1:].split(None, 1)
            if parts:
                self.meta[parts[0]] = parts[1] if len(parts) > 1 else ""
            return

        m = FRAME_RE.match(line)
        if not m:
            # Bootloader chatter and anything else is not an error - only a line
            # that claims to be a frame but will not parse counts as one.
            if line[:2] in ("T ", "R ", "T(", "R("):
                self.parse_errors += 1
            return
        direction, _ts, can_id_s, data_s = m.groups()
        if direction == "T":
            self.tx += 1
            return
        self.rx += 1
        self._rx_times.append(time.monotonic())

        try:
            data = bytes.fromhex(data_s)
        except ValueError:
            self.parse_errors += 1
            return

        payload = self._payload(int(can_id_s, 16), data)
        if not payload or len(payload) < 2:
            return

        if payload[0] == 0x41:               # positive response to Service 01
            pid, body = payload[1], payload[2:]
            self._record(pid, body)
        elif payload[0] == 0x49 and len(payload) > 3:   # Service 09
            text = "".join(chr(c) for c in payload[3:] if 32 <= c < 127)
            self.meta.setdefault({0x02: "VIN", 0x04: "CAL", 0x0A: "ECU"}
                                 .get(payload[1], "S09"), text)

    def _record(self, pid, body):
        now = time.monotonic()
        entry = PIDS.get(pid)
        if entry is None or not body:
            self.raw_pids[pid] = body
            return
        name, unit, fn = entry
        try:
            value = float(fn(body + b"\x00\x00\x00\x00"))
        except (IndexError, ZeroDivisionError):
            self.parse_errors += 1
            return
        self.values[pid] = (value, unit, name, now)
        self.history.setdefault(pid, deque(maxlen=HISTORY)).append((now, value))

    # ---- derived ------------------------------------------------------
    def delta(self, pid):
        h = self.history.get(pid)
        if not h or len(h) < 2:
            return None
        now = h[-1][0]
        for t, v in reversed(h):
            if now - t >= DELTA_WINDOW_S:
                return h[-1][1] - v
        return h[-1][1] - h[0][1]

    def spark(self, pid, width):
        h = self.history.get(pid)
        if not h or len(h) < 2 or width < 2:
            return ""
        vals = [v for _, v in list(h)[-width:]]
        lo, hi = min(vals), max(vals)
        if hi - lo < 1e-9:
            return SPARK[0] * len(vals)
        return "".join(SPARK[min(len(SPARK) - 1,
                                 int((v - lo) / (hi - lo) * (len(SPARK) - 1)))]
                       for v in vals)

    def rx_rate(self):
        now = time.monotonic()
        recent = [t for t in self._rx_times if now - t <= 2.0]
        return len(recent) / 2.0

    def reset_history(self):
        self.history.clear()
        self._rx_times.clear()


def reader_thread(port, baud, decoder, raw, stop, logfile):
    """Owns the serial port; never touches curses."""
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except serial.SerialException as e:
        raw.append(f"!! cannot open {port}: {e}")
        stop.set()
        return
    buf = b""
    while not stop.is_set():
        try:
            chunk = ser.read(4096)
        except serial.SerialException as e:
            raw.append(f"!! serial lost: {e}")
            stop.set()
            break
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("utf-8", "replace").rstrip("\r")
            decoder.feed(text)
            raw.append(text)
            if logfile:
                logfile.write(text + "\n")
                logfile.flush()
    ser.close()


def draw(stdscr, dec, raw, port, paused):
    stdscr.erase()
    h, w = stdscr.getmaxyx()
    if h < 10 or w < 60:
        stdscr.addnstr(0, 0, "terminal too small (need 60x10)", w - 1)
        return

    mode = dec.meta.get("MODE", "")
    ok = mode.startswith("activa-obd")

    title = f" Activa ECU  {port} "
    stdscr.addnstr(0, 0, title.ljust(w), w - 1, curses.A_REVERSE | curses.A_BOLD)

    if not ok:
        msg = ("WAITING for '# MODE activa-obd' - flash env:esp32s3_activa_obd"
               if not mode else f"WRONG MODE: {mode}")
        stdscr.addnstr(1, 0, msg.ljust(w - 1), w - 1, curses.A_BOLD)
    else:
        link = dec.meta.get("LINK", "?")
        vin = dec.meta.get("VIN", "?")
        ecu = dec.meta.get("ECU", "?").strip()
        stdscr.addnstr(1, 0, f" link {link}   VIN {vin}   ECU {ecu}"[:w - 1], w - 1)

    dtc = dec.meta.get("DTC", "")
    stdscr.addnstr(2, 0, f" DTC {dtc}   ring {dec.meta.get('RING','')}"[:w - 1], w - 1)

    hdr = f"{'PID':>4} {'parameter':<16}{'value':>11} {'unit':<5}{'Δ/s':>9}  history"
    stdscr.addnstr(3, 0, hdr.ljust(w - 1), w - 1, curses.A_UNDERLINE)

    rows = sorted(dec.values.items(), key=lambda kv: kv[0])
    table_h = max(3, (h - 6) // 2)
    spark_w = max(8, w - 50)
    y = 4
    now = time.monotonic()
    for pid, (value, unit, name, ts) in rows[:table_h]:
        d = dec.delta(pid)
        dtxt = "  --" if d is None else f"{d:+9.1f}"
        stale = curses.A_DIM if now - ts > 3.0 else curses.A_NORMAL
        line = (f"{pid:>4X} {name:<16}{value:>11.1f} {unit:<5}{dtxt}  "
                f"{dec.spark(pid, spark_w)}")
        stdscr.addnstr(y, 0, line[:w - 1], w - 1, stale)
        y += 1

    if dec.raw_pids:
        undec = " ".join(f"{p:02X}" for p in sorted(dec.raw_pids))
        stdscr.addnstr(y, 0, f"  no scaling for: {undec}"[:w - 1], w - 1, curses.A_DIM)
        y += 1

    sep = f"── raw {'(PAUSED)' if paused else ''} "
    stdscr.addnstr(y, 0, sep.ljust(w - 1, "─"), w - 1, curses.A_DIM)
    y += 1
    tail = list(raw)[-(h - y - 1):]
    for line in tail:
        if y >= h - 1:
            break
        stdscr.addnstr(y, 0, line[:w - 1], w - 1)
        y += 1

    status = (f" rx {dec.rx}  tx {dec.tx}  {dec.rx_rate():.1f} frame/s  "
              f"parse-err {dec.parse_errors}   q quit  r reset  p pause ")
    stdscr.addnstr(h - 1, 0, status.ljust(w - 1), w - 1, curses.A_REVERSE)
    stdscr.refresh()


def main():
    ap = argparse.ArgumentParser(description="Live Activa ECU dashboard")
    ap.add_argument("port", help="serial port, e.g. /dev/ttyACM1 or a "
                                 "/dev/serial/by-id/... path")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-l", "--log", help="also append the raw stream to this file")
    args = ap.parse_args()

    dec = Decoder()
    raw = deque(maxlen=500)
    stop = threading.Event()
    logfile = open(args.log, "a") if args.log else None

    t = threading.Thread(target=reader_thread,
                         args=(args.port, args.baud, dec, raw, stop, logfile),
                         daemon=True)
    t.start()

    def ui(stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        paused = False
        frozen = None
        while not stop.is_set():
            k = stdscr.getch()
            if k in (ord("q"), 27):
                break
            if k == ord("r"):
                dec.reset_history()
            if k == ord("p"):
                paused = not paused
                frozen = list(raw) if paused else None
            draw(stdscr, dec, frozen if paused else raw, args.port, paused)
            time.sleep(0.1)

    try:
        curses.wrapper(ui)
    finally:
        stop.set()
        t.join(timeout=1.0)
        if logfile:
            logfile.close()
        # The reader reports open/lost errors through the raw pane, which is
        # gone once curses exits - so repeat them on the way out.
        for line in list(raw)[-5:]:
            if line.startswith("!!"):
                print(line, file=sys.stderr)


if __name__ == "__main__":
    main()
