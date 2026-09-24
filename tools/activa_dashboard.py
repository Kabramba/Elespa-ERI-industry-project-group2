#!/usr/bin/env python3
"""Web dashboard for the Activa ECU.

    ./dash esp

The firmware emits frames; this decodes them. It reads the 'T'/'R' lines the
esp32s3_activa_obd application prints, reassembles ISO-TP, decodes OBD-II
Service 01 PIDs, and serves a live page over a WebSocket.

Only the activa-obd application is supported - the page says so plainly rather
than silently misreading some other firmware's output.
"""
import argparse
import asyncio
import json
import re
import sys
import threading
import time
import webbrowser
from collections import deque
from pathlib import Path

try:
    import serial
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse
except ImportError as e:
    sys.exit(f"missing dependency ({e.name}).  "
             f".venv/bin/pip install -r tools/requirements.txt")

# --------------------------------------------------------------------------
# OBD-II Service 01 decode. Formulas from docs/activa-can.md s6.6.
# A, B, C, D are the bytes after the PID. A PID absent here is still surfaced
# raw - guessing a scaling is worse than admitting we do not have one.
# --------------------------------------------------------------------------
PIDS = {
    0x04: ("Engine load",     "%",    lambda b: b[0] * 100 / 255),
    0x05: ("Engine temp",     "°C",   lambda b: b[0] - 40),
    0x06: ("Short fuel trim", "%",    lambda b: b[0] * 100 / 128 - 100),
    0x07: ("Long fuel trim",  "%",    lambda b: b[0] * 100 / 128 - 100),
    0x0B: ("MAP",             "kPa",  lambda b: b[0]),
    0x0C: ("RPM",             "rpm",  lambda b: (256 * b[0] + b[1]) / 4),
    0x0D: ("Speed",           "km/h", lambda b: b[0]),
    0x0E: ("Timing advance",  "°",    lambda b: b[0] / 2 - 64),
    0x0F: ("Intake air",      "°C",   lambda b: b[0] - 40),
    0x10: ("MAF",             "g/s",  lambda b: (256 * b[0] + b[1]) / 100),
    0x11: ("Throttle",        "%",    lambda b: b[0] * 100 / 255),
    0x1F: ("Run time",        "s",    lambda b: 256 * b[0] + b[1]),
    0x21: ("Dist w/ MIL",     "km",   lambda b: 256 * b[0] + b[1]),
    0x2F: ("Fuel level",      "%",    lambda b: b[0] * 100 / 255),
    0x31: ("Dist cleared",    "km",   lambda b: 256 * b[0] + b[1]),
    0x33: ("Barometric",      "kPa",  lambda b: b[0]),
    0x42: ("Module voltage",  "V",    lambda b: (256 * b[0] + b[1]) / 1000),
    0x45: ("Rel throttle",    "%",    lambda b: b[0] * 100 / 255),
    0x46: ("Ambient temp",    "°C",   lambda b: b[0] - 40),
    0x5C: ("Oil temp",        "°C",   lambda b: b[0] - 40),
    0xA6: ("Odometer",        "km",
           lambda b: ((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]) / 10),
}

FRAME_RE = re.compile(r"^([TR])\s+\(([\d.]+)\)\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)$")
HISTORY = 180          # samples kept per PID for the charts
DELTA_WINDOW_S = 1.0   # rate of change measured over this window


class RawLog:
    """Raw serial lines with a running total, so each client gets only what it
    has not already seen rather than the whole backlog every tick."""

    def __init__(self, maxlen=4000):
        self._lines = deque(maxlen=maxlen)
        self.total = 0
        self._lock = threading.Lock()

    def append(self, line):
        with self._lock:
            self._lines.append(line)
            self.total += 1

    def since(self, seq):
        with self._lock:
            if seq >= self.total:
                return self.total, []
            behind = min(self.total - seq, len(self._lines))
            return self.total, list(self._lines)[-behind:]


class Decoder:
    def __init__(self):
        self.values = {}      # pid -> (value, unit, name, monotonic time)
        self.history = {}     # pid -> deque[(t, value)]
        self.raw_pids = {}    # pid -> raw bytes, for PIDs with no known scaling
        self.meta = {}
        self.rx = self.tx = self.parse_errors = 0
        self.connected = False
        self.error = None
        self._rx_times = deque(maxlen=400)
        self._asm = {}
        self._lock = threading.Lock()

    # ---- ISO-TP reassembly -------------------------------------------
    def _payload(self, can_id, data):
        if not data:
            return None
        pci = data[0] >> 4
        if pci == 0x0:
            return data[1:1 + (data[0] & 0x0F)]
        if pci == 0x1:
            total = ((data[0] & 0x0F) << 8) | data[1]
            self._asm[can_id] = (total, bytearray(data[2:]))
            return None
        if pci == 0x2:
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
                with self._lock:
                    self.meta[parts[0]] = parts[1] if len(parts) > 1 else ""
            return

        m = FRAME_RE.match(line)
        if not m:
            # Bootloader chatter is not an error; only a line claiming to be a
            # frame that will not parse counts.
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
        if payload[0] == 0x41:
            self._record(payload[1], payload[2:])
        elif payload[0] == 0x49 and len(payload) > 3:
            text = "".join(chr(c) for c in payload[3:] if 32 <= c < 127)
            key = {0x02: "VIN", 0x04: "CAL", 0x0A: "ECU"}.get(payload[1], "S09")
            with self._lock:
                self.meta.setdefault(key, text)

    def _record(self, pid, body):
        now = time.monotonic()
        entry = PIDS.get(pid)
        if entry is None or not body:
            with self._lock:
                self.raw_pids[pid] = body.hex().upper()
            return
        name, unit, fn = entry
        try:
            value = float(fn(body + b"\x00\x00\x00\x00"))
        except (IndexError, ZeroDivisionError):
            self.parse_errors += 1
            return
        with self._lock:
            self.values[pid] = (value, unit, name, now)
            self.history.setdefault(pid, deque(maxlen=HISTORY)).append((now, value))

    # ---- derived ------------------------------------------------------
    def _delta(self, pid):
        h = self.history.get(pid)
        if not h or len(h) < 2:
            return None
        now = h[-1][0]
        for t, v in reversed(h):
            if now - t >= DELTA_WINDOW_S:
                return h[-1][1] - v
        return h[-1][1] - h[0][1]

    def rate(self):
        now = time.monotonic()
        return len([t for t in self._rx_times if now - t <= 2.0]) / 2.0

    def reset_history(self):
        with self._lock:
            self.history.clear()
            self._rx_times.clear()

    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            mode = self.meta.get("MODE", "")
            values = []
            for pid, (value, unit, name, ts) in sorted(self.values.items()):
                h = self.history.get(pid)
                values.append({
                    "pid": f"{pid:02X}",
                    "name": name,
                    "unit": unit,
                    "value": round(value, 2),
                    "delta": (lambda d: None if d is None else round(d, 2))(self._delta(pid)),
                    "age": round(now - ts, 2),
                    "history": [round(v, 2) for _, v in h] if h else [],
                })
            return {
                "ok": mode.startswith("activa-obd"),
                "mode": mode,
                "connected": self.connected,
                "error": self.error,
                "meta": dict(self.meta),
                "values": values,
                "undecoded": [f"{p:02X}" for p in sorted(self.raw_pids)],
                "stats": {
                    "rx": self.rx, "tx": self.tx,
                    "rate": round(self.rate(), 1),
                    "errors": self.parse_errors,
                },
            }


def reader_thread(port, baud, dec, raw, stop, logfile):
    """Owns the serial port. Reconnects on its own if the board is replugged."""
    while not stop.is_set():
        try:
            ser = serial.Serial(port, baud, timeout=0.2)
        except serial.SerialException as e:
            dec.connected = False
            dec.error = f"cannot open {port}: {e}"
            raw.append(f"!! {dec.error}")
            time.sleep(2.0)
            continue

        dec.connected = True
        dec.error = None
        raw.append(f"## connected to {port} @ {baud}")
        buf = b""
        while not stop.is_set():
            try:
                chunk = ser.read(4096)
            except serial.SerialException as e:
                dec.error = f"serial lost: {e}"
                raw.append(f"!! {dec.error}")
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", "replace").rstrip("\r")
                dec.feed(text)
                raw.append(text)
                if logfile:
                    logfile.write(text + "\n")
                    logfile.flush()
        dec.connected = False
        try:
            ser.close()
        except Exception:
            pass


def build_app(dec, raw, port_name):
    app = FastAPI(title="Activa ECU dashboard")
    html_path = Path(__file__).with_name("dashboard.html")

    @app.get("/", response_class=HTMLResponse)
    async def index():
        return html_path.read_text(encoding="utf-8")

    @app.post("/api/reset")
    async def reset():
        dec.reset_history()
        return {"ok": True}

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        seq = max(0, raw.total - 200)   # start with a little context
        try:
            while True:
                seq, lines = raw.since(seq)
                payload = dec.snapshot()
                payload["raw"] = lines
                payload["port"] = port_name
                await sock.send_text(json.dumps(payload))
                await asyncio.sleep(0.25)
        except (WebSocketDisconnect, RuntimeError, ConnectionError):
            pass

    return app


def main():
    ap = argparse.ArgumentParser(description="Live Activa ECU web dashboard")
    ap.add_argument("port", help="serial port, e.g. /dev/ttyACM0 or a "
                                 "/dev/serial/by-id/... path")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-l", "--log", help="also append the raw stream to this file")
    ap.add_argument("--http-port", type=int, default=8477)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    dec = Decoder()
    raw = RawLog()
    stop = threading.Event()
    logfile = open(args.log, "a") if args.log else None

    t = threading.Thread(target=reader_thread,
                         args=(args.port, args.baud, dec, raw, stop, logfile),
                         daemon=True)
    t.start()

    url = f"http://127.0.0.1:{args.http_port}/"
    print(f"\n  Activa ECU dashboard")
    print(f"    serial : {args.port} @ {args.baud}")
    print(f"    open   : {url}")
    print(f"    stop   : Ctrl-C\n")
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

    app = build_app(dec, raw, args.port)
    try:
        uvicorn.run(app, host="127.0.0.1", port=args.http_port, log_level="warning")
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        t.join(timeout=1.0)
        if logfile:
            logfile.close()


if __name__ == "__main__":
    main()
