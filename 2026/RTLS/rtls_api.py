import argparse
import json
import math
import re
import threading
import time
from datetime import datetime, timezone
from typing import Any, Dict, Optional

import serial
from fastapi import FastAPI
from fastapi.responses import HTMLResponse

try:
    import serial.tools.list_ports as list_ports
except Exception:
    list_ports = None

TAG_POS_RE = re.compile(r"TAG POS m:\\s*X=([-+]?\\d*\\.?\\d+)\\s*Y=([-+]?\\d*\\.?\\d+)")
RANGES_RE = re.compile(r"RANGES m:\\s*A1=([^\\s]+)\\s*A2=([^\\s]+)\\s*A3=([^\\s]+)")


def _to_float(value: str) -> Optional[float]:
    try:
        v = float(value)
        if math.isnan(v):
            return None
        return v
    except Exception:
        return None


class RtlsState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.state: Dict[str, Any] = {
            "x": None,
            "y": None,
            "ranges": {"a1": None, "a2": None, "a3": None},
            "raw": "",
            "updated_at": None,
            "last_line": "",
            "source": "none",
        }

    def update(self, patch: Dict[str, Any], source: str, line: str) -> None:
        with self.lock:
            if "ranges" in patch and isinstance(patch["ranges"], dict):
                self.state["ranges"].update(patch["ranges"])
                patch = {k: v for k, v in patch.items() if k != "ranges"}
            self.state.update(patch)
            self.state["source"] = source
            self.state["last_line"] = line.strip()
            self.state["updated_at"] = datetime.now(timezone.utc).isoformat()

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "x": self.state.get("x"),
                "y": self.state.get("y"),
                "ranges": dict(self.state.get("ranges", {})),
                "raw": self.state.get("raw", ""),
                "updated_at": self.state.get("updated_at"),
                "last_line": self.state.get("last_line", ""),
                "source": self.state.get("source", "none"),
            }


class SerialReader(threading.Thread):
    def __init__(self, port: str, baudrate: int, state: RtlsState) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.state = state
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                with serial.Serial(self.port, self.baudrate, timeout=1) as ser:
                    self.state.update({"raw": f"connected:{self.port}"}, "system", "")
                    while not self._stop_event.is_set():
                        line = ser.readline().decode("utf-8", errors="replace").strip()
                        if not line:
                            continue
                        self._handle_line(line)
            except Exception as exc:
                self.state.update({"raw": f"serial_error:{exc}"}, "system", "")
                time.sleep(2)

    def _handle_line(self, line: str) -> None:
        if line.startswith("{") and line.endswith("}"):
            try:
                payload = json.loads(line)
                patch: Dict[str, Any] = {}
                if isinstance(payload, dict):
                    pos = payload.get("pos")
                    if isinstance(pos, dict):
                        patch["x"] = pos.get("x")
                        patch["y"] = pos.get("y")
                    ranges = payload.get("ranges")
                    if isinstance(ranges, dict):
                        patch["ranges"] = {
                            "a1": ranges.get("a1"),
                            "a2": ranges.get("a2"),
                            "a3": ranges.get("a3"),
                        }
                    patch["raw"] = line
                    self.state.update(patch, "json", line)
                    return
            except Exception:
                pass

        m = TAG_POS_RE.search(line)
        if m:
            self.state.update(
                {
                    "x": _to_float(m.group(1)),
                    "y": _to_float(m.group(2)),
                    "raw": line,
                },
                "text",
                line,
            )
            return

        r = RANGES_RE.search(line)
        if r:
            self.state.update(
                {
                    "ranges": {
                        "a1": _to_float(r.group(1)),
                        "a2": _to_float(r.group(2)),
                        "a3": _to_float(r.group(3)),
                    },
                    "raw": line,
                },
                "text",
                line,
            )
            return

        self.state.update({"raw": line}, "log", line)


def auto_detect_port() -> Optional[str]:
    if list_ports is None:
        return None
    preferred = ["CP210", "Silicon Labs", "USB Serial", "CH340", "wchusb"]
    ports = list(list_ports.comports())
    for p in ports:
        desc = f"{p.device} {p.description} {p.hwid}".lower()
        if any(k.lower() in desc for k in preferred):
            return p.device
    return ports[0].device if ports else None


HTML_PAGE = """
<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width,initial-scale=1\" />
  <title>RTLS Position Viewer</title>
  <style>
    :root {
      --bg: #f4f1ea;
      --card: #ffffff;
      --ink: #1f2a30;
      --accent: #1b8f7a;
      --muted: #6f7e86;
      --line: #d8dee3;
    }
    body { font-family: \"Space Grotesk\", \"Segoe UI\", sans-serif; background: radial-gradient(circle at 15% 10%, #e7f5ef 0%, var(--bg) 45%), var(--bg); margin: 0; color: var(--ink); }
    .wrap { max-width: 980px; margin: 24px auto; padding: 0 16px; }
    .card { background: var(--card); border: 1px solid var(--line); border-radius: 16px; padding: 16px; box-shadow: 0 8px 24px rgba(0,0,0,0.05); }
    .grid { display: grid; grid-template-columns: 1fr; gap: 16px; }
    @media (min-width: 900px) { .grid { grid-template-columns: 1.2fr .8fr; } }
    h1 { margin: 0 0 10px; font-size: 1.3rem; letter-spacing: .3px; }
    #map { width: 100%; aspect-ratio: 4 / 3; border: 1px solid var(--line); border-radius: 12px; background: #fffdf8; }
    .kv { display: flex; justify-content: space-between; border-bottom: 1px dashed var(--line); padding: 7px 0; }
    .k { color: var(--muted); }
    .v { font-weight: 600; }
    code { font-family: Consolas, monospace; font-size: .85rem; }
  </style>
</head>
<body>
  <div class=\"wrap\">
    <h1>RTLS Tag Position</h1>
    <div class=\"grid\">
      <div class=\"card\">
        <canvas id=\"map\" width=\"640\" height=\"480\"></canvas>
      </div>
      <div class=\"card\">
        <div class=\"kv\"><span class=\"k\">X (m)</span><span class=\"v\" id=\"x\">-</span></div>
        <div class=\"kv\"><span class=\"k\">Y (m)</span><span class=\"v\" id=\"y\">-</span></div>
        <div class=\"kv\"><span class=\"k\">A1 (m)</span><span class=\"v\" id=\"a1\">-</span></div>
        <div class=\"kv\"><span class=\"k\">A2 (m)</span><span class=\"v\" id=\"a2\">-</span></div>
        <div class=\"kv\"><span class=\"k\">A3 (m)</span><span class=\"v\" id=\"a3\">-</span></div>
        <div class=\"kv\"><span class=\"k\">Source</span><span class=\"v\" id=\"source\">-</span></div>
        <div class=\"kv\"><span class=\"k\">Updated</span><span class=\"v\" id=\"updated\">-</span></div>
        <p><strong>Last line</strong></p>
        <code id=\"last\">-</code>
      </div>
    </div>
  </div>
  <script>
    const anchors = [
      {id: 'A1', x: 0, y: 0},
      {id: 'A2', x: 4, y: 0},
      {id: 'A3', x: 0, y: 3},
    ];

    function fmt(v) {
      return (v === null || v === undefined) ? '-' : Number(v).toFixed(2);
    }

    function draw(state) {
      const c = document.getElementById('map');
      const ctx = c.getContext('2d');
      const w = c.width, h = c.height;
      ctx.clearRect(0, 0, w, h);

      const maxX = 4.5, maxY = 3.5;
      const sx = (x) => 30 + (x / maxX) * (w - 60);
      const sy = (y) => h - 30 - (y / maxY) * (h - 60);

      ctx.strokeStyle = '#d8dee3';
      ctx.lineWidth = 1;
      ctx.strokeRect(30, 30, w - 60, h - 60);

      for (const a of anchors) {
        const x = sx(a.x), y = sy(a.y);
        ctx.fillStyle = '#1b8f7a';
        ctx.beginPath(); ctx.arc(x, y, 7, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = '#1f2a30';
        ctx.fillText(a.id + ` (${a.x},${a.y})`, x + 10, y - 10);
      }

      if (typeof state.x === 'number' && typeof state.y === 'number') {
        const x = sx(state.x), y = sy(state.y);
        ctx.fillStyle = '#c2452d';
        ctx.beginPath(); ctx.arc(x, y, 8, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = '#1f2a30';
        ctx.fillText(`TAG (${state.x.toFixed(2)}, ${state.y.toFixed(2)})`, x + 10, y + 18);
      }
    }

    async function tick() {
      try {
        const r = await fetch('/state');
        const s = await r.json();
        document.getElementById('x').textContent = fmt(s.x);
        document.getElementById('y').textContent = fmt(s.y);
        document.getElementById('a1').textContent = fmt(s.ranges?.a1);
        document.getElementById('a2').textContent = fmt(s.ranges?.a2);
        document.getElementById('a3').textContent = fmt(s.ranges?.a3);
        document.getElementById('source').textContent = s.source || '-';
        document.getElementById('updated').textContent = s.updated_at || '-';
        document.getElementById('last').textContent = s.last_line || '-';
        draw(s);
      } catch (e) {
        console.error(e);
      }
    }
    setInterval(tick, 200);
    tick();
  </script>
</body>
</html>
"""


def create_app(state: RtlsState) -> FastAPI:
    app = FastAPI(title="RTLS Position API")

    @app.get("/health")
    def health() -> Dict[str, Any]:
        return {"ok": True}

    @app.get("/state")
    def get_state() -> Dict[str, Any]:
        return state.snapshot()

    @app.get("/", response_class=HTMLResponse)
    def home() -> str:
        return HTML_PAGE

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="RTLS serial to HTTP API")
    parser.add_argument("--port", default="", help="Serial port, e.g. COM6")
    parser.add_argument("--baud", default=115200, type=int, help="Serial baudrate")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP host")
    parser.add_argument("--http-port", default=8000, type=int, help="HTTP port")
    args = parser.parse_args()

    port = args.port or auto_detect_port()
    if not port:
        raise SystemExit("No serial port found. Use --port COMx")

    state = RtlsState()
    reader = SerialReader(port, args.baud, state)
    reader.start()

    import uvicorn

    app = create_app(state)
    print(f"RTLS API reading {port} @ {args.baud}")
    print(f"Open http://{args.host}:{args.http_port}")
    uvicorn.run(app, host=args.host, port=args.http_port)


if __name__ == "__main__":
    main()
