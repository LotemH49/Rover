"""Visual, browser-based simulator for the rover.

Runs the *real* ``rover.py`` against faked Raspberry Pi hardware (see
``sim_hardware.py``), simulates differential-drive physics + quadrature
encoders, and serves a live HTML5 canvas you can watch and control.

Usage:
    python3 simulator.py

Then open http://localhost:8000 (it also tries to open automatically).
Use the on-screen buttons to drive/turn, or run the scripted demo. Everything
goes through rover.drive() / rover.turn(), so you are testing the actual code.

Pure standard library -- no pip installs required.
"""

import json
import os
import queue
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from socketserver import TCPServer
from urllib.parse import urlparse, parse_qs

import sim_hardware

# Install fakes BEFORE importing rover so its hardware imports resolve.
SIM = sim_hardware.install()
import rover as rover_mod  # noqa: E402

# Bind encoder pins (defined in rover.py) to the physics engine.
sim_hardware.bind_encoders_from(rover_mod.ENC_PINS)

HOST = "127.0.0.1"
PORT = 8000


class Server(ThreadingHTTPServer):
    """ThreadingHTTPServer that skips the slow reverse-DNS getfqdn() call in
    HTTPServer.server_bind (which can hang for many seconds on some networks)."""

    daemon_threads = True
    allow_reuse_address = True

    def server_bind(self):
        TCPServer.server_bind(self)
        host, port = self.server_address[:2]
        self.server_name = host
        self.server_port = port

# A radiation source for future search-algorithm testing (mm).
SOURCE = {"x": 700.0, "y": 500.0}


# =========================================================================
# Command worker -- executes rover commands one at a time off a queue
# =========================================================================
class RobotController:
    def __init__(self):
        self.rover = rover_mod.Rover()
        self.q = queue.Queue()
        self.busy = False
        self.current = "idle"
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def enqueue(self, cmd_type, value):
        self.q.put((cmd_type, value))

    def clear_queue(self):
        try:
            while True:
                self.q.get_nowait()
        except queue.Empty:
            pass

    def reset(self):
        # Only safe to re-zero pose when nothing is moving.
        if not self.busy and self.q.empty():
            SIM.reset()
            for k in self.rover.counts:
                self.rover.counts[k] = 0
            return True
        return False

    def enqueue_demo(self):
        for cmd in (("drive", 500), ("turn", 90), ("drive", 300),
                    ("turn", -90), ("drive", 200)):
            self.q.put(cmd)

    def enqueue_basic_search(self, x_max=500, y_max=500, angle_initial=0):
        self.q.put(("basic_search", (x_max, y_max, angle_initial)))

    def _run(self):
        while True:
            cmd_type, value = self.q.get()
            self.busy = True
            self.current = (
                f"basic_search{value}" if cmd_type == "basic_search"
                else f"{cmd_type}({value})"
            )
            try:
                if cmd_type == "drive":
                    self.rover.drive(value)
                elif cmd_type == "turn":
                    self.rover.turn(value)
                elif cmd_type == "basic_search":
                    x_max, y_max, angle_initial = value
                    rover_mod.basic_search(
                        x_max, y_max, angle_initial, rover=self.rover
                    )
            except Exception as exc:  # keep the worker alive
                print("command error:", exc)
            finally:
                self.rover.stop()
                self.busy = False
                self.current = "idle"


CONTROLLER = None  # set in main()


# =========================================================================
# State for the UI
# =========================================================================
def build_state():
    snap = SIM.snapshot()
    import math
    dx = SOURCE["x"] - snap["x"]
    dy = SOURCE["y"] - snap["y"]
    dist = math.hypot(dx, dy)
    # Simple inverse-square-ish sensor model (placeholder for real search).
    sensor = 1_000_000.0 / (dist * dist + 1.0)
    return {
        "x": snap["x"],
        "y": snap["y"],
        "theta_deg": math.degrees(snap["theta"]),
        "throttles": snap["throttles"],
        "counts": dict(CONTROLLER.rover.counts),
        "trail": snap["trail"],
        "busy": CONTROLLER.busy,
        "current": CONTROLLER.current,
        "queued": CONTROLLER.q.qsize(),
        "source": SOURCE,
        "sensor": sensor,
        "distance": dist,
    }


# =========================================================================
# HTTP handler
# =========================================================================
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # quiet

    def _send(self, code, content_type, body):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._send(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
        elif parsed.path == "/state":
            body = json.dumps(build_state()).encode("utf-8")
            self._send(200, "application/json", body)
        elif parsed.path == "/cmd":
            params = parse_qs(parsed.query)
            self._handle_cmd(params)
            self._send(200, "application/json", b'{"ok":true}')
        else:
            self._send(404, "text/plain", b"not found")

    def _handle_cmd(self, params):
        action = params.get("action", [""])[0]
        if action == "drive":
            CONTROLLER.enqueue("drive", float(params.get("value", ["0"])[0]))
        elif action == "turn":
            CONTROLLER.enqueue("turn", float(params.get("value", ["0"])[0]))
        elif action == "demo":
            CONTROLLER.enqueue_demo()
        elif action == "search":
            x_max = float(params.get("x_max", ["500"])[0])
            y_max = float(params.get("y_max", ["500"])[0])
            angle = float(params.get("angle", ["0"])[0])
            CONTROLLER.enqueue_basic_search(x_max, y_max, angle)
        elif action == "clear":
            CONTROLLER.clear_queue()
        elif action == "reset":
            CONTROLLER.reset()


# =========================================================================
# Front-end (single page, canvas + controls)
# =========================================================================
PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>Rover Simulator</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin: 0; font-family: ui-monospace, Menlo, Consolas, monospace;
         background: #0d1117; color: #c9d1d9; display: flex; height: 100vh; }
  #stage { flex: 1; display: flex; align-items: center; justify-content: center;
           background: radial-gradient(circle at 50% 40%, #11161d, #0d1117); }
  canvas { background: #0a0e13; border: 1px solid #21262d; border-radius: 8px; }
  #panel { width: 290px; padding: 18px; border-left: 1px solid #21262d;
           overflow-y: auto; }
  h1 { font-size: 15px; margin: 0 0 14px; color: #58a6ff; letter-spacing: .5px; }
  h2 { font-size: 11px; text-transform: uppercase; color: #8b949e;
       margin: 18px 0 8px; letter-spacing: 1px; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  button { font-family: inherit; font-size: 13px; padding: 9px 6px; cursor: pointer;
           background: #21262d; color: #c9d1d9; border: 1px solid #30363d;
           border-radius: 6px; transition: background .12s; }
  button:hover { background: #30363d; }
  button.primary { background: #1f6feb; border-color: #1f6feb; color: #fff; }
  button.primary:hover { background: #388bfd; }
  button.danger { background: #2d1416; border-color: #6e2c30; color: #ff9b9b; }
  .full { grid-column: 1 / -1; }
  .stat { display: flex; justify-content: space-between; font-size: 12px;
          padding: 3px 0; border-bottom: 1px solid #161b22; }
  .stat span:last-child { color: #e6edf3; }
  .pill { display:inline-block; padding:2px 8px; border-radius:10px; font-size:11px; }
  .busy { background:#3b2f00; color:#ffd866; }
  .idle { background:#0f2e1a; color:#7ee787; }
</style>
</head>
<body>
  <div id="stage"><canvas id="cv" width="900" height="640"></canvas></div>
  <div id="panel">
    <h1>ROVER SIMULATOR</h1>

    <h2>Drive</h2>
    <div class="grid">
      <button onclick="cmd('drive',100)">Fwd 100mm</button>
      <button onclick="cmd('drive',-100)">Back 100mm</button>
      <button onclick="cmd('drive',500)">Fwd 500mm</button>
      <button onclick="cmd('drive',-500)">Back 500mm</button>
    </div>

    <h2>Turn (in place)</h2>
    <div class="grid">
      <button onclick="cmd('turn',15)">Left 15&deg;</button>
      <button onclick="cmd('turn',-15)">Right 15&deg;</button>
      <button onclick="cmd('turn',90)">Left 90&deg;</button>
      <button onclick="cmd('turn',-90)">Right 90&deg;</button>
    </div>

    <h2>Session</h2>
    <div class="grid">
      <button class="primary full" onclick="cmd('search')">Run basic search</button>
      <button onclick="cmd('demo')">Run demo sequence</button>
      <button onclick="cmd('clear')">Clear queue</button>
      <button class="danger" onclick="cmd('reset')">Reset pose</button>
    </div>

    <h2>Telemetry</h2>
    <div class="stat"><span>status</span><span id="status"></span></div>
    <div class="stat"><span>command</span><span id="cur">idle</span></div>
    <div class="stat"><span>queued</span><span id="queued">0</span></div>
    <div class="stat"><span>x (mm)</span><span id="x">0</span></div>
    <div class="stat"><span>y (mm)</span><span id="y">0</span></div>
    <div class="stat"><span>heading</span><span id="th">0&deg;</span></div>
    <div class="stat"><span>dist to source</span><span id="dist">0</span></div>
    <div class="stat"><span>sensor</span><span id="sensor">0</span></div>

    <h2>Encoder counts</h2>
    <div class="stat"><span>FL / FR</span><span id="c13">0 / 0</span></div>
    <div class="stat"><span>RL / RR</span><span id="c24">0 / 0</span></div>

    <h2>Throttles</h2>
    <div class="stat"><span>FL / FR</span><span id="t12">0 / 0</span></div>
    <div class="stat"><span>RL / RR</span><span id="t34">0 / 0</span></div>
  </div>

<script>
const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');
const SCALE = 0.42;            // px per mm
let state = null;

function cmd(action, value) {
  let url = '/cmd?action=' + action;
  if (value !== undefined) url += '&value=' + value;
  fetch(url);
}

async function poll() {
  try {
    const r = await fetch('/state');
    state = await r.json();
    updatePanel();
  } catch (e) {}
}

function fmt(n, d=1){ return Number(n).toFixed(d); }

function updatePanel() {
  if (!state) return;
  const st = document.getElementById('status');
  st.textContent = state.busy ? 'MOVING' : 'idle';
  st.className = 'pill ' + (state.busy ? 'busy' : 'idle');
  document.getElementById('cur').textContent = state.current;
  document.getElementById('queued').textContent = state.queued;
  document.getElementById('x').textContent = fmt(state.x);
  document.getElementById('y').textContent = fmt(state.y);
  document.getElementById('th').textContent = fmt(state.theta_deg) + '\u00b0';
  document.getElementById('dist').textContent = fmt(state.distance) + ' mm';
  document.getElementById('sensor').textContent = fmt(state.sensor, 2);
  const c = state.counts, t = state.throttles;
  document.getElementById('c13').textContent = c['1'] + ' / ' + c['2'];
  document.getElementById('c24').textContent = c['3'] + ' / ' + c['4'];
  const tf = v => (v===null? 'coast' : Number(v).toFixed(2));
  document.getElementById('t12').textContent = tf(t['1']) + ' / ' + tf(t['2']);
  document.getElementById('t34').textContent = tf(t['3']) + ' / ' + tf(t['4']);
}

function draw() {
  requestAnimationFrame(draw);
  ctx.clearRect(0,0,cv.width,cv.height);
  if (!state) return;

  const camx = state.x, camy = state.y;
  const cx = cv.width/2, cy = cv.height/2;
  const w2s = (wx, wy) => [cx + (wx-camx)*SCALE, cy - (wy-camy)*SCALE];

  // grid (every 100mm)
  ctx.strokeStyle = '#161b22'; ctx.lineWidth = 1;
  const step = 100;
  const halfW = cv.width/2/SCALE, halfH = cv.height/2/SCALE;
  const startX = Math.floor((camx-halfW)/step)*step;
  const endX = camx+halfW;
  for (let gx=startX; gx<=endX; gx+=step){
    const [sx] = w2s(gx, 0);
    ctx.beginPath(); ctx.moveTo(sx,0); ctx.lineTo(sx,cv.height); ctx.stroke();
  }
  const startY = Math.floor((camy-halfH)/step)*step;
  const endY = camy+halfH;
  for (let gy=startY; gy<=endY; gy+=step){
    const [,sy] = w2s(0, gy);
    ctx.beginPath(); ctx.moveTo(0,sy); ctx.lineTo(cv.width,sy); ctx.stroke();
  }

  // origin marker
  const [ox, oy] = w2s(0,0);
  ctx.strokeStyle = '#2d333b';
  ctx.beginPath(); ctx.moveTo(ox-8,oy); ctx.lineTo(ox+8,oy);
  ctx.moveTo(ox,oy-8); ctx.lineTo(ox,oy+8); ctx.stroke();

  // radiation source
  const [srx, sry] = w2s(state.source.x, state.source.y);
  const g = ctx.createRadialGradient(srx,sry,2, srx,sry,40);
  g.addColorStop(0, 'rgba(255,210,80,0.9)');
  g.addColorStop(1, 'rgba(255,210,80,0)');
  ctx.fillStyle = g;
  ctx.beginPath(); ctx.arc(srx,sry,40,0,7); ctx.fill();
  ctx.fillStyle = '#ffd866';
  ctx.beginPath(); ctx.arc(srx,sry,5,0,7); ctx.fill();

  // trail
  if (state.trail.length > 1){
    ctx.strokeStyle = '#1f6feb'; ctx.lineWidth = 2;
    ctx.beginPath();
    state.trail.forEach((p,i)=>{
      const [sx,sy] = w2s(p[0],p[1]);
      if (i===0) ctx.moveTo(sx,sy); else ctx.lineTo(sx,sy);
    });
    ctx.stroke();
  }

  // rover body
  const [rx, ry] = w2s(state.x, state.y);
  const th = state.theta_deg * Math.PI/180;
  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(-th);
  const L = 220*SCALE, W = 300*SCALE;       // body length (x), width (y)
  // wheels
  ctx.fillStyle = '#30363d';
  const wl = 70*SCALE, ww = 26*SCALE;
  for (const sx of [-L/2*0.6, L/2*0.6]){
    for (const sy of [-W/2, W/2-ww]){
      ctx.fillRect(sx-wl/2, sy, wl, ww);
    }
  }
  // chassis
  ctx.fillStyle = '#238636';
  ctx.strokeStyle = '#7ee787'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.rect(-L/2, -W/2+ww*0.2, L, W-ww*0.4); ctx.fill(); ctx.stroke();
  // heading arrow (points +x = forward)
  ctx.fillStyle = '#7ee787';
  ctx.beginPath();
  ctx.moveTo(L/2, 0); ctx.lineTo(L/2-22, -12); ctx.lineTo(L/2-22, 12);
  ctx.closePath(); ctx.fill();
  ctx.restore();
}

setInterval(poll, 60);
draw();
</script>
</body>
</html>
"""


def main():
    global CONTROLLER
    CONTROLLER = RobotController()
    SIM.start()
    CONTROLLER.enqueue_basic_search(500, 500, 0)

    server = Server((HOST, PORT), Handler)
    url = f"http://{HOST}:{PORT}"
    print(f"Rover simulator running at {url}", flush=True)
    print("Press Ctrl-C to stop.", flush=True)

    # Open the browser without blocking serve_forever (set ROVER_SIM_NO_OPEN=1
    # to skip, e.g. for headless runs).
    if not os.environ.get("ROVER_SIM_NO_OPEN"):
        threading.Thread(
            target=lambda: webbrowser.open(url), daemon=True
        ).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        SIM.stop()
        server.server_close()


if __name__ == "__main__":
    main()
