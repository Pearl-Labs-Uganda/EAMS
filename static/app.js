/* RC car client.
   - touch joystick + Gamepad API -> same arcade mixer
   - sends {"type":"cmd",left,right,seq} at 20 Hz, including zeros
   - renders merged 20 Hz telemetry; Jetson ships raw, we scale here
   - staleness gray-out, STALL/DEADMAN banners, WS auto-reconnect w/ backoff
   - control switch: many viewers, ONE driver. Must send {"type":"take"} to
     drive; {"type":"release"} gives it up. The button grays out while
     someone else holds control and ungrays the moment they release or
     disconnect. Enforcement is SERVER-side; this UI is convenience only.
*/
"use strict";

const CMD_HZ = 20;
const STALE_MS = 500;
const US_BAR_MAX_CM = 200;
// MPU6050 defaults: accel ±2 g -> 16384 counts/g, gyro ±250 dps -> 131 counts/dps
const ACC_SCALE = 16384, GYR_SCALE = 131;

// ---------------------------------------------------------------- websocket
let ws = null, wsOpen = false, backoff = 500;
// control state, driven by telemetry (f.ctrl): held = anyone has it,
// mine = this connection has it
let ctrlMine = false, ctrlHeld = false;

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => { wsOpen = true; backoff = 500; setFlag("flag-ws", true); };
  ws.onclose = () => {
    wsOpen = false; setFlag("flag-ws", false);
    ctrlMine = false; ctrlHeld = false;   // server releases us on disconnect
    updateCtrlBtn();
    setTimeout(connect, backoff);
    backoff = Math.min(backoff * 2, 8000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    try { render(JSON.parse(ev.data)); } catch (_) {}
  };
}
connect();

// ---------------------------------------------------------------- joystick
const canvas = document.getElementById("stick");
const ctx = canvas.getContext("2d");
let stickX = 0, stickY = 0;        // -1..1, y positive = forward
let pointerId = null;

function drawStick() {
  const w = canvas.width, h = canvas.height, cx = w / 2, cy = h / 2;
  const R = w * 0.42, r = w * 0.13;
  ctx.clearRect(0, 0, w, h);
  // well
  ctx.strokeStyle = "#2a312d"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, 7); ctx.stroke();
  ctx.strokeStyle = "#161a18";
  ctx.beginPath(); ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
  ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R); ctx.stroke();
  // knob
  const kx = cx + stickX * (R - r), ky = cy - stickY * (R - r);
  const g = ctx.createRadialGradient(kx, ky, 2, kx, ky, r);
  g.addColorStop(0, "#ffd469"); g.addColorStop(1, "#b87b00");
  ctx.fillStyle = g;
  ctx.beginPath(); ctx.arc(kx, ky, r, 0, 7); ctx.fill();
}
drawStick();

function pointerToStick(ev) {
  const rect = canvas.getBoundingClientRect();
  const cx = rect.width / 2, cy = rect.height / 2;
  let x = (ev.clientX - rect.left - cx) / (rect.width * 0.42);
  let y = -(ev.clientY - rect.top - cy) / (rect.height * 0.42);
  const m = Math.hypot(x, y);
  if (m > 1) { x /= m; y /= m; }
  stickX = x; stickY = y;
}

canvas.addEventListener("pointerdown", (ev) => {
  pointerId = ev.pointerId;
  canvas.setPointerCapture(pointerId);
  pointerToStick(ev); drawStick();
});
canvas.addEventListener("pointermove", (ev) => {
  if (ev.pointerId !== pointerId) return;
  pointerToStick(ev); drawStick();
});
function releaseStick(ev) {
  if (ev.pointerId !== pointerId) return;
  pointerId = null; stickX = 0; stickY = 0; drawStick();
}
canvas.addEventListener("pointerup", releaseStick);
canvas.addEventListener("pointercancel", releaseStick);

// ---------------------------------------------------------------- gamepad
let padIndex = null;
window.addEventListener("gamepadconnected", (e) => { padIndex = e.gamepad.index; setFlag("flag-pad", true); });
window.addEventListener("gamepaddisconnected", () => { padIndex = null; setFlag("flag-pad", false); });

function readPad() {
  if (padIndex === null) return null;
  const gp = navigator.getGamepads()[padIndex];
  if (!gp) return null;
  const dz = (v) => Math.abs(v) < 0.12 ? 0 : v;
  return { x: dz(gp.axes[0] || 0), y: dz(-(gp.axes[1] || 0)) };
}

// ------------------------------------------------------------ mixer + send
let seq = 0;
function mix(x, y) {
  let l = y + x, r = y - x;
  const m = Math.max(1, Math.abs(l), Math.abs(r));   // normalize
  return [l / m, r / m];
}

setInterval(() => {
  // only the controller streams commands; viewers stay silent so the
  // server's deadman logic sees exactly one talker
  if (!ctrlMine) return;
  const pad = readPad();
  const x = pad && (pad.x || pad.y) ? pad.x : stickX;
  const y = pad && (pad.x || pad.y) ? pad.y : stickY;
  const [left, right] = mix(x, y);
  if (wsOpen && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: "cmd", left, right, seq: seq++ }));
  }
}, 1000 / CMD_HZ);   // continuous, including zeros — silence fires the deadman

document.getElementById("reset-btn").addEventListener("click", () => {
  if (wsOpen) ws.send(JSON.stringify({ type: "reset" }));
});

// ---------------------------------------------------------- control switch
// three states:
//   mine            -> green  "RELEASE"        (click gives control up)
//   free (not held) -> amber  "TAKE CONTROL"   (click requests it)
//   held by other   -> grayed "IN USE"         (disabled until they release)
const ctrlBtn = document.getElementById("ctrl-btn");

function updateCtrlBtn() {
  ctrlBtn.classList.toggle("mine", ctrlMine);
  ctrlBtn.classList.toggle("held", ctrlHeld && !ctrlMine);
  ctrlBtn.disabled = ctrlHeld && !ctrlMine;
  ctrlBtn.textContent = ctrlMine ? "RELEASE" :
                        ctrlHeld ? "IN USE" : "TAKE CONTROL";
}
updateCtrlBtn();

ctrlBtn.addEventListener("click", () => {
  if (!wsOpen) return;
  if (ctrlMine) {
    // drop the stick before letting go, so no stale command lingers
    stickX = 0; stickY = 0; drawStick();
    ws.send(JSON.stringify({ type: "release" }));
  } else if (!ctrlHeld) {
    ws.send(JSON.stringify({ type: "take" }));
  }
  // actual state lands via the next telemetry frame (f.ctrl)
});

// ---------------------------------------------------------------- render
const $ = (id) => document.getElementById(id);
let lastFrameWall = 0;

function setFlag(id, on) { $(id).classList.toggle("on", !!on); }

function usRow(cm, barId, valId) {
  if (cm === null || cm === undefined) {
    $(valId).textContent = "—";
    $(barId).style.width = "0%";
  } else {
    $(valId).textContent = cm + " cm";
    $(barId).style.width = Math.min(100, (cm / US_BAR_MAX_CM) * 100) + "%";
  }
}

function dutyBar(el, v) {
  el.classList.toggle("rev", v < 0);
  const pct = Math.min(50, Math.abs(v) * 50);
  el.style.width = pct + "%";
  el.style.left = v >= 0 ? "50%" : (50 - pct) + "%";
}

function render(f) {
  lastFrameWall = performance.now();

  usRow(f.us.front, "us-front-bar", "us-front-val");
  usRow(f.us.rear, "us-rear-bar", "us-rear-val");

  document.querySelectorAll(".dot").forEach((d) => {
    d.classList.toggle("on", !!(f.ir & (1 << +d.dataset.bit)));
  });

  const ax = f.imu.ax / ACC_SCALE, ay = f.imu.ay / ACC_SCALE, az = f.imu.az / ACC_SCALE;
  $("imu-ax").textContent = ax.toFixed(2);
  $("imu-ay").textContent = ay.toFixed(2);
  $("imu-az").textContent = az.toFixed(2);
  $("imu-gx").textContent = (f.imu.gx / GYR_SCALE).toFixed(1);
  $("imu-gy").textContent = (f.imu.gy / GYR_SCALE).toFixed(1);
  $("imu-gz").textContent = (f.imu.gz / GYR_SCALE).toFixed(1);

  // tilt bubble: ±1 g maps to edge of the dial
  const b = $("tilt-bubble");
  b.style.left = (50 + Math.max(-1, Math.min(1, ax)) * 40) + "%";
  b.style.top = (50 - Math.max(-1, Math.min(1, ay)) * 40) + "%";

  dutyBar($("duty-left"), f.motor.left);
  dutyBar($("duty-right"), f.motor.right);
  setFlag("flag-cap", f.motor.capped);

  $("stall-banner").classList.toggle("hidden", !f.motor.stall);
  $("dead-banner").classList.toggle("hidden", !f.dead);

  // control switch state comes from the server, never assumed locally —
  // this is what ungrays the button for everyone when control is released
  if (f.ctrl) {
    ctrlMine = !!f.ctrl.mine;
    ctrlHeld = !!f.ctrl.held;
    updateCtrlBtn();
  }
}

// staleness check
setInterval(() => {
  const stale = performance.now() - lastFrameWall > STALE_MS;
  $("grid").classList.toggle("stale", stale);
  $("conn-warning").classList.toggle("hidden", !stale);
}, 200);