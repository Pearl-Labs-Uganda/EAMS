/* RC car client.
   - touch joystick + Gamepad API -> same arcade mixer
   - sends {"type":"cmd",left,right,seq} at 20 Hz, including zeros
   - renders merged 20 Hz telemetry; Jetson ships raw, we scale here
   - staleness gray-out, STALL/DEADMAN banners, WS auto-reconnect w/ backoff
   - control switch: many viewers, ONE driver. Must send {"type":"take"} to
     drive; {"type":"release"} gives it up. The button grays out while
     someone else holds control and ungrays the moment they release or
     disconnect. Enforcement is SERVER-side; this UI is convenience only.
   - AUTONOMY LAB: slide-out panel toggles sensor mode + motor output,
     edits the dummy scenario and the policy target, engages/disengages
     the policy, and shows a live observability panel.
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
    let f;
    try { f = JSON.parse(ev.data); } catch (_) { return; }
    if (f && f.type === "ack") { handleAck(f); return; }
    render(f);
  };
}
connect();

function wsSend(obj) {
  if (wsOpen && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

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
  wsSend({ type: "cmd", left, right, seq: seq++ });
}, 1000 / CMD_HZ);   // continuous, including zeros — silence fires the deadman

document.getElementById("reset-btn").addEventListener("click", () => {
  wsSend({ type: "reset" });
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
    wsSend({ type: "release" });
  } else if (!ctrlHeld) {
    wsSend({ type: "take" });
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

  document.querySelectorAll("#ir-dots .dot").forEach((d) => {
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

  // ---- mode / policy flag chips in the pilot page ---------------------
  const sm = f.sensor_mode || "hardware";
  const smFlag = $("flag-sensor");
  smFlag.textContent = "SENS:" + (sm === "hardware" ? "HW" :
                                  sm === "dummy_static" ? "STA" : "KIN");
  smFlag.classList.toggle("dummy", sm !== "hardware");

  const motorReal = !!f.motor.output_enabled;
  const mFlag = $("flag-motor");
  mFlag.textContent = "MTR:" + (motorReal ? "REAL" : "DUMMY");
  mFlag.classList.toggle("dummy", !motorReal);

  const pol = f.policy;
  const pFlag = $("flag-policy");
  const engaged = !!(pol && pol.engaged);
  pFlag.textContent = "POL:" + (engaged ? "ON" : "OFF");
  pFlag.classList.toggle("on", engaged);

  // ---- Autonomy Lab -----------------------------------------------------
  renderLab(f);
}

// staleness check
setInterval(() => {
  const stale = performance.now() - lastFrameWall > STALE_MS;
  $("grid").classList.toggle("stale", stale);
  $("conn-warning").classList.toggle("hidden", !stale);
}, 200);

// =====================================================================
// AUTONOMY LAB
// =====================================================================
const labPanel = $("lab-panel");
$("lab-open").addEventListener("click", () => labPanel.classList.remove("hidden"));
$("lab-close").addEventListener("click", () => labPanel.classList.add("hidden"));

// ---- sensor mode segmented control
const segSensor = $("seg-sensor");
segSensor.querySelectorAll("button").forEach((btn) => {
  btn.addEventListener("click", () => {
    wsSend({ type: "set_sensor_mode", mode: btn.dataset.mode });
  });
});
function setSegActive(seg, key, dataKey) {
  seg.querySelectorAll("button").forEach((b) => {
    b.classList.toggle("active", b.dataset[dataKey] === key);
  });
}

// ---- motor output segmented control (with confirm on real)
const segMotor = $("seg-motor");
segMotor.querySelectorAll("button").forEach((btn) => {
  btn.addEventListener("click", () => {
    const wantReal = btn.dataset.motor === "real";
    if (wantReal && !confirm(
        "Enable real motor output?\n\nH-bridge pins will be driven this tick. Make sure wheels are clear."
    )) return;
    wsSend({ type: "set_motor_output", enabled: wantReal });
  });
});

// ---- static scenario editor
let currentScenario = null;
let irBits = 0;

function bindScenarioInputs() {
  ["sc-usf","sc-usr","sc-ir","sc-gz","sc-ax"].forEach((id) => {
    $(id).addEventListener("change", () => {}); // apply is explicit
  });
  document.querySelectorAll(".ir-bit").forEach((btn) => {
    btn.addEventListener("click", () => {
      const bit = +btn.dataset.bit;
      irBits ^= (1 << bit);
      btn.classList.toggle("on", !!(irBits & (1 << bit)));
      $("sc-ir").value = irBits;
    });
  });
  $("sc-apply").addEventListener("click", () => {
    const fields = {
      us_front_cm: +$("sc-usf").value || 0,
      us_rear_cm: +$("sc-usr").value || 0,
      ir_mask: (+$("sc-ir").value) & 0x3F,
      imu_gz_dps: +$("sc-gz").value || 0,
      imu_ax_g: +$("sc-ax").value || 0,
    };
    wsSend({ type: "set_scenario", fields });
  });
}
bindScenarioInputs();

function syncScenarioUI(s) {
  if (!s) return;
  // Only overwrite if the user isn't actively editing (input not focused)
  if (document.activeElement && document.activeElement.tagName === "INPUT") return;
  $("sc-usf").value = s.us_front_cm;
  $("sc-usr").value = s.us_rear_cm;
  $("sc-ir").value = s.ir_mask;
  $("sc-gz").value = s.imu_gz_dps;
  $("sc-ax").value = s.imu_ax_g;
  irBits = s.ir_mask & 0x3F;
  document.querySelectorAll(".ir-bit").forEach((btn) => {
    const bit = +btn.dataset.bit;
    btn.classList.toggle("on", !!(irBits & (1 << bit)));
  });
}

// ---- target editor
$("tgt-apply").addEventListener("click", () => {
  const distance_m = +$("tgt-dist").value || 0;
  const bearing_deg = +$("tgt-bear").value || 0;
  wsSend({ type: "set_target", distance_m, bearing_deg });
});

function syncTargetUI(t) {
  if (!t) return;
  if (document.activeElement && document.activeElement.tagName === "INPUT") return;
  $("tgt-dist").value = t.distance_m.toFixed(1);
  $("tgt-bear").value = t.bearing_deg.toFixed(0);
  // compass arrow: bearing 0 = up, positive = left = counterclockwise
  $("tc-arrow").style.transform = `translate(-50%, 0) rotate(${-t.bearing_deg}deg)`;
}

// ---- engage
const engageBtn = $("engage-btn");
engageBtn.addEventListener("click", () => {
  const engaged = engageBtn.classList.contains("engaged");
  if (engaged) {
    wsSend({ type: "policy_disengage" });
  } else {
    wsSend({ type: "policy_engage",
             wheels_off_ground: $("wheels-off").checked });
  }
});

function handleAck(a) {
  if (a.kind === "policy_engage" && !a.ok) {
    $("engage-hint").textContent = "engage refused: " + a.reason;
  } else if (a.kind === "policy_engage" && a.ok) {
    $("engage-hint").textContent = "engaged";
  } else if (a.kind === "policy_disengage" && a.ok) {
    $("engage-hint").textContent = "disengaged";
  } else if (a.kind === "set_motor_output" && !a.ok) {
    alert("motor output change refused: " + a.reason);
  } else if (a.kind === "set_sensor_mode" && !a.ok) {
    $("sensor-hint").textContent = "refused: " + a.reason;
  }
}

// ---- policy live rendering
function polBar(id, v) {
  const el = $(id);
  el.classList.toggle("rev", v < 0);
  const pct = Math.min(50, Math.abs(v) * 50);
  el.style.width = pct + "%";
  el.style.left = v >= 0 ? "50%" : (50 - pct) + "%";
  $(id + "-val").textContent = v.toFixed(2);
}

const traceCanvas = $("trace");
const traceCtx = traceCanvas.getContext("2d");
const traceLen = 200;                        // ~10 s at 20 Hz
const traceThrottle = new Array(traceLen).fill(0);
const traceSteer = new Array(traceLen).fill(0);
const traceUs = new Array(traceLen).fill(1);

function pushTrace(pol) {
  traceThrottle.shift(); traceThrottle.push(pol.throttle);
  traceSteer.shift();    traceSteer.push(pol.steer);
  const usn = pol.obs && pol.obs.us_norm
              ? Math.min(pol.obs.us_norm.front, pol.obs.us_norm.rear)
              : 1;
  traceUs.shift(); traceUs.push(usn);
}

function drawTrace() {
  const W = traceCanvas.width, H = traceCanvas.height;
  traceCtx.clearRect(0, 0, W, H);
  // center line
  traceCtx.strokeStyle = "#2a312d";
  traceCtx.beginPath(); traceCtx.moveTo(0, H/2); traceCtx.lineTo(W, H/2); traceCtx.stroke();
  const drawSeries = (arr, color, mapY) => {
    traceCtx.strokeStyle = color; traceCtx.lineWidth = 1.2;
    traceCtx.beginPath();
    for (let i = 0; i < arr.length; i++) {
      const x = (i / (arr.length - 1)) * W;
      const y = mapY(arr[i]);
      if (i === 0) traceCtx.moveTo(x, y); else traceCtx.lineTo(x, y);
    }
    traceCtx.stroke();
  };
  // throttle/steer in [-1,1] map to [H, 0] with center at H/2
  drawSeries(traceThrottle, "#ffb000", v => H/2 - v * (H/2 - 4));
  drawSeries(traceSteer,    "#4aa3ff", v => H/2 - v * (H/2 - 4));
  // us-norm in [0,1] map to bottom-half amplitude (0=red spike UP)
  drawSeries(traceUs,       "#ff4444", v => H - v * (H - 4));
}

function renderLab(f) {
  // segmented button states
  setSegActive(segSensor, f.sensor_mode || "hardware", "mode");
  setSegActive(segMotor, f.motor.output_enabled ? "real" : "dummy", "motor");

  // sensor mode hint
  const smHint = {
    hardware: "Real sensors via GPIO/I²C.",
    dummy_static: "Sensors report the static scenario below. Edit and APPLY.",
    dummy_kinematic: "Sensors derived from a fake pose driven by applied motor duty.",
  }[f.sensor_mode || "hardware"];
  $("sensor-hint").textContent = smHint;

  // scenario card visibility
  $("scenario-card").style.display =
    (f.sensor_mode === "dummy_static") ? "" : "none";
  if (f.scenario) { currentScenario = f.scenario; syncScenarioUI(f.scenario); }

  // engage button state
  const pol = f.policy;
  const loaded = !!f.policy_loaded;
  if (!loaded) {
    engageBtn.disabled = true;
    engageBtn.textContent = "POLICY NOT LOADED";
    $("engage-hint").textContent = "server booted without an ONNX policy";
  } else {
    engageBtn.disabled = false;
    const eng = !!(pol && pol.engaged);
    engageBtn.classList.toggle("engaged", eng);
    engageBtn.textContent = eng ? "DISENGAGE" : "ENGAGE POLICY";
  }

  if (pol) {
    // target sync
    if (pol.target) syncTargetUI(pol.target);
    // action bars
    polBar("pb-throttle", pol.throttle || 0);
    polBar("pb-steer", pol.steer || 0);
    // wheels
    $("pw-tl").textContent = (pol.target_left  || 0).toFixed(2);
    $("pw-tr").textContent = (pol.target_right || 0).toFixed(2);
    $("pw-al").textContent = f.motor.left.toFixed(2);
    $("pw-ar").textContent = f.motor.right.toFixed(2);
    // obs
    if (pol.obs) {
      const o = pol.obs;
      if (o.us_norm) {
        $("obs-usf").textContent = o.us_norm.front.toFixed(2);
        $("obs-usr").textContent = o.us_norm.rear.toFixed(2);
      }
      $("obs-yaw").textContent = (o.yaw_rate_rad_s || 0).toFixed(2);
      $("obs-yaw-n").textContent = (o.yaw_norm || 0).toFixed(2);
      if (o.target) {
        $("obs-tgt-d").textContent = o.target.distance_m.toFixed(1);
        $("obs-tgt-b").textContent = o.target.bearing_deg.toFixed(0);
      }
      // IR in policy order
      const irLabels = ["FL","FR","RL","RR","L","R"];
      document.querySelectorAll(".ir-policy .dot").forEach((d) => {
        const label = irLabels[+d.dataset.idx];
        d.classList.toggle("on", !!(o.ir && o.ir[label]));
      });
    }
    $("obs-lat").textContent    = (pol.latency_ms || 0).toFixed(1);
    $("obs-lat-max").textContent = (pol.max_latency_ms || 0).toFixed(1);

    pushTrace(pol);
    drawTrace();
  }
}
