#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <math.h>
#include <vector>
#include <algorithm>

// ===== STRUCTS FIRST =====
struct Inputs {
  float target_cm;
  float u, mass, k, x_spring, efficiency;
  float Cd, rho, diameter;
  float h0, hTarget;
};

struct TrajectoryPoint { float x, y, t; };

struct Solution {
  bool   valid            = false;
  String error;
  float  angle_deg        = 0.0f;
  float  speed_mps        = 0.0f;
  float  range_m          = 0.0f;
  float  flight_time      = 0.0f;
  float  spring_speed_est = 0.0f;
  float  ideal_range_m    = NAN;
  float  ideal_angle_deg  = NAN;
  bool   has_second_angle = false;
  float  angle2_deg       = 0.0f;
  String note;
  std::vector<TrajectoryPoint> dragTraj;
  std::vector<TrajectoryPoint> idealTraj;
};

// ===== CONFIG =====
const char* WIFI_SSID = "Airtel_Zari";
const char* WIFI_PASS = "asus1234";

#define SIMULATION_MODE true
#define I2C_SDA     8
#define I2C_SCL     9
#define OLED_ADDR   0x3C
#define SCREEN_W    128
#define SCREEN_H    64
#define SERVO1_PIN  4
#define SERVO2_PIN  5
#define RELEASE_PULSE_MS 400

static const float G              = 9.80665f;
static const float MIN_DIST_CM    = 1.0f;
static const float MAX_DIST_CM    = 100.0f;
static const float HEIGHT_TOL_M   = 0.001f;
static const float ANGLE_MIN_DEG  = 1.0f;
static const float ANGLE_MAX_DEG  = 89.0f;
static const float SCAN_STEP_DEG  = 2.0f;
static const float SCAN_DT        = 0.002f;
static const float FINAL_DT       = 0.0005f;
static const float RECORD_DT      = 0.020f;
static const int   MAX_POINTS     = 300;
static const float MAX_SIM_TIME   = 5.0f;
static const int   BISECT_ITERS   = 15;
static const float MATCH_TOL_M    = 0.02f;

// ===== OBJECTS AND STATE =====
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
Servo servo1, servo2;
WebServer server(80);

enum State { SAFE, ARMED, LAUNCHING, ERROR_STATE };
State state = SAFE;

float  target_cm    = 0.0f;
float  angle_deg    = 0.0f;
float  speed_mps    = 0.0f;
bool   target_valid = false;
String last_error   = "";
unsigned long launch_start_ms = 0;

// ===== VALIDATION =====
static bool finitePositive(float v)    { return isfinite(v) && v > 0.0f; }
static bool finiteNonNegative(float v) { return isfinite(v) && v >= 0.0f; }

static bool validateInputs(const Inputs& in, String& err) {
  if (!isfinite(in.target_cm))      { err = "Target distance is not finite.";                return false; }
  if (in.target_cm <= 0.0f)         { err = "Target distance must be greater than zero.";    return false; }
  if (in.target_cm < MIN_DIST_CM)   { err = "Target distance below minimum (1 cm).";         return false; }
  if (in.target_cm > MAX_DIST_CM)   { err = "Target distance above 100 cm.";                 return false; }
  if (!finitePositive(in.u))        { err = "Initial speed must be finite and > 0.";         return false; }
  if (!finitePositive(in.mass))     { err = "Projectile mass must be finite and > 0.";       return false; }
  if (!finitePositive(in.k))        { err = "Spring constant must be finite and > 0.";       return false; }
  if (!finitePositive(in.x_spring)) { err = "Spring compression must be finite and > 0.";    return false; }
  if (!finiteNonNegative(in.Cd))    { err = "Drag coefficient must be finite and >= 0.";     return false; }
  if (!finitePositive(in.rho))      { err = "Air density must be finite and > 0.";           return false; }
  if (!finitePositive(in.diameter)) { err = "Projectile diameter must be finite and > 0.";   return false; }
  if (!isfinite(in.h0))             { err = "Launch height is not finite.";                  return false; }
  if (in.h0 < 0.0f)                 { err = "Launch height must be >= 0.";                   return false; }
  if (!isfinite(in.hTarget))        { err = "Target height is not finite.";                  return false; }
  if (in.hTarget < 0.0f)            { err = "Target height must be >= 0.";                   return false; }
  if (in.efficiency <= 0.0f || in.efficiency > 1.0f) { err = "Efficiency factor must be in (0, 1]."; return false; }
  return true;
}

static float crossSectionArea(float d) { float r = d * 0.5f; return PI * r * r; }
static float springSpeedEstimate(float k, float x, float m, float eff) {
  float E = 0.5f * k * x * x;
  return sqrtf(2.0f * E * eff / m);
}

// ===== RK4: state = float[4] {x, y, vx, vy} =====
static void deriv(const float s[4], float kDrag, float d[4]) {
  float v = sqrtf(s[2] * s[2] + s[3] * s[3]);
  d[0] = s[2];
  d[1] = s[3];
  d[2] = -kDrag * s[2] * v;
  d[3] = -G - kDrag * s[3] * v;
}

static void rk4Step(const float s[4], float dt, float kDrag, float out[4]) {
  float k1[4], k2[4], k3[4], k4[4], tmp[4];
  deriv(s, kDrag, k1);
  for (int i = 0; i < 4; i++) tmp[i] = s[i] + 0.5f * dt * k1[i];
  deriv(tmp, kDrag, k2);
  for (int i = 0; i < 4; i++) tmp[i] = s[i] + 0.5f * dt * k2[i];
  deriv(tmp, kDrag, k3);
  for (int i = 0; i < 4; i++) tmp[i] = s[i] + dt * k3[i];
  deriv(tmp, kDrag, k4);
  for (int i = 0; i < 4; i++)
    out[i] = s[i] + (dt / 6.0f) * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);
}

static bool integrateToTarget(const Inputs& in, float theta_rad,
                              std::vector<TrajectoryPoint>& out,
                              float& rangeOut, float& timeOut, String& err) {
  rangeOut = 0.0f; timeOut = 0.0f;
  float A = crossSectionArea(in.diameter);
  float kDrag = (in.rho * in.Cd * A) / (2.0f * in.mass);
  float s[4] = { 0.0f, in.h0, in.u * cosf(theta_rad), in.u * sinf(theta_rad) };
  out.clear();
  out.reserve(MAX_POINTS + 2);
  out.push_back({ s[0], s[1], 0.0f });
  float t = 0.0f, lastRec = 0.0f;
  bool hit = false;
  const float targetY = in.hTarget;
  while (t < MAX_SIM_TIME) {
    float prev[4]; for (int i = 0; i < 4; i++) prev[i] = s[i];
    float next[4]; rk4Step(s, FINAL_DT, kDrag, next);
    for (int i = 0; i < 4; i++) s[i] = next[i];
    t += FINAL_DT;
    if (s[1] <= 0.0f && prev[1] > 0.0f) {
      float frac = prev[1] / (prev[1] - s[1]);
      float xHit = prev[0] + frac * (s[0] - prev[0]);
      float tHit = (t - FINAL_DT) + frac * FINAL_DT;
      out.push_back({ xHit, 0.0f, tHit });
      rangeOut = xHit; timeOut = tHit; hit = true; break;
    }
    if (s[3] < 0.0f && s[1] <= targetY && prev[1] > targetY) {
      float frac = (prev[1] - targetY) / (prev[1] - s[1]);
      float xHit = prev[0] + frac * (s[0] - prev[0]);
      float tHit = (t - FINAL_DT) + frac * FINAL_DT;
      out.push_back({ xHit, targetY, tHit });
      rangeOut = xHit; timeOut = tHit; hit = true; break;
    }
    if (t - lastRec >= RECORD_DT && (int)out.size() < MAX_POINTS) {
      out.push_back({ s[0], s[1], t });
      lastRec = t;
    }
  }
  if (!hit) { err = "Trajectory did not reach target height within time limit."; return false; }
  return true;
}

static void integrateIdeal(const Inputs& in, float theta_rad,
                           std::vector<TrajectoryPoint>& out) {
  out.clear();
  out.reserve(MAX_POINTS);
  float ux = in.u * cosf(theta_rad);
  float uy = in.u * sinf(theta_rad);
  for (float t = 0.0f; t < MAX_SIM_TIME; t += RECORD_DT) {
    float x = ux * t;
    float y = in.h0 + uy * t - 0.5f * G * t * t;
    out.push_back({ x, y, t });
    if (y < 0.0f) break;
    if ((int)out.size() >= MAX_POINTS) break;
  }
}

static float xAtTargetHeightDescending(const Inputs& in, float theta_deg) {
  float A = crossSectionArea(in.diameter);
  float kDrag = (in.rho * in.Cd * A) / (2.0f * in.mass);
  float th = theta_deg * DEG_TO_RAD;
  float s[4] = { 0.0f, in.h0, in.u * cosf(th), in.u * sinf(th) };
  float t = 0.0f;
  while (t < MAX_SIM_TIME) {
    float prev[4]; for (int i = 0; i < 4; i++) prev[i] = s[i];
    float next[4]; rk4Step(s, SCAN_DT, kDrag, next);
    for (int i = 0; i < 4; i++) s[i] = next[i];
    t += SCAN_DT;
    if (s[3] < 0.0f && s[1] <= in.hTarget && prev[1] > in.hTarget) {
      float frac = (prev[1] - in.hTarget) / (prev[1] - s[1]);
      return prev[0] + frac * (s[0] - prev[0]);
    }
    if (s[1] <= 0.0f && prev[1] > 0.0f) return -1.0f;
  }
  return -1.0f;
}

static bool findLaunchAngles(const Inputs& in, Solution& sol) {
  const float target_m = in.target_cm / 100.0f;
  struct Hit { float deg; float x; };
  std::vector<Hit> hits;
  for (float deg = ANGLE_MIN_DEG; deg <= ANGLE_MAX_DEG; deg += SCAN_STEP_DEG) {
    float x = xAtTargetHeightDescending(in, deg);
    if (x >= 0.0f) hits.push_back({ deg, x });
  }
  std::vector<float> candidates;
  for (size_t i = 1; i < hits.size(); ++i) {
    float d_lo = hits[i-1].x - target_m;
    float d_hi = hits[i].x   - target_m;
    if (d_lo * d_hi > 0.0f) continue;
    float lo = hits[i-1].deg, hi = hits[i].deg;
    bool ok = true;
    for (int k = 0; k < BISECT_ITERS; ++k) {
      float mid = 0.5f * (lo + hi);
      float x   = xAtTargetHeightDescending(in, mid);
      if (x < 0.0f) { ok = false; break; }
      float d = x - target_m;
      if (fabsf(d) < 1e-4f) { lo = hi = mid; break; }
      if ((d < 0.0f) == (d_lo < 0.0f)) { lo = mid; d_lo = d; }
      else                              { hi = mid; }
    }
    if (!ok) continue;
    float angle = 0.5f * (lo + hi);
    bool dup = false;
    for (float c : candidates) if (fabsf(c - angle) < 0.5f) { dup = true; break; }
    if (!dup) candidates.push_back(angle);
  }
  if (candidates.empty()) {
    sol.valid = false;
    sol.error = "No valid launch angle found. Target is physically unreachable with these parameters.";
    return false;
  }
  std::sort(candidates.begin(), candidates.end());
  sol.angle_deg        = candidates[0];
  sol.has_second_angle = (candidates.size() >= 2);
  if (sol.has_second_angle) sol.angle2_deg = candidates[1];
  return true;
}

static Solution computeSolution(const Inputs& in) {
  Solution sol;
  String err;
  if (!validateInputs(in, err)) { sol.valid = false; sol.error = err; return sol; }
  sol.speed_mps        = in.u;
  sol.spring_speed_est = springSpeedEstimate(in.k, in.x_spring, in.mass, in.efficiency);
  const float target_m    = in.target_cm / 100.0f;
  const bool  equalHeight = fabsf(in.h0 - in.hTarget) < HEIGHT_TOL_M;
  if (equalHeight) {
    float ratio = target_m * G / (in.u * in.u);
    if (ratio >= -1.0f && ratio <= 1.0f) {
      float thIdeal       = 0.5f * asinf(ratio);
      sol.ideal_angle_deg = thIdeal * RAD_TO_DEG;
      sol.ideal_range_m   = (in.u * in.u * sinf(2.0f * thIdeal)) / G;
    }
  }
  if (!findLaunchAngles(in, sol)) return sol;
  float theta_rad = sol.angle_deg * DEG_TO_RAD;
  float range, time;
  String ierr;
  if (!integrateToTarget(in, theta_rad, sol.dragTraj, range, time, ierr)) {
    sol.valid = false;
    sol.error = "Angle found but final integration failed: " + ierr;
    return sol;
  }
  if (fabsf(range - target_m) > MATCH_TOL_M) {
    sol.valid = false;
    sol.error = "Drag trajectory missed the target by more than the match tolerance.";
    return sol;
  }
  sol.valid       = true;
  sol.range_m     = range;
  sol.flight_time = time;
  integrateIdeal(in, theta_rad, sol.idealTraj);
  sol.note = equalHeight
    ? "Equal launch/target height. Drag-corrected angle found numerically."
    : "Different launch/target heights. The equal-height range formula is not applicable; drag-corrected angle found numerically.";
  return sol;
}

// ===== SERVO / DISPLAY =====
static void safeWrite(Servo& s, int deg) { if (!SIMULATION_MODE) s.write(deg); }

static void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  String st = (state == SAFE) ? "SAFE" :
              (state == ARMED) ? "ARMED" :
              (state == LAUNCHING) ? "LAUNCHING" : "ERROR";
  display.setCursor(0,  0); display.print("State:  " + st + (SIMULATION_MODE ? " [SIM]" : ""));
  display.setCursor(0, 10); display.print("Target: " + String(target_cm, 1) + " cm");
  display.setCursor(0, 20); display.print("Angle:  " + String(angle_deg, 1) + " deg");
  display.setCursor(0, 30); display.print("Speed:  " + String(speed_mps, 2) + " m/s");
  display.setCursor(0, 40); display.print("Valid:  " + String(target_valid ? "YES" : "NO"));
  display.setCursor(0, 50); display.print("Error:  " + (last_error.length() ? last_error.substring(0, 20) : "none"));
  display.display();
}

static void goSafe() {
  state = SAFE;
  safeWrite(servo1, 0);
  safeWrite(servo2, 0);
  updateDisplay();
}

// ===== EMBEDDED HTML =====
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Projectile Motion</title>
<style>
  body { font-family: system-ui, sans-serif; max-width: 960px; margin: 1rem auto; padding: 0 1rem; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 0.6rem; }
  label { display: block; font-size: 0.85rem; margin-top: 0.4rem; }
  input { width: 100%; padding: 0.35rem; box-sizing: border-box; }
  button { padding: 0.5rem 0.8rem; margin: 0.3rem 0.2rem 0.3rem 0; cursor: pointer; }
  #results { margin-top: 1rem; padding: 0.8rem; background: #f4f4f4; border-radius: 6px; white-space: pre-wrap; }
  #status { margin-top: 0.6rem; font-size: 0.85rem; }
  .badge { display: inline-block; padding: 0.15rem 0.5rem; border-radius: 999px; font-size: 0.75rem; margin-right: 0.4rem; }
  .safe { background: #c8e6c9; } .armed { background: #ffe082; }
  .launching { background: #ffccbc; } .error { background: #ef9a9a; }
  canvas { border: 1px solid #ccc; width: 100%; height: 320px; }
</style>
</head>
<body>
<h2>Projectile Motion - ESP32-S3 (Simulation Mode)</h2>
<div class="grid">
  <div><label>Target distance (cm, 1-100)</label><input id="target_cm" value="80"></div>
  <div><label>Initial speed u (m/s)</label><input id="u" value="8"></div>
  <div><label>Mass (kg)</label><input id="mass" value="0.05"></div>
  <div><label>Spring constant k (N/m)</label><input id="k" value="120"></div>
  <div><label>Spring compression x (m)</label><input id="x_spring" value="0.08"></div>
  <div><label>Efficiency (0-1)</label><input id="efficiency" value="0.35"></div>
  <div><label>Drag coefficient Cd</label><input id="Cd" value="0.47"></div>
  <div><label>Air density rho (kg/m3)</label><input id="rho" value="1.225"></div>
  <div><label>Projectile diameter (m)</label><input id="diameter" value="0.02"></div>
  <div><label>Launch height h0 (m)</label><input id="h0" value="0.5"></div>
  <div><label>Target height hT (m)</label><input id="hTarget" value="0.5"></div>
</div>
<button onclick="calculate()">Calculate</button>
<button onclick="resetForm()">Reset</button>
<hr>
<button onclick="arm()">Arm</button>
<button onclick="disarm()">Disarm</button>
<button onclick="launch()">Launch</button>
<div id="results">Results appear here.</div>
<div id="status">System status: waiting...</div>
<canvas id="graph" width="900" height="320"></canvas>
<script>
const fields = ["target_cm","u","mass","k","x_spring","efficiency","Cd","rho","diameter","h0","hTarget"];
function val(id){ return parseFloat(document.getElementById(id).value); }
function payload(){ const o = {}; fields.forEach(f => o[f] = val(f)); return o; }
async function calculate() {
  document.getElementById("results").textContent = "Calculating...";
  let data;
  try {
    const res = await fetch("/calculate", {
      method:"POST", headers:{"Content-Type":"application/json"},
      body: JSON.stringify(payload())
    });
    data = await res.json();
  } catch (e) {
    document.getElementById("results").textContent = "Request failed: " + e;
    return;
  }
  const box = document.getElementById("results");
  if (!data.valid) { box.textContent = "ERROR: " + data.error; drawGraph([], []); return; }
  const fmt = (v, n) => (typeof v === "number" && isFinite(v)) ? v.toFixed(n) : "n/a";
  box.textContent =
    "Estimated angle: " + fmt(data.angle_deg, 2) + " deg" +
    (data.has_second_angle ? "  (second: " + fmt(data.angle2_deg, 2) + " deg)" : "") +
    "\nSpeed used: " + fmt(data.speed_mps, 2) + " m/s" +
    "\nSpring-energy speed estimate: " + fmt(data.spring_speed_est, 2) + " m/s (requires calibration)" +
    "\nPredicted range (drag): " + fmt(data.range_m, 3) + " m" +
    "\nFlight time: " + fmt(data.flight_time, 3) + " s" +
    "\nIdeal range (no drag): " + fmt(data.ideal_range_m, 3) + " m" +
    "\nIdeal angle (closed form): " + fmt(data.ideal_angle_deg, 2) + " deg" +
    "\nNote: " + data.note;
  drawGraph(data.ideal, data.drag);
}
function drawGraph(ideal, drag) {
  const c = document.getElementById("graph"), ctx = c.getContext("2d");
  ctx.clearRect(0, 0, c.width, c.height);
  const pts = (ideal || []).concat(drag || []);
  if (!pts.length) return;
  let maxX = 0.1, maxY = 0.1;
  pts.forEach(p => { maxX = Math.max(maxX, p[0]); maxY = Math.max(maxY, p[1]); });
  maxX *= 1.1; maxY *= 1.1;
  const m = 40, w = c.width - m * 2, h = c.height - m * 2;
  const sx = x => m + (x / maxX) * w;
  const sy = y => c.height - m - (y / maxY) * h;
  ctx.strokeStyle = "#999"; ctx.beginPath();
  ctx.moveTo(m, c.height - m); ctx.lineTo(c.width - m, c.height - m);
  ctx.moveTo(m, c.height - m); ctx.lineTo(m, m); ctx.stroke();
  function plot(arr, color, dash) {
    if (!arr || !arr.length) return;
    ctx.strokeStyle = color; ctx.setLineDash(dash || []);
    ctx.beginPath();
    arr.forEach((p, i) => i ? ctx.lineTo(sx(p[0]), sy(p[1])) : ctx.moveTo(sx(p[0]), sy(p[1])));
    ctx.stroke(); ctx.setLineDash([]);
  }
  plot(ideal, "#1565c0", [5, 4]);
  plot(drag,  "#c62828", []);
  ctx.fillStyle = "#333"; ctx.font = "12px sans-serif";
  ctx.fillText("ideal (no drag) - dashed blue", m, 20);
  ctx.fillText("drag (RK4) - solid red", m + 190, 20);
}
async function arm() {
  if (!confirm("Arm the aiming servo?")) return;
  await fetch("/arm", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({confirm:true})});
}
async function disarm(){ await fetch("/disarm", {method:"POST"}); }
async function launch(){
  if (!confirm("LAUNCH?")) return;
  await fetch("/launch", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({confirm:true})});
}
function resetForm() {
  const d = {target_cm:80,u:8,mass:0.05,k:120,x_spring:0.08,efficiency:0.35,Cd:0.47,rho:1.225,diameter:0.02,h0:0.5,hTarget:0.5};
  fields.forEach(f => document.getElementById(f).value = d[f]);
  document.getElementById("results").textContent = "Results appear here.";
  drawGraph([], []);
}
function safeStr(v, fallback){ return (v === undefined || v === null || v === "") ? fallback : v; }
setInterval(async () => {
  try {
    const r = await fetch("/status");
    const s = await r.json();
    const el = document.getElementById("status");
    el.innerHTML =
      '<span class="badge ' + (s.state || "error").toLowerCase() + '">' + safeStr(s.state, "?") + '</span>' +
      'target ' + safeStr(s.target_cm, "-") + ' cm . angle ' + safeStr(s.angle_deg, "-") + ' deg . ' +
      'speed ' + safeStr(s.speed_mps, "-") + ' m/s . valid ' + (s.valid ? "YES" : "NO") + ' . sim ' + (s.sim ? "ON" : "OFF") +
      (s.error ? ' . <span style="color:#b71c1c">' + s.error + '</span>' : '');
  } catch (e) {
    document.getElementById("status").textContent = "System status: unreachable";
  }
}, 800);
</script>
</body>
</html>
)rawliteral";

// ===== WEB HANDLERS =====
static void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

static void handleCalculate() {
  if (state == LAUNCHING) {
    server.send(409, "application/json", "{\"error\":\"Launch in progress.\"}");
    return;
  }
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"error\":\"Missing JSON body\"}"); return; }
  StaticJsonDocument<512> req;
  if (deserializeJson(req, server.arg("plain"))) { server.send(400, "application/json", "{\"error\":\"Malformed JSON\"}"); return; }
  Inputs in;
  in.target_cm  = req["target_cm"]  | NAN;
  in.u          = req["u"]          | NAN;
  in.mass       = req["mass"]       | NAN;
  in.k          = req["k"]          | NAN;
  in.x_spring   = req["x_spring"]   | NAN;
  in.efficiency = req["efficiency"] | 0.35f;
  in.Cd         = req["Cd"]         | NAN;
  in.rho        = req["rho"]        | NAN;
  in.diameter   = req["diameter"]   | NAN;
  in.h0         = req["h0"]         | NAN;
  in.hTarget    = req["hTarget"]    | NAN;
  Solution sol = computeSolution(in);
  if (state != SAFE) goSafe();
  target_cm    = in.target_cm;
  angle_deg    = sol.angle_deg;
  speed_mps    = sol.speed_mps;
  target_valid = sol.valid;
  last_error   = sol.valid ? "" : sol.error;
  updateDisplay();
  DynamicJsonDocument res(32768);
  res["valid"]            = sol.valid;
  res["error"]            = sol.error;
  res["angle_deg"]        = sol.angle_deg;
  res["angle2_deg"]       = sol.has_second_angle ? sol.angle2_deg : sol.angle_deg;
  res["has_second_angle"] = sol.has_second_angle;
  res["speed_mps"]        = sol.speed_mps;
  res["spring_speed_est"] = sol.spring_speed_est;
  res["range_m"]          = sol.range_m;
  res["flight_time"]      = sol.flight_time;
  res["ideal_range_m"]    = sol.ideal_range_m;
  res["ideal_angle_deg"]  = sol.ideal_angle_deg;
  res["note"]             = sol.note;
  JsonArray drag = res.createNestedArray("drag");
  for (auto& p : sol.dragTraj) { JsonArray pt = drag.createNestedArray();  pt.add(p.x); pt.add(p.y); }
  JsonArray ideal = res.createNestedArray("ideal");
  for (auto& p : sol.idealTraj) { JsonArray pt = ideal.createNestedArray(); pt.add(p.x); pt.add(p.y); }
  String out;
  serializeJson(res, out);
  if (out.length() >= 32000) { server.send(500, "application/json", "{\"error\":\"Response too large\"}"); return; }
  server.send(200, "application/json", out);
}

static void handleArm() {
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"error\":\"Missing body\"}"); return; }
  StaticJsonDocument<64> req;
  if (deserializeJson(req, server.arg("plain")) || req["confirm"] != true) {
    server.send(400, "application/json", "{\"error\":\"Arming requires confirmation\"}");
    return;
  }
  if (!target_valid) { server.send(400, "application/json", "{\"error\":\"No valid calculation\"}"); return; }
  if (state != SAFE)  { server.send(400, "application/json", "{\"error\":\"Not in SAFE state\"}"); return; }
  if (!isfinite(angle_deg) || angle_deg < 0.0f || angle_deg > 180.0f) {
    server.send(500, "application/json", "{\"error\":\"Angle out of range\"}");
    return;
  }
  state = ARMED;
  safeWrite(servo1, (int)constrain(angle_deg, 0.0f, 180.0f));
  safeWrite(servo2, 0);
  updateDisplay();
  server.send(200, "application/json", "{\"status\":\"armed\"}");
}

static void handleDisarm() {
  goSafe();
  server.send(200, "application/json", "{\"status\":\"safe\"}");
}

static void handleLaunch() {
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"error\":\"Missing body\"}"); return; }
  StaticJsonDocument<64> req;
  if (deserializeJson(req, server.arg("plain")) || req["confirm"] != true) {
    server.send(400, "application/json", "{\"error\":\"Launch requires confirmation\"}");
    return;
  }
  if (state != ARMED) { server.send(400, "application/json", "{\"error\":\"Not ARMED\"}"); return; }
  state = LAUNCHING;
  launch_start_ms = millis();
  safeWrite(servo2, 90);
  updateDisplay();
  server.send(200, "application/json", "{\"status\":\"launching\"}");
}

static void handleStatus() {
  StaticJsonDocument<256> doc;
  doc["state"]     = (state == SAFE) ? "SAFE" :
                     (state == ARMED) ? "ARMED" :
                     (state == LAUNCHING) ? "LAUNCHING" : "ERROR";
  doc["target_cm"] = target_cm;
  doc["angle_deg"] = angle_deg;
  doc["speed_mps"] = speed_mps;
  doc["valid"]     = target_valid;
  doc["error"]     = last_error;
  doc["sim"]       = SIMULATION_MODE;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// ===== SETUP / LOOP =====
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay(); display.display();

  if (!SIMULATION_MODE) {
    servo1.attach(SERVO1_PIN, 500, 2500);
    servo2.attach(SERVO2_PIN, 500, 2500);
    servo1.write(0);
    servo2.write(0);
    Serial.println("Servos attached and set to 0 degrees.");
  } else {
    Serial.println("Simulation mode: servos are NOT attached.");
  }

  state = SAFE;
  updateDisplay();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  const unsigned long joinStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - joinStart < 20000) {
    delay(500); Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi failed. Check credentials/AP. Web UI unavailable.");
  }

  server.on("/",          HTTP_GET,  handleRoot);
  server.on("/calculate", HTTP_POST, handleCalculate);
  server.on("/arm",       HTTP_POST, handleArm);
  server.on("/disarm",    HTTP_POST, handleDisarm);
  server.on("/launch",    HTTP_POST, handleLaunch);
  server.on("/status",    HTTP_GET,  handleStatus);
  server.begin();

  Serial.println("Web server started. Simulation mode: " +
                 String(SIMULATION_MODE ? "ON" : "OFF"));
}

void loop() {
  server.handleClient();
  if (state == LAUNCHING && millis() - launch_start_ms > RELEASE_PULSE_MS) {
    goSafe();
  }
  static unsigned long lastOled = 0;
  if (millis() - lastOled > 1000) {
    lastOled = millis();
    updateDisplay();
  }
}