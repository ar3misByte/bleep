// Bleep -- Wall Node 2 (wall_n.ino)
//
// Pairs with worker_n.ino. wall_node.ino and worker_node.ino are left
// untouched -- this is a second, independent pair, not a replacement.
//
// Two things this board does that the original wall_node.ino did not:
//   1. It has its OWN local sensors wired directly to it (see
//      "Node 2 -- ESP32 connections" pin map below) -- an MPU6050,
//      BMP180 and DHT11 for room/wall-mounted environmental readings,
//      plus an HC-SR04 ultrasonic sensor for distance/proximity (e.g.
//      a door, a hazard boundary, or clearance sensing). These are
//      read and printed to Serial on their own, independent of
//      anything coming over the radio.
//   2. It still hosts the SoftAP + ESP-NOW receiver + embedded
//      dashboard, exactly like wall_node.ino, but the SAME Serial
//      port now interleaves three kinds of output: this board's own
//      local sensor readings, and worker_n.ino's STATUS/DISTRESS
//      packets (which now include a tinyML anomalyScore field) --
//      all tagged so they're easy to tell apart in one Serial Monitor
//      window, and all merged into one JSON API for the dashboard.
//
// PIN MAP -- "Node 2 -- ESP32 connections":
//   MPU6050         VCC->3.3V  GND->GND  SDA->21  SCL->22
//   BMP180          VCC->3.3V  GND->GND  SDA->21  SCL->22  (shares the bus)
//   DHT11           VCC->3.3V  GND->GND  DATA->4
//   Ultrasonic      VCC->5V/VIN  GND->GND  TRIG->5  ECHO->18
//     HC-SR04
//
// Requires: ESP32 Arduino core >= 2.0.0 (esp_now_recv_info_t callback
// signature), ArduinoJson (v6.x), Adafruit MPU6050, Adafruit BMP085,
// DHT sensor library -- same library set as wall_node.ino plus the
// two used here for local sensing.

#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <DHT.h>
#include "tinyml_model.h"

// ---------------------------------------------------------------------
// EDIT THESE
// ---------------------------------------------------------------------
const char* AP_SSID     = "Bleep-WallNode2";
const char* AP_PASSWORD = "bleepsafety";

// Must match worker_n.ino's WIFI_CHANNEL exactly.
const int WIFI_CHANNEL = 6;

const char* NODE_ID = "WALL2";

// worker_n.ino's MAC -- read off that board's own "Worker MAC: ..."
// boot log.
const uint8_t WORKER_MAC[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// ---------------------------------------------------------------------
// LOCAL SENSOR PIN MAP -- "Node 2 -- ESP32 connections"
// ---------------------------------------------------------------------
const int SDA_PIN = 21;
const int SCL_PIN = 22;

const int DHT_PIN = 4;
#define DHT_TYPE DHT11

const int ULTRASONIC_TRIG_PIN = 5;
const int ULTRASONIC_ECHO_PIN = 18;

const unsigned long LOCAL_SENSOR_INTERVAL_MS = 1000;   // MPU6050/BMP180/DHT11/ultrasonic
const unsigned long ULTRASONIC_TIMEOUT_US = 25000UL;   // ~4.3m round trip at 343 m/s, plenty for a room

// ---------------------------------------------------------------------
// ESP-NOW PACKET -- MUST MATCH worker_n.ino EXACTLY
// ---------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  char workerId[8];
  uint8_t msgType;              // 0 = STATUS, 1 = DISTRESS
  char riskState[16];
  char hazardType[16];
  float motionEnergy;
  float secondsSinceMotion;
  float motionEnergyAtTrigger;
  float temperatureC;
  float humidityPct;
  float pressureHPa;
  float airQualityRaw;
  float combustibleGasRaw;
  float heartRateBpm;
  float spo2Pct;
  float soilMoisturePct;
  float anomalyScore;
  uint32_t seq;
} WorkerPacket;

WebServer server(80);

// Diagnostics for the ESP-NOW link specifically -- see onDataReceive()
// and the "still waiting" heartbeat in loop(). If packetsReceivedTotal
// stays at 0 forever, the packet isn't arriving at all (check
// WORKER_MAC in worker_n.ino, WIFI_CHANNEL match, and that both boards
// are actually powered).
uint32_t packetsReceivedTotal = 0;
unsigned long lastPacketReceivedMs = 0;

// ---------------------------------------------------------------------
// LOCAL SENSOR OBJECTS + LATEST READINGS
// ---------------------------------------------------------------------
Adafruit_MPU6050 mpu;
bool mpuFound = false;
Adafruit_BMP085 bmp;
bool bmpFound = false;
DHT dht(DHT_PIN, DHT_TYPE);

float localAccelX = 0, localAccelY = 0, localAccelZ = 0;
float localGyroX = 0, localGyroY = 0, localGyroZ = 0;
float localTemperatureC = NAN;      // from DHT11
float localHumidityPct = NAN;       // from DHT11
float localBmpTemperatureC = NAN;   // from BMP180 (separate sensor, kept distinct)
float localPressureHPa = NAN;       // from BMP180
float localDistanceCm = -1;         // from HC-SR04, -1 = no echo / out of range
unsigned long lastLocalSensorAtMs = 0;
unsigned long lastLocalSensorReadMs = 0; // millis() timestamp of the last successful read, for ageMs

// ---------------------------------------------------------------------
// VIBRATION DETECTION -- this wall node's own job for its MPU6050
// (worker_n.ino's MPU6050 does FALL detection instead; see that
// file). Sampled much faster than the 1s LOCAL_SENSOR_INTERVAL_MS
// above, same reasoning as worker_n.ino's fast impact-peak scan: a
// real shake/knock shows up as a brief spike that a 1Hz read could
// step right over.
// ---------------------------------------------------------------------
const unsigned long VIBRATION_SAMPLE_INTERVAL_MS = 20; // ~50 Hz
// Starting value only -- tune against your actual mount/surface, same
// caveat as every other hand-picked threshold in this project. A
// wall-mounted sensor picking up hand-shaking or nearby machinery
// will see much smaller accel deltas than worker_n.ino's wearable
// fall detector, hence the much lower threshold than that file's
// LOW_MOTION_THRESHOLD (4.0).
const float VIBRATION_THRESHOLD = 1.5;
const unsigned long VIBRATION_REPEAT_PRINT_MS = 300; // while confirmed vibrating, print at most this often

// Require the energy to stay above VIBRATION_THRESHOLD continuously
// for this long before calling it "vibrating" -- a single brief jolt
// (someone bumping the wall once, a door closing) should not print
// VIBRATING; sustained shaking/machinery for a couple of seconds
// should.
const unsigned long VIBRATION_CONFIRM_MS = 2000;

float vibPrevAccelX = 0, vibPrevAccelY = 0, vibPrevAccelZ = 0;
float vibrationEnergy = 0.0;
bool isVibrating = false;             // CONFIRMED (>= VIBRATION_CONFIRM_MS) vibration state
unsigned long vibrationStartMs = 0;   // 0 = not currently above threshold
unsigned long lastVibrationSampleMs = 0;
unsigned long lastVibrationPrintMs = 0;

// ---------------------------------------------------------------------
// IN-MEMORY STORE -- same contract as wall_node.ino: no wall-clock
// (no internet -> no NTP), every entry keeps a millis() timestamp and
// every API response computes ageMs fresh at request time.
// ---------------------------------------------------------------------
struct StoredWorker {
  bool used;
  char workerId[8];
  char nodeId[8];
  char msgType[10];
  uint32_t seq;
  bool haveRssi;
  int rssi;
  char riskState[16];
  char hazardType[16];
  float motionEnergy;
  float secondsSinceMotion;
  float motionEnergyAtTrigger;
  float temperatureC;
  float humidityPct;
  float pressureHPa;
  float airQualityRaw;
  float combustibleGasRaw;
  float heartRateBpm;
  float spo2Pct;
  float soilMoisturePct;
  float anomalyScore;
  unsigned long receivedAtMs;
};
const int MAX_WORKERS = 8;
StoredWorker workersStore[MAX_WORKERS];

struct StoredEvent {
  char workerId[8];
  char nodeId[8];
  char msgType[10];
  char riskState[16];
  char hazardType[16];
  unsigned long receivedAtMs;
};
const int MAX_EVENTS = 20;
StoredEvent eventsStore[MAX_EVENTS];
int eventWriteIdx = 0;
int eventsFilled = 0;

int findOrAllocWorker(const char* workerId) {
  for (int i = 0; i < MAX_WORKERS; i++) {
    if (workersStore[i].used && strcmp(workersStore[i].workerId, workerId) == 0) return i;
  }
  for (int i = 0; i < MAX_WORKERS; i++) {
    if (!workersStore[i].used) return i;
  }
  return 0;
}

void pushEvent(const char* workerId, const char* nodeId, const char* msgType,
               const char* riskState, const char* hazardType) {
  StoredEvent& e = eventsStore[eventWriteIdx];
  strncpy(e.workerId, workerId, sizeof(e.workerId) - 1); e.workerId[sizeof(e.workerId) - 1] = '\0';
  strncpy(e.nodeId, nodeId, sizeof(e.nodeId) - 1); e.nodeId[sizeof(e.nodeId) - 1] = '\0';
  strncpy(e.msgType, msgType, sizeof(e.msgType) - 1); e.msgType[sizeof(e.msgType) - 1] = '\0';
  strncpy(e.riskState, riskState, sizeof(e.riskState) - 1); e.riskState[sizeof(e.riskState) - 1] = '\0';
  strncpy(e.hazardType, hazardType, sizeof(e.hazardType) - 1); e.hazardType[sizeof(e.hazardType) - 1] = '\0';
  e.receivedAtMs = millis();
  eventWriteIdx = (eventWriteIdx + 1) % MAX_EVENTS;
  if (eventsFilled < MAX_EVENTS) eventsFilled++;
}

void recordMessage(const WorkerPacket& pkt, bool haveRssi, int rssi) {
  int idx = findOrAllocWorker(pkt.workerId);
  StoredWorker& w = workersStore[idx];
  w.used = true;
  strncpy(w.workerId, pkt.workerId, sizeof(w.workerId) - 1); w.workerId[sizeof(w.workerId) - 1] = '\0';
  strncpy(w.nodeId, NODE_ID, sizeof(w.nodeId) - 1); w.nodeId[sizeof(w.nodeId) - 1] = '\0';
  const char* msgTypeStr = pkt.msgType == 1 ? "DISTRESS" : "STATUS";
  strncpy(w.msgType, msgTypeStr, sizeof(w.msgType) - 1); w.msgType[sizeof(w.msgType) - 1] = '\0';
  w.seq = pkt.seq;
  w.haveRssi = haveRssi;
  w.rssi = rssi;
  strncpy(w.riskState, pkt.riskState, sizeof(w.riskState) - 1); w.riskState[sizeof(w.riskState) - 1] = '\0';
  strncpy(w.hazardType, pkt.hazardType, sizeof(w.hazardType) - 1); w.hazardType[sizeof(w.hazardType) - 1] = '\0';
  w.motionEnergy = pkt.motionEnergy;
  w.secondsSinceMotion = pkt.secondsSinceMotion;
  w.motionEnergyAtTrigger = pkt.motionEnergyAtTrigger;
  w.temperatureC = pkt.temperatureC;
  w.humidityPct = pkt.humidityPct;
  w.pressureHPa = pkt.pressureHPa;
  w.airQualityRaw = pkt.airQualityRaw;
  w.combustibleGasRaw = pkt.combustibleGasRaw;
  w.heartRateBpm = pkt.heartRateBpm;
  w.spo2Pct = pkt.spo2Pct;
  w.soilMoisturePct = pkt.soilMoisturePct;
  w.anomalyScore = pkt.anomalyScore;
  w.receivedAtMs = millis();

  pushEvent(pkt.workerId, NODE_ID, msgTypeStr, pkt.riskState, pkt.hazardType);
}

// ---------------------------------------------------------------------
// LOCAL SENSOR READING
// ---------------------------------------------------------------------
long readUltrasonicDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long durationUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (durationUs == 0) return -1; // no echo within timeout -- out of range or nothing reflecting

  // Speed of sound ~343 m/s -> ~29.1 us per cm round trip.
  return durationUs / 58;
}

// ---------------------------------------------------------------------
// VIBRATION SCAN -- called every loop() iteration, internally gated to
// VIBRATION_SAMPLE_INTERVAL_MS. Independent of readLocalSensors()'s
// 1s cycle so a brief shake/knock isn't missed between reads.
// ---------------------------------------------------------------------
void sampleVibration() {
  if (!mpuFound) return;
  unsigned long now = millis();
  if (now - lastVibrationSampleMs < VIBRATION_SAMPLE_INTERVAL_MS) return;
  lastVibrationSampleMs = now;

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  float dx = ax - vibPrevAccelX;
  float dy = ay - vibPrevAccelY;
  float dz = az - vibPrevAccelZ;
  float change = sqrt(dx * dx + dy * dy + dz * dz);
  vibPrevAccelX = ax;
  vibPrevAccelY = ay;
  vibPrevAccelZ = az;

  // Same smoothing approach as worker_n.ino's motionEnergy -- an EMA
  // instead of a raw instantaneous delta, so a single noisy sample
  // can't flip the vibrating/not-vibrating state on its own.
  vibrationEnergy = (vibrationEnergy * 0.8) + (change * 0.2);

  bool aboveThreshold = vibrationEnergy > VIBRATION_THRESHOLD;

  if (aboveThreshold) {
    if (vibrationStartMs == 0) vibrationStartMs = now; // just crossed the threshold

    if (!isVibrating && now - vibrationStartMs >= VIBRATION_CONFIRM_MS) {
      // Sustained for VIBRATION_CONFIRM_MS -- confirmed, not just a
      // brief bump/knock.
      isVibrating = true;
      Serial.println();
      Serial.println(">>> VIBRATING");
      lastVibrationPrintMs = now;
    } else if (isVibrating && now - lastVibrationPrintMs >= VIBRATION_REPEAT_PRINT_MS) {
      Serial.println("VIBRATING");
      lastVibrationPrintMs = now;
    }
  } else {
    if (isVibrating) Serial.println(">>> vibration stopped");
    // Below threshold again -- reset, whether or not it was ever
    // confirmed. A brief sub-2s blip that never got announced is
    // silently discarded here, on purpose.
    vibrationStartMs = 0;
    isVibrating = false;
  }
}

void readLocalSensors() {
  if (mpuFound) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    localAccelX = accel.acceleration.x;
    localAccelY = accel.acceleration.y;
    localAccelZ = accel.acceleration.z;
    localGyroX = gyro.gyro.x;
    localGyroY = gyro.gyro.y;
    localGyroZ = gyro.gyro.z;
  }

  if (bmpFound) {
    localBmpTemperatureC = bmp.readTemperature();
    localPressureHPa = bmp.readPressure() / 100.0;
  }

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h) && !isnan(t)) {
    localHumidityPct = h;
    localTemperatureC = t;
  }

  localDistanceCm = readUltrasonicDistanceCm();
  lastLocalSensorReadMs = millis();

  Serial.println();
  Serial.println("---- [WALL-LOCAL] sensor reading ----");
  // MPU6050 -- wall_n.ino's job for this sensor is VIBRATION sensing
  // (e.g. structural shaking, drilling, equipment strain near this
  // wall mount), not fall detection -- that's worker_n.ino's job for
  // its own, separate MPU6050. Full 6-axis reading for visibility.
  Serial.print("[MPU6050 -- vibration detection] Accel(m/s^2) X="); Serial.print(localAccelX, 2);
  Serial.print(" Y="); Serial.print(localAccelY, 2);
  Serial.print(" Z="); Serial.print(localAccelZ, 2);
  Serial.print(" | Gyro(rad/s) X="); Serial.print(localGyroX, 2);
  Serial.print(" Y="); Serial.print(localGyroY, 2);
  Serial.print(" Z="); Serial.println(localGyroZ, 2);
  Serial.print("BMP180: T="); Serial.print(localBmpTemperatureC, 1);
  Serial.print("C P="); Serial.print(localPressureHPa, 1); Serial.println("hPa");
  Serial.print("DHT11: T="); Serial.print(localTemperatureC, 1);
  Serial.print("C H="); Serial.print(localHumidityPct, 1); Serial.println("%");
  Serial.print("Ultrasonic distance: ");
  if (localDistanceCm < 0) Serial.println("out of range / no echo");
  else { Serial.print(localDistanceCm); Serial.println(" cm"); }
  Serial.println("--------------------------------------");
}

// ---------------------------------------------------------------------
// EMBEDDED DASHBOARD -- served directly from flash, same ageMs-based
// API contract as wall_node.ino / index.html, extended with a
// "Local Node Sensors" panel and a live ML threshold table so the
// tinyML-derived numbers are visible without opening the Serial
// Monitor.
// ---------------------------------------------------------------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Bleep - Wall Node 2</title>
<style>
  * { box-sizing: border-box; }
  :root {
    --bg:#f5f7f5; --surface:#ffffff; --border:#e3e6e2;
    --text:#14181f; --text-muted:#6b7280;
    --accent:#0f766e; --accent-soft:#e3f2ef;
    --good:#0ca30c; --critical:#d03b3b; --offline:#8b93a0; --waiting:#b45309;
  }
  body { margin:0; font-family: system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; color:var(--text); background:var(--bg); }
  .mono { font-family: ui-monospace, "SFMono-Regular", Menlo, Consolas, monospace; }
  .topbar { display:flex; align-items:center; justify-content:space-between; padding:16px 20px; background:var(--surface); border-bottom:1px solid var(--border); flex-wrap:wrap; gap:8px; }
  .content { padding:20px; max-width:900px; margin:0 auto; display:flex; flex-direction:column; gap:20px; }
  .status-flag { display:flex; align-items:center; gap:6px; border-radius:4px; padding:3px 9px; font-size:11px; font-weight:600; letter-spacing:0.03em; }
  .status-flag.live { background:#e3f2ef; border:1px solid #bfe3de; color:var(--accent); }
  .status-flag.waiting { background:#fbf1e0; border:1px solid #e8d3a3; color:var(--waiting); }
  .status-flag.down { background:#f1e4e4; border:1px solid #e0bcbc; color:var(--critical); }
  .card { background:var(--surface); border:1px solid var(--border); border-radius:10px; }
  .card-head { padding:12px 16px; border-bottom:1px solid var(--border); font-weight:700; font-size:14px; }
  .empty-state { padding:20px 16px; text-align:center; font-size:13px; color:var(--text-muted); }
  .stat-card { background:var(--surface); border:1px solid var(--border); border-radius:10px; padding:14px 16px; }
  .stat-label { font-size:11px; color:var(--text-muted); margin-bottom:8px; }
  .stat-value { font-size:22px; font-weight:700; }
  .grid-4 { display:grid; grid-template-columns:repeat(4,minmax(0,1fr)); gap:12px; }
  .grid-2 { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:12px; }
  .worker-card { border:1px solid #e2e6ea; border-radius:10px; padding:14px; background:var(--surface); }
  .worker-head { display:flex; justify-content:space-between; align-items:center; margin-bottom:10px; }
  .status-pill { font-size:10px; font-weight:700; letter-spacing:0.04em; color:#fff; border-radius:4px; padding:3px 8px; }
  .worker-metrics { display:grid; grid-template-columns:1fr 1fr; gap:8px 12px; margin-bottom:10px; font-size:13px; }
  .metric-label { font-size:10px; color:var(--text-muted); }
  .progress-track { height:6px; background:#eceeec; border-radius:3px; overflow:hidden; }
  .progress-fill { height:100%; border-radius:3px; }
  .event-row { display:flex; gap:8px; padding:7px 0; font-size:12.5px; align-items:baseline; }
  .event-dot { width:6px; height:6px; border-radius:50%; margin-top:3px; flex-shrink:0; }
  .event-time { color:var(--text-muted); flex-shrink:0; }
  .alert-banner { display:none; align-items:center; gap:10px; background:#fbe9e9; border:1px solid #edb3b3; border-left:4px solid var(--critical); border-radius:6px; padding:12px 16px; font-weight:700; color:#7a1f1f; }
  .alert-banner.visible { display:flex; }
  table.thresh { width:100%; border-collapse:collapse; font-size:12.5px; }
  table.thresh th, table.thresh td { text-align:left; padding:6px 10px; border-bottom:1px solid var(--border); }
  @media (max-width:640px) { .grid-4, .grid-2 { grid-template-columns:repeat(2,minmax(0,1fr)); } }
</style>
</head>
<body>
<div class="topbar">
  <strong>Bleep &middot; Wall Node 2 (WALL2)</strong>
  <div class="status-flag waiting" id="connection-flag"><span id="connection-flag-text">CONNECTING...</span></div>
</div>
<div class="content">
  <div class="grid-4">
    <div class="stat-card"><div class="stat-label">WALL NODES ONLINE</div><div class="mono stat-value" id="stat-nodes">-</div></div>
    <div class="stat-card"><div class="stat-label">WORKERS REPORTING</div><div class="mono stat-value" id="stat-workers">-</div></div>
    <div class="stat-card"><div class="stat-label">ACTIVE ALERTS</div><div class="mono stat-value" id="stat-alerts">-</div></div>
    <div class="stat-card"><div class="stat-label">LAST UPDATE</div><div class="mono stat-value" id="stat-last-update" style="font-size:15px;">-</div></div>
  </div>
  <div class="alert-banner" id="alert-banner"><span id="alert-text"></span></div>
  <div class="card">
    <div class="card-head">Wall Node 2 Local Sensors</div>
    <div class="worker-metrics" style="padding:14px 16px;" id="local-metrics">
      <div><div class="metric-label">Ambient Temp (DHT11)</div><div class="mono" id="local-temp">-</div></div>
      <div><div class="metric-label">Humidity</div><div class="mono" id="local-humidity">-</div></div>
      <div><div class="metric-label">Pressure (BMP180)</div><div class="mono" id="local-pressure">-</div></div>
      <div><div class="metric-label">Distance (HC-SR04)</div><div class="mono" id="local-distance">-</div></div>
      <div><div class="metric-label">Accel (MPU6050)</div><div class="mono" id="local-accel">-</div></div>
      <div><div class="metric-label">Reading Age</div><div class="mono" id="local-age">-</div></div>
    </div>
  </div>
  <div>
    <h3 style="margin:0 0 10px; font-size:14px;">Worker Roster</h3>
    <div class="grid-2" id="roster-grid"><div class="card" style="grid-column:1/-1;"><div class="empty-state">Waiting for a worker message...</div></div></div>
  </div>
  <div class="card">
    <div class="card-head">tinyML Threshold Bands (learned, see ml/train_tinyml_model.py)</div>
    <div style="overflow-x:auto;"><table class="thresh" id="thresh-table"><thead><tr><th>Sensor</th><th>Safe low</th><th>Safe high</th></tr></thead><tbody></tbody></table></div>
  </div>
  <div class="card">
    <div class="card-head">Event Log</div>
    <div style="max-height:300px; overflow-y:auto;" id="event-log-wrap">
      <div class="empty-state" id="event-log-empty">No events yet.</div>
      <div id="event-log-list" style="padding:0 16px;"></div>
    </div>
  </div>
</div>
<script>
(function(){
  var POLL_MS=2000, STALE_MS=15000;
  function fmtAge(ms){ if(ms==null) return '-'; if(ms<1000) return 'just now'; var s=Math.round(ms/1000); if(s<60) return s+'s ago'; return Math.round(s/60)+'m ago'; }
  function statusFromMsg(msg){ if(msg.ageMs>STALE_MS) return 'OFFLINE'; if(msg.msgType==='DISTRESS'){ return (msg.payload&&msg.payload.riskState)||'ALERT'; } return 'OK'; }
  function isAlertStatus(status){ return status!=='OK' && status!=='OFFLINE'; }
  function setFlag(state,text){ var f=document.getElementById('connection-flag'); f.className='status-flag '+state; document.getElementById('connection-flag-text').textContent=text; }

  function renderRoster(workersObj){
    var ids=Object.keys(workersObj).sort();
    var grid=document.getElementById('roster-grid');
    if(ids.length===0){ grid.innerHTML='<div class="card" style="grid-column:1/-1;"><div class="empty-state">Waiting for a worker message...</div></div>'; return {alertNames:[]}; }
    grid.innerHTML=''; var alertNames=[];
    ids.forEach(function(id){
      var msg=workersObj[id]; var status=statusFromMsg(msg); var alerting=isAlertStatus(status);
      if(alerting) alertNames.push(id+' ('+status.replace(/_/g,' ')+')');
      var pillColor= alerting ? 'var(--critical)' : (status==='OFFLINE' ? 'var(--offline)' : 'var(--good)');
      var payload=msg.payload||{};
      var motion= payload.motionEnergy!=null ? payload.motionEnergy.toFixed(2)+' g' : '-';
      var rssi= msg.espnowRssi!=null ? msg.espnowRssi+' dBm' : '-';
      var seq= msg.seq!=null ? msg.seq : '-';
      var temp= payload.temperatureC!=null ? payload.temperatureC.toFixed(1)+'C' : '-';
      var gas= payload.airQualityRaw!=null ? payload.airQualityRaw.toFixed(0)+' / '+ (payload.combustibleGasRaw!=null?payload.combustibleGasRaw.toFixed(0):'-') : '-';
      var vitals= payload.heartRateBpm!=null && payload.heartRateBpm>0 ? payload.heartRateBpm.toFixed(0)+'bpm / '+(payload.spo2Pct!=null?payload.spo2Pct.toFixed(0):'-')+'%' : '-';
      var soil= payload.soilMoisturePct!=null ? payload.soilMoisturePct.toFixed(0)+'%' : '-';
      var anomaly= payload.anomalyScore!=null ? payload.anomalyScore.toFixed(4) : '-';
      var card=document.createElement('div');
      card.className='worker-card'; card.style.opacity= status==='OFFLINE'?'0.7':'1';
      card.innerHTML=
        '<div class="worker-head"><strong>'+id+'</strong><span class="status-pill" style="background:'+pillColor+';">'+status.replace(/_/g,' ')+'</span></div>'+
        '<div class="worker-metrics">'+
          '<div><div class="metric-label">Motion Energy</div><div class="mono">'+motion+'</div></div>'+
          '<div><div class="metric-label">RSSI</div><div class="mono">'+rssi+'</div></div>'+
          '<div><div class="metric-label">Temp</div><div class="mono">'+temp+'</div></div>'+
          '<div><div class="metric-label">Gas (MQ135/MQ4)</div><div class="mono">'+gas+'</div></div>'+
          '<div><div class="metric-label">HR / SpO2</div><div class="mono">'+vitals+'</div></div>'+
          '<div><div class="metric-label">Soil Moisture</div><div class="mono">'+soil+'</div></div>'+
          '<div><div class="metric-label">ML Anomaly Score</div><div class="mono">'+anomaly+'</div></div>'+
          '<div><div class="metric-label">Last Seq</div><div class="mono">'+seq+'</div></div>'+
        '</div>';
      grid.appendChild(card);
    });
    return {alertNames:alertNames};
  }

  function renderEvents(list){
    var logEl=document.getElementById('event-log-list'); var emptyEl=document.getElementById('event-log-empty');
    if(!list||list.length===0){ emptyEl.style.display=''; logEl.innerHTML=''; return; }
    emptyEl.style.display='none'; logEl.innerHTML='';
    list.forEach(function(msg){
      var dotColor= msg.msgType==='DISTRESS' ? 'var(--critical)' : 'var(--good)';
      var payload=msg.payload||{}; var detail=payload.riskState||payload.hazardType||'';
      var row=document.createElement('div'); row.className='event-row';
      row.innerHTML='<span class="event-dot" style="background:'+dotColor+';"></span><span class="mono event-time">'+fmtAge(msg.ageMs)+'</span><span>'+msg.workerId+' ('+msg.nodeId+') &middot; '+msg.msgType+' &middot; '+detail+'</span>';
      logEl.appendChild(row);
    });
  }

  function renderLocal(local){
    document.getElementById('local-temp').textContent = local.temperatureC!=null ? local.temperatureC.toFixed(1)+' C' : '-';
    document.getElementById('local-humidity').textContent = local.humidityPct!=null ? local.humidityPct.toFixed(1)+' %' : '-';
    document.getElementById('local-pressure').textContent = local.pressureHPa!=null ? local.pressureHPa.toFixed(1)+' hPa' : '-';
    document.getElementById('local-distance').textContent = (local.distanceCm!=null && local.distanceCm>=0) ? local.distanceCm.toFixed(0)+' cm' : 'out of range';
    document.getElementById('local-accel').textContent = 'X='+local.accelX.toFixed(2)+' Y='+local.accelY.toFixed(2)+' Z='+local.accelZ.toFixed(2);
    document.getElementById('local-age').textContent = fmtAge(local.ageMs);
  }

  function renderThresholds(list){
    var tbody=document.querySelector('#thresh-table tbody');
    tbody.innerHTML='';
    list.forEach(function(row){
      var tr=document.createElement('tr');
      tr.innerHTML='<td>'+row.name+'</td><td class="mono">'+row.low.toFixed(2)+'</td><td class="mono">'+row.high.toFixed(2)+'</td>';
      tbody.appendChild(tr);
    });
  }

  var thresholdsLoaded=false;
  function poll(){
    var calls=[
      fetch('/api/workers').then(function(r){return r.ok?r.json():Promise.reject();}),
      fetch('/api/events?limit=30').then(function(r){return r.ok?r.json():Promise.reject();}),
      fetch('/api/nodes').then(function(r){return r.ok?r.json():Promise.reject();}),
      fetch('/api/local').then(function(r){return r.ok?r.json():Promise.reject();})
    ];
    if(!thresholdsLoaded){ calls.push(fetch('/api/thresholds').then(function(r){return r.ok?r.json():Promise.reject();})); }

    Promise.all(calls).then(function(results){
      var allWorkers=results[0], eventsList=results[1], allNodes=results[2], local=results[3];
      var summary=renderRoster(allWorkers);
      renderEvents(eventsList);
      renderLocal(local);
      if(!thresholdsLoaded && results[4]){ renderThresholds(results[4]); thresholdsLoaded=true; }

      var nodeIds=Object.keys(allNodes);
      var onlineNodeCount= nodeIds.filter(function(id){ return allNodes[id].ageMs<=STALE_MS; }).length;
      document.getElementById('stat-nodes').textContent=String(onlineNodeCount);

      var workerCount=Object.keys(allWorkers).length;
      document.getElementById('stat-workers').textContent=String(workerCount);
      document.getElementById('stat-alerts').textContent=String(summary.alertNames.length);

      var banner=document.getElementById('alert-banner');
      banner.classList.toggle('visible', summary.alertNames.length>0);
      if(summary.alertNames.length>0){
        document.getElementById('alert-text').textContent='ALERT - '+summary.alertNames.join(', ')+' - immediate response required';
      }

      var newestAgeMs=null;
      Object.keys(allWorkers).forEach(function(id){ var a=allWorkers[id].ageMs; if(newestAgeMs==null||a<newestAgeMs) newestAgeMs=a; });
      document.getElementById('stat-last-update').textContent=fmtAge(newestAgeMs);

      if(onlineNodeCount===0){ setFlag('down','WALL NODE OFFLINE'); }
      else if(workerCount===0){ setFlag('waiting','NODE OK - NO WORKER DATA'); }
      else if(newestAgeMs!=null && newestAgeMs>STALE_MS){ setFlag('waiting','NODE OK - STALE WORKER DATA'); }
      else { setFlag('live','LIVE'); }
    }).catch(function(){
      setFlag('down','CONNECTION LOST');
    });
  }
  poll(); setInterval(poll, POLL_MS);
})();
</script>
</body>
</html>
)rawliteral";

// ---------------------------------------------------------------------
// HTTP HANDLERS
// ---------------------------------------------------------------------
void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleApiWorkers() {
  DynamicJsonDocument doc(5120);
  JsonObject root = doc.to<JsonObject>();
  unsigned long now = millis();
  for (int i = 0; i < MAX_WORKERS; i++) {
    if (!workersStore[i].used) continue;
    StoredWorker& w = workersStore[i];
    JsonObject o = root.createNestedObject(w.workerId);
    o["workerId"] = w.workerId;
    o["nodeId"] = w.nodeId;
    o["msgType"] = w.msgType;
    o["seq"] = w.seq;
    if (w.haveRssi) o["espnowRssi"] = w.rssi; else o["espnowRssi"] = nullptr;
    o["ageMs"] = now - w.receivedAtMs;
    JsonObject payload = o.createNestedObject("payload");
    payload["riskState"] = w.riskState;
    payload["hazardType"] = w.hazardType;
    payload["motionEnergy"] = w.motionEnergy;
    payload["secondsSinceMotion"] = w.secondsSinceMotion;
    payload["secondsInactive"] = w.secondsSinceMotion;
    payload["motionEnergyAtTrigger"] = w.motionEnergyAtTrigger;
    payload["temperatureC"] = w.temperatureC;
    payload["humidityPct"] = w.humidityPct;
    payload["pressureHPa"] = w.pressureHPa;
    payload["airQualityRaw"] = w.airQualityRaw;
    payload["combustibleGasRaw"] = w.combustibleGasRaw;
    payload["heartRateBpm"] = w.heartRateBpm;
    payload["spo2Pct"] = w.spo2Pct;
    payload["soilMoisturePct"] = w.soilMoisturePct;
    payload["anomalyScore"] = w.anomalyScore;
    payload["battery"] = nullptr;
  }
  String out;
  serializeJson(doc, out);
  // CORS: lets a dashboard page running from a DIFFERENT origin (a
  // locally-saved HTML file, or any laptop on this SoftAP's network)
  // fetch() this API. Without this header the browser silently
  // blocks the response even though the ESP32 served it -- see
  // dashboard_features.md for the full wiring walkthrough.
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", out);
}

void handleApiEvents() {
  int limit = 30;
  if (server.hasArg("limit")) limit = server.arg("limit").toInt();

  DynamicJsonDocument doc(4096);
  JsonArray arr = doc.to<JsonArray>();
  unsigned long now = millis();
  int n = eventsFilled < limit ? eventsFilled : limit;
  for (int k = 0; k < n; k++) {
    int idx = (eventWriteIdx - 1 - k + MAX_EVENTS * 2) % MAX_EVENTS;
    StoredEvent& e = eventsStore[idx];
    JsonObject o = arr.createNestedObject();
    o["workerId"] = e.workerId;
    o["nodeId"] = e.nodeId;
    o["msgType"] = e.msgType;
    o["ageMs"] = now - e.receivedAtMs;
    JsonObject payload = o.createNestedObject("payload");
    payload["riskState"] = e.riskState;
    payload["hazardType"] = e.hazardType;
  }
  String out;
  serializeJson(doc, out);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", out);
}

void handleApiNodes() {
  DynamicJsonDocument doc(128);
  JsonObject root = doc.to<JsonObject>();
  JsonObject o = root.createNestedObject(NODE_ID);
  o["nodeId"] = NODE_ID;
  o["ageMs"] = 0;
  String out;
  serializeJson(doc, out);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", out);
}

// This wall node's own directly-wired sensors -- same ageMs contract
// as everything else, so dashboard code doesn't need a special case
// for "sensor data that didn't come over ESP-NOW."
void handleApiLocal() {
  DynamicJsonDocument doc(512);
  JsonObject o = doc.to<JsonObject>();
  o["nodeId"] = NODE_ID;
  if (isnan(localTemperatureC)) o["temperatureC"] = nullptr; else o["temperatureC"] = localTemperatureC;
  if (isnan(localHumidityPct)) o["humidityPct"] = nullptr; else o["humidityPct"] = localHumidityPct;
  if (isnan(localPressureHPa)) o["pressureHPa"] = nullptr; else o["pressureHPa"] = localPressureHPa;
  if (isnan(localBmpTemperatureC)) o["bmpTemperatureC"] = nullptr; else o["bmpTemperatureC"] = localBmpTemperatureC;
  o["distanceCm"] = localDistanceCm;
  o["accelX"] = localAccelX;
  o["accelY"] = localAccelY;
  o["accelZ"] = localAccelZ;
  o["gyroX"] = localGyroX;
  o["gyroY"] = localGyroY;
  o["gyroZ"] = localGyroZ;
  o["vibrating"] = isVibrating;
  o["vibrationEnergy"] = vibrationEnergy;
  if (lastLocalSensorReadMs == 0) o["ageMs"] = nullptr;
  else o["ageMs"] = millis() - lastLocalSensorReadMs;
  String out;
  serializeJson(doc, out);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", out);
}

// The tinyML-derived per-sensor threshold bands, straight out of
// tinyml_model.h -- lets the dashboard render "safe range" bars
// without hand-copying numbers out of firmware source.
void handleApiThresholds() {
  // No pressureHPa here -- worker_n.ino has no BMP180, so that
  // feature was dropped from the trained model (see
  // ml/train_tinyml_model.py). This wall node's own local BMP180
  // (see handleApiLocal()) is a separate, unrelated sensor.
  static const char* FEATURE_NAMES[TINYML_N_FEATURES] = {
    "temperatureC", "humidityPct", "airQualityRaw",
    "combustibleGasRaw", "heartRateBpm", "spo2Pct", "soilMoisturePct", "motionEnergy"
  };
  DynamicJsonDocument doc(1024);
  JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < TINYML_N_FEATURES; i++) {
    JsonObject o = arr.createNestedObject();
    o["name"] = FEATURE_NAMES[i];
    o["low"] = TINYML_FEATURE_LOW_THRESHOLD[i];
    o["high"] = TINYML_FEATURE_HIGH_THRESHOLD[i];
  }
  String out;
  serializeJson(doc, out);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", out);
}

void onDataReceive(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len) {
  packetsReceivedTotal++;
  lastPacketReceivedMs = millis();

  Serial.println();
  Serial.println("##########################################");
  Serial.print("  [WORKER] ESP-NOW PACKET RECEIVED  (#"); Serial.print(packetsReceivedTotal); Serial.println(")");
  Serial.println("##########################################");

  if (len != sizeof(WorkerPacket)) {
    Serial.print("ERROR: Invalid packet size = ");
    Serial.print(len);
    Serial.print(" bytes | Expected = ");
    Serial.print(sizeof(WorkerPacket));
    Serial.println(" bytes");
    Serial.println("==========================================");
    return;
  }

  WorkerPacket pkt;
  memcpy(&pkt, incomingData, sizeof(WorkerPacket));
  pkt.workerId[sizeof(pkt.workerId) - 1] = '\0';
  pkt.riskState[sizeof(pkt.riskState) - 1] = '\0';
  pkt.hazardType[sizeof(pkt.hazardType) - 1] = '\0';

  bool haveRssi = false;
  int rssi = 0;
  if (info != nullptr && info->rx_ctrl != nullptr) {
    rssi = info->rx_ctrl->rssi;
    haveRssi = true;
  }

  Serial.print("Node ID: "); Serial.println(NODE_ID);
  Serial.print("Worker ID: "); Serial.println(pkt.workerId);
  Serial.print("Message Type: "); Serial.println(pkt.msgType == 0 ? "STATUS (0)" : (pkt.msgType == 1 ? "DISTRESS (1)" : "UNKNOWN"));
  Serial.print("Risk State: "); Serial.println(pkt.riskState);
  Serial.print("Hazard Type: "); Serial.println(pkt.hazardType);
  Serial.print("Motion Energy: "); Serial.println(pkt.motionEnergy, 2);
  Serial.print("Seconds Since Motion: "); Serial.println(pkt.secondsSinceMotion, 2);
  Serial.print("Temperature: "); Serial.println(pkt.temperatureC, 1);
  Serial.print("Air Quality (MQ135): "); Serial.println(pkt.airQualityRaw, 0);
  Serial.print("Combustible Gas (MQ4): "); Serial.println(pkt.combustibleGasRaw, 0);
  Serial.print("Heart Rate: "); Serial.println(pkt.heartRateBpm, 0);
  Serial.print("SpO2: "); Serial.println(pkt.spo2Pct, 0);
  Serial.print("Soil Moisture: "); Serial.println(pkt.soilMoisturePct, 0);
  Serial.print("ML Anomaly Score: "); Serial.print(pkt.anomalyScore, 5);
  Serial.print(" (threshold "); Serial.print(TINYML_ANOMALY_THRESHOLD, 5); Serial.println(")");
  Serial.print("Sequence Number: "); Serial.println(pkt.seq);
  Serial.print("ESP-NOW RSSI: ");
  if (haveRssi) { Serial.print(rssi); Serial.println(" dBm"); } else { Serial.println("N/A"); }

  if (pkt.msgType == 1) {
    Serial.println();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("             !!! DISTRESS !!!");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.print("Worker: "); Serial.println(pkt.workerId);
    Serial.print("Hazard: "); Serial.println(pkt.hazardType);
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  Serial.println("==========================================");

  recordMessage(pkt, haveRssi, rssi);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // ------------------------------------------------------------
  // LOCAL SENSORS
  // ------------------------------------------------------------
  Wire.begin(SDA_PIN, SCL_PIN);

  mpuFound = mpu.begin();
  Serial.println(mpuFound ? "MPU6050 FOUND (local)" : "[WARN] MPU6050 NOT FOUND -- local accel disabled");
  if (mpuFound) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Prime the vibration baseline with a real reading (gravity is
    // ~9.8 on whichever axis is "down") -- starting from 0,0,0 would
    // make the very first sampleVibration() call see a huge fake
    // "change" and falsely report VIBRATING once at boot.
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    vibPrevAccelX = accel.acceleration.x;
    vibPrevAccelY = accel.acceleration.y;
    vibPrevAccelZ = accel.acceleration.z;
  }

  bmpFound = bmp.begin();
  Serial.println(bmpFound ? "BMP180 FOUND (local)" : "[WARN] BMP180 NOT FOUND -- local pressure disabled");

  dht.begin();
  Serial.println("DHT11 initialized (local)");

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  Serial.println("HC-SR04 initialized (local)");

  // ------------------------------------------------------------
  // RADIO + DASHBOARD -- same peer-to-peer setup as wall_node.ino
  // ------------------------------------------------------------
  WiFi.mode(WIFI_AP_STA);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL);
  Serial.println(apOk ? "SoftAP started." : "[ERROR] SoftAP failed to start.");
  Serial.print("Network name: "); Serial.println(AP_SSID);
  Serial.print("Connect your laptop to it, then browse to: http://"); Serial.println(WiFi.softAPIP());

  // worker_n.ino needs ONE of these two as its WALL_MAC constant to
  // actually be able to esp_now_send() to this board -- without a
  // correct destination MAC, sends fail silently and nothing ever
  // shows up here. Try the STA MAC first (ESP-NOW binds to the
  // WIFI_IF_STA interface by default even in AP_STA mode); if
  // worker_n.ino's Serial log still shows "SEND FAILED" after using
  // it, try the SoftAP MAC instead.
  Serial.print("Wall STA MAC   (try this first for worker_n.ino's WALL_MAC): "); Serial.println(WiFi.macAddress());
  Serial.print("Wall SoftAP MAC (try this if the STA MAC doesn't work):      "); Serial.println(WiFi.softAPmacAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] esp_now_init failed");
    return;
  }
  esp_now_register_recv_cb(onDataReceive);

  esp_now_peer_info_t workerPeer = {};
  memcpy(workerPeer.peer_addr, WORKER_MAC, 6);
  workerPeer.channel = WIFI_CHANNEL;
  workerPeer.encrypt = false;
  esp_err_t peerResult = esp_now_add_peer(&workerPeer);
  if (peerResult == ESP_OK) Serial.println("Worker peer added successfully");
  else if (peerResult == ESP_ERR_ESPNOW_EXIST) Serial.println("Worker peer already exists");
  else { Serial.print("[ERROR] Failed to add worker peer: "); Serial.println(peerResult); }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/workers", HTTP_GET, handleApiWorkers);
  server.on("/api/events", HTTP_GET, handleApiEvents);
  server.on("/api/nodes", HTTP_GET, handleApiNodes);
  server.on("/api/local", HTTP_GET, handleApiLocal);
  server.on("/api/thresholds", HTTP_GET, handleApiThresholds);
  server.begin();
  Serial.println("Web server started on port 80.");

  lastLocalSensorAtMs = millis();

  Serial.println("Wall node 2 ready.");
}

void loop() {
  server.handleClient();
  sampleVibration();

  unsigned long now = millis();
  if (now - lastLocalSensorAtMs >= LOCAL_SENSOR_INTERVAL_MS) {
    lastLocalSensorAtMs = now;
    readLocalSensors();
  }

  // Heartbeat so a quiet Serial Monitor is legible: every 5s, confirm
  // this board is alive and say plainly whether it has EVER heard from
  // a worker. If this keeps printing "0 received" forever, the packet
  // genuinely isn't arriving -- check worker_n.ino's WALL_MAC (its own
  // Serial Monitor will show a loud warning if that's still the
  // placeholder), and that WIFI_CHANNEL matches on both boards.
  static unsigned long lastHeartbeatMs = 0;
  if (now - lastHeartbeatMs >= 5000) {
    lastHeartbeatMs = now;
    if (packetsReceivedTotal == 0) {
      Serial.println("[HEARTBEAT] wall_n alive, listening for ESP-NOW -- 0 worker packets received so far");
    } else {
      Serial.print("[HEARTBEAT] wall_n alive -- ");
      Serial.print(packetsReceivedTotal);
      Serial.print(" worker packet(s) received, last one ");
      Serial.print((now - lastPacketReceivedMs) / 1000);
      Serial.println("s ago");
    }
  }
}
