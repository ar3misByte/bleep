// Sensora — Wall Node (ESP32), peer-to-peer edition
//
// No router. No mobile hotspot. No internet. This board hosts its
// OWN WiFi network (a SoftAP) so a laptop can connect to it directly,
// receives WorkerPacket telemetry from a worker ESP32 over ESP-NOW,
// stores it in memory, and serves the dashboard itself over that
// same SoftAP -- entirely peer-to-peer between three devices (worker
// ESP32 -> wall node ESP32 -> laptop browser), nothing else involved.
//
// This replaces the earlier design where the wall node joined a
// router and POSTed telemetry to a Flask server on a laptop
// (server/app.py in this repo still exists for that router-based
// path, if you ever want it back -- this sketch does not need it).
//
// Requires: ESP32 Arduino core >= 2.0.0 (for the esp_now_recv_info_t
// callback signature used below).
//
// Library Manager: ArduinoJson (v6.x)

#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ---------------------------------------------------------------------
// EDIT THESE
// ---------------------------------------------------------------------
// This is the network your laptop (or phone) connects to directly --
// there is no router in this picture at all. WPA2 requires the
// password to be at least 8 characters.
const char* AP_SSID     = "Bleep-WallNode";
const char* AP_PASSWORD = "bleepsafety";

// The radio channel this board's SoftAP runs on. worker_node.ino's
// WIFI_CHANNEL must be set to this exact same number -- ESP-NOW only
// reaches devices on the same channel, and since neither board joins
// an external router anymore, nothing assigns this automatically.
// Whoever hosts the AP (this board) is what decides the channel now,
// which is actually simpler than the old router-assigned model: pick
// a number here, put the same number in worker_node.ino, done.
const int WIFI_CHANNEL = 6;

const char* NODE_ID = "WALL1";

// The worker's wearable ESP32 — read this off that board's own
// "Worker MAC: ..." boot log. Receiving doesn't strictly require
// pairing (ESP-NOW delivers to the registered callback from anyone
// in range, paired or not), but adding it as a peer here leaves the
// door open for the wall node to send something back to the worker
// later.
const uint8_t WORKER_MAC[] = { 0x00, 0x70, 0x07, 0x26, 0xC3, 0x90 };

// Set to false once a real wearable is sending real ESP-NOW packets —
// leaving both on would feed the dashboard two "W1" sources at once.
const bool SIMULATE_LOCAL_DATA = false;
const char* SIM_WORKER_ID = "W1";
const unsigned long SIM_STATUS_INTERVAL_MS = 2000;   // matches the "every ~2s" protocol recommendation
const unsigned long SIM_FALL_INTERVAL_MS = 45000;    // simulate a fall roughly every 45s
const unsigned long SIM_RECOVERY_DELAY_MS = 15000;   // "worker gets back up" this long after a fall
// ---------------------------------------------------------------------

// Must match the wearable's struct exactly — field order, types and
// sizes are the ESP-NOW wire format.
typedef struct __attribute__((packed)) {
  char workerId[8];
  uint8_t msgType;              // 0 = STATUS, 1 = DISTRESS
  char riskState[16];
  char hazardType[16];
  float motionEnergy;
  float secondsSinceMotion;
  float motionEnergyAtTrigger;
  uint32_t seq;
} WorkerPacket;

WebServer server(80);

// ---------------------------------------------------------------------
// IN-MEMORY STORE — replaces the Flask server's role entirely. There
// is no wall-clock here (no internet means no NTP), so every stored
// entry keeps a millis() timestamp and every API response computes
// "ageMs" (elapsed time) fresh at request time. The dashboard never
// needs an absolute time, only "how long ago" — see index.html /
// server/app.py in this repo for the identical contract on the
// router-based path.
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
  float secondsSinceMotion; // doubles as "secondsInactive" for DISTRESS
  float motionEnergyAtTrigger;
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
  return 0; // full — overwrite the oldest slot rather than drop the message
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

void recordMessage(const char* workerId, const char* nodeId, uint8_t msgType, uint32_t seq,
                    bool haveRssi, int rssi, const char* riskState, const char* hazardType,
                    float motionEnergy, float secondsSinceMotion, float motionEnergyAtTrigger) {
  int idx = findOrAllocWorker(workerId);
  StoredWorker& w = workersStore[idx];
  w.used = true;
  strncpy(w.workerId, workerId, sizeof(w.workerId) - 1); w.workerId[sizeof(w.workerId) - 1] = '\0';
  strncpy(w.nodeId, nodeId, sizeof(w.nodeId) - 1); w.nodeId[sizeof(w.nodeId) - 1] = '\0';
  const char* msgTypeStr = msgType == 1 ? "DISTRESS" : "STATUS";
  strncpy(w.msgType, msgTypeStr, sizeof(w.msgType) - 1); w.msgType[sizeof(w.msgType) - 1] = '\0';
  w.seq = seq;
  w.haveRssi = haveRssi;
  w.rssi = rssi;
  strncpy(w.riskState, riskState, sizeof(w.riskState) - 1); w.riskState[sizeof(w.riskState) - 1] = '\0';
  strncpy(w.hazardType, hazardType, sizeof(w.hazardType) - 1); w.hazardType[sizeof(w.hazardType) - 1] = '\0';
  w.motionEnergy = motionEnergy;
  w.secondsSinceMotion = secondsSinceMotion;
  w.motionEnergyAtTrigger = motionEnergyAtTrigger;
  w.receivedAtMs = millis();

  pushEvent(workerId, nodeId, msgTypeStr, riskState, hazardType);
}

// ---------------------------------------------------------------------
// EMBEDDED DASHBOARD — served directly from this board's flash, no
// external CSS/font/JS dependency, because the laptop viewing it may
// have no internet at all (it's on this board's isolated network).
// Same ageMs-based API contract as index.html / server/app.py, so the
// same mental model applies whichever path you use.
// ---------------------------------------------------------------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Bleep - Wall Node</title>
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
  @media (max-width:640px) { .grid-4, .grid-2 { grid-template-columns:repeat(2,minmax(0,1fr)); } }
</style>
</head>
<body>
<div class="topbar">
  <strong>Bleep &middot; Wall Node WALL1</strong>
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
  <div>
    <h3 style="margin:0 0 10px; font-size:14px;">Worker Roster</h3>
    <div class="grid-2" id="roster-grid"><div class="card" style="grid-column:1/-1;"><div class="empty-state">Waiting for a worker message...</div></div></div>
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
      var secsInactive= payload.secondsInactive!=null?payload.secondsInactive:payload.secondsSinceMotion;
      if(secsInactive!=null && status==='OK'){ secsInactive = secsInactive + (msg.ageMs/1000); }
      var timerLabel= secsInactive!=null? (Math.round(secsInactive)+'s / 20s') : '- / 20s';
      var fillPct= secsInactive!=null? Math.min(100, Math.round((secsInactive/20)*100)) : 0;
      var fillColor= alerting ? 'var(--critical)' : 'var(--accent)';
      var card=document.createElement('div');
      card.className='worker-card'; card.style.opacity= status==='OFFLINE'?'0.7':'1';
      card.innerHTML=
        '<div class="worker-head"><strong>'+id+'</strong><span class="status-pill" style="background:'+pillColor+';">'+status.replace(/_/g,' ')+'</span></div>'+
        '<div class="worker-metrics">'+
          '<div><div class="metric-label">Motion Energy</div><div class="mono">'+motion+'</div></div>'+
          '<div><div class="metric-label">RSSI</div><div class="mono">'+rssi+'</div></div>'+
          '<div><div class="metric-label">Battery</div><div class="mono">-</div></div>'+
          '<div><div class="metric-label">Last Seq</div><div class="mono">'+seq+'</div></div>'+
        '</div>'+
        '<div><div style="display:flex; justify-content:space-between; font-size:10.5px; color:var(--text-muted); margin-bottom:3px;"><span>Inactivity Timer</span><span class="mono">'+timerLabel+'</span></div>'+
        '<div class="progress-track"><div class="progress-fill" style="width:'+fillPct+'%; background:'+fillColor+';"></div></div></div>';
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

  function poll(){
    Promise.all([
      fetch('/api/workers').then(function(r){return r.ok?r.json():Promise.reject();}),
      fetch('/api/events?limit=30').then(function(r){return r.ok?r.json():Promise.reject();}),
      fetch('/api/nodes').then(function(r){return r.ok?r.json():Promise.reject();})
    ]).then(function(results){
      var allWorkers=results[0], eventsList=results[1], allNodes=results[2];
      var summary=renderRoster(allWorkers);
      renderEvents(eventsList);

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
  DynamicJsonDocument doc(3072);
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
    payload["battery"] = nullptr;
  }
  String out;
  serializeJson(doc, out);
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
  server.send(200, "application/json", out);
}

void handleApiNodes() {
  // There's exactly one node here, and it's the one you're talking
  // to -- if this handler ran at all, this node is online right now.
  DynamicJsonDocument doc(128);
  JsonObject root = doc.to<JsonObject>();
  JsonObject o = root.createNestedObject(NODE_ID);
  o["nodeId"] = NODE_ID;
  o["ageMs"] = 0;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void onDataReceive(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len) {
  Serial.println();
  Serial.println("==========================================");
  Serial.println("          ESP-NOW PACKET RECEIVED");
  Serial.println("==========================================");

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

  // Defensively NUL-terminate — a corrupt/short char[] field must
  // never run off into adjacent memory when read as a C string.
  pkt.workerId[sizeof(pkt.workerId) - 1] = '\0';
  pkt.riskState[sizeof(pkt.riskState) - 1] = '\0';
  pkt.hazardType[sizeof(pkt.hazardType) - 1] = '\0';

  bool haveRssi = false;
  int rssi = 0;
  if (info != nullptr && info->rx_ctrl != nullptr) {
    rssi = info->rx_ctrl->rssi;
    haveRssi = true;
  }

  Serial.print("Node ID: ");
  Serial.println(NODE_ID);
  Serial.print("Worker ID: ");
  Serial.println(pkt.workerId);

  Serial.print("Message Type: ");
  if (pkt.msgType == 0) {
    Serial.println("STATUS (0)");
  } else if (pkt.msgType == 1) {
    Serial.println("DISTRESS (1)");
  } else {
    Serial.print("UNKNOWN (");
    Serial.print(pkt.msgType);
    Serial.println(")");
  }

  Serial.print("Risk State: ");
  Serial.println(pkt.riskState);
  Serial.print("Hazard Type: ");
  Serial.println(pkt.hazardType);
  Serial.print("Motion Energy: ");
  Serial.println(pkt.motionEnergy, 2);
  Serial.print("Seconds Since Motion: ");
  Serial.println(pkt.secondsSinceMotion, 2);
  Serial.print("Motion Energy At Trigger: ");
  Serial.println(pkt.motionEnergyAtTrigger, 2);
  Serial.print("Sequence Number: ");
  Serial.println(pkt.seq);

  Serial.print("ESP-NOW RSSI: ");
  if (haveRssi) {
    Serial.print(rssi);
    Serial.println(" dBm");
  } else {
    Serial.println("N/A");
  }

  if (pkt.msgType == 1) {
    Serial.println();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("             !!! DISTRESS !!!");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.print("Worker: ");
    Serial.println(pkt.workerId);
    Serial.print("Hazard: ");
    Serial.println(pkt.hazardType);
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  Serial.println("==========================================");

  // Store it directly -- no HTTP POST anywhere, this board IS the
  // server now.
  recordMessage(pkt.workerId, NODE_ID, pkt.msgType, pkt.seq, haveRssi, rssi,
                pkt.riskState, pkt.hazardType, pkt.motionEnergy,
                pkt.secondsSinceMotion, pkt.motionEnergyAtTrigger);
}

// ---------------------------------------------------------------------
// LOCAL FALL SIMULATION — generates dummy WorkerPackets on a timer and
// feeds them straight into recordMessage(), the exact same path a
// real ESP-NOW packet would use, so the dashboard can't tell the
// difference.
// ---------------------------------------------------------------------
uint32_t simSeqCounter = 0;
unsigned long lastSimStatusMs = 0;
unsigned long lastSimFallMs = 0;
bool simRecoveryPending = false;
unsigned long simRecoveryDueMs = 0;

void sendSimulatedStatus() {
  float motionEnergy = 0.30f + (random(0, 40) / 100.0f); // dummy "normal movement", 0.30-0.70 g
  recordMessage(SIM_WORKER_ID, NODE_ID, 0, simSeqCounter++, false, 0, "OK", "", motionEnergy, 0, 0);
}

void sendSimulatedFall() {
  Serial.println("[SIM] *** simulating a fall now ***");
  float secondsInactive = 25.0f + (random(0, 1500) / 100.0f); // ~25-40s inactive
  float motionAtTrigger = 0.10f + (random(0, 25) / 100.0f);    // low residual motion
  recordMessage(SIM_WORKER_ID, NODE_ID, 1, simSeqCounter++, false, 0,
                "FALL_SUSPECTED", "FALL_SUSPECTED", 0, secondsInactive, motionAtTrigger);
}

void runLocalSimulation() {
  unsigned long now = millis();

  if (now - lastSimStatusMs >= SIM_STATUS_INTERVAL_MS) {
    lastSimStatusMs = now;
    if (!simRecoveryPending) sendSimulatedStatus(); // pause routine STATUS while "down"
  }

  if (!simRecoveryPending && now - lastSimFallMs >= SIM_FALL_INTERVAL_MS) {
    lastSimFallMs = now;
    sendSimulatedFall();
    simRecoveryPending = true;
    simRecoveryDueMs = now + SIM_RECOVERY_DELAY_MS;
  }

  if (simRecoveryPending && now >= simRecoveryDueMs) {
    simRecoveryPending = false;
    Serial.println("[SIM] *** worker back up — sending recovery STATUS ***");
    sendSimulatedStatus();
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  randomSeed(analogRead(0));

  // AP_STA: AP so the laptop can join directly, STA alongside it
  // because that's the combination ESP-NOW is most reliably tested
  // against. This board never calls WiFi.begin() and never joins
  // anyone else's network.
  WiFi.mode(WIFI_AP_STA);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL);
  Serial.println(apOk ? "SoftAP started." : "[ERROR] SoftAP failed to start.");
  Serial.print("Network name: ");
  Serial.println(AP_SSID);
  Serial.print("Connect your laptop to it, then browse to: http://");
  Serial.println(WiFi.softAPIP());

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
  if (peerResult == ESP_OK) {
    Serial.println("Worker peer added successfully");
  } else if (peerResult == ESP_ERR_ESPNOW_EXIST) {
    Serial.println("Worker peer already exists");
  } else {
    Serial.print("[ERROR] Failed to add worker peer: ");
    Serial.println(peerResult);
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/workers", HTTP_GET, handleApiWorkers);
  server.on("/api/events", HTTP_GET, handleApiEvents);
  server.on("/api/nodes", HTTP_GET, handleApiNodes);
  server.begin();
  Serial.println("Web server started on port 80.");

  if (SIMULATE_LOCAL_DATA) {
    Serial.println("[SIM] Local fall simulation ENABLED — sending dummy STATUS/DISTRESS");
    Serial.println("[SIM] with no wearable required. Set SIMULATE_LOCAL_DATA = false once");
    Serial.println("[SIM] real ESP-NOW hardware is sending real data.");
    unsigned long now = millis();
    lastSimStatusMs = now;
    lastSimFallMs = now;
  }

  Serial.println("Wall node ready.");
}

void loop() {
  // ESP-NOW delivery is interrupt-driven via onDataReceive() and
  // needs nothing here. server.handleClient() is what actually
  // services dashboard requests -- it must run every loop iteration,
  // not on a timer, or the browser will see laggy/dropped requests.
  server.handleClient();

  if (SIMULATE_LOCAL_DATA) {
    runLocalSimulation();
  }
}
