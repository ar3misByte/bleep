// Sensora — Wall Node (ESP32)
//
// Receives WorkerPacket over ESP-NOW from a worker's wearable ESP32,
// converts it to the Sensora telemetry JSON envelope, and POSTs it to
// the dashboard's Flask server (see server/app.py in this repo).
//
// Requires: ESP32 Arduino core >= 2.0.0 (for the esp_now_recv_info_t
// callback signature used below — older cores only pass a MAC address
// and won't compile this file as-is).
//
// Library Manager: ArduinoJson (v6.x)

#include <WiFi.h>
#include <esp_now.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ---------------------------------------------------------------------
// EDIT THESE
// ---------------------------------------------------------------------
// Must match worker_node.ino's WIFI_SSID/WIFI_PASSWORD exactly — both
// boards need to be on the same WiFi network for ESP-NOW to reach
// across them (this also gives the wall node an IP to POST from).
const char* WIFI_SSID     = "shitstorm";
const char* WIFI_PASSWORD = "boombox1";
const char* DASHBOARD_URL = "http://10.177.169.223:5000/api/telemetry"; // laptop's WiFi IPv4 — update this if the laptop's IP changes
const char* NODE_ID       = "WALL1";

// How often to tell the dashboard "this wall node is alive," separate
// from relaying worker telemetry — this is what lets the dashboard
// show the wall node as online even during a quiet stretch with no
// worker messages to forward.
const unsigned long HEARTBEAT_INTERVAL_MS = 5000;

// The worker's wearable ESP32 — get this by reading "Wall MAC
// Address:" style output from WiFi.macAddress() on that board's own
// Serial Monitor at boot. Receiving doesn't strictly require pairing
// (ESP-NOW delivers to the registered callback from anyone in range,
// paired or not), but adding it as a peer here leaves the door open
// for the wall node to send something back to the worker later.
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

bool wifiIsUp() {
  return WiFi.status() == WL_CONNECTED;
}

// Sends one JSON document to the dashboard. Never blocks on a dead
// WiFi link — callers decide whether/how to retry. Returns the HTTP
// status code (e.g. 200), or a negative HTTPClient error code if the
// request itself never completed (bad URL, connection refused,
// timeout, etc.) — logging this is what makes "nothing shows up on
// the dashboard" debuggable instead of silent.
int sendToDashboard(JsonDocument& doc) {
  if (!wifiIsUp()) {
    Serial.println("[HTTP] skipped — WiFi not connected");
    return -1000;
  }

  HTTPClient http;
  http.begin(DASHBOARD_URL);
  http.addHeader("Content-Type", "application/json");

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  http.end(); // always release the connection, success or failure

  if (code > 0) {
    Serial.printf("[HTTP] POST %s -> %d\n", DASHBOARD_URL, code);
  } else {
    Serial.printf("[HTTP] POST %s FAILED, client error %d (%s)\n",
                  DASHBOARD_URL, code, HTTPClient::errorToString(code).c_str());
  }

  return code;
}

// Same host/port as DASHBOARD_URL, just a different path — derived at
// runtime so there's only one IP to edit when the laptop's IP changes.
String heartbeatUrl() {
  String url = String(DASHBOARD_URL);
  int idx = url.indexOf("/api/telemetry");
  if (idx == -1) return url; // shouldn't happen; DASHBOARD_URL always ends in /api/telemetry
  return url.substring(0, idx) + "/api/node-heartbeat";
}

// A dropped heartbeat isn't a big deal — the next one follows in
// HEARTBEAT_INTERVAL_MS, so this only logs on failure to avoid
// spamming Serial every few seconds when everything is fine.
void sendHeartbeat() {
  if (!wifiIsUp()) return;

  StaticJsonDocument<96> doc;
  doc["nodeId"] = NODE_ID;
  doc["timestamp"] = millis();

  HTTPClient http;
  http.begin(heartbeatUrl());
  http.addHeader("Content-Type", "application/json");
  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  http.end();

  if (code != 200) {
    Serial.printf("[HEARTBEAT] POST failed, code %d\n", code);
  }
}

void sendStatus(const WorkerPacket& pkt, int rssi, bool haveRssi) {
  StaticJsonDocument<256> doc;
  doc["schemaVersion"] = 1;
  doc["msgType"] = "STATUS";
  doc["workerId"] = pkt.workerId;
  doc["nodeId"] = NODE_ID;
  doc["seq"] = pkt.seq;
  if (haveRssi) doc["espnowRssi"] = rssi; else doc["espnowRssi"] = nullptr;
  doc["timestamp"] = millis();

  JsonObject payload = doc.createNestedObject("payload");
  payload["riskState"] = pkt.riskState;
  payload["motionEnergy"] = pkt.motionEnergy;
  payload["secondsSinceMotion"] = pkt.secondsSinceMotion;
  payload["battery"] = nullptr;

  serializeJson(doc, Serial);
  Serial.println();

  // STATUS is not retried on failure — the next one follows in a few
  // seconds, and retrying every STATUS send would stall the main loop.
  sendToDashboard(doc);
}

void sendDistress(const WorkerPacket& pkt, int rssi, bool haveRssi) {
  StaticJsonDocument<256> doc;
  doc["schemaVersion"] = 1;
  doc["msgType"] = "DISTRESS";
  doc["workerId"] = pkt.workerId;
  doc["nodeId"] = NODE_ID;
  doc["seq"] = pkt.seq;
  if (haveRssi) doc["espnowRssi"] = rssi; else doc["espnowRssi"] = nullptr;
  doc["timestamp"] = millis();

  JsonObject payload = doc.createNestedObject("payload");
  payload["hazardType"] = pkt.hazardType;
  payload["riskState"] = pkt.riskState;
  payload["secondsInactive"] = pkt.secondsSinceMotion;
  payload["motionEnergyAtTrigger"] = pkt.motionEnergyAtTrigger;
  payload["battery"] = nullptr;

  serializeJson(doc, Serial);
  Serial.println();

  // A dropped DISTRESS message is not acceptable — retry a few times.
  // The short delay() here is the one deliberate exception to the
  // "no blocking delay()" rule: it's on the DISTRESS path only, not
  // the routine ESP-NOW receive path.
  for (int attempt = 0; attempt < 3; attempt++) {
    if (sendToDashboard(doc) == 200) return;
    delay(300);
  }
  Serial.println("[WARN] DISTRESS POST failed after 3 attempts");
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

  // This is the part the standalone receiver sketch doesn't do —
  // forward whatever we just printed on to the actual dashboard.
  if (pkt.msgType == 1) {
    sendDistress(pkt, rssi, haveRssi);
  } else {
    sendStatus(pkt, rssi, haveRssi);
  }
}

// ---------------------------------------------------------------------
// LOCAL FALL SIMULATION — generates dummy WorkerPackets on a timer and
// feeds them through the exact same sendStatus()/sendDistress() path
// a real ESP-NOW packet would use, so the dashboard can't tell the
// difference. haveRssi is false throughout: there's no real
// over-the-air reception happening, so we send null rather than a
// faked signal strength.
// ---------------------------------------------------------------------
uint32_t simSeqCounter = 0;
unsigned long lastSimStatusMs = 0;
unsigned long lastSimFallMs = 0;
bool simRecoveryPending = false;
unsigned long simRecoveryDueMs = 0;

unsigned long lastHeartbeatMs = 0;

void sendSimulatedStatus() {
  WorkerPacket pkt = {};
  strncpy(pkt.workerId, SIM_WORKER_ID, sizeof(pkt.workerId) - 1);
  pkt.msgType = 0;
  strncpy(pkt.riskState, "OK", sizeof(pkt.riskState) - 1);
  pkt.motionEnergy = 0.30f + (random(0, 40) / 100.0f); // dummy "normal movement", 0.30-0.70 g
  pkt.secondsSinceMotion = 0;
  pkt.seq = simSeqCounter++;
  sendStatus(pkt, 0, false);
}

void sendSimulatedFall() {
  Serial.println("[SIM] *** simulating a fall now ***");
  WorkerPacket pkt = {};
  strncpy(pkt.workerId, SIM_WORKER_ID, sizeof(pkt.workerId) - 1);
  pkt.msgType = 1;
  strncpy(pkt.riskState, "FALL_SUSPECTED", sizeof(pkt.riskState) - 1);
  strncpy(pkt.hazardType, "FALL_SUSPECTED", sizeof(pkt.hazardType) - 1);
  pkt.secondsSinceMotion = 25.0f + (random(0, 1500) / 100.0f); // ~25-40s inactive
  pkt.motionEnergyAtTrigger = 0.10f + (random(0, 25) / 100.0f); // low residual motion
  pkt.seq = simSeqCounter++;
  sendDistress(pkt, 0, false);
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

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");
  uint32_t startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    Serial.print(".");
    delay(250); // acceptable here: one-time boot connect, not the main loop
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WARN] WiFi not connected yet — will keep retrying via wifiIsUp() checks");
  }

  Serial.print("Wall MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] esp_now_init failed");
    return;
  }
  esp_now_register_recv_cb(onDataReceive);

  esp_now_peer_info_t workerPeer = {};
  memcpy(workerPeer.peer_addr, WORKER_MAC, 6);
  workerPeer.channel = 0; // use current WiFi channel
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

  Serial.print("Posting telemetry to: ");
  Serial.println(DASHBOARD_URL);
  if (String(DASHBOARD_URL).indexOf("192.168.1.50") != -1) {
    Serial.println("[WARN] DASHBOARD_URL still looks like the placeholder IP — "
                    "edit it at the top of this file to your dashboard laptop's actual IP.");
  }

  if (SIMULATE_LOCAL_DATA) {
    Serial.println("[SIM] Local fall simulation ENABLED — sending dummy STATUS/DISTRESS");
    Serial.println("[SIM] with no wearable required. Set SIMULATE_LOCAL_DATA = false once");
    Serial.println("[SIM] real ESP-NOW hardware is sending real data.");
    unsigned long now = millis();
    lastSimStatusMs = now;
    lastSimFallMs = now;
  }

  lastHeartbeatMs = millis();
  sendHeartbeat(); // announce presence immediately at boot, don't wait for the first interval

  Serial.println("Wall node ready.");
}

void loop() {
  // ESP-NOW delivery (real hardware) is interrupt-driven via
  // onDataReceive() and needs nothing here. WiFi reconnection is handled
  // by the core's built-in auto-reconnect (on by default in WIFI_STA
  // mode). The only polling this loop does is the local simulator and
  // the heartbeat below, both millis()-based, not blocking delay().
  if (SIMULATE_LOCAL_DATA) {
    runLocalSimulation();
  }

  unsigned long now = millis();
  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    sendHeartbeat();
  }
}
