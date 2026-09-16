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
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* DASHBOARD_URL = "http://192.168.1.50:5000/api/telemetry"; // set to the laptop's actual IP
const char* NODE_ID       = "WALL1";
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

void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len) {
  if (len != sizeof(WorkerPacket)) {
    Serial.printf("[WARN] dropped packet: expected %u bytes, got %d\n", (unsigned)sizeof(WorkerPacket), len);
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

  if (pkt.msgType == 1) {
    sendDistress(pkt, rssi, haveRssi);
  } else {
    sendStatus(pkt, rssi, haveRssi);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

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

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] esp_now_init failed");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);

  Serial.print("Posting telemetry to: ");
  Serial.println(DASHBOARD_URL);
  if (String(DASHBOARD_URL).indexOf("192.168.1.50") != -1) {
    Serial.println("[WARN] DASHBOARD_URL still looks like the placeholder IP — "
                    "edit it at the top of this file to your dashboard laptop's actual IP.");
  }

  Serial.println("Wall node ready.");
}

void loop() {
  // Nothing to poll here — ESP-NOW delivery is interrupt-driven via
  // onDataRecv(); WiFi reconnection is handled by the core's built-in
  // auto-reconnect (WiFi.setAutoReconnect(true) is on by default in
  // WIFI_STA mode). Keep loop() free of blocking delay().
}
