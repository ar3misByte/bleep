// Sensora — Wearable Fall Simulator (ESP32)
//
// Stands in for a real worker wearable so you can test the wall node,
// the Flask relay, and the dashboard end-to-end without fall-detection
// sensor hardware. Sends STATUS packets every ~2s (normal movement)
// and a simulated DISTRESS ("fall") packet on a timer, followed by an
// automatic recovery STATUS a bit later — so you get the full
// OK -> FALL_SUSPECTED -> OK cycle the dashboard is built to show.
//
// This is a TEST TOOL, not the real wearable firmware — the real
// wearable would compute msgType/riskState/motionEnergy from an
// actual accelerometer instead of faking them on a timer.

#include <WiFi.h>
#include <esp_now.h>

// ---------------------------------------------------------------------
// EDIT THESE
// ---------------------------------------------------------------------
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"; // same network as the wall node —
                                                   // this locks the ESP-NOW radio to
                                                   // the router's WiFi channel so the
                                                   // wall node can hear it
const char* WORKER_ID     = "W1";                 // must match a workerId the dashboard roster expects

const unsigned long STATUS_INTERVAL_MS   = 2000;   // matches the "every ~2s" protocol recommendation
const unsigned long FALL_INTERVAL_MS     = 45000;  // simulate a fall roughly every 45s
const unsigned long RECOVERY_DELAY_MS    = 15000;  // "worker gets back up" this long after a fall
// ---------------------------------------------------------------------

// Must match the wall node's struct exactly — field order, types and
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

// Broadcast address — every ESP32 in ESP-NOW range on this WiFi
// channel receives it. Simpler for testing than pairing to the wall
// node's exact MAC address; a production wearable would target a
// specific wall node instead.
uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

uint32_t seqCounter = 0;
unsigned long lastStatusMs = 0;
unsigned long lastFallMs = 0;
bool recoveryPending = false;
unsigned long recoveryDueMs = 0;

void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "[ESP-NOW] sent OK" : "[ESP-NOW] send FAILED");
}

void sendPacket(uint8_t msgType, const char* riskState, const char* hazardType,
                float motionEnergy, float secondsSinceMotion, float motionEnergyAtTrigger) {
  WorkerPacket pkt = {};
  strncpy(pkt.workerId, WORKER_ID, sizeof(pkt.workerId) - 1);
  pkt.msgType = msgType;
  strncpy(pkt.riskState, riskState, sizeof(pkt.riskState) - 1);
  strncpy(pkt.hazardType, hazardType, sizeof(pkt.hazardType) - 1);
  pkt.motionEnergy = motionEnergy;
  pkt.secondsSinceMotion = secondsSinceMotion;
  pkt.motionEnergyAtTrigger = motionEnergyAtTrigger;
  pkt.seq = seqCounter++;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&pkt, sizeof(pkt));
  Serial.printf("[SIM] seq=%u msgType=%u riskState=%-16s -> %s\n",
                (unsigned)pkt.seq, pkt.msgType, pkt.riskState,
                result == ESP_OK ? "queued" : "FAILED to queue");
}

void sendNormalStatus() {
  // Dummy "normal movement" reading, 0.30-0.70 g.
  float motionEnergy = 0.30f + (random(0, 40) / 100.0f);
  sendPacket(0, "OK", "", motionEnergy, 0, 0);
}

void sendSimulatedFall() {
  Serial.println("[SIM] *** simulating a fall now ***");
  float secondsInactive = 25.0f + (random(0, 1500) / 100.0f); // ~25-40s
  float motionAtTrigger = 0.10f + (random(0, 25) / 100.0f);    // low residual motion
  sendPacket(1, "FALL_SUSPECTED", "FALL_SUSPECTED", 0, secondsInactive, motionAtTrigger);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  randomSeed(analogRead(0));

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi (to match the wall node's channel)");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    Serial.print(".");
    delay(250); // one-time boot connect, not the main loop
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected. WiFi channel: ");
    Serial.println(WiFi.channel());
  } else {
    Serial.println("[WARN] WiFi not connected — ESP-NOW likely won't reach the wall node if channels differ");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] esp_now_init failed");
    return;
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0; // use whatever channel WiFi is already on
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ERROR] esp_now_add_peer failed");
  }

  Serial.println("Wearable simulator ready.");
  Serial.println("  - STATUS every 2s");
  Serial.println("  - simulated fall roughly every 45s, auto-recovers 15s later");

  lastStatusMs = millis();
  lastFallMs = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    if (!recoveryPending) sendNormalStatus(); // pause routine STATUS while "down"
  }

  if (!recoveryPending && now - lastFallMs >= FALL_INTERVAL_MS) {
    lastFallMs = now;
    sendSimulatedFall();
    recoveryPending = true;
    recoveryDueMs = now + RECOVERY_DELAY_MS;
  }

  if (recoveryPending && now >= recoveryDueMs) {
    recoveryPending = false;
    Serial.println("[SIM] *** worker back up — sending recovery STATUS ***");
    sendNormalStatus();
  }
}
