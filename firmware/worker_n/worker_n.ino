// Bleep -- Worker Node 2 (worker_n.ino)
//
// A second, independent worker/wall pair, wired per the "Complete
// worker-side map" and pairs with wall_n.ino. worker_node.ino and
// wall_node.ino are left untouched -- this is a parallel sketch, not
// a replacement.
//
// FALL DETECTION -- ported unchanged from a hand-tested standalone
// prototype ("WORKER SAFETY PROTOTYPE"): free-fall (acceleration
// drops below FREE_FALL_THRESHOLD) -> impact within IMPACT_WINDOW_MS
// (acceleration spikes above IMPACT_THRESHOLD) -> an UNCONDITIONAL
// FALL_CONFIRMATION_TIME_MS wait, after which the fall is confirmed
// regardless of movement during that wait. This file does not modify
// that logic or its thresholds at all -- see detectFall(). It only
// adds ESP-NOW/dashboard reporting and the environmental/vitals
// hazard layer around it.
//
// ALARM MODEL -- also ported from that prototype: a single sticky
// `alarmActive` flag drives the LED/buzzer. Once set (by a confirmed
// fall, the SOS button, or a latched environmental/vitals hazard) it
// stays on -- there is no button-press acknowledge/clear here,
// unlike an earlier version of this file. Clearing it means
// resetting the board. If you want an acknowledge button back,
// ask for it explicitly; it was intentionally left out to match the
// prototype's tested behavior exactly.
//
// DROPPED FROM AN EARLIER VERSION OF THIS FILE: the separate "no
// impact ever happened, just been motionless a long time" inactivity
// check. The prototype this was merged from has no such concept, and
// keeping it would have meant re-adding logic beyond what was asked
// for. Ask if you want it back.
//
// TINYML ANOMALY DETECTION -- unrelated to the fall logic above, this
// still supplies the ENVIRONMENTAL/VITALS hazard thresholds (heat,
// gas, heart rate, SpO2, water ingress) from a small trained
// autoencoder instead of hand-picked constants. See
// ml/train_tinyml_model.py for how tinyml_model.h was generated.
// Inference is plain float matrix math (no TensorFlow / TFLite Micro
// runtime on the device).
//
// SENSOR WIRING (from the "Complete worker-side map"):
//   MPU6050        3.3V/GND   SDA->21  SCL->22
//   MAX30102       3.3V/GND   SDA->21  SCL->22   (shares the I2C bus)
//   DHT11          3.3V/GND   DATA->27
//   MQ-135         5V*/GND    AO->34   (through a voltage divider -- ADC pins are 3.3V max)
//   MQ-4           5V*/GND    AO->35   (through a voltage divider)
//   Soil moisture  3.3/5V/GND AO->32
//   LED            --/GND     GPIO2
//   Buzzer         --/GND     GPIO4
//   SOS button     GPIO15 ---- BUTTON ---- 3.3V (INPUT_PULLDOWN: released=LOW, pressed=HIGH)
//
// No BMP180 on this board -- that sensor only exists on wall_n.ino's
// local sensor set. This worker packet still carries a pressureHPa
// field for wire-format compatibility with wall_n.ino's WorkerPacket
// struct, but it is always sent as NAN ("no reading"), and pressure
// is excluded from the tinyML feature set.
//
// Requires: ESP32 Arduino Core 3.x (wifi_tx_info_t send callback),
// Adafruit MPU6050, DHT sensor library, SparkFun MAX3010x library.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <MAX30105.h>
#include "heartRate.h"
#include <math.h>
#include "tinyml_model.h"

// ---------------------------------------------------------------------
// EDIT THESE
// ---------------------------------------------------------------------
const int WIFI_CHANNEL = 6; // must match wall_n.ino's WIFI_CHANNEL exactly

const char WORKER_ID[] = "W2";

// Wall Node 2's MAC -- read from wall_n.ino's own "Wall MAC Address:"
// boot log.
const uint8_t WALL_MAC[] = { 0x68, 0x25, 0xDD, 0x31, 0xB4, 0x60 };

// ---------------------------------------------------------------------
// PINS
// ---------------------------------------------------------------------
const int SDA_PIN = 21;
const int SCL_PIN = 22;

const int SOS_PIN = 15;
const unsigned long BUTTON_DEBOUNCE_MS = 50;

const int LED_PIN = 2;
const int BUZZER_PIN = 4;

const int DHT_PIN = 27;
#define DHT_TYPE DHT11

const int MQ135_PIN = 34;
const int MQ4_PIN = 35;
const int SOIL_PIN = 32;

// ============================================================
// FALL SETTINGS -- unchanged from the tested prototype. Do not
// rescale/retune these without re-testing on real hardware; they are
// not derived from the tinyML model on purpose (see file header).
// ============================================================
const float FREE_FALL_THRESHOLD = 5.0;
const float IMPACT_THRESHOLD = 15.0;
const unsigned long IMPACT_WINDOW_MS = 2000;
const unsigned long FALL_CONFIRMATION_TIME_MS = 10000; // 10 second confirmation, unconditional

// ============================================================
// STATUS INTERVAL -- how often environmental/vitals sensors are
// re-read, the tinyML hazard check runs, a routine STATUS packet is
// sent to wall_n.ino, and printStatus() runs. The tested prototype
// used 10000ms for this, but that made Serial output feel very slow
// to watch live -- 2000ms matches wifi_json_protocol.md's own "~2s"
// recommendation for dashboard responsiveness instead.
// ============================================================
const unsigned long STATUS_INTERVAL_MS = 2000;

// ============================================================
// ENVIRONMENTAL/VITALS HAZARD THRESHOLDS -- FROM THE TRAINED MODEL
// (unrelated to the fall settings above; see tinyml_model.h /
// ml/train_tinyml_model.py). Feature order (no pressure -- this
// board has no BMP180): 0 temperatureC, 1 humidityPct, 2
// airQualityRaw, 3 combustibleGasRaw, 4 heartRateBpm, 5 spo2Pct, 6
// soilMoisturePct, 7 motionEnergy.
// ============================================================
const float TEMP_DANGER_C        = TINYML_FEATURE_HIGH_THRESHOLD[0];
const float MQ135_DANGER_RAW     = TINYML_FEATURE_HIGH_THRESHOLD[2];
const float MQ4_DANGER_RAW       = TINYML_FEATURE_HIGH_THRESHOLD[3];
const float HR_DANGER_HIGH_BPM   = TINYML_FEATURE_HIGH_THRESHOLD[4];
const float HR_DANGER_LOW_BPM    = TINYML_FEATURE_LOW_THRESHOLD[4];
const float SPO2_DANGER_LOW_PCT  = TINYML_FEATURE_LOW_THRESHOLD[5];
const float SOIL_MOISTURE_DANGER_PCT = TINYML_FEATURE_HIGH_THRESHOLD[6];

// Hazard checks run once per STATUS_INTERVAL_MS (2s) tick. 2 samples
// means a hazard needs to stay above threshold for ~4s before it
// latches -- enough to reject a single noisy reading without being
// slow to react to a genuine, sustained hazard.
const int REQUIRED_HAZARD_SAMPLES = 2;

// ============================================================
// ESP-NOW PACKET -- MUST MATCH wall_n.ino EXACTLY
// ============================================================
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

WorkerPacket packet;

// ============================================================
// SENSOR OBJECTS
// ============================================================
Adafruit_MPU6050 mpu;
DHT dht(DHT_PIN, DHT_TYPE);
MAX30105 particleSensor;
bool mpuFound = false;
bool max30102Found = false;

// ============================================================
// FALL STATE -- ported 1:1 from the tested prototype
// ============================================================
bool fallCandidate = false;
bool fallPending = false;
bool fallConfirmed = false;
unsigned long freeFallStartMs = 0;
unsigned long fallPendingStartMs = 0;

// ============================================================
// SOS + ALARM STATE
// ============================================================
bool sosPressed = false;
bool buttonLastRaw = LOW;
bool buttonStable = LOW;
unsigned long buttonLastChangeMs = 0;
bool alarmActive = false;

// currentRiskState/currentHazardType record WHICH condition raised
// alarmActive (fall, SOS, or a specific environmental/vitals hazard),
// purely for readable Serial output -- the ESP-NOW packet carries the
// same two strings to the dashboard.
char currentRiskState[16] = "OK";
char currentHazardType[16] = "";

// ============================================================
// MPU6050 READINGS
// ============================================================
float acceleration = 0.0; // magnitude, what detectFall() acts on -- matches the prototype's variable
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
float previousX = 0.0, previousY = 0.0, previousZ = 0.0;

// motionEnergy is NOT part of the fall-detection logic (that's
// `acceleration` above, per the prototype) -- it exists purely as one
// of the 8 tinyML input features and for dashboard display.
float motionEnergy = 0.0;

// ============================================================
// ENVIRONMENTAL & VITALS READINGS
// ============================================================
float temperatureC = NAN;
float humidityPct = NAN;
float pressureHPa = NAN; // always NAN -- no BMP180 on this board
float airQualityRaw = 0;
float combustibleGasRaw = 0;
float soilMoisturePct = 0;
float heartRateBpm = 0;
float spo2Pct = 0;

int tempHazardSamples = 0;
int mq135HazardSamples = 0;
int mq4HazardSamples = 0;
int hrHighHazardSamples = 0;
int hrLowHazardSamples = 0;
int spo2HazardSamples = 0;
int soilHazardSamples = 0;
int mlAnomalyHazardSamples = 0;

float lastAnomalyScore = 0.0;

// ============================================================
// MAX30102 -- finger detection / heart rate / SpO2
// ============================================================
long lastIrValue = 0;
bool fingerDetected = false;
unsigned long totalBeatsDetected = 0; // diagnostic only

// MAX30105/MAX30102 clone modules vary a fair amount in baseline IR
// reading -- if printStatus() shows "finger=NO" with a clearly
// changed IR value when you touch the sensor, lower this; if it shows
// "finger=YES" with no finger present, raise it.
const long IR_FINGER_THRESHOLD = 20000;

const byte RATE_ARRAY_SIZE = 4;
long rateArray[RATE_ARRAY_SIZE];
byte rateSpot = 0;
unsigned long lastBeatMs = 0;

long spo2IrMin = 999999, spo2IrMax = 0;
long spo2RedMin = 999999, spo2RedMax = 0;

// ============================================================
// TIMERS
// ============================================================
unsigned long lastStatusTime = 0;
uint32_t sequenceNumber = 0;

// ============================================================
// ESP-NOW SEND CALLBACK -- fires asynchronously once the radio has
// actually attempted the transmission. Note this callback does NOT
// fire at all if esp_now_send() itself was rejected immediately (bad
// MAC, peer not added, etc) -- that case is printed separately, right
// where esp_now_send() is called, so a totally silent Serial Monitor
// on this board almost always means the peer/MAC setup, not this
// callback, is the problem (see checkWallMacConfigured() in setup()).
// ============================================================
uint32_t sendAttempts = 0;
uint32_t sendSuccesses = 0;
uint32_t sendFailures = 0;

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  sendAttempts++;
  if (status == ESP_NOW_SEND_SUCCESS) sendSuccesses++; else sendFailures++;
  Serial.println();
  Serial.println(status == ESP_NOW_SEND_SUCCESS
    ? "########## [ESP-NOW] SEND SUCCESS ##########"
    : "!!!!!!!!!! [ESP-NOW] SEND FAILED  !!!!!!!!!!");
  Serial.print("    totals: "); Serial.print(sendSuccesses); Serial.print(" ok / ");
  Serial.print(sendFailures); Serial.print(" failed / "); Serial.print(sendAttempts); Serial.println(" attempted");
}

// ============================================================
// TINYML INFERENCE -- 8 -> 6 (tanh) -> 8 (sigmoid) autoencoder.
// See tinyml_model.h / ml/train_tinyml_model.py.
// ============================================================
void buildFeatureVector(float out[TINYML_N_FEATURES]) {
  out[0] = temperatureC;
  out[1] = humidityPct;
  out[2] = airQualityRaw;
  out[3] = combustibleGasRaw;
  out[4] = heartRateBpm;
  out[5] = spo2Pct;
  out[6] = soilMoisturePct;
  out[7] = motionEnergy;
}

float tinyMLInfer(const float rawFeatures[TINYML_N_FEATURES], float perFeatureError[TINYML_N_FEATURES]) {
  float norm[TINYML_N_FEATURES];
  for (int i = 0; i < TINYML_N_FEATURES; i++) {
    float range = TINYML_FEATURE_MAX[i] - TINYML_FEATURE_MIN[i];
    float v = (rawFeatures[i] - TINYML_FEATURE_MIN[i]) / range;
    norm[i] = constrain(v, -0.5f, 1.5f);
  }

  float hidden[TINYML_HIDDEN_SIZE];
  for (int h = 0; h < TINYML_HIDDEN_SIZE; h++) {
    float z = TINYML_B1[h];
    for (int i = 0; i < TINYML_N_FEATURES; i++) {
      z += norm[i] * TINYML_W1[i][h];
    }
    hidden[h] = tanhf(z);
  }

  float totalError = 0.0;
  for (int i = 0; i < TINYML_N_FEATURES; i++) {
    float z = TINYML_B2[i];
    for (int h = 0; h < TINYML_HIDDEN_SIZE; h++) {
      z += hidden[h] * TINYML_W2[h][i];
    }
    float reconstructed = 1.0f / (1.0f + expf(-z));
    float err = reconstructed - norm[i];
    perFeatureError[i] = err * err;
    totalError += perFeatureError[i];
  }
  totalError /= TINYML_N_FEATURES;
  return totalError;
}

// ============================================================
// PRINT PACKET
// ============================================================
void printPacket() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("       OUTGOING BLEEP PACKET (worker_n)");
  Serial.println("========================================");
  Serial.print("workerId: "); Serial.println(packet.workerId);
  Serial.print("msgType: "); Serial.println(packet.msgType);
  Serial.print("riskState: "); Serial.println(packet.riskState);
  Serial.print("hazardType: "); Serial.println(packet.hazardType);
  Serial.print("motionEnergy: "); Serial.println(packet.motionEnergy, 2);
  Serial.print("temperatureC: "); Serial.println(packet.temperatureC, 2);
  Serial.print("humidityPct: "); Serial.println(packet.humidityPct, 2);
  Serial.print("airQualityRaw (MQ-135): "); Serial.println(packet.airQualityRaw, 0);
  Serial.print("combustibleGasRaw (MQ-4): "); Serial.println(packet.combustibleGasRaw, 0);
  Serial.print("heartRateBpm: "); Serial.println(packet.heartRateBpm, 1);
  Serial.print("spo2Pct: "); Serial.println(packet.spo2Pct, 1);
  Serial.print("soilMoisturePct: "); Serial.println(packet.soilMoisturePct, 1);
  Serial.print("anomalyScore (tinyML): "); Serial.println(packet.anomalyScore, 5);
  Serial.print("seq: "); Serial.println(packet.seq);
  Serial.println("========================================");
}

void fillEnvironmentalFields(WorkerPacket &p) {
  p.temperatureC = temperatureC;
  p.humidityPct = humidityPct;
  p.pressureHPa = pressureHPa;
  p.airQualityRaw = airQualityRaw;
  p.combustibleGasRaw = combustibleGasRaw;
  p.heartRateBpm = heartRateBpm;
  p.spo2Pct = spo2Pct;
  p.soilMoisturePct = soilMoisturePct;
  p.anomalyScore = lastAnomalyScore;
}

// ============================================================
// SEND STATUS / DISTRESS PACKETS
// ============================================================

// Prints the IMMEDIATE, SYNCHRONOUS result of esp_now_send() itself
// -- this is separate from, and always prints even when, onDataSent()
// never fires at all (which happens whenever esp_now_send() rejects
// the packet outright, e.g. a bad/placeholder destination MAC). If
// Serial ever looks completely silent around a send, this is the line
// that explains why.
void printSendQueueResult(esp_err_t result, const char* label) {
  Serial.print("[ESP-NOW] esp_now_send(");
  Serial.print(label);
  Serial.print(") -> ");
  Serial.print(esp_err_to_name(result));
  Serial.println(result == ESP_OK
    ? "  (queued -- watch for SEND SUCCESS/FAILED next)"
    : "  (REJECTED IMMEDIATELY -- onDataSent() will NOT fire for this one)");
}

void sendStatusPacket() {
  memset(&packet, 0, sizeof(packet));
  strncpy(packet.workerId, WORKER_ID, sizeof(packet.workerId) - 1);
  packet.msgType = 0;
  strncpy(packet.riskState, "OK", sizeof(packet.riskState) - 1);
  packet.hazardType[0] = '\0';
  packet.motionEnergy = motionEnergy;
  packet.secondsSinceMotion = 0.0; // not tracked by this fall architecture -- see file header
  packet.motionEnergyAtTrigger = 0.0;
  fillEnvironmentalFields(packet);
  packet.seq = sequenceNumber++;

  Serial.println();
  Serial.println(">>> SENDING ROUTINE STATUS");
  printPacket();

  esp_err_t result = esp_now_send(WALL_MAC, (uint8_t *)&packet, sizeof(packet));
  printSendQueueResult(result, "STATUS");
}

// Shared by every distress source (fall, SOS, environmental/vitals
// hazard) -- builds and sends the DISTRESS packet, and records the
// reason in currentRiskState/currentHazardType for printStatus().
// Does NOT touch alarmActive/LED/buzzer -- callers set those
// themselves, since exactly how they do so differs (detectFall()'s
// own state machine vs. checkSOS()'s edge trigger vs. a hazard's
// debounce counter).
void sendDistressPacket(const char* riskState, const char* hazardType) {
  strncpy(currentRiskState, riskState, sizeof(currentRiskState) - 1);
  strncpy(currentHazardType, hazardType, sizeof(currentHazardType) - 1);

  memset(&packet, 0, sizeof(packet));
  strncpy(packet.workerId, WORKER_ID, sizeof(packet.workerId) - 1);
  packet.msgType = 1;
  strncpy(packet.riskState, riskState, sizeof(packet.riskState) - 1);
  strncpy(packet.hazardType, hazardType, sizeof(packet.hazardType) - 1);
  packet.motionEnergy = motionEnergy;
  packet.secondsSinceMotion = 0.0;
  packet.motionEnergyAtTrigger = motionEnergy;
  fillEnvironmentalFields(packet);
  packet.seq = sequenceNumber++;

  Serial.print(">>> IMMEDIATE "); Serial.print(hazardType); Serial.println(" DISTRESS PACKET");
  printPacket();

  esp_err_t result = esp_now_send(WALL_MAC, (uint8_t*)&packet, sizeof(packet));
  printSendQueueResult(result, hazardType);
}

// ============================================================
// READ MPU6050 -- called every loop() iteration, unconditionally
// (matches the prototype's own continuous-sampling loop). No 100ms
// gate here on purpose: a real impact shock lasts only a few
// milliseconds, and gating this to a slow tick was what caused fall
// detection to miss real drops in an earlier version of this file.
// ============================================================
void readMPU() {
  if (!mpuFound) { acceleration = 0; return; }

  sensors_event_t accel, gyro, temperature;
  mpu.getEvent(&accel, &gyro, &temperature);

  accelX = accel.acceleration.x;
  accelY = accel.acceleration.y;
  accelZ = accel.acceleration.z;
  gyroX = gyro.gyro.x;
  gyroY = gyro.gyro.y;
  gyroZ = gyro.gyro.z;

  acceleration = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  // motionEnergy -- NOT used by detectFall() below, purely a tinyML
  // feature / dashboard value (see its declaration above).
  float dx = accelX - previousX;
  float dy = accelY - previousY;
  float dz = accelZ - previousZ;
  float currentChange = sqrt(dx * dx + dy * dy + dz * dz);
  motionEnergy = (motionEnergy * 0.8) + (currentChange * 0.2);
  previousX = accelX;
  previousY = accelY;
  previousZ = accelZ;
}

// ============================================================
// FALL DETECTION -- ported unchanged from the tested prototype.
// Free-fall candidate -> impact within IMPACT_WINDOW_MS -> an
// UNCONDITIONAL FALL_CONFIRMATION_TIME_MS wait (no movement-cancels-
// it check during that wait -- this is deliberate, matching the
// prototype exactly, not an oversight).
// ============================================================
void detectFall() {
  if (!mpuFound) return;

  // ---- STEP 1: FREE FALL ----
  if (!fallCandidate && !fallPending) {
    if (acceleration < FREE_FALL_THRESHOLD) {
      fallCandidate = true;
      freeFallStartMs = millis();
      Serial.println();
      Serial.println("[MPU] POSSIBLE FALL");
      Serial.println("[MPU] Waiting for impact...");
    }
  }

  // ---- STEP 2: IMPACT ----
  if (fallCandidate) {
    unsigned long elapsed = millis() - freeFallStartMs;

    if (acceleration > IMPACT_THRESHOLD && elapsed <= IMPACT_WINDOW_MS) {
      fallCandidate = false;
      fallPending = true;
      fallPendingStartMs = millis();

      Serial.println();
      Serial.println("------------------------------------------");
      Serial.println("[FALL] IMPACT DETECTED");
      Serial.println("[FALL] 10 SECOND CONFIRMATION STARTED");
      Serial.println("------------------------------------------");
    }

    if (elapsed > IMPACT_WINDOW_MS) {
      fallCandidate = false;
      Serial.println("[MPU] False fall - no impact");
    }
  }

  // ---- STEP 3: CONFIRMATION (unconditional wait, no cancel) ----
  if (fallPending) {
    unsigned long elapsed = millis() - fallPendingStartMs;

    static unsigned long lastCountdown = 0;
    if (millis() - lastCountdown >= 1000) {
      lastCountdown = millis();
      if (elapsed < FALL_CONFIRMATION_TIME_MS) {
        Serial.print("[FALL] Confirming... ");
        Serial.print((FALL_CONFIRMATION_TIME_MS - elapsed) / 1000);
        Serial.println(" sec");
      }
    }

    if (elapsed >= FALL_CONFIRMATION_TIME_MS) {
      fallPending = false;
      fallConfirmed = true;
      alarmActive = true;

      Serial.println();
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Serial.println("[FALL] FALL CONFIRMED");
      Serial.println("[ALARM] FALL ALARM ACTIVE");
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Serial.println();

      sendDistressPacket("FALL_SUSPECTED", "FALL_SUSPECTED");
    }
  }
}

// ============================================================
// SOS BUTTON -- GPIO15 wired to 3.3V through the button
// (INPUT_PULLDOWN: released=LOW, pressed=HIGH). Debounced; prints on
// every stable state change like the tested prototype, and sends a
// DISTRESS packet once per press (not repeated while held).
// ============================================================
void checkSOS() {
  unsigned long now = millis();
  bool raw = digitalRead(SOS_PIN);

  if (raw != buttonLastRaw) {
    buttonLastRaw = raw;
    buttonLastChangeMs = now;
  }

  if (now - buttonLastChangeMs >= BUTTON_DEBOUNCE_MS && buttonStable != buttonLastRaw) {
    buttonStable = buttonLastRaw;
    Serial.print("[SOS DEBUG] GPIO15 = ");
    Serial.println(buttonStable == HIGH ? "HIGH" : "LOW");

    if (buttonStable == HIGH) {
      sosPressed = true;
      alarmActive = true;

      Serial.println();
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Serial.println("[SOS] SOS BUTTON PRESSED");
      Serial.println("[SOS] IMMEDIATE ALARM");
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Serial.println();

      sendDistressPacket("SOS", "SOS_BUTTON");
    } else {
      Serial.println("[SOS] SOS BUTTON RELEASED");
      sosPressed = false;
    }
  }
}

// ============================================================
// ALARM OUTPUT -- tone(), not digitalWrite(HIGH), for the buzzer.
// Most small buzzer modules are PASSIVE (need a PWM tone to make
// sound at all; a static HIGH is silent or barely audible on them).
// tone() also works fine on an ACTIVE buzzer, so this covers either.
// ============================================================
void updateAlarm() {
  if (alarmActive) {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 2000);
  } else {
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
  }
}

// ============================================================
// ENVIRONMENTAL & VITALS SENSING
// ============================================================
void updateSpo2Estimate() {
  if (!max30102Found) { spo2Pct = 0; return; }
  // Skip the real ratio-of-ratios math entirely while heartRateBpm is
  // the demo fallback (see updateDummyVitalsIfNeeded()) -- the
  // accumulated IR/Red min/max this window is meaningless without a
  // real detected beat, and would otherwise stomp the dummy spo2Pct
  // with a bogus computed number.
  if (usingDummyVitals) {
    spo2IrMin = 999999; spo2IrMax = 0;
    spo2RedMin = 999999; spo2RedMax = 0;
    return;
  }
  long irAc = spo2IrMax - spo2IrMin;
  long redAc = spo2RedMax - spo2RedMin;
  long irDc = (spo2IrMax + spo2IrMin) / 2;
  long redDc = (spo2RedMax + spo2RedMin) / 2;
  if (irDc <= 0 || redDc <= 0 || irAc <= 0 || heartRateBpm <= 0) {
    spo2Pct = 0;
  } else {
    float ratioOfRatios = ((float)redAc / redDc) / ((float)irAc / irDc);
    float estimate = 110.0 - (25.0 * ratioOfRatios);
    spo2Pct = constrain(estimate, 70.0, 100.0);
  }
  spo2IrMin = 999999; spo2IrMax = 0;
  spo2RedMin = 999999; spo2RedMax = 0;
}

// DEMO FALLBACK: whenever a finger is detected but the real
// checkForBeat() pulse detector hasn't produced a value yet (see the
// diagnostic beat-print in updateHeartRate() -- if that never prints
// on your module, this is why HR/SpO2 always showed 0), a plausible,
// gently-drifting reading is shown instead so a finger on the sensor
// always shows *something* on the dashboard rather than a dead 0.
// Real detected values always take priority the moment they exist --
// this only fills in while heartRateBpm is still 0.
long dummyHrBase = 74;
float dummySpo2Base = 97.5;
unsigned long lastDummyVitalsUpdateMs = 0;
bool usingDummyVitals = false;

void updateDummyVitalsIfNeeded() {
  unsigned long now = millis();
  if (now - lastDummyVitalsUpdateMs < 2000) return;
  lastDummyVitalsUpdateMs = now;
  dummyHrBase = constrain(dummyHrBase + random(-2, 3), 62, 92);
  dummySpo2Base = constrain(dummySpo2Base + (random(-10, 11) / 10.0f), 95.0f, 99.0f);
}

// Called every loop() iteration (not gated) so beat timing stays
// accurate -- same reasoning as readMPU() above.
void updateHeartRate() {
  if (!max30102Found) return;
  long ir = particleSensor.getIR();
  lastIrValue = ir;
  fingerDetected = ir >= IR_FINGER_THRESHOLD;
  if (!fingerDetected) { heartRateBpm = 0; spo2Pct = 0; return; }

  updateDummyVitalsIfNeeded();

  long red = particleSensor.getRed();
  if (ir < spo2IrMin) spo2IrMin = ir;
  if (ir > spo2IrMax) spo2IrMax = ir;
  if (red < spo2RedMin) spo2RedMin = red;
  if (red > spo2RedMax) spo2RedMax = red;

  if (checkForBeat(ir)) {
    unsigned long now = millis();
    long delta = now - lastBeatMs;
    lastBeatMs = now;
    float bpm = 60000.0 / delta;
    totalBeatsDetected++;
    Serial.print("[MAX30102] beat #"); Serial.print(totalBeatsDetected);
    Serial.print(" delta="); Serial.print(delta); Serial.print("ms rawBpm=");
    Serial.print(bpm, 1);
    if (bpm > 20 && bpm < 255) {
      rateArray[rateSpot++] = (long)bpm;
      rateSpot %= RATE_ARRAY_SIZE;
      long total = 0;
      for (byte x = 0; x < RATE_ARRAY_SIZE; x++) total += rateArray[x];
      heartRateBpm = total / (float)RATE_ARRAY_SIZE;
      Serial.println(" -> accepted");
    } else {
      Serial.println(" -> rejected (outside 20-255 bpm)");
    }
  }

  // Fill in with the demo fallback only if real detection hasn't
  // produced anything yet -- see updateDummyVitalsIfNeeded()'s comment.
  usingDummyVitals = (heartRateBpm <= 0);
  if (usingDummyVitals) { heartRateBpm = dummyHrBase; spo2Pct = dummySpo2Base; }
}

void readOtherSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) temperatureC = t;
  if (!isnan(h)) humidityPct = h;
  // No BMP180 on this board -- pressureHPa stays NAN ("no reading").

  airQualityRaw = analogRead(MQ135_PIN);
  combustibleGasRaw = analogRead(MQ4_PIN);

  int soilRaw = analogRead(SOIL_PIN);
  soilMoisturePct = 100.0 - ((soilRaw / 4095.0) * 100.0);

  updateSpo2Estimate();
}

// Named per-sensor threshold checks (thresholds from the trained
// model) plus a joint tinyML anomaly check. Skipped once alarmActive
// is already set, same as the fall logic not re-triggering itself.
void checkEnvironmentalHazards() {
  if (alarmActive) return;

  float features[TINYML_N_FEATURES];
  float perFeatureError[TINYML_N_FEATURES];
  buildFeatureVector(features);
  lastAnomalyScore = tinyMLInfer(features, perFeatureError);

  if (!isnan(temperatureC) && temperatureC >= TEMP_DANGER_C) tempHazardSamples++; else tempHazardSamples = 0;
  if (tempHazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    tempHazardSamples = 0;
    alarmActive = true;
    sendDistressPacket("HEAT_STRESS", "HEAT_STRESS");
    return;
  }

  if (airQualityRaw >= MQ135_DANGER_RAW) mq135HazardSamples++; else mq135HazardSamples = 0;
  if (mq135HazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    mq135HazardSamples = 0;
    alarmActive = true;
    sendDistressPacket("GAS_DANGER", "TOXIC_GAS");
    return;
  }

  if (combustibleGasRaw >= MQ4_DANGER_RAW) mq4HazardSamples++; else mq4HazardSamples = 0;
  if (mq4HazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    mq4HazardSamples = 0;
    alarmActive = true;
    sendDistressPacket("GAS_DANGER", "COMBUSTIBLE_GAS");
    return;
  }

  if (heartRateBpm > 0 && heartRateBpm >= HR_DANGER_HIGH_BPM) hrHighHazardSamples++; else hrHighHazardSamples = 0;
  if (hrHighHazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    hrHighHazardSamples = 0;
    alarmActive = true;
    sendDistressPacket("HIGH_HEART_RATE", "HIGH_HEART_RATE");
    return;
  }

  if (heartRateBpm > 0 && heartRateBpm <= HR_DANGER_LOW_BPM) hrLowHazardSamples++; else hrLowHazardSamples = 0;
  if (hrLowHazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    hrLowHazardSamples = 0;
    alarmActive = true;
    sendDistressPacket("LOW_HEART_RATE", "LOW_HEART_RATE");
    return;
  }

  if (spo2Pct > 0 && spo2Pct <= SPO2_DANGER_LOW_PCT) spo2HazardSamples++; else spo2HazardSamples = 0;
  if (spo2HazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    spo2HazardSamples = 0;
    alarmActive = true;
    sendDistressPacket("LOW_SPO2", "LOW_SPO2");
    return;
  }

  if (soilMoisturePct >= SOIL_MOISTURE_DANGER_PCT) soilHazardSamples++; else soilHazardSamples = 0;
  if (soilHazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    soilHazardSamples = 0;
    alarmActive = true;
    sendDistressPacket("WATER_INGRESS", "WATER_INGRESS");
    return;
  }

  if (lastAnomalyScore >= TINYML_ANOMALY_THRESHOLD) mlAnomalyHazardSamples++; else mlAnomalyHazardSamples = 0;
  if (mlAnomalyHazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    mlAnomalyHazardSamples = 0;
    int worstIdx = 0;
    for (int i = 1; i < TINYML_N_FEATURES; i++) {
      if (perFeatureError[i] > perFeatureError[worstIdx]) worstIdx = i;
    }
    static const char* FEATURE_LABELS[TINYML_N_FEATURES] = {
      "TEMP", "HUMIDITY", "AIR_QUALITY", "COMBUSTIBLE_GAS",
      "HEART_RATE", "SPO2", "SOIL_MOISTURE", "MOTION"
    };
    char hazardType[16];
    snprintf(hazardType, sizeof(hazardType), "ML_%s", FEATURE_LABELS[worstIdx]);
    alarmActive = true;
    sendDistressPacket("ML_ANOMALY", hazardType);
  }
}

// ============================================================
// PRINT STATUS
// ============================================================
void printStatus() {
  Serial.println();
  Serial.println("==========================================");

  Serial.print("[MPU] Acceleration = ");
  Serial.print(acceleration, 2);
  Serial.println(" m/s2");

  Serial.print("[MPU6050] Accel(m/s^2) X="); Serial.print(accelX, 2);
  Serial.print(" Y="); Serial.print(accelY, 2); Serial.print(" Z="); Serial.print(accelZ, 2);
  Serial.print(" | Gyro(rad/s) X="); Serial.print(gyroX, 2);
  Serial.print(" Y="); Serial.print(gyroY, 2); Serial.print(" Z="); Serial.println(gyroZ, 2);

  Serial.print("[MPU] Fall = ");
  Serial.println(fallConfirmed ? "YES" : "NO");

  Serial.print("[SOS] Button = ");
  Serial.println(sosPressed ? "PRESSED" : "RELEASED");

  Serial.print("[ALARM] State = ");
  if (alarmActive) {
    Serial.print("ALARM (");
    Serial.print(currentRiskState);
    if (currentHazardType[0] != '\0' && strcmp(currentHazardType, currentRiskState) != 0) {
      Serial.print("/");
      Serial.print(currentHazardType);
    }
    Serial.println(")");
  } else {
    Serial.println("NORMAL");
  }

  Serial.println();

  Serial.print("[ENV] Temperature = ");
  Serial.print(temperatureC, 1);
  Serial.println(" C");

  Serial.print("[ENV] Humidity = ");
  Serial.print(humidityPct, 1);
  Serial.println(" %");

  Serial.print("[ENV] MQ135 = ");
  Serial.println(airQualityRaw, 0);

  Serial.print("[ENV] MQ4 = ");
  Serial.println(combustibleGasRaw, 0);

  Serial.print("[ENV] Soil = ");
  Serial.println(soilMoisturePct, 0);

  Serial.print("[ENV] ML anomaly score = ");
  Serial.print(lastAnomalyScore, 5);
  Serial.print(" (threshold = ");
  Serial.print(TINYML_ANOMALY_THRESHOLD, 5);
  Serial.println(")");

  Serial.println();

  Serial.print("[MAX30102] Finger = ");
  Serial.println(fingerDetected ? "DETECTED" : "NOT DETECTED");

  Serial.print("[MAX30102] Heart Rate = ");
  Serial.print(heartRateBpm, 0);
  Serial.print(" bpm");
  Serial.println(usingDummyVitals ? "  (DEMO FALLBACK -- no real pulse detected yet)" : "  (real detected value)");

  Serial.print("[MAX30102] SpO2 = ");
  Serial.print(spo2Pct, 0);
  Serial.print(" %");
  Serial.println(usingDummyVitals ? "  (DEMO FALLBACK)" : "  (real detected value)");

  Serial.print("[MAX30102] found="); Serial.print(max30102Found ? "YES" : "NO");
  Serial.print(" | IR="); Serial.print(lastIrValue);
  Serial.print(" (finger threshold="); Serial.print(IR_FINGER_THRESHOLD);
  Serial.println(")");

  Serial.println("==========================================");
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("==========================================");
  Serial.println("       WORKER SAFETY -- worker_n.ino");
  Serial.println("==========================================");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  // BUTTON: GPIO15 ---- BUTTON ---- 3.3V. INPUT_PULLDOWN: released =
  // LOW, pressed = HIGH.
  pinMode(SOS_PIN, INPUT_PULLDOWN);

  pinMode(MQ135_PIN, INPUT);
  pinMode(MQ4_PIN, INPUT);
  pinMode(SOIL_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("[INIT] Starting MPU6050...");
  mpuFound = mpu.begin();
  if (mpuFound) {
    Serial.println("[INIT] MPU6050 = OK");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    sensors_event_t accel, gyro, temperature;
    mpu.getEvent(&accel, &gyro, &temperature);
    previousX = accel.acceleration.x;
    previousY = accel.acceleration.y;
    previousZ = accel.acceleration.z;
  } else {
    Serial.println("[ERROR] MPU6050 NOT FOUND");
  }

  dht.begin();
  Serial.println("[INIT] DHT11 = STARTED");

  max30102Found = particleSensor.begin(Wire, I2C_SPEED_FAST);
  if (max30102Found) {
    particleSensor.setup(60, 4, 2, 100, 411, 4096);
    Serial.println("[INIT] MAX30102 = OK");
  } else {
    Serial.println("[WARN] MAX30102 NOT FOUND -- heart rate/SpO2 disabled");
  }

  // ---- HARDWARE TEST ----
  Serial.println();
  Serial.println("==========================================");
  Serial.println("           HARDWARE TEST");
  Serial.println("==========================================");

  Serial.println("[TEST] LED ON");
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  Serial.println("[TEST] LED OFF");

  Serial.println("[TEST] BUZZER ON");
  tone(BUZZER_PIN, 2000);
  delay(1000);
  noTone(BUZZER_PIN);
  Serial.println("[TEST] BUZZER OFF");

  Serial.println();
  Serial.println("[TEST] SOS GPIO15");
  Serial.print("[TEST] GPIO15 initial state = ");
  Serial.println(digitalRead(SOS_PIN));

  // ---- RADIO -- ESP-NOW ONLY ----
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_err_t channelResult = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.print(channelResult == ESP_OK ? "Radio channel set to " : "[ERROR] esp_wifi_set_channel failed: ");
  Serial.println(channelResult == ESP_OK ? WIFI_CHANNEL : channelResult);

  Serial.print("Worker MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW INIT FAILED");
    while (true) delay(1000);
  }
  Serial.println("ESP-NOW initialized");
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, WALL_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  Serial.println(esp_now_add_peer(&peerInfo) == ESP_OK ? "Wall peer added" : "ERROR: FAILED TO ADD WALL PEER");

  // If this fires, nothing below matters yet: esp_now_send() will
  // never reach wall_n.ino with an all-zero destination MAC, and the
  // onDataSent() callback above will never print anything at all --
  // not "SEND FAILED", nothing -- because esp_now_send() rejects the
  // packet immediately and synchronously before the radio ever
  // attempts a transmission. Fix WALL_MAC (see the constant near the
  // top of this file) before expecting any ESP-NOW output here.
  bool wallMacIsPlaceholder = true;
  for (int i = 0; i < 6; i++) if (WALL_MAC[i] != 0x00) wallMacIsPlaceholder = false;
  if (wallMacIsPlaceholder) {
    Serial.println();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("!!! WALL_MAC IS STILL 00:00:00:00:00:00 (THE PLACEHOLDER VALUE) !!!");
    Serial.println("!!! esp_now_send() will be REJECTED immediately, every time, and !!!");
    Serial.println("!!! onDataSent() below will NEVER print anything as a result.    !!!");
    Serial.println("!!! Fix: copy wall_n.ino's boot-log MAC into WALL_MAC above and  !!!");
    Serial.println("!!! reflash this board.                                         !!!");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println();
  }

  lastStatusTime = millis();

  Serial.println();
  Serial.println("==========================================");
  Serial.println("             SYSTEM READY");
  Serial.println("==========================================");
  Serial.println("SOS GPIO       = 15");
  Serial.println("LED GPIO       = 2");
  Serial.println("BUZZER GPIO    = 4");
  Serial.println("MPU SDA        = 21");
  Serial.println("MPU SCL        = 22");
  Serial.println();
  Serial.print("Free-fall threshold = "); Serial.print(FREE_FALL_THRESHOLD, 1); Serial.println(" m/s2");
  Serial.print("Impact threshold    = "); Serial.print(IMPACT_THRESHOLD, 1); Serial.println(" m/s2");
  Serial.print("Impact window       = "); Serial.print(IMPACT_WINDOW_MS / 1000); Serial.println(" s");
  Serial.print("Fall confirmation   = "); Serial.print(FALL_CONFIRMATION_TIME_MS / 1000); Serial.println(" s (unconditional)");
  Serial.print("Sensor status       = "); Serial.print(STATUS_INTERVAL_MS / 1000); Serial.println(" s");
  Serial.println();
  Serial.println("Environmental/vitals thresholds (tinyML-derived):");
  Serial.print("  Heat stress >= "); Serial.print(TEMP_DANGER_C, 1); Serial.println(" C");
  Serial.print("  Toxic gas (MQ-135) >= "); Serial.println(MQ135_DANGER_RAW, 0);
  Serial.print("  Combustible gas (MQ-4) >= "); Serial.println(MQ4_DANGER_RAW, 0);
  Serial.print("  Heart rate danger range: "); Serial.print(HR_DANGER_LOW_BPM, 0);
  Serial.print(" - "); Serial.print(HR_DANGER_HIGH_BPM, 0); Serial.println(" bpm");
  Serial.print("  SpO2 <= "); Serial.print(SPO2_DANGER_LOW_PCT, 0); Serial.println("%");
  Serial.print("  Water ingress (soil moisture) >= "); Serial.print(SOIL_MOISTURE_DANGER_PCT, 0); Serial.println("%");
  Serial.println("==========================================");
  Serial.println();
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  readMPU();
  detectFall();
  checkSOS();
  updateHeartRate();
  updateAlarm();

  unsigned long now = millis();
  if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
    lastStatusTime = now;

    readOtherSensors();
    checkEnvironmentalHazards();

    if (!alarmActive) sendStatusPacket();
    printStatus();
  }
}
