#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <MAX30105.h>
#include "heartRate.h"

// ============================================================
// BLEEP - WORKER ESP32
// MPU6050 fall detection + DHT11/BMP180/MQ-135/MQ-4/MAX30102/soil
// moisture hazard sensing, all over ESP-NOW only -- no WiFi/internet
// ESP32 Arduino Core 3.x compatible (wifi_tx_info_t send callback)
// ============================================================

// -------------------- RADIO CHANNEL ----------------------------
//
// This board never joins any network and never gets an IP -- it
// talks ESP-NOW directly to WALL_MAC below, nothing else. ESP-NOW
// only reaches devices on the same WiFi CHANNEL, though, so this
// still has to match wherever the wall node's radio actually is.
//
// In the current peer-to-peer setup (wall_node.ino), the wall node
// hosts its own SoftAP and its WIFI_CHANNEL constant is what decides
// the channel outright -- there's no router assigning it anymore.
// Keep this number identical to that file's WIFI_CHANNEL; if you
// ever change one, change the other and reflash both.
const int WIFI_CHANNEL = 6;

// -------------------- WORKER SETTINGS -----------------------

const char WORKER_ID[] = "W1";

// Wall ESP32 MAC — read from that board's own "Wall MAC Address:"
// boot log. Sending directly to it (instead of broadcasting) means
// only this specific wall node receives it.
const uint8_t WALL_MAC[] = {
  0x68, 0x25, 0xDD, 0x31, 0xB4, 0x60
};


// -------------------- SOS BUTTON -----------------------------
//
// Wire a momentary pushbutton between this pin and GND -- the
// internal pull-up means the pin reads HIGH when not pressed and
// LOW when pressed, no external resistor needed. Two behaviors on
// one button: pressed during normal monitoring, it sends an
// immediate manual SOS distress packet; pressed while FALL_CONFIRMED
// is latched, it acknowledges/clears the alarm and sends an updated
// STATUS right away.
const int SOS_BUTTON_PIN = 15;
const unsigned long BUTTON_DEBOUNCE_MS = 50;

// -------------------- MPU6050 -------------------------------

const int SDA_PIN = 21;
const int SCL_PIN = 22;

// BMP180 and MAX30102 share the same I2C bus as the MPU6050 above --
// all three sit at different addresses (0x68, 0x77, 0x57) so no
// conflict, just three sensors_event_t sources on one Wire bus.


// -------------------- DHT11 (TEMP / HUMIDITY) -----------------

const int DHT_PIN = 27;
#define DHT_TYPE DHT11


// -------------------- ANALOG GAS / MOISTURE SENSORS -----------
//
// All three are plain analogRead() on ADC1 pins -- ADC2 shares
// hardware with WiFi and reads garbage while the radio is active,
// so these must stay off GPIO0/2/4/12-15/25-27.
const int MQ135_PIN = 34;          // toxic/air-quality gas, raw ADC 0-4095
const int MQ4_PIN = 35;            // combustible gas (methane/LPG), raw ADC 0-4095
const int SOIL_MOISTURE_PIN = 32;  // water ingress / flooding at floor level


// -------------------- ALERT OUTPUTS --------------------------

const int LED_PIN = 2;
const int BUZZER_PIN = 4;


// -------------------- SAMPLING -------------------------------

// MPU is checked continuously at approximately 10 Hz
const unsigned long SENSOR_INTERVAL_MS = 100;

// DHT11 and BMP180 are read far slower than the IMU -- the DHT11
// datasheet caps reliable reads at ~1 Hz, and there's no reason to
// hammer the gas/soil ADCs faster than that either.
const unsigned long ENV_SENSOR_INTERVAL_MS = 1000;


// -------------------- STATUS TRANSMISSION --------------------

// 2 seconds -- matches the original protocol recommendation
// (wifi_json_protocol.md) so the dashboard's motion/RSSI/inactivity
// readings feel live rather than updating in 10s jumps. Comfortably
// under the dashboard's 15s offline-staleness threshold either way.
const unsigned long STATUS_INTERVAL_MS = 2000;


// ============================================================
// FALL DETECTION PARAMETERS
// ============================================================

// Strong acceleration required to create a fall candidate.
// Starting value only — tune using real testing.
const float IMPACT_THRESHOLD = 18.0;


// Movement threshold during the 20-second confirmation.
// If worker moves above this level, the possible fall is cancelled.
const float LOW_MOTION_THRESHOLD = 4.0;


// Ignore movement immediately after impact.
const unsigned long SETTLING_TIME_MS = 2000;


// Worker gets 20 seconds to recover/move.
const unsigned long FALL_CONFIRMATION_TIME_MS = 20000;


// Require multiple impact samples to reduce noise.
const int REQUIRED_IMPACT_SAMPLES = 2;


// ============================================================
// ENVIRONMENTAL & VITALS HAZARD THRESHOLDS
// Starting values only -- tune using real testing, same caveat as
// the fall-detection thresholds above. Gas thresholds referenced
// against typical MQ-135/MQ-4 hobbyist-module ADC ranges (0-4095);
// gas sensors especially need on-site calibration against a known
// source, these are not calibrated ppm values.
// ============================================================

const float TEMP_DANGER_C = 40.0;              // heat stress
const int MQ135_DANGER_RAW = 4000;             // toxic/air-quality gas
const int MQ4_DANGER_RAW = 3000;               // combustible gas (methane/LPG) -- most safety-critical reading underground
const float HR_DANGER_HIGH_BPM = 130.0;
const float HR_DANGER_LOW_BPM = 45.0;
const float SPO2_DANGER_LOW_PCT = 90.0;
const float SOIL_MOISTURE_DANGER_PCT = 80.0;   // water ingress / flooding at floor level

// Consecutive breaching samples required before latching an alarm --
// same noise-rejection idea as REQUIRED_IMPACT_SAMPLES, checked at
// the 100ms sensor tick rate, so 5 samples is roughly half a second
// of sustained breach before anything latches.
const int REQUIRED_HAZARD_SAMPLES = 5;


// ============================================================
// DEBUG
// ============================================================

// Print sensor status once per second.
// This does NOT affect the STATUS transmission interval above.
const unsigned long DEBUG_INTERVAL_MS = 1000;


// ============================================================
// ESP-NOW PACKET
// MUST MATCH WALL NODE EXACTLY
// ============================================================

typedef struct __attribute__((packed)) {

  char workerId[8];

  uint8_t msgType;

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

  uint32_t seq;

} WorkerPacket;


WorkerPacket packet;


// ============================================================
// MPU6050
// ============================================================

Adafruit_MPU6050 mpu;


// ============================================================
// DHT11 / BMP180 / MAX30102
// ============================================================

DHT dht(DHT_PIN, DHT_TYPE);

Adafruit_BMP085 bmp;
bool bmpFound = false;

MAX30105 particleSensor;
bool max30102Found = false;


// ============================================================
// ENVIRONMENTAL & VITALS READINGS
// 0 / NAN means "no reading yet" for a given sensor, never a real
// measurement -- hazard checks below treat those as "not available"
// rather than a dangerous value.
// ============================================================

float temperatureC = NAN;
float humidityPct = NAN;
float pressureHPa = NAN;
float airQualityRaw = 0;
float combustibleGasRaw = 0;
float heartRateBpm = 0;
float spo2Pct = 0;
float soilMoisturePct = 0;

unsigned long lastEnvSensorTime = 0;

// Hazard debounce counters, one per environmental/vitals condition --
// mirrors impactSamples' role for the fall path.
int tempHazardSamples = 0;
int mq135HazardSamples = 0;
int mq4HazardSamples = 0;
int hrHighHazardSamples = 0;
int hrLowHazardSamples = 0;
int spo2HazardSamples = 0;
int soilHazardSamples = 0;

// Heart-rate beat detection (SparkFun checkForBeat() pattern):
// average the last few beat-to-beat intervals rather than trusting
// any single one, which is far too noisy on its own.
const byte RATE_ARRAY_SIZE = 4;
long rateArray[RATE_ARRAY_SIZE];
byte rateSpot = 0;
unsigned long lastBeatMs = 0;

// SpO2 is estimated from the ratio of AC/DC swing between the red and
// IR channels over a rolling ~1s window -- a widely used approximation,
// not a calibrated medical reading, but good enough for a hazard
// threshold. Reset every ENV_SENSOR_INTERVAL_MS in updateSpo2Estimate().
long spo2IrMin = 999999, spo2IrMax = 0;
long spo2RedMin = 999999, spo2RedMax = 0;


// ============================================================
// SENSOR VARIABLES
// ============================================================

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

float accelerationMagnitude = 0.0;

float previousX = 0.0;
float previousY = 0.0;
float previousZ = 0.0;

float motionEnergy = 0.0;


// ============================================================
// FALL STATE MACHINE
// ============================================================

enum FallState {

  NORMAL,
  SETTLING,
  CONFIRMING,
  FALL_CONFIRMED

};

FallState state = NORMAL;


// ============================================================
// TIMERS
// ============================================================

unsigned long lastSensorTime = 0;

unsigned long lastStatusTime = 0;

unsigned long lastDebugTime = 0;

unsigned long impactTime = 0;

unsigned long confirmationStartTime = 0;

// Time of the last sample where the worker was actually moving
// (motionEnergy above LOW_MOTION_THRESHOLD) -- this, not impactTime,
// is what "seconds since motion" should be measured from. impactTime
// only updates on a hard jolt (>= IMPACT_THRESHOLD), so using it for
// routine inactivity reporting meant the number climbed forever
// during ordinary handling, never resetting on real movement.
unsigned long lastMotionTime = 0;


// ============================================================
// IMPACT VALIDATION
// ============================================================

int impactSamples = 0;


// ============================================================
// SOS BUTTON STATE
// ============================================================

bool buttonLastRaw = HIGH;
bool buttonStable = HIGH;
unsigned long buttonLastChangeMs = 0;


// ============================================================
// FALL DATA
// ============================================================

float motionEnergyAtTrigger = 0.0;


// ============================================================
// PACKET SEQUENCE
// ============================================================

// Starts at 0 and increments for every packet.
// It does not reset while the ESP32 remains running.
uint32_t sequenceNumber = 0;


// ============================================================
// ESP-NOW SEND CALLBACK
// Compatible with ESP32 Arduino Core 3.x
// ============================================================

void onDataSent(
  const wifi_tx_info_t *info,
  esp_now_send_status_t status
) {

  Serial.print("[ESP-NOW] ");

  if (status == ESP_NOW_SEND_SUCCESS) {

    Serial.println("SEND SUCCESS");

  } else {

    Serial.println("SEND FAILED");
  }
}


// ============================================================
// PRINT PACKET
// ============================================================

void printPacket() {

  Serial.println();
  Serial.println("========================================");
  Serial.println("       OUTGOING BLEEP PACKET");
  Serial.println("========================================");

  Serial.print("workerId: ");
  Serial.println(packet.workerId);

  Serial.print("msgType: ");
  Serial.println(packet.msgType);

  Serial.print("riskState: ");
  Serial.println(packet.riskState);

  Serial.print("hazardType: ");
  Serial.println(packet.hazardType);

  Serial.print("motionEnergy: ");
  Serial.println(packet.motionEnergy, 2);

  Serial.print("secondsSinceMotion: ");
  Serial.println(packet.secondsSinceMotion, 2);

  Serial.print("motionEnergyAtTrigger: ");
  Serial.println(packet.motionEnergyAtTrigger, 2);

  Serial.print("temperatureC: ");
  Serial.println(packet.temperatureC, 2);

  Serial.print("humidityPct: ");
  Serial.println(packet.humidityPct, 2);

  Serial.print("pressureHPa: ");
  Serial.println(packet.pressureHPa, 2);

  Serial.print("airQualityRaw (MQ-135): ");
  Serial.println(packet.airQualityRaw, 0);

  Serial.print("combustibleGasRaw (MQ-4): ");
  Serial.println(packet.combustibleGasRaw, 0);

  Serial.print("heartRateBpm: ");
  Serial.println(packet.heartRateBpm, 1);

  Serial.print("spo2Pct: ");
  Serial.println(packet.spo2Pct, 1);

  Serial.print("soilMoisturePct: ");
  Serial.println(packet.soilMoisturePct, 1);

  Serial.print("seq: ");
  Serial.println(packet.seq);

  Serial.println("========================================");
}


// ============================================================
// ENVIRONMENTAL & VITALS SENSING
// ============================================================

// Copies the latest environmental/vitals readings into an outgoing
// packet -- called from every send path (STATUS, fall DISTRESS, and
// every triggerHazardAlert() distress) so the dashboard always has
// live gauge data regardless of which packet carried it.
void fillEnvironmentalFields(WorkerPacket &p) {
  p.temperatureC = temperatureC;
  p.humidityPct = humidityPct;
  p.pressureHPa = pressureHPa;
  p.airQualityRaw = airQualityRaw;
  p.combustibleGasRaw = combustibleGasRaw;
  p.heartRateBpm = heartRateBpm;
  p.spo2Pct = spo2Pct;
  p.soilMoisturePct = soilMoisturePct;
}

// SpO2 estimate from the ratio of AC/DC swing between the red and IR
// channels accumulated by updateHeartRate() over the last ~1s window.
// See the spo2Ir*/spo2Red* comment above for what this is and isn't.
void updateSpo2Estimate() {
  if (!max30102Found) {
    spo2Pct = 0;
    return;
  }

  long irAc = spo2IrMax - spo2IrMin;
  long redAc = spo2RedMax - spo2RedMin;
  long irDc = (spo2IrMax + spo2IrMin) / 2;
  long redDc = (spo2RedMax + spo2RedMin) / 2;

  if (irDc <= 0 || redDc <= 0 || irAc <= 0 || heartRateBpm <= 0) {
    // No usable finger/skin contact this window -- 0 means "no
    // reading," same convention as heartRateBpm.
    spo2Pct = 0;
  } else {
    float ratioOfRatios = ((float)redAc / redDc) / ((float)irAc / irDc);
    float estimate = 110.0 - (25.0 * ratioOfRatios);
    spo2Pct = constrain(estimate, 70.0, 100.0);
  }

  spo2IrMin = 999999; spo2IrMax = 0;
  spo2RedMin = 999999; spo2RedMax = 0;
}

// Called every loop() iteration (not gated by an interval) so beat
// timing stays accurate -- this mirrors pollButton()'s reasoning.
void updateHeartRate() {
  if (!max30102Found) return;

  long ir = particleSensor.getIR();

  if (ir < 50000) {
    // No finger/skin contact detected -- zero out so hazard checks
    // treat this as "no reading," not "flatline."
    heartRateBpm = 0;
    return;
  }

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
    if (bpm > 20 && bpm < 255) {
      rateArray[rateSpot++] = (long)bpm;
      rateSpot %= RATE_ARRAY_SIZE;

      long total = 0;
      for (byte x = 0; x < RATE_ARRAY_SIZE; x++) total += rateArray[x];
      heartRateBpm = total / (float)RATE_ARRAY_SIZE;
    }
  }
}

// Called on ENV_SENSOR_INTERVAL_MS -- DHT11 and the gas/soil ADCs
// don't need (and shouldn't be read at) the IMU's 10 Hz rate.
void readEnvironmentalSensors() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h) && !isnan(t)) {
    humidityPct = h;
    temperatureC = t;
  } // else: keep the last good reading -- DHT11 read failures are common and shouldn't blank the value out

  if (bmpFound) {
    pressureHPa = bmp.readPressure() / 100.0;
  }

  airQualityRaw = analogRead(MQ135_PIN);
  combustibleGasRaw = analogRead(MQ4_PIN);

  // Inverted so a rising percentage always means "wetter" -- most
  // resistive soil moisture modules read HIGH-ish when dry and drop
  // as moisture increases; flip this if your specific module wires
  // it the other way.
  int soilRaw = analogRead(SOIL_MOISTURE_PIN);
  soilMoisturePct = 100.0 - ((soilRaw / 4095.0) * 100.0);

  updateSpo2Estimate();
}

// Only evaluated while state == NORMAL, same as the prolonged
// inactivity check -- an environmental/vitals hazard shouldn't
// interrupt an already-latched or already-confirming fall sequence.
void checkEnvironmentalHazards() {

  if (!isnan(temperatureC) && temperatureC >= TEMP_DANGER_C) {
    tempHazardSamples++;
  } else {
    tempHazardSamples = 0;
  }
  if (tempHazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    tempHazardSamples = 0;
    triggerHazardAlert("HEAT_STRESS", "HEAT_STRESS", "HEAT STRESS DETECTED");
    return;
  }

  if (airQualityRaw >= MQ135_DANGER_RAW) {
    mq135HazardSamples++;
  } else {
    mq135HazardSamples = 0;
  }
  if (mq135HazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    mq135HazardSamples = 0;
    triggerHazardAlert("GAS_DANGER", "TOXIC_GAS", "TOXIC GAS DETECTED");
    return;
  }

  if (combustibleGasRaw >= MQ4_DANGER_RAW) {
    mq4HazardSamples++;
  } else {
    mq4HazardSamples = 0;
  }
  if (mq4HazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    mq4HazardSamples = 0;
    triggerHazardAlert("GAS_DANGER", "COMBUSTIBLE_GAS", "COMBUSTIBLE GAS DETECTED");
    return;
  }

  if (heartRateBpm > 0 && heartRateBpm >= HR_DANGER_HIGH_BPM) {
    hrHighHazardSamples++;
  } else {
    hrHighHazardSamples = 0;
  }
  if (hrHighHazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    hrHighHazardSamples = 0;
    triggerHazardAlert("HIGH_HEART_RATE", "HIGH_HEART_RATE", "HIGH HEART RATE DETECTED");
    return;
  }

  if (heartRateBpm > 0 && heartRateBpm <= HR_DANGER_LOW_BPM) {
    hrLowHazardSamples++;
  } else {
    hrLowHazardSamples = 0;
  }
  if (hrLowHazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    hrLowHazardSamples = 0;
    triggerHazardAlert("LOW_HEART_RATE", "LOW_HEART_RATE", "LOW HEART RATE DETECTED");
    return;
  }

  if (spo2Pct > 0 && spo2Pct <= SPO2_DANGER_LOW_PCT) {
    spo2HazardSamples++;
  } else {
    spo2HazardSamples = 0;
  }
  if (spo2HazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    spo2HazardSamples = 0;
    triggerHazardAlert("LOW_SPO2", "LOW_SPO2", "LOW BLOOD OXYGEN DETECTED");
    return;
  }

  if (soilMoisturePct >= SOIL_MOISTURE_DANGER_PCT) {
    soilHazardSamples++;
  } else {
    soilHazardSamples = 0;
  }
  if (soilHazardSamples >= REQUIRED_HAZARD_SAMPLES) {
    soilHazardSamples = 0;
    triggerHazardAlert("WATER_INGRESS", "WATER_INGRESS", "WATER INGRESS DETECTED");
    return;
  }
}


// ============================================================
// SEND STATUS PACKET
// ============================================================

void sendStatusPacket() {

  memset(
    &packet,
    0,
    sizeof(packet)
  );


  strncpy(
    packet.workerId,
    WORKER_ID,
    sizeof(packet.workerId) - 1
  );


  packet.msgType = 0;


  strncpy(
    packet.riskState,
    "OK",
    sizeof(packet.riskState) - 1
  );


  packet.hazardType[0] = '\0';


  packet.motionEnergy =
    motionEnergy;


  // Time since the worker last actually moved -- not since the last
  // hard impact, which could be a long time ago or never.
  packet.secondsSinceMotion =
    (millis() - lastMotionTime) / 1000.0;


  packet.motionEnergyAtTrigger =
    0.0;


  fillEnvironmentalFields(packet);


  packet.seq =
    sequenceNumber++;


  Serial.println();
  Serial.println(">>> SENDING ROUTINE STATUS");


  printPacket();


  esp_err_t result =
    esp_now_send(
      WALL_MAC,
      (uint8_t *)&packet,
      sizeof(packet)
    );


  if (result != ESP_OK) {

    Serial.print(
      "[ESP-NOW] STATUS error: "
    );

    Serial.println(result);
  }
}


// ============================================================
// TRIGGER FALL
// ============================================================

void triggerFall() {

  if (
    state == FALL_CONFIRMED
  ) {

    return;
  }


  state = FALL_CONFIRMED;


  motionEnergyAtTrigger =
    motionEnergy;


  // ==========================================================
  // LOCAL ALERT
  // ==========================================================

  digitalWrite(
    LED_PIN,
    HIGH
  );

  digitalWrite(
    BUZZER_PIN,
    HIGH
  );


  Serial.println();
  Serial.println();
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.println("          !!! FALL CONFIRMED !!!");
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  Serial.println("LED    : ON");
  Serial.println("BUZZER : ON");
  Serial.print("LED_PIN readback: ");
  Serial.println(digitalRead(LED_PIN));
  Serial.print("BUZZER_PIN readback: ");
  Serial.println(digitalRead(BUZZER_PIN));


  // ==========================================================
  // BUILD DISTRESS PACKET
  // ==========================================================

  memset(
    &packet,
    0,
    sizeof(packet)
  );


  strncpy(
    packet.workerId,
    WORKER_ID,
    sizeof(packet.workerId) - 1
  );


  packet.msgType = 1;


  strncpy(
    packet.riskState,
    "FALL_SUSPECTED",
    sizeof(packet.riskState) - 1
  );


  strncpy(
    packet.hazardType,
    "FALL_SUSPECTED",
    sizeof(packet.hazardType) - 1
  );


  packet.motionEnergy =
    motionEnergy;


  packet.secondsSinceMotion =
    (millis() - lastMotionTime) / 1000.0;


  packet.motionEnergyAtTrigger =
    motionEnergyAtTrigger;


  fillEnvironmentalFields(packet);


  packet.seq =
    sequenceNumber++;


  // ==========================================================
  // PRINT BEFORE TRANSMISSION
  // ==========================================================

  Serial.println(
    ">>> IMMEDIATE DISTRESS PACKET"
  );

  printPacket();


  // ==========================================================
  // SEND IMMEDIATELY
  // ==========================================================

  esp_err_t result =
    esp_now_send(
      WALL_MAC,
      (uint8_t *)&packet,
      sizeof(packet)
    );


  if (
    result == ESP_OK
  ) {

    Serial.println(
      ">>> DISTRESS PACKET HANDED TO ESP-NOW"
    );

  } else {

    Serial.print(
      ">>> DISTRESS SEND ERROR: "
    );

    Serial.println(result);
  }


  Serial.println(
    "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
  );
}


// ============================================================
// GENERIC LATCHED HAZARD ALERT
// ============================================================
//
// Shared by every non-fall distress source (SOS button, prolonged
// inactivity, and the environmental/vitals hazards below): same
// latched-alarm behavior as triggerFall() (state = FALL_CONFIRMED,
// LED/buzzer on, routine STATUS paused, requires a button ack to
// clear) but with riskState/hazardType describing which condition
// fired, so the dashboard can tell every distress source apart --
// they're all genuine distress, but not the same event.

void triggerHazardAlert(const char* riskState, const char* hazardType, const char* bannerLabel) {

  state = FALL_CONFIRMED;

  motionEnergyAtTrigger = motionEnergy;

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);

  Serial.println();
  Serial.println();
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.print("        !!! ");
  Serial.print(bannerLabel);
  Serial.println(" !!!");
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.println("LED    : ON");
  Serial.println("BUZZER : ON");
  Serial.print("LED_PIN readback: ");
  Serial.println(digitalRead(LED_PIN));
  Serial.print("BUZZER_PIN readback: ");
  Serial.println(digitalRead(BUZZER_PIN));

  memset(&packet, 0, sizeof(packet));
  strncpy(packet.workerId, WORKER_ID, sizeof(packet.workerId) - 1);
  packet.msgType = 1;
  strncpy(packet.riskState, riskState, sizeof(packet.riskState) - 1);
  strncpy(packet.hazardType, hazardType, sizeof(packet.hazardType) - 1);
  packet.motionEnergy = motionEnergy;
  packet.secondsSinceMotion = (millis() - lastMotionTime) / 1000.0;
  packet.motionEnergyAtTrigger = motionEnergyAtTrigger;
  fillEnvironmentalFields(packet);
  packet.seq = sequenceNumber++;

  Serial.print(">>> IMMEDIATE ");
  Serial.print(bannerLabel);
  Serial.println(" PACKET");
  printPacket();

  esp_err_t result = esp_now_send(WALL_MAC, (uint8_t*)&packet, sizeof(packet));
  if (result == ESP_OK) {
    Serial.println(">>> PACKET HANDED TO ESP-NOW");
  } else {
    Serial.print(">>> SEND ERROR: ");
    Serial.println(result);
  }

  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
}

void triggerSOS() {
  triggerHazardAlert("SOS", "SOS_BUTTON", "SOS BUTTON TRIGGERED");
}

void triggerInactivityAlert() {
  triggerHazardAlert("INACTIVITY", "INACTIVITY", "PROLONGED INACTIVITY");
}


// ============================================================
// SOS BUTTON HANDLING
// ============================================================
//
// One button, two jobs depending on current state:
//  - NORMAL/SETTLING/CONFIRMING -> pressing it raises a manual SOS.
//  - FALL_CONFIRMED -> pressing it acknowledges/clears the alarm and
//    sends an updated STATUS immediately. Without this there is no
//    way to ever see an update after a fall: routine STATUS sending
//    is deliberately paused while FALL_CONFIRMED (see loop()), so
//    the dashboard would otherwise be stuck showing the original
//    distress packet forever.

void onButtonPressed() {
  Serial.println();
  Serial.println(">>> SOS BUTTON PRESSED");

  if (state == FALL_CONFIRMED) {
    Serial.println(">>> Acknowledging and clearing FALL_CONFIRMED");
    state = NORMAL;
    impactSamples = 0;
    impactTime = millis();
    confirmationStartTime = 0;
    // Clear every hazard debounce counter too -- otherwise an
    // acknowledged environmental alarm could immediately re-latch
    // from samples counted before the ack.
    tempHazardSamples = 0;
    mq135HazardSamples = 0;
    mq4HazardSamples = 0;
    hrHighHazardSamples = 0;
    hrLowHazardSamples = 0;
    spo2HazardSamples = 0;
    soilHazardSamples = 0;
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    sendStatusPacket(); // immediate update -- don't wait for the next STATUS_INTERVAL_MS tick
    return;
  }

  triggerSOS();
}

void pollButton() {
  unsigned long now = millis();
  bool raw = digitalRead(SOS_BUTTON_PIN);

  if (raw != buttonLastRaw) {
    buttonLastRaw = raw;
    buttonLastChangeMs = now;
  }

  if (now - buttonLastChangeMs >= BUTTON_DEBOUNCE_MS && buttonStable != buttonLastRaw) {
    buttonStable = buttonLastRaw;
    if (buttonStable == LOW) {
      onButtonPressed();
    }
  }
}


// ============================================================
// RESET POSSIBLE FALL
// ============================================================

void cancelFall() {

  state = NORMAL;

  impactSamples = 0;

  impactTime = 0;

  confirmationStartTime = 0;


  Serial.println();
  Serial.println(
    ">>> MOVEMENT DETECTED"
  );

  Serial.println(
    ">>> FALL CANDIDATE CANCELLED"
  );

  Serial.println(
    ">>> WORKER CONTINUED MOVING"
  );

  Serial.println(
    ">>> RETURNING TO NORMAL"
  );

  Serial.println();
}


// ============================================================
// READ MPU6050
// ============================================================

void readMPU() {

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temperature;


  mpu.getEvent(
    &accel,
    &gyro,
    &temperature
  );


  accelX =
    accel.acceleration.x;

  accelY =
    accel.acceleration.y;

  accelZ =
    accel.acceleration.z;


  // ==========================================================
  // TOTAL ACCELERATION
  // ==========================================================

  accelerationMagnitude =
    sqrt(

      accelX * accelX +
      accelY * accelY +
      accelZ * accelZ

    );


  // ==========================================================
  // MOTION ENERGY
  // ==========================================================

  float dx =
    accelX - previousX;

  float dy =
    accelY - previousY;

  float dz =
    accelZ - previousZ;


  float currentChange =
    sqrt(

      dx * dx +
      dy * dy +
      dz * dz

    );


  // Smooth the motion energy.
  motionEnergy =
    (motionEnergy * 0.8) +
    (currentChange * 0.2);


  previousX =
    accelX;

  previousY =
    accelY;

  previousZ =
    accelZ;
}


// ============================================================
// DEBUG STATUS
// ============================================================

void printDebug() {

  unsigned long now =
    millis();


  if (
    now - lastDebugTime <
    DEBUG_INTERVAL_MS
  ) {

    return;
  }


  lastDebugTime =
    now;


  Serial.print("[DATA] ");

  Serial.print(
    "A="
  );

  Serial.print(
    accelerationMagnitude,
    2
  );

  Serial.print(
    " m/s2"
  );


  Serial.print(
    " | Energy="
  );

  Serial.print(
    motionEnergy,
    2
  );


  Serial.print(
    " | State="
  );


  switch (state) {

    case NORMAL:
      Serial.println("NORMAL");
      break;

    case SETTLING:
      Serial.println("SETTLING");
      break;

    case CONFIRMING:
      Serial.println("CONFIRMING");
      break;

    case FALL_CONFIRMED:
      Serial.println("FALL_CONFIRMED");
      break;
  }

  Serial.print("[ENV]  T=");
  Serial.print(temperatureC, 1);
  Serial.print("C | H=");
  Serial.print(humidityPct, 1);
  Serial.print("% | P=");
  Serial.print(pressureHPa, 1);
  Serial.print("hPa | MQ135=");
  Serial.print(airQualityRaw, 0);
  Serial.print(" | MQ4=");
  Serial.print(combustibleGasRaw, 0);
  Serial.print(" | HR=");
  Serial.print(heartRateBpm, 0);
  Serial.print("bpm | SpO2=");
  Serial.print(spo2Pct, 0);
  Serial.print("% | Soil=");
  Serial.print(soilMoisturePct, 0);
  Serial.println("%");
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(500);


  // ==========================================================
  // ALERT OUTPUTS
  // ==========================================================

  pinMode(
    LED_PIN,
    OUTPUT
  );

  pinMode(
    BUZZER_PIN,
    OUTPUT
  );


  digitalWrite(
    LED_PIN,
    LOW
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );


  // ==========================================================
  // SOS BUTTON
  // ==========================================================

  pinMode(
    SOS_BUTTON_PIN,
    INPUT_PULLUP
  );

  Serial.print("SOS button initial read (should be HIGH if unpressed and wired correctly): ");
  Serial.println(digitalRead(SOS_BUTTON_PIN));


  // ==========================================================
  // I2C
  // ==========================================================

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );


  Serial.println();
  Serial.println("========================================");
  Serial.println("       BLEEP WORKER ESP32");
  Serial.println("========================================");


  // ==========================================================
  // MPU6050
  // ==========================================================

  Serial.println(
    "Initializing MPU6050..."
  );


  if (
    !mpu.begin()
  ) {

    Serial.println(
      "ERROR: MPU6050 NOT FOUND"
    );


    while (true) {

      digitalWrite(
        LED_PIN,
        HIGH
      );

      delay(200);

      digitalWrite(
        LED_PIN,
        LOW
      );

      delay(200);
    }
  }


  Serial.println(
    "MPU6050 FOUND"
  );


  mpu.setAccelerometerRange(
    MPU6050_RANGE_8_G
  );

  mpu.setGyroRange(
    MPU6050_RANGE_500_DEG
  );

  mpu.setFilterBandwidth(
    MPU6050_BAND_21_HZ
  );


  // ==========================================================
  // INITIAL MPU READING
  // ==========================================================

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temperature;


  mpu.getEvent(
    &accel,
    &gyro,
    &temperature
  );


  previousX =
    accel.acceleration.x;

  previousY =
    accel.acceleration.y;

  previousZ =
    accel.acceleration.z;


  // ==========================================================
  // DHT11 / BMP180 / MAX30102 / GAS & SOIL ADCs
  // ==========================================================
  //
  // Unlike the MPU6050 above, a missing environmental/vitals sensor
  // does not halt the board -- fall detection is the safety-critical
  // feature and must keep running even if, say, the MAX30102 isn't
  // wired up yet. Each sensor just reports 0/NaN ("no reading") if
  // it's not found, which the hazard checks already treat as
  // "not available."

  dht.begin();
  Serial.println("DHT11 initialized (temperature/humidity)");

  bmpFound = bmp.begin();
  Serial.println(bmpFound ? "BMP180 FOUND" : "[WARN] BMP180 NOT FOUND -- pressure readings disabled");

  max30102Found = particleSensor.begin(Wire, I2C_SPEED_FAST);
  if (max30102Found) {
    byte ledBrightness = 60;
    byte sampleAverage = 4;
    byte ledMode = 2;      // red + IR
    int sampleRate = 100;
    int pulseWidth = 411;
    int adcRange = 4096;
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    Serial.println("MAX30102 FOUND");
  } else {
    Serial.println("[WARN] MAX30102 NOT FOUND -- heart rate/SpO2 readings disabled");
  }

  Serial.println("MQ-135, MQ-4 and soil moisture read directly via analogRead(), no init needed");


  // ==========================================================
  // RADIO -- ESP-NOW ONLY, NO WIFI/INTERNET
  // ==========================================================
  //
  // WIFI_STA mode is required to use the radio at all (ESP-NOW
  // rides on the WiFi hardware), but WiFi.begin() is deliberately
  // never called -- this board never associates with a router,
  // never requests an IP, and never touches the internet. Instead
  // esp_wifi_set_channel() pins the radio to WIFI_CHANNEL directly
  // so it lands on the same channel as the wall node without ever
  // joining its network.

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // make sure no stale AP association survives a reset

  esp_err_t channelResult = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (channelResult == ESP_OK) {
    Serial.print("Radio channel set to ");
    Serial.println(WIFI_CHANNEL);
  } else {
    Serial.print("[ERROR] esp_wifi_set_channel failed: ");
    Serial.println(channelResult);
  }


  // ==========================================================
  // ESP-NOW
  // ==========================================================

  Serial.print(
    "Worker MAC: "
  );

  Serial.println(
    WiFi.macAddress()
  );


  if (
    esp_now_init() != ESP_OK
  ) {

    Serial.println(
      "ERROR: ESP-NOW INIT FAILED"
    );

    while (true) {

      delay(1000);
    }
  }


  Serial.println(
    "ESP-NOW initialized"
  );


  esp_now_register_send_cb(
    onDataSent
  );


  // ==========================================================
  // ADD WALL PEER
  // ==========================================================

  esp_now_peer_info_t peerInfo = {};


  memcpy(
    peerInfo.peer_addr,
    WALL_MAC,
    6
  );


  peerInfo.channel = 0;

  peerInfo.encrypt = false;


  if (
    esp_now_add_peer(
      &peerInfo
    ) != ESP_OK
  ) {

    Serial.println(
      "ERROR: FAILED TO ADD WALL PEER"
    );

  } else {

    Serial.println(
      "Wall peer added"
    );
  }


  // ==========================================================
  // START TIMERS
  // ==========================================================

  lastSensorTime =
    millis();

  lastStatusTime =
    millis();

  lastDebugTime =
    millis();

  impactTime =
    millis();

  lastMotionTime =
    millis();

  lastEnvSensorTime =
    millis();


  // ==========================================================
  // READY
  // ==========================================================

  Serial.println();
  Serial.println("----------------------------------------");

  Serial.print(
    "Status interval: "
  );

  Serial.print(
    STATUS_INTERVAL_MS / 1000
  );

  Serial.println(
    " seconds"
  );


  Serial.print(
    "Impact threshold: "
  );

  Serial.print(
    IMPACT_THRESHOLD,
    2
  );

  Serial.println(
    " m/s2"
  );


  Serial.print(
    "Fall confirmation: "
  );

  Serial.print(
    FALL_CONFIRMATION_TIME_MS / 1000
  );

  Serial.println(
    " seconds"
  );

  Serial.print("Heat stress threshold: ");
  Serial.print(TEMP_DANGER_C, 1);
  Serial.println(" C");

  Serial.print("Toxic gas (MQ-135) threshold: ");
  Serial.println(MQ135_DANGER_RAW);

  Serial.print("Combustible gas (MQ-4) threshold: ");
  Serial.println(MQ4_DANGER_RAW);

  Serial.print("Heart rate danger range: ");
  Serial.print(HR_DANGER_LOW_BPM, 0);
  Serial.print(" - ");
  Serial.print(HR_DANGER_HIGH_BPM, 0);
  Serial.println(" bpm");

  Serial.print("SpO2 danger threshold: <= ");
  Serial.print(SPO2_DANGER_LOW_PCT, 0);
  Serial.println("%");

  Serial.print("Water ingress (soil moisture) threshold: >= ");
  Serial.print(SOIL_MOISTURE_DANGER_PCT, 0);
  Serial.println("%");


  Serial.println("----------------------------------------");

  Serial.println(
    "BLEEP WORKER READY"
  );

  Serial.println();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  unsigned long now =
    millis();


  // ==========================================================
  // 0. SOS BUTTON -- polled every loop iteration, not gated by
  //    SENSOR_INTERVAL_MS, so a press is caught promptly.
  // ==========================================================

  pollButton();


  // ==========================================================
  // 0b. HEART RATE BEAT DETECTION -- like the button, this needs
  //     every loop iteration for accurate beat timing, not just the
  //     100ms sensor tick.
  // ==========================================================

  updateHeartRate();


  // ==========================================================
  // 0c. ENVIRONMENTAL SENSORS (DHT11 / BMP180 / MQ-135 / MQ-4 /
  //     soil moisture) -- far slower than the IMU, own interval.
  // ==========================================================

  if (
    now - lastEnvSensorTime >=
    ENV_SENSOR_INTERVAL_MS
  ) {

    lastEnvSensorTime =
      now;

    readEnvironmentalSensors();
  }


  // ==========================================================
  // 1. CONTINUOUS MPU6050 MONITORING
  // ==========================================================

  if (
    now - lastSensorTime >=
    SENSOR_INTERVAL_MS
  ) {

    lastSensorTime =
      now;


    readMPU();


    // ========================================================
    // TRACK LAST REAL MOTION -- independent of state, this is
    // what "seconds since motion" is measured from everywhere else
    // in this file.
    // ========================================================

    if (
      motionEnergy >
      LOW_MOTION_THRESHOLD
    ) {

      lastMotionTime =
        now;
    }


    // ========================================================
    // NORMAL
    // ========================================================

    if (
      state == NORMAL
    ) {

      // Prolonged inactivity on its own -- no impact required.
      // This is the other half of "Fall & Inactivity Detection":
      // a worker who simply hasn't moved for the full confirmation
      // window is worth flagging even if nothing ever hit
      // IMPACT_THRESHOLD. Previously this file only ever checked
      // inactivity AFTER an impact, so standing still alone did
      // nothing at all.
      if (
        now - lastMotionTime >=
        FALL_CONFIRMATION_TIME_MS
      ) {

        Serial.println();
        Serial.println(
          ">>> PROLONGED INACTIVITY DETECTED (no impact)"
        );

        triggerInactivityAlert();
      }


      // Environmental/vitals hazards (gas, heat, heart rate, SpO2,
      // water ingress) -- immediate, no 20s confirmation delay like
      // a fall, because there's no "did the worker just move a lot"
      // ambiguity to rule out for these. Skipped if the inactivity
      // check just above already latched an alarm this tick.
      if (state == NORMAL) {
        checkEnvironmentalHazards();
      }


      // Guarded the same way -- checkEnvironmentalHazards() (or the
      // inactivity check above) may have just latched an alarm this
      // tick, and impact detection must not then overwrite that with
      // state = SETTLING.
      if (state == NORMAL && (
        accelerationMagnitude >=
        IMPACT_THRESHOLD
      )) {

        impactSamples++;

      } else if (state == NORMAL) {

        impactSamples = 0;
      }


      if (
        state == NORMAL &&
        impactSamples >=
        REQUIRED_IMPACT_SAMPLES
      ) {

        state =
          SETTLING;


        impactTime =
          now;


        Serial.println();
        Serial.println(
          ">>> POSSIBLE IMPACT DETECTED"
        );


        Serial.print(
          ">>> Acceleration: "
        );

        Serial.print(
          accelerationMagnitude,
          2
        );

        Serial.println(
          " m/s2"
        );


        Serial.println(
          ">>> Starting 2-second settling period..."
        );


        impactSamples = 0;
      }
    }


    // ========================================================
    // SETTLING
    // ========================================================

    if (
      state == SETTLING
    ) {

      if (
        now - impactTime >=
        SETTLING_TIME_MS
      ) {

        state =
          CONFIRMING;


        confirmationStartTime =
          now;


        Serial.println();
        Serial.println(
          ">>> SETTLING COMPLETE"
        );


        Serial.println(
          ">>> 20-SECOND FALL CONFIRMATION STARTED"
        );


        Serial.println(
          ">>> Worker movement will CANCEL the fall"
        );


        Serial.println();
      }
    }


    // ========================================================
    // 20-SECOND CONFIRMATION
    // ========================================================

    if (
      state == CONFIRMING
    ) {

      unsigned long inactiveTime =
        now - confirmationStartTime;


      // ------------------------------------------------------
      // MOVEMENT DETECTED
      // ------------------------------------------------------

      if (
        motionEnergy >
        LOW_MOTION_THRESHOLD
      ) {

        cancelFall();
      }


      // ------------------------------------------------------
      // STILL / LOW MOVEMENT
      // ------------------------------------------------------

      else {

        static unsigned long lastCountdown =
          0;


        if (
          now - lastCountdown >=
          1000
        ) {

          lastCountdown =
            now;


          Serial.print(
            "[FALL CHECK] "
          );

          Serial.print(
            inactiveTime / 1000
          );

          Serial.print(
            " / "
          );

          Serial.print(
            FALL_CONFIRMATION_TIME_MS / 1000
          );

          Serial.print(
            " sec | Energy="
          );

          Serial.println(
            motionEnergy,
            2
          );
        }


        // ----------------------------------------------------
        // FALL CONFIRMED
        // ----------------------------------------------------

        if (
          inactiveTime >=
          FALL_CONFIRMATION_TIME_MS
        ) {

          triggerFall();
        }
      }
    }


    // ========================================================
    // FALL CONFIRMED
    // ========================================================

    if (
      state == FALL_CONFIRMED
    ) {

      // Keep local alarm ON.

      digitalWrite(
        LED_PIN,
        HIGH
      );

      digitalWrite(
        BUZZER_PIN,
        HIGH
      );
    }
  }


  // ==========================================================
  // 2. LOW-FREQUENCY DEBUG OUTPUT
  // ==========================================================

  printDebug();


  // ==========================================================
  // 3. STATUS PACKET (every STATUS_INTERVAL_MS)
  // ==========================================================

  if (
    state != FALL_CONFIRMED &&
    now - lastStatusTime >=
    STATUS_INTERVAL_MS
  ) {

    lastStatusTime =
      now;


    sendStatusPacket();
  }
}
