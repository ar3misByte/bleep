#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ============================================================
// SENSORA - WORKER ESP32
// MPU6050 + FALL DETECTION + ESP-NOW ONLY -- no WiFi/internet
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


// -------------------- ALERT OUTPUTS --------------------------

const int LED_PIN = 2;
const int BUZZER_PIN = 4;


// -------------------- SAMPLING -------------------------------

// MPU is checked continuously at approximately 10 Hz
const unsigned long SENSOR_INTERVAL_MS = 100;


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

  uint32_t seq;

} WorkerPacket;


WorkerPacket packet;


// ============================================================
// MPU6050
// ============================================================

Adafruit_MPU6050 mpu;


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
  Serial.println("       OUTGOING SENSORA PACKET");
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

  Serial.print("seq: ");
  Serial.println(packet.seq);

  Serial.println("========================================");
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
// MANUAL SOS (button press during normal monitoring)
// ============================================================
//
// Reuses the same "latched alarm" behavior as an automatic fall
// (state = FALL_CONFIRMED, LED/buzzer on, routine STATUS paused) but
// with riskState/hazardType marked "SOS" instead of "FALL_SUSPECTED"
// so the dashboard can tell a worker-triggered SOS apart from an
// automatically detected fall -- they're both genuine distress, but
// not the same event.

void triggerSOS() {

  state = FALL_CONFIRMED;

  motionEnergyAtTrigger = motionEnergy;

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);

  Serial.println();
  Serial.println();
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.println("        !!! SOS BUTTON TRIGGERED !!!");
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
  strncpy(packet.riskState, "SOS", sizeof(packet.riskState) - 1);
  strncpy(packet.hazardType, "SOS_BUTTON", sizeof(packet.hazardType) - 1);
  packet.motionEnergy = motionEnergy;
  packet.secondsSinceMotion = (millis() - lastMotionTime) / 1000.0;
  packet.motionEnergyAtTrigger = motionEnergyAtTrigger;
  packet.seq = sequenceNumber++;

  Serial.println(">>> IMMEDIATE SOS PACKET");
  printPacket();

  esp_err_t result = esp_now_send(WALL_MAC, (uint8_t*)&packet, sizeof(packet));
  if (result == ESP_OK) {
    Serial.println(">>> SOS PACKET HANDED TO ESP-NOW");
  } else {
    Serial.print(">>> SOS SEND ERROR: ");
    Serial.println(result);
  }

  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
}


// ============================================================
// PROLONGED INACTIVITY (no impact required)
// ============================================================
//
// Same latched-alarm behavior as triggerFall(), but riskState/
// hazardType marked "INACTIVITY" -- this fires purely from the
// worker not moving for FALL_CONFIRMATION_TIME_MS, with no impact
// ever detected, so the dashboard can tell it apart from an actual
// impact-based fall.

void triggerInactivityAlert() {

  state = FALL_CONFIRMED;

  motionEnergyAtTrigger = motionEnergy;

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);

  Serial.println();
  Serial.println();
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.println("     !!! PROLONGED INACTIVITY !!!");
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
  strncpy(packet.riskState, "INACTIVITY", sizeof(packet.riskState) - 1);
  strncpy(packet.hazardType, "INACTIVITY", sizeof(packet.hazardType) - 1);
  packet.motionEnergy = motionEnergy;
  packet.secondsSinceMotion = (millis() - lastMotionTime) / 1000.0;
  packet.motionEnergyAtTrigger = motionEnergyAtTrigger;
  packet.seq = sequenceNumber++;

  Serial.println(">>> IMMEDIATE INACTIVITY PACKET");
  printPacket();

  esp_err_t result = esp_now_send(WALL_MAC, (uint8_t*)&packet, sizeof(packet));
  if (result == ESP_OK) {
    Serial.println(">>> INACTIVITY PACKET HANDED TO ESP-NOW");
  } else {
    Serial.print(">>> INACTIVITY SEND ERROR: ");
    Serial.println(result);
  }

  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
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
  Serial.println("       SENSORA WORKER ESP32");
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


  Serial.println("----------------------------------------");

  Serial.println(
    "SENSORA WORKER READY"
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


      if (
        accelerationMagnitude >=
        IMPACT_THRESHOLD
      ) {

        impactSamples++;

      } else {

        impactSamples = 0;
      }


      if (
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
