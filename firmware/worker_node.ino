#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ============================================================
// SENSORA - WORKER ESP32
// MPU6050 + FALL DETECTION + ESP-NOW
// ESP32 Arduino Core 3.x compatible (wifi_tx_info_t send callback)
// ============================================================

// -------------------- WIFI ------------------------------------
//
// Must be the SAME network the wall node connects to in
// wall_node.ino — ESP-NOW only reaches devices on the same WiFi
// channel, and joining the router is the easiest way to guarantee
// that. This board never needs an IP of its own for anything; it
// only sends ESP-NOW unicasts to WALL_MAC below.

const char* WIFI_SSID     = "shitstorm";
const char* WIFI_PASSWORD = "boombox1";

// -------------------- WORKER SETTINGS -----------------------

const char WORKER_ID[] = "W1";

// Wall ESP32 MAC — read from that board's own "Wall MAC Address:"
// boot log. Sending directly to it (instead of broadcasting) means
// only this specific wall node receives it.
const uint8_t WALL_MAC[] = {
  0x68, 0x25, 0xDD, 0x31, 0xB4, 0x60
};


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

// 10 seconds. Stays comfortably under the dashboard's 15s
// offline-staleness threshold (see dashboard_features.md) — don't
// push this much higher without also raising that threshold, or a
// single missed send will show the worker as OFFLINE.
const unsigned long STATUS_INTERVAL_MS = 10000;


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


// ============================================================
// IMPACT VALIDATION
// ============================================================

int impactSamples = 0;


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


  // Time since the last detected impact event.
  packet.secondsSinceMotion =
    (millis() - impactTime) / 1000.0;


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
    (millis() - impactTime) / 1000.0;


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
  // WIFI
  // ==========================================================
  //
  // Joining the router (rather than just setting WIFI_STA mode)
  // is what locks this board onto the same channel the wall node
  // is on — without this, ESP-NOW frames sent to WALL_MAC may
  // never arrive if the two boards end up on different channels.

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi (to match the wall node's channel)");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
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
    // NORMAL
    // ========================================================

    if (
      state == NORMAL
    ) {

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
