#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <esp_now.h>

// ============================================================
// SENSORA WORKER NODE
// CONSERVATIVE FALL DETECTOR
// MPU6050 + LED + BUZZER + ESP-NOW
// ============================================================

// -------------------- WIFI / ESP-NOW ---------------
//
// Must join the SAME WiFi network as the wall node — ESP-NOW
// send only reaches devices on the same WiFi channel, and
// joining the router is the easiest way to guarantee that
// (the wall node does the same thing in wall_node.ino).
// This board never needs an IP of its own for anything; it
// only sends ESP-NOW broadcasts.

const char* WIFI_SSID     = "kkkk";
const char* WIFI_PASSWORD = "123456879";
const char* WORKER_ID     = "W1";

// How often to send a routine STATUS packet while nothing is
// wrong. Matches the wall node / dashboard's "every ~2s" protocol
// recommendation.
const unsigned long STATUS_INTERVAL_MS = 2000;

// -------------------- MPU6050 --------------------

const int SDA_PIN = 21;
const int SCL_PIN = 22;

// -------------------- ALERT OUTPUTS ----------------

const int LED_PIN = 2;
const int BUZZER_PIN = 4;

// -------------------- SAMPLING --------------------

const unsigned long SAMPLE_INTERVAL_MS = 100;

// -------------------- FALL PARAMETERS -------------
//
// These are STARTING values.
// They MUST be tuned using your actual MPU6050 readings.
//

// Strong acceleration required to create a fall candidate.
// 18 m/s² ≈ 1.84 g.
const float IMPACT_THRESHOLD = 18.0;

// Very low motion after the candidate is considered inactivity.
// This will be tuned after testing.
const float LOW_MOTION_THRESHOLD = 4.0;

// Ignore the immediate movement caused by impact.
const unsigned long SETTLING_TIME_MS = 2000;

// Worker gets 20 seconds to recover/move before alarm.
const unsigned long FALL_CONFIRMATION_TIME_MS = 20000;

// To avoid reacting to one noisy sample,
// impact must be observed more than once.
const int REQUIRED_IMPACT_SAMPLES = 2;

// -------------------- DEBUG ------------------------

const unsigned long DEBUG_INTERVAL_MS = 1000;


// ============================================================
// ESP-NOW WIRE FORMAT
// ============================================================
//
// Must match the wall node's struct EXACTLY — field order, types
// and sizes are the ESP-NOW wire format. If you change one side,
// change the other. See firmware/wall_node.ino in this repo.

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

// Broadcast address — reaches any ESP32 in range on this WiFi
// channel without needing to know the wall node's exact MAC.
uint8_t wallNodeAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

uint32_t seqCounter = 0;
unsigned long lastStatusMs = 0;


// ============================================================
// MPU6050
// ============================================================

Adafruit_MPU6050 mpu;


// ============================================================
// SENSOR VARIABLES
// ============================================================

float accelX = 0;
float accelY = 0;
float accelZ = 0;

float accelerationMagnitude = 0;

float previousX = 0;
float previousY = 0;
float previousZ = 0;

float motionEnergy = 0;


// ============================================================
// STATE MACHINE
// ============================================================

enum FallState {

  NORMAL,
  IMPACT_DETECTED,
  SETTLING,
  CONFIRMING,
  FALL_CONFIRMED

};

FallState state = NORMAL;


// ============================================================
// TIMERS
// ============================================================

unsigned long lastSampleTime = 0;
unsigned long lastDebugTime = 0;

unsigned long impactTime = 0;
unsigned long confirmationStartTime = 0;


// ============================================================
// IMPACT VALIDATION
// ============================================================

int impactSamples = 0;


// ============================================================
// ESP-NOW SEND HELPERS
// ============================================================

void onEspNowSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "[ESP-NOW] sent OK" : "[ESP-NOW] send FAILED");
}

void sendWorkerPacket(uint8_t msgType, const char* riskState, const char* hazardType,
                       float sentMotionEnergy, float secondsSinceMotion, float motionEnergyAtTrigger) {
  WorkerPacket pkt = {};
  strncpy(pkt.workerId, WORKER_ID, sizeof(pkt.workerId) - 1);
  pkt.msgType = msgType;
  strncpy(pkt.riskState, riskState, sizeof(pkt.riskState) - 1);
  strncpy(pkt.hazardType, hazardType, sizeof(pkt.hazardType) - 1);
  pkt.motionEnergy = sentMotionEnergy;
  pkt.secondsSinceMotion = secondsSinceMotion;
  pkt.motionEnergyAtTrigger = motionEnergyAtTrigger;
  pkt.seq = seqCounter++;

  esp_err_t result = esp_now_send(wallNodeAddress, (uint8_t*)&pkt, sizeof(pkt));
  Serial.printf("[ESP-NOW] seq=%u msgType=%u riskState=%-16s -> %s\n",
                (unsigned)pkt.seq, pkt.msgType, pkt.riskState,
                result == ESP_OK ? "queued" : "FAILED to queue");
}

void sendStatusPacket() {
  // secondsSinceMotion is approximate here — 0 whenever we're
  // actively sampling in NORMAL state. It only matters precisely
  // on the DISTRESS path below, which is what the dashboard's
  // inactivity timer actually uses.
  sendWorkerPacket(0, "OK", "", motionEnergy, 0, 0);
}

void sendDistressPacket() {
  // FALL_CONFIRMATION_TIME_MS is the exact threshold that just
  // fired, so it's the right "seconds inactive" to report — the
  // real elapsed time is >= this value by at most one sample
  // interval (100ms), close enough for the dashboard's timer.
  sendWorkerPacket(1, "FALL_SUSPECTED", "FALL_SUSPECTED",
                   motionEnergy, FALL_CONFIRMATION_TIME_MS / 1000.0, motionEnergy);
}


// ============================================================
// HELPER: RESET FALL DETECTION
// ============================================================

void resetFallDetection() {

  state = NORMAL;

  impactSamples = 0;

  impactTime = 0;

  confirmationStartTime = 0;

  Serial.println();
  Serial.println(">>> FALL CANDIDATE CANCELLED");
  Serial.println(">>> Worker movement detected");
  Serial.println(">>> Returning to NORMAL");
  Serial.println();
}


// ============================================================
// HELPER: ALERT ON
// ============================================================

void triggerAlarm() {

  state = FALL_CONFIRMED;

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);

  // Tell the wall node right away — a dropped fall alert is not
  // acceptable, so send it a couple of extra times over the next
  // second in case the first ESP-NOW frame is lost in the air.
  sendDistressPacket();
  sendDistressPacket();

  Serial.println();
  Serial.println("========================================");
  Serial.println("          !!! FALL CONFIRMED !!!");
  Serial.println("========================================");

  Serial.println("LED    : ON");
  Serial.println("BUZZER : ON");

  Serial.print("Acceleration magnitude: ");
  Serial.println(accelerationMagnitude, 2);

  Serial.print("Motion energy: ");
  Serial.println(motionEnergy, 2);

  Serial.println("========================================");
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


  accelX = accel.acceleration.x;
  accelY = accel.acceleration.y;
  accelZ = accel.acceleration.z;


  // ----------------------------------------------------------
  // TOTAL ACCELERATION MAGNITUDE
  // ----------------------------------------------------------

  accelerationMagnitude = sqrt(

    accelX * accelX +
    accelY * accelY +
    accelZ * accelZ

  );


  // ----------------------------------------------------------
  // MOTION ENERGY
  // ----------------------------------------------------------

  float dx = accelX - previousX;
  float dy = accelY - previousY;
  float dz = accelZ - previousZ;

  float currentChange = sqrt(

    dx * dx +
    dy * dy +
    dz * dz

  );


  // Smooth the energy value instead of reacting to one sample.

  motionEnergy =
    (motionEnergy * 0.8) +
    (currentChange * 0.2);


  previousX = accelX;
  previousY = accelY;
  previousZ = accelZ;
}


// ============================================================
// DEBUG DISPLAY
// ============================================================

void printDebug() {

  unsigned long now = millis();

  if (
    now - lastDebugTime <
    DEBUG_INTERVAL_MS
  ) {

    return;
  }

  lastDebugTime = now;


  Serial.print("[DATA] ");

  Serial.print("A=");
  Serial.print(
    accelerationMagnitude,
    2
  );

  Serial.print(" m/s2");

  Serial.print(" | Energy=");
  Serial.print(
    motionEnergy,
    2
  );

  Serial.print(" | State=");


  switch (state) {

    case NORMAL:
      Serial.println("NORMAL");
      break;

    case IMPACT_DETECTED:
      Serial.println("IMPACT_DETECTED");
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


  // ----------------------------------------------------------
  // OUTPUTS
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // WIFI + ESP-NOW
  // ----------------------------------------------------------

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

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] esp_now_init failed");
  } else {
    esp_now_register_send_cb(onEspNowSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, wallNodeAddress, 6);
    peerInfo.channel = 0; // use whatever channel WiFi is already on
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("[ERROR] esp_now_add_peer failed");
    }
  }


  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );


  Serial.println();
  Serial.println("========================================");
  Serial.println("       SENSORA FALL DETECTOR");
  Serial.println("========================================");


  // ----------------------------------------------------------
  // MPU6050
  // ----------------------------------------------------------

  Serial.println(
    "Initializing MPU6050..."
  );


  if (!mpu.begin()) {

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


  // ----------------------------------------------------------
  // INITIAL SENSOR READ
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // READY
  // ----------------------------------------------------------

  lastSampleTime = millis();

  lastDebugTime = millis();

  lastStatusMs = millis();


  Serial.println();
  Serial.println("----------------------------------------");

  Serial.print(
    "Impact threshold: "
  );

  Serial.print(
    IMPACT_THRESHOLD
  );

  Serial.println(
    " m/s2"
  );


  Serial.print(
    "Low-motion threshold: "
  );

  Serial.println(
    LOW_MOTION_THRESHOLD
  );


  Serial.print(
    "Settling time: "
  );

  Serial.print(
    SETTLING_TIME_MS / 1000
  );

  Serial.println(
    " sec"
  );


  Serial.print(
    "Fall confirmation: "
  );

  Serial.print(
    FALL_CONFIRMATION_TIME_MS / 1000
  );

  Serial.println(
    " sec"
  );


  Serial.println("----------------------------------------");

  Serial.println(
    "Worker fall detector READY"
  );

  Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  unsigned long now = millis();


  // ==========================================================
  // ROUTINE STATUS OVER ESP-NOW
  // ==========================================================
  //
  // Independent of the sensor-sampling cadence below — paused
  // while FALL_CONFIRMED so a confirmed fall isn't drowned out
  // by routine "OK" pings on the dashboard's event log.

  if (
    now - lastStatusMs >= STATUS_INTERVAL_MS &&
    state != FALL_CONFIRMED
  ) {

    lastStatusMs = now;

    sendStatusPacket();
  }


  // ==========================================================
  // SENSOR SAMPLING
  // ==========================================================

  if (
    now - lastSampleTime >=
    SAMPLE_INTERVAL_MS
  ) {

    lastSampleTime = now;


    readMPU();


    // ========================================================
    // NORMAL STATE
    // ========================================================

    if (
      state == NORMAL
    ) {

      if (
        accelerationMagnitude >=
        IMPACT_THRESHOLD
      ) {

        impactSamples++;

      }

      else {

        impactSamples = 0;
      }


      // Require multiple impact samples
      // to reduce single-sample noise.

      if (
        impactSamples >=
        REQUIRED_IMPACT_SAMPLES
      ) {

        state = IMPACT_DETECTED;

        impactTime = now;

        Serial.println();
        Serial.println(
          ">>> POSSIBLE IMPACT DETECTED"
        );

        Serial.print(
          ">>> Acceleration = "
        );

        Serial.print(
          accelerationMagnitude,
          2
        );

        Serial.println(
          " m/s2"
        );

        Serial.println(
          ">>> Starting settling period..."
        );

        impactSamples = 0;
      }
    }


    // ========================================================
    // IMPACT DETECTED → SETTLING
    // ========================================================

    if (
      state == IMPACT_DETECTED
    ) {

      state = SETTLING;
    }


    // ========================================================
    // SETTLING PERIOD
    // ========================================================

    if (
      state == SETTLING
    ) {

      unsigned long elapsed =
        now - impactTime;


      if (
        elapsed >=
        SETTLING_TIME_MS
      ) {

        state = CONFIRMING;

        confirmationStartTime =
          now;

        Serial.println();
        Serial.println(
          ">>> SETTLING COMPLETE"
        );

        Serial.println(
          ">>> 20 SECOND FALL CONFIRMATION STARTED"
        );

        Serial.println(
          ">>> Worker movement can CANCEL the alert"
        );

        Serial.println();
      }
    }


    // ========================================================
    // CONFIRMATION
    // ========================================================

    if (
      state == CONFIRMING
    ) {

      unsigned long inactiveTime =
        now - confirmationStartTime;


      // ------------------------------------------------------
      // WORKER MOVEMENT DETECTED
      // ------------------------------------------------------

      if (
        motionEnergy >
        LOW_MOTION_THRESHOLD
      ) {

        resetFallDetection();
      }


      // ------------------------------------------------------
      // WORKER REMAINS INACTIVE
      // ------------------------------------------------------

      else {

        // Print countdown every second.

        static unsigned long lastCountdown =
          0;


        if (
          now - lastCountdown >=
          1000
        ) {

          lastCountdown = now;


          Serial.print(
            "[FALL CHECK] Inactive: "
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
        // CONFIRMED
        // ----------------------------------------------------

        if (
          inactiveTime >=
          FALL_CONFIRMATION_TIME_MS
        ) {

          triggerAlarm();
        }
      }
    }


    // ========================================================
    // FALL CONFIRMED
    // ========================================================

    if (
      state == FALL_CONFIRMED
    ) {

      // Alarm stays ON.
      // No automatic reset yet.

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
  // DEBUG
  // ==========================================================

  printDebug();
}
