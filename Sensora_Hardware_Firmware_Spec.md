# Sensora — Hardware & Firmware Implementation Specification
### Mine Deployment: Wall-Mounted Anchor Backbone + Worker Sensor Node

**Audience:** this document is written to be handed to another AI coding agent (or a human firmware engineer) with no other context, and used as the sole spec for implementing the embedded firmware. Where a decision is left open, it says so explicitly and names the config constant that must expose it — do not silently hardcode a guess.

**Relationship to earlier project docs:** `architecture-plan.md` (in this project) covers the backend, dashboard, and ML layers, and a *generic* flat-mesh topology. This document **supersedes** that doc's §2 (Communication Layer) and §3 (Hardware BOM) for the topology now confirmed: fixed wall-mounted anchor nodes forming the network backbone, plus mobile worker nodes that are sensor sources but not relays. Everything downstream of the gateway (MQTT topics, DB schema, dashboard) still applies unchanged — only the packet format is extended (§4) to carry the new fields this topology needs.

**Deliberate mandate — read this before writing any code:** the team already knows of other implementations using the same sensor types (temperature, humidity, MPU6050, atmospheric pressure, air quality). The requirement here is that **the underlying logic must be different, not just the wiring.** §7, §8, and §9 exist specifically to replace the "read a sensor, compare to a fixed threshold" pattern used elsewhere with adaptive, justified algorithms. §14 is a checklist of exactly what to avoid re-implementing. Treat that checklist as a hard constraint, not a suggestion.

---

## 0. System Topology

```
   (surface / mine entrance)                                    (mine face, deeper in)
        ┌──────────┐        ┌──────────┐        ┌──────────┐        ┌──────────┐
        │ ANCHOR 1  │◄──────►│ ANCHOR 2  │◄──────►│ ANCHOR 3  │◄──────►│ ANCHOR N  │  ...
        │ =GATEWAY  │        │           │        │           │        │           │
        │ (USB→PC)  │        │           │        │           │        │           │
        └────┬─────┘        └────┬─────┘        └────┬─────┘        └────┬─────┘
             │  ▲                 │  ▲                 │  ▲                 │  ▲
             │  │ ESP-NOW         │  │                 │  │                 │  │
             ▼  │ (worker→        ▼  │                 ▼  │                 ▼  │
        ┌──────────┐        ┌──────────┐        ┌──────────┐        ┌──────────┐
        │ WORKER A  │        │ WORKER B  │        │  (empty)  │        │ WORKER C  │
        │ (mobile)  │        │ (mobile)  │        │           │        │ (mobile)  │
        └──────────┘        └──────────┘        └──────────┘        └──────────┘
```

Two roles only. This is a deliberate simplification from a fully general mesh, and it matches how real fixed-reader / mobile-tag industrial tracking systems are built underground — fixed infrastructure is reliable and gets the hard job (multi-hop relaying); mobile nodes get the simple job (talk to whichever fixed node is closest).

- **ANCHOR** (`ROLE_ANCHOR`): wall-mounted, fixed position, mains/battery-buffered, no sensors required. Forms the backbone chain by relaying to its neighbors. One specific anchor (physically the one nearest the mine entrance / nearest existing connectivity) is additionally flagged `IS_GATEWAY` and bridges to the surface backend over USB serial. Anchors are installed **in physical order** at regular intervals (e.g. every 40–60 m) and are given sequential IDs in that order — the ID sequence *is* the position reference (§6).
- **WORKER** (`ROLE_WORKER`): worn by a person, mobile, carries the full sensor set (§1). A worker is a **leaf node**: it only ever talks to whichever anchor it currently hears best. It never relays another worker's traffic. This is what makes the system robust to workers moving in and out of range — you're only ever depending on the fixed backbone, not on other people also being nearby.

---

## 1. Bill of Materials

### 1.1 WORKER node

| Component | Part | Interface | Purpose |
|---|---|---|---|
| MCU | ESP32 dev board (WROOM-32, 30- or 38-pin) | — | Compute + ESP-NOW radio |
| Temp / humidity / pressure | **BME280** (not separate DHT22 + barometer — one I²C chip covers all three, faster sampling, and is itself a departure from the DHT22-based designs this needs to differ from) | I²C, addr `0x76` or `0x77` (check module's SDO strap) | §7 fusion inputs; pressure specifically for **rate-of-change**, not absolute value (§7.2) |
| Motion | **MPU6050** (accel + gyro) | I²C, addr `0x68` (or `0x69` if `AD0` strapped high) | Fall detection + tilt/orientation (§7.3) |
| Gas | **MQ-7** (CO) as primary — see the hardware-honesty note below | Analog (ADC1 channel) + 1 GPIO for heater control (§8) | The acutely dangerous gas underground is carbon monoxide (blasting, diesel equipment, fire), not generic "air quality" |
| SOS | Momentary push button, normally-open, to GND | GPIO w/ internal pull-up, interrupt-driven | Hard override, always wins (§7.4) |
| Local feedback | Piezo buzzer (GPIO + transistor if >20mA) **and** a status LED — prefer a single WS2812 addressable RGB LED on one GPIO over three discrete LEDs, fewer pins, more states | GPIO (buzzer), GPIO (LED data) | Must show state with **zero network connectivity** (§7.5) |
| Power | 2000–3000 mAh LiPo + TP4056 charge module + a 100kΩ/100kΩ divider into an ADC pin (battery is well above 3.3V logic max, divider is mandatory, do not connect raw cell voltage to a GPIO/ADC) | ADC1 channel | Battery % telemetry |

**Hardware-honesty note on "air quality":** if the team's "air quality" sensor on hand is a generic MQ-135 (broad VOC/CO2-ish index, uncalibrated), it is not a good proxy for the two gases that actually kill people underground: **carbon monoxide** and **methane**. Wire and read whatever sensor is physically available (the firmware in §7 is written generically against "gas channel," swap the calibration constants in `config.h`), but flag to the team in the pitch that a real deployment needs a CO-specific sensor (MQ-7) at minimum, and methane (MQ-4) as a second channel if the mine has blasting/gas pockets. Don't let "air quality" imply broader safety coverage than the actual sensor provides.

### 1.2 ANCHOR node

| Component | Part | Interface | Purpose |
|---|---|---|---|
| MCU | ESP32 dev board | — | Compute + ESP-NOW radio |
| Status LED | Single LED (or WS2812) | GPIO | Visual "alive / relaying / gateway" indicator for the demo — this is the node someone will physically power off on stage |
| Power | USB / mains adapter, or LiPo + charge module if no wall power at that point in the tunnel | — | Anchors are meant to be always-on infrastructure |
| Gateway variant only | USB cable to the laptop/Pi running the backend | UART (native USB-serial on most ESP32 dev boards) | Bridges mesh → backend (§10) |

Anchors carry **no sensors** in this spec. If the team wants an anchor to double as the flood/environmental node from `architecture-plan.md` §3.3, that is an additive role flag (`ROLE_ANCHOR | ROLE_ENV`) — out of scope for this document, mention it back to the team rather than improvising it.

### 1.3 Pin map (WORKER, ESP32 WROOM-32 30-pin reference — renumber for the exact board on hand)

| Signal | GPIO | Notes |
|---|---|---|
| I²C SDA (BME280 + MPU6050 shared bus) | GPIO21 | Both chips on one bus; addresses differ, no conflict |
| I²C SCL | GPIO22 | 4.7kΩ pull-ups to 3.3V if not already on the breakout modules |
| MQ-7 analog out | GPIO34 (ADC1_CH6, input-only pin, correct choice for analog) | Never use ADC2 pins if Wi-Fi/ESP-NOW is active — ADC2 conflicts with the radio driver |
| MQ-7 heater control | GPIO25 (PWM-capable) | Through a logic-level MOSFET (e.g. 2N7000), not driven directly (§8) |
| SOS button | GPIO27, `INPUT_PULLUP`, interrupt on `FALLING` | Debounce in software (§9), not just hardware RC, so timing is inspectable/loggable |
| Buzzer | GPIO26 (via transistor if current draw requires it) | PWM tone generation |
| Status LED (WS2812) | GPIO4 | Single-wire addressable — one pin, many colors/states |
| Battery ADC | GPIO35 (ADC1_CH7, input-only) | Through the voltage divider — never raw cell voltage |

**Do not use GPIO 6–11** (connected to the onboard SPI flash) or GPIO 0/2/15 without checking the specific board's strapping behavior at boot.

---

## 2. Firmware File Structure (both node types share this skeleton)

Both ANCHOR and WORKER firmware should be **one shared codebase** compiled with a role flag, not two divergent sketches — this is what makes "kill the anchor, worker keeps working" a property of the design rather than luck.

```
/firmware
  /include
    config.h          # every tunable constant, every pin, ROLE_* defines — nothing tunable lives outside this file
    packet.h          # wire-format structs (§4) — shared verbatim by both roles
  /src
    main.cpp          # setup()/loop(), role dispatch
    mesh.cpp / .h      # ESP-NOW init, peer management, routing table, heartbeat, failure detection (§5, §6)
    sensors.cpp / .h   # BME280 / MPU6050 / MQ-7 acquisition (§8) — WORKER only, compiled out for ANCHOR
    risk_engine.cpp / .h  # adaptive baseline + fusion + hysteresis + overrides (§7) — WORKER only
    localization.cpp / .h # RSSI-based position estimate (§6.3) — WORKER only
    gateway_bridge.cpp / .h # serial JSON bridge (§10) — GATEWAY build only
    ui.cpp / .h        # LED/buzzer state rendering (§7.5) — WORKER only (ANCHOR just needs a simple "alive" blink)
```

`config.h` must define, at minimum: `NODE_ROLE` (`ROLE_ANCHOR` / `ROLE_WORKER`), `IS_GATEWAY` (bool, anchors only), `NODE_ID` (uint8), `ANCHOR_POSITION_M` (float, anchors only), `WIFI_CHANNEL` (must match across every device on site — pick one, default `6`), plus every threshold/weight named in §7 and §8. **No magic numbers in the `.cpp` files** — if a value affects behavior, it lives in `config.h` with a comment explaining what it controls.

---

## 3. ESP-NOW API note (check before coding)

Arduino-ESP32 core has two coexisting APIs:

1. **Raw ESP-IDF C API** (`esp_now.h`: `esp_now_init()`, `esp_now_register_recv_cb()`, `esp_now_register_send_cb()`, `esp_now_add_peer()`, `esp_now_send()`) — stable across core 2.x and 3.x, the most widely documented, and what this spec's pseudocode assumes.
2. A newer **class-based wrapper** (`ESP_NOW`, `ESP_NOW_Peer` classes) shipped in recent core 3.x — object-oriented, less example coverage as of this writing.

**Use the raw C API (option 1)** unless the team's installed core version doesn't expose it — it's more portable across board packages and every ESP-NOW tutorial the team will find while debugging uses it. Confirm the installed `arduino-esp32` core version on every laptop before Phase 0 (per the build-plan doc) — a version mismatch across team laptops is a common, avoidable source of "works on my machine."

ESP-NOW payload cap is **250 bytes** on classic ESP32 (WROOM-32) — the packet format in §4 is sized to fit with headroom. All nodes must be on the **same Wi-Fi channel** (`config.h: WIFI_CHANNEL`) — ESP-NOW does not negotiate this for you.

---

## 4. Wire Packet Format

```c
// packet.h — shared verbatim by ANCHOR and WORKER builds
typedef enum : uint8_t {
  ROLE_ANCHOR = 0,
  ROLE_WORKER = 1,
} NodeRole;

typedef enum : uint8_t {
  MSG_ANCHOR_BEACON = 0,   // anchor -> everyone in range, periodic
  MSG_TELEMETRY      = 1,  // worker -> its chosen anchor, routine
  MSG_EMERGENCY       = 2, // worker -> its chosen anchor (unicast attempt) or all anchors (broadcast fallback)
  MSG_ANCHOR_RELAY    = 3, // anchor -> anchor, carries a forwarded TELEMETRY or EMERGENCY payload toward the gateway
  MSG_ACK             = 4,
} MsgType;

typedef struct __attribute__((packed)) {
  uint8_t  version;        // = 1
  uint8_t  msgType;        // MsgType
  uint8_t  srcId;          // original sender's NODE_ID (worker or anchor)
  uint8_t  srcRole;        // NodeRole of the ORIGINAL sender (unchanged as it's relayed)
  uint16_t seq;            // per-source sequence number — dedup + replay rejection
  uint8_t  ttl;            // hop budget, decremented by each relaying anchor
  uint8_t  hopCount;       // hops traveled so far
  uint8_t  priority;       // 0 = normal, 1 = emergency
  uint8_t  payloadLen;
  uint8_t  payload[64];    // interpreted per msgType, see below — pad unused bytes with 0
} MeshPacket;              // sizeof <= 74 bytes, comfortably under the 250-byte ESP-NOW cap

// payload for MSG_ANCHOR_BEACON:
typedef struct __attribute__((packed)) {
  float    positionM;         // this anchor's surveyed distance from entrance, meters
  uint8_t  hopCountToGateway; // 0 for the gateway anchor itself
  uint8_t  battery;           // 0-100, or 0xFF if mains-powered (no battery)
} BeaconPayload;

// payload for MSG_TELEMETRY:
typedef struct __attribute__((packed)) {
  int16_t  tempC_x10;      // temperature * 10 (fixed point, avoids float in the packet)
  uint8_t  humidityPct;
  int16_t  pressureRate_x100; // hPa/min * 100 (signed — can rise or fall)
  uint16_t gasRaw;         // raw ADC reading, calibration happens off-device
  uint8_t  motionEnergy;   // 0-255, scaled rolling accel-variance metric
  uint8_t  battery;        // 0-100
  uint8_t  riskState;      // 0 NORMAL / 1 WARNING / 2 CRITICAL (worker's own local computation, sent for cross-checking against backend fusion)
} TelemetryPayload;

// payload for MSG_EMERGENCY: TelemetryPayload fields PLUS:
typedef struct __attribute__((packed)) {
  TelemetryPayload base;
  uint8_t  hazardType;     // 0 GAS, 1 FALL, 2 SOS, 3 HEAT, 4 MULTI (see §7.4)
  uint8_t  severity;       // 1 WARNING / 2 CRITICAL / 3 SOS
} EmergencyPayload;
```

Notes an implementing agent must respect:
- `seq` increments per **originating** node, never reset by a relay — this is what lets any anchor detect and drop a duplicate it's already forwarded (§6.2).
- The gateway anchor decodes `MeshPacket` into JSON **before** it reaches the backend (§10) — nothing past the gateway needs to know this wire format.
- Fixed-point integers (`tempC_x10`, `pressureRate_x100`) are deliberate — floats in a packed struct sent over the air invite endianness/alignment bugs on some toolchains; fixed-point sidesteps it entirely and is cheap to decode on the backend.

---

## 5. Anchor Backbone Routing

Anchors run the same gradient-routing idea used in the general architecture doc, now applied only among fixed nodes (much more stable than applying it to mobile ones):

1. Every anchor broadcasts `MSG_ANCHOR_BEACON` every **2 s** (`config.h: BEACON_INTERVAL_MS`), containing its `positionM`, its own `hopCountToGateway`, and battery.
2. Each anchor keeps a neighbor table of other anchors it hears: `{anchorId, hopCountToGateway, lastSeenMs}`. It computes `myHopCount = min(neighbor.hopCountToGateway) + 1` and remembers `bestNextHop = argmin` (tie-break: most recently heard). The gateway anchor hardcodes `hopCountToGateway = 0`.
3. **Failure detection:** if `bestNextHop` hasn't beaconed in `3 × BEACON_INTERVAL_MS` (6 s, `config.h: ANCHOR_TIMEOUT_MS`), drop it from the table and recompute from whoever remains. This is the exact mechanism behind "power off Anchor 3, watch Anchor 2 and Anchor 4 route around it."
4. **Routine relay** (`MSG_ANCHOR_RELAY` wrapping a `TELEMETRY` payload, priority 0): unicast hop-by-hop via `bestNextHop`.
5. **Emergency relay** (priority 1): **broadcast** to all currently-known neighbor anchors, each of which forwards once (dedup via §6.2) and decrements `ttl` (`config.h: EMERGENCY_TTL`, default 10 — generous, this is a small backbone). This guarantees delivery via every simultaneously-live path, which is the point: an emergency should not be waiting on route recomputation.

---

## 6. Worker-to-Anchor Association

### 6.1 Choosing an anchor

The worker never actively "connects" to an anchor (ESP-NOW is connectionless) — it just decides which anchor to address its unicast packets to, based on what it's hearing:

- Maintain an EWMA-smoothed RSSI per heard anchor: `rssi_smoothed = 0.3 * rssi_new + 0.7 * rssi_smoothed` (`config.h: RSSI_EWMA_ALPHA`).
- `currentAnchor` only changes if a different anchor's smoothed RSSI exceeds the current one by more than `config.h: ANCHOR_SWITCH_MARGIN_DB` (default **6 dB**) for **two consecutive beacon periods**. Without this hysteresis, a worker standing roughly equidistant between two anchors will flap its target every beacon — visible on the dashboard as a worker "teleporting" between two positions, and a bad look in a demo.

### 6.2 Sequence-number dedup (also the replay defense named in the security section of `architecture-plan.md`)

Every node — anchor or worker — keeps a small ring buffer (`config.h: DEDUP_CACHE_SIZE`, default 32) of `{srcId, seq}` pairs it has already forwarded/processed, evicted oldest-first or after `config.h: DEDUP_TTL_MS` (default 8000 ms). A packet whose `{srcId, seq}` is already in the cache, or whose `seq` is at or below the last one accepted from that `srcId`, is dropped silently. This single mechanism does two jobs: stops flood storms from re-forwarding the same emergency packet forever, and rejects replayed/duplicated packets — implement it once, in `mesh.cpp`, shared by both roles.

### 6.3 Position estimate (this is a genuine capability, not a demo trick — GPS does not work underground)

Because anchors are installed at known, surveyed positions along the tunnel, a worker can estimate its own linear position without any location hardware:

```
if only one anchor heard within HEARING_FLOOR_DBM (config.h, default -85):
    position_estimate = thatAnchor.positionM        // coarse: "near Anchor K"
elif two anchors heard above the floor:
    w1 = pow(10, rssi1_smoothed / 20.0)
    w2 = pow(10, rssi2_smoothed / 20.0)
    position_estimate = (anchor1.positionM * w1 + anchor2.positionM * w2) / (w1 + w2)
else:
    position_estimate = last known value, flagged STALE after LOCATION_STALE_MS
```

This is a coarse, 1-D estimate, correct for the common case of a mostly-linear tunnel with anchors placed along it — it is **not** true trilateration and will be ambiguous exactly at a junction where the tunnel branches (two different corridors can be equidistant between the same anchor pair). If the deployment has branches, either place an anchor at every junction (so a worker near a branch always hears a junction-specific anchor most strongly) or add a `branchId` field to `BeaconPayload` and don't interpolate across a branch boundary. Say this explicitly to the team rather than letting the position estimate silently misrepresent a branch.

---

## 7. Risk Fusion (WORKER only) — this is the section that must not read like a copy of a threshold-based design

### 7.1 Adaptive baseline, not fixed thresholds

On boot, each worker runs a **calibration window** (`config.h: CALIBRATION_MS`, default 90 000 ms) during which it computes a running mean and variance for temperature, humidity, and gas raw value using Welford's online algorithm, then continues updating that baseline for the rest of runtime via an EWMA with a slow time constant (`config.h: BASELINE_TAU_MS`, default 600 000 ms / 10 min) — slow enough that a fast-onset hazard (seconds to low minutes) shows up as a clear deviation, but the baseline still tracks legitimate slow drift (e.g. the mine gradually warming toward end of shift) so the system doesn't need re-tuning every deployment.

**During calibration, the local state must display `CALIBRATING`, not `NORMAL`.** Showing a green "all clear" LED while the gas sensor heater hasn't even finished its first warm-up cycle (§8) is actively misleading and is exactly the kind of shortcut this spec is asking you not to take.

### 7.2 Signals actually fed into the fusion

| Signal | Derived as | Why not the raw value |
|---|---|---|
| `zTemp` | `(temp - baselineMean) / baselineStd` (std floored at `config.h: MIN_TEMP_STD`, default 0.5°C, to avoid division blowing up in a very stable environment) | Standard z-score against a live baseline, not a hardcoded "above 40°C" line that means something different in a naturally-hot mine versus a temperate one |
| `zHumidity` | Same z-score treatment | Same reasoning |
| `zPressureRate` | z-score of **dP/dt** (hPa per minute, computed from a rolling buffer of the last 3 BME280 readings), not of absolute pressure | Absolute pressure just reflects depth/weather and isn't itself hazardous; a **sudden rate of change** is what can indicate a structural event (a roof fall's pressure wave) or a ventilation failure. This channel does not exist in any of the threshold-based designs surveyed for the project's novelty analysis — treat it as a first-class hazard signal, not a logging afterthought |
| `zGas` | `max(0, (gasRaw - baselineMean) / baselineStd)` — **clipped at zero**, only positive deviation counts | A gas reading *below* baseline is not a hazard; don't let a symmetric z-score treat "slightly cleaner air than average" as equally suspicious as "slightly worse" |
| `motionEnergy` | Variance of accel-magnitude over a rolling 2 s window, scaled to 0–255 for the packet | Feeds fall-detection (§7.3) and is reported in telemetry, but is **not** part of the continuous z-score sum below — motion anomaly and fall are handled as a dedicated state machine, not folded into the ambient-risk score |

### 7.3 Fall / impact detection (context, not a bare threshold)

Maintain a calibrated "worn, at-rest" orientation from a complementary filter during the calibration window:
```
tiltAngle = 0.98 * (tiltAngle + gyroRate * dt) + 0.02 * accelDerivedAngle
```
Fall state machine (per-worker, runs every IMU sample, ~50 Hz recommended):
1. **Impact:** accel magnitude jerk exceeds `config.h: FALL_IMPACT_G` (default 3.0 g) within a `config.h: FALL_IMPACT_WINDOW_MS` (default 150 ms) window.
2. **Confirm:** after an impact, if `motionEnergy` stays below `config.h: FALL_STILLNESS_THRESHOLD` for `config.h: FALL_STILLNESS_MS` (default 2000 ms) **and** the tilt angle has deviated from the calibrated worn orientation by more than `config.h: FALL_TILT_DEG` (default 45°) → declare `hazardType = FALL`, `riskState = CRITICAL`.
3. If stillness doesn't hold (person gets back up) or tilt doesn't confirm (it was a jump, not a fall) → no state change, log the impact as a minor telemetry event only.

This directly targets the specific, named failure mode of accelerometer-threshold-only designs (can't distinguish a fall from a jump) rather than reproducing it.

### 7.4 Combined ambient risk score

```
R = sqrt( wG * zGas^2 + wT * zTemp^2 + wH * zHumidity^2 + wP * zPressureRate^2 )
```
Default weights in `config.h` (tune during field calibration, do not bury these in `risk_engine.cpp`): `wG = 0.40`, `wP = 0.25`, `wT = 0.20`, `wH = 0.15` — gas weighted highest because CO onset is the most acutely dangerous of the four ambient signals; pressure-rate weighted meaningfully because it's the one channel that can indicate a sudden structural/ventilation event rather than gradual drift.

**State transitions, with asymmetric hysteresis to prevent flicker:**
| Transition | Condition |
|---|---|
| → WARNING | `R > config.h: WARNING_ENTER` (default 2.5) |
| WARNING → CRITICAL | `R > config.h: CRITICAL_ENTER` (default 4.0) |
| CRITICAL → WARNING | `R < config.h: CRITICAL_EXIT` (default 3.2) — not the same as the entry threshold |
| WARNING → NORMAL | `R < config.h: WARNING_EXIT` (default 1.8) |

**Hard overrides — checked every loop iteration, independent of `R`, and always win:**
1. SOS button pressed → `riskState = SOS`, `hazardType = SOS`, latched (does not clear on its own; requires a long-press to acknowledge/reset locally, mirroring the "supervisor acknowledges" step in the user flow).
2. Fall confirmed (§7.3) → `CRITICAL`, `hazardType = FALL`.
3. **Absolute gas ceiling:** if raw gas ADC value corresponds to a concentration above `config.h: GAS_ABSOLUTE_CEILING_PPM` — a value the team must set from an actual mine-safety reference for CO (do not invent this number; it needs to reflect whatever exposure limit the deployment is meant to respect) — declare `CRITICAL` regardless of the adaptive baseline. This is deliberate belt-and-suspenders: the adaptive z-score catches relative anomalies you wouldn't think to hardcode; the absolute ceiling guarantees a known-dangerous concentration always triggers even if the baseline has drifted upward for some other reason.

### 7.5 Local feedback (must work with zero radio connectivity)

The WS2812 status LED and buzzer are driven directly from `riskState`, computed and updated locally regardless of whether any anchor is currently in range:
- `CALIBRATING` → slow blue pulse, silent.
- `NORMAL` → solid dim green, silent.
- `WARNING` → pulsing amber, short periodic beep.
- `CRITICAL` → pulsing red, continuous fast beep.
- `SOS` → solid red, distinct alarm tone (different pattern from CRITICAL so it's audibly distinguishable).

This local rendering must not depend on anything in `mesh.cpp` — a worker whose radio can't reach any anchor still needs to see and hear their own state.

---

## 8. MQ-7 Acquisition (the sensor most likely to be implemented lazily elsewhere — don't)

MQ-7 requires a **duty-cycled heater** per its datasheet: ~60 s at ~5V (burn-off / high-temperature phase) alternating with ~90 s at ~1.4V (low-temperature sensing phase); the analog reading is only meaningful near the end of the low-temperature phase.

**First, check which physical module the team has:**
- **Module with onboard heater-cycle driver** (common on breakout boards with a dedicated heater-control chip/timer already on the PCB): firmware only needs to read the ADC on a schedule synced to the module's known cycle — check the specific module's datasheet for its cycle timing and sync to it rather than assuming the 60/90 split above.
- **Bare MQ-7 element / module with an exposed heater pin:** firmware must drive the heater itself via a logic-level MOSFET on `config.h: MQ7_HEATER_PIN`:
  ```
  state HIGH_HEAT (60s):  heaterPin = PWM 100% duty  (≈5V across the heater through the MOSFET)
  state LOW_HEAT  (90s):  heaterPin = PWM ~28% duty   (approximates 1.4V average — verify against
                                                        the specific MOSFET/heater resistance on hand,
                                                        this duty value is a starting point, not a
                                                        guarantee)
  read ADC only in the last 5s of LOW_HEAT
  ```
  This state machine must be non-blocking (driven by `millis()` comparisons in the main loop or a hardware timer callback, never `delay(60000)`), because the mesh/heartbeat logic in `mesh.cpp` needs to keep running throughout.

Whichever path applies, **do not treat a raw `analogRead()` taken at an arbitrary moment as a valid gas reading** — this is the single most common shortcut in hobbyist MQ-7 code and it produces numbers that look plausible but aren't meaningfully calibrated to anything.

---

## 9. Main Loop Discipline

Both roles run a single non-blocking loop — no `delay()` anywhere in steady-state code (a `delay()` in `loop()` stalls ESP-NOW packet reception and heartbeat timing, which is exactly the kind of bug that makes the reroute demo flaky). Structure `loop()` as a set of independently-timed tasks checked every pass:

```
loop():
    now = millis()
    if now - lastBeacon   >= BEACON_INTERVAL_MS:    doBeacon(); lastBeacon = now      // ANCHOR
    if now - lastSample   >= SENSOR_SAMPLE_MS:       doSensorRead(); lastSample = now  // WORKER
    if now - lastFusion   >= FUSION_INTERVAL_MS:     doRiskFusion(); lastFusion = now  // WORKER
    if now - lastTelemetry>= TELEMETRY_INTERVAL_MS:  sendTelemetry(); lastTelemetry = now // WORKER
    checkSosButton()                                  // interrupt-flagged, handled every pass
    pruneStaleNeighbors(now)                          // both roles, §5/§6
    // ESP-NOW receive is callback-driven (esp_now_register_recv_cb), not polled here
```

Recommended defaults (`config.h`): `SENSOR_SAMPLE_MS = 200`, `FUSION_INTERVAL_MS = 500`, `TELEMETRY_INTERVAL_MS = 2000` (routine), emergency packets are sent **immediately** on state transition into CRITICAL/SOS, not on the next scheduled telemetry tick.

---

## 10. Gateway Bridge (GATEWAY-flagged anchor only)

The gateway anchor decodes every `MeshPacket` it (a) originates as relay traffic or (b) receives directly, and writes one JSON line per packet to USB serial at a fixed baud (`config.h: SERIAL_BAUD`, default 115200):

```json
{"msgType":"TELEMETRY","srcId":103,"srcRole":"WORKER","seq":4821,"hopCount":2,
 "tempC":24.3,"humidityPct":61,"pressureRateHpaMin":-0.02,"gasRaw":812,
 "motionEnergy":6,"battery":78,"riskState":"NORMAL","positionEstM":142.5,
 "rxAtMs":183820113}
```
Field names here are illustrative — the exact schema should be agreed with whoever implements the backend ingestion script (per `architecture-plan.md` §5), but the gateway's job is fixed regardless of exact field names: **decode the binary packet fully before it leaves the gateway**, so nothing downstream needs to understand `MeshPacket`.

---

## 11. Config Reference (every constant this spec has named, in one place)

| Constant | Default | Section |
|---|---|---|
| `WIFI_CHANNEL` | 6 | §2 |
| `BEACON_INTERVAL_MS` | 2000 | §5 |
| `ANCHOR_TIMEOUT_MS` | 6000 | §5 |
| `EMERGENCY_TTL` | 10 | §5 |
| `RSSI_EWMA_ALPHA` | 0.3 | §6.1 |
| `ANCHOR_SWITCH_MARGIN_DB` | 6 | §6.1 |
| `DEDUP_CACHE_SIZE` | 32 | §6.2 |
| `DEDUP_TTL_MS` | 8000 | §6.2 |
| `HEARING_FLOOR_DBM` | -85 | §6.3 |
| `LOCATION_STALE_MS` | 10000 | §6.3 |
| `CALIBRATION_MS` | 90000 | §7.1 |
| `BASELINE_TAU_MS` | 600000 | §7.1 |
| `MIN_TEMP_STD` | 0.5 | §7.2 |
| `FALL_IMPACT_G` | 3.0 | §7.3 |
| `FALL_IMPACT_WINDOW_MS` | 150 | §7.3 |
| `FALL_STILLNESS_MS` | 2000 | §7.3 |
| `FALL_TILT_DEG` | 45 | §7.3 |
| `wG / wP / wT / wH` | 0.40 / 0.25 / 0.20 / 0.15 | §7.4 |
| `WARNING_ENTER / EXIT` | 2.5 / 1.8 | §7.4 |
| `CRITICAL_ENTER / EXIT` | 4.0 / 3.2 | §7.4 |
| `GAS_ABSOLUTE_CEILING_PPM` | **not defaulted — team must set from a real reference** | §7.4 |
| `SENSOR_SAMPLE_MS` | 200 | §9 |
| `FUSION_INTERVAL_MS` | 500 | §9 |
| `TELEMETRY_INTERVAL_MS` | 2000 | §9 |
| `SERIAL_BAUD` | 115200 | §10 |

---

## 12. Acceptance Checklist (the implementing agent should self-verify these before calling the firmware done)

1. A single WORKER, powered on with **no anchor in range**, shows `CALIBRATING` then `NORMAL` on its own LED/buzzer, with no crash and no false CRITICAL, purely from local sensors.
2. Two ANCHORs powered on, a WORKER placed near Anchor 1: confirm (via serial log or the gateway's JSON stream) that telemetry routes through Anchor 1.
3. Move the WORKER (or attenuate signal) until Anchor 2 is clearly stronger: confirm the association switches once, not repeatedly, near the midpoint (validates the hysteresis in §6.1).
4. With three ANCHORs in a chain (1=gateway, 2, 3) and a WORKER associated with Anchor 3: power off Anchor 2. Within `ANCHOR_TIMEOUT_MS`, confirm Anchor 3's telemetry still reaches the gateway via an alternate path if one physically exists, or confirm the emergency-flood path succeeds even with no alternate path for an EMERGENCY-class packet specifically.
5. Trigger the SOS button: confirm an `EMERGENCY` packet is sent immediately (not on the next telemetry tick) and that local state changes to `SOS` even if radio is out of range.
6. Simulate a fall (a sharp shake meeting the impact threshold, followed by stillness): confirm `hazardType = FALL` fires; confirm a shake **without** subsequent stillness (e.g. continued movement) does **not** fire.
7. Confirm the gas channel's local state does not leave `CALIBRATING` before the MQ-7 heater cycle (§8) has completed at least one full low-heat phase.
8. Confirm no `delay()` calls exist anywhere reachable from `loop()` in either role's build.
9. Confirm every numeric threshold used in `risk_engine.cpp`, `mesh.cpp`, and `sensors.cpp` traces back to a named constant in `config.h` — grep for bare numeric literals in conditionals as a final check.

---

## 13. Open Items the Team Must Supply (do not guess these)

1. Exact number of anchors and their planned spacing/positions (`ANCHOR_POSITION_M` per unit) — depends on the specific mine tunnel being used for the demo.
2. `GAS_ABSOLUTE_CEILING_PPM` — must come from an actual CO exposure guideline the team is willing to cite, not an invented number.
3. Which MQ-7 module variant is physically on hand (onboard heater driver vs. bare element) — determines which branch of §8 applies.
4. Confirm the installed `arduino-esp32` core version across every team laptop before starting (§3) — pin it in a shared README so nobody discovers a mismatch mid-build.
5. Whether any anchor position corresponds to a tunnel junction/branch — affects whether §6.3's linear interpolation needs a `branchId` guard.

---

## 14. Explicit "Do Differently" Checklist

Compare against this before considering any module finished — each item names the shortcut to avoid and the section that replaces it:

- ❌ Fixed `if (gasValue > 500)` thresholds → ✅ adaptive baseline + z-score fusion (§7.1–7.4)
- ❌ Bare accelerometer-magnitude threshold for "fall" → ✅ impact + stillness + tilt confirmation state machine (§7.3)
- ❌ Treating absolute barometric pressure as a hazard signal → ✅ rate-of-change (dP/dt) as the actual signal (§7.2)
- ❌ `analogRead()` on the MQ-7 at an arbitrary moment → ✅ heater-cycle-synced reading (§8)
- ❌ GPS or a hardcoded "Zone 1/2/3" label for location → ✅ RSSI-interpolated position from surveyed anchor positions (§6.3)
- ❌ A flat mesh where every node (including mobile ones) relays for every other node → ✅ fixed-backbone-relays / mobile-leaves split (§0, §5, §6)
- ❌ `delay()`-based sensor or heater timing that stalls the radio loop → ✅ `millis()`-scheduled non-blocking tasks (§9)
- ❌ Silently showing "NORMAL" during sensor warm-up → ✅ explicit `CALIBRATING` state (§7.1, §7.5)
- ❌ Symmetric thresholding on gas deviation (equally suspicious of cleaner-than-baseline air) → ✅ one-sided clipped z-score (§7.2)
- ❌ A single entry/exit threshold per state (causes flicker at the boundary) → ✅ asymmetric hysteresis bands (§7.4)

---

## 15. References

- Bosch BME280 datasheet — combined humidity/pressure/temperature sensor: [bosch-sensortec.com](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- MQ-7 carbon monoxide sensor — datasheet, pinout, heater-cycle timing: [The Engineering Projects](https://www.theengineeringprojects.com/2024/02/mq-7-carbon-monoxide-sensor-datasheet-pinout-working.html), [SparkFun product page](https://www.sparkfun.com/carbon-monoxide-sensor-mq-7.html)
- Arduino-ESP32 ESP-NOW API reference (current, class-based) and the underlying ESP-IDF C API this spec targets: [docs.espressif.com — Arduino-ESP32 ESP-NOW](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/espnow.html), [ESP-IDF ESP-NOW programming guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_now.html)
- Espressif reference multi-node ESP-NOW example (current idioms): [ESP_NOW_Network.ino, espressif/arduino-esp32](https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP_NOW/examples/ESP_NOW_Network/ESP_NOW_Network.ino)
- Underground mine gas monitoring context (why CO/CH4-specific sensing matters more than generic AQI): [Application of Gas Monitoring Sensors in Underground Coal Mines and Hazardous Areas — ResearchGate](https://www.researchgate.net/publication/292148696_Application_of_Gas_Monitoring_Sensors_in_Underground_Coal_Mines_and_Hazardous_Areas)
