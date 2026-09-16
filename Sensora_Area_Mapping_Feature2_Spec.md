# Feature 2 — Area Mapping with MPU6050 + HC-SR04 (gyro-swept ultrasonic scan)

**Supersedes the "proximity snapshot" scope-down in `dashboard-features.md` / `feature-build-checklist.md`.** Those docs parked full mapping because a continuous mechanical sweep needs a servo you may not have. This spec instead uses the **worker's own arm/body motion as the sweep mechanism** — MPU6050 gyro-integrated yaw tells you the angle, HC-SR04 tells you the distance at that angle, and the firmware fuses the two into a 2D point cloud. No servo required. This is genuinely buildable without new hardware if you already have both sensors, and it turns Feature 2 from a snapshot into an actual on-screen room/tunnel outline.

---

## 0. The core idea

A single ultrasonic sensor only measures distance along one ray. To get a *shape*, you need distance samples across a range of angles. Instead of motorizing the sensor, you have the person holding/wearing the unit **sweep it by hand** (like panning a flashlight across a room) while the firmware:

1. tracks the sensor's heading in real time by integrating the MPU6050's gyro Z-axis,
2. pings the HC-SR04 repeatedly during the sweep,
3. pairs each distance reading with the heading at that instant,
4. converts each (angle, distance) pair into an (x, y) point relative to the sweep's starting position,
5. streams the resulting point set to the dashboard, which draws it as a 2D outline.

This is a **relative, single-sweep scan**, not persistent SLAM — it answers "what does the space around the worker look like right now," which is exactly what Feature 2's brief was aiting at, and it's a much stronger demo than a single-ray snapshot.

---

## 1. Hardware (no new parts beyond what's already in the BOM)

| Component | Already in BOM? | Role here |
|---|---|---|
| ESP32 | Yes (worker or a dedicated scan node) | Compute, drives the sweep logic |
| MPU6050 | Yes (`hardware-firmware-spec.md` §1.1 — I²C, addr `0x68`) | Gyro Z-axis → yaw integration during the sweep |
| HC-SR04 | Yes, per `architecture-plan.md` §3.3 (used there for water level; you need a second unit, or repurpose logic — ultrasonic modules are ~$1–2, worth having two) | Distance ray at the current heading |

**Wiring (HC-SR04, in addition to the existing MPU6050 I²C bus from `hardware-firmware-spec.md` §1.3):**

| Signal | Suggested GPIO | Notes |
|---|---|---|
| TRIG | GPIO32 | Output, 10µs HIGH pulse to fire a ping |
| ECHO | GPIO33 | Input — **use a voltage divider (1kΩ/2kΩ) if your HC-SR04 is 5V-logic**, ESP32 GPIOs are not 5V tolerant |

Both GPIO32/33 are free ADC1-adjacent pins not used elsewhere in the worker pin map (§1.3 of `hardware-firmware-spec.md` uses 21/22/34/25/27/26/4/35) — confirm against your actual board before wiring.

If you're mounting this on a **separate dedicated scan unit** rather than the worker's own helmet (recommended — see §5), the same two sensors plus an ESP32 is the entire BOM; it doesn't need the gas/temp sensors or the buzzer.

---

## 2. Firmware: the sweep algorithm

### 2.1 Trigger a scan

Add a dedicated button (or repurpose a long-press on the existing SOS button if you want zero new parts) to start/stop a scan session. Keep sessions **short — 5 to 8 seconds** for a single sweep; this bounds gyro drift (§2.2) and keeps the data volume manageable over ESP-NOW.

```
on scanButtonPressed():
    if not scanning:
        startScan()   // zero the yaw integrator, clear the point buffer, set scanning = true
    else:
        endScan()     // stop sampling, package and send the accumulated points
```

### 2.2 Yaw tracking (gyro integration, with the drift caveat already flagged in your own docs)

This reuses exactly the pattern `architecture-plan.md` names for the (cut) GPS-replacement `yawDeg` field, and the complementary-filter idea already used for tilt in `hardware-firmware-spec.md` §7.3:

```cpp
// called every IMU sample (~50 Hz, non-blocking, per the main-loop discipline in §9 of the firmware spec)
void updateYaw(float gyroZ_degPerSec, float dt_s) {
    yawDeg += gyroZ_degPerSec * dt_s;
    // no magnetometer on hand -> no absolute heading correction.
    // Over a short (5-8s) sweep the drift is small enough to be visually
    // acceptable for a hackathon demo; DO NOT trust this for anything
    // longer-running or safety-critical. Label it as such on the dashboard
    // (see §4) exactly the way GPS-cut and simulated-data fields are labeled
    // elsewhere in this project.
}
```

Zero `yawDeg` at `startScan()` so every sweep is self-consistent even though absolute compass heading isn't available.

### 2.3 Ultrasonic sampling

```cpp
float pingDistanceCm() {
    digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    unsigned long durationUs = pulseIn(ECHO_PIN, HIGH, 30000UL); // 30ms timeout ~= 5m range cap
    if (durationUs == 0) return -1;          // no echo / out of range -> discard sample
    float distanceCm = durationUs / 58.0f;
    if (distanceCm < 2 || distanceCm > 400) return -1;  // HC-SR04's real usable range
    return distanceCm;
}
```

**Important non-blocking note:** `pulseIn` blocks for up to its timeout (here 30ms). That's short enough not to meaningfully stall the mesh/heartbeat loop, but don't call it more often than every ~60ms (the HC-SR04 needs settling time between pings per its datasheet, matching the `MQ7`-style "don't take a shortcut on sensor timing" instruction already in `hardware-firmware-spec.md` §8) — schedule it the same way as the other periodic tasks in §9's `loop()` pattern:

```cpp
if (scanning && (now - lastPing >= SCAN_PING_INTERVAL_MS)) {   // config.h, default 100ms
    float d = pingDistanceCm();
    if (d > 0) {
        recordScanPoint(yawDeg, d);   // buffer the (angle, distance) pair
    }
    lastPing = now;
}
```

At 100ms per ping over a 6-second sweep, that's up to ~60 points — plenty for a recognizable outline, and small enough to fit in the packet budget below.

### 2.4 Polar → Cartesian conversion

Do this conversion **on the worker/scan node**, not the backend — it's cheap, and it means the packet carries directly plottable coordinates:

```cpp
struct ScanPoint { int16_t x_cm; int16_t y_cm; };  // fixed-point, matches the packet.h convention

ScanPoint toCartesian(float yawDeg, float distanceCm) {
    float rad = yawDeg * (PI / 180.0f);
    return { (int16_t)(distanceCm * cosf(rad)), (int16_t)(distanceCm * sinf(rad)) };
}
```

---

## 3. Wire format: extend `packet.h`, don't invent a parallel protocol

Per the note already in `dashboard-features.md` ("this is meant to extend the existing DISTRESS payload... not create a parallel envelope") and the versioned-payload pattern in `hardware-firmware-spec.md` §4, add a new `MsgType` rather than a new protocol:

```c
// packet.h additions
MSG_SCAN_CHUNK = 5,   // one chunk of a multi-packet area scan

typedef struct __attribute__((packed)) {
    uint8_t  scanId;        // increments per scan session, lets the gateway group chunks
    uint8_t  chunkIndex;    // 0-based
    uint8_t  chunkCount;    // total chunks in this scan, so the gateway knows when it's complete
    uint8_t  pointCount;    // points in THIS chunk
    int16_t  points[10][2]; // up to 10 (x_cm, y_cm) pairs per chunk
} ScanChunkPayload;         // 10*4 + 4 = 44 bytes, comfortably inside the 64-byte payload[] and the 250-byte ESP-NOW cap
```

A 60-point sweep needs 6 chunks of 10 points each — trivial over ESP-NOW, and it reuses the exact same anchor-relay/dedup path (§5, §6.2 of `hardware-firmware-spec.md`) every other packet type already uses, so nothing new is needed in the mesh layer.

**Gateway side:** buffer chunks by `scanId` as they arrive, and once `chunkCount` chunks are seen (or a timeout elapses — don't block waiting forever on a lost packet), assemble the full point list and emit one JSON message to the backend:

```json
{"msgType":"AREA_SCAN","srcId":103,"scanId":7,"originAnchorId":2,
 "points":[[120,45],[118,60],[95,88], ...], "pointCount":58, "rxAtMs":183820113}
```

---

## 4. Backend + dashboard

**Backend (`architecture-plan.md` §5 pattern):**
- New table: `area_scans(id, worker_id, ts, origin_anchor_id, points_json, is_partial)`.
- New MQTT topic: `sensora/scan/{nodeId}`.
- New WS diff type pushed to the dashboard when a scan completes.

**Dashboard — new panel, e.g. `AreaScanPanel`, following the design tokens already fixed in `dashboard-features.md`:**
- A simple 2D scatter/polygon plot (SVG, no charting library needed for point-count this small — same "hand-laid-out SVG beats a heavy graph library" reasoning already used for `NetworkTopology` in `architecture-plan.md` §6).
- Plot each point at `(x_cm, y_cm)` relative to a centered origin marker representing the worker.
- Connect consecutive points with a thin line in sweep order to suggest a wall outline (don't force a closed polygon — a partial sweep is an open arc, and closing it would misrepresent the room).
- **Label it clearly**, matching the project's existing honesty pattern (`SIMULATED DATA` flag, `SIMULATED MODEL` tag): something like **"Relative scan — gyro-only heading, may drift over the sweep"** near the panel, exactly the same treatment already used for the cut GPS/`yawDeg` field.
- If you also have the RSSI-based coarse worker position (`hardware-firmware-spec.md` §6.3) wired into the dashboard, you can offset the scan's origin to that estimated position on the main topology view — a nice-to-have, not required for the panel to be useful on its own.

---

## 5. Practical recommendation for the demo

Two ways to run this, pick based on time left:

1. **Handheld dedicated scan unit (recommended):** a second ESP32 + MPU6050 + HC-SR04 on a small breadboard/enclosure, separate from the worker's helmet. Someone holds it and slowly pans it in an arc in front of a judge-visible obstacle (a box, a chair leg) while pressing the scan button. This is the safest to rehearse because it's a standalone prop, not entangled with the worker's own fall/SOS firmware.
2. **Bundled into the worker node:** reuse the worker's existing MPU6050, add the HC-SR04, and trigger a scan either manually (extra button) or automatically on a DISTRESS event (the original intent noted in `dashboard-features.md`: "extend the existing DISTRESS payload"). More impressive narratively ("the system automatically scans the area around a fallen worker") but higher integration risk since it shares firmware/timing with the safety-critical path — only do this after option 1 works standalone, and don't let it block Feature 1.

Either way, rehearse the actual sweeping motion a few times before the demo — a jerky or too-fast sweep undersamples badly and produces a noisy scatter instead of a recognizable outline. A slow, steady ~90–180° arc over 5–8 seconds gives the cleanest result.

---

## 6. What to say if a judge asks about accuracy

> "This is a relative scan, not absolute mapping — the heading comes from gyro integration with no magnetometer, so it drifts over time. We bound that by keeping each sweep short, a few seconds, and treating each scan as a fresh, self-contained snapshot rather than accumulating drift over a long session. It's the same honesty pattern we use elsewhere in the project — labeled clearly on the dashboard rather than presented as more precise than it is."

This is consistent with, and reuses, the exact justification style already used for the cut GPS feature and the simulated flood model elsewhere in the project docs.
