# Sensora — System Architecture Plan

**Project:** Intelligent Worker Safety & Resilient Emergency Response System
**Prepared:** 15 Sep 2026 · Assumptions: 36–48h hackathon, 4–6 person team with role specialization, a few ESP32 boards + basic sensors already on hand (rest sourced/substituted below)
**Status:** Planning document — update as decisions change during the build

---

## 0. How to read this document

This is the architecture the team should build against, plus the reasoning behind each call and the places it's likely to break. Section 12 ("What a judge will challenge") is deliberately unflattering — read it before you pitch, not after a judge asks.

Everything below is scoped to **one coherent narrative**: an edge device assesses risk locally, a resilient mesh carries that risk assessment even as nodes die, and a command dashboard makes the whole chain visible in one glance. Every component earns its place by serving one of the five required demos. Nothing else gets built first.

---

## 1. System Overview

```
                         ┌───────────────────────────┐
                         │   COMMAND DASHBOARD (React) │
                         │  WebSocket live feed + REST │
                         └──────────────┬───────────────┘
                                        │ WS / HTTP
                         ┌──────────────┴───────────────┐
                         │   BACKEND (FastAPI + MQTT)    │
                         │  risk fusion · DB · APIs      │
                         └──────────────┬───────────────┘
                                        │ MQTT (paho)
                    ┌───────────────────┼────────────────────┐
                    │                                        │
         ┌──────────┴──────────┐                  ┌──────────┴──────────┐
         │  SCENARIO SIMULATOR  │                  │   GATEWAY NODE (ESP32) │
         │ (flood/scale/failure)│                  │  ESP-NOW ↔ USB-Serial │
         └───────────────────────┘                  └──────────┬──────────┘
                                                                │ ESP-NOW (2.4GHz)
                                        ┌───────────────────────┼───────────────────────┐
                                        │                       │                       │
                                 ┌──────┴──────┐        ┌───────┴──────┐        ┌───────┴──────┐
                                 │ HELMET NODE  │        │  RELAY NODE   │        │  ENV/FLOOD NODE│
                                 │ temp/gas/IMU │  ◄──►  │ (no sensors,  │  ◄──►  │ water level +  │
                                 │ /SOS/buzzer  │  mesh  │  forwards only)│        │ rainfall input │
                                 └──────────────┘        └───────────────┘        └────────────────┘
```

Two things make this "one system" instead of five demos glued together:

1. **Every physical node speaks the same packet format and joins the same mesh**, whether it's a helmet, a bare relay, or the flood sensor. The flood node's CRITICAL alert travels through exactly the same routing/failover logic as a worker's SOS.
2. **Real and simulated data enter the backend through the identical path** (MQTT topics). The flood scenario, a 14-worker site, and a mid-transmission node failure are all just different publishers on the same bus — this is the strongest architectural point to make to judges, because it proves the pipeline rather than the props.

---

## 2. Communication Layer — the central engineering decision

This is the feature judges will probe hardest, so the comparison has to be real, not decorative.

| | **ESP-NOW (custom mesh)** | **WiFi Mesh (painlessMesh)** | **LoRa (point-to-point / Meshtastic)** | **BLE / BLE Mesh** |
|---|---|---|---|---|
| Range (indoor, obstructed) | ~15–40m | ~15–40m | 500m–several km | ~5–15m |
| Throughput | Low, small packets (≤250B) | Higher (TCP over WiFi) | Very low (kbps) | Low |
| Power draw | Low–medium | Medium–high (WiFi radio stays up) | Very low (duty-cycled) | Low |
| Native multi-hop | No — you write it | Yes (library handles it) | No — point-to-multipoint only, mesh needs extra firmware/protocol | Only with BLE Mesh spec (complex) |
| Hardware cost/node | ESP32 only (~$4–6) | ESP32 only | ESP32 + LoRa module (~$8–15 extra) + antenna | ESP32 only |
| Implementation complexity | Medium (you own the routing logic, but that's also full control) | Low–medium (library does self-healing) | High (new radio stack, range testing, antenna tuning) | High for real mesh |
| **Demo control** (can you force a failure/reroute exactly on cue?) | **High** — deterministic, you wrote it | Medium — library's internal healing timing is a black box | Low in the time available | N/A here |
| Fit for "break a node → judges watch it reroute" | **Best** | Good, less predictable | Would need too much new firmware to trust live | Not viable in this window |

**Recommendation: ESP-NOW as the radio, with a small custom application-layer routing protocol on top.**

Why not the "better" options:
- **WiFi mesh (painlessMesh)** is a legitimate alternative and less code to write, but you're trading control for convenience — its self-healing timing isn't yours to tune, which is risky when the entire Demo 3 depends on the reroute happening within a judge's attention span. If your team already knows painlessMesh well, it's an acceptable swap; otherwise don't learn a new library and a new protocol in the same 48 hours.
- **LoRa/Meshtastic** is the "sexiest" answer and genuinely fits a disaster narrative better (km-scale range), but it adds a second hardware type, an unfamiliar stack, and antenna/range testing you won't have time to debug if it goes wrong the night before. Flag it explicitly to the team and judges as **"the deployment-scale answer, out of scope for this prototype"** — that's an honest, technically credible statement, not a cop-out.
- **BLE mesh** is out — short range and the mesh spec is too heavy to implement correctly in this window.

### 2.1 Routing protocol ("SensoraMesh" — hackathon-scoped, hybrid)

This mirrors the idea behind Collection Tree Protocol (a well-known wireless-sensor-network pattern) for normal traffic, plus **flooding for anything safety-critical**, because flooding is trivially self-healing — it doesn't depend on any single path being correct, which is exactly the property you want on stage.

- Every node broadcasts a `HEARTBEAT` every 2–3s: `{nodeID, role, hopCountToGateway, battery, seq}`.
- Each node keeps a small neighbor table (`neighborID, RSSI, lastSeen, theirHopCount`) and computes `myHopCount = min(neighbor hopCount) + 1`, picking the neighbor with the lowest hop count (tie-break: strongest RSSI) as `bestNextHop`. This is a simple distance-vector/gradient routing toward the gateway.
- **Failure detection:** if no heartbeat from the current `bestNextHop` for 3 missed intervals (~9s), mark it stale and recompute from the remaining table. This 9s window is your reroute demo's visible "thinking" pause — don't try to make it instant, a visible-but-quick recovery actually reads better to judges than something so fast it looks scripted.
- **Normal telemetry** (NORMAL/WARNING state, low priority): unicast hop-by-hop via `bestNextHop`. Cheap, low airtime.
- **Emergency traffic** (CRITICAL/SOS, priority=1): **constrained flood** — broadcast to all neighbors, each relay forwards once (dedup cache keyed by `{srcID, seq}`, TTL-expired after a few seconds), hop-limited (e.g. TTL=8). This guarantees delivery via multiple simultaneous paths without waiting for route recomputation — the actual mechanism behind Demo 3.
- Gateway node bridges the mesh to the backend over **USB serial** (not WiFi) — this keeps the last hop deterministic during the demo (no dependency on venue WiFi/hotspot flakiness). The mesh itself is what's "resilient"; the laptop tether is a deliberate, disclosed simplification.

**If a judge says "isn't flooding just cheating instead of real routing?"** — answer: for safety-critical low-node-count networks, controlled flooding (TTL + dedup) is a legitimate, widely-used resilience pattern (it's the reliability mechanism behind Bluetooth Mesh and many disaster-relief mesh designs), traded deliberately against airtime efficiency. You're using the efficient path (distance-vector) for routine data and the robust path (flood) only for the traffic where guaranteed delivery matters more than efficiency — that's the "priority-aware" differentiator, not an accident.

### 2.2 Packet format

Fixed binary header to stay inside ESP-NOW's ~250-byte payload:

```
byte 0    : version
byte 1    : msgType        (0=HEARTBEAT, 1=TELEMETRY, 2=EMERGENCY, 3=ENV, 4=ACK)
byte 2    : srcID          (1-byte node address, fine for a demo-scale network)
bytes 3-4 : seq            (uint16, per-source sequence number — used for dedup + replay rejection)
byte 5    : ttl            (hop limit, decremented per forward)
byte 6    : hopCount       (hops traveled so far, for routing table math)
byte 7    : priority       (0=normal, 1=emergency)
byte 8    : payloadLen
bytes 9-9+len : payload    (type-specific, see below)
last 2 bytes : CRC16       (integrity check)
```

- `TELEMETRY` payload: `tempC (int16), gasRaw (uint16), motionFlag (uint8), batteryPct (uint8)`
- `EMERGENCY` payload: `hazardType (uint8 enum), severity (uint8 enum), tempC, gasRaw, motionFlag, workerID, zoneID` (no live GPS — see §3.4)
- `ENV` payload: `waterLevelCm (uint16), rainfallIntensity (uint16), rateOfRiseCmPerMin (int16), isSimulatedFlag (uint8)`

The gateway decodes this into JSON before it ever reaches the backend, so nothing downstream needs to know about the wire format.

### 2.3 Security (address this honestly, don't ignore it)

Full mesh security (per-hop TLS, key rotation) is out of scope for 48 hours. What's cheap and worth doing, and what to say about the rest:

- **Already free:** the `seq` field doubles as replay/duplicate protection — reject packets with a `seq` at or below the last seen for that `srcID`.
- **Cheap to add:** a pre-shared-key truncated HMAC (e.g. HMAC-SHA256 truncated to 1–2 bytes) appended per packet, keyed per-node, to stop trivial spoofed-SOS packets. A few lines of firmware, a real talking point.
- **Explicitly out of scope, say so on stage:** encrypted payloads, key rotation/provisioning, and TLS on the mesh hop. State the real-world mitigation (per-node provisioned keys, DTLS-style handshake) as "how this would harden for deployment" rather than leaving it unaddressed — judges respect a named gap far more than a silent one.
- Dashboard/API layer: basic auth token on REST + WS in this prototype; note that a deployable version needs proper auth/roles for supervisors vs. read-only viewers.

---

## 3. Hardware — per-node bill of materials

You said you have **a few ESP32 boards + basic sensors already** — treat "basic sensors" as DHT-family temp + maybe an MPU6050, and assume gas and water-level sensors still need sourcing (cheap, ~$2–5 each, widely stocked). Confirm this against what's actually in the box before hour 0.

### 3.1 Helmet / Worker Node
| Sensor | Purpose | Notes / risk |
|---|---|---|
| ESP32 dev board | Compute + radio | — |
| MPU6050 (I2C accel+gyro) | Fall/impact + abnormal-motion detection | Cheap, reliable, well-documented — lowest-risk sensor here |
| MQ-2 or MQ-135 (analog) | Gas/smoke presence | **Flag:** these need 24–48h burn-in for a stable baseline and are not calibrated to ppm. Detect *relative deviation from a power-on baseline*, not absolute concentration, and say so explicitly on the dashboard/pitch. Don't claim industrial-grade gas detection. |
| DHT22 or similar | Ambient temperature | Cheap, adequate for "excessive heat" demo |
| Push button (SOS) | Explicit emergency override | Debounce in firmware, use interrupt not polling |
| Buzzer + status LED (or small SSD1306 OLED if time allows) | Local feedback of NORMAL/WARNING/CRITICAL/SOS **even with zero connectivity** | This is what proves "risk assessed locally" — don't skip it for the mesh/dashboard and forget the helmet needs its own visible state |
| LiPo + TP4056 charge module + voltage divider into ADC | Power + battery % | Needed for the "Battery" field the brief explicitly asks the dashboard to show |
| GPS (NEO-6M) | Location | **Cut for the live demo unless you'll be outdoors with clear sky** — cold-start fix can take 30s–minutes and won't reliably lock indoors. Use fixed **Zone labels (Zone 1/2/3)** set per node instead; mention GPS as a designed-in, deployment-time capability. |

Rough power budget: ESP32 active + ESP-NOW ≈ 120–180mA, sensors add ~20–50mA → a 2000mAh LiPo gives roughly 8–12h continuous. Good enough to state a number if asked; don't over-promise beyond that.

### 3.2 Relay Node
Just an ESP32 running the mesh firmware in `RELAY` role, no sensors — this is the node you'll physically power off mid-demo. Add a single status LED so judges can see it die and later see a different relay light up as the new path.

### 3.3 Environmental / Flood Node
| Sensor | Purpose | Notes |
|---|---|---|
| Ultrasonic (HC-SR04) pointed down into a water container/tube | **Real, physical** water level measurement | Genuinely real sensor data — good to have at least one real physical signal in the flood demo, not everything simulated |
| Potentiometer (or a slider control) | Presenter-controlled **simulated** rainfall intensity | Transparent, honestly-labeled simulation the presenter can visibly turn up live — this satisfies "clearly label simulated data" *while still being an interactive physical demo*, better than a screen-only slider |
| ESP32 | Compute + radio | Same firmware family as other nodes, `ENV` role |

This hybrid (real water level + presenter-driven simulated rainfall) is a deliberate design choice: it's honest about what's simulated while still giving you something tangible to physically manipulate on stage, which reads much better than "trust me, the number changed in software."

### 3.4 Gateway Node
ESP32 in `GATEWAY` role, connected via **USB serial** to the laptop/Pi running the backend (see §2.1 for why serial over WiFi at this one hop). No sensors.

---

## 4. Risk Engine — edge (deterministic) + backend (ML)

Two layers, deliberately different in character:

### 4.1 Edge (on each helmet ESP32, C/C++, must be 100% reliable)

A small, explainable, weighted rule engine — not ML, and that's correct here, not a shortcut:

```
if SOS pressed:                          → SOS         (always overrides everything)
if fall detected AND no motion for Ns:   → CRITICAL    (impact spike, then near-zero variance)
score = w1*tempAnomaly + w2*gasDeviation + w3*motionAnomaly
if score > highThreshold:                → CRITICAL
elif score > medThreshold:                → WARNING
else:                                      → NORMAL
```

This runs and drives the local buzzer/LED even with **zero radio connectivity** — that's the point: risk assessment is local, communication is a separate, best-effort layer on top. Keep this deterministic; a safety-critical local state should not depend on a model that could behave unpredictably on stage.

### 4.2 Backend (Python/scikit-learn) — where "AI" is actually earned

- **Flood risk (primary ML component):** engineer features from the env-node stream — current water level, rate-of-rise over 1/5/15-minute rolling windows, rainfall intensity, accumulated rainfall — and train a small classifier (Gradient Boosted Trees or even a well-tuned Logistic/Ordinal Regression) on a **clearly-labeled simulated dataset** of many synthetic monsoon-style episodes, outputting NORMAL → WATCH → WARNING → CRITICAL with hysteresis (don't flip state on a single noisy sample). This is the strongest, most defensible ML claim in the system because the brief explicitly asks for exactly this multi-signal short-horizon estimate, and a trained model demonstrably beats a bare threshold on the synthetic data (bring that comparison chart to the demo).
- **Cross-worker anomaly detection (secondary, if time allows):** a simple z-score / Isolation Forest over the *current fleet's* readings — flags "multiple workers simultaneously showing elevated readings" as a site-wide pattern distinct from one worker's local spike. Cheap to build, genuinely differentiated, and a good example of fusion that a single edge device structurally cannot do on its own.
- **Do not** reach for deep learning anywhere in this system — every signal here is low-dimensional and well-served by classical ML; a judge asking "why not a neural net" should get "because it wouldn't outperform this on this data, and it would cost us interpretability we need for a safety system" as a confident answer, not a dodge.

Label every simulated-data-driven output on the dashboard, literally, as the brief requires (e.g. a small "SIMULATED MODEL" tag next to the flood risk badge).

---

## 5. Backend

**Stack:** FastAPI (Python) + MQTT (Mosquitto broker, `paho-mqtt` client) + SQLite (via SQLAlchemy) + WebSocket for the dashboard feed.

Why MQTT specifically, not just an in-process queue: it decouples ingestion from processing, it's the stack a judge will recognize as the "correct" IoT choice, and — most importantly — **it lets the simulator and the real gateway be two publishers on the identical topics**, so simulated flood/scale/failure scenarios flow through the exact same risk-engine and dashboard code path as real hardware. That architectural fact is worth stating out loud in the pitch.

**Topics:**
```
sensora/telemetry/{nodeId}
sensora/emergency/{nodeId}
sensora/env/{nodeId}
sensora/network/{nodeId}/heartbeat
sensora/network/topology        (backend-published, derived from heartbeats)
```

**Data flow:** Gateway (serial) → small Python reader script → publishes JSON to MQTT → FastAPI backend subscribes → runs risk fusion (§4.2) → writes to DB → pushes diffs to connected dashboards over WebSocket, and republishes derived events (state changes, reroutes, alerts) back onto MQTT for anything else that wants them (e.g. the simulator, for closed-loop scenario scripting).

**Database (SQLite is the right call here — zero ops, plenty for demo scale):**

```
workers(worker_id, name, role, last_seen, battery, status, zone, node_id)
nodes(node_id, role, last_seen, battery, rssi_to_next_hop, next_hop_id, hop_count, status)
telemetry(id, node_id, ts, temp, gas, motion_flag, battery, raw_json)
events(id, ts, node_id, worker_id, event_type, severity, hazard_type, message, acknowledged)
network_events(id, ts, event_type[NODE_UP|NODE_DOWN|ROUTE_CHANGE], node_id, detail_json)
env_readings(id, node_id, ts, rainfall_intensity, water_level_cm, rate_of_rise, risk_state, is_simulated)
```

**API surface:**
- `GET /workers`, `GET /nodes`, `GET /events`, `GET /env/latest` — initial dashboard load
- `WS /ws/live` — streaming diffs (worker status, network topology, events, env risk)
- `POST /events/{id}/ack` — supervisor acknowledgment (the brief's user-flow step 8)

Security for this layer: token-based auth on the REST/WS endpoints in the prototype; note real deployment needs per-role auth (supervisor vs. viewer) and TLS — same "named gap, stated mitigation" treatment as §2.3.

---

## 6. Dashboard

React + Vite (skip Next.js unless the frontend person already knows it well — it buys nothing extra at this scale and costs setup time) + native WebSocket client.

Component breakdown, matching the brief's mockup directly:

- **StatusBar** (always visible, top): workers online/total, network health badge, active emergency count, environmental risk badge — the "understand this in 3 seconds" requirement lives here.
- **WorkerPanel**: one row/card per worker — status **shape + color + text label** (not color alone, per the accessibility requirement), battery, zone, connectivity, last update.
- **NetworkTopology**: small fixed-layout node graph (5–8 nodes at demo scale — don't reach for a force-directed graph library, a hand-laid-out SVG mirroring the physical table arrangement of your boards is easier to get right and reads better to judges who can look at the table and the screen together). Node fill = status, edge = current `bestNextHop` link, redraw on `ROUTE_CHANGE` events.
- **EnvironmentalPanel**: water level + rainfall + rate-of-rise as simple gauges/sparklines, risk badge (NORMAL/WATCH/WARNING/CRITICAL), explicit "SIMULATED" tag on the rainfall input.
- **EventTimeline**: scrolling log, formatted exactly like the brief's example (`HH:MM:SS — event text`).
- **AlertBanner**: full-width, dominant, appears only when a CRITICAL/SOS event is active — this is the "ACTIVE ALERT" hero block from the mockup, and it should visually out-rank everything else on the page when present.

Visual language: dark control-room background, status colors (green/amber/orange/red) always paired with an icon + text label, minimal decoration — the brief is explicit that this should look like an operations center, not a student IoT dashboard. When you get to actually building this screen, load the `dataviz` and `artifact-design` skills for the color/accessibility/layout specifics rather than improvising them.

---

## 7. Build Plan — 36–48h, 4–6 people

Assumes specialization: 1–2 embedded/hardware, 1 comms/firmware, 1–2 backend, 1 frontend (adjust to your actual split, but keep at least one person owning comms full-time — it's the highest-risk piece).

**Phase 0 (hours 0–3) — de-risk, don't build features yet**
Flash a blink/hello-world to *every* ESP32 board you plan to use. Confirm Arduino/PlatformIO toolchain, library versions (ESP-NOW examples, MPU6050 lib) on every laptop that needs them. This phase's only job is finding out now which board or cable is dead, not at hour 40.

**Phase 1 (hours 3–16) — parallel tracks, no cross-dependencies yet**
- *Embedded:* single helmet node reading all sensors + local rule engine + local buzzer/LED state. Demo-1-critical-path, and it works standalone with zero network.
- *Comms:* two boards exchanging heartbeats, neighbor table, hop-count computation. Get failure-detection timing tuned in isolation before adding more nodes.
- *Backend/Frontend:* build the FastAPI + MQTT skeleton and the React dashboard shell **against a fake data generator**, so this track is never blocked waiting on hardware. This generator becomes your permanent simulator (§4.2, §5) — not throwaway code.

**Phase 2 (hours 16–28) — integration checkpoint 1**
Merge sensors + rule engine + comms into full helmet firmware. Extend the mesh to 3+ physical nodes (helmet, relay, gateway). Swap the backend's fake generator for the real gateway feed. Verify Demo 1 (hazard escalation) and Demo 2 (SOS) end-to-end with real hardware.

**Phase 3 (hours 28–38) — integration checkpoint 2**
Implement and rehearse the kill-a-relay reroute (Demo 3) — know in advance exactly which physical node gets unplugged and when. Bring up the flood/env node and wire its output into the dashboard (Demo 4).

**Phase 4 (hours 38–end minus buffer) — polish + rehearse**
Dashboard visual pass against §6 and the brief's mockup (Demo 5). Full run-through of the single narrative end-to-end, multiple times, with different people driving.

**Reserve the final 10–15% of total time purely for integration bugs and rehearsal.** This is the single most common reason hackathon demos fail — teams build until the last hour and never rehearse the failure-and-reroute moment, which is exactly the demo with the most moving parts. If Phase 4 is running short, cut dashboard polish before you cut rehearsal time.

**Fallback plan:** record a clean run of the full demo (screen capture + phone video of the physical rig) by the start of Phase 4, in case live hardware misbehaves in front of judges. Never present this as the primary demo, but have it ready.

---

## 8. What a judge will challenge — say this before they ask

- **"Your gas sensor readings aren't real ppm values."** Correct — MQ-series sensors need days of burn-in for calibrated absolute readings. We detect deviation from a power-on baseline, and we say so on the dashboard. This is a deliberate, disclosed scope cut, not an oversight.
- **"GPS isn't live."** Correct, and intentionally cut — GPS cold-start doesn't reliably lock indoors within a demo window. We use fixed zone labels; GPS is a designed-in field the firmware/packet already supports for outdoor deployment.
- **"Isn't flooding just avoiding real routing?"** No — it's a deliberate reliability/efficiency trade-off: distance-vector for routine telemetry, flood-with-TTL for anything safety-critical, because guaranteed delivery matters more than airtime for an SOS. This is a known pattern in disaster-relief and sensor-network mesh design, not an ad hoc shortcut.
- **"Your flood model is trained on data you made up."** Correct, and labeled as such live on the dashboard. It proves the pipeline — real sensor → features → model → calibrated alert → resilient transmission — works end to end; it does not claim validated flood science. That's explicitly the brief's own instruction for this feature.
- **"Why not just use LoRaWAN/Meshtastic/existing infrastructure?"** Those solve long-range radio connectivity, not the vertical integration this system provides: multi-sensor fusion at the edge, a safety state computed locally with zero connectivity, and priority-aware routing tailored to an emergency packet format. The differentiator is the whole chain, not the radio.
- **"Does this scale past your demo's node count?"** Not as built — 1-byte node addressing, an in-memory backend state, and SQLite cap out well before hundreds of nodes. That's a known, named ceiling; a real deployment would move to a proper mesh routing protocol (e.g. something in the B.A.T.M.A.N./Zigbee family), a real broker cluster, and a production database. Say this proactively; it reads as engineering maturity, not weakness.
- **"What's the actual battery life?"** ~8–12 hours on a 2000mAh LiPo at the current draw estimate (§3.1) — enough to state with confidence, not enough to over-promise a multi-day deployment without a bigger battery or duty-cycling the radio.

---

## 9. Open items to confirm before Phase 0

1. Exact contents of the "basic sensors" already on hand — confirm against §3 before assuming what still needs sourcing (gas sensor and water-level sensor are the most likely gaps).
2. Confirm venue realities: will you have table space to physically lay out helmet/relay/env/gateway nodes in a fixed arrangement matching the on-screen topology? This materially simplifies §6's topology panel.
3. Decide now who owns the comms/firmware track full-time — of everything in this document, it's the piece with the least forgiving debugging loop, and it should not be a shared/rotating responsibility.
