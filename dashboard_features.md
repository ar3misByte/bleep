# Bleep — Dashboard Feature Tracker

Live artifact: **Sensora Watch** — https://claude.ai/artifact/MbasRczLbHmSZs2SWE92Aj

This file tracks what the dashboard actually does, feature by feature, matching the "implement on hardware, then wire into dashboard" workflow. Update this doc every time a new feature lands on the dashboard — don't let it drift from the live artifact.

**As of the latest publish, the artifact is a full command dashboard, not just the Feature 1 demo described below.** It now has seven tabbed sections — Overview, Workers, Alerts, Vitals & Gas, Mine Map, Reports, Admin — built for reference against MineGuard/MineGaurd-Pro/IoT-Safety-Monitoring-System's feature sets (worker registration, a 0–10 risk score, an alert history with acknowledge, per-sensor gauge bands against the tinyML thresholds, a schematic mine map with wall-node anchors, a generated master report, and admin-editable alarm thresholds). Everything below this point documents Feature 1/4 specifically (still accurate for the roster-card and hazard-latching behavior it describes) — it hasn't been rewritten section-by-section for the wider dashboard yet. Key points about the new build:

- **Data model** extends the schema below with a worker registry (`id`, `name`, `role`, `nodeId`) and a wall-node registry (`id`, `label`, `zone`, map `x`/`y`), both persisted to `localStorage` as a per-viewer convenience only — this is demo-grade persistence, not a real backend, and does not sync across devices or reach Claude.
- **Risk score** (0–10, shown fleet-wide as "Average Risk Score" and per-worker) is a dashboard-only indicator computed from proportional excess over each admin-editable threshold, plus inactivity and the tinyML anomaly score — it is not the same thing as the firmware's own latched pass/fail alarm logic, and changing a threshold in Admin never changes what the hardware itself alarms on.
- **Dummy data**: two demo workers (W1/W2) ship pre-registered against WALL1/WALL2 with a simulated STATUS ticker (default every 3s, editable in Admin) standing in for real ESP-NOW traffic, plus one seeded warning alert and one scripted demo distress ~9s after load so the page never opens looking empty. All of it is clearly a simulation, not real telemetry, and the "SIMULATED DATA" flag in the utility bar is clickable to pause/resume it.
- **Reports**: "Generate master report" builds a fleet-summary + per-worker + alert-history report in-page; "Print / Save as PDF" uses the browser print dialog (print-only CSS isolates the report sheet) and "Copy report text" uses the clipboard API with a manual-select textarea fallback — there is no file-download button, since the artifact sandbox blocks script-driven downloads.

---

## Status legend

- `LIVE` — implemented in the dashboard today, running on simulated data
- `WIRED` — implemented and confirmed working against a real ESP32 over WiFi
- `PLANNED` — designed (schema/protocol exists) but not yet in the dashboard UI

---

## Feature 1 — Fall & Inactivity Detection

**Status: `LIVE` (simulated data only — see "Known limitation" below)**

What's on screen:

- **Utility bar** — dark-navy top strip with Sensora branding, a live clock (updates every 1s), and a "SIMULATED DATA" flag so nobody mistakes demo output for a real feed.
- **Header** — page title, a scope chip reading "Feature 1 · Fall & Inactivity Detection" (keeps the demo honest about what's implemented vs. planned), and a wall-node link status indicator.
- **Stat tiles (4)** — Workers Monitored, Active Alerts, Network status, Last Update. Pulls from the same message stream every other panel uses, so nothing can go out of sync.
- **Alert banner** — appears only while at least one worker is in `FALL_SUSPECTED`; set in Atkinson Hyperlegible for maximum legibility on the highest-urgency element on the page. Disappears automatically once the event clears.
- **Worker roster** — one card per worker (currently W1). Each card shows: status pill (`OK` / `FALL SUSPECTED` / `OFFLINE`), current motion energy, ESP-NOW RSSI to the wall node, battery (renders "—" until battery reporting ships), last sequence number, and an inactivity-timer progress bar that fills toward the trigger threshold.
- **Event log** — scrollable, newest-first list of every STATUS/DISTRESS message received, each stamped with the dashboard's own arrival time (not the device's `millis()` — see protocol notes).
- **Demo & Testing Controls** — a dashed-border panel, visually separated from the "real" dashboard, with three buttons: Trigger fall (W1), Simulate movement (W1), Reset demo. Lets anyone demo the distress flow live without hardware in the room.

Data flow: every inbound message (simulated or real) goes through one function, `handleIncomingMessage(msg)`, which matches the wall-node JSON schema exactly (`schemaVersion`, `msgType`, `workerId`, `nodeId`, `seq`, `espnowRssi`, `timestamp`, `payload`). A worker is marked `OFFLINE` if no message has arrived in 15s. An ambient auto-tick every 3s keeps the page feeling alive between demo button presses.

**Known limitation:** the dashboard is published as a hosted HTTPS page. Browsers block an HTTPS page from fetching a plain-HTTP endpoint (mixed content) — so this hosted link cannot pull from the local Flask server described in `wifi_json_protocol.md` even on the same LAN. To go from `LIVE` to `WIRED`: run `sensora_watch.html` locally (not the hosted link) and add a `fetch('http://<laptop-ip>:5000/api/workers')` poll into `handleIncomingMessage()`. This is documented in more detail in `dashboard-feature1.md`.

---

## Feature 2 — Proximity / LiDAR-like Mapping

**Status: `PLANNED`**

Not yet on the dashboard. Build checklist scopes this down to a "proximity snapshot" (HC-SR04 distance + gyro-integrated yaw) rather than a full scan. Dashboard work for this hasn't started — expect a new panel (likely a simple radial/polar plot per worker) once the hardware side sends `distanceCm`/`yawDeg`. Protocol note: this is meant to extend the existing DISTRESS payload and bump `schemaVersion` to `2`, not create a parallel envelope.

## Feature 3 — Worker In/Out Counting

**Status: `PLANNED` — blocked on hardware (2 additional beam sensors needed at the mine entrance)**

Not yet on the dashboard. Once unblocked, expect a small counter tile (workers currently underground) added to the stat-tile row, likely fed by the wall node nearest the entrance rather than a change to the worker-worn payload.

## Feature 4 — Environmental & Vitals Hazard Sensing

**Status: `LIVE` (wired to real ESP32 hardware, same as Feature 1)**

Adds six more sensors to `worker_node.ino`, reusing the exact same latched-alarm pattern as fall detection and the SOS button (`triggerHazardAlert()`, a generalization of what used to be separate `triggerSOS()`/`triggerInactivityAlert()` implementations):

- **DHT11** — temperature & humidity. Sustained temperature ≥ 40°C latches a `HEAT_STRESS` alert.
- **BMP180** — barometric pressure, shown as a live reading (no alert threshold yet).
- **MQ-135** — toxic/air-quality gas (raw ADC). Sustained reading ≥ 4000 latches a `GAS_DANGER` / `TOXIC_GAS` alert.
- **MQ-4** — combustible gas / methane (raw ADC), the most safety-critical reading underground. Sustained reading ≥ 3000 latches `GAS_DANGER` / `COMBUSTIBLE_GAS`.
- **MAX30102** — heart rate and an approximate SpO2 estimate. Sustained HR ≥ 130 or ≤ 45 bpm, or SpO2 ≤ 90%, each latch their own alert.
- **Soil moisture** — floor-level water ingress / flooding proxy. Sustained reading ≥ 80% latches `WATER_INGRESS`.

All eight readings ride on every STATUS and DISTRESS packet regardless of which hazard (if any) is active, so the worker roster cards always show live gauges. Thresholds are starting values only — see `TEMP_DANGER_C` and friends near the top of `worker_node.ino`, and tune against real sensor behavior on-site. Every hazard requires `REQUIRED_HAZARD_SAMPLES` (5) consecutive breaching samples before latching, mirroring the existing impact-detection debounce, so a single noisy ADC read can't trigger a false alarm.

On the dashboard, each worker card now shows Temperature, Humidity, Pressure, Gas (MQ-135 / MQ-4), Heart rate / SpO2, and Soil moisture, with the value turned red once it crosses the same threshold the firmware alarms on — a supervisor can see a reading trending toward danger before it actually fires.

Not implemented from the reference projects this was scoped against (no matching hardware on this build): RFID/UWB/LoRa zone tracking, GPS/subsidence physics, an AI chat assistant, or LCD/haptic on-device displays.

## Other basic features (from the build checklist, not yet on the dashboard)

- Battery monitoring (schema field already reserved as `null`)
- Worker heartbeat / liveness beyond the 15s staleness check
- Wall-node watchdog / reboot detection
- Server-side persistent event log (currently in-memory in the demo Flask server)

---

## Design system reference (applies to all features, not just Feature 1)

- **Fonts:** Public Sans (headers/nav), Inter (body/UI), Roboto Mono with `tabular-nums` (all numeric/timestamp data), Atkinson Hyperlegible (alert banner only — highest-urgency text on the page).
- **Status colors (reserved, never decorative):** good `#0ca30c`, critical `#d03b3b`, offline `#8b93a0`. A neutral accent `#2a5f8a` is used for buttons/progress bars and never overlaps with status meaning.
- **Mode:** light mode only, by explicit design decision — no dark-theme block.
- New features should reuse these tokens rather than introducing new colors/fonts, so the roster/alert/event-log pattern stays consistent as Features 2 and 3 land.

---

*Keep this file in sync with the live artifact. When a feature moves from `LIVE` to `WIRED`, or a `PLANNED` feature ships, update its section here.*
