# Sensora Watch — Dashboard Feature Tracker

Live artifact: **Sensora Watch** — https://claude.ai/artifact/MbasRczLbHmSZs2SWE92Aj

This file tracks what the dashboard actually does, feature by feature, matching the "implement on hardware, then wire into dashboard" workflow. Update this doc every time a new feature lands on the dashboard — don't let it drift from the live artifact.

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

## Other basic features (from the build checklist, not yet on the dashboard)

- SOS button (manual worker-triggered distress)
- Local buzzer/LED on the worker unit
- Battery monitoring (schema field already reserved as `null`)
- Worker heartbeat / liveness beyond the 15s staleness check
- Wall-node watchdog / reboot detection
- Basic temp/gas sensing surfaced on the roster card
- Server-side persistent event log (currently in-memory in the demo Flask server)

---

## Design system reference (applies to all features, not just Feature 1)

- **Fonts:** Public Sans (headers/nav), Inter (body/UI), Roboto Mono with `tabular-nums` (all numeric/timestamp data), Atkinson Hyperlegible (alert banner only — highest-urgency text on the page).
- **Status colors (reserved, never decorative):** good `#0ca30c`, critical `#d03b3b`, offline `#8b93a0`. A neutral accent `#2a5f8a` is used for buttons/progress bars and never overlaps with status meaning.
- **Mode:** light mode only, by explicit design decision — no dark-theme block.
- New features should reuse these tokens rather than introducing new colors/fonts, so the roster/alert/event-log pattern stays consistent as Features 2 and 3 land.

---

*Keep this file in sync with the live artifact. When a feature moves from `LIVE` to `WIRED`, or a `PLANNED` feature ships, update its section here.*
