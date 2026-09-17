# Bleep

Worker safety monitoring for underground mine sites — a worker-worn ESP32 detects falls, prolonged inactivity, and environmental/vitals hazards, and a live dashboard surfaces alerts to site supervisors.

## Sensors on the worker unit

- **MPU6050** — 6-axis IMU: fall & impact detection, prolonged-inactivity detection
- **DHT11** — ambient temperature & humidity (heat-stress alerting)
- **BMP180** — barometric pressure
- **MQ-135** — toxic/air-quality gas
- **MQ-4** — combustible gas (methane/LPG) — the most safety-critical reading underground
- **MAX30102** — heart rate & an approximate SpO2 estimate
- **Soil moisture sensor** — floor-level water-ingress/flooding proxy
- **Physical SOS button** — manual worker-triggered distress

Every one of these feeds the same latched-alarm path (LED + buzzer on, dashboard shows a live DISTRESS event, cleared by pressing the SOS button again to acknowledge) — see [`dashboard_features.md`](./dashboard_features.md) for the full breakdown and current threshold values.

## Dashboard

[`index.html`](./index.html) is the dashboard — open it directly in a browser, or serve the repo with GitHub Pages. It's built with plain HTML/CSS/JS (no build step).

- **Worker roster cards** show live per-worker data wired to the real ESP32 hardware: motion energy, RSSI, temperature/humidity/pressure, gas readings, heart rate/SpO2, and soil moisture — the last four colored red once they cross the same danger threshold the firmware alarms on.
- **Event log** and **alert banner** cover every distress source (fall, SOS, inactivity, heat, gas, heart rate, SpO2, water ingress) generically, since `riskState`/`hazardType` are open strings, not a fixed enum.

See [`dashboard_features.md`](./dashboard_features.md) for the feature-by-feature status tracker (`LIVE` / `WIRED` / `PLANNED`) that the dashboard is kept in sync with.

### The three dashboard files

There are three HTML dashboards in this repo, not one — they're different snapshots, not alternatives to pick between at random:

- [`index.html`](./index.html) — the relay-server-connected dashboard described above. This is the one you actually serve from `server/app.py` to see real hardware.
- [`sensora_watch.html`](./sensora_watch.html) — the standalone, no-server "Sensora Watch" command dashboard (seven tabs: Overview, Workers, Alerts, Vitals & Gas, Mine Map, Reports, Admin) published as a Claude Artifact; see [`dashboard_features.md`](./dashboard_features.md) for what it does. Runs entirely on simulated data — open it directly in a browser.
- [`bleep_watch.html`](./bleep_watch.html) — the same command dashboard, rebranded ("Bleep Command" instead of "Sensora Watch") and with one addition: a **Live Data** tab that polls a wall node's own `/api/workers` endpoint directly (see "Second node pair" below) and shows the raw, unprocessed feed, so it's obvious where real hardware readings appear instead of them blending in with simulated worker cards. Configure the wall node address under Admin → Live Hardware.

## Connecting real hardware

The dashboard talks to hardware through a small relay server, not directly — the wall node is an HTTP **client**, the dashboard machine runs the HTTP **server**.

```
[worker wearable] --ESP-NOW--> [wall node ESP32] --WiFi/HTTP POST--> [server/app.py] --serves--> [index.html in your browser]
```

0. **Install the worker's Arduino libraries** (Library Manager): `Adafruit MPU6050`, `Adafruit Unified Sensor`, `DHT sensor library` (Adafruit), `Adafruit BMP085 Library` (covers BMP180), and `SparkFun MAX3010x Pulse and Proximity Sensor Library` (provides `MAX30105.h` and `heartRate.h`, used for the MAX30102). MQ-135, MQ-4, and the soil moisture sensor are plain `analogRead()` — no library needed.
1. **Put the wall node and your dashboard machine on the same WiFi network**, and find your machine's local IP (`ipconfig` on Windows, look for the WiFi adapter's IPv4 address — e.g. `192.168.1.50`).
2. **Edit [`firmware/wall_node.ino`](./firmware/wall_node.ino)**: set `WIFI_SSID`, `WIFI_PASSWORD`, and `DASHBOARD_URL` (`http://<your-ip>:5000/api/telemetry`) at the top, then flash it to the wall node's ESP32. It receives `WorkerPacket` structs over ESP-NOW from the wearable and forwards them as the telemetry JSON described in [`wifi_json_protocol.md`](./wifi_json_protocol.md).
3. **Run the relay server**:
   ```bash
   pip install -r server/requirements.txt
   python server/app.py
   ```
4. **Open the dashboard from the server, not the static file**: go to `http://<your-ip>:5000/` in a browser (on the same machine or anywhere else on the LAN). The page polls `/api/workers` and `/api/events` every 2 seconds — as soon as the wall node sends its first message, the page switches out of demo mode automatically: the worker roster, alert banner, and event log start showing real data, and the "Demo & Testing Controls" panel disables itself.

**Don't have wearable hardware yet?** Flash [`firmware/wearable_fall_simulator.ino`](./firmware/wearable_fall_simulator.ino) to a second ESP32 instead — it stands in for the wearable, sending STATUS packets every 2s and a simulated fall roughly every 45s (auto-recovering 15s later), so you can test the wall node → server → dashboard pipeline with no accelerometer involved.

**Why not just open `index.html` directly, or use the GitHub Pages copy?** Browsers block a page from fetching a plain-`http://` endpoint when the page itself was loaded over `https://` (GitHub Pages) or as a bare local file with no server behind it — the fetches to `/api/workers` fail silently and the page just stays in demo mode. Serving `index.html` from `server/app.py` (step 4) puts the page and the API on the same origin, which sidesteps this entirely. The hosted GitHub Pages copy is fine for showing the design, but it will never show live hardware data.

## Second node pair — tinyML anomaly detection ([`firmware/wall_n/`](./firmware/wall_n/), [`firmware/worker_n/`](./firmware/worker_n/))

[`firmware/wall_node.ino`](./firmware/wall_node.ino) and [`firmware/worker_node.ino`](./firmware/worker_node.ino) are left untouched. `wall_n.ino` and `worker_n.ino` are a **second, independent worker/wall pair** — not a replacement — that adds two things:

- **Its own local sensors on the wall node** — an MPU6050, BMP180, and DHT11 for room/wall-mounted environmental readings, plus an HC-SR04 ultrasonic sensor for distance/proximity (door, hazard boundary, or clearance sensing). See the pin map at the top of [`wall_n.ino`](./firmware/wall_n/wall_n.ino).
- **A tinyML anomaly-detection model on the worker node**, replacing hand-picked hazard thresholds with a small trained autoencoder (9 → 6 → 9) baked into `tinyml_model.h`. The model learns what a *normal* combination of the nine worker sensor readings looks like and flags a worker anomalous when the network can no longer reconstruct their current readings accurately — this naturally accounts for cross-sensor correlations (e.g. "high heart rate is only normal alongside high motion energy") that independent static thresholds can't express. Inference is plain float matrix math on-device, no TensorFlow Lite Micro runtime required.

`worker_n.ino`'s fall-detection and alarm logic is ported unchanged from a hand-tested standalone prototype — see the comment block at the top of [`worker_n.ino`](./firmware/worker_n/worker_n.ino) for exactly what carried over and what was intentionally dropped (e.g. there is no acknowledge button; the alarm stays latched until reset).

### Retraining the model ([`ml/train_tinyml_model.py`](./ml/train_tinyml_model.py))

`tinyml_model.h` (in both `firmware/wall_n/` and `firmware/worker_n/` — same generated file, kept in sync) is generated by `ml/train_tinyml_model.py`. There is no logged real-world sensor history for this project yet, so the script currently trains on **synthetic data** sampled from the plausible "normal" ranges already documented as static thresholds in `worker_node.ino` (`TEMP_DANGER_C`, `MQ135_DANGER_RAW`, `HR_DANGER_*`, etc.) plus typical sensor datasheet ranges. Treat this as a placeholder, not a substitute for real data — re-run the script against logged real sensor values as soon as they exist so the model learns the actual normal envelope for your site and workers. Run with `python ml/train_tinyml_model.py`; it has no dependency on the rest of the codebase beyond regenerating the header.

## PCB reference designs ([`kicad/`](./kicad/))

Vendored third-party reference material, not designs authored for this project — currently just Adafruit's own open-source [MPU6050 breakout PCB](https://www.adafruit.com/product/3886) (EagleCAD schematic/board files, CC BY-SA per Adafruit's license). Kept here as a starting reference for anyone laying out a custom PCB for the worker unit instead of prototyping on a breadboard.

## Design source

The `Main.dc.html` + `canvas.json` pair is the source for the same dashboard as an editable [Claude Design](https://claude.ai/design) canvas.

## Project docs

- [`Sensora_Hardware_Firmware_Spec.md`](./Sensora_Hardware_Firmware_Spec.md)
- [`wifi_json_protocol.md`](./wifi_json_protocol.md)
- [`sensora_telemetry.schema.json`](./sensora_telemetry.schema.json)
- [`sensora_architecture.md`](./sensora_architecture.md)
- [`Sensora_Feature_Build_Checklist.docx`](./Sensora_Feature_Build_Checklist.docx)
- [`Sensora_Novelty_Analysis.docx`](./Sensora_Novelty_Analysis.docx)
- [`Problem_Statement_SENSORA.pdf`](./Problem_Statement_SENSORA.pdf)
