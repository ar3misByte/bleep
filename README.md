# Bleep

Worker safety monitoring for underground mine sites — wall-node sensors detect falls and prolonged inactivity, and a live dashboard surfaces alerts to site supervisors.

## Dashboard

[`index.html`](./index.html) is the finished dashboard — open it directly in a browser, or serve the repo with GitHub Pages. It's built with plain HTML/CSS/JS (no build step).

- **Fall & Inactivity Detection** (top section) is the one feature that's actually implemented and wired to live data on the real device — everything on this page is simulated for the demo, but the "Demo & Testing Controls" panel drives real interactive state (trigger a fall, simulate movement, reset).
- Everything below the divider (**Site Coverage**, **Device Security**) is illustrative UI for planned/future panels, framed in mine-safety terms.

See [`dashboard_features.md`](./dashboard_features.md) for the feature-by-feature status tracker (`LIVE` / `WIRED` / `PLANNED`) that the dashboard is kept in sync with.

## Connecting real hardware

The dashboard talks to hardware through a small relay server, not directly — the wall node is an HTTP **client**, the dashboard machine runs the HTTP **server**.

```
[worker wearable] --ESP-NOW--> [wall node ESP32] --WiFi/HTTP POST--> [server/app.py] --serves--> [index.html in your browser]
```

1. **Put the wall node and your dashboard machine on the same WiFi network**, and find your machine's local IP (`ipconfig` on Windows, look for the WiFi adapter's IPv4 address — e.g. `192.168.1.50`).
2. **Edit [`firmware/wall_node.ino`](./firmware/wall_node.ino)**: set `WIFI_SSID`, `WIFI_PASSWORD`, and `DASHBOARD_URL` (`http://<your-ip>:5000/api/telemetry`) at the top, then flash it to the wall node's ESP32. It receives `WorkerPacket` structs over ESP-NOW from the wearable and forwards them as the telemetry JSON described in [`wifi_json_protocol.md`](./wifi_json_protocol.md).
3. **Run the relay server**:
   ```bash
   pip install -r server/requirements.txt
   python server/app.py
   ```
4. **Open the dashboard from the server, not the static file**: go to `http://<your-ip>:5000/` in a browser (on the same machine or anywhere else on the LAN). The page polls `/api/workers` and `/api/events` every 2 seconds — as soon as the wall node sends its first message, the page switches out of demo mode automatically: the worker roster, alert banner, and event log start showing real data, and the "Demo & Testing Controls" panel disables itself.

**Why not just open `index.html` directly, or use the GitHub Pages copy?** Browsers block a page from fetching a plain-`http://` endpoint when the page itself was loaded over `https://` (GitHub Pages) or as a bare local file with no server behind it — the fetches to `/api/workers` fail silently and the page just stays in demo mode. Serving `index.html` from `server/app.py` (step 4) puts the page and the API on the same origin, which sidesteps this entirely. The hosted GitHub Pages copy is fine for showing the design, but it will never show live hardware data.

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
