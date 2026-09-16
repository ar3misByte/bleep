# Bleep

Worker safety monitoring for underground mine sites — wall-node sensors detect falls and prolonged inactivity, and a live dashboard surfaces alerts to site supervisors.

## Dashboard

[`index.html`](./index.html) is the finished dashboard — open it directly in a browser, or serve the repo with GitHub Pages. It's built with plain HTML/CSS/JS (no build step).

- **Fall & Inactivity Detection** (top section) is the one feature that's actually implemented and wired to live data on the real device — everything on this page is simulated for the demo, but the "Demo & Testing Controls" panel drives real interactive state (trigger a fall, simulate movement, reset).
- Everything below the divider (**Site Coverage**, **Device Security**) is illustrative UI for planned/future panels, framed in mine-safety terms.

See [`dashboard_features.md`](./dashboard_features.md) for the feature-by-feature status tracker (`LIVE` / `WIRED` / `PLANNED`) that the dashboard is kept in sync with.

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
