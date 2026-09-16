# Sensora — Wall Node → Dashboard WiFi/JSON Protocol (Feature 1)

**Scope:** this covers only what Feature 1 (fall/inactivity detection) needs — the wall node's ESP-NOW link to the worker is unchanged from the build checklist; this document is specifically the **wall node → dashboard** hop, now over WiFi instead of USB serial. The formal schema is `sensora_telemetry.schema.json` (same folder) — treat it as the source of truth; this doc explains how to use it.

---

## 1. Transport

The **wall node is the HTTP client**; the **dashboard laptop runs a small HTTP server**. The wall node POSTs one JSON object per event/update — this is push, not polling, so a DISTRESS message reaches the dashboard the instant it happens, with no delay waiting on a poll interval.

- Endpoint: `POST http://<dashboard-laptop-ip>:5000/api/telemetry`
- Header: `Content-Type: application/json`
- Body: one JSON object matching the schema (§2)
- Expected response: `200 OK` with a small JSON body — the wall node doesn't need to do anything with it, but checking the status code tells you whether the send actually worked

**Before writing any firmware for this:** put the wall node and the dashboard laptop on the **same WiFi network**, find the laptop's IP (`ipconfig` on Windows / `ifconfig` or `ip addr` on Mac/Linux — look for the WiFi adapter's address, e.g. `192.168.1.50`), and confirm you can `curl` that endpoint from another machine on the same network before you flash anything. A firewall blocking inbound connections on port 5000 is a common, easy-to-miss blocker — test this first.

---

## 2. Message shape

Every message has the same envelope; `payload` differs by `msgType`. Full field-by-field detail with descriptions is in `sensora_telemetry.schema.json` — this section just shows what it looks like on the wire.

**STATUS** (recommended addition — send every ~2s so the dashboard can show "worker OK" instead of going silent between events):
```json
{
  "schemaVersion": 1,
  "msgType": "STATUS",
  "workerId": "W1",
  "nodeId": "WALL1",
  "seq": 184,
  "espnowRssi": -58,
  "timestamp": 1928433,
  "payload": {
    "riskState": "OK",
    "motionEnergy": 14.2,
    "secondsSinceMotion": 2,
    "battery": null
  }
}
```

**DISTRESS** (required — this is what the build checklist's Feature 1 already triggers; matches `{workerId, msgType=DISTRESS, timestamp}` from that plan, just fleshed out into the full envelope):
```json
{
  "schemaVersion": 1,
  "msgType": "DISTRESS",
  "workerId": "W1",
  "nodeId": "WALL1",
  "seq": 185,
  "espnowRssi": -61,
  "timestamp": 1958712,
  "payload": {
    "hazardType": "FALL_SUSPECTED",
    "riskState": "FALL_SUSPECTED",
    "secondsInactive": 32.4,
    "motionEnergyAtTrigger": 0.3,
    "battery": null
  }
}
```

**Why `battery: null` instead of leaving the field out:** the dashboard code you write today should not need to change when battery monitoring gets added later — it should already know to expect a `battery` key and just render "—" for null. Same reasoning for `riskState` being treated as an open string rather than a hardcoded switch/enum on the dashboard side: you will add `WARNING`, `CRITICAL`, `SOS`, `OFFLINE` etc. as later features land, and the dashboard shouldn't need a code change every time.

**`timestamp` is `millis()`, not wall-clock time.** The wall node doesn't need NTP sync for this feature — that's real complexity for no real benefit at hackathon scale. Instead, have the **dashboard** stamp its own arrival time (`serverReceivedAt = now()`) the moment a message lands, and use that for anything you display, log, or put on the event timeline. Use `seq` (not `timestamp`) if you need to detect a dropped or out-of-order message.

---

## 3. Wall node (ESP32) — sending side

Requires the `ArduinoJson` library (Library Manager → search "ArduinoJson", use the v6 API shown here) and WiFi already connected (`WiFi.begin(ssid, password)` in `setup()`, standard pattern, not shown).

```cpp
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* DASHBOARD_URL = "http://192.168.1.50:5000/api/telemetry"; // set to the laptop's actual IP

bool sendToDashboard(JsonDocument& doc) {
  if (WiFi.status() != WL_CONNECTED) return false; // don't block waiting on a dead WiFi link
  HTTPClient http;
  http.begin(DASHBOARD_URL);
  http.addHeader("Content-Type", "application/json");
  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  http.end(); // always release the connection, success or failure
  return code == 200;
}

void sendDistress(const char* workerId, uint32_t seq, int rssi,
                   float secondsInactive, float motionAtTrigger) {
  StaticJsonDocument<256> doc;
  doc["schemaVersion"] = 1;
  doc["msgType"] = "DISTRESS";
  doc["workerId"] = workerId;
  doc["nodeId"] = "WALL1";
  doc["seq"] = seq;
  doc["espnowRssi"] = rssi;
  doc["timestamp"] = millis();
  JsonObject payload = doc.createNestedObject("payload");
  payload["hazardType"] = "FALL_SUSPECTED";
  payload["riskState"] = "FALL_SUSPECTED";
  payload["secondsInactive"] = secondsInactive;
  payload["motionEnergyAtTrigger"] = motionAtTrigger;
  payload["battery"] = nullptr;

  // A dropped DISTRESS message is not acceptable — retry a few times, unlike STATUS below.
  for (int attempt = 0; attempt < 3; attempt++) {
    if (sendToDashboard(doc)) break;
    delay(300); // short backoff; this is the one place a brief delay() is acceptable —
                // it's on the DISTRESS path, not blocking the routine sensor loop
  }
}

void sendStatus(const char* workerId, uint32_t seq, int rssi,
                const char* riskState, float motionEnergy, float secondsSinceMotion) {
  StaticJsonDocument<256> doc;
  doc["schemaVersion"] = 1;
  doc["msgType"] = "STATUS";
  doc["workerId"] = workerId;
  doc["nodeId"] = "WALL1";
  doc["seq"] = seq;
  doc["espnowRssi"] = rssi;
  doc["timestamp"] = millis();
  JsonObject payload = doc.createNestedObject("payload");
  payload["riskState"] = riskState;
  payload["motionEnergy"] = motionEnergy;
  payload["secondsSinceMotion"] = secondsSinceMotion;
  payload["battery"] = nullptr;
  sendToDashboard(doc); // fine to just drop this one on failure — another STATUS follows in ~2s
}
```

Note the asymmetry: DISTRESS gets retries because losing it matters; STATUS doesn't, because the next one is seconds away. Don't apply the same retry logic to both — retrying every STATUS send on a flaky WiFi link will stall your main loop.

---

## 4. Dashboard — minimal receiving side (Python/Flask, enough to test against today)

```python
from flask import Flask, request, jsonify
from datetime import datetime, timezone

app = Flask(__name__)
latest_by_worker = {}   # workerId -> last message received, with server-side arrival time

@app.route("/api/telemetry", methods=["POST"])
def telemetry():
    msg = request.get_json(force=True)
    msg["serverReceivedAt"] = datetime.now(timezone.utc).isoformat()
    latest_by_worker[msg["workerId"]] = msg
    print(f"[{msg['msgType']}] {msg['workerId']}: {msg['payload']}")
    return jsonify({"ok": True}), 200

@app.route("/api/workers", methods=["GET"])
def workers():
    return jsonify(latest_by_worker)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
```

Test it stand-alone before involving any hardware:
```bash
curl -X POST http://localhost:5000/api/telemetry \
  -H "Content-Type: application/json" \
  -d '{"schemaVersion":1,"msgType":"DISTRESS","workerId":"W1","nodeId":"WALL1","seq":1,"espnowRssi":-60,"timestamp":1000,"payload":{"hazardType":"FALL_SUSPECTED","riskState":"FALL_SUSPECTED","secondsInactive":30,"motionEnergyAtTrigger":0.2,"battery":null}}'
```
If this doesn't print in your Flask console, the problem is your server/network setup — fix that before suspecting the ESP32 firmware.

---

## 5. Validating messages against the schema (optional, but catches mismatches fast)

If your dashboard is Node-based:
```js
const Ajv = require("ajv");
const schema = require("./sensora_telemetry.schema.json");
const ajv = new Ajv();
const validate = ajv.compile(schema);
// validate(msg) === true/false; validate.errors explains what's wrong
```
If it's Python:
```python
import jsonschema, json
schema = json.load(open("sensora_telemetry.schema.json"))
jsonschema.validate(instance=msg, schema=schema)  # raises on mismatch
```
This is worth wiring in early — it turns "the dashboard silently shows nothing" into a clear error naming the exact field that's wrong, which is a much faster debugging loop than comparing JSON by eye.

---

## 6. What changes when you add the next feature

Don't redesign the envelope. When Feature 2 (proximity snapshot) lands, add `distanceCm` and `yawDeg` to the DISTRESS payload and bump `schemaVersion` to `2`; when battery monitoring lands, start sending a real number instead of `null` — the field is already there. This is the entire point of shipping `null` placeholders now instead of omitting fields.
