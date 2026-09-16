# Sensora — dashboard relay server
#
# Receives wall-node telemetry over HTTP and serves the dashboard
# (index.html) itself, so the browser's fetch() calls are same-origin
# (no HTTPS/mixed-content or CORS problems on the local network).
#
# NOTE: this is the ROUTER-BASED path (wall node joins your WiFi
# network and POSTs here). For the peer-to-peer path -- wall node
# hosts its own network directly, no router/hotspot at all -- see
# firmware/wall_node.ino, which now runs its own copy of this same
# API and dashboard on the ESP32 itself. Both speak the same
# ageMs-based contract (see below) so this file and that firmware
# are interchangeable from the dashboard's point of view.
#
# Run:
#   pip install flask
#   python server/app.py
# Then open http://<this-machine's-ip>:5000/ on any device on the
# same WiFi network (including the machine running this script).

import os
import time
from collections import deque

from flask import Flask, jsonify, request, send_from_directory

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

app = Flask(__name__, static_folder=None)

latest_by_worker = {}       # workerId -> (message dict, receivedAt monotonic seconds)
latest_by_node = {}         # nodeId -> receivedAt monotonic seconds
events = deque(maxlen=200)  # (message dict, receivedAt monotonic seconds), newest first

# Ages are computed fresh on every response from time.monotonic(), not
# from a stored wall-clock timestamp. time.monotonic() never jumps
# (no NTP sync, no timezone, no DST) and needs no synchronization with
# the browser's clock -- the dashboard only ever needs to know "how
# long ago," never "what time." This also matches the ESP32-hosted
# path in wall_node.ino, which has no wall-clock at all (no internet,
# so no NTP) and computes the same kind of age from millis().


def _age_ms(received_at):
    return int((time.monotonic() - received_at) * 1000)


def _with_age(msg, received_at):
    out = dict(msg)
    out["ageMs"] = _age_ms(received_at)
    return out


def _touch_node(node_id):
    latest_by_node[node_id] = time.monotonic()


@app.route("/api/telemetry", methods=["POST"])
def telemetry():
    msg = request.get_json(force=True, silent=True)
    if not msg or "workerId" not in msg or "msgType" not in msg:
        return jsonify({"ok": False, "error": "malformed telemetry payload"}), 400

    now = time.monotonic()
    latest_by_worker[msg["workerId"]] = (msg, now)
    events.appendleft((msg, now))
    if "nodeId" in msg:
        _touch_node(msg["nodeId"])

    print(f"[{msg['msgType']}] {msg['workerId']}: {msg.get('payload')}")
    return jsonify({"ok": True}), 200


@app.route("/api/node-heartbeat", methods=["POST"])
def node_heartbeat():
    msg = request.get_json(force=True, silent=True)
    if not msg or "nodeId" not in msg:
        return jsonify({"ok": False, "error": "malformed heartbeat payload"}), 400

    _touch_node(msg["nodeId"])
    return jsonify({"ok": True}), 200


@app.route("/api/workers", methods=["GET"])
def workers():
    return jsonify({wid: _with_age(msg, ts) for wid, (msg, ts) in latest_by_worker.items()})


@app.route("/api/nodes", methods=["GET"])
def nodes():
    return jsonify({nid: {"nodeId": nid, "ageMs": _age_ms(ts)} for nid, ts in latest_by_node.items()})


@app.route("/api/events", methods=["GET"])
def get_events():
    limit = request.args.get("limit", default=30, type=int)
    return jsonify([_with_age(msg, ts) for msg, ts in list(events)[:limit]])


@app.route("/")
def dashboard():
    return send_from_directory(REPO_ROOT, "index.html")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
