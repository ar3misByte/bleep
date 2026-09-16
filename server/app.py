# Sensora — dashboard relay server
#
# Receives wall-node telemetry over HTTP and serves the dashboard
# (index.html) itself, so the browser's fetch() calls are same-origin
# (no HTTPS/mixed-content or CORS problems on the local network).
#
# Run:
#   pip install flask
#   python server/app.py
# Then open http://<this-machine's-ip>:5000/ on any device on the
# same WiFi network (including the machine running this script).

import os
from collections import deque
from datetime import datetime, timezone

from flask import Flask, jsonify, request, send_from_directory

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

app = Flask(__name__, static_folder=None)

latest_by_worker = {}       # workerId -> last message, with server arrival time
events = deque(maxlen=200)  # newest-first ring buffer of every message received


def _arrival_iso():
    return datetime.now(timezone.utc).isoformat()


@app.route("/api/telemetry", methods=["POST"])
def telemetry():
    msg = request.get_json(force=True, silent=True)
    if not msg or "workerId" not in msg or "msgType" not in msg:
        return jsonify({"ok": False, "error": "malformed telemetry payload"}), 400

    msg["serverReceivedAt"] = _arrival_iso()
    latest_by_worker[msg["workerId"]] = msg
    events.appendleft(msg)

    print(f"[{msg['msgType']}] {msg['workerId']}: {msg.get('payload')}")
    return jsonify({"ok": True}), 200


@app.route("/api/workers", methods=["GET"])
def workers():
    return jsonify(latest_by_worker)


@app.route("/api/events", methods=["GET"])
def get_events():
    limit = request.args.get("limit", default=30, type=int)
    return jsonify(list(events)[:limit])


@app.route("/")
def dashboard():
    return send_from_directory(REPO_ROOT, "index.html")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
