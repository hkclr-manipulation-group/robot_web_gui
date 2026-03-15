#!/usr/bin/env python3
"""
Very small local HTTP gateway example for WiFi robots.
Use this on a PC/NUC/Jetson inside the same LAN as the robot controllers.
GitHub Pages serves the frontend; this file is only a demo backend adapter.
"""
from http.server import BaseHTTPRequestHandler, HTTPServer
import json

ROBOTS = {
    "left-arm": {"ip": "192.168.1.10", "port": 10001},
    "right-arm": {"ip": "192.168.1.11", "port": 10001},
    "preview-arm": {"ip": "preview", "port": 0},
}


def fake_send_to_robot(robot_id, payload):
    robot = ROBOTS.get(robot_id)
    if robot is None:
        return {"ok": False, "error": f"Unknown robot_id: {robot_id}"}
    return {
        "ok": True,
        "robot_id": robot_id,
        "target_ip": robot["ip"],
        "target_port": robot["port"],
        "echo": payload,
    }


class Handler(BaseHTTPRequestHandler):
    def _set_headers(self, status=200):
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_OPTIONS(self):
        self._set_headers(200)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        data = json.loads(raw.decode("utf-8"))
        robot_id = data.get("robot_id", "preview-arm")

        if self.path == "/ping":
            result = {"ok": True, "message": "gateway alive", "robot_id": robot_id}
        else:
            result = fake_send_to_robot(robot_id, {"path": self.path, **data})

        status = 200 if result.get("ok", False) else 400
        self._set_headers(status)
        self.wfile.write(json.dumps(result).encode("utf-8"))


if __name__ == "__main__":
    server = HTTPServer(("0.0.0.0", 9000), Handler)
    print("Gateway listening on http://0.0.0.0:9000")
    server.serve_forever()
