#!/usr/bin/env python3
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import gc
import json
import math
import os
import re
import sys
import threading
import time
import traceback

repo_path = os.path.dirname(os.path.dirname(__file__))
parent_path = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
spark2_sdk_path = os.path.join(parent_path, "spark2_sdk", "spark2_python_dist")
sys.path.insert(0, spark2_sdk_path)

from spark2_sdk import (
    Spark2,
    JointState6b,
    JointState6f,
    Pose,
    Position,
    EulerAngle,
    SmoothingMethod,
)

KNOWN_ROBOT_IDS = {"spark2", "preview-arm"}

SEND_LOCK_TIMEOUT_SEC = 30.0
STREAM_INTERVAL_SEC = 0.01

_CONFIG_VARIANTS = ("spark2_v2", "spark2_v1")


def resolve_config_prefix_path():
    """Resolve Spark2 SDK config prefix (directory containing config.yaml)."""
    explicit = os.environ.get("SPARK2_CONFIG_PREFIX", "").strip()
    if explicit:
        return explicit.rstrip("/")

    default = os.path.join(parent_path, "cuarm_configuration", "spark2_v2")
    if os.path.isfile(os.path.join(default, "config.yaml")):
        return default

    config_root = os.environ.get("CONFIG_PREFIX_PATH", "").strip().rstrip("/")
    if config_root:
        if os.path.isfile(os.path.join(config_root, "config.yaml")):
            return config_root
        for variant in _CONFIG_VARIANTS:
            candidate = os.path.join(config_root, variant)
            if os.path.isfile(os.path.join(candidate, "config.yaml")):
                return candidate

    return default


CONFIG_PREFIX_PATH = resolve_config_prefix_path()

robot = None
robot_started = False
robot_hardware_connected = False
teach_active = False
runtime_start_lock = threading.Lock()
send_to_robot_lock = threading.Lock()
controller_lock = threading.Lock()
current_controller_ip = None
last_heartbeat = 0
last_stream_error = ""
last_stream_error_log = 0.0
RUNTIME_RETRY_SEC = 1.0

_SMOOTHING_BY_NAME = {
    "LINEAR": SmoothingMethod.LINEAR,
    "COS": SmoothingMethod.COS,
    "CUBIC": SmoothingMethod.CUBIC,
    "QUINTIC": SmoothingMethod.QUINTIC,
    "NONE": SmoothingMethod.NONE,
}


def _get_robot():
    global robot
    if robot is None:
        config_yaml = os.path.join(CONFIG_PREFIX_PATH, "config.yaml")
        if not os.path.isfile(config_yaml):
            raise RuntimeError(
                f"Spark2 config not found: {config_yaml}. "
                "Set SPARK2_CONFIG_PREFIX to a directory that contains config.yaml "
                "(for example .../cuarm_configuration/spark2_v2)."
            )
        robot = Spark2(CONFIG_PREFIX_PATH)
    return robot


def _panel_udp_port():
    config_yaml = os.path.join(CONFIG_PREFIX_PATH, "config.yaml")
    try:
        with open(config_yaml, encoding="utf-8") as fh:
            in_panel = False
            for line in fh:
                if line.strip().startswith("panel:"):
                    in_panel = True
                    continue
                if in_panel and line and not line.startswith((" ", "\t")):
                    break
                match = re.match(r"\s+port:\s*(\d+)", line)
                if in_panel and match:
                    return int(match.group(1))
    except Exception:
        pass
    return 12309


def _panel_port_holder_hint(port):
    return (
        f"Panel UDP port {port} is unavailable (already in use). "
        "Close Qt upper software and any other rt_control_server.py instance, "
        f"then run: ss -ulnp | grep {port}"
    )


def _format_runtime_error(exc):
    message = str(exc)
    if "Failed to initialize UDP node" in message or "Failed to initialize UDP communication" in message:
        return _panel_port_holder_hint(_panel_udp_port())
    return message


def _reset_robot_session():
    """Release Spark2 UDP resources so panel port can be reused after a failed start()."""
    global robot, robot_started, robot_hardware_connected, teach_active
    if robot is not None:
        old_robot = robot
        robot = None
        try:
            old_robot.stop()
        except Exception:
            pass
        del old_robot
        gc.collect()
    robot_started = False
    robot_hardware_connected = False
    teach_active = False


def ensure_robot_runtime():
    """Start Spark2 UDP session for /stream (panel simulation until Connect)."""
    global robot_started, teach_active

    if robot_started:
        return True, "Success"

    with runtime_start_lock:
        if robot_started:
            return True, "Success"
        try:
            robot_instance = _get_robot()
            robot_instance.start_stream()
            robot_started = True
            teach_active = False
        except Exception:
            _reset_robot_session()
            raise
    return True, "Success"


def ensure_hardware_control():
    """Connect to real hardware using the same sequence as Qt ConnectionButton."""
    global robot_hardware_connected

    ensure_robot_runtime()
    if robot_hardware_connected:
        return True, "Success"

    robot_instance = _get_robot()
    robot_instance.connect_hardware()
    robot_instance.enable_arm_joint(
        JointState6b([True, True, True, True, True, True])
    )
    robot_hardware_connected = True
    return True, "Success"


def release_hardware_control():
    """Return to simulation mode; keep UDP stream alive."""
    global robot_hardware_connected, teach_active

    if not robot_started:
        return True, "Success"

    robot_instance = _get_robot()
    if teach_active:
        robot_instance.stop_teach()
        teach_active = False
    if robot_hardware_connected:
        robot_instance.disconnect_hardware()
        robot_hardware_connected = False
    return True, "Success"


def _runtime_startup_worker():
    global last_stream_error
    while True:
        if robot_started:
            time.sleep(RUNTIME_RETRY_SEC)
            continue
        try:
            ensure_robot_runtime()
            last_stream_error = ""
            print("Spark2 runtime connected to rt_control (simulation mode, stream only).")
        except Exception as exc:
            last_stream_error = _format_runtime_error(exc)
            print(f"Waiting for rt_control... {last_stream_error}")
            time.sleep(RUNTIME_RETRY_SEC)


def _smoothing_method_from_payload(payload):
    raw = payload.get("interpolation_type")
    if raw is None:
        raw = payload.get("interpolation")
    if raw is None:
        raw = payload.get("interpolationType")
    if raw is None:
        return None
    if isinstance(raw, int):
        try:
            return SmoothingMethod(raw)
        except ValueError:
            return None
    key = str(raw).strip().upper()
    return _SMOOTHING_BY_NAME.get(key)


def _motion_timing_from_payload(payload, default_acc_time):
    explicit_method = _smoothing_method_from_payload(payload)
    explicit_acc = payload.get("interpolation_acc_time")
    if explicit_acc is None:
        explicit_acc = payload.get("interpolationAccTime")
    explicit_vel = payload.get("interpolation_const_vel_time")
    if explicit_vel is None:
        explicit_vel = payload.get("interpolationConstVelTime")

    acc_time = float(explicit_acc) if explicit_acc is not None else float(default_acc_time)
    speed = int(payload.get("speed", 50))
    return explicit_method, acc_time, speed


def _apply_smoothing(robot_instance, method):
    if method is not None:
        robot_instance.set_arm_smoothing_method(method)


def _joint_values_from_payload(payload):
    """Resolve joint vector in SDK order (J1..J6). Web GUI sends radians."""
    joint_values = payload.get("joint_values")
    joint_names = payload.get("joint_names")
    if joint_values is None or len(joint_values) == 0:
        return None
    if not joint_names or len(joint_names) != len(joint_values):
        return list(joint_values)

    by_name = {
        str(name): float(value)
        for name, value in zip(joint_names, joint_values)
    }

    def joint_index(name):
        digits = "".join(ch for ch in str(name) if ch.isdigit())
        return int(digits) if digits else 0

    ordered_names = sorted(by_name.keys(), key=joint_index)
    return [by_name[name] for name in ordered_names]


def _rad_list_to_deg(values):
    """Web GUI joint commands use radians; Spark2 SDK expects degrees."""
    return [math.degrees(v) for v in values]


def _deg_list_to_rad(values):
    """Spark2 SDK feedback is in degrees; Web GUI expects radians."""
    return [math.radians(v) for v in values]


def _pose_from_euler_rad(robot_instance, pose_values):
    """Web GUI task-space commands use meters + intrinsic XYZ Euler radians."""
    x, y, z, rx, ry, rz = pose_values[:6]
    quat = robot_instance.get_kinematics().euler_to_quaternion(
        EulerAngle(roll=rx, pitch=ry, yaw=rz)
    )
    return Pose(position=Position(x=x, y=y, z=z), orientation=quat)


def _ee_pose_stream_values(robot_instance):
    """Stream EE pose as position + quaternion (w,x,y,z), same as Qt state display."""
    pose = robot_instance.get_ee_pose()
    quat = pose.orientation
    return [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        quat.w,
        quat.x,
        quat.y,
        quat.z,
    ]


def _joint_pos_stream_values(robot_instance):
    joint_pos_deg = robot_instance.get_pos().arm.to_list()
    return [_deg_list_to_rad(joint_pos_deg)]


def _read_stream_state():
    global last_stream_error, last_stream_error_log

    if not robot_started:
        return {
            "connected": False,
                "ee_pose": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            "joint_pos": [[0.0] * 6],
            "hardware_connected": False,
            "error": last_stream_error or "Waiting for rt_control (start cuarm_rt_control first).",
        }

    try:
        ee_pose = _ee_pose_stream_values(robot)
        joint_pos = _joint_pos_stream_values(robot)
        last_stream_error = ""
        return {
            "connected": True,
            "hardware_connected": robot_hardware_connected,
            "ee_pose": ee_pose,
            "joint_pos": joint_pos,
            "error": "",
        }
    except Exception as exc:
        last_stream_error = str(exc)
        now = time.monotonic()
        if now - last_stream_error_log >= 5.0:
            print(f"[stream] failed to read robot state: {exc}")
            last_stream_error_log = now
        return {
            "connected": False,
                "ee_pose": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            "joint_pos": [[0.0] * 6],
            "hardware_connected": robot_hardware_connected,
            "error": last_stream_error,
        }


def execute_robot_action(robot_id, action, require_connection=True, require_hardware=False):
    if robot_id == "preview-arm":
        return False, "Select robot_id 'spark2' to control hardware (preview-arm is simulation-only)."
    if robot_id != "spark2":
        return False, f"Unknown robot_id: {robot_id}"

    acquired = send_to_robot_lock.acquire(timeout=SEND_LOCK_TIMEOUT_SEC)
    if not acquired:
        return False, "Wait for the previous action to finish."

    success, message = True, "Success"
    try:
        if require_connection and not robot_started:
            return False, (
                "Robot runtime not ready. Start cuarm_rt_control, then wait for "
                "'Spark2 runtime connected to rt_control (simulation mode, stream only).' "
                "in the gateway log."
            )
        if require_hardware and not robot_hardware_connected:
            return False, (
                "Hardware not connected. Click Connect in the web UI before this action."
            )
        action()
    except Exception as exc:
        success, message = False, _format_runtime_error(exc)
    finally:
        send_to_robot_lock.release()

    return success, message


def connect_to_hardware(robot_id, payload):
    global teach_active

    if robot_id == "preview-arm":
        return True, "Preview mode: no hardware connection required."
    if robot_id != "spark2":
        return False, f"Unknown robot_id: {robot_id}"

    enable = bool(payload.get("enable", False))

    def action():
        global teach_active
        if enable:
            ensure_hardware_control()
        else:
            release_hardware_control()

    return execute_robot_action(robot_id, action, require_connection=False)


def move_joint(robot_id, payload):
    global teach_active

    if teach_active:
        return False, "Disable teach mode first."

    joint_values = _joint_values_from_payload(payload)
    if joint_values is None:
        return False, "No valid joint_values received"

    path = payload.get("path", "/move_joint")
    default_acc_time = 5.0 if path == "/home" else 1.0
    method, acc_time, speed = _motion_timing_from_payload(payload, default_acc_time)

    def action():
        robot_instance = _get_robot()
        _apply_smoothing(robot_instance, method)
        if path == "/home":
            robot_instance.go_home(speed, acc_time)
        else:
            joint_values_deg = _rad_list_to_deg(joint_values)
            robot_instance.move_pos(JointState6f(joint_values_deg), speed, acc_time)
            print(
                f"[move_joint] path={path}, joints_rad={joint_values}, "
                f"joints_deg={joint_values_deg}, t={acc_time}, v={speed}"
            )
            return
        print(f"[move_joint] path={path}, t={acc_time}, v={speed}")

    return execute_robot_action(robot_id, action)


def move_pose(robot_id, payload):
    global teach_active

    if teach_active:
        return False, "Disable teach mode first."

    pose_values = payload.get("pose_values")
    if pose_values is None or len(pose_values) < 6:
        return False, "Invalid pose data"

    method, acc_time, speed = _motion_timing_from_payload(payload, 1.0)

    def action():
        robot_instance = _get_robot()
        _apply_smoothing(robot_instance, method)
        target_pose = _pose_from_euler_rad(robot_instance, pose_values)
        robot_instance.move_ee_point(target_pose, speed, acc_time)
        print(f"[move_pose] target={pose_values}, t={acc_time}, v={speed}")

    return execute_robot_action(robot_id, action)


def move_pose_incremental(robot_id, payload):
    global teach_active

    if teach_active:
        return False, "Disable teach mode first."

    pose_delta_values = payload.get("pose_delta_values")
    if pose_delta_values is None or len(pose_delta_values) < 6:
        return False, "Invalid pose delta data"

    method, acc_time, speed = _motion_timing_from_payload(payload, 1.0)

    def action():
        robot_instance = _get_robot()
        _apply_smoothing(robot_instance, method)

        current_pose = robot_instance.get_ee_pose()
        kinematics = robot_instance.get_kinematics()
        current_euler = kinematics.quaternion_to_euler(current_pose.orientation)

        dx, dy, dz, drx, dry, drz = pose_delta_values[:6]
        target_pose = _pose_from_euler_rad(
            robot_instance,
            [
                current_pose.position.x + dx,
                current_pose.position.y + dy,
                current_pose.position.z + dz,
                current_euler.roll + drx,
                current_euler.pitch + dry,
                current_euler.yaw + drz,
            ],
        )
        robot_instance.move_ee_point(target_pose, speed, acc_time)
        print(f"[move_pose_incremental] delta={pose_delta_values}, t={acc_time}, v={speed}")

    return execute_robot_action(robot_id, action)


def enable_teach(robot_id, payload):
    global teach_active

    def action():
        global teach_active
        robot_instance = _get_robot()
        if payload.get("enable", False):
            robot_instance.start_teach()
            teach_active = True
        else:
            robot_instance.stop_teach()
            teach_active = False

    return execute_robot_action(robot_id, action, require_hardware=True)


def post_request(robot_id, payload, handler):
    if robot_id not in KNOWN_ROBOT_IDS:
        return {"ok": False, "error": f"Unknown robot_id: {robot_id}"}

    success, message = handler(robot_id, payload)
    print("success:", success, "message:", message)

    return {
        "ok": success,
        "robot_id": robot_id,
        "echo": payload,
        "success": success,
        "message": message,
    }


def is_authorized(ip):
    global current_controller_ip, last_heartbeat
    with controller_lock:
        now = time.time()

        if current_controller_ip is None or (now - last_heartbeat > 10):
            current_controller_ip = ip
            last_heartbeat = now
            return True
        return current_controller_ip == ip


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
        client_ip = self.client_address[0]
        if not is_authorized(client_ip):
            self.send_response(403)
            self.end_headers()
            self.wfile.write(
                json.dumps({"ok": False, "error": "Robot is busy (Controlled by another IP)"}).encode()
            )
            return

        global last_heartbeat
        last_heartbeat = time.time()

        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        data = json.loads(raw.decode("utf-8"))
        robot_id = data.get("robot_id", "preview-arm")

        if self.path == "/ping":
            result = {"ok": True, "message": "gateway alive", "robot_id": robot_id}
        elif self.path == "/disconnect":
            result = post_request(
                robot_id,
                {"path": self.path, "enable": False},
                connect_to_hardware,
            )
        elif self.path == "/connect":
            result = post_request(
                robot_id,
                {"path": self.path, "enable": True},
                connect_to_hardware,
            )
        elif self.path in ("/move_joint", "/home", "/zero"):
            result = post_request(
                robot_id,
                {"path": self.path, **data},
                move_joint,
            )
        elif self.path == "/move_pose":
            result = post_request(
                robot_id,
                {"path": self.path, **data},
                move_pose,
            )
        elif self.path == "/move_pose_incremental":
            result = post_request(
                robot_id,
                {"path": self.path, **data},
                move_pose_incremental,
            )
        elif self.path == "/teach":
            result = post_request(
                robot_id,
                {"path": self.path, **data},
                enable_teach,
            )
        else:
            result = {"ok": False, "error": f"Unknown path: {self.path}"}

        if result.get("ok", False):
            status = 200
        elif self.path in (
            "/ping",
            "/disconnect",
            "/connect",
            "/move_joint",
            "/home",
            "/zero",
            "/move_pose",
            "/move_pose_incremental",
            "/teach",
        ):
            status = 400
        else:
            status = 404
        self._set_headers(status)
        self.wfile.write(json.dumps(result).encode("utf-8"))

    def do_GET(self):
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                while True:
                    state = json.dumps(_read_stream_state())
                    self.wfile.write(f"data: {state}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(STREAM_INTERVAL_SEC)
            except Exception:
                print("Client disconnected (browser closed)")


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


if __name__ == "__main__":
    print(f"Spark2 SDK config prefix: {CONFIG_PREFIX_PATH}")
    threading.Thread(target=_runtime_startup_worker, daemon=True).start()
    try:
        server = ThreadedHTTPServer(("0.0.0.0", 9000), Handler)
        print("Gateway listening on http://0.0.0.0:9000")
        print("Using Spark2 Python SDK.")
        print("State stream starts once rt_control is reachable.")
        server.serve_forever()
    except KeyboardInterrupt:
        print("Ctrl+C detected. Exiting...")
    except Exception:
        traceback.print_exc()
    finally:
        _reset_robot_session()
