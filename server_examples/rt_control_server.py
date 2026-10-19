#!/usr/bin/env python3
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import gc
import json
import math
import os
import sys
import threading
import time
import traceback

repo_path = os.path.dirname(os.path.dirname(__file__))
parent_path = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))


def _robot_sdk_wheel_hint():
    root = os.environ.get("ROBOT_SDK_ROOT", "").strip()
    if not root:
        root = os.path.join(
            parent_path,
            "robot_sdk-1.3.1-20261013-152751-linux_x86_64-gcc_linux",
        )
    ver = f"{sys.version_info.major}.{sys.version_info.minor}"
    return os.path.join(
        root, "bindings", "python", f"robot_sdk-1.3.1-cp{ver}-cp{ver}-linux_x86_64.whl"
    )


try:
    from robot_sdk import (
    Robot,
    RobotError,
    RtConnectionInfo,
    SmoothingMethod,
    enumerate_robots,
    Pose,
    Position,
    EulerAngle,
    Quaternion,
    )
except ImportError as exc:
    wheel = _robot_sdk_wheel_hint()
    raise ImportError(
        f"{exc}. Install the release wheel, for example: pip install {wheel}"
    ) from exc

KNOWN_ROBOT_IDS = {"spark2", "preview-arm"}

SEND_LOCK_TIMEOUT_SEC = 30.0
STREAM_INTERVAL_SEC = 0.01

def _resolve_rt_connection_info():
    """RT server target: ROBOT_RT_IP / RT_SERVER_IP, else auto-discover on connect()."""
    ip = (
        os.environ.get("ROBOT_RT_IP", "").strip()
        or os.environ.get("RT_SERVER_IP", "").strip()
    )
    if ip:
        return RtConnectionInfo.to(ip)
    return RtConnectionInfo()


def _ensure_ok(result, context=""):
    if not result.ok:
        prefix = f"{context}: " if context else ""
        raise RuntimeError(f"{prefix}{result.message} ({result.code.name})")


def _euler_xyz_to_quaternion(roll, pitch, yaw):
    """Intrinsic XYZ (matches THREE.js / Web GUI task-space commands)."""
    c1, s1 = math.cos(roll / 2), math.sin(roll / 2)
    c2, s2 = math.cos(pitch / 2), math.sin(pitch / 2)
    c3, s3 = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        w=c1 * c2 * c3 - s1 * s2 * s3,
        x=s1 * c2 * c3 + c1 * s2 * s3,
        y=c1 * s2 * c3 - s1 * c2 * s3,
        z=c1 * c2 * s3 + s1 * s2 * c3,
    )


def _quaternion_to_euler_xyz(quat):
    """Inverse of _euler_xyz_to_quaternion (intrinsic XYZ)."""
    w, x, y, z = quat.w, quat.x, quat.y, quat.z
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return EulerAngle(roll=roll, pitch=pitch, yaw=yaw)


# Jog defaults (aligned with robot_web_gui/js/config.js SLIDER_CONTROL); overridable per request.
_DEFAULT_JOG_INTERVAL_MS = 40
_DEFAULT_JOG_SPEED = 50
_DEFAULT_JOG_ACC_TIME = 0.0
_jog_interval_sec = _DEFAULT_JOG_INTERVAL_MS / 1000.0

robot = None
robot_started = False
robot_hardware_connected = False
teach_active = False
runtime_start_lock = threading.Lock()
runtime_connect_requested = False
runtime_connect_lock = threading.Lock()
send_to_robot_lock = threading.Lock()
controller_lock = threading.Lock()
_jog_lock = threading.Lock()
_jog_state = {
    "space": None,
    "cmds": [0] * 6,
    "active": False,
    "speed": _DEFAULT_JOG_SPEED,
    "acc_time": _DEFAULT_JOG_ACC_TIME,
}
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
        robot = Robot()
    return robot


def _connect_robot_session(robot_instance):
    info = _resolve_rt_connection_info()
    result = robot_instance.connect(info)
    if not result.ok and not info.server_ip:
        discovered = enumerate_robots()
        if discovered:
            result = robot_instance.connect(discovered[0])
    _ensure_ok(result, "connect")


def _format_runtime_error(exc):
    if isinstance(exc, RobotError):
        return str(exc)
    return str(exc)


def request_runtime_connect():
    """Defer SDK connect until /stream or /ping (matches upper_software starting after rt_control)."""
    global runtime_connect_requested
    with runtime_connect_lock:
        runtime_connect_requested = True


def _stop_jog_state():
    with _jog_lock:
        _jog_state["active"] = False
        _jog_state["space"] = None
        _jog_state["cmds"] = [0] * 6


def _reset_robot_session():
    """Release robot_sdk session (disconnect transport)."""
    global robot, robot_started, robot_hardware_connected, teach_active
    _stop_jog_state()
    if robot is not None:
        old_robot = robot
        robot = None
        try:
            old_robot.disconnect()
        except Exception:
            pass
        del old_robot
        gc.collect()
    robot_started = False
    robot_hardware_connected = False
    teach_active = False


def ensure_robot_runtime():
    """Connect robot_sdk to rt_control for /stream (telemetry before Web Connect)."""
    global robot_started, teach_active

    request_runtime_connect()

    if robot_started:
        return True, "Success"

    with runtime_start_lock:
        if robot_started:
            return True, "Success"
        try:
            robot_instance = _get_robot()
            _connect_robot_session(robot_instance)
            robot_started = True
            teach_active = False
        except Exception:
            _reset_robot_session()
            raise
    return True, "Success"


def ensure_hardware_control():
    """Enable all arm joints after RT connect (Web UI Connect)."""
    global robot_hardware_connected

    ensure_robot_runtime()
    if robot_hardware_connected:
        return True, "Success"

    robot_instance = _get_robot()
    mask = list(robot_instance.all_enabled_mask())
    _ensure_ok(robot_instance.enable_arm_joint(mask), "enable_arm_joint")
    robot_hardware_connected = True
    return True, "Success"


def release_hardware_control():
    """Disable arm joints; keep RT session alive for /stream."""
    global robot_hardware_connected, teach_active

    if not robot_started:
        return True, "Success"

    robot_instance = _get_robot()
    if teach_active:
        _ensure_ok(robot_instance.stop_teach(), "stop_teach")
        teach_active = False
    if robot_hardware_connected:
        n = robot_instance.joint_count()
        _ensure_ok(
            robot_instance.enable_arm_joint([False] * n),
            "disable_arm_joint",
        )
        robot_hardware_connected = False
    _stop_jog_state()
    return True, "Success"


def _jog_tick(robot_instance, space, cmds, speed, acc_time):
    if hasattr(robot_instance, "jog_joint") and hasattr(robot_instance, "jog_cartesian"):
        if space == "joint":
            _ensure_ok(robot_instance.jog_joint(cmds, speed, acc_time), "jog_joint")
        elif space == "cartesian":
            _ensure_ok(
                robot_instance.jog_cartesian(cmds, speed, acc_time),
                "jog_cartesian",
            )
        return

    if space == "joint":
        n = robot_instance.joint_count()
        max_vel = 30.0 * max(1, min(100, int(speed))) / 100.0
        vel = [0.0] * n
        for i in range(min(len(cmds), n)):
            if cmds[i] == 1:
                vel[i] = max_vel
            elif cmds[i] == 2:
                vel[i] = -max_vel
        _ensure_ok(robot_instance.move_vel(vel, a=10, t=acc_time), "move_vel")
        return

    if space == "cartesian":
        raise RuntimeError(
            "Cartesian jog requires robot_sdk.internal (streaming jog). "
            "Use joint jog, incremental task sliders, or install the internal SDK wheel."
        )


def _jog_worker():
    while True:
        with _jog_lock:
            active = _jog_state["active"]
            space = _jog_state["space"]
            cmds = list(_jog_state["cmds"])
            speed = _jog_state["speed"]
            acc_time = _jog_state["acc_time"]
            interval_sec = _jog_interval_sec
        if robot_started and active and space:
            try:
                acquired = send_to_robot_lock.acquire(timeout=0.05)
                if not acquired:
                    time.sleep(interval_sec)
                    continue
                try:
                    _jog_tick(_get_robot(), space, cmds, speed, acc_time)
                finally:
                    send_to_robot_lock.release()
            except Exception as exc:
                message = str(exc)
                print(f"[jog] tick failed: {message}")
                if "control lost" in message.lower():
                    _stop_jog_state()
                    _reset_robot_session()
        time.sleep(interval_sec)


def _normalize_jog_commands(raw):
    if raw is None:
        return [0] * 6
    if len(raw) != 6:
        raise ValueError("jog_commands must have length 6")
    name_map = {
        "stop": 0, "increase": 1, "decrease": 2,
        "inc": 1, "dec": 2, "+": 1, "-": 2,
    }
    out = []
    for item in raw:
        if isinstance(item, str):
            key = item.strip().lower()
            if key not in name_map:
                raise ValueError(f"Invalid jog command: {item}")
            out.append(name_map[key])
        else:
            value = int(item)
            if value not in (0, 1, 2):
                raise ValueError(f"Invalid jog command value: {value}")
            out.append(value)
    return out


def _update_jog_state(payload, space):
    global _jog_interval_sec
    cmds = _normalize_jog_commands(payload.get("jog_commands"))
    with _jog_lock:
        _jog_state["space"] = space
        _jog_state["cmds"] = cmds
        _jog_state["active"] = any(c != 0 for c in cmds)
        if "speed" in payload:
            _jog_state["speed"] = int(payload["speed"])
        if "acc_time" in payload:
            _jog_state["acc_time"] = float(payload["acc_time"])
        elif "jog_acc_time" in payload:
            _jog_state["acc_time"] = float(payload["jog_acc_time"])
        if "jog_interval_ms" in payload:
            _jog_interval_sec = max(10, int(payload["jog_interval_ms"])) / 1000.0
    return True, "Success"


def jog_joint(robot_id, payload):
    if teach_active:
        return False, "Disable teach mode first."
    request_runtime_connect()
    try:
        return _update_jog_state(payload, "joint")
    except ValueError as exc:
        return False, str(exc)


def jog_cartesian(robot_id, payload):
    if teach_active:
        return False, "Disable teach mode first."
    request_runtime_connect()
    try:
        return _update_jog_state(payload, "cartesian")
    except ValueError as exc:
        return False, str(exc)


def jog_stop(robot_id, payload):
    _stop_jog_state()

    def action():
        if not robot_started:
            return
        robot_instance = _get_robot()
        if hasattr(robot_instance, "jog_stop"):
            _ensure_ok(robot_instance.jog_stop(), "jog_stop")
            return
        n = robot_instance.joint_count()
        _ensure_ok(robot_instance.move_vel([0.0] * n, a=10, t=0.0), "move_vel")

    return execute_robot_action(robot_id, action)


def _runtime_startup_worker():
    global last_stream_error
    while True:
        if robot_started:
            time.sleep(RUNTIME_RETRY_SEC)
            continue
        with runtime_connect_lock:
            should_connect = runtime_connect_requested
        if not should_connect:
            time.sleep(RUNTIME_RETRY_SEC)
            continue
        try:
            ensure_robot_runtime()
            last_stream_error = ""
            print("robot_sdk connected to rt_control (stream active; click Connect to enable joints).")
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
        _ensure_ok(
            robot_instance.set_arm_smoothing_method(method),
            "set_arm_smoothing_method",
        )


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


def _pose_from_euler_rad(_robot_instance, pose_values):
    """Web GUI task-space commands use meters + intrinsic XYZ Euler radians."""
    x, y, z, rx, ry, rz = pose_values[:6]
    quat = _euler_xyz_to_quaternion(rx, ry, rz)
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
    joint_pos_deg = list(robot_instance.joint_positions())
    return [_deg_list_to_rad(joint_pos_deg)]


def _gripper_pos_stream_value(robot_instance):
    try:
        gripper = list(robot_instance.get_joint_state().gripper)
        return float(gripper[0]) if gripper else None
    except Exception:
        return None


def _read_stream_state():
    global last_stream_error, last_stream_error_log

    if not robot_started:
        return {
            "connected": False,
                "ee_pose": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            "joint_pos": [[0.0] * 6],
            "gripper_pos": None,
            "hardware_connected": False,
            "error": last_stream_error or "Waiting for rt_control (start RT server, set ROBOT_RT_IP if needed).",
        }

    try:
        ee_pose = _ee_pose_stream_values(robot)
        joint_pos = _joint_pos_stream_values(robot)
        gripper_pos = _gripper_pos_stream_value(robot)
        last_stream_error = ""
        return {
            "connected": True,
            "hardware_connected": robot_hardware_connected,
            "ee_pose": ee_pose,
            "joint_pos": joint_pos,
            "gripper_pos": gripper_pos,
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
            "gripper_pos": None,
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
                "Robot runtime not ready. Start rt_control on the robot/PC, set "
                "ROBOT_RT_IP when auto-discovery fails, then wait for "
                "'robot_sdk connected to rt_control' in the gateway log."
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

    _stop_jog_state()

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
            _ensure_ok(robot_instance.go_home(speed, acc_time), "go_home")
        else:
            joint_values_deg = _rad_list_to_deg(joint_values)
            _ensure_ok(
                robot_instance.move_pos(joint_values_deg, speed, acc_time),
                "move_pos",
            )
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

    _stop_jog_state()

    pose_values = payload.get("pose_values")
    if pose_values is None or len(pose_values) < 6:
        return False, "Invalid pose data"

    method, acc_time, speed = _motion_timing_from_payload(payload, 1.0)

    def action():
        robot_instance = _get_robot()
        _apply_smoothing(robot_instance, method)
        target_pose = _pose_from_euler_rad(robot_instance, pose_values)
        _ensure_ok(
            robot_instance.move_ee_point(target_pose, speed, acc_time),
            "move_ee_point",
        )
        print(f"[move_pose] target={pose_values}, t={acc_time}, v={speed}")

    return execute_robot_action(robot_id, action)


def move_pose_incremental(robot_id, payload):
    global teach_active

    if teach_active:
        return False, "Disable teach mode first."

    _stop_jog_state()

    pose_delta_values = payload.get("pose_delta_values")
    if pose_delta_values is None or len(pose_delta_values) < 6:
        return False, "Invalid pose delta data"

    method, acc_time, speed = _motion_timing_from_payload(payload, 1.0)

    def action():
        robot_instance = _get_robot()
        _apply_smoothing(robot_instance, method)

        current_pose = robot_instance.get_ee_pose()
        current_euler = _quaternion_to_euler_xyz(current_pose.orientation)

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
        _ensure_ok(
            robot_instance.move_ee_point(target_pose, speed, acc_time),
            "move_ee_point",
        )
        print(f"[move_pose_incremental] delta={pose_delta_values}, t={acc_time}, v={speed}")

    return execute_robot_action(robot_id, action)


def move_gripper(robot_id, payload):
    global teach_active

    if teach_active:
        return False, "Disable teach mode first."

    gripper_pos = payload.get("gripper_pos")
    if gripper_pos is None:
        return False, "No gripper_pos received"
    if isinstance(gripper_pos, (list, tuple)):
        if len(gripper_pos) == 0:
            return False, "No gripper_pos received"
        gripper_pos = gripper_pos[0]

    try:
        gripper_pos = float(gripper_pos)
    except (TypeError, ValueError):
        return False, "Invalid gripper_pos"

    try:
        speed = int(payload.get("v", payload.get("speed", 50)))
    except (TypeError, ValueError):
        speed = 50
    try:
        acc_time = float(payload.get("t", 0.0))
    except (TypeError, ValueError):
        acc_time = 0.0

    def action():
        robot_instance = _get_robot()
        _ensure_ok(
            robot_instance.move_gripper_pos([gripper_pos], speed, acc_time),
            "move_gripper_pos",
        )
        print(f"[move_gripper] pos={gripper_pos}, t={acc_time}, v={speed}")

    return execute_robot_action(robot_id, action)


def enable_teach(robot_id, payload):
    global teach_active

    def action():
        global teach_active
        robot_instance = _get_robot()
        if payload.get("enable", False):
            _ensure_ok(robot_instance.start_teach(), "start_teach")
            teach_active = True
        else:
            _ensure_ok(robot_instance.stop_teach(), "stop_teach")
            teach_active = False

    return execute_robot_action(robot_id, action, require_hardware=True)


def control_playback(robot_id, payload):
    """SDK teach/playback lifecycle: reset | start | stop (see teach_and_playback.py)."""
    global teach_active

    if teach_active:
        return False, "Disable teach mode first."

    action = str(payload.get("action", "")).strip().lower()
    if action not in ("reset", "start", "stop"):
        return False, "playback action must be reset, start, or stop."

    def run():
        robot_instance = _get_robot()
        if action == "reset":
            _ensure_ok(robot_instance.reset_playback(), "reset_playback")
        elif action == "start":
            _ensure_ok(robot_instance.start_playback(), "start_playback")
        else:
            _ensure_ok(robot_instance.stop_playback(), "stop_playback")
        print(f"[playback] action={action}")

    return execute_robot_action(robot_id, run, require_hardware=True)


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
            request_runtime_connect()
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
        elif self.path == "/move_gripper":
            result = post_request(
                robot_id,
                {"path": self.path, **data},
                move_gripper,
            )
        elif self.path == "/jog_joint":
            result = post_request(
                robot_id,
                {"path": self.path, **data},
                jog_joint,
            )
        elif self.path == "/jog_cartesian":
            result = post_request(
                robot_id,
                {"path": self.path, **data},
                jog_cartesian,
            )
        elif self.path == "/jog_stop":
            result = post_request(
                robot_id,
                {"path": self.path, **data},
                jog_stop,
            )
        elif self.path == "/teach":
            result = post_request(
                robot_id,
                {"path": self.path, **data},
                enable_teach,
            )
        elif self.path == "/playback":
            result = post_request(
                robot_id,
                {"path": self.path, **data},
                control_playback,
            )
        elif self.path == "/stop":
            result = post_request(
                robot_id,
                {"path": self.path, "action": "stop"},
                control_playback,
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
            "/move_gripper",
            "/jog_joint",
            "/jog_cartesian",
            "/jog_stop",
            "/teach",
            "/playback",
            "/stop",
        ):
            status = 400
        else:
            status = 404
        self._set_headers(status)
        self.wfile.write(json.dumps(result).encode("utf-8"))

    def do_GET(self):
        if self.path == "/stream":
            request_runtime_connect()
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
    rt_info = _resolve_rt_connection_info()
    if rt_info.server_ip:
        print(f"robot_sdk RT target: {rt_info.server_ip}")
    else:
        print("robot_sdk RT target: auto-discover (set ROBOT_RT_IP for a fixed host)")
    print("Recommended: start rt_control first, then open the Web UI.")
    threading.Thread(target=_runtime_startup_worker, daemon=True).start()
    threading.Thread(target=_jog_worker, daemon=True).start()
    try:
        server = ThreadedHTTPServer(("0.0.0.0", 9000), Handler)
        print("Gateway listening on http://0.0.0.0:9000")
        print("Using robot_sdk Python API (see robot_sdk-1.3.1 docs).")
        print("State stream connects on first /stream or /ping (after rt_control is up).")
        server.serve_forever()
    except KeyboardInterrupt:
        print("Ctrl+C detected. Exiting...")
    except Exception:
        traceback.print_exc()
    finally:
        _reset_robot_session()
