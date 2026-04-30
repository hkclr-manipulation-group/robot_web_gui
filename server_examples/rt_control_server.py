#!/usr/bin/env python3
from curses.panel import panel
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import json
import threading
import time
import traceback
import os, sys, platform
from turtle import update

repo_path = os.path.dirname(os.path.dirname(__file__))
parent_path = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))

#Add scripts path
script_path = repo_path + "/scripts"
udp_repo_path = parent_path + "/curi_udp/python"
cuarm_config_scripts_path = parent_path + "/cuarm_configuration/share/python/scripts"
sys.path = [repo_path, parent_path, script_path, udp_repo_path, cuarm_config_scripts_path] + sys.path

#Add from cuarm_configuration repo
from cuarm_udp import CuarmUdp, CuarmUdpThread
from cuarm_state import RtPanelCommand, RtPlannerState, RtMotionControl, RtConnectionState, RtForceControlMode, ControlType, PanelTargetMode, RtInterpolationMethod

arch = platform.machine().lower()
if arch in ("x86_64", "amd64"):
    lib_path = parent_path + "/cuarm_configuration/share/python/lib/x86"
elif arch in ("aarch64", "arm64"):
    lib_path = parent_path + "/cuarm_configuration/share/python/lib/arm64"
else:
    raise RuntimeError(f"Unsupported arch: {arch}")
sys.path.append(lib_path)
import config_loader # cuarm_configuration.share.python.lib

ROBOTS = {
    "arm_v1": {"ip": "192.168.1.10", "port": 10001},
    "preview-arm": {"ip": "preview", "port": 0},
}
rt_planner_state = None
rt_panel_command = RtPanelCommand()
send_to_robot_lock = threading.Lock()
udp_dt_send = 0
udp = None
current_controller_ip = None
controller_lock = threading.Lock()
last_heartbeat = 0
_default_interpolation_acc = 1.0
_default_interpolation_const_vel = 1.0

_INTERPOLATION_BY_NAME = {
    "LINEAR": RtInterpolationMethod.LINEAR,
    "COS": RtInterpolationMethod.COS,
    "CUBIC": RtInterpolationMethod.CUBIC,
    "QUINTIC": RtInterpolationMethod.QUINTIC,
    "NONE": RtInterpolationMethod.NONE,
    "QUINTIC_PATH": RtInterpolationMethod.QUINTIC_PATH,
}

def _interpolation_method_from_payload(payload):
    raw = payload.get("interpolation_type")
    if raw is None:
        raw = payload.get("interpolation")
    if raw is None:
        raw = payload.get("interpolationType")
    if raw is None:
        return None
    if isinstance(raw, int):
        try:
            return RtInterpolationMethod(raw)
        except ValueError:
            return None
    key = str(raw).strip().upper()
    return _INTERPOLATION_BY_NAME.get(key)

def _apply_move_joint_interpolation(panel: RtPanelCommand, payload: dict):
    """Set interpolation for joint moves. Path defaults: /home → COS 5s; /move_joint and /zero → COS 1s. Override via payload."""
    path = payload.get("path", "/move_joint")
    explicit_method = _interpolation_method_from_payload(payload)
    explicit_acc = payload.get("interpolation_acc_time")
    if explicit_acc is None:
        explicit_acc = payload.get("interpolationAccTime")
    explicit_vel = payload.get("interpolation_const_vel_time")
    if explicit_vel is None:
        explicit_vel = payload.get("interpolationConstVelTime")

    if explicit_method is not None:
        panel.interpolation_type = explicit_method
    else:
        panel.interpolation_type = RtInterpolationMethod.COS

    if explicit_acc is not None:
        panel.interpolation_acc_time = float(explicit_acc)
    elif explicit_method is None:
        panel.interpolation_acc_time = 5.0 if path == "/home" else 1.0
    else:
        panel.interpolation_acc_time = _default_interpolation_acc

    if explicit_vel is not None:
        panel.interpolation_const_vel_time = float(explicit_vel)
    else:
        panel.interpolation_const_vel_time = _default_interpolation_const_vel

def initialize_panel_command(config_path, panel: RtPanelCommand):
    global udp_dt_send, _default_interpolation_acc, _default_interpolation_const_vel
    yaml_node = config_loader.load_yaml(config_path)
    config = yaml_node.as_dict()
    
    udp_dt_send = config["panel"]["dt_send"]
    
    panel.need_setting_update = True
    panel.simulation = True
    panel.motion_type = RtMotionControl.JOINT
    panel.connection_state = RtConnectionState.REMOTE
    panel.reset_control_mem = False
    panel.reset_joint_interpolation = False
    panel.force_control = RtForceControlMode.NONE

    # Mapping string from config to Enum
    panel.target_type = ControlType.VELOCITY if config["panel"]["general"]["target_type"] == "Velocity" else ControlType.POSITION
    panel.actuator_mode = ControlType.VELOCITY if config["panel"]["general"]["actuator_mode"] == "Velocity" else ControlType.POSITION
    panel.interpolation_type = RtInterpolationMethod.COS
    panel.interpolation_acc_time = config["panel"]["general"]["acc_time"]
    panel.interpolation_const_vel_time = config["panel"]["general"]["vel_time"]
    panel.none_interpolation_saturation_ratio = config["panel"]["general"]["none_interpolation_saturation_adjust"]
    _default_interpolation_acc = float(config["panel"]["general"]["acc_time"])
    _default_interpolation_const_vel = float(config["panel"]["general"]["vel_time"])
    
    panel.arm_target_mode = PanelTargetMode.SINGLE_POINT
    panel.arm_waypoint_size = 0
    panel.arm_size = len(config["robot"]["arm"])
    
    # Initialize Arm Lists/Arrays
    panel.joint_size = [arm["joint_size"] for arm in config["robot"]["arm"]]
    panel.robot_name = []
    panel.enable_joint = []
    panel.motor_cmd = []
    panel.joint_cmd = []
    panel.task_cmd = []
    
    for arm_i in range(panel.arm_size):
        # Name handling
        arm_node = config["robot"]["arm"][arm_i]
        robot_name = f"{arm_node['name']}-{arm_node['type']}"
        panel.robot_name.append(robot_name)

        # Joint based arrays
        n_joints = panel.joint_size[arm_i]
        panel.motor_cmd.append([0.0]* n_joints)
        panel.joint_cmd.append([0.0]* n_joints)
        panel.task_cmd.append([0.0]*6) # Fixed size 6
        
        # Enable state from config
        enables = [bool(e) for e in arm_node["enable_joint"]]
        panel.enable_joint.append(enables)

    # Gripper related
    panel.gripper_size = len(config["robot"]["gripper"]) if "gripper" in config["robot"] else 0
    if panel.gripper_size > 0:
        panel.gripper_joint_size = [arm["joint_size"] for arm in config["robot"]["gripper"]]
        panel.gripper_joint_command = []
        for gri_i in range(panel.gripper_size):
            n_gri_joints = panel.gripper_joint_size[gri_i]
            panel.gripper_joint_command.append([0.0]*n_gri_joints)

def send_to_robot(robot_id, update_func=None):
    global rt_panel_command, rt_planner_state
    acquired = send_to_robot_lock.acquire(timeout=udp_dt_send)
    success, message = True, ""
    if acquired:
        if rt_planner_state is None: return False, "Open rt_control connection first."

        first_update = True
        while first_update or rt_panel_command.need_setting_update or rt_planner_state.setting_update_finished:
            first_update = False
            
            #Update rt_panel_command
            if update_func: 
                success, message = update_func(robot_id)
                if not success: break
            
            if rt_panel_command.need_setting_update and not first_update:
                time_diff_sec = (rt_panel_command.send_timestamp - rt_planner_state.received_panel_command_timestamp)/1e6
                if rt_planner_state.setting_update_finished and time_diff_sec < 10*udp_dt_send:
                    rt_panel_command.need_setting_update = False
                print(f"time diff: {time_diff_sec:.3f} sec, smaller: {time_diff_sec < 10*udp_dt_send}")

            rt_panel_command.send_timestamp = int(time.time_ns() / 1e3)
            udp.send(rt_panel_command)
            time.sleep(udp_dt_send)
            print(f"rt_panel_command.send_timestamp: {rt_panel_command.send_timestamp}, setting_update_finished: {rt_planner_state.setting_update_finished}, need_setting_update: {rt_panel_command.need_setting_update}")
        print("rt_panel_command.joint_cmd:", rt_panel_command.joint_cmd)
        print("rt_panel_command.task_cmd:", rt_panel_command.task_cmd)
        send_to_robot_lock.release()
    else:
        return False, "Wait for the previous action to finish."
    return success, message

def move_joint(robot_id, payload):
    global rt_panel_command
    if rt_panel_command.force_control != RtForceControlMode.NONE: return False, "Disable teach mode first."

    if robot_id == "arm_v1":
        joint_values = payload.get("joint_values", None)
        if joint_values is not None and len(joint_values) > 0:
            _apply_move_joint_interpolation(rt_panel_command, payload)
            # 🔧 切换到关节控制模式
            rt_panel_command.motion_type = 0  # 0 表示关节空间控制
            rt_panel_command.joint_cmd[0] = joint_values
            # 清空任务空间命令，避免冲突
            rt_panel_command.task_cmd[0] = [0.0] * 6
            print(f"[move_joint] Updated joint_cmd for {robot_id}: {joint_values}, motion_type set to 0")
        else:
            print(f"[move_joint] Warning: No valid joint_values received for {robot_id}")
    else:
        print(f"[move_joint] Warning: Unknown robot_id: {robot_id}")
        return False, f"Unknown robot_id: {robot_id}"
    
    return True, "Success"

def move_pose(robot_id, payload):
    """处理任务空间位姿命令"""
    global rt_panel_command
    if rt_panel_command.force_control != RtForceControlMode.NONE: return False, "Disable teach mode first."

    # 🔧 支持 arm_v1
    if robot_id == "arm_v1":
        pose_values = payload.get("pose_values", None)
        if pose_values is not None and len(pose_values) >= 6:
            # 🔧 切换到任务空间控制模式
            rt_panel_command.motion_type = 2  # 2 表示任务空间控制
            
            # 🔧 单位转换：前端发送的是 [x(m), y(m), z(m), rx(deg), ry(deg), rz(deg)]
            # 底层期望的是 [x(m), y(m), z(m), rx(rad), ry(rad), rz(rad)]
            import math
            x, y, z, rx_deg, ry_deg, rz_deg = pose_values[:6]
            rx_rad = math.radians(rx_deg)
            ry_rad = math.radians(ry_deg)
            rz_rad = math.radians(rz_deg)
            
            rt_panel_command.task_cmd[0] = [x, y, z, rx_rad, ry_rad, rz_rad]
            # 注意：不清空 joint_cmd，因为底层可能需要当前关节位置作为参考
            print(f"[move_pose] Updated task_cmd for {robot_id}: pos=[{x:.4f}, {y:.4f}, {z:.4f}], ori_deg=[{rx_deg:.2f}, {ry_deg:.2f}, {rz_deg:.2f}], ori_rad=[{rx_rad:.4f}, {ry_rad:.4f}, {rz_rad:.4f}], motion_type set to 2")
        else:
            print(f"[move_pose] Warning: Invalid pose_values received for {robot_id}")
            return False, "Invalid pose data"
    else:
        print(f"[move_pose] Warning: Unknown robot_id: {robot_id}")
        return False, f"Unknown robot_id: {robot_id}"
    
    return True, "Success"

def move_pose_incremental(robot_id, payload):
    """处理任务空间增量位姿命令"""
    global rt_panel_command
    if rt_panel_command.force_control != RtForceControlMode.NONE: return False, "Disable teach mode first."

    # 🔧 支持 arm_v1
    if robot_id == "arm_v1":
        pose_delta_values = payload.get("pose_delta_values", None)
        if pose_delta_values is not None and len(pose_delta_values) >= 6:
            # 🔧 切换到任务空间控制模式
            rt_panel_command.motion_type = 2  # 2 表示任务空间控制
            
            # 🔧 单位转换：前端发送的是 [dx(m), dy(m), dz(m), drx(deg), dry(deg), drz(deg)]
            # 底层期望的是 [x(m), y(m), z(m), rx(rad), ry(rad), rz(rad)]
            # 需要将增量累加到当前位姿
            import math
            dx, dy, dz, drx_deg, dry_deg, drz_deg = pose_delta_values[:6]
            
            # 获取当前位姿
            current_task_cmd = rt_panel_command.task_cmd[0]
            current_x, current_y, current_z = current_task_cmd[0], current_task_cmd[1], current_task_cmd[2]
            current_rx_rad, current_ry_rad, current_rz_rad = current_task_cmd[3], current_task_cmd[4], current_task_cmd[5]
            
            # 计算新位姿（平移直接累加，旋转需要转换）
            new_x = current_x + dx
            new_y = current_y + dy
            new_z = current_z + dz
            
            # 旋转增量：将角度增量转换为弧度并累加
            drx_rad = math.radians(drx_deg)
            dry_rad = math.radians(dry_deg)
            drz_rad = math.radians(drz_deg)
            
            new_rx_rad = current_rx_rad + drx_rad
            new_ry_rad = current_ry_rad + dry_rad
            new_rz_rad = current_rz_rad + drz_rad
            
            # 更新任务命令
            rt_panel_command.task_cmd[0] = [new_x, new_y, new_z, new_rx_rad, new_ry_rad, new_rz_rad]
            
            print(f"[move_pose_incremental] Updated task_cmd for {robot_id}: delta=[{dx:.4f}, {dy:.4f}, {dz:.4f}, {drx_deg:.2f}°, {dry_deg:.2f}°, {drz_deg:.2f}°]")
            print(f"[move_pose_incremental] New absolute pose: pos=[{new_x:.4f}, {new_y:.4f}, {new_z:.4f}], ori_rad=[{new_rx_rad:.4f}, {new_ry_rad:.4f}, {new_rz_rad:.4f}]")
        else:
            print(f"[move_pose_incremental] Warning: Invalid pose_delta_values received for {robot_id}")
            return False, "Invalid pose delta data"
    else:
        print(f"[move_pose_incremental] Warning: Unknown robot_id: {robot_id}")
        return False, f"Unknown robot_id: {robot_id}"
    
    return True, "Success"
        
def enable_teach(robot_id, payload):
    global rt_panel_command, rt_planner_state
    if rt_planner_state is None: return False, "Open rt_control connection first."
    
    if robot_id == "arm_v1":
        prev_force_control = rt_panel_command.force_control
        if payload.get("enable", False):
            rt_panel_command.target_type = ControlType.TORQUE
            rt_panel_command.actuator_mode = ControlType.TORQUE
            rt_panel_command.force_control = RtForceControlMode.GRAVITY
            rt_panel_command.joint_cmd = [0.0]*6
        else:
            rt_panel_command.target_type = ControlType.POSITION
            rt_panel_command.actuator_mode = ControlType.POSITION
            rt_panel_command.force_control = RtForceControlMode.NONE
            rt_panel_command.joint_cmd = rt_planner_state.joint_pos
        rt_panel_command.need_setting_update = True if prev_force_control != rt_panel_command.force_control else False
    return True, "Success"
    
def connect_to_hardware(robot_id, payload):
    global rt_panel_command, rt_planner_state
    if rt_planner_state is None: return False, "Open rt_control connection first."
    if robot_id == "arm_v1":
        rt_panel_command.simulation = False if payload.get("enable", False) else True
        rt_panel_command.force_control = RtForceControlMode.NONE
        rt_panel_command.target_type = ControlType.POSITION
        rt_panel_command.actuator_mode = ControlType.POSITION
        rt_panel_command.joint_cmd = rt_planner_state.joint_pos
        rt_panel_command.need_setting_update = True
    return True, "Success"

def post_request(robot_id, payload, update_func):
    global rt_panel_command, rt_planner_state
    robot = ROBOTS.get(robot_id)
    if robot is None:
        return {"ok": False, "error": f"Unknown robot_id: {robot_id}"}
    
    success, message = send_to_robot(robot_id, update_func)
    print("success:", success, "message:", message)
    
    # 如果操作失败，返回 ok=False 以便前端正确处理错误
    return {
        "ok": success,  # 修复：根据实际成功状态设置 ok
        "robot_id": robot_id,
        "echo": payload,
        "success": success,
        "message": message
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
        #Prevent multiple connection
        client_ip = self.client_address[0]
        if not is_authorized(client_ip):
            self.send_response(403)
            self.end_headers()
            self.wfile.write(json.dumps({"ok": False, "error": "Robot is busy (Controlled by another IP)"}).encode())
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
            func_ptr = lambda a: connect_to_hardware(a, {"path": self.path, "enable": False})
            result = post_request(robot_id, {"path": self.path, "enable": False}, func_ptr)
        elif self.path == "/connect":
            func_ptr = lambda a: connect_to_hardware(a, {"path": self.path, "enable": True})
            result = post_request(robot_id, {"path": self.path, "enable": True}, func_ptr)
        elif self.path == "/move_joint" or self.path == "/home" or self.path == "/zero":
            func_ptr = lambda a: move_joint(a, {"path": self.path, **data})
            result = post_request(robot_id, {"path": self.path, **data}, func_ptr)
        elif self.path == "/move_pose":
            func_ptr = lambda a: move_pose(a, {"path": self.path, **data})
            result = post_request(robot_id, {"path": self.path, **data}, func_ptr)
        elif self.path == "/move_pose_incremental":
            func_ptr = lambda a: move_pose_incremental(a, {"path": self.path, **data})
            result = post_request(robot_id, {"path": self.path, **data}, func_ptr)
        elif self.path == "/teach":
            func_ptr = lambda a: enable_teach(a, {"path": self.path, **data})
            result = post_request(robot_id, {"path": self.path, **data}, func_ptr)
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
        global rt_planner_state
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                while True:
                    rt_planner_state = udp.receive()
                    # if rt_planner_state:
                    #     print(f"Stream received: joint_pos={rt_planner_state.joint_pos}, ee_pose={rt_planner_state.ee_pose}")
                    ee_pose = rt_planner_state.ee_pose if rt_planner_state else [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
                    joint_pos = rt_planner_state.joint_pos if rt_planner_state else [0.0]*6
                    state = json.dumps({"ee_pose": ee_pose, "joint_pos": joint_pos})
                    self.wfile.write(f"data: {state}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(0.01)
            except Exception:
                print("Client disconnected (browser closed)")

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in separate threads."""
    daemon_threads = True
    
if __name__ == "__main__":
    config_path = parent_path + "/cuarm_configuration/arm_v1/config.yaml"
    initialize_panel_command(config_path, rt_panel_command)
    udp = CuarmUdpThread("127.0.0.1", 12301, "127.0.0.1", 12201, CuarmUdp.unpack_rt_planner_state, CuarmUdp.pack_rt_panel_command, recv_delay_ms=5)
    try:
        server = ThreadedHTTPServer(("0.0.0.0", 9000), Handler)
        print("Gateway listening on http://0.0.0.0:9000")
        server.serve_forever()
    except KeyboardInterrupt:
        print("Ctrl+C detected. Exiting...")
    except Exception:
        traceback.print_exc()
    udp.terminate()
