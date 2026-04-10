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

def initialize_panel_command(config_path, panel: RtPanelCommand):
    global udp_dt_send
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

        if robot_id == "arm_v1":
            first_update = True
        while first_update or rt_panel_command.need_setting_update or rt_planner_state.setting_update_finished:
            first_update = False
            
            #Update rt_panel_command
            if update_func: 
                success, message = update_func(robot_id)
                if not success: break
            
            rt_panel_command.send_timestamp = int(time.time_ns() / 1e3)
            udp.send(rt_panel_command)
            time.sleep(udp_dt_send)
            
            if rt_panel_command.need_setting_update:
                time_diff_sec = (rt_panel_command.send_timestamp - rt_planner_state.received_panel_command_timestamp)/1e6
                if rt_planner_state.setting_update_finished and time_diff_sec < 10*udp_dt_send:
                    rt_panel_command.need_setting_update = False
                print(f"time diff: {time_diff_sec:.3f} sec, smaller: {time_diff_sec < 10*udp_dt_send}")
            print(f"rt_panel_command.send_timestamp: {rt_panel_command.send_timestamp}, setting_update_finished: {rt_planner_state.setting_update_finished}, need_setting_update: {rt_panel_command.need_setting_update}")
        print("rt_panel_command.joint_cmd:", rt_panel_command.joint_cmd)
        send_to_robot_lock.release()
    else:
        return False, "Wait for the previous action to finish."
    return success, message

def move_joint(robot_id, payload):
    global rt_panel_command
    if rt_panel_command.force_control != RtForceControlMode.NONE: return False, "Disable teach mode first."
    
    if robot_id == "arm_v1":
        rt_panel_command.joint_cmd[0] = payload.get("joint_values", rt_panel_command.joint_cmd[0])
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
        prev_force_control = rt_panel_command.force_control
        rt_panel_command.simulation = False if payload.get("enable", False) else True
        rt_panel_command.force_control = RtForceControlMode.NONE
        rt_panel_command.target_type = ControlType.POSITION
        rt_panel_command.actuator_mode = ControlType.POSITION
        rt_panel_command.joint_cmd = rt_planner_state.joint_pos
        rt_panel_command.need_setting_update = True if prev_force_control != rt_panel_command.force_control else False
    return True, "Success"

def post_request(robot_id, payload, update_func):
    global rt_panel_command, rt_planner_state
    robot = ROBOTS.get(robot_id)
    if robot is None:
        return {"ok": False, "error": f"Unknown robot_id: {robot_id}"}
    
    success, message = send_to_robot(robot_id, update_func)
    print("success:", success, "message:", message)
    return {
        "ok": True,
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
        elif self.path == "/teach":
            func_ptr = lambda a: enable_teach(a, {"path": self.path, **data})
            result = post_request(robot_id, {"path": self.path, **data}, func_ptr)

        status = 200 if result.get("ok", False) else 400
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
