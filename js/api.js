import { DEFAULT_ROBOTS, findRobotByName } from './config.js';

const state = {
  gatewayUrl: '',
  /** Selected robot variant (`name`); gateway still receives robot `id`. */
  activeRobotName: DEFAULT_ROBOTS[0].name,
  connected: false,
};

function ensureNoTrailingSlash(url) {
  return (url || '').trim().replace(/\/+$/, '');
}

function currentRobot() {
  return findRobotByName(state.activeRobotName);
}

async function post(path, payload = {}) {
  const gateway = ensureNoTrailingSlash(state.gatewayUrl);
  const robot = currentRobot();
  if (!gateway) {
    return { ok: true, mode: 'preview', path, payload, robot };
  }

  const response = await fetch(`${gateway}${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ robot_id: robot.id, ...payload }),
  });
  const text = await response.text();
  let data = null;
  try { data = text ? JSON.parse(text) : null; } catch { data = { raw: text }; }
  
  // 如果响应失败，优先使用后端返回的 message 字段，其次是 error 字段
  if (!response.ok) {
    const errorMessage = data?.message || data?.error || `Gateway request failed: ${response.status}`;
    throw new Error(errorMessage);
  }
  
  return { ok: true, mode: 'gateway', path, data };
}

export function setGatewayUrl(url) {
  state.gatewayUrl = ensureNoTrailingSlash(url);
}

export function setActiveRobot(robotName) {
  state.activeRobotName = findRobotByName(robotName).name;
}

export function getApiState() {
  const robot = currentRobot();
  return { ...state, robot, activeRobotId: robot.id };
}

/** Gateway configured and user clicked Connect (real hardware control). */
export function isHardwareControlActive() {
  return !!state.gatewayUrl && state.connected;
}

/** Gateway URL configured — motion goes to rt_control (hardware or panel simulation). */
export function isGatewayActive() {
  return !!state.gatewayUrl;
}

export async function connectRobot() {
  const result = await post('/connect', {});
  state.connected = true;
  return result;
}

export async function disconnectRobot() {
  const result = await post('/disconnect', {});
  state.connected = false;
  return result;
}

export async function pingGateway() {
  return post('/ping', {});
}

export async function enableTeachModeApi(enable) {
  return post('/teach', { enable });
}

/** SDK teach playback: action = 'reset' | 'start' | 'stop'. */
export async function playbackApi(action) {
  return post('/playback', { action });
}

export async function sendJointCommand(jointNames, jointValues, interpolation = {}) {
  return post('/move_joint', { joint_names: jointNames, joint_values: jointValues, ...interpolation });
}

export async function sendPoseCommand(poseArray) {
  // poseArray 应该是 [x, y, z, rx, ry, rz]
  return post('/move_pose', { pose_values: poseArray });
}

export async function sendPoseIncrementalCommand(deltaPoseArray) {
  // deltaPoseArray 应该是 [dx, dy, dz, drx, dry, drz] - 增量值
  return post('/move_pose_incremental', { pose_delta_values: deltaPoseArray });
}

export async function sendStopCommand() {
  return post('/stop', {});
}

export async function sendHomeCommand(jointNames, jointValues, interpolation = {}) {
  return post('/home', { joint_names: jointNames, joint_values: jointValues, ...interpolation });
}

export async function sendZeroCommand(jointNames, jointValues) {
  return post('/zero', { joint_names: jointNames, joint_values: jointValues });
}

export async function sendGripperCommand(pos, v = 50, t = 0) {
  return post('/move_gripper', { gripper_pos: pos, v, t });
}
