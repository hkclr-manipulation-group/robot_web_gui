import { DEFAULT_ROBOTS } from './config.js';

const state = {
  gatewayUrl: '',
  activeRobotId: DEFAULT_ROBOTS[0].id,
  connected: false,
};

function ensureNoTrailingSlash(url) {
  return (url || '').trim().replace(/\/+$/, '');
}

function currentRobot() {
  return DEFAULT_ROBOTS.find((item) => item.id === state.activeRobotId) || DEFAULT_ROBOTS[0];
}

async function post(path, payload = {}) {
  const gateway = ensureNoTrailingSlash(state.gatewayUrl);
  if (!gateway) {
    return { ok: true, mode: 'preview', path, payload, robot: currentRobot() };
  }

  const response = await fetch(`${gateway}${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ robot_id: state.activeRobotId, ...payload }),
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

export function setActiveRobot(robotId) {
  state.activeRobotId = robotId;
}

export function getApiState() {
  return { ...state, robot: currentRobot() };
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
