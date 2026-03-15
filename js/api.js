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
  if (!response.ok) {
    throw new Error(data?.error || `Gateway request failed: ${response.status}`);
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

export async function sendJointCommand(jointNames, jointValues) {
  return post('/move_joint', { joint_names: jointNames, joint_values: jointValues });
}

export async function sendPoseCommand(pose) {
  return post('/move_pose', { pose });
}

export async function sendStopCommand() {
  return post('/stop', {});
}

export async function sendHomeCommand() {
  return post('/home', {});
}

export async function sendZeroCommand() {
  return post('/zero', {});
}
