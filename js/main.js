import { DEFAULT_ROBOTS, DEFAULT_URDF_PATH, PATH_DEFAULTS, STORAGE_KEYS } from './config.js';
import {
  connectRobot,
  disconnectRobot,
  getApiState,
  pingGateway,
  sendHomeCommand,
  sendJointCommand,
  sendPoseCommand,
  sendStopCommand,
  sendZeroCommand,
  setActiveRobot,
  setGatewayUrl,
} from './api.js';
import { JointsUI } from './joints-ui.js';
import { RobotKinematics } from './kinematics.js';
import { planCartesianTrajectory, planJointTrajectory } from './planner.js';
import { saveTrajectoryToFile, loadTrajectoryFromFile } from './storage.js';
import { TaskSpaceUI } from './taskspace-ui.js';
import { TeachSystem } from './teach.js';
import { loadRobotFromUrdf } from './urdf-loader-wrapper.js';
import { formatPoseText, readFileAsText, sleep } from './utils.js';
import { RobotViewer } from './viewer.js';

const viewer = new RobotViewer(document.getElementById('viewer'));
const statusEl = document.getElementById('status');
const urdfPathEl = document.getElementById('urdfPath');
const gatewayUrlEl = document.getElementById('gatewayUrl');
const robotSelectEl = document.getElementById('robotSelect');
const connectionBadgeEl = document.getElementById('connectionBadge');
const activeRobotTextEl = document.getElementById('activeRobotText');
const jointCountEl = document.getElementById('jointCount');
const jointContainerEl = document.getElementById('jointContainer');
const taskSpaceContainerEl = document.getElementById('taskSpaceContainer');
const eePoseEl = document.getElementById('eePose');
const baseLinkEl = document.getElementById('baseLink');
const tipLinkEl = document.getElementById('tipLink');
const robotIdTextEl = document.getElementById('robotIdText');
const teachCountEl = document.getElementById('teachCount');
const pathPreviewEl = document.getElementById('pathPreview');
const planStepsEl = document.getElementById('planSteps');
const playDelayEl = document.getElementById('playDelay');

let robot = null;
let kinematics = null;
let isBusy = false;
let lastGoalMap = null;
let lastGoalPose = null;
const teachSystem = new TeachSystem();

const jointsUI = new JointsUI(jointContainerEl, jointCountEl, {
  onJointInput: () => refreshPoseReadout(),
  onJointCommitted: async () => {
    if (!kinematics) return;
    const map = kinematics.getCurrentJointMap();
    await sendJointCommand(Object.keys(map), Object.values(map));
    refreshPoseReadout();
  },
});

const taskUI = new TaskSpaceUI(taskSpaceContainerEl, {
  onReadCurrent: () => {
    if (!kinematics) return;
    taskUI.setPose(kinematics.getEndEffectorPose());
  },
  onMove: async (pose) => {
    if (!kinematics || isBusy) return;
    setStatus('Solving IK...', 'warn');
    const result = kinematics.solveIK(pose);
    jointsUI.setValuesByMap(vectorToMap(result.q), true);
    refreshPoseReadout();
    await sendPoseCommand(pose);
    setStatus(result.success ? 'IK move finished.' : 'IK reached an approximate solution.', result.success ? 'ok' : 'warn');
  },
  onSetGoal: (pose) => {
    lastGoalPose = pose;
    setStatus('Task-space goal snapshot captured.', 'ok');
  },
  onPlanPose: () => document.getElementById('planCartesianBtn').click(),
});

taskUI.build();

function setStatus(text, cls = '') {
  statusEl.textContent = text;
  statusEl.className = `status-text ${cls}`.trim();
}

function updateConnectionUi(kind = 'preview') {
  const apiState = getApiState();
  const robotInfo = DEFAULT_ROBOTS.find((item) => item.id === apiState.activeRobotId) || DEFAULT_ROBOTS[0];
  activeRobotTextEl.textContent = `${robotInfo.name} · ${robotInfo.mode}`;
  robotIdTextEl.textContent = robotInfo.id;

  connectionBadgeEl.className = 'badge';
  if (kind === 'ok') {
    connectionBadgeEl.classList.add('badge-ok');
    connectionBadgeEl.textContent = 'connected';
  } else if (kind === 'warn') {
    connectionBadgeEl.classList.add('badge-warn');
    connectionBadgeEl.textContent = 'preview';
  } else if (kind === 'danger') {
    connectionBadgeEl.classList.add('badge-danger');
    connectionBadgeEl.textContent = 'error';
  } else {
    connectionBadgeEl.classList.add('badge-muted');
    connectionBadgeEl.textContent = 'preview';
  }
}

function vectorToMap(q) {
  return kinematics.getJointNames().reduce((acc, name, idx) => {
    acc[name] = q[idx];
    return acc;
  }, {});
}

function getPlanSteps() {
  return Math.max(2, parseInt(planStepsEl.value || PATH_DEFAULTS.steps, 10));
}

function getPlayDelay() {
  return Math.max(10, parseInt(playDelayEl.value || PATH_DEFAULTS.delayMs, 10));
}

function refreshPoseReadout() {
  if (!kinematics) return;
  eePoseEl.textContent = formatPoseText(kinematics.getEndEffectorPose());
}

function syncMeta() {
  if (!kinematics) return;
  baseLinkEl.textContent = kinematics.baseLinkName;
  tipLinkEl.textContent = kinematics.tipLinkName;
  refreshPoseReadout();
  taskUI.setPose(kinematics.getEndEffectorPose());
}

function updateTeachUi() {
  teachCountEl.textContent = `${teachSystem.count} poses`;
  pathPreviewEl.textContent = JSON.stringify(teachSystem.getPath(), null, 2);
  localStorage.setItem(STORAGE_KEYS.lastTrajectory, JSON.stringify(teachSystem.getPath()));
}

function populateRobotSelector() {
  robotSelectEl.innerHTML = '';
  DEFAULT_ROBOTS.forEach((item) => {
    const option = document.createElement('option');
    option.value = item.id;
    option.textContent = `${item.name}${item.ip && item.ip !== '-' ? ` (${item.ip})` : ''}`;
    robotSelectEl.appendChild(option);
  });
}

async function executeTrajectory(trajectory) {
  if (!kinematics || !trajectory?.length) return;
  isBusy = true;
  setStatus(`Playing ${trajectory.length} waypoints...`, 'warn');
  for (let i = 0; i < trajectory.length; i++) {
    if (!isBusy) break;
    const map = trajectory[i];
    kinematics.setJointMap(map);
    jointsUI.setValuesByMap(map, true);
    refreshPoseReadout();
    await sendJointCommand(Object.keys(map), Object.values(map));
    await sleep(getPlayDelay());
  }
  isBusy = false;
  setStatus('Path playback completed.', 'ok');
}

async function loadCurrentRobot(path) {
  try {
    setStatus('Loading URDF...', 'warn');
    robot = await loadRobotFromUrdf(path);
    viewer.setRobot(robot);
    kinematics = new RobotKinematics(robot);
    jointsUI.build(robot);
    syncMeta();
    localStorage.setItem(STORAGE_KEYS.lastUrdfPath, path);
    setStatus('URDF loaded.', 'ok');
  } catch (error) {
    console.error(error);
    setStatus(`Failed to load URDF: ${error.message || error}`, 'danger-text');
  }
}

async function saveGateway() {
  setGatewayUrl(gatewayUrlEl.value.trim());
  localStorage.setItem(STORAGE_KEYS.gatewayUrl, gatewayUrlEl.value.trim());
  setStatus(gatewayUrlEl.value.trim() ? 'Gateway URL saved.' : 'Gateway cleared. Preview mode enabled.', 'ok');
  updateConnectionUi(gatewayUrlEl.value.trim() ? 'ok' : 'warn');
}

async function connectSelectedRobot() {
  try {
    const result = await connectRobot();
    updateConnectionUi(result.mode === 'preview' ? 'warn' : 'ok');
    setStatus(result.mode === 'preview' ? 'Preview mode active. No gateway configured.' : 'Robot connected through gateway.', result.mode === 'preview' ? 'warn' : 'ok');
  } catch (error) {
    updateConnectionUi('danger');
    setStatus(error.message || 'Connect failed.', 'danger-text');
  }
}

function bindButtons() {
  document.getElementById('loadUrdfBtn').onclick = () => loadCurrentRobot(urdfPathEl.value.trim() || DEFAULT_URDF_PATH);
  document.getElementById('fitBtn').onclick = () => viewer.fitToRobot();
  document.getElementById('resetViewBtn').onclick = () => viewer.resetView();
  document.getElementById('refreshPoseBtn').onclick = () => refreshPoseReadout();
  document.getElementById('readCurrentPoseBtn').onclick = () => taskUI.setPose(kinematics?.getEndEffectorPose());

  document.getElementById('captureJointGoalBtn').onclick = () => {
    if (!kinematics) return;
    lastGoalMap = kinematics.getCurrentJointMap();
    setStatus('Joint-space goal snapshot captured.', 'ok');
  };

  document.getElementById('capturePoseGoalBtn').onclick = () => {
    if (!kinematics) return;
    lastGoalPose = kinematics.getEndEffectorPose();
    taskUI.setPose(lastGoalPose);
    setStatus('Task-space goal snapshot captured.', 'ok');
  };

  document.getElementById('homeBtn').onclick = async () => {
    if (!kinematics) return;
    kinematics.setJointVector(kinematics.getCurrentJointVector().map(() => 0));
    jointsUI.zeroAll();
    refreshPoseReadout();
    await sendHomeCommand();
    setStatus('Moved to home.', 'ok');
  };

  document.getElementById('zeroBtn').onclick = async () => {
    if (!kinematics) return;
    kinematics.setJointVector(kinematics.getCurrentJointVector().map(() => 0));
    jointsUI.zeroAll();
    refreshPoseReadout();
    await sendZeroCommand();
    setStatus('All joints zeroed.', 'ok');
  };

  document.getElementById('stopBtn').onclick = async () => {
    isBusy = false;
    await sendStopCommand();
    setStatus('Stop requested.', 'warn');
  };

  document.getElementById('recordPoseBtn').onclick = () => {
    if (!kinematics) return;
    teachSystem.record(kinematics.getCurrentJointMap());
    updateTeachUi();
    setStatus('Current pose recorded.', 'ok');
  };

  document.getElementById('clearPathBtn').onclick = () => {
    teachSystem.clear();
    updateTeachUi();
    setStatus('Path cleared.', 'ok');
  };

  document.getElementById('savePathBtn').onclick = () => {
    saveTrajectoryToFile(teachSystem.getPath());
    setStatus('Trajectory file saved.', 'ok');
  };

  document.getElementById('playPathBtn').onclick = async () => executeTrajectory(teachSystem.getPath());

  document.getElementById('planJointBtn').onclick = () => {
    if (!kinematics) return;
    if (!lastGoalMap) {
      lastGoalMap = kinematics.getCurrentJointMap();
      setStatus('Saved current joint state as the goal snapshot. Move the arm, then click again to plan.', 'warn');
      return;
    }
    const current = kinematics.getCurrentJointMap();
    const trajectory = planJointTrajectory(current, lastGoalMap, getPlanSteps());
    teachSystem.replaceAll(trajectory);
    updateTeachUi();
    setStatus('Joint trajectory generated.', 'ok');
  };

  document.getElementById('planCartesianBtn').onclick = () => {
    if (!kinematics) return;
    const goalPose = lastGoalPose || taskUI.getPose();
    const startPose = kinematics.getEndEffectorPose();
    const trajectory = planCartesianTrajectory(kinematics, startPose, goalPose, getPlanSteps());
    teachSystem.replaceAll(trajectory);
    updateTeachUi();
    setStatus('Cartesian trajectory generated via IK.', 'ok');
  };

  document.getElementById('pathFile').onchange = async (event) => {
    const file = event.target.files?.[0];
    if (!file) return;
    const trajectory = await loadTrajectoryFromFile(file);
    teachSystem.replaceAll(trajectory);
    updateTeachUi();
    setStatus('Trajectory loaded.', 'ok');
    event.target.value = '';
  };

  document.getElementById('urdfFile').onchange = async (event) => {
    const file = event.target.files?.[0];
    if (!file) return;
    const text = await readFileAsText(file);
    const blob = new Blob([text], { type: 'application/xml' });
    const url = URL.createObjectURL(blob);
    urdfPathEl.value = file.name;
    await loadCurrentRobot(url);
    event.target.value = '';
  };

  document.getElementById('saveGatewayBtn').onclick = saveGateway;
  document.getElementById('connectBtn').onclick = connectSelectedRobot;
  document.getElementById('disconnectBtn').onclick = async () => {
    try {
      const result = await disconnectRobot();
      updateConnectionUi(result.mode === 'preview' ? 'warn' : 'ok');
      setStatus(result.mode === 'preview' ? 'Preview disconnect completed.' : 'Robot disconnected.', 'ok');
    } catch (error) {
      updateConnectionUi('danger');
      setStatus(error.message || 'Disconnect failed.', 'danger-text');
    }
  };
  document.getElementById('pingBtn').onclick = async () => {
    try {
      const result = await pingGateway();
      updateConnectionUi(result.mode === 'preview' ? 'warn' : 'ok');
      setStatus(result.mode === 'preview' ? 'Preview mode ping.' : 'Gateway ping succeeded.', result.mode === 'preview' ? 'warn' : 'ok');
    } catch (error) {
      updateConnectionUi('danger');
      setStatus(error.message || 'Ping failed.', 'danger-text');
    }
  };

  robotSelectEl.onchange = () => {
    setActiveRobot(robotSelectEl.value);
    localStorage.setItem(STORAGE_KEYS.robotId, robotSelectEl.value);
    updateConnectionUi(gatewayUrlEl.value.trim() ? 'ok' : 'warn');
    setStatus(`Switched active robot to ${robotSelectEl.selectedOptions[0]?.textContent || robotSelectEl.value}.`, 'ok');
  };
}

(function boot() {
  populateRobotSelector();

  const savedGateway = localStorage.getItem(STORAGE_KEYS.gatewayUrl) || '';
  const savedRobotId = localStorage.getItem(STORAGE_KEYS.robotId) || DEFAULT_ROBOTS[0].id;
  const savedUrdf = localStorage.getItem(STORAGE_KEYS.lastUrdfPath) || DEFAULT_URDF_PATH;
  const savedTraj = localStorage.getItem(STORAGE_KEYS.lastTrajectory);

  gatewayUrlEl.value = savedGateway;
  setGatewayUrl(savedGateway);
  robotSelectEl.value = savedRobotId;
  setActiveRobot(savedRobotId);
  urdfPathEl.value = savedUrdf;
  updateConnectionUi(savedGateway ? 'ok' : 'warn');
  bindButtons();

  if (savedTraj) {
    try {
      teachSystem.replaceAll(JSON.parse(savedTraj));
      updateTeachUi();
    } catch {
      teachSystem.clear();
    }
  }

  loadCurrentRobot(savedUrdf);
})();
