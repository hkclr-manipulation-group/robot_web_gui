import { DEFAULT_URDF_PATH, PATH_DEFAULTS, STORAGE_KEYS } from './config.js';
import { sendHomeCommand, sendJointCommand, sendPoseCommand, sendStopCommand, sendZeroCommand } from './api.js';
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
const jointCountEl = document.getElementById('jointCount');
const jointContainerEl = document.getElementById('jointContainer');
const taskSpaceContainerEl = document.getElementById('taskSpaceContainer');
const eePoseEl = document.getElementById('eePose');
const baseLinkEl = document.getElementById('baseLink');
const tipLinkEl = document.getElementById('tipLink');
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
    setStatus(result.success ? 'IK move finished.' : 'IK reached approximate solution.', result.success ? 'ok' : 'warn');
  },
  onSetGoal: (pose) => {
    lastGoalPose = pose;
    setStatus('Task-space goal snapshot saved.', 'ok');
  },
  onPlanPose: () => document.getElementById('planCartesianBtn').click(),
});

taskUI.build();

function setStatus(text, cls = '') {
  statusEl.textContent = text;
  statusEl.className = cls;
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
  const pose = kinematics.getEndEffectorPose();
  eePoseEl.textContent = formatPoseText(pose);
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

async function executeTrajectory(trajectory) {
  if (!kinematics || !trajectory?.length) return;
  isBusy = true;
  setStatus(`Playing ${trajectory.length} waypoints...`, 'warn');
  for (let i = 0; i < trajectory.length; i++) {
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
    setStatus(`Failed to load URDF: ${error.message || error}`, 'danger');
  }
}

document.getElementById('loadUrdfBtn').onclick = () => loadCurrentRobot(urdfPathEl.value.trim() || DEFAULT_URDF_PATH);
document.getElementById('fitBtn').onclick = () => viewer.fitToRobot();
document.getElementById('refreshPoseBtn').onclick = () => refreshPoseReadout();
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
    setStatus('Saved current joint state as the planning goal seed. Move robot, then click again to plan.', 'warn');
    return;
  }
  const current = kinematics.getCurrentJointMap();
  const trajectory = planJointTrajectory(current, lastGoalMap, getPlanSteps());
  teachSystem.replaceAll(trajectory);
  updateTeachUi();
  lastGoalMap = current;
  setStatus('Joint trajectory generated. Previous snapshot swapped with current.', 'ok');
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

(function boot() {
  const savedUrdf = localStorage.getItem(STORAGE_KEYS.lastUrdfPath) || DEFAULT_URDF_PATH;
  urdfPathEl.value = savedUrdf;
  const savedTraj = localStorage.getItem(STORAGE_KEYS.lastTrajectory);
  if (savedTraj) {
    try {
      teachSystem.replaceAll(JSON.parse(savedTraj));
      updateTeachUi();
    } catch {}
  }
  loadCurrentRobot(savedUrdf);
})();
