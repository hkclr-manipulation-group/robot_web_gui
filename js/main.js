import {
  DEFAULT_ROBOTS,
  DEFAULT_URDF_PATH,
  PATH_DEFAULTS,
  RT_INTERPOLATION,
  STORAGE_KEYS,
  rtInterpolationPayload,
} from "./config.js";

import {
  connectRobot,
  disconnectRobot,
  getApiState,
  pingGateway,
  sendHomeCommand,
  sendJointCommand,
  sendPoseCommand,
  sendPoseIncrementalCommand,
  sendStopCommand,
  sendZeroCommand,
  setActiveRobot,
  setGatewayUrl,
} from "./api.js";

import { JointsUI } from "./joints-ui.js";
import { KinematicsLab } from "./kinematics-lab.js";
import { RobotKinematics } from "./kinematics.js";
import { planCartesianTrajectory, planJointTrajectory } from "./planner.js";
import { saveTrajectoryToFile, loadTrajectoryFromFile } from "./storage.js";
import { TaskSpaceUI } from "./taskspace-ui.js";
import { createTeachModule } from "./teach.js";
import {
  applyGhostVisualStyle,
  applyHardwareContrastStyle,
  cloneMaterialsPerMesh,
  loadRobotFromUrdf,
} from "./urdf-loader-wrapper.js";
import { formatPoseText, readFileAsText, sleep, quaternionToPose } from "./utils.js";
import { RobotViewer } from "./viewer.js";
import * as THREE from "three";

/* ----------6---------------------------------------------------------------- */
/* DOM                                                                         */
/* -------------------------------------------------------------------------- */

const viewer = new RobotViewer(document.getElementById("viewer"));

const statusEl = document.getElementById("status");
const urdfPathEl = document.getElementById("urdfPath");
const gatewayUrlEl = document.getElementById("gatewayUrl");
const robotSelectEl = document.getElementById("robotSelect");

const connectionBadgeEl = document.getElementById("connectionBadge");
const activeRobotTextEl = document.getElementById("activeRobotText");
const robotIdTextEl = document.getElementById("robotIdText");

const jointCountEl = document.getElementById("jointCount");
const jointContainerEl = document.getElementById("jointContainer");
const taskSpaceContainerEl = document.getElementById("taskSpaceContainer");
const kinematicsLabContainerEl = document.getElementById("kinematicsLabPage");
const jointContainerTeachEl = document.getElementById("jointContainerTeach");
const teachRecordBtnEl = document.getElementById("teachRecordBtn");
const teachPlayBtnEl = document.getElementById("teachPlayBtn");
const teachStopBtnEl = document.getElementById("teachStopBtn");

const eePoseEl = document.getElementById("eePose");
const baseLinkEl = document.getElementById("baseLink");
const tipLinkEl = document.getElementById("tipLink");

const teachCountEl = document.getElementById("teachCount");
const pathPreviewEl = document.getElementById("pathPreview");

const planStepsEl = document.getElementById("planSteps");
const playDelayEl = document.getElementById("playDelay");

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

/** URDF tuned from commands / IK (lighter “ghost”). */
let robotGhost = null;
/** Duplicate URDF driven by streamed joint telemetry (full materials). */
let robotHardware = null;
let kinematics = null;

let isBusy = false;
let isSyncing = false; // 防止 UI/FK/IK 相互触发造成循环
let ikBusy = false;    // 防止拖动时重复进入 IK

let lastGoalMap = null;
let lastGoalPose = null;

const RETRY_DELAY = 3000;
let robotStream = null; // EventSource for streaming robot data from gateway
let robotStreamRetryTimer = null;
let latestJointPosition = null; // latest joint_pos from /stream

/* -------------------------------------------------------------------------- */
/* Utilities                                                                   */
/* -------------------------------------------------------------------------- */

function setStatus(text, cls = "") {
  statusEl.textContent = text;
  statusEl.className = `status-text ${cls}`.trim();
}

function withSyncGuard(fn) {
  if (isSyncing) return;
  isSyncing = true;
  try {
    fn();
  } finally {
    isSyncing = false;
  }
}

function getPlanSteps() {
  if (!planStepsEl) return Math.max(2, PATH_DEFAULTS.steps);
  return Math.max(2, parseInt(planStepsEl.value || PATH_DEFAULTS.steps, 10));
}

function getPlayDelay() {
  if (!playDelayEl) return Math.max(10, PATH_DEFAULTS.delayMs);
  return Math.max(10, parseInt(playDelayEl.value || PATH_DEFAULTS.delayMs, 10));
}

function hasRobot() {
  return !!kinematics;
}

/** Apply numeric joint vector using the same naming order as kinematics (parallel URDF clones). */
function applyJointVectorToUrdfRobot(urdfRobot, jointVector) {
  if (!urdfRobot || !kinematics || !jointVector?.length) return;
  const names = kinematics.getJointNames();
  for (let i = 0; i < names.length; i++) {
    const joint = urdfRobot.joints?.[names[i]];
    if (!joint) continue;
    const v = jointVector[i];
    if (!Number.isFinite(v)) continue;
    if (typeof joint.setJointValue === "function") joint.setJointValue(v);
    else joint.angle = v;
  }
  urdfRobot.updateMatrixWorld(true);
}

/** rad：仿真指令与遥测对齐后仿真臂淡出 */
const JOINT_ALIGNMENT_TOL_RAD = 0.04;

function telemetryMatchesGhostCommand(telemJoint) {
  if (!kinematics || !telemJoint?.length) return false;
  const n = kinematics.getJointNames().length;
  const t = telemJoint.slice(0, n);
  if (t.length < n) return false;

  const cmd = kinematics.getCurrentJointVector();
  let maxD = 0;
  for (let i = 0; i < n; i++) {
    const a = cmd[i];
    const b = t[i];
    if (!Number.isFinite(a) || !Number.isFinite(b)) continue;
    maxD = Math.max(maxD, Math.abs(a - b));
  }
  return maxD <= JOINT_ALIGNMENT_TOL_RAD;
}

/** 有新的关节指令或 IK 轨迹时调用：在真实硬件与指令未对齐前先显示仿真臂 */
function noteGhostShowsCommandAheadOfTelemetry() {
  if (robotHardware?.visible) {
    viewer.setGhostRobotVisible(true);
  }
}

/**
 * joint_pos 刷新：hardware 网格 + 面板；仿真臂在未对齐指令前不被遥测拉回，对齐后隐藏并入位。
 */
function refreshGhostVersusTelemetry(telemJoint) {
  if (!kinematics || !robotGhost) return;

  const hardwareActive = !!(robotHardware && robotHardware.visible);
  const aligned = telemetryMatchesGhostCommand(telemJoint);
  const updateGhostUrdfJoints = !hardwareActive || aligned;

  jointsUI.syncFromStreamData(telemJoint, { updateGhostUrdfJoints });
  viewer.setGhostRobotVisible(hardwareActive ? !aligned : true);
}

function vectorToMap(q) {
  if (!kinematics) return {};
  return kinematics.getJointNames().reduce((acc, name, idx) => {
    acc[name] = q[idx];
    return acc;
  }, {});
}

function isTargetJointMapReached(targetMap, toleranceRad = JOINT_ALIGNMENT_TOL_RAD) {
  if (!kinematics || !latestJointPosition?.length || !targetMap) return false;
  const names = kinematics.getJointNames();
  for (let i = 0; i < names.length; i++) {
    const target = targetMap[names[i]];
    const actual = latestJointPosition[i];
    if (!Number.isFinite(target) || !Number.isFinite(actual)) continue;
    if (Math.abs(target - actual) > toleranceRad) return false;
  }
  return true;
}

async function waitUntilTargetReached(targetMap, timeoutMs) {
  const start = Date.now();
  while (isBusy && Date.now() - start < timeoutMs) {
    if (isTargetJointMapReached(targetMap)) return true;
    await sleep(20);
  }
  return isTargetJointMapReached(targetMap);
}

function getCurrentPose() {
  if (!kinematics) return null;
  return kinematics.getEndEffectorPose();
}

function updateConnectionUi(kind = "preview") {
  const apiState = getApiState();
  const robotInfo =
    DEFAULT_ROBOTS.find((item) => item.id === apiState.activeRobotId) ||
    DEFAULT_ROBOTS[0];

  // activeRobotTextEl.textContent = `${robotInfo.name} · ${robotInfo.mode}`;
  robotIdTextEl.textContent = robotInfo.id;

  connectionBadgeEl.className = "badge";

  if (kind === "connect") {
    connectionBadgeEl.classList.add("badge-ok");
    connectionBadgeEl.textContent = "connected";
  } else if (kind === "disconnect") {
    connectionBadgeEl.classList.add("badge-muted");
    connectionBadgeEl.textContent = "disconnect";
  } else if (kind === "ready") {
    connectionBadgeEl.classList.add("badge-muted");
    connectionBadgeEl.textContent = "ready";
  }  else if (kind === "warn") {
    connectionBadgeEl.classList.add("badge-warn");
    connectionBadgeEl.textContent = "preview";
  } else if (kind === "danger") {
    connectionBadgeEl.classList.add("badge-danger");
    connectionBadgeEl.textContent = "error";
  }else {
    connectionBadgeEl.classList.add("badge-muted");
    connectionBadgeEl.textContent = "preview";
  }
}

/* -------------------------------------------------------------------------- */
/* Sync: Robot -> UI                                                           */
/* -------------------------------------------------------------------------- */

function refreshPoseReadout() {

  if (!kinematics) return;

  const pose = kinematics.getEndEffectorPose();

  if (!pose) return;

  // eePoseEl.textContent = formatPoseText(pose);

  viewer.updateTargetPose(pose);

}

function syncViewerFromRobot() {
  if (!kinematics) return;
  const pose = kinematics.getEndEffectorPose();
  viewer.updateTargetPose(pose);
}

function syncTaskUiFromRobot() {
  if (!kinematics) return;
  const pose = kinematics.getEndEffectorPose();
  taskUI.setPose(pose);
}

function syncViewerFromStreamData(position, quaternion) {
  const pose = quaternionToPose(position, quaternion)
  viewer.updateTargetPose(pose);
}

function syncMeta() {

  if (!kinematics) return;

  baseLinkEl.textContent = kinematics.baseLinkName;
  tipLinkEl.textContent = kinematics.tipLinkName;

  const pose = kinematics.getEndEffectorPose();

  if (!pose) return;

  refreshPoseReadout();
  taskUI.setPose(pose);

}

function syncAllFromRobot() {
  if (!kinematics) return;

  refreshPoseReadout();
  syncTaskUiFromRobot();
  syncViewerFromRobot();
}

/* -------------------------------------------------------------------------- */
/* Sync: Apply Joint / Apply Pose                                              */
/* -------------------------------------------------------------------------- */

function applyJointVector(q, options = {}) {
  if (!kinematics || !q) return false;

  const { syncJointUi = true, syncTaskUi = true, syncViewer = true } = options;

  withSyncGuard(() => {
    kinematics.setJointVector(q);

    const map = vectorToMap(q);

    if (syncJointUi) {
      jointsUI.setValuesByMap(map, true);
    }

    noteGhostShowsCommandAheadOfTelemetry();

    // refreshPoseReadout();

    // if (syncTaskUi) {
    //   syncTaskUiFromRobot();
    // }

    // if (syncViewer) {
    //   syncViewerFromRobot();
    // }
  });

  return true;
}

function applyJointMap(map, options = {}) {
  if (!kinematics || !map) return false;

  const names = kinematics.getJointNames();
  const q = names.map((name) => map[name] ?? 0);

  return applyJointVector(q, options);
}

function applyTaskPoseByIK(pose, options = {}) {
  if (!kinematics || !pose || ikBusy) return false;

  const {
    syncJointUi = true,
    syncTaskUi = true,
    syncViewer = true,
    setAsLastGoal = false,
  } = options;

  ikBusy = true;

  try {
    const q0 = kinematics.getCurrentJointVector();
    const result = kinematics.solveIK(pose, q0);

    if (!result || !result.q) return false;

    applyJointVector(result.q, {
      syncJointUi,
      syncTaskUi,
      syncViewer,
    });

    if (setAsLastGoal) {
      lastGoalPose = pose;
    }

    return true;
  } finally {
    ikBusy = false;
  }
}

/* -------------------------------------------------------------------------- */
/* UI Components                                                               */
/* -------------------------------------------------------------------------- */

const jointsUI = new JointsUI(jointContainerEl, jointCountEl, {
  onJointInput: async (name, value) => {
    if (!kinematics || isSyncing) return;

    // 实时下发命令到机器人（先发送命令，成功后再更新UI）
    try {
      const jointNames = Object.keys(kinematics.getCurrentJointMap());
      const jointValues = Object.values(kinematics.getCurrentJointMap());
      console.log(`[onJointInput] ${name}: Sending real-time command with value=${value.toFixed(6)} rad (${(value * 180 / Math.PI).toFixed(2)}°)`);
      
      const result = await sendJointCommand(
        jointNames,
        jointValues,
        rtInterpolationPayload(RT_INTERPOLATION.moveJoint)
      );
      
      if (result.mode === "preview") {
        console.warn(`[onJointInput] ${name}: Preview mode - no gateway configured`);
        // Preview 模式下仍然更新 UI
        withSyncGuard(() => {
          const currentMap = kinematics.getCurrentJointMap();
          currentMap[name] = value;
          kinematics.setJointMap(currentMap);
          refreshPoseReadout();
          syncTaskUiFromRobot();
          syncViewerFromRobot();
          noteGhostShowsCommandAheadOfTelemetry();
        });
      } else if (result.data && result.data.success) {
        console.log(`[onJointInput] ${name}: ✅ Real-time command succeeded`);
        // 成功后更新 UI
        withSyncGuard(() => {
          const currentMap = kinematics.getCurrentJointMap();
          currentMap[name] = value;
          kinematics.setJointMap(currentMap);
          refreshPoseReadout();
          syncTaskUiFromRobot();
          syncViewerFromRobot();
          noteGhostShowsCommandAheadOfTelemetry();
        });
      } else if (result.data && !result.data.success) {
        console.error(`[onJointInput] ${name}: ❌ Command failed: ${result.data.message}`);
        setStatus(`Failed to move joint: ${result.data.message}`, "danger-text");
        // 失败时不更新 UI，保持原状
      }
    } catch (error) {
      console.error(`[onJointInput] ${name}: Error sending command:`, error);
      setStatus(`Error moving joint: ${error.message}`, "danger-text");
    }
  },

  onJointCommitted: async (name, value) => {
    if (!kinematics || isSyncing) {
      console.warn(`[onJointCommitted] ${name}: Skipped (kinematics=${!!kinematics}, isSyncing=${isSyncing})`);
      return;
    }

    // 验证值的合法性
    if (value === undefined || value === null || isNaN(value)) {
      console.error(`[onJointCommitted] ${name}: Invalid value received: ${value}`);
      return;
    }

    console.log(`[onJointCommitted] ${name}: Received value=${value.toFixed(6)} rad (${(value * 180 / Math.PI).toFixed(2)}°)`);

    // 使用滑动条传递的实际目标值构建命令
    const map = kinematics.getCurrentJointMap();
    
    // 检查当前值与目标值的差异
    const currentValue = map[name];
    console.log(`[onJointCommitted] ${name}: Current kinematics value=${currentValue?.toFixed(6) || 'undefined'}, Target value=${value.toFixed(6)}`);
    
    map[name] = value; // 确保使用最新的目标值
    
    refreshPoseReadout();
    syncTaskUiFromRobot();
    syncViewerFromRobot();
    
    console.log(`[onJointCommitted] ${name}: Sending command to backend...`);
    try {
      const result = await sendJointCommand(
        Object.keys(map),
        Object.values(map),
        rtInterpolationPayload(RT_INTERPOLATION.moveJoint)
      );
      console.log(`[onJointCommitted] ${name}: Command result:`, result);
      
      if (result.mode === "preview") {
        setStatus("Preview mode active. No gateway configured.", "warn");
      } else if (result.data && result.data.success) {
        setStatus("Successfully sent joint command.", "ok");
        console.log(`[onJointCommitted] ${name}: ✅ Command succeeded`);
        noteGhostShowsCommandAheadOfTelemetry();
      } else if (result.data && !result.data.success) {
        setStatus(`Failed to sent joint command. ${result.data.message}`, "danger-text");
        console.error(`[onJointCommitted] ${name}: ❌ Command failed: ${result.data.message}`);
      } else {
        console.warn(`[onJointCommitted] ${name}: Unexpected result format:`, result);
      }
    } catch (error) {
      console.error(`[onJointCommitted] ${name}: Exception occurred:`, error);
      setStatus(`Error sending command: ${error.message}`, "danger-text");
    }
  },
}, {
  intervalMs: 100,  // 连续调节的时间间隔（毫秒），可根据需要调整
  stepDeg: 1        // 每次步进的角度，可根据需要调整
});

const kinematicsLab = new KinematicsLab(kinematicsLabContainerEl, {
  viewer,
  setStatus,
});

const taskUI = new TaskSpaceUI(taskSpaceContainerEl, {
  onReadCurrent: () => {
    if (!kinematics) return;
    taskUI.setPose(kinematics.getEndEffectorPose());
  },

  onMove: async (pose) => {
    if (!kinematics || isSyncing) return;

    // 🔧 选项1：通过 IK 求解并发送关节命令
    // const ok = applyTaskPoseByIK(pose, {
    //   syncJointUi: true,
    //   syncTaskUi: true,
    //   syncViewer: true,
    // });

    // if (!ok) {
    //   setStatus("IK solve failed for task move.", "warn");
    // }
    
    // 🔧 选项2：直接发送任务空间命令到后端
    
    try {
      console.log(`[TaskSpace onMove] Sending absolute pose command:`, pose);
      const poseArray = [pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz];
      const result = await sendPoseCommand(poseArray);
      
      if (result.mode === "preview") {
        console.warn(`[TaskSpace onMove] Preview mode - no gateway configured`);
      } else if (result.data && result.data.success) {
        console.log(`[TaskSpace onMove] ✅ Absolute pose command succeeded`);
        setStatus("Task-space absolute command sent.", "ok");
      } else if (result.data && !result.data.success) {
        console.error(`[TaskSpace onMove] ❌ Command failed: ${result.data.message}`);
        setStatus(`Failed to send absolute pose command. ${result.data.message}`, "danger-text");
      }
    } catch (error) {
      console.error(`[TaskSpace onMove] Error sending command:`, error);
      setStatus(`Error sending absolute pose command: ${error.message}`, "danger-text");
    }
    
  },

  onMoveIncremental: async (deltaPose) => {
    if (!kinematics || isSyncing) return;

    try {
      console.log(`[TaskSpace onMoveIncremental] Sending incremental pose command:`, deltaPose);
      const deltaPoseArray = [deltaPose.x, deltaPose.y, deltaPose.z, deltaPose.rx, deltaPose.ry, deltaPose.rz];
      const result = await sendPoseIncrementalCommand(deltaPoseArray);
      
      if (result.mode === "preview") {
        console.warn(`[TaskSpace onMoveIncremental] Preview mode - no gateway configured`);
      } else if (result.data && result.data.success) {
        console.log(`[TaskSpace onMoveIncremental] ✅ Incremental pose command succeeded`);
        setStatus("Task-space incremental command sent.", "ok");
      } else if (result.data && !result.data.success) {
        console.error(`[TaskSpace onMoveIncremental] ❌ Command failed: ${result.data.message}`);
        setStatus(`Failed to send incremental pose command. ${result.data.message}`, "danger-text");
      }
    } catch (error) {
      console.error(`[TaskSpace onMoveIncremental] Error sending command:`, error);
      setStatus(`Error sending incremental pose command: ${error.message}`, "danger-text");
    }
  },

  onSetGoal: (pose) => {
    lastGoalPose = pose;
    setStatus("Task-space goal snapshot captured.", "ok");
  },

  onPlanPose: () => {
    document.getElementById("planCartesianBtn").click();
  },
}, {
  intervalMs: 1000,     // 连续调节的时间间隔（毫秒），可根据需要调整
  stepTrans: 0.01,     // 平移每次步进的米数，可根据需要调整
  stepRot: 1,          // 旋转每次步进的角度，可根据需要调整
  controlMode: 0       // 默认控制模式：1=绝对位姿, 0=增量位姿
});

taskUI.build();

/* -------------------------------------------------------------------------- */
/* Viewer callbacks                                                            */
/* -------------------------------------------------------------------------- */

viewer.callbacks.onTaskMove = (pose) => {
  if (!kinematics || isSyncing) return;

  const ok = applyTaskPoseByIK(pose, {
    syncJointUi: true,
    syncTaskUi: true,
    syncViewer: false, // viewer 自己已经在这个 pose 上了
  });

  if (!ok) {
    setStatus("IK solve failed for dragged target.", "warn");
  }
};

/* -------------------------------------------------------------------------- */
/* Robot loading                                                               */
/* -------------------------------------------------------------------------- */

async function loadCurrentRobot(path) {
  try {
    setStatus("Loading URDF...", "warn");

    const [ghost, hardware] = await Promise.all([
      loadRobotFromUrdf(path),
      loadRobotFromUrdf(path),
    ]);

    robotGhost = ghost;
    robotHardware = hardware;

    /* 打散加载器对不同实例复用的 Material，否则两臂会改到同一实例 */
    cloneMaterialsPerMesh(robotGhost);
    cloneMaterialsPerMesh(robotHardware);

    applyHardwareContrastStyle(robotHardware);
    applyGhostVisualStyle(robotGhost);
    viewer.setDualRobot(robotHardware, robotGhost);
    viewer.setHardwareRobotVisible(false);
    viewer.setGhostRobotVisible(true);

    kinematics = new RobotKinematics(robotGhost);
    kinematicsLab.setRobotContext(kinematics);

    jointsUI.build(robotGhost);
    teach.syncTeachJointMirror();

    syncMeta();

    localStorage.setItem(STORAGE_KEYS.lastUrdfPath, path);

    setStatus("URDF loaded.", "ok");
    return true;
  } catch (error) {
    console.error(error);
    setStatus(`Failed to load URDF: ${error.message || error}`, "danger-text");
    return false;
  }
}

/* -------------------------------------------------------------------------- */
/* Robot selector / gateway                                                    */
/* -------------------------------------------------------------------------- */

function populateRobotSelector() {
  robotSelectEl.innerHTML = "";

  DEFAULT_ROBOTS.forEach((item) => {
    const option = document.createElement("option");
    option.value = item.id;
    option.textContent = `${item.name}${
      item.ip && item.ip !== "-" ? ` (${item.ip})` : ""
    }`;
    robotSelectEl.appendChild(option);
  });
}

async function saveGateway() {
  const url = gatewayUrlEl.value.trim();

  setGatewayUrl(url);
  connectStream(url);
  localStorage.setItem(STORAGE_KEYS.gatewayUrl, url);

  setStatus(
    url
      ? "Gateway URL saved."
      : "Gateway cleared. Preview mode enabled.",
    "ok"
  );

  updateConnectionUi(url ? "ready" : "warn");
}

async function connectSelectedRobot() {
  try {
    const result = await connectRobot();

    updateConnectionUi(result.mode === "preview" ? "warn" : "connect");

    if (result.mode === "preview") {
      setStatus("Preview mode active. No gateway configured.", "warn");
    }else if (result.data.success) {
      setStatus("Robot connected to hardware.", "ok");
    }else if (!result.data.success) {
      setStatus(`Failed to connect to hardware. ${result.data.message}`, "danger-text");
    }

  } catch (error) {
    updateConnectionUi("danger");
    setStatus(error.message || "Connect failed.", "danger-text");
  }
}

/* -------------------------------------------------------------------------- */
/* Trajectory                                                                  */
/* -------------------------------------------------------------------------- */

async function executeTrajectory(trajectory) {
  if (!kinematics || !trajectory?.length) return;

  isBusy = true;
  let stoppedEarly = false;
  setStatus(`Playing ${trajectory.length} waypoints...`, "warn");

  for (let i = 0; i < trajectory.length; i++) {
    if (!isBusy) {
      stoppedEarly = true;
      break;
    }

    const map = trajectory[i];

    applyJointMap(map, {
      syncJointUi: true,
      syncTaskUi: true,
      syncViewer: true,
    });

    const teachInterp =
      i === 0
        ? rtInterpolationPayload(RT_INTERPOLATION.teach.first)
        : rtInterpolationPayload(RT_INTERPOLATION.teach.rest);
    const result = await sendJointCommand(
      Object.keys(map),
      Object.values(map),
      teachInterp
    );

    if (i === 0) {
      const firstAccSec = Number(teachInterp?.interpolation_acc_time);
      const timeoutMs =
        Number.isFinite(firstAccSec) && firstAccSec > 0
          ? firstAccSec * 1000 + 2000
          : 7000;

      // Wait for first waypoint convergence before dispatching remaining waypoints.
      if (result.mode === "preview") {
        await sleep(timeoutMs);
      } else {
        const reached = await waitUntilTargetReached(map, timeoutMs);
        if (!reached) {
          console.warn(
            `[executeTrajectory] First waypoint not confirmed within ${timeoutMs} ms, continuing playback.`
          );
        }
      }
    } else {
      await sleep(getPlayDelay());
    }
  }

  isBusy = false;
  if (stoppedEarly) {
    setStatus("Path playback stopped.", "warn");
    return false;
  }
  setStatus("Path playback completed.", "ok");
  return true;
}

const teach = createTeachModule({
  elements: {
    teachCountEl,
    pathPreviewEl,
    jointContainerTeachEl,
    jointContainerEl,
    teachRecordBtnEl,
    teachPlayBtnEl,
    teachStopBtnEl,
  },
  getKinematics: () => kinematics,
  setStatus,
  executeTrajectory,
  sendStopCommand,
  onClearBusy: () => {
    isBusy = false;
  },
});
const teachSystem = teach.system;

/* -------------------------------------------------------------------------- */
/* Buttons                                                                     */
/* -------------------------------------------------------------------------- */

function bindButtons() {
  document.getElementById("loadUrdfBtn").onclick = () => {
    loadCurrentRobot(urdfPathEl.value.trim() || DEFAULT_URDF_PATH);
  };

  // document.getElementById("fitBtn").onclick = () => {
  //   viewer.fitToRobot();
  // };

  // document.getElementById("resetViewBtn").onclick = () => {
  //   viewer.resetView();
  // };

  // document.getElementById("refreshPoseBtn").onclick = () => {
  //   refreshPoseReadout();
  //   syncViewerFromRobot();
  //   syncTaskUiFromRobot();
  // };

  // document.getElementById("readCurrentPoseBtn").onclick = () => {
  //   if (!kinematics) return;
  //   taskUI.setPose(kinematics.getEndEffectorPose());
  // };

  // document.getElementById("captureJointGoalBtn").onclick = () => {
  //   if (!kinematics) return;

  //   lastGoalMap = kinematics.getCurrentJointMap();
  //   setStatus("Joint-space goal snapshot captured.", "ok");
  // };

  // document.getElementById("capturePoseGoalBtn").onclick = () => {
  //   if (!kinematics) return;

  //   lastGoalPose = kinematics.getEndEffectorPose();
  //   taskUI.setPose(lastGoalPose);

  //   setStatus("Task-space goal snapshot captured.", "ok");
  // };

  document.getElementById("homeBtn").onclick = async () => {
    if (!kinematics) return;
    viewer.fitToRobot();
    const zeroQ = kinematics.getCurrentJointVector().map(() => 0);
    const jointNames = kinematics.getJointNames();

    applyJointVector(zeroQ, {
      syncJointUi: true,
      syncTaskUi: true,
      syncViewer: true,
    });

    const result = await sendHomeCommand(
      jointNames,
      zeroQ,
      rtInterpolationPayload(RT_INTERPOLATION.home)
    );
    if (result.mode === "preview") {
      setStatus("Preview mode active. No gateway configured.", "warn");
    }else if (result.data.success) {
      setStatus("Successfully moved to home position.", "ok");
    }else if (!result.data.success) {
      setStatus(`Failed to move to home position. ${result.data.message}`, "danger-text");
    }
  };

  // document.getElementById("zeroBtn").onclick = async () => {
  //   if (!kinematics) return;

  //   const zeroQ = kinematics.getCurrentJointVector().map(() => 0);

  //   applyJointVector(zeroQ, {
  //     syncJointUi: true,
  //     syncTaskUi: true,
  //     syncViewer: true,
  //   });

  //   const result = await sendZeroCommand(Object.keys(zeroQ), Object.values(zeroQ));
  //   if (result.mode === "preview") {
  //     setStatus("Preview mode active. No gateway configured.", "warn");
  //   }else if (result.data.success) {
  //     setStatus("All joints set to zero.", "ok");
  //   }else if (!result.data.success) {
  //     setStatus(`Failed to set all joints to zero. ${result.data.message}`, "danger-text");
  //   }
  // };

  // document.getElementById("stopBtn").onclick = async () => {
  //   isBusy = false;
  //   await sendStopCommand();
  //   setStatus("Stop requested.", "warn");
  // };

  // document.getElementById("recordPoseBtn").onclick = () => {
  //   if (!kinematics) return;

  //   teachSystem.record(kinematics.getCurrentJointMap());
  //   teach.updateTeachUi();

  //   setStatus("Current pose recorded.", "ok");
  // };

  // document.getElementById("clearPathBtn").onclick = () => {
  //   teachSystem.clear();
  //   teach.updateTeachUi();
  //   setStatus("Path cleared.", "ok");
  // };

  // document.getElementById("savePathBtn").onclick = () => {
  //   saveTrajectoryToFile(teachSystem.getPath());
  //   setStatus("Trajectory file saved.", "ok");
  // };

  // document.getElementById("playPathBtn").onclick = async () => {
  //   await executeTrajectory(teachSystem.getPath());
  // };

  // document.getElementById("planJointBtn").onclick = () => {
  //   if (!kinematics) return;

  //   if (!lastGoalMap) {
  //     lastGoalMap = kinematics.getCurrentJointMap();
  //     setStatus(
  //       "Saved current joint state as the goal snapshot. Move the arm, then click again to plan.",
  //       "warn"
  //     );
  //     return;
  //   }

  //   const current = kinematics.getCurrentJointMap();
  //   const trajectory = planJointTrajectory(current, lastGoalMap, getPlanSteps());

  //   teachSystem.replaceAll(trajectory);
  //   teach.updateTeachUi();

  //   setStatus("Joint trajectory generated.", "ok");
  // };

  // document.getElementById("planCartesianBtn").onclick = () => {
  //   if (!kinematics) return;

  //   const goalPose = lastGoalPose || taskUI.getPose();
  //   const startPose = kinematics.getEndEffectorPose();

  //   const trajectory = planCartesianTrajectory(
  //     kinematics,
  //     startPose,
  //     goalPose,
  //     getPlanSteps()
  //   );

  //   teachSystem.replaceAll(trajectory);
  //   teach.updateTeachUi();

  //   setStatus("Cartesian trajectory generated via IK.", "ok");
  // };

  // document.getElementById("pathFile").onchange = async (event) => {
  //   const file = event.target.files?.[0];
  //   if (!file) return;

  //   const trajectory = await loadTrajectoryFromFile(file);

  //   teachSystem.replaceAll(trajectory);
  //   teach.updateTeachUi();

  //   setStatus("Trajectory loaded.", "ok");
  //   event.target.value = "";
  // };

  document.getElementById("urdfFile").onchange = async (event) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const text = await readFileAsText(file);
    const blob = new Blob([text], { type: "application/xml" });
    const url = URL.createObjectURL(blob);

    urdfPathEl.value = file.name;

    await loadCurrentRobot(url);

    event.target.value = "";
  };

  document.getElementById("saveGatewayBtn").onclick = saveGateway;

  document.getElementById("connectBtn").onclick = connectSelectedRobot;

  document.getElementById("disconnectBtn").onclick = async () => {
    try {
      const result = await disconnectRobot();

      updateConnectionUi(result.mode === "preview" ? "warn" : "disconnect");

      if (result.mode === "preview") {
        setStatus("Preview mode active. No gateway configured.", "warn");
      }else if (result.data.success) {
        setStatus("Robot disconnected to hardware.", "ok");
      }else if (!result.data.success) {
        setStatus(`Failed to connect to hardware. ${result.data.message}`, "danger-text");
      }
      
    } catch (error) {
      updateConnectionUi("danger");
      setStatus(error.message || "Disconnect failed.", "danger-text");
    }
  };

  document.getElementById("pingBtn").onclick = async () => {
    try {
      const result = await pingGateway();
      if (result.mode === "preview") updateConnectionUi("warn");

      setStatus(
        result.mode === "preview"
          ? "Preview mode ping."
          : "Gateway ping succeeded.",
        result.mode === "preview" ? "warn" : "ok"
      );

    } catch (error) {
      updateConnectionUi("danger");
      setStatus(error.message || "Ping failed.", "danger-text");
    }
  };

  teach.bindTeachButtons();

   // Robot Manager floating panel toggle
  const mgrBtn = document.getElementById("openRobotManagerBtn");
  const mgrPanel = document.getElementById("robotManagerFloating");
  if (mgrBtn && mgrPanel) {
    mgrBtn.onclick = (e) => {
      e.stopPropagation();
      const isActive = mgrPanel.classList.toggle("active");
      mgrPanel.setAttribute("aria-hidden", isActive ? "false" : "true");
      
      if (isActive) {
        const btnRect = mgrBtn.getBoundingClientRect();
        const panelWidth = 380;
        const panelHeight = mgrPanel.scrollHeight || 400; 
        const viewportWidth = window.innerWidth;
        const viewportHeight = window.innerHeight;
        
        let leftPosition = btnRect.left;
        
        if (leftPosition + panelWidth > viewportWidth) {
          leftPosition = viewportWidth - panelWidth - 10; 
        }
        
        if (leftPosition < 10) {
          leftPosition = 10;
        }
        
        let topPosition = btnRect.bottom + 8;
        
        if (topPosition + panelHeight > viewportHeight) {
          topPosition = btnRect.top - panelHeight - 8;
        }
        
        mgrPanel.style.left = leftPosition + "px";
        mgrPanel.style.top = topPosition + "px";
        mgrPanel.style.right = "auto"; 
      }
    };
    
    document.addEventListener("click", (ev) => {
      if (!mgrPanel.classList.contains("active")) return;
      if (mgrPanel.contains(ev.target) || mgrBtn.contains(ev.target)) return;
      mgrPanel.classList.remove("active");
      mgrPanel.setAttribute("aria-hidden", "true");
    });
  }

  robotSelectEl.onchange = () => {
    setActiveRobot(robotSelectEl.value);
    localStorage.setItem(STORAGE_KEYS.robotId, robotSelectEl.value);

    updateConnectionUi(gatewayUrlEl.value.trim() ? "warn" : "warn");

    setStatus(
      `Switched active robot to ${
        robotSelectEl.selectedOptions[0]?.textContent || robotSelectEl.value
      }.`,
      "ok"
    );
  };
}

function bindCardTabs() {
  const init = () => {
    const tabs = document.querySelectorAll(".tab-btn");
    const pages = document.querySelectorAll(".tab-page");
    const viewerPanelEl = document.querySelector(".viewer-panel");

    if (!tabs.length) return;

    const syncTeachVisibility = (pageId) => {
      const teachActive = pageId === "planning";
      viewerPanelEl?.classList.toggle("teach-active", teachActive);
      if (teachActive) {
        teach.syncTeachJointMirror();
      }
    };

    const initialActiveTab = document.querySelector(".tab-btn.active");
    syncTeachVisibility(initialActiveTab?.dataset.page || "");

    tabs.forEach((tab) => {
      tab.addEventListener("click", (e) => {
        e.preventDefault();
        const pageId = tab.dataset.page;

        // Remove active from all tabs and pages
        tabs.forEach((t) => t.classList.remove("active"));
        pages.forEach((p) => p.classList.remove("active"));

        // Add active to clicked tab
        tab.classList.add("active");

        // Add active to corresponding page
        const activePage = document.querySelector(`.tab-page[data-page="${pageId}"]`);
        if (activePage) {
          activePage.classList.add("active");
        }

        syncTeachVisibility(pageId);
      });
    });
  };

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
}

function connectStream(url) {
  const gatewayUrl = (url || "").trim().replace(/\/+$/, "");

  if (robotStreamRetryTimer) {
    clearTimeout(robotStreamRetryTimer);
    robotStreamRetryTimer = null;
  }

  if (robotStream) {
    robotStream.close();
    robotStream = null;
  }

  if (!gatewayUrl) {
    viewer.setHardwareRobotVisible(false);
    viewer.setGhostRobotVisible(true);
    return;
  }

  const normalizeStreamArray = (value) => {
    if (!Array.isArray(value)) return null;
    return Array.isArray(value[0]) ? value[0] : value;
  };

  robotStream = new EventSource(`${gatewayUrl}/stream`);

  robotStream.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      const eePose = normalizeStreamArray(data.ee_pose);
      const jointPosition = normalizeStreamArray(data.joint_pos);

      if (eePose?.length >= 3) {
        updateEePoseCard(eePose.slice(0, 3));
        
        // eePose 包含完整的6维位姿信息，同步到 TaskSpaceUI
        if (eePose.length >= 6 && kinematics) {
          const pose = {
            x: eePose[0],
            y: eePose[1],
            z: eePose[2],
            rx: THREE.MathUtils.radToDeg(eePose[3]),
            ry: THREE.MathUtils.radToDeg(eePose[4]),
            rz: THREE.MathUtils.radToDeg(eePose[5])
          };
          taskUI.syncFromStreamData(pose);
        }
      }

      if (jointPosition?.length) {
        latestJointPosition = jointPosition.slice();
        if (kinematics && robotHardware) {
          applyJointVectorToUrdfRobot(robotHardware, jointPosition);
          viewer.setHardwareRobotVisible(true);
        }
        refreshGhostVersusTelemetry(jointPosition);
        teach.syncTeachJointMirror();
      }
    } catch (error) {
      console.error("Failed to parse stream payload:", error, event.data);
    }
  };

  robotStream.onerror = (err) => {
    console.error("Stream failed:", err);

    if (robotStream) {
      robotStream.close();
      robotStream = null;
    }

    robotStreamRetryTimer = setTimeout(() => {
      connectStream(gatewayUrl);
    }, RETRY_DELAY);
  };
}

// Update End Effector Pose Card display
function updateEePoseCard(position) {
  if (!position || position.length < 3) return;
  
  const eePoseXEl = document.getElementById("eePoseX");
  const eePoseYEl = document.getElementById("eePoseY");
  const eePoseZEl = document.getElementById("eePoseZ");
  
  if (eePoseXEl) eePoseXEl.textContent = position[0].toFixed(4);
  if (eePoseYEl) eePoseYEl.textContent = position[1].toFixed(4);
  if (eePoseZEl) eePoseZEl.textContent = position[2].toFixed(4);
}

/* -------------------------------------------------------------------------- */
/* Boot                                                                        */
/* -------------------------------------------------------------------------- */

(async function boot() {
  populateRobotSelector();

  const savedGateway = localStorage.getItem(STORAGE_KEYS.gatewayUrl) || "";
  const savedRobotId =
    localStorage.getItem(STORAGE_KEYS.robotId) || DEFAULT_ROBOTS[0].id;
  const savedUrdf =
    localStorage.getItem(STORAGE_KEYS.lastUrdfPath) || DEFAULT_URDF_PATH;
  const savedTraj = localStorage.getItem(STORAGE_KEYS.lastTrajectory);

  gatewayUrlEl.value = savedGateway;
  setGatewayUrl(savedGateway);

  robotSelectEl.value = savedRobotId;
  setActiveRobot(savedRobotId);

  urdfPathEl.value = savedUrdf;

  updateConnectionUi(savedGateway ? "ready" : "warn");

  bindButtons();
  bindCardTabs();
  teach.refreshTeachControls();

  // if (savedTraj) {
  //   try {
  //     teachSystem.replaceAll(JSON.parse(savedTraj));
  //     teach.updateTeachUi();
  //   } catch {
  //     teachSystem.clear();
  //     teach.updateTeachUi();
  //   }
  // } else {
  //   teach.updateTeachUi();
  // }

  connectStream(savedGateway);

  const loaded = await loadCurrentRobot(savedUrdf);
  if (!loaded && savedUrdf !== DEFAULT_URDF_PATH) {
    urdfPathEl.value = DEFAULT_URDF_PATH;
    await loadCurrentRobot(DEFAULT_URDF_PATH);
  }
})();