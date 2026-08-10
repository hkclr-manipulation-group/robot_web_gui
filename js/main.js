import {
  CONTINUOUS_RANGE,
  DEFAULT_ROBOTS,
  GRIPPER,
  PATH_DEFAULTS,
  ROTARY_FALLBACK_RANGE,
  RT_INTERPOLATION,
  SLIDER_CONTROL,
  STORAGE_KEYS,
  VIEWER,
  findRobotByName,
  getUrdfPathForRobot,
  rtInterpolationPayload,
} from "./config.js";

import {
  connectRobot,
  disconnectRobot,
  getApiState,
  isHardwareControlActive,
  isGatewayActive,
  pingGateway,
  sendGripperCommand,
  sendHomeCommand,
  sendJogCartesianCommand,
  sendJogJointCommand,
  sendJogStopCommand,
  sendJointCommand,
  sendPoseCommand,
  sendPoseIncrementalCommand,
  sendZeroCommand,
  setActiveRobot,
  setGatewayUrl,
} from "./api.js";

import { JointsUI } from "./joints-ui.js";
import { JointsInputUI } from "./joints-input-ui.js";
import { TaskInputUI } from "./task-input-ui.js";
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
import { createStreamEulerStabilizer, formatEePoseValue, formatJointInput, formatPoseText, sleep, quaternionToPose } from "./utils.js";
import { RobotViewer } from "./viewer.js";
import { initFullscreen } from "./fullscreen.js";
import { copyEnvInfoToClipboard } from "./env-info.js";
import * as THREE from "three";

/* ----------6---------------------------------------------------------------- */
/* DOM                                                                         */
/* -------------------------------------------------------------------------- */

const statusEl = document.getElementById("status");
const gatewayUrlEl = document.getElementById("gatewayUrl");
const robotSelectEl = document.getElementById("robotSelect");

const connectionBadgeEl = document.getElementById("connectionBadge");
let viewerInitFailure = false;

function createViewerFallback() {
  const noOp = () => {};
  return {
    callbacks: {},
    setDualRobot: noOp,
    setRobot: noOp,
    setHardwareRobotVisible: noOp,
    setGhostRobotVisible: noOp,
    fitToRobot: noOp,
    resetView: noOp,
    updateTargetPose: noOp,
    setTransformMode: noOp,
    getTransformMode: () => "translate",
    setTransformFrame: noOp,
    getTransformFrame: () => "base",
    setTaskGizmoVisible: noOp,
    setLabMarkers: noOp,
    setLabTrajectories: noOp,
    clearLabVisualization: noOp,
  };
}

const viewer = (() => {
  try {
    return new RobotViewer(document.getElementById("viewer"));
  } catch (error) {
    console.error("RobotViewer init failed:", error);
    viewerInitFailure = true;
    setStatus(
      "WebGL initialization failed. ",
      "danger-text"
    );
    return createViewerFallback();
  }
})();
const activeRobotTextEl = document.getElementById("activeRobotText");
const robotIdTextEl = document.getElementById("robotIdText");

const jointCountEl = document.getElementById("jointCount");
const jointContainerEl = document.getElementById("jointContainer");
const jointJogPanelEl = document.getElementById("jointJogPanel");
const jointInputPanelEl = document.getElementById("jointInputPanel");
const jointInputListEl = document.getElementById("jointInputList");
const moveJointsBtnEl = document.getElementById("moveJointsBtn");
const saveInitialPoseBtnEl = document.getElementById("saveInitialPoseBtn");
const saveInitialPoseIconEl = document.getElementById("saveInitialPoseIcon");
const jointKeypadEl = document.getElementById("jointKeypad");
const initialPoseBtnEl = document.getElementById("initialPoseBtn");
const taskSpaceContainerEl = document.getElementById("taskSpaceContainer");
const taskJogPanelEl = document.getElementById("taskJogPanel");
const taskInputPanelEl = document.getElementById("taskInputPanel");
const taskInputListEl = document.getElementById("taskInputList");
const movePoseBtnEl = document.getElementById("movePoseBtn");
const taskKeypadEl = document.getElementById("taskKeypad");
const kinematicsLabContainerEl = document.getElementById("kinematicsLabPage");
const jointContainerTeachEl = document.getElementById("jointContainerTeach");
const teachRecordBtnEl = document.getElementById("teachRecordBtn");
const teachPlayBtnEl = document.getElementById("teachPlayBtn");
const teachStopBtnEl = document.getElementById("teachStopBtn");

const eePoseEl = document.getElementById("eePose");
const gripperCardEl = document.getElementById("gripperCard");
const gripperSliderEl = document.getElementById("gripperSlider");
const gripperValueEl = document.getElementById("gripperValue");
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
/** User issued a joint/task command not yet matched by telemetry (ghost = target). */
let ghostCommandPending = false;
/** Teach tab (`planning` page) active — ghost robot stays hidden while true. */
let teachTabActive = false;
/** Populated in createTeachModule(); null until then (avoids TDZ in early sync helpers). */
let teach = null;
/** Duplicate URDF driven by streamed joint telemetry (full materials). */
let robotHardware = null;
let kinematics = null;

let isBusy = false;
let isSyncing = false; // 防止 UI/FK/IK 相互触发造成循环
let ikBusy = false;    // 防止拖动时重复进入 IK
let gripperInteracting = false;
let gripperSendTimer = null;
let gripperBound = false;
/** Runtime slider limits from loaded URDF `gripper_J1` (fallback: GRIPPER.min/max). */
let gripperLimits = { min: GRIPPER.min, max: GRIPPER.max };

let lastGoalMap = null;
let lastGoalPose = null;

const RETRY_DELAY = 3000;
let robotStream = null; // EventSource for streaming robot data from gateway
let robotStreamRetryTimer = null;
let lastStreamError = "";
const streamEulerStabilizer = createStreamEulerStabilizer();
let latestJointPosition = null; // latest joint_pos from /stream

/* -------------------------------------------------------------------------- */
/* Utilities                                                                   */
/* -------------------------------------------------------------------------- */

/** Status bar helper. cls: "ok" | "warn" | "danger-text" | "" */
function setStatus(text, cls = "") {
  if (!statusEl) return;
  if (viewerInitFailure && cls !== "danger-text") {
    return;
  }
  statusEl.textContent = text;
  statusEl.className = `status-text ${cls}`.trim();
}

function clearStatusIf(text) {
  if (statusEl?.textContent === text) {
    setStatus("Ready.");
  }
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

function applyGhostRobotVisibility(desiredVisible) {
  const show =
    VIEWER.showGhostRobot && !!desiredVisible;
  viewer.setGhostRobotVisible(show);
}

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
  ghostCommandPending = true;
  if (robotHardware?.visible) {
    applyGhostRobotVisibility(true);
  }
}

/** Teach 关节角与 Joint Space 共用同一套数值源。 */
function syncTeachMirrorFromJointMap(map) {
  if (!map || !teach) return;
  teach.syncTeachJointMirror(map);
}

function syncTeachMirrorFromJointVector(q) {
  if (!q?.length || !kinematics) return;
  syncTeachMirrorFromJointMap(vectorToMap(q));
}

/** 无用户目标时：ghost / 指令状态跟遥测对齐（连接后不应留在 URDF 零位）。 */
function syncCommandStateFromTelemetry(telemJoint, options = {}) {
  if (!kinematics || !robotGhost || !telemJoint?.length) return;

  const { syncJointUi = true } = options;
  const names = kinematics.getJointNames();
  const q = telemJoint.slice(0, names.length);

  withSyncGuard(() => {
    applyJointVectorToUrdfRobot(robotGhost, q);
    kinematics.setJointVector(q);
    if (syncJointUi) {
      jointsUI.syncFromStreamData(q, { updateGhostUrdfJoints: false });
      syncTeachMirrorFromJointVector(q);
    }
    refreshPoseReadout();
    syncTaskUiFromRobot();
  });
}

/**
 * joint_pos 刷新：hardware 网格 + 面板；有 pending 目标时 ghost 显示指令，否则跟遥测重合并隐藏。
 */
function refreshGhostVersusTelemetry(telemJoint) {
  if (!kinematics || !robotGhost) return;

  const hardwareActive = !!(robotHardware && robotHardware.visible);

  if (!ghostCommandPending) {
    syncCommandStateFromTelemetry(telemJoint, { syncJointUi: !isSyncing });
    applyGhostRobotVisibility(false);
    return;
  }

  const aligned = telemetryMatchesGhostCommand(telemJoint);
  const updateGhostUrdfJoints = !hardwareActive || aligned;

  jointsUI.syncFromStreamData(telemJoint, { updateGhostUrdfJoints });
  syncTeachMirrorFromJointVector(telemJoint);
  applyGhostRobotVisibility(hardwareActive ? !aligned : true);

  if (aligned) {
    ghostCommandPending = false;
  }
}

function isLocalPreviewOnly() {
  return !isGatewayActive();
}

function applyLocalJointMap(map) {
  withSyncGuard(() => {
    kinematics.setJointMap(map);
    refreshPoseReadout();
    syncTaskUiFromRobot();
    syncViewerFromRobot();
    noteGhostShowsCommandAheadOfTelemetry();
    syncTeachMirrorFromJointMap(map);
  });
  // Keep Input fields in lockstep with slider / local joint changes.
  jointsInputUI.seedFromRadians(map, { force: true });
  if (jointPanelMode === "input") {
    jointsInputUI.updateMatchState(getActualJointMap());
  }
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

function poseRadToUiDeg(pose) {
  if (!pose) return null;
  return {
    x: pose.x,
    y: pose.y,
    z: pose.z,
    rx: THREE.MathUtils.radToDeg(pose.rx || 0),
    ry: THREE.MathUtils.radToDeg(pose.ry || 0),
    rz: THREE.MathUtils.radToDeg(pose.rz || 0),
  };
}

function poseUiDegToRad(pose) {
  if (!pose) return null;
  return {
    x: pose.x,
    y: pose.y,
    z: pose.z,
    rx: THREE.MathUtils.degToRad(pose.rx || 0),
    ry: THREE.MathUtils.degToRad(pose.ry || 0),
    rz: THREE.MathUtils.degToRad(pose.rz || 0),
  };
}

function poseToArray(pose) {
  return [pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz];
}

function transformPoseToRobotPose(transformPose) {
  if (!transformPose) return null;

  if (transformPose.position && transformPose.quaternion) {
    const euler = new THREE.Euler().setFromQuaternion(transformPose.quaternion, "XYZ");
    return {
      x: transformPose.position.x,
      y: transformPose.position.y,
      z: transformPose.position.z,
      rx: euler.x,
      ry: euler.y,
      rz: euler.z,
    };
  }

  return transformPose.x !== undefined ? transformPose : null;
}

function updateConnectionUi(kind = "preview") {
  const apiState = getApiState();
  const robotInfo = apiState.robot || findRobotByName(apiState.activeRobotName);

  // activeRobotTextEl.textContent = `${robotInfo.name} · ${robotInfo.mode}`;
  robotIdTextEl.textContent = robotInfo.id;

  connectionBadgeEl.className = "badge";

  if (kind === "connect") {
    connectionBadgeEl.classList.add("badge-ok");
    connectionBadgeEl.textContent = "connected";
  } else if (kind === "disconnect" || kind === "ready") {
    // Gateway on, hardware off → rt_control simulation
    connectionBadgeEl.classList.add("badge-muted");
    connectionBadgeEl.textContent = "simulation";
  } else if (kind === "warn") {
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

function refreshPoseReadout({ syncViewer = true } = {}) {

  if (!kinematics) return;

  const pose = kinematics.getEndEffectorPose();

  if (!pose) return;

  // eePoseEl.textContent = formatPoseText(pose);

  updateEePoseCard([pose.x, pose.y, pose.z]);

  if (syncViewer) {
    viewer.updateTargetPose(pose);
  }

}

function syncViewerFromRobot() {
  if (!kinematics) return;
  const pose = kinematics.getEndEffectorPose();
  viewer.updateTargetPose(pose);
}

function syncTaskUiFromRobot() {
  if (!kinematics) return;
  const pose = kinematics.getEndEffectorPose();
  const uiPose = poseRadToUiDeg(pose);
  taskUI.setPose(uiPose);
  syncTaskInputFromPose(uiPose, { force: false });
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
  const uiPose = poseRadToUiDeg(pose);
  taskUI.setPose(uiPose);
  syncTaskInputFromPose(uiPose, { force: false });

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
      // Teach mirror follows the same joint values as Joint Space.
      syncTeachMirrorFromJointMap(map);
      jointsInputUI.seedFromRadians(map, { force: true });
      if (jointPanelMode === "input") {
        jointsInputUI.updateMatchState(getActualJointMap());
      }
    }

    noteGhostShowsCommandAheadOfTelemetry();

    kinematics.setJointMap(map);
    refreshPoseReadout({ syncViewer });

    if (syncTaskUi) {
      syncTaskUiFromRobot();
    }

    if (syncViewer) syncViewerFromRobot();
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
    ikOptions = {},
  } = options;

  ikBusy = true;

  try {
    const q0 = kinematics.getCurrentJointVector();
    const result = kinematics.solveIK(pose, q0, ikOptions);

    if (!result?.success || !result.q) {
      kinematics.setJointVector(q0);
      return false;
    }

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

/** URDF joint limits for a named revolute joint (rad). Continuous joints are unbounded here. */
function getUrdfJointLimitRad(name) {
  const joint = kinematics?.robot?.joints?.[name];
  if (!joint) return { ...ROTARY_FALLBACK_RANGE, continuous: false };
  if (joint.jointType === "continuous") {
    return { ...CONTINUOUS_RANGE, continuous: true };
  }
  let min = ROTARY_FALLBACK_RANGE.min;
  let max = ROTARY_FALLBACK_RANGE.max;
  if (joint.limit) {
    if (Number.isFinite(joint.limit.lower)) min = joint.limit.lower;
    if (Number.isFinite(joint.limit.upper)) max = joint.limit.upper;
  }
  return { min, max, continuous: false };
}

/**
 * @param {number[]} q joint vector in kinematics order (base→tip)
 * @returns {{ ok: true } | { ok: false, joint: string, value: number, min: number, max: number }}
 */
function jointVectorWithinUrdfLimits(q) {
  if (!kinematics || !q?.length) {
    return { ok: false, joint: "?", value: NaN, min: 0, max: 0 };
  }
  const names = kinematics.getJointNames();
  for (let i = 0; i < names.length; i++) {
    const name = names[i];
    const { min, max, continuous } = getUrdfJointLimitRad(name);
    if (continuous) continue;
    const v = q[i];
    if (!Number.isFinite(v) || v < min || v > max) {
      return { ok: false, joint: name, value: v, min, max };
    }
  }
  return { ok: true };
}

/**
 * Solve IK for an absolute EE pose without committing UI/motion on failure.
 * On limit failure, restores the seed configuration.
 * @returns {{ ok: true, q: number[] } | { ok: false, reason: 'ik' | 'limits' | 'busy', detail?: object }}
 */
function solveTaskPoseIkChecked(pose, ikOptions = {}) {
  if (!kinematics || !pose) return { ok: false, reason: "ik" };
  if (ikBusy) return { ok: false, reason: "busy" };

  ikBusy = true;
  const q0 = kinematics.getCurrentJointVector();
  try {
    const result = kinematics.solveIK(pose, q0, ikOptions);
    if (!result?.success || !result.q) {
      kinematics.setJointVector(q0);
      return { ok: false, reason: "ik" };
    }

    const lim = jointVectorWithinUrdfLimits(result.q);
    if (!lim.ok) {
      kinematics.setJointVector(q0);
      return { ok: false, reason: "limits", detail: lim };
    }

    return { ok: true, q: result.q };
  } finally {
    ikBusy = false;
  }
}

/* -------------------------------------------------------------------------- */
/* Jog / slider mode                                                           */
/* -------------------------------------------------------------------------- */

let sliderControlMode = "incremental";
const jogCmdsJoint = [0, 0, 0, 0, 0, 0];
const jogCmdsCartesian = [0, 0, 0, 0, 0, 0];
const TASK_AXIS_INDEX = { x: 0, y: 1, z: 2, rx: 3, ry: 4, rz: 5 };

function jointNameToSdkIndex(name) {
  const names = jointsUI?.jointNames || [];
  const uiIndex = names.indexOf(name);
  if (uiIndex < 0) return -1;
  return names.length - 1 - uiIndex;
}

/** Preview: always incremental; gateway: read `SLIDER_CONTROL.mode`. */
function effectiveSliderControlMode() {
  return isGatewayActive() ? SLIDER_CONTROL.mode : "incremental";
}

function applySliderControlMode(mode) {
  const next = mode === "jog" ? "jog" : "incremental";
  if (sliderControlMode === next) return;
  sliderControlMode = next;
  jointsUI?.setSliderMode(next);
  taskUI?.setSliderMode(next);
  console.log(`[slider] control mode: ${next}`);
}

function syncSliderControlMode() {
  applySliderControlMode(effectiveSliderControlMode());
}

function jogRequestOptions() {
  return {
    speed: SLIDER_CONTROL.jogSpeed,
    accTime: SLIDER_CONTROL.jogAccTime,
    jogIntervalMs: SLIDER_CONTROL.jogIntervalMs,
  };
}

/* -------------------------------------------------------------------------- */
/* UI Components                                                               */
/* -------------------------------------------------------------------------- */

const jointsUI = new JointsUI(jointContainerEl, jointCountEl, {
  onJointInput: async (name, value) => {
    if (!kinematics || isSyncing) return;

    const commandMap = kinematics.getCurrentJointMap();
    commandMap[name] = value;

    if (isLocalPreviewOnly()) {
      applyLocalJointMap({ ...commandMap });
      return;
    }

    // 实时下发命令到机器人（先发送命令，成功后再更新UI）
    try {
      const jointNames = Object.keys(commandMap);
      const jointValues = Object.values(commandMap);
      console.log(`[onJointInput] ${name}: Sending real-time command with value=${value.toFixed(6)} rad (${(value * 180 / Math.PI).toFixed(2)}°)`);
      
      const result = await sendJointCommand(
        jointNames,
        jointValues,
        rtInterpolationPayload(RT_INTERPOLATION.moveJoint)
      );
      
      if (result.mode === "preview") {
        console.warn(`[onJointInput] ${name}: Preview mode - no gateway configured`);
        applyLocalJointMap({ ...commandMap });
      } else if (result.data && result.data.success) {
        console.log(`[onJointInput] ${name}: ✅ Real-time command succeeded`);
        withSyncGuard(() => {
          const currentMap = kinematics.getCurrentJointMap();
          currentMap[name] = value;
          kinematics.setJointMap(currentMap);
          refreshPoseReadout();
          syncTaskUiFromRobot();
          syncViewerFromRobot();
          noteGhostShowsCommandAheadOfTelemetry();
          syncTeachMirrorFromJointMap(currentMap);
          jointsInputUI.seedFromRadians(currentMap, { force: true });
          if (jointPanelMode === "input") {
            jointsInputUI.updateMatchState(getActualJointMap());
          }
        });
        if (!isHardwareControlActive()) {
          setStatus("Simulation: joint command sent to rt_control.", "ok");
        }
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
    
    if (isLocalPreviewOnly()) {
      applyLocalJointMap({ ...map });
      setStatus("Local preview: joint command applied (no gateway).", "warn");
      return;
    }

    kinematics.setJointMap(map);
    refreshPoseReadout();
    syncTaskUiFromRobot();
    syncViewerFromRobot();
    syncTeachMirrorFromJointMap(map);
    
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
        setStatus(
          isHardwareControlActive()
            ? "Successfully sent joint command."
            : "Simulation: joint command sent to rt_control.",
          "ok"
        );
        console.log(`[onJointCommitted] ${name}: ✅ Command succeeded`);
        noteGhostShowsCommandAheadOfTelemetry();
      } else if (result.data && !result.data.success) {
        setStatus(`Failed to send joint command. ${result.data.message}`, "danger-text");
        console.error(`[onJointCommitted] ${name}: ❌ Command failed: ${result.data.message}`);
      } else {
        console.warn(`[onJointCommitted] ${name}: Unexpected result format:`, result);
      }
    } catch (error) {
      console.error(`[onJointCommitted] ${name}: Exception occurred:`, error);
      setStatus(`Error sending command: ${error.message}`, "danger-text");
    }
  },

  onJogInput: (name, cmd) => {
    if (isLocalPreviewOnly()) return;
    const idx = jointNameToSdkIndex(name);
    if (idx < 0) return;
    jogCmdsJoint[idx] = cmd;
    sendJogJointCommand([...jogCmdsJoint], jogRequestOptions()).catch((err) => {
      console.warn(`[onJogInput] ${name}:`, err);
    });
  },

  onJogRelease: async (name) => {
    if (isLocalPreviewOnly()) return;
    const idx = jointNameToSdkIndex(name);
    if (idx >= 0) jogCmdsJoint[idx] = 0;
    try {
      if (!jogCmdsJoint.some((c) => c !== 0)) {
        await sendJogStopCommand();
      } else {
        await sendJogJointCommand([...jogCmdsJoint], jogRequestOptions());
      }
    } catch (err) {
      console.warn(`[onJogRelease] ${name}:`, err);
    }
  },
}, {
  intervalMs: 100,  // 连续调节的时间间隔（毫秒），可根据需要调整
  stepDeg: 1,       // 每次步进的角度，可根据需要调整
  sliderMode: "incremental",
});

/** Joint sidebar: 'jog' (sliders) | 'input' (absolute + Move Joints). */
let jointPanelMode = "jog";

const jointsInputUI = new JointsInputUI({
  listEl: jointInputListEl,
  moveBtn: moveJointsBtnEl,
  saveBtn: saveInitialPoseBtnEl,
  saveIconEl: saveInitialPoseIconEl,
  keypadEl: jointKeypadEl,
  callbacks: {
    onDraftChange: () => {
      if (jointPanelMode === "input") {
        jointsInputUI.updateMatchState(getActualJointMap());
      }
    },
    onMoveRequest: () => {
      handleMoveJoints();
    },
    onSaveRequest: () => {
      handleSaveInitialPose();
    },
  },
});

function getActualJointMap() {
  if (!kinematics) return {};
  if (latestJointPosition?.length) {
    return vectorToMap(latestJointPosition);
  }
  return kinematics.getCurrentJointMap();
}

function currentRobotStorageName() {
  return getApiState().activeRobotName || DEFAULT_ROBOTS[0].name;
}

function initialPoseStorageKey(robotName = currentRobotStorageName()) {
  return `${STORAGE_KEYS.initialPose}.${robotName}`;
}

function readSavedInitialPose(robotName = currentRobotStorageName()) {
  try {
    const raw = localStorage.getItem(initialPoseStorageKey(robotName));
    if (!raw) return null;
    const data = JSON.parse(raw);
    if (!data || data.version !== 1 || data.unit !== "rad") return null;
    if (!Array.isArray(data.jointNames) || !Array.isArray(data.jointValues)) {
      return null;
    }
    if (data.jointNames.length !== data.jointValues.length) return null;
    if (!data.jointValues.every((v) => Number.isFinite(Number(v)))) return null;
    return data;
  } catch {
    return null;
  }
}

function savedInitialPoseMatchesRobot(data) {
  if (!kinematics || !data) return false;
  const names = kinematics.getJointNames();
  if (names.length !== data.jointNames.length) return false;
  return names.every((name, i) => name === data.jointNames[i]);
}

function formatSavedPoseStatus(jointNames, jointValuesRad) {
  return jointNames
    .map((name, i) => {
      const deg = formatJointInput(jointValuesRad[i], false);
      return `${name}=${deg}°`;
    })
    .join(" ");
}

function refreshInitialPoseButton() {
  if (!initialPoseBtnEl) return;
  const data = readSavedInitialPose();
  const ok = !!(data && savedInitialPoseMatchesRobot(data));
  initialPoseBtnEl.disabled = !ok;
  initialPoseBtnEl.setAttribute("aria-disabled", ok ? "false" : "true");
  initialPoseBtnEl.title = ok
    ? "Go to saved initial position"
    : "Save an initial pose first";
}

function setJointPanelMode(mode) {
  const next = mode === "input" ? "input" : "jog";
  jointPanelMode = next;

  // Only Joint tabs ([data-joint-mode]); Task tabs share .joint-mode-tab class.
  document.querySelectorAll("[data-joint-mode]").forEach((tab) => {
    const active = tab.dataset.jointMode === next;
    tab.classList.toggle("is-active", active);
    tab.setAttribute("aria-selected", active ? "true" : "false");
  });

  if (jointJogPanelEl) {
    jointJogPanelEl.hidden = next !== "jog";
    jointJogPanelEl.classList.toggle("is-active", next === "jog");
  }
  if (jointInputPanelEl) {
    jointInputPanelEl.hidden = next !== "input";
    jointInputPanelEl.classList.toggle("is-active", next === "input");
  }

  if (next === "input") {
    jointsInputUI.seedFromRadians(getActualJointMap(), { force: true });
    jointsInputUI.updateMatchState(getActualJointMap());
  }
}

function syncInputMatchFromActual() {
  const actual = getActualJointMap();
  // Always keep Input fields aligned with live joints (Jog drag / telemetry),
  // unless the user is mid-edit on a Move draft (dirty).
  jointsInputUI.syncFromRadians(actual);
  if (jointPanelMode === "input") {
    jointsInputUI.updateMatchState(actual);
  }
}

function forceSyncInputFromActual() {
  const actual = getActualJointMap();
  jointsInputUI.seedFromRadians(actual, { force: true });
  jointsInputUI.updateMatchState(actual);
}

async function dispatchAbsoluteJointMove(map, { statusOk, statusPreview }) {
  if (!kinematics || !map) return false;

  const jointNames = kinematics.getJointNames();
  const jointValues = jointNames.map((name) => map[name] ?? 0);

  if (isLocalPreviewOnly()) {
    applyJointMap(map, {
      syncJointUi: true,
      syncTaskUi: true,
      syncViewer: true,
    });
    forceSyncInputFromActual();
    forceSyncTaskInputFromActual();
    setStatus(statusPreview, "warn");
    return true;
  }

  try {
    const result = await sendJointCommand(
      jointNames,
      jointValues,
      rtInterpolationPayload(RT_INTERPOLATION.moveJoint)
    );

    if (result.mode === "preview") {
      applyJointMap(map, {
        syncJointUi: true,
        syncTaskUi: true,
        syncViewer: true,
      });
      forceSyncInputFromActual();
      forceSyncTaskInputFromActual();
      setStatus(statusPreview, "warn");
      return true;
    }

    if (result.data && result.data.success) {
      withSyncGuard(() => {
        kinematics.setJointMap(map);
        jointsUI.setValuesByMap(map, true);
        refreshPoseReadout();
        syncTaskUiFromRobot();
        syncViewerFromRobot();
        noteGhostShowsCommandAheadOfTelemetry();
        syncTeachMirrorFromJointMap(map);
      });
      // Keep drafts aligned with the commanded pose; live telemetry continues syncing after.
      jointsInputUI.seedFromRadians(map, { force: true });
      jointsInputUI.updateMatchState(getActualJointMap());
      forceSyncTaskInputFromActual();
      setStatus(
        isHardwareControlActive()
          ? statusOk
          : "Simulation: joint command sent to rt_control.",
        "ok"
      );
      return true;
    }

    setStatus(
      `Failed to move joints. ${result.data?.message || "Unknown error"}`,
      "danger-text"
    );
    return false;
  } catch (error) {
    setStatus(`Error moving joints: ${error.message}`, "danger-text");
    return false;
  }
}

async function handleMoveJoints() {
  if (!kinematics) return;
  const result = jointsInputUI.validateForMove();
  if (!result.ok) {
    setStatus(result.message, "danger-text");
    jointsInputUI.updateMatchState(getActualJointMap());
    return;
  }
  await dispatchAbsoluteJointMove(result.map, {
    statusOk: "Successfully sent joint command.",
    statusPreview: "Local preview: joint angles applied (no gateway).",
  });
}

function handleSaveInitialPose() {
  if (!kinematics) return;
  const actual = getActualJointMap();
  if (!jointsInputUI.updateMatchState(actual)) {
    setStatus(
      "Save Initial Position requires draft angles to match the arm (±0.1°).",
      "warn"
    );
    return;
  }

  const jointNames = kinematics.getJointNames();
  const jointValues = jointNames.map((name) => actual[name] ?? 0);
  const payload = {
    version: 1,
    unit: "rad",
    jointNames,
    jointValues,
    updatedAt: new Date().toISOString(),
  };

  localStorage.setItem(
    initialPoseStorageKey(),
    JSON.stringify(payload)
  );
  refreshInitialPoseButton();
  setStatus(
    `Initial pose saved: ${formatSavedPoseStatus(jointNames, jointValues)}`,
    "ok"
  );
}

async function handleGoToInitialPose() {
  if (!kinematics) return;
  const data = readSavedInitialPose();
  if (!data || !savedInitialPoseMatchesRobot(data)) {
    refreshInitialPoseButton();
    setStatus("No saved initial pose for this robot.", "warn");
    return;
  }

  const map = {};
  data.jointNames.forEach((name, i) => {
    map[name] = Number(data.jointValues[i]);
  });

  viewer.fitToRobot?.();
  await dispatchAbsoluteJointMove(map, {
    statusOk: "Moved to saved initial position.",
    statusPreview: "Local preview: initial pose applied (no gateway).",
  });
}

const kinematicsLab = kinematicsLabContainerEl
  ? new KinematicsLab(kinematicsLabContainerEl, {
      viewer,
      setStatus,
    })
  : null;

const taskUI = new TaskSpaceUI(taskSpaceContainerEl, {
  onReadCurrent: () => {
    if (!kinematics) return;
    const uiPose = poseRadToUiDeg(kinematics.getEndEffectorPose());
    taskUI.setPose(uiPose);
    syncTaskInputFromPose(uiPose, { force: true });
  },

  onMove: async (pose) => {
    if (!kinematics || isSyncing) return;

    const targetPose = poseUiDegToRad(pose);
    const ok = applyTaskPoseByIK(targetPose, {
      syncJointUi: true,
      syncTaskUi: true,
      syncViewer: true,
      setAsLastGoal: true,
    });

    if (!ok) {
      setStatus("IK solve failed for task move.", "danger-text");
      return;
    }

    syncTaskInputFromPose(poseRadToUiDeg(targetPose), { force: true });

    if (isLocalPreviewOnly()) {
      setStatus("Local preview: task-space move applied (no gateway).", "warn");
      return;
    }
    
    // 🔧 选项2：直接发送任务空间命令到后端
    
    try {
      console.log(`[TaskSpace onMove] Sending absolute pose command:`, targetPose);
      const result = await sendPoseCommand(poseToArray(targetPose));
      
      if (result.mode === "preview") {
        setStatus("Preview task-space move applied locally.", "warn");
      } else if (result.data && result.data.success) {
        console.log(`[TaskSpace onMove] ✅ Absolute pose command succeeded`);
        setStatus(
          isHardwareControlActive()
            ? "Task-space absolute command sent."
            : "Simulation: task-space command sent to rt_control.",
          "ok"
        );
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

    const targetPose = poseUiDegToRad(taskUI.getPose());
    const deltaPoseRad = {
      x: deltaPose.x || 0,
      y: deltaPose.y || 0,
      z: deltaPose.z || 0,
      rx: THREE.MathUtils.degToRad(deltaPose.rx || 0),
      ry: THREE.MathUtils.degToRad(deltaPose.ry || 0),
      rz: THREE.MathUtils.degToRad(deltaPose.rz || 0),
    };
    const ok = applyTaskPoseByIK(targetPose, {
      syncJointUi: true,
      syncTaskUi: false,
      syncViewer: true,
      setAsLastGoal: true,
    });

    if (!ok) {
      setStatus("IK solve failed for task jog.", "danger-text");
      return;
    }

    // Jog drag: keep Input fields following the absolute pose (when not dirty).
    syncTaskInputFromPose(taskUI.getPose(), { force: true });

    if (isLocalPreviewOnly()) {
      setStatus("Local preview: task jog applied (no gateway).", "warn");
      return;
    }

    try {
      console.log(`[TaskSpace onMoveIncremental] Sending incremental pose command:`, deltaPoseRad);
      const result = await sendPoseIncrementalCommand(poseToArray(deltaPoseRad));
      
      if (result.mode === "preview") {
        setStatus("Preview task-space jog applied locally.", "warn");
      } else if (result.data && result.data.success) {
        console.log(`[TaskSpace onMoveIncremental] ✅ Incremental pose command succeeded`);
        setStatus(
          isHardwareControlActive()
            ? "Task-space incremental command sent."
            : "Simulation: Task-space incremental command sent to rt_control.",
          "ok"
        );
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
    lastGoalPose = poseUiDegToRad(pose);
    setStatus("Task-space goal snapshot captured.", "ok");
  },

  onPlanPose: () => {
    document.getElementById("planCartesianBtn").click();
  },

  onJogInput: (axisKey, cmd) => {
    if (isLocalPreviewOnly()) return;
    const idx = TASK_AXIS_INDEX[axisKey];
    if (idx === undefined) return;
    jogCmdsCartesian[idx] = cmd;
    sendJogCartesianCommand([...jogCmdsCartesian], jogRequestOptions()).catch((err) => {
      console.warn(`[TaskSpace onJogInput] ${axisKey}:`, err);
    });
  },

  onJogRelease: async (axisKey) => {
    if (isLocalPreviewOnly()) return;
    const idx = TASK_AXIS_INDEX[axisKey];
    if (idx !== undefined) jogCmdsCartesian[idx] = 0;
    try {
      if (!jogCmdsCartesian.some((c) => c !== 0)) {
        await sendJogStopCommand();
      } else {
        await sendJogCartesianCommand([...jogCmdsCartesian], jogRequestOptions());
      }
    } catch (err) {
      console.warn(`[TaskSpace onJogRelease] ${axisKey}:`, err);
    }
  },
}, {
  intervalMs: 100,     // 连续调节的时间间隔（毫秒），可根据需要调整
  stepTrans: 0.01,     // 平移每次步进的米数，可根据需要调整
  stepRot: 1,          // 旋转每次步进的角度，可根据需要调整
  controlMode: 0,      // 默认控制模式：1=绝对位姿, 0=增量位姿
  sliderMode: "incremental",
  setStatus,
});

/** Task sidebar: 'jog' (sliders) | 'input' (absolute Move Pose). */
let taskPanelMode = "jog";

const taskInputUI = new TaskInputUI({
  listEl: taskInputListEl,
  moveBtn: movePoseBtnEl,
  keypadEl: taskKeypadEl,
  callbacks: {
    onMoveRequest: () => {
      handleMovePose();
    },
  },
});
taskInputUI.build();

function getActualTaskPoseUi() {
  if (!kinematics) return null;
  return poseRadToUiDeg(kinematics.getEndEffectorPose());
}

function syncTaskInputFromPose(poseUi, { force = false } = {}) {
  if (!poseUi || !taskInputUI) return;
  if (force) {
    taskInputUI.seedFromPose(poseUi, { force: true });
  } else {
    taskInputUI.syncFromPose(poseUi);
  }
}

function forceSyncTaskInputFromActual() {
  const pose = getActualTaskPoseUi();
  if (pose) syncTaskInputFromPose(pose, { force: true });
}

function setTaskPanelMode(mode) {
  const next = mode === "input" ? "input" : "jog";
  taskPanelMode = next;

  document.querySelectorAll("[data-task-mode]").forEach((tab) => {
    const active = tab.dataset.taskMode === next;
    tab.classList.toggle("is-active", active);
    tab.setAttribute("aria-selected", active ? "true" : "false");
  });

  if (taskJogPanelEl) {
    taskJogPanelEl.hidden = next !== "jog";
    taskJogPanelEl.classList.toggle("is-active", next === "jog");
  }
  if (taskInputPanelEl) {
    taskInputPanelEl.hidden = next !== "input";
    taskInputPanelEl.classList.toggle("is-active", next === "input");
  }

  if (next === "input") {
    forceSyncTaskInputFromActual();
  }
}

async function handleMovePose() {
  if (!kinematics) return;
  const result = taskInputUI.validateForMove();
  if (!result.ok) {
    setStatus(result.message, "danger-text");
    return;
  }

  const targetPose = poseUiDegToRad(result.pose);
  const solved = solveTaskPoseIkChecked(targetPose);

  if (!solved.ok) {
    // Restore display after failed IK / limit check (robot already reset to q0).
    applyJointVector(kinematics.getCurrentJointVector(), {
      syncJointUi: true,
      syncTaskUi: true,
      syncViewer: true,
    });
    // Clear dirty draft so fields match the restored actual pose.
    forceSyncTaskInputFromActual();
    if (solved.reason === "limits") {
      const d = solved.detail;
      const deg = (r) => ((r * 180) / Math.PI).toFixed(1);
      setStatus(
        `IK solution exceeds joint limits (${d.joint}: ${deg(d.value)}° not in [${deg(d.min)}°, ${deg(d.max)}°]).`,
        "danger-text"
      );
    } else if (solved.reason === "busy") {
      setStatus("IK solver busy, try again.", "warn");
    } else {
      setStatus("IK solve failed for task move.", "danger-text");
    }
    return;
  }

  applyJointVector(solved.q, {
    syncJointUi: true,
    syncTaskUi: true,
    syncViewer: true,
  });
  lastGoalPose = targetPose;
  noteGhostShowsCommandAheadOfTelemetry();

  // Keep Input fields on the commanded absolute pose (base frame).
  taskInputUI.seedFromPose(result.pose, { force: true });

  if (isLocalPreviewOnly()) {
    setStatus("Local preview: task-space move applied (no gateway).", "warn");
    return;
  }

  try {
    // Gateway: move_ee_point via /move_pose (only after IK + joint-limit gate).
    const cmdResult = await sendPoseCommand(poseToArray(targetPose));
    if (cmdResult.mode === "preview") {
      setStatus("Preview task-space move applied locally.", "warn");
    } else if (cmdResult.data && cmdResult.data.success) {
      setStatus(
        isHardwareControlActive()
          ? "Task-space move point command sent."
          : "Simulation: task-space move point sent to rt_control.",
        "ok"
      );
    } else if (cmdResult.data && !cmdResult.data.success) {
      setStatus(
        `Failed to send move point command. ${cmdResult.data.message}`,
        "danger-text"
      );
    }
  } catch (error) {
    setStatus(`Error sending move point command: ${error.message}`, "danger-text");
  }
}

taskUI.build();

/* -------------------------------------------------------------------------- */
/* Viewer callbacks                                                            */
/* -------------------------------------------------------------------------- */

viewer.callbacks.onTaskMove = (pose) => {
  if (!kinematics || isSyncing || !shouldShowTaskGizmo()) return;

  const ikFailStatus = "IK solve failed for dragged target.";
  const targetPose = transformPoseToRobotPose(pose);
  if (!targetPose) return;

  taskUI.setPose(poseRadToUiDeg(targetPose));
  syncTaskInputFromPose(poseRadToUiDeg(targetPose), { force: true });

  const ok = applyTaskPoseByIK(targetPose, {
    syncJointUi: true,
    syncTaskUi: pose?.mode !== "rotate",
    syncViewer: false, // viewer 自己已经在这个 pose 上了
    ikOptions: pose?.mode === "rotate"
      ? { maxIterations: 140, positionTolerance: 5e-4, orientationTolerance: 8e-3 }
      : {},
  });

  if (!ok) {
    syncTaskUiFromRobot();
    syncViewerFromRobot();
    setStatus(ikFailStatus, "danger-text");
    return;
  }

  clearStatusIf(ikFailStatus);
};

const gizmoTranslateBtn = document.getElementById("gizmoTranslateBtn");
const gizmoRotateBtn = document.getElementById("gizmoRotateBtn");
const gizmoFrameSelectEl = document.getElementById("gizmoFrameSelect");
const gizmoModeBarEl = document.querySelector(".gizmo-mode-bar");

function getActiveViewerTab() {
  return document.querySelector(".tab-btn.active")?.dataset.page || "joint";
}

function shouldShowTaskGizmo() {
  if (!VIEWER.hideTaskGizmoOnJointTeachInGateway) return true;
  if (!isGatewayActive()) return true;
  return getActiveViewerTab() === "task";
}

function syncTaskGizmoVisibility() {
  const show = shouldShowTaskGizmo();
  viewer.setTaskGizmoVisible(show);
  gizmoModeBarEl?.toggleAttribute("hidden", !show);
}

function syncGizmoModeButtons(mode) {
  gizmoTranslateBtn?.classList.toggle("active", mode === "translate");
  gizmoRotateBtn?.classList.toggle("active", mode === "rotate");
}

function syncGizmoFrameSelect(frame) {
  if (gizmoFrameSelectEl) gizmoFrameSelectEl.value = frame;
}

if (!viewerInitFailure) {
  viewer.callbacks.onTransformModeChange = syncGizmoModeButtons;
  viewer.callbacks.onTransformFrameChange = syncGizmoFrameSelect;
  syncGizmoModeButtons(viewer.getTransformMode());
  syncGizmoFrameSelect(viewer.getTransformFrame());

  gizmoTranslateBtn?.addEventListener("click", () => {
    viewer.setTransformMode("translate");
  });
  gizmoRotateBtn?.addEventListener("click", () => {
    viewer.setTransformMode("rotate");
  });
  gizmoFrameSelectEl?.addEventListener("change", () => {
    viewer.setTransformFrame(gizmoFrameSelectEl.value);
  });

  syncTaskGizmoVisibility();
}

/* -------------------------------------------------------------------------- */
/* Robot loading                                                               */
/* -------------------------------------------------------------------------- */

/** path -> Promise<{ ghost, hardware }> — share in-flight + warm cache for switches */
const robotPairCache = new Map();
/** Paths whose pair has finished preparing (instant switch, no loading status) */
const robotPairReady = new Set();
/** path -> Set<(info) => void> progress listeners for in-flight prepares */
const robotPairProgressListeners = new Map();
/** Ignore superseded async loads when the user switches robots quickly */
let robotLoadGeneration = 0;
let activeRobotPath = null;

function robotNameFromUrdfPath(path) {
  const parts = String(path || "").split("/").filter(Boolean);
  // ./urdf/<name>/robot.urdf
  if (parts.length >= 2 && parts[parts.length - 1] === "robot.urdf") {
    return parts[parts.length - 2];
  }
  return parts[parts.length - 1] || path;
}

function subscribeRobotPairProgress(path, fn) {
  if (!fn) return () => {};
  let set = robotPairProgressListeners.get(path);
  if (!set) {
    set = new Set();
    robotPairProgressListeners.set(path, set);
  }
  set.add(fn);
  return () => {
    set.delete(fn);
    if (!set.size) robotPairProgressListeners.delete(path);
  };
}

function emitRobotPairProgress(path, info) {
  const set = robotPairProgressListeners.get(path);
  if (!set?.size) return;
  for (const fn of set) {
    try {
      fn(info);
    } catch {
      /* ignore */
    }
  }
}

function formatUrdfLoadStatus(robotName, info = {}) {
  const name = robotName || "URDF";
  if (info.phase === "prepare" || info.phase === "start") {
    return info.phase === "prepare"
      ? `Loading ${name}… preparing model`
      : `Loading ${name}…`;
  }
  const loaded = Number(info.loaded) || 0;
  const total = Number(info.total) || 0;
  const downloading = Array.isArray(info.downloading) ? info.downloading : [];
  const current =
    downloading.length > 0
      ? downloading.slice(0, 2).join(", ")
      : info.file || "";
  if (total > 0) {
    const pct = Math.min(100, Math.round((loaded / total) * 100));
    return current
      ? `Loading ${name}… ${loaded}/${total} (${pct}%) — ${current}`
      : `Loading ${name}… ${loaded}/${total} (${pct}%)`;
  }
  if (current) return `Loading ${name}… ${current}`;
  return `Loading ${name}…`;
}

/**
 * Detach the on-screen robot so the dropdown name and 3D model stay consistent
 * while a new URDF loads. Does NOT dispose or drop robotPairCache (keeps speed).
 */
function clearDisplayedRobot() {
  robotGhost = null;
  robotHardware = null;
  activeRobotPath = null;
  kinematics = null;

  viewer.setDualRobot(null, null);
  kinematicsLab?.setRobotContext(null);
  jointsUI.clear();
  // {} clears teach mirror; null would fall back to getRecordJointMap().
  teach?.syncTeachJointMirror({});

  if (baseLinkEl) baseLinkEl.textContent = "—";
  if (tipLinkEl) tipLinkEl.textContent = "—";
}

/**
 * Load URDF once, clone for ghost/hardware, cache the prepared pair.
 * Subsequent selects for the same path reuse the cache (no network).
 */
function prepareRobotPair(path) {
  let entry = robotPairCache.get(path);
  if (entry) return entry;

  entry = (async () => {
    const t0 = performance.now();
    const mark = (label) =>
      console.log(`[URDF][pair] ${label} +${(performance.now() - t0).toFixed(1)}ms`);

    console.log(`[URDF][pair] prepare start: ${path}`);
    const base = await loadRobotFromUrdf(path, {
      onProgress: (info) => emitRobotPairProgress(path, info),
    });
    mark("loadRobotFromUrdf done");
    emitRobotPairProgress(path, { phase: "prepare" });

    const hardware = base;
    const cloneT0 = performance.now();
    const ghost = typeof base.clone === "function" ? base.clone(true) : base;
    console.log(
      `[URDF][pair] clone(true) ${(performance.now() - cloneT0).toFixed(1)}ms` +
        (ghost === hardware ? " (NO clone — fallback path)" : ""),
    );

    if (ghost === hardware) {
      // Fallback: loader without clone — load a second copy (slower).
      const second = await loadRobotFromUrdf(path, {
        onProgress: (info) => emitRobotPairProgress(path, info),
      });
      mark("second loadRobotFromUrdf done");
      emitRobotPairProgress(path, { phase: "prepare" });
      const styleT0 = performance.now();
      cloneMaterialsPerMesh(hardware);
      cloneMaterialsPerMesh(second);
      applyHardwareContrastStyle(hardware);
      applyGhostVisualStyle(second);
      console.log(
        `[URDF][pair] materials/style ${(performance.now() - styleT0).toFixed(1)}ms`,
      );
      robotPairReady.add(path);
      mark("prepare DONE (fallback)");
      return { ghost: second, hardware };
    }

    const styleT0 = performance.now();
    cloneMaterialsPerMesh(hardware);
    cloneMaterialsPerMesh(ghost);
    applyHardwareContrastStyle(hardware);
    applyGhostVisualStyle(ghost);
    console.log(
      `[URDF][pair] materials/style ${(performance.now() - styleT0).toFixed(1)}ms`,
    );
    robotPairReady.add(path);
    mark("prepare DONE");
    return { ghost, hardware };
  })();

  robotPairCache.set(path, entry);
  entry.catch(() => {
    // Allow retry on next select if this preparation failed.
    if (robotPairCache.get(path) === entry) robotPairCache.delete(path);
    robotPairReady.delete(path);
  });
  return entry;
}

function shouldPreloadBackgroundRobots() {
  return false;
}

function preloadBackgroundRobots(exceptName) {
  if (!shouldPreloadBackgroundRobots()) return;

  for (const item of DEFAULT_ROBOTS) {
    if (item.name === exceptName) continue;
    const path = getUrdfPathForRobot(item);
    prepareRobotPair(path).catch((error) => {
      console.warn(`[URDF] background preload failed for ${item.name}:`, error);
    });
  }
}

async function loadCurrentRobot(path) {
  const generation = ++robotLoadGeneration;
  const t0 = performance.now();
  const cached = robotPairReady.has(path);
  const robotName = robotNameFromUrdfPath(path);
  console.log(
    `[URDF][ui] loadCurrentRobot gen=${generation} path=${path} cached=${cached}`,
  );

  // Drop the previous on-screen model as soon as the dropdown changes so the
  // viewer never disagrees with the selected name. Cache is kept for speed.
  if (activeRobotPath !== path) {
    clearDisplayedRobot();
  }
  if (!cached) {
    setStatus(formatUrdfLoadStatus(robotName, { phase: "start" }), "warn");
  }

  const unsubProgress = subscribeRobotPairProgress(path, (info) => {
    if (generation !== robotLoadGeneration) return;
    if (robotPairReady.has(path)) return;
    setStatus(formatUrdfLoadStatus(robotName, info), "warn");
  });

  try {
    const { ghost, hardware } = await prepareRobotPair(path);
    console.log(
      `[URDF][ui] prepareRobotPair resolved +${(performance.now() - t0).toFixed(1)}ms gen=${generation}`,
    );
    if (generation !== robotLoadGeneration) {
      console.warn(
        `[URDF][ui] superseded after prepare (gen=${generation}, current=${robotLoadGeneration})`,
      );
      return false;
    }

    const mountT0 = performance.now();
    robotGhost = ghost;
    robotHardware = hardware;
    activeRobotPath = path;

    viewer.setDualRobot(robotHardware, robotGhost);
    viewer.setHardwareRobotVisible(false);
    applyGhostRobotVisibility(true);

    kinematics = new RobotKinematics(robotGhost);
    kinematicsLab?.setRobotContext(kinematics);

    jointsUI.build(robotGhost);
    jointsInputUI.build(robotGhost);
    if (jointPanelMode === "input") {
      jointsInputUI.seedFromRadians(getActualJointMap(), { force: true });
      jointsInputUI.updateMatchState(getActualJointMap());
    }
    refreshInitialPoseButton();
    syncTeachMirrorFromJointMap(kinematics.getCurrentJointMap());

    syncMeta();

    console.log(
      `[URDF][ui] mount/UI ${(performance.now() - mountT0).toFixed(1)}ms; total +${(performance.now() - t0).toFixed(1)}ms`,
    );
    return true;
  } catch (error) {
    if (generation !== robotLoadGeneration) return false;
    console.error(
      `[URDF][ui] FAILED +${(performance.now() - t0).toFixed(1)}ms`,
      error,
    );
    setStatus(`Failed to load URDF: ${error.message || error}`, "danger-text");
    return false;
  } finally {
    unsubProgress();
  }
}

/* -------------------------------------------------------------------------- */
/* Robot selector / gateway                                                    */
/* -------------------------------------------------------------------------- */

function populateRobotSelector() {
  robotSelectEl.innerHTML = "";

  DEFAULT_ROBOTS.forEach((item) => {
    const option = document.createElement("option");
    option.value = item.name;
    option.textContent = item.name;
    robotSelectEl.appendChild(option);
  });
}

async function selectRobotAndLoadUrdf(robotName, { announce = true } = {}) {
  const robot = findRobotByName(robotName);
  const path = getUrdfPathForRobot(robot);

  setActiveRobot(robot.name);
  robotSelectEl.value = robot.name;
  localStorage.setItem(STORAGE_KEYS.robotId, robot.name);
  syncGripperCardForRobot(robot.name);

  if (activeRobotPath === path && robotGhost && robotHardware) {
    refreshInitialPoseButton();
    if (announce) setStatus(`Active robot: ${robot.name}.`, "ok");
    return true;
  }

  const loaded = await loadCurrentRobot(path);
  if (!loaded && robot.name !== DEFAULT_ROBOTS[0].name) {
    return selectRobotAndLoadUrdf(DEFAULT_ROBOTS[0].name, { announce });
  }

  if (loaded) {
    syncGripperCardForRobot(robot.name, robotGhost);
    if (robot.hasGripper) {
      applyGripperUiToUrdf(Number(gripperSliderEl?.value ?? GRIPPER.default));
    }
    preloadBackgroundRobots(robot.name);
    // Always clear the in-progress "Loading…" line; announce only affects wording.
    setStatus(
      announce ? `URDF loaded: ${robot.name}.` : "Ready.",
      announce ? "ok" : ""
    );
  }
  return loaded;
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
  syncSliderControlMode();
  syncTaskGizmoVisibility();
}

async function connectSelectedRobot() {
  try {
    const result = await connectRobot();

    updateConnectionUi(result.mode === "preview" ? "warn" : "connect");

    if (result.mode === "preview") {
      setStatus("This is currently in preview mode. Please configure the gateway and try again.", "warn");
    }else if (result.data.success) {
      setStatus("Successfully connected to robot.", "ok");
      ghostCommandPending = false;
      if (latestJointPosition?.length) {
        syncCommandStateFromTelemetry(latestJointPosition);
        applyGhostRobotVisibility(false);
      }
    }else if (!result.data.success) {
      setStatus(`Failed to connect to robot. ${result.data.message}`, "danger-text");
    }

    syncTaskGizmoVisibility();

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
        await sleep(getPlayDelay());
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

// Teach recording uses live joint_pos from /stream
function getTeachRecordJointMap() {
  if (!kinematics) return null;
  if (latestJointPosition?.length) {
    return vectorToMap(latestJointPosition);
  }
  return kinematics.getCurrentJointMap();
}

teach = createTeachModule({
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
  getRecordJointMap: getTeachRecordJointMap,
  setStatus,
  executeTrajectory,
  isHardwareControlActive,
  waitUntilTargetReached,
  isBusy: () => isBusy,
  onSetBusy: () => {
    isBusy = true;
  },
  onClearBusy: () => {
    isBusy = false;
  },
});
const teachSystem = teach.system;

/* -------------------------------------------------------------------------- */
/* Buttons                                                                     */
/* -------------------------------------------------------------------------- */

function bindButtons() {
  document.querySelectorAll("[data-joint-mode]").forEach((tab) => {
    tab.addEventListener("click", () => {
      setJointPanelMode(tab.dataset.jointMode);
    });
  });

  document.querySelectorAll("[data-task-mode]").forEach((tab) => {
    tab.addEventListener("click", () => {
      setTaskPanelMode(tab.dataset.taskMode);
    });
  });

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
    forceSyncInputFromActual();
    forceSyncTaskInputFromActual();

    if (isLocalPreviewOnly()) {
      setStatus("Local preview: home pose applied (no gateway).", "warn");
      return;
    }

    const result = await sendHomeCommand(
      jointNames,
      zeroQ,
      rtInterpolationPayload(RT_INTERPOLATION.home)
    );
    if (result.mode === "preview") {
      setStatus("Preview mode active. No gateway configured.", "warn");
    }else if (result.data.success) {
      setStatus(
        isHardwareControlActive()
          ? "Successfully moved to home position."
          : "Simulation: home command sent to rt_control.",
        "ok"
      );
    }else if (!result.data.success) {
      setStatus(`Failed to move to home position. ${result.data.message}`, "danger-text");
    }
  };

  if (initialPoseBtnEl) {
    initialPoseBtnEl.onclick = () => {
      handleGoToInitialPose();
    };
  }

  document.getElementById("saveGatewayBtn").onclick = saveGateway;

  document.getElementById("connectBtn").onclick = connectSelectedRobot;

  document.getElementById("disconnectBtn").onclick = async () => {
    try {
      const result = await disconnectRobot();

      updateConnectionUi(result.mode === "preview" ? "warn" : "disconnect");

      if (result.mode === "preview") {
        setStatus("Preview mode active. No gateway configured.", "warn");
      }else if (result.data.success) {
        setStatus("Robot disconnected — rt_control simulation active.", "ok");
        ghostCommandPending = false;
        if (latestJointPosition?.length) {
          syncCommandStateFromTelemetry(latestJointPosition);
        }
      }else if (!result.data.success) {
        setStatus(`Failed to disconnect from hardware. ${result.data.message}`, "danger-text");
      }

      syncTaskGizmoVisibility();
      
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

  document.getElementById("copyEnvInfoBtn").onclick = async () => {
    try {
      await copyEnvInfoToClipboard();
      setStatus("Environment info copied to clipboard.", "ok");
    } catch (error) {
      setStatus(error.message || "Copy env info failed.", "danger-text");
    }
  };

  teach.bindTeachButtons();
  initFullscreen();

  // Robot Manager floating panel toggle
  const mgrBtn = document.getElementById("openRobotManagerBtn");
  const mgrPanel = document.getElementById("robotManagerFloating");
  if (mgrBtn && mgrPanel) {
    const positionMgrPanel = () => {
      const btnRect = mgrBtn.getBoundingClientRect();
      const margin = 10;
      const gap = 8;
      const viewportWidth = window.innerWidth;
      const viewportHeight = window.innerHeight;
      const panelWidth = Math.min(380, viewportWidth - margin * 2);

      // Always open below the Manager button
      let topPosition = btnRect.bottom + gap;
      let leftPosition = btnRect.left;

      if (leftPosition + panelWidth > viewportWidth - margin) {
        leftPosition = viewportWidth - panelWidth - margin;
      }
      if (leftPosition < margin) {
        leftPosition = margin;
      }

      // Keep panel in view by scrolling internally instead of flipping above the button
      const availableBelow = Math.max(160, viewportHeight - topPosition - margin);
      mgrPanel.style.width = panelWidth + "px";
      mgrPanel.style.maxHeight = availableBelow + "px";
      mgrPanel.style.left = leftPosition + "px";
      mgrPanel.style.top = topPosition + "px";
      mgrPanel.style.right = "auto";
      mgrPanel.style.bottom = "auto";
    };

    mgrBtn.onclick = (e) => {
      e.stopPropagation();
      const isActive = mgrPanel.classList.toggle("active");
      mgrPanel.setAttribute("aria-hidden", isActive ? "false" : "true");
      if (isActive) positionMgrPanel();
    };

    window.addEventListener("resize", () => {
      if (mgrPanel.classList.contains("active")) positionMgrPanel();
    });

    document.addEventListener("click", (ev) => {
      if (!mgrPanel.classList.contains("active")) return;
      if (mgrPanel.contains(ev.target) || mgrBtn.contains(ev.target)) return;
      mgrPanel.classList.remove("active");
      mgrPanel.setAttribute("aria-hidden", "true");
    });
  }

  robotSelectEl.onchange = async () => {
    updateConnectionUi(gatewayUrlEl.value.trim() ? "ready" : "warn");
    await selectRobotAndLoadUrdf(robotSelectEl.value);
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
      teachTabActive = teachActive;
      viewerPanelEl?.classList.toggle("teach-active", teachActive);
      if (teachActive) {
        // Match Joint Space: prefer current command/kinematics pose over stale telemetry.
        syncTeachMirrorFromJointMap(kinematics?.getCurrentJointMap() ?? null);
        applyGhostRobotVisibility(true);
      } else if (latestJointPosition?.length) {
        refreshGhostVersusTelemetry(latestJointPosition);
      } else {
        applyGhostRobotVisibility(true);
      }
    };

    const initialActiveTab = document.querySelector(".tab-btn.active");
    syncTeachVisibility(initialActiveTab?.dataset.page || "");
    syncTaskGizmoVisibility();

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
        syncTaskGizmoVisibility();
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
    applyGhostRobotVisibility(true);
    streamEulerStabilizer.reset();
    return;
  }

  streamEulerStabilizer.reset();

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

      if (data.connected === false) {
        updateConnectionUi("danger");
        if (data.error && data.error !== lastStreamError) {
          lastStreamError = data.error;
          setStatus(`Robot stream disconnected: ${data.error}`, "danger-text");
        }
      } else if (data.connected === true) {
        if (lastStreamError) {
          lastStreamError = "";
          setStatus("Robot stream connected.", "ok");
        }
        if (isHardwareControlActive() && data.hardware_connected) {
          updateConnectionUi("connect");
        } else if (!isHardwareControlActive()) {
          updateConnectionUi("ready");
        }
      }

      if (eePose?.length >= 3) {
        updateEePoseCard(eePose.slice(0, 3));
        
        // eePose: [x, y, z, qw, qx, qy, qz] — quaternion like Qt state panel (not raw Euler)
        if (eePose.length >= 6 && kinematics) {
          const pose = streamEulerStabilizer.taskPoseDegFromStream(eePose);
          taskUI.syncFromStreamData(pose);
          syncTaskInputFromPose(pose, { force: false });
        }
      }

      if (jointPosition?.length) {
        latestJointPosition = jointPosition.slice();
        if (kinematics && robotHardware) {
          applyJointVectorToUrdfRobot(robotHardware, jointPosition);
          viewer.setHardwareRobotVisible(true);
        }
        refreshGhostVersusTelemetry(jointPosition);
        syncInputMatchFromActual();
      }

      if (data.gripper_pos !== undefined && data.gripper_pos !== null) {
        syncGripperFromStream(data.gripper_pos);
      }
    } catch (error) {
      console.error("Failed to parse stream payload:", error, event.data);
    }
  };

  robotStream.onerror = (err) => {
    console.error("Stream failed:", err);
    updateConnectionUi("danger");
    if (lastStreamError !== "stream-transport") {
      lastStreamError = "stream-transport";
      setStatus("Robot stream disconnected. Reconnecting…", "danger-text");
    }

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
  
  if (eePoseXEl) eePoseXEl.textContent = formatEePoseValue(position[0]);
  if (eePoseYEl) eePoseYEl.textContent = formatEePoseValue(position[1]);
  if (eePoseZEl) eePoseZEl.textContent = formatEePoseValue(position[2]);
}

/* -------------------------------------------------------------------------- */
/* Gripper card                                                                */
/* UI 0 = open (joint upper), UI 1 = close (joint lower); SDK/URDF use meters. */
/* -------------------------------------------------------------------------- */

function formatGripperUiValue(ui) {
  const n = Number(ui);
  if (!Number.isFinite(n)) return "0.00";
  return n.toFixed(2);
}

function getGripperLimitsFromUrdf(urdfRobot) {
  const joint = urdfRobot?.joints?.[GRIPPER.jointName];
  const lower = Number(joint?.limit?.lower);
  const upper = Number(joint?.limit?.upper);
  if (Number.isFinite(lower) && Number.isFinite(upper) && lower < upper) {
    return { min: lower, max: upper };
  }
  return { min: GRIPPER.min, max: GRIPPER.max };
}

/** Open / close joint positions from current URDF (or fallback) limits. */
function getGripperOpenCloseJoint() {
  return {
    open: gripperLimits.max,
    close: gripperLimits.min,
  };
}

function clampGripperUi(ui) {
  const n = Number(ui);
  if (!Number.isFinite(n)) return GRIPPER.default;
  return Math.min(GRIPPER.uiMax, Math.max(GRIPPER.uiMin, n));
}

function clampGripperJoint(pos) {
  const n = Number(pos);
  if (!Number.isFinite(n)) return gripperLimits.max;
  return Math.min(gripperLimits.max, Math.max(gripperLimits.min, n));
}

/** Map UI [0,1] → joint meters (0=open/upper, 1=close/lower). */
function gripperUiToJoint(ui) {
  const u = clampGripperUi(ui);
  const { open, close } = getGripperOpenCloseJoint();
  return open + u * (close - open);
}

/** Map joint meters → UI [0,1]. */
function gripperJointToUi(pos) {
  const p = clampGripperJoint(pos);
  const { open, close } = getGripperOpenCloseJoint();
  const span = close - open;
  if (!Number.isFinite(span) || Math.abs(span) < 1e-12) return GRIPPER.default;
  return clampGripperUi((p - open) / span);
}

function applyGripperLimitsToSlider() {
  if (!gripperSliderEl) return;
  gripperSliderEl.min = String(GRIPPER.uiMin);
  gripperSliderEl.max = String(GRIPPER.uiMax);
  gripperSliderEl.step = String(GRIPPER.step);
}

function applyGripperJointToUrdf(jointPos) {
  const pos = clampGripperJoint(jointPos);
  for (const urdfRobot of [robotGhost, robotHardware]) {
    const joint = urdfRobot?.joints?.[GRIPPER.jointName];
    if (!joint) continue;
    if (typeof joint.setJointValue === "function") joint.setJointValue(pos);
    else joint.angle = pos;
    urdfRobot.updateMatrixWorld?.(true);
  }
  return pos;
}

function applyGripperUiToUrdf(ui) {
  return applyGripperJointToUrdf(gripperUiToJoint(ui));
}

function updateGripperUi(ui, { updateSlider = true } = {}) {
  const u = clampGripperUi(ui);
  if (gripperValueEl) gripperValueEl.textContent = formatGripperUiValue(u);
  if (updateSlider && gripperSliderEl && !gripperInteracting) {
    gripperSliderEl.value = String(u);
  }
  return u;
}

function setGripperCardVisible(visible) {
  if (!gripperCardEl) return;
  gripperCardEl.hidden = !visible;
  if (visible && gripperSliderEl) {
    applyGripperLimitsToSlider();
    const current = Number(gripperSliderEl.value);
    const ui = Number.isFinite(current) ? current : GRIPPER.default;
    updateGripperUi(ui);
    applyGripperUiToUrdf(ui);
  }
}

function syncGripperCardForRobot(robotName, urdfRobot = robotGhost) {
  const robot = findRobotByName(robotName);
  if (robot.hasGripper && urdfRobot) {
    gripperLimits = getGripperLimitsFromUrdf(urdfRobot);
  } else {
    gripperLimits = { min: GRIPPER.min, max: GRIPPER.max };
  }
  setGripperCardVisible(!!robot.hasGripper);
}

async function sendGripperUi(ui) {
  const u = clampGripperUi(ui);
  const jointPos = gripperUiToJoint(u);

  if (isLocalPreviewOnly()) {
    applyGripperJointToUrdf(jointPos);
    updateGripperUi(u);
    setStatus("Local preview: gripper move applied (no gateway).", "warn");
    return;
  }

  try {
    const result = await sendGripperCommand(jointPos, GRIPPER.speed, GRIPPER.accTime);
    if (result.mode === "preview") {
      applyGripperJointToUrdf(jointPos);
      updateGripperUi(u);
      setStatus("Preview gripper move applied locally.", "warn");
      return;
    }
    if (result.data && result.data.success === false) {
      setStatus(`Failed to move gripper: ${result.data.message}`, "danger-text");
      return;
    }
    applyGripperJointToUrdf(jointPos);
    updateGripperUi(u, { updateSlider: false });
    if (!isHardwareControlActive()) {
      setStatus("Simulation: gripper command sent to rt_control.", "ok");
    }
  } catch (error) {
    console.error("[gripper] Error sending command:", error);
    setStatus(`Error moving gripper: ${error.message}`, "danger-text");
  }
}

function scheduleGripperSend(ui) {
  if (gripperSendTimer) clearTimeout(gripperSendTimer);
  gripperSendTimer = setTimeout(() => {
    gripperSendTimer = null;
    sendGripperUi(ui);
  }, 80);
}

function bindGripperControls() {
  if (gripperBound || !gripperSliderEl) return;
  gripperBound = true;

  applyGripperLimitsToSlider();
  gripperSliderEl.value = String(GRIPPER.default);
  updateGripperUi(GRIPPER.default);
  applyGripperUiToUrdf(GRIPPER.default);

  const onStart = () => {
    gripperInteracting = true;
  };
  const onEnd = () => {
    gripperInteracting = false;
    const ui = clampGripperUi(gripperSliderEl.value);
    applyGripperUiToUrdf(ui);
    updateGripperUi(ui, { updateSlider: false });
    if (gripperSendTimer) {
      clearTimeout(gripperSendTimer);
      gripperSendTimer = null;
    }
    sendGripperUi(ui);
  };

  gripperSliderEl.addEventListener("pointerdown", onStart);
  gripperSliderEl.addEventListener("pointerup", onEnd);
  gripperSliderEl.addEventListener("pointercancel", onEnd);
  gripperSliderEl.addEventListener("input", () => {
    const ui = clampGripperUi(gripperSliderEl.value);
    applyGripperUiToUrdf(ui);
    updateGripperUi(ui, { updateSlider: false });
    scheduleGripperSend(ui);
  });
}

function syncGripperFromStream(gripperPos) {
  if (gripperCardEl?.hidden || gripperInteracting) return;
  if (!Number.isFinite(Number(gripperPos))) return;
  const jointPos = clampGripperJoint(gripperPos);
  const ui = gripperJointToUi(jointPos);
  applyGripperJointToUrdf(jointPos);
  updateGripperUi(ui);
}

/* -------------------------------------------------------------------------- */
/* Boot                                                                        */
/* -------------------------------------------------------------------------- */

(async function boot() {
  populateRobotSelector();

  const savedGateway = localStorage.getItem(STORAGE_KEYS.gatewayUrl) || "";
  const storedRobotKey = localStorage.getItem(STORAGE_KEYS.robotId);
  // Prefer robot `name`; migrate legacy stored gateway ids (spark / spark2).
  const savedRobotName = findRobotByName(
    storedRobotKey === "spark" || storedRobotKey === "spark2"
      ? DEFAULT_ROBOTS[0].name
      : (storedRobotKey || DEFAULT_ROBOTS[0].name)
  ).name;

  gatewayUrlEl.value = savedGateway;
  setGatewayUrl(savedGateway);
  syncSliderControlMode();

  updateConnectionUi(savedGateway ? "ready" : "warn");

  bindButtons();
  bindCardTabs();
  bindGripperControls();
  teach.refreshTeachControls();
  setJointPanelMode("jog");
  setTaskPanelMode("jog");
  refreshInitialPoseButton();

  connectStream(savedGateway);

  await selectRobotAndLoadUrdf(savedRobotName, { announce: false });
})();
