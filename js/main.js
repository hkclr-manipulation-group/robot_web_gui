import {
  DEFAULT_ROBOTS,
  DEFAULT_URDF_PATH,
  PATH_DEFAULTS,
  STORAGE_KEYS,
} from "./config.js";

import {
  connectRobot,
  disconnectRobot,
  getApiState,
  pingGateway,
  sendHomeCommand,
  sendJointCommand,
  sendStopCommand,
  sendZeroCommand,
  setActiveRobot,
  setGatewayUrl,
  enableTeachModeApi,
} from "./api.js";

import { JointsUI } from "./joints-ui.js";
import { KinematicsLab } from "./kinematics-lab.js";
import { RobotKinematics } from "./kinematics.js";
import { planCartesianTrajectory, planJointTrajectory } from "./planner.js";
import { saveTrajectoryToFile, loadTrajectoryFromFile } from "./storage.js";
import { TaskSpaceUI } from "./taskspace-ui.js";
import { TeachSystem } from "./teach.js";
import { loadRobotFromUrdf } from "./urdf-loader-wrapper.js";
import { formatPoseText, readFileAsText, sleep, quaternionToPose } from "./utils.js";
import { RobotViewer } from "./viewer.js";

/* -------------------------------------------------------------------------- */
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

let robot = null;
let kinematics = null;

let isBusy = false;
let isSyncing = false; // 防止 UI/FK/IK 相互触发造成循环
let ikBusy = false;    // 防止拖动时重复进入 IK

let lastGoalMap = null;
let lastGoalPose = null;

const RETRY_DELAY = 3000;
let robotStream = null; // EventSource for streaming robot data from gateway

const teachSystem = new TeachSystem();

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
  return Math.max(2, parseInt(planStepsEl.value || PATH_DEFAULTS.steps, 10));
}

function getPlayDelay() {
  return Math.max(10, parseInt(playDelayEl.value || PATH_DEFAULTS.delayMs, 10));
}

function hasRobot() {
  return !!kinematics;
}

function vectorToMap(q) {
  if (!kinematics) return {};
  return kinematics.getJointNames().reduce((acc, name, idx) => {
    acc[name] = q[idx];
    return acc;
  }, {});
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
  // if (!kinematics) return;
  // const pose = kinematics.getEndEffectorPose();
  // taskUI.setPose(pose);
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
  // taskUI.setPose(pose);

}

function syncAllFromRobot() {
  if (!kinematics) return;

  refreshPoseReadout();
  syncTaskUiFromRobot();
  syncViewerFromRobot();
}

function updateTeachUi() {
  teachCountEl.textContent = `${teachSystem.count} poses`;
  pathPreviewEl.textContent = JSON.stringify(teachSystem.getPath(), null, 2);
  localStorage.setItem(
    STORAGE_KEYS.lastTrajectory,
    JSON.stringify(teachSystem.getPath())
  );
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
  onJointInput: (name, value) => {
    if (!kinematics || isSyncing) return;

    withSyncGuard(() => {
      // 更新kinematics中的关节值以反映用户的目标值
      const currentMap = kinematics.getCurrentJointMap();
      currentMap[name] = value;
      kinematics.setJointMap(currentMap);
      robot?.updateMatrixWorld(true);
      refreshPoseReadout();
      syncTaskUiFromRobot();
      syncViewerFromRobot();
    });
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
      const result = await sendJointCommand(Object.keys(map), Object.values(map));
      console.log(`[onJointCommitted] ${name}: Command result:`, result);
      
      if (result.mode === "preview") {
        setStatus("Preview mode active. No gateway configured.", "warn");
      } else if (result.data && result.data.success) {
        setStatus("Successfully sent joint command.", "ok");
        console.log(`[onJointCommitted] ${name}: ✅ Command succeeded`);
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
});

const kinematicsLab = new KinematicsLab(kinematicsLabContainerEl, {
  viewer,
  setStatus,
});

// const taskUI = new TaskSpaceUI(taskSpaceContainerEl, {
//   onReadCurrent: () => {
//     if (!kinematics) return;
//     taskUI.setPose(kinematics.getEndEffectorPose());
//   },

//   onMove: (pose) => {
//     if (!kinematics || isSyncing) return;

//     const ok = applyTaskPoseByIK(pose, {
//       syncJointUi: true,
//       syncTaskUi: true,
//       syncViewer: true,
//     });

//     if (!ok) {
//       setStatus("IK solve failed for task move.", "warn");
//     }
//   },

//   onSetGoal: (pose) => {
//     lastGoalPose = pose;
//     setStatus("Task-space goal snapshot captured.", "ok");
//   },

//   onPlanPose: () => {
//     document.getElementById("planCartesianBtn").click();
//   },
// });

// taskUI.build();

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

    robot = await loadRobotFromUrdf(path);
    viewer.setRobot(robot);
    // robot.updateMatrixWorld(true);
    kinematics = new RobotKinematics(robot);
    kinematicsLab.setRobotContext(kinematics);

    jointsUI.build(robot);

    syncMeta();

    localStorage.setItem(STORAGE_KEYS.lastUrdfPath, path);

    setStatus("URDF loaded.", "ok");
  } catch (error) {
    console.error(error);
    setStatus(`Failed to load URDF: ${error.message || error}`, "danger-text");
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
  setStatus(`Playing ${trajectory.length} waypoints...`, "warn");

  for (let i = 0; i < trajectory.length; i++) {
    if (!isBusy) break;

    const map = trajectory[i];

    applyJointMap(map, {
      syncJointUi: true,
      syncTaskUi: true,
      syncViewer: true,
    });

    await sendJointCommand(Object.keys(map), Object.values(map));
    await sleep(getPlayDelay());
  }

  isBusy = false;
  setStatus("Path playback completed.", "ok");
}

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

  document.getElementById("captureJointGoalBtn").onclick = () => {
    if (!kinematics) return;

    lastGoalMap = kinematics.getCurrentJointMap();
    setStatus("Joint-space goal snapshot captured.", "ok");
  };

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

    applyJointVector(zeroQ, {
      syncJointUi: true,
      syncTaskUi: true,
      syncViewer: true,
    });

    const result = await sendHomeCommand(Object.keys(zeroQ), Object.values(zeroQ));
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
  //   updateTeachUi();

  //   setStatus("Current pose recorded.", "ok");
  // };

  // document.getElementById("clearPathBtn").onclick = () => {
  //   teachSystem.clear();
  //   updateTeachUi();
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
  //   updateTeachUi();

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
  //   updateTeachUi();

  //   setStatus("Cartesian trajectory generated via IK.", "ok");
  // };

  // document.getElementById("pathFile").onchange = async (event) => {
  //   const file = event.target.files?.[0];
  //   if (!file) return;

  //   const trajectory = await loadTrajectoryFromFile(file);

  //   teachSystem.replaceAll(trajectory);
  //   updateTeachUi();

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

  document.getElementById("startTeachBtn").onclick = () => enableTeachMode(true);
  document.getElementById("stopTeachBtn").onclick = () => enableTeachMode(false);

  async function enableTeachMode(enable) {
    try {
      const result = await enableTeachModeApi(enable);

      if (result.mode === "preview") {
        setStatus("Preview mode active. No gateway configured.", "warn");
      }else if (enable && result.data.success) {
        setStatus("Teach mode enabled.", "ok");
      }else if (enable && !result.data.success) {
        setStatus(`Failed to enable teach mode. ${result.data.message}`, "danger-text");
      }else if (!enable && result.data.success) {
        setStatus("Teach mode disabled.", "ok");
      }else if (!enable && !result.data.success) {
        setStatus(`Failed to disable teach mode. ${result.data.message}`, "danger-text");
      }

    } catch (error) {
      updateConnectionUi("danger");
      setStatus(error.message || "Connect failed.", "danger-text");
    }
  }

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

    if (!tabs.length) return;

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
  if (robotStream) {
      robotStream.close();
  }

  robotStream = new EventSource(url + "/stream");
  
  robotStream.onmessage = (event) => {
      const data = JSON.parse(event.data);
      const position = data.ee_pose[0].slice(0, 3);
      const quaternion = data.ee_pose[0].slice(3, 7);
      const joint_position = data.joint_pos[0];
      
      // Update EE Pose Card
      updateEePoseCard(position);
      
      // syncViewerFromStreamData(position, quaternion);
      jointsUI.syncFromStreamData(joint_position);
  };

  robotStream.onerror = (err) => {
      console.error("Stream failed:", err);
      robotStream.close();
      setTimeout(connectStream(url), RETRY_DELAY); //Retry
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

(function boot() {
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

  // if (savedTraj) {
  //   try {
  //     teachSystem.replaceAll(JSON.parse(savedTraj));
  //     updateTeachUi();
  //   } catch {
  //     teachSystem.clear();
  //     updateTeachUi();
  //   }
  // } else {
  //   updateTeachUi();
  // }

  connectStream(savedGateway);

  loadCurrentRobot(savedUrdf);
})();