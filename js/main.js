import { DEFAULT_URDF_PATH, PATH_DEFAULTS, STORAGE_KEYS } from "./config.js";
import {
  getCurrentPose,
  sendHomeCommand,
  sendJointCommand,
  sendPoseCommand,
  sendStopCommand,
  sendZeroCommand,
} from "./api.js";
import { JointsUI } from "./joints-ui.js";
import { loadRobotFromUrdf } from "./urdf-loader-wrapper.js";
import { RobotViewer } from "./viewer.js";

const viewerEl = document.getElementById("viewer");
const statusEl = document.getElementById("status");
const urdfPathEl = document.getElementById("urdfPath");
const jointContainerEl = document.getElementById("jointContainer");
const jointCountEl = document.getElementById("jointCount");

const loadBtn = document.getElementById("loadBtn");
const resetViewBtn = document.getElementById("resetViewBtn");
const zeroBtn = document.getElementById("zeroBtn");
const homeBtn = document.getElementById("homeBtn");
const stopBtn = document.getElementById("stopBtn");

const txEl = document.getElementById("tx");
const tyEl = document.getElementById("ty");
const tzEl = document.getElementById("tz");
const rxEl = document.getElementById("rx");
const ryEl = document.getElementById("ry");
const rzEl = document.getElementById("rz");

const movePoseBtn = document.getElementById("movePoseBtn");
const savePoseAsKeyBtn = document.getElementById("savePoseAsKeyBtn");
const readPoseBtn = document.getElementById("readPoseBtn");

const keypointCountEl = document.getElementById("keypointCount");
const keypointListEl = document.getElementById("keypointList");
const clearKeysBtn = document.getElementById("clearKeysBtn");
const saveKeysBtn = document.getElementById("saveKeysBtn");
const loadKeysBtn = document.getElementById("loadKeysBtn");

const interpStepsEl = document.getElementById("interpSteps");
const generatePathBtn = document.getElementById("generatePathBtn");
const runPathBtn = document.getElementById("runPathBtn");
const clearPathBtn = document.getElementById("clearPathBtn");
const exportPathBtn = document.getElementById("exportPathBtn");
const importPathBtn = document.getElementById("importPathBtn");
const importPathFileEl = document.getElementById("importPathFile");
const pathCountEl = document.getElementById("pathCount");
const pathListEl = document.getElementById("pathList");

const viewer = new RobotViewer(viewerEl);

const jointsUI = new JointsUI(jointContainerEl, jointCountEl, {
  onJointCommitted: async (name, value) => {
    try {
      setStatus(`Sending ${name} = ${value.toFixed(3)}`);
      await sendJointCommand(name, value);
      setStatus(`Joint command sent: ${name}`);
    } catch (error) {
      console.warn(error);
      setStatus(`Joint API failed: ${error.message}`);
    }
  },
});

let keypoints = [];
let generatedPath = [];
let isRunningPath = false;

urdfPathEl.value = DEFAULT_URDF_PATH;
interpStepsEl.value = String(PATH_DEFAULTS.interpolationSteps);

loadBtn.addEventListener("click", async () => {
  await loadCurrentRobot();
});

resetViewBtn.addEventListener("click", () => {
  viewer.resetView();
  setStatus("View reset");
});

zeroBtn.addEventListener("click", async () => {
  jointsUI.zeroAll();
  try {
    await sendZeroCommand();
    setStatus("Zero command sent");
  } catch (error) {
    setStatus(`Zero API failed: ${error.message}`);
  }
});

homeBtn.addEventListener("click", async () => {
  try {
    await sendHomeCommand();
    setStatus("Home command sent");
  } catch (error) {
    setStatus(`Home API failed: ${error.message}`);
  }
});

stopBtn.addEventListener("click", async () => {
  try {
    await sendStopCommand();
    isRunningPath = false;
    setStatus("Stop command sent");
  } catch (error) {
    setStatus(`Stop API failed: ${error.message}`);
  }
});

movePoseBtn.addEventListener("click", async () => {
  const pose = getPoseFromInputs();
  try {
    setStatus("Sending task-space pose...");
    await sendPoseCommand(pose);
    setStatus("Task-space pose sent");
  } catch (error) {
    setStatus(`Move pose failed: ${error.message}`);
  }
});

savePoseAsKeyBtn.addEventListener("click", () => {
  const pose = getPoseFromInputs();
  keypoints.push({ ...pose });
  renderKeypoints();
  setStatus(`Key point saved (${keypoints.length})`);
});

readPoseBtn.addEventListener("click", async () => {
  try {
    const pose = await getCurrentPose();
    setPoseInputs(pose);
    setStatus("Current pose loaded from backend");
  } catch (error) {
    setStatus(`Read current pose failed: ${error.message}`);
  }
});

clearKeysBtn.addEventListener("click", () => {
  keypoints = [];
  renderKeypoints();
  setStatus("Key points cleared");
});

saveKeysBtn.addEventListener("click", () => {
  localStorage.setItem(STORAGE_KEYS.keypoints, JSON.stringify(keypoints));
  setStatus("Key points saved locally");
});

loadKeysBtn.addEventListener("click", () => {
  const raw = localStorage.getItem(STORAGE_KEYS.keypoints);
  if (!raw) {
    setStatus("No saved key points found");
    return;
  }

  try {
    const parsed = JSON.parse(raw);
    keypoints = Array.isArray(parsed) ? parsed : [];
    renderKeypoints();
    setStatus("Key points loaded from local storage");
  } catch (error) {
    setStatus(`Load local key points failed: ${error.message}`);
  }
});

generatePathBtn.addEventListener("click", () => {
  const steps = clampInt(parseInt(interpStepsEl.value, 10), 2, 1000);
  interpStepsEl.value = String(steps);

  if (keypoints.length < 2) {
    setStatus("Need at least 2 key points to generate a path");
    return;
  }

  generatedPath = interpolatePath(keypoints, steps);
  renderPath();
  localStorage.setItem(STORAGE_KEYS.path, JSON.stringify(generatedPath));
  setStatus(`Generated ${generatedPath.length} path points`);
});

runPathBtn.addEventListener("click", async () => {
  if (generatedPath.length === 0) {
    setStatus("Path is empty");
    return;
  }

  if (isRunningPath) {
    setStatus("Path is already running");
    return;
  }

  isRunningPath = true;
  setStatus("Running path...");

  try {
    for (let i = 0; i < generatedPath.length; i += 1) {
      if (!isRunningPath) break;

      const pose = generatedPath[i];
      setPoseInputs(pose);
      await sendPoseCommand(pose);
      setStatus(`Running path ${i + 1}/${generatedPath.length}`);
      await sleep(PATH_DEFAULTS.runDelayMs);
    }

    if (isRunningPath) {
      setStatus("Path run complete");
    } else {
      setStatus("Path run stopped");
    }
  } catch (error) {
    setStatus(`Run path failed: ${error.message}`);
  } finally {
    isRunningPath = false;
  }
});

clearPathBtn.addEventListener("click", () => {
  generatedPath = [];
  renderPath();
  setStatus("Path cleared");
});

exportPathBtn.addEventListener("click", () => {
  const data = {
    keypoints,
    path: generatedPath,
    exportedAt: new Date().toISOString(),
  };

  const blob = new Blob([JSON.stringify(data, null, 2)], {
    type: "application/json",
  });

  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = "robot_path.json";
  a.click();
  URL.revokeObjectURL(url);

  setStatus("Path exported");
});

importPathBtn.addEventListener("click", () => {
  importPathFileEl.click();
});

importPathFileEl.addEventListener("change", async (event) => {
  const file = event.target.files?.[0];
  if (!file) return;

  try {
    const text = await file.text();
    const data = JSON.parse(text);

    if (Array.isArray(data.keypoints)) {
      keypoints = data.keypoints;
    }
    if (Array.isArray(data.path)) {
      generatedPath = data.path;
    }

    renderKeypoints();
    renderPath();
    setStatus("Path JSON imported");
  } catch (error) {
    setStatus(`Import failed: ${error.message}`);
  } finally {
    importPathFileEl.value = "";
  }
});

async function loadCurrentRobot() {
  const urdfPath = urdfPathEl.value.trim();
  if (!urdfPath) {
    setStatus("Please provide a URDF path");
    return;
  }

  setStatus("Loading URDF...");
  jointsUI.clear();

  try {
    const robot = await loadRobotFromUrdf(urdfPath);
    viewer.setRobot(robot);
    jointsUI.build(robot);
    setStatus("URDF loaded successfully");
    console.log("Robot:", robot);
    console.log("Joints:", robot.joints);
  } catch (error) {
    console.error(error);
    viewer.clearRobot();
    setStatus(`Load failed: ${error?.message || error}`);
  }
}

function getPoseFromInputs() {
  return {
    x: parseFloat(txEl.value) || 0,
    y: parseFloat(tyEl.value) || 0,
    z: parseFloat(tzEl.value) || 0,
    rx: parseFloat(rxEl.value) || 0,
    ry: parseFloat(ryEl.value) || 0,
    rz: parseFloat(rzEl.value) || 0,
  };
}

function setPoseInputs(pose) {
  txEl.value = formatNumber(pose.x);
  tyEl.value = formatNumber(pose.y);
  tzEl.value = formatNumber(pose.z);
  rxEl.value = formatNumber(pose.rx);
  ryEl.value = formatNumber(pose.ry);
  rzEl.value = formatNumber(pose.rz);
}

function renderKeypoints() {
  keypointCountEl.textContent = `${keypoints.length} points`;
  keypointListEl.innerHTML = "";

  if (keypoints.length === 0) {
    keypointListEl.innerHTML = `<div class="muted">No key points yet.</div>`;
    return;
  }

  keypoints.forEach((pose, index) => {
    const item = document.createElement("div");
    item.className = "list-item";

    const main = document.createElement("div");
    main.className = "list-item-main";

    const title = document.createElement("div");
    title.className = "list-item-title";
    title.textContent = `Key ${index + 1}`;

    const desc = document.createElement("div");
    desc.className = "list-item-desc";
    desc.textContent = poseToText(pose);

    main.appendChild(title);
    main.appendChild(desc);

    const actions = document.createElement("div");
    actions.className = "list-item-actions";

    const useBtn = document.createElement("button");
    useBtn.className = "secondary";
    useBtn.textContent = "Use";
    useBtn.addEventListener("click", () => {
      setPoseInputs(pose);
      setStatus(`Loaded key point ${index + 1} into task-space inputs`);
    });

    const delBtn = document.createElement("button");
    delBtn.className = "danger";
    delBtn.textContent = "Del";
    delBtn.addEventListener("click", () => {
      keypoints.splice(index, 1);
      renderKeypoints();
      setStatus(`Deleted key point ${index + 1}`);
    });

    actions.appendChild(useBtn);
    actions.appendChild(delBtn);

    item.appendChild(main);
    item.appendChild(actions);
    keypointListEl.appendChild(item);
  });
}

function renderPath() {
  pathCountEl.textContent = `${generatedPath.length} path points`;
  pathListEl.innerHTML = "";

  if (generatedPath.length === 0) {
    pathListEl.innerHTML = `<div class="muted">No generated path.</div>`;
    return;
  }

  generatedPath.forEach((pose, index) => {
    const item = document.createElement("div");
    item.className = "list-item";

    const main = document.createElement("div");
    main.className = "list-item-main";

    const title = document.createElement("div");
    title.className = "list-item-title";
    title.textContent = `P${index + 1}`;

    const desc = document.createElement("div");
    desc.className = "list-item-desc";
    desc.textContent = poseToText(pose);

    main.appendChild(title);
    main.appendChild(desc);

    const actions = document.createElement("div");
    actions.className = "list-item-actions";

    const useBtn = document.createElement("button");
    useBtn.className = "secondary";
    useBtn.textContent = "Use";
    useBtn.addEventListener("click", () => {
      setPoseInputs(pose);
      setStatus(`Loaded path point ${index + 1}`);
    });

    actions.appendChild(useBtn);
    item.appendChild(main);
    item.appendChild(actions);
    pathListEl.appendChild(item);
  });
}

function interpolatePath(points, stepsPerSegment) {
  const path = [];

  for (let i = 0; i < points.length - 1; i += 1) {
    const a = points[i];
    const b = points[i + 1];

    for (let s = 0; s < stepsPerSegment; s += 1) {
      const t = s / stepsPerSegment;
      path.push(lerpPose(a, b, t));
    }
  }

  path.push({ ...points[points.length - 1] });
  return path;
}

function lerpPose(a, b, t) {
  return {
    x: lerp(a.x, b.x, t),
    y: lerp(a.y, b.y, t),
    z: lerp(a.z, b.z, t),
    rx: lerp(a.rx, b.rx, t),
    ry: lerp(a.ry, b.ry, t),
    rz: lerp(a.rz, b.rz, t),
  };
}

function lerp(a, b, t) {
  return a * (1 - t) + b * t;
}

function poseToText(pose) {
  return `x=${formatNumber(pose.x)}, y=${formatNumber(pose.y)}, z=${formatNumber(pose.z)}, rx=${formatNumber(pose.rx)}, ry=${formatNumber(pose.ry)}, rz=${formatNumber(pose.rz)}`;
}

function formatNumber(value) {
  return Number(value).toFixed(3);
}

function clampInt(value, min, max) {
  if (!Number.isFinite(value)) return min;
  return Math.max(min, Math.min(max, value));
}

function sleep(ms) {
  return new Promise((resolve) => {
    window.setTimeout(resolve, ms);
  });
}

function setStatus(text) {
  statusEl.textContent = text;
}

loadCurrentRobot();
renderKeypoints();
renderPath();