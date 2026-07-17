import { STORAGE_KEYS, TEACH } from "./config.js";
import { enableTeachModeApi } from "./api.js";

import { formatJointValue } from "./utils.js";

function jointProgress(value) {
  const min = -Math.PI;
  const max = Math.PI;
  const pct = ((Number(value) || 0) - min) / (max - min);
  return `${Math.max(0, Math.min(1, pct)) * 100}%`;
}

export class TeachSystem {
  constructor() {
    this.poses = [];
  }

  record(jointMap) {
    this.poses.push({ ...jointMap });
  }

  replaceAll(poses) {
    this.poses = poses.map((item) => ({ ...item }));
  }

  clear() {
    this.poses = [];
  }

  getPath() {
    return this.poses.map((item) => ({ ...item }));
  }

  get count() {
    return this.poses.length;
  }
}

function setTeachButtonState(buttonEl, enabled) {
  if (!buttonEl) return;
  buttonEl.disabled = !enabled;
  buttonEl.classList.toggle("is-enabled", enabled);
  buttonEl.classList.toggle("is-disabled", !enabled);
}

/**
 * @param {object} options
 * @param {{ teachCountEl: HTMLElement | null; pathPreviewEl: HTMLElement | null; jointContainerTeachEl: HTMLElement | null; jointContainerEl: HTMLElement | null; teachRecordBtnEl: HTMLElement | null; teachPlayBtnEl: HTMLElement | null; teachStopBtnEl: HTMLElement | null }} options.elements
 * @param {() => import("./kinematics.js").RobotKinematics | null} options.getKinematics
 * @param {() => Record<string, number> | null} options.getRecordJointMap Joint map for teach recording (prefer live telemetry over ghost URDF).
 * @param {(text: string, cls?: string) => void} options.setStatus
 * @param {(trajectory: Record<string, number>[]) => Promise<boolean | void>} options.executeTrajectory
 * @param {() => Promise<unknown>} options.sendStopCommand
 * @param {() => void} options.onClearBusy
 */
export function createTeachModule(options) {
  const {
    elements: {
      teachCountEl,
      pathPreviewEl,
      jointContainerTeachEl,
      jointContainerEl,
      teachRecordBtnEl,
      teachPlayBtnEl,
      teachStopBtnEl,
    },
    getKinematics,
    getRecordJointMap,
    setStatus,
    executeTrajectory,
    sendStopCommand,
    onClearBusy,
  } = options;

  const teachSystem = new TeachSystem();
  let teachUiState = "idle"; // idle | recording | ready | playing
  let teachRecordTimer = null;

  function updateTeachUi() {
    if (teachCountEl) {
      teachCountEl.textContent = `${teachSystem.count} poses`;
    }
    if (pathPreviewEl) {
      pathPreviewEl.textContent = JSON.stringify(teachSystem.getPath(), null, 2);
    }
    localStorage.setItem(
      STORAGE_KEYS.lastTrajectory,
      JSON.stringify(teachSystem.getPath())
    );
  }

  function refreshTeachControls() {
    const hasRecording = teachSystem.count > 0;
    const canRecord = teachUiState === "idle" || teachUiState === "ready";
    const canPlay = teachUiState === "ready" && hasRecording;
    const canStop = teachUiState === "recording" || teachUiState === "playing";

    setTeachButtonState(teachRecordBtnEl, canRecord);
    setTeachButtonState(teachPlayBtnEl, canPlay);
    setTeachButtonState(teachStopBtnEl, canStop);
  }

  function syncTeachJointMirror(jointMap = null) {
    if (!jointContainerTeachEl) return;

    const map = jointMap || getRecordJointMap();
    if (!map) {
      jointContainerTeachEl.innerHTML = "";
      return;
    }

    const entries = Object.entries(map).reverse();
    const existingCards = jointContainerTeachEl.querySelectorAll(".teach-joint-card");

    // Same joint set: update values in place (matches Joint Space drag/stream cadence).
    if (existingCards.length === entries.length) {
      let namesMatch = true;
      for (let i = 0; i < entries.length; i++) {
        const nameEl = existingCards[i].querySelector(".joint-name");
        if (!nameEl || nameEl.textContent !== entries[i][0]) {
          namesMatch = false;
          break;
        }
      }

      if (namesMatch) {
        entries.forEach(([_, value], i) => {
          const card = existingCards[i];
          const valueEl = card.querySelector(".joint-value");
          const progressFill = card.querySelector(".progress-fill");
          if (valueEl) valueEl.textContent = formatJointValue(value);
          if (progressFill) progressFill.style.width = jointProgress(value);
        });
        return;
      }
    }

    const fragment = document.createDocumentFragment();

    entries.forEach(([name, value]) => {
      const card = document.createElement("div");
      card.className = "joint-card teach-joint-card";

      const top = document.createElement("div");
      top.className = "joint-top";

      const nameEl = document.createElement("div");
      nameEl.className = "joint-name";
      nameEl.textContent = name;

      const valueEl = document.createElement("div");
      valueEl.className = "joint-value";
      valueEl.textContent = formatJointValue(value);

      const progressWrap = document.createElement("div");
      progressWrap.className = "progress-wrap";

      const progressTrack = document.createElement("div");
      progressTrack.className = "progress-track";

      const progressFill = document.createElement("div");
      progressFill.className = "progress-fill";
      progressFill.style.width = jointProgress(value);

      progressTrack.appendChild(progressFill);
      progressWrap.appendChild(progressTrack);

      top.appendChild(nameEl);
      top.appendChild(valueEl);
      card.appendChild(top);
      card.appendChild(progressWrap);
      fragment.appendChild(card);
    });

    jointContainerTeachEl.replaceChildren(fragment);
  }

  async function enableTeachMode(enable) {
    const result = await enableTeachModeApi(enable);
    if (result.mode === "preview") return true;
    return !!result.data?.success;
  }

  function startTeachSampling() {
    if (teachRecordTimer) {
      clearInterval(teachRecordTimer);
      teachRecordTimer = null;
    }

    teachRecordTimer = setInterval(() => {
      if (teachUiState !== "recording") return;
      const jointMap = getRecordJointMap();
      if (!jointMap) return;
      teachSystem.record(jointMap);
      syncTeachJointMirror();
      refreshTeachControls();
    }, TEACH.recordSampleIntervalMs);
  }

  function stopTeachSampling() {
    if (!teachRecordTimer) return;
    clearInterval(teachRecordTimer);
    teachRecordTimer = null;
  }

  async function onTeachRecordClick() {
    if (teachUiState === "recording" || teachUiState === "playing") return;
    const jointMap = getRecordJointMap();
    if (!jointMap) return;

    teachSystem.clear();
    teachSystem.record(jointMap);
    teachUiState = "recording";
    refreshTeachControls();
    setStatus("Teach recording started.", "warn");

    try {
      const ok = await enableTeachMode(true);
      if (!ok) {
        teachUiState = "idle";
        refreshTeachControls();
        setStatus("Failed to enable teach mode.", "danger-text");
        return;
      }

      startTeachSampling();
    } catch (error) {
      teachUiState = "idle";
      refreshTeachControls();
      setStatus(error.message || "Failed to enable teach mode.", "danger-text");
    }
  }

  async function onTeachPlayClick() {
    if (teachUiState === "recording" || teachUiState === "playing") return;
    if (!teachSystem.count) return;

    teachUiState = "playing";
    refreshTeachControls();

    try {
      const completed = await executeTrajectory(teachSystem.getPath());
      teachUiState = teachSystem.count ? "ready" : "idle";
      refreshTeachControls();

      if (!completed) {
        setStatus("Teach playback stopped.", "warn");
      }
    } catch (error) {
      teachUiState = teachSystem.count ? "ready" : "idle";
      refreshTeachControls();
      setStatus(error.message || "Teach playback failed.", "danger-text");
    }
  }

  async function onTeachStopClick() {
    if (teachUiState !== "recording" && teachUiState !== "playing") return;

    if (teachUiState === "recording") {
      stopTeachSampling();
      let disableOk = true;
      try {
        const ok = await enableTeachMode(false);
        if (!ok) {
          disableOk = false;
          setStatus("Failed to disable teach mode.", "danger-text");
        }
      } catch (error) {
        disableOk = false;
        setStatus(error.message || "Failed to disable teach mode.", "danger-text");
      }

      teachUiState = teachSystem.count ? "ready" : "idle";
      if (disableOk) {
        setStatus(
          teachSystem.count
            ? `Teach recording stopped. ${teachSystem.count} poses captured.`
            : "Teach recording stopped.",
          teachSystem.count ? "ok" : "warn"
        );
      }
      refreshTeachControls();
      return;
    }

    onClearBusy();
    try {
      await sendStopCommand();
    } catch (error) {
      console.warn("Failed to send stop command:", error);
    }
    // Final status comes from executeTrajectory / onTeachPlayClick ("…stopped").
    teachUiState = teachSystem.count ? "ready" : "idle";
    refreshTeachControls();
  }

  function bindTeachButtons() {
    if (teachRecordBtnEl) teachRecordBtnEl.onclick = onTeachRecordClick;
    if (teachPlayBtnEl) teachPlayBtnEl.onclick = onTeachPlayClick;
    if (teachStopBtnEl) teachStopBtnEl.onclick = onTeachStopClick;
  }

  return {
    system: teachSystem,
    updateTeachUi,
    refreshTeachControls,
    syncTeachJointMirror,
    bindTeachButtons,
  };
}
