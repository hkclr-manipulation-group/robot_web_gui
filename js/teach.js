import { STORAGE_KEYS } from "./config.js";
import { enableTeachModeApi } from "./api.js";

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

  function syncTeachJointMirror() {
    if (!jointContainerTeachEl || !jointContainerEl) return;
    jointContainerTeachEl.innerHTML = jointContainerEl.innerHTML;
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
      const kinematics = getKinematics();
      if (!kinematics || teachUiState !== "recording") return;
      teachSystem.record(kinematics.getCurrentJointMap());
      syncTeachJointMirror();
      refreshTeachControls();
    }, 180);
  }

  function stopTeachSampling() {
    if (!teachRecordTimer) return;
    clearInterval(teachRecordTimer);
    teachRecordTimer = null;
  }

  async function onTeachRecordClick() {
    if (teachUiState === "recording" || teachUiState === "playing") return;
    const kinematics = getKinematics();
    if (!kinematics) return;

    teachSystem.clear();
    teachSystem.record(kinematics.getCurrentJointMap());
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
      try {
        const ok = await enableTeachMode(false);
        if (!ok) {
          setStatus("Failed to disable teach mode.", "danger-text");
        }
      } catch (error) {
        setStatus(error.message || "Failed to disable teach mode.", "danger-text");
      }

      teachUiState = teachSystem.count ? "ready" : "idle";
      setStatus(
        teachSystem.count
          ? `Teach recording stopped. ${teachSystem.count} poses captured.`
          : "Teach recording stopped.",
        teachSystem.count ? "ok" : "warn"
      );
      refreshTeachControls();
      return;
    }

    onClearBusy();
    try {
      await sendStopCommand();
    } catch (error) {
      console.warn("Failed to send stop command:", error);
    }
    teachUiState = teachSystem.count ? "ready" : "idle";
    setStatus("Teach playback stop requested.", "warn");
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
