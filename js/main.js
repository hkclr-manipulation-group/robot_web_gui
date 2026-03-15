import { DEFAULT_URDF_PATH } from "./config.js";
import { sendHomeCommand, sendJointCommand, sendStopCommand, sendZeroCommand } from "./api.js";
import { JointsUI } from "./joints-ui.js";
import { loadRobotFromUrdf } from "./urdf-loader-wrapper.js";
import { RobotViewer } from "./viewer.js";

window.addEventListener('DOMContentLoaded', start)

function start() {
  const viewerEl = document.getElementById("viewer");
  const statusEl = document.getElementById("status");
  const urdfPathEl = document.getElementById("urdfPath");
  const jointContainerEl = document.getElementById("jointContainer");
  const jointCountEl = document.getElementById("jointCount");

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
    } catch (error) {
      viewer.clearRobot();
      setStatus(`Load failed: ${error?.message || error}`);
    }
  }

  function setStatus(text) {
    statusEl.textContent = text;
  }

  document.getElementById("loadBtn").onclick = loadCurrentRobot;

  document.getElementById("movePoseBtn").onclick = () => {
    const pose = {
      x: parseFloat(document.getElementById("tx").value),
      y: parseFloat(document.getElementById("ty").value),
      z: parseFloat(document.getElementById("tz").value)
    };
    sendPoseCommand(pose);
  }
}
