import { DEFAULT_URDF_PATH } from './config.js';
import { sendHomeCommand, sendJointCommand, sendStopCommand, sendZeroCommand } from './api.js';
import { JointsUI } from './joints-ui.js';
import { loadRobotFromUrdf } from './urdf-loader-wrapper.js';
import { RobotViewer } from './viewer.js';

const viewerEl = document.getElementById('viewer');
const statusEl = document.getElementById('status');
const urdfPathEl = document.getElementById('urdfPath');
const jointContainerEl = document.getElementById('jointContainer');
const jointCountEl = document.getElementById('jointCount');

const loadBtn = document.getElementById('loadBtn');
const resetViewBtn = document.getElementById('resetViewBtn');
const zeroBtn = document.getElementById('zeroBtn');
const homeBtn = document.getElementById('homeBtn');
const stopBtn = document.getElementById('stopBtn');

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

urdfPathEl.value = DEFAULT_URDF_PATH;

loadBtn.addEventListener('click', async () => {
  await loadCurrentRobot();
});

resetViewBtn.addEventListener('click', () => {
  viewer.resetView();
  setStatus('View reset');
});

zeroBtn.addEventListener('click', async () => {
  jointsUI.zeroAll();
  try {
    await sendZeroCommand();
    setStatus('Zero command sent');
  } catch (error) {
    setStatus(`Zero API failed: ${error.message}`);
  }
});

homeBtn.addEventListener('click', async () => {
  try {
    await sendHomeCommand();
    setStatus('Home command sent');
  } catch (error) {
    setStatus(`Home API failed: ${error.message}`);
  }
});

stopBtn.addEventListener('click', async () => {
  try {
    await sendStopCommand();
    setStatus('Stop command sent');
  } catch (error) {
    setStatus(`Stop API failed: ${error.message}`);
  }
});

async function loadCurrentRobot() {
  const urdfPath = urdfPathEl.value.trim();
  if (!urdfPath) {
    setStatus('Please provide a URDF path');
    return;
  }

  setStatus('Loading URDF...');
  jointsUI.clear();

  try {
    const robot = await loadRobotFromUrdf(urdfPath);
    viewer.setRobot(robot);
    jointsUI.build(robot);
    setStatus('URDF loaded successfully');
    console.log('Robot:', robot);
    console.log('Joints:', robot.joints);
  } catch (error) {
    console.error(error);
    viewer.clearRobot();
    setStatus(`Load failed: ${error?.message || error}`);
  }
}

function setStatus(text) {
  statusEl.textContent = text;
}

loadCurrentRobot();
