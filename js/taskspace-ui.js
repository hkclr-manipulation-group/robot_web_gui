import * as THREE from 'three';
import { TASK_LIMITS } from './config.js';

const AXES = [
  { key: 'x', label: 'X', unit: 'm', isAngle: false },
  { key: 'y', label: 'Y', unit: 'm', isAngle: false },
  { key: 'z', label: 'Z', unit: 'm', isAngle: false },
  { key: 'rx', label: 'RX', unit: 'deg', isAngle: true },
  { key: 'ry', label: 'RY', unit: 'deg', isAngle: true },
  { key: 'rz', label: 'RZ', unit: 'deg', isAngle: true },
];

export class TaskSpaceUI {
  constructor(container, callbacks) {
    this.container = container;
    this.callbacks = callbacks;
    this.inputs = {};
    this.sliders = {};
  }

  build(initialPose = null) {
    if (!this.container) return;
    this.container.innerHTML = '';

    const block = document.createElement('div');
    block.className = 'task-block';

    AXES.forEach((axis) => {
      const limits = TASK_LIMITS[axis.key];
      const row = document.createElement('div');
      row.className = 'task-row';

      const label = document.createElement('div');
      label.className = 'axis-label';
      label.textContent = `${axis.label} (${axis.unit})`;

      const slider = document.createElement('input');
      slider.type = 'range';
      slider.min = String(limits.min);
      slider.max = String(limits.max);
      slider.step = String(limits.step);
      slider.value = '0';

      const input = document.createElement('input');
      input.type = 'number';
      input.step = String(limits.step);
      input.value = '0';

      slider.addEventListener('input', () => {
        input.value = Number(slider.value).toFixed(axis.isAngle ? 1 : 4);
      });

      input.addEventListener('change', () => {
        const value = Math.min(limits.max, Math.max(limits.min, parseFloat(input.value || 0)));
        slider.value = String(value);
        input.value = Number(value).toFixed(axis.isAngle ? 1 : 4);
      });

      this.inputs[axis.key] = input;
      this.sliders[axis.key] = slider;

      row.appendChild(label);
      row.appendChild(slider);
      row.appendChild(input);
      block.appendChild(row);
    });

    const actions = document.createElement('div');
    actions.className = 'task-actions';
    actions.innerHTML = `
      <button id="movePoseBtn">Move by IK</button>
      <button id="setAsGoalBtn">Set Goal Snapshot</button>
      <button id="planPoseBtn">Plan Cartesian</button>
      <button id="syncPoseBtn">Sync Inputs</button>
    `;

    actions.querySelector('#movePoseBtn').onclick = () => this.callbacks?.onMove?.(this.getPose());
    actions.querySelector('#setAsGoalBtn').onclick = () => this.callbacks?.onSetGoal?.(this.getPose());
    actions.querySelector('#planPoseBtn').onclick = () => this.callbacks?.onPlanPose?.(this.getPose());
    actions.querySelector('#syncPoseBtn').onclick = () => this.callbacks?.onReadCurrent?.();

    this.container.appendChild(block);
    this.container.appendChild(actions);

    if (initialPose) this.setPose(initialPose);
  }

  getPose() {
    return {
      x: parseFloat(this.inputs.x.value || 0),
      y: parseFloat(this.inputs.y.value || 0),
      z: parseFloat(this.inputs.z.value || 0),
      rx: THREE.MathUtils.degToRad(parseFloat(this.inputs.rx.value || 0)),
      ry: THREE.MathUtils.degToRad(parseFloat(this.inputs.ry.value || 0)),
      rz: THREE.MathUtils.degToRad(parseFloat(this.inputs.rz.value || 0)),
    };
  }

  setPose(pose) {
    if (!pose) return;
    const mapped = {
      x: pose.x,
      y: pose.y,
      z: pose.z,
      rx: THREE.MathUtils.radToDeg(pose.rx),
      ry: THREE.MathUtils.radToDeg(pose.ry),
      rz: THREE.MathUtils.radToDeg(pose.rz),
    };
    AXES.forEach((axis) => {
      const value = Number(mapped[axis.key] || 0);
      if (this.inputs[axis.key]) this.inputs[axis.key].value = value.toFixed(axis.isAngle ? 1 : 4);
      if (this.sliders[axis.key]) this.sliders[axis.key].value = String(value);
    });
  }
}
