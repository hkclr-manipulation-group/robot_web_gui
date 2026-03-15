import * as THREE from 'three';

export class TaskSpaceUI {
  constructor(container, callbacks) {
    this.container = container;
    this.callbacks = callbacks;
    this.inputs = {};
  }

  build(initialPose = null) {
    if (!this.container) return;
    this.container.innerHTML = `
      <div class="task-grid">
        <label>X (m)<input id="tx" type="number" step="0.001" value="0"></label>
        <label>Y (m)<input id="ty" type="number" step="0.001" value="0"></label>
        <label>Z (m)<input id="tz" type="number" step="0.001" value="0"></label>
        <label>RX (deg)<input id="rx" type="number" step="0.1" value="0"></label>
        <label>RY (deg)<input id="ry" type="number" step="0.1" value="0"></label>
        <label>RZ (deg)<input id="rz" type="number" step="0.1" value="0"></label>
      </div>
      <div class="task-actions">
        <button id="readCurrentPoseBtn">Read Current</button>
        <button id="movePoseBtn">Move by IK</button>
        <button id="setAsGoalBtn">Set Goal Snapshot</button>
        <button id="planPoseBtn">Plan Cartesian</button>
      </div>
    `;

    this.inputs = {
      x: this.container.querySelector('#tx'),
      y: this.container.querySelector('#ty'),
      z: this.container.querySelector('#tz'),
      rx: this.container.querySelector('#rx'),
      ry: this.container.querySelector('#ry'),
      rz: this.container.querySelector('#rz'),
    };

    this.container.querySelector('#readCurrentPoseBtn').onclick = () => this.callbacks?.onReadCurrent?.();
    this.container.querySelector('#movePoseBtn').onclick = () => this.callbacks?.onMove?.(this.getPose());
    this.container.querySelector('#setAsGoalBtn').onclick = () => this.callbacks?.onSetGoal?.(this.getPose());
    this.container.querySelector('#planPoseBtn').onclick = () => this.callbacks?.onPlanPose?.(this.getPose());

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
    if (!pose || !this.inputs.x) return;
    this.inputs.x.value = Number(pose.x).toFixed(4);
    this.inputs.y.value = Number(pose.y).toFixed(4);
    this.inputs.z.value = Number(pose.z).toFixed(4);
    this.inputs.rx.value = THREE.MathUtils.radToDeg(Number(pose.rx)).toFixed(2);
    this.inputs.ry.value = THREE.MathUtils.radToDeg(Number(pose.ry)).toFixed(2);
    this.inputs.rz.value = THREE.MathUtils.radToDeg(Number(pose.rz)).toFixed(2);
  }
}
