import * as THREE from "three";
import { TASK_LIMITS } from "./config.js";

const AXES = [
  { key: "x", label: "X", unit: "m", angle: false },
  { key: "y", label: "Y", unit: "m", angle: false },
  { key: "z", label: "Z", unit: "m", angle: false },
  { key: "rx", label: "RX", unit: "deg", angle: true },
  { key: "ry", label: "RY", unit: "deg", angle: true },
  { key: "rz", label: "RZ", unit: "deg", angle: true },
];

export class TaskSpaceUI {

  constructor(container, callbacks) {

    this.container = container;
    this.callbacks = callbacks;

    this.inputs = {};
    this.sliders = {};

    this._ikTimer = null;
    this._lock = false;   // 防止 setPose → triggerIK 循环

  }

  build(initialPose = null) {

    if (!this.container) return;

    this.container.innerHTML = "";

    const block = document.createElement("div");
    block.className = "task-block";

    AXES.forEach(axis => {

      const limits = TASK_LIMITS[axis.key];

      const row = document.createElement("div");
      row.className = "task-row";

      const label = document.createElement("div");
      label.className = "axis-label";
      label.textContent = `${axis.label} (${axis.unit})`;

      const slider = document.createElement("input");
      slider.type = "range";
      slider.min = limits.min;
      slider.max = limits.max;
      slider.step = limits.step;
      slider.value = 0;

      const input = document.createElement("input");
      input.type = "number";
      input.step = limits.step;
      input.value = 0;

      slider.addEventListener("input", () => {

        if (this._lock) return;

        const v = parseFloat(slider.value);
        input.value = v.toFixed(axis.angle ? 1 : 4);

        this.#triggerIK();

      });

      input.addEventListener("change", () => {

        if (this._lock) return;

        let v = parseFloat(input.value || 0);
        v = Math.min(limits.max, Math.max(limits.min, v));

        slider.value = v;
        input.value = v.toFixed(axis.angle ? 1 : 4);

        this.#triggerIK();

      });

      this.inputs[axis.key] = input;
      this.sliders[axis.key] = slider;

      row.appendChild(label);
      row.appendChild(slider);
      row.appendChild(input);

      block.appendChild(row);

    });

    const actions = document.createElement("div");
    actions.className = "task-actions";

    actions.innerHTML = `
      <button id="setGoalBtn">Set Goal</button>
      <button id="planPoseBtn">Plan Cartesian</button>
      <button id="syncPoseBtn">Sync Current</button>
    `;

    actions.querySelector("#setGoalBtn").onclick =
      () => this.callbacks?.onSetGoal?.(this.getPose());

    actions.querySelector("#planPoseBtn").onclick =
      () => this.callbacks?.onPlanPose?.(this.getPose());

    actions.querySelector("#syncPoseBtn").onclick =
      () => this.callbacks?.onReadCurrent?.();

    this.container.appendChild(block);
    this.container.appendChild(actions);

    if (initialPose) this.setPose(initialPose);

  }

  #triggerIK() {

    if (!this.callbacks?.onMove) return;

    clearTimeout(this._ikTimer);

    this._ikTimer = setTimeout(() => {

      const pose = this.getPose();

      this.callbacks.onMove(pose);

    }, 30);

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

    this._lock = true;   // 防止触发 IK

    const mapped = {

      x: pose.x,
      y: pose.y,
      z: pose.z,

      rx: THREE.MathUtils.radToDeg(pose.rx),
      ry: THREE.MathUtils.radToDeg(pose.ry),
      rz: THREE.MathUtils.radToDeg(pose.rz),

    };

    AXES.forEach(axis => {

      const v = mapped[axis.key] || 0;

      if (this.inputs[axis.key])
        this.inputs[axis.key].value = v.toFixed(axis.angle ? 1 : 4);

      if (this.sliders[axis.key])
        this.sliders[axis.key].value = v;

    });

    this._lock = false;

  }

}