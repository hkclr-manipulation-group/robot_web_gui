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
    this.rows = {};

    this._ikTimer = null;
    this._lock = false;

  }

  /* ---------------- build ---------------- */

  build(initialPose = null) {

    if (!this.container) return;

    this.container.innerHTML = "";

    const block = document.createElement("div");
    block.className = "task-block";

    const fragment = document.createDocumentFragment();

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

      this.inputs[axis.key] = input;
      this.sliders[axis.key] = slider;
      this.rows[axis.key] = row;

      slider.addEventListener("input", () => {

        if (this._lock) return;

        const v = parseFloat(slider.value);

        input.value = v.toFixed(axis.angle ? 1 : 4);

        this.#scheduleIK();

      });

      input.addEventListener("change", () => {

        if (this._lock) return;

        let v = parseFloat(input.value || 0);

        v = Math.min(limits.max, Math.max(limits.min, v));

        slider.value = v;
        input.value = v.toFixed(axis.angle ? 1 : 4);

        this.#scheduleIK();

      });

      row.appendChild(label);
      row.appendChild(slider);
      row.appendChild(input);

      fragment.appendChild(row);

    });

    block.appendChild(fragment);

    const actions = this.#buildActions();

    this.container.appendChild(block);
    this.container.appendChild(actions);

    if (initialPose) this.setPose(initialPose);

  }

  /* ---------------- actions ---------------- */

  #buildActions() {

    const actions = document.createElement("div");
    actions.className = "task-actions";

    const setBtn = document.createElement("button");
    setBtn.textContent = "Set Goal";

    const planBtn = document.createElement("button");
    planBtn.textContent = "Plan Cartesian";

    const syncBtn = document.createElement("button");
    syncBtn.textContent = "Sync Current";

    setBtn.onclick =
      () => this.callbacks?.onSetGoal?.(this.getPose());

    planBtn.onclick =
      () => this.callbacks?.onPlanPose?.(this.getPose());

    syncBtn.onclick =
      () => this.callbacks?.onReadCurrent?.();

    actions.appendChild(setBtn);
    actions.appendChild(planBtn);
    actions.appendChild(syncBtn);

    return actions;

  }

  /* ---------------- IK debounce ---------------- */

  #scheduleIK() {

    if (!this.callbacks?.onMove) return;

    clearTimeout(this._ikTimer);

    this._ikTimer = setTimeout(() => {

      const pose = this.getPose();

      this.callbacks.onMove(pose);

    }, 40); // 稍微加一点更稳定

  }

  /* ---------------- pose getters ---------------- */

  getPose() {

    const pose = {};

    AXES.forEach(axis => {

      const v = parseFloat(this.inputs[axis.key]?.value || 0);

      pose[axis.key] =
        axis.angle ? THREE.MathUtils.degToRad(v) : v;

    });

    return pose;

  }

  getPoseVector() {

    const pose = this.getPose();

    return [
      pose.x,
      pose.y,
      pose.z,
      pose.rx,
      pose.ry,
      pose.rz
    ];

  }

  /* ---------------- pose setters ---------------- */

  setPose(pose) {

    if (!pose) return;

    this._lock = true;

    const mapped = {

      x: pose.x ?? 0,
      y: pose.y ?? 0,
      z: pose.z ?? 0,

      rx: THREE.MathUtils.radToDeg(pose.rx ?? 0),
      ry: THREE.MathUtils.radToDeg(pose.ry ?? 0),
      rz: THREE.MathUtils.radToDeg(pose.rz ?? 0),

    };

    AXES.forEach(axis => {

      const limits = TASK_LIMITS[axis.key];

      let v = mapped[axis.key];

      if (limits) {

        v = Math.min(limits.max, Math.max(limits.min, v));

      }

      if (this.inputs[axis.key])
        this.inputs[axis.key].value =
          v.toFixed(axis.angle ? 1 : 4);

      if (this.sliders[axis.key])
        this.sliders[axis.key].value = v;

    });

    this._lock = false;

  }

  setPoseVector(v) {

    this.setPose({
      x: v[0],
      y: v[1],
      z: v[2],
      rx: v[3],
      ry: v[4],
      rz: v[5],
    });

  }

}