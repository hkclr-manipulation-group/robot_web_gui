import {
  CONTINUOUS_RANGE,
  PRISMATIC_FALLBACK_RANGE,
  ROTARY_FALLBACK_RANGE,
} from "./config.js";

import {
  clamp,
  cssSafe,
  formatJointInput,
  formatJointValue,
  parseJointInput,
} from "./utils.js";

export class JointsUI {
  constructor(container, countElement, callbacks) {
    this.container = container;
    this.countElement = countElement;
    this.callbacks = callbacks;

    this.jointMap = {};
    this.uiMap = {}; // 缓存DOM
    this.jointNames = [];
  }

  /* ---------------- clear ---------------- */

  clear() {
    if (this.container) this.container.innerHTML = "";
    if (this.countElement) this.countElement.textContent = "0 joints";

    this.jointMap = {};
    this.uiMap = {};
    this.jointNames = [];
  }

  /* ---------------- build ---------------- */

  build(robot) {
    this.clear();

    if (!this.container || !robot) return;

    const joints = robot.joints || {};

    const names = Object.keys(joints).filter((name) => {
      const type = joints[name]?.jointType;
      return type === "revolute" || type === "continuous" || type === "prismatic";
    });

    this.jointNames = names;

    if (this.countElement)
      this.countElement.textContent = `${names.length} joints`;

    if (!names.length) {
      this.container.innerHTML =
        '<div class="label">No controllable joints found.</div>';
      return;
    }

    const fragment = document.createDocumentFragment();

    names.forEach((name) => {
      const joint = joints[name];
      this.jointMap[name] = joint;

      const card = this.#createJointCard(name, joint);
      fragment.appendChild(card);
    });

    this.container.appendChild(fragment);
  }

  /* ---------------- state access ---------------- */

  getJointNames() {
    return this.jointNames;
  }

  getValuesAsMap() {
    const map = {};

    this.jointNames.forEach((name) => {
      const joint = this.jointMap[name];
      map[name] = joint.angle ?? 0;
    });

    return map;
  }

  getValuesAsVector() {
    return this.jointNames.map((name) => this.jointMap[name].angle ?? 0);
  }

  /* ---------------- set values ---------------- */

  setJointValue(name, value, updateUi = true) {
    const joint = this.jointMap[name];
    const ui = this.uiMap[name];

    if (!joint) return;

    joint.setVal = value;
    // if (typeof joint.setJointValue === "function") {
    //   joint.setJointValue(value);
    // } else {
    //   joint.angle = value;
    // }

    if (!updateUi || !ui) return;

    const isPrismatic = joint.jointType === "prismatic";

    ui.slider.value = String(value);
    ui.value.textContent = formatJointValue(value, isPrismatic);
    ui.input.value = formatJointInput(value, isPrismatic);
  }

  setValuesByMap(map, updateUi = true) {
    Object.entries(map).forEach(([name, value]) =>
      this.setJointValue(name, value, updateUi)
    );
  }

  setValuesByVector(q, updateUi = true) {
    this.jointNames.forEach((name, i) =>
      this.setJointValue(name, q[i], updateUi)
    );
  }

  zeroAll() {
    this.jointNames.forEach((name) =>
      this.setJointValue(name, 0, true)
    );
  }

  syncFromStreamData(q) {
    this.jointNames.forEach((name) => {
      const joint = this.jointMap[name];
      const index = this.jointNames.indexOf(name);
      const value = q[index];
      if (typeof joint.setJointValue === "function") {
        joint.setJointValue(value);
      } else {
        joint.angle = value;
      }
    });
  }

  /* ---------------- card creation ---------------- */

  #createJointCard(name, joint) {
    const type = joint.jointType || "unknown";
    const isPrismatic = type === "prismatic";

    const range = this.#getRange(joint);
    const current = Number.isFinite(joint.angle) ? joint.angle : 0;

    const safe = cssSafe(name);

    const card = document.createElement("div");
    card.className = "joint-card";

    const top = document.createElement("div");
    top.className = "joint-top";

    const nameEl = document.createElement("div");
    nameEl.className = "joint-name";
    nameEl.textContent = `${name} (${type})`;

    const valueEl = document.createElement("div");
    valueEl.className = "joint-value";
    valueEl.textContent = formatJointValue(current, isPrismatic);

    const slider = document.createElement("input");
    slider.type = "range";
    slider.min = String(range.min);
    slider.max = String(range.max);
    slider.step = String(range.step);
    slider.value = String(clamp(current, range.min, range.max));

    const input = document.createElement("input");
    input.type = "number";
    input.step = isPrismatic ? "0.001" : "0.1";
    input.value = formatJointInput(current, isPrismatic);

    /* ---------- cache UI ---------- */

    this.uiMap[name] = {
      slider,
      input,
      value: valueEl,
    };

    /* ---------- slider input ---------- */

    slider.addEventListener("input", (e) => {
      const value = parseFloat(e.target.value);

      this.setJointValue(name, value, false);

      valueEl.textContent = formatJointValue(value, isPrismatic);
      input.value = formatJointInput(value, isPrismatic);

      this.callbacks?.onJointInput?.(name, value);
    });

    /* ---------- slider commit ---------- */

    slider.addEventListener("change", async (e) => {
      const value = parseFloat(e.target.value);

      await this.callbacks?.onJointCommitted?.(name, value);
    });

    /* ---------- numeric input ---------- */

    input.addEventListener("change", async (e) => {
      const parsed = clamp(
        parseJointInput(e.target.value, isPrismatic),
        range.min,
        range.max
      );

      slider.value = String(parsed);

      this.setJointValue(name, parsed, false);

      valueEl.textContent = formatJointValue(parsed, isPrismatic);
      input.value = formatJointInput(parsed, isPrismatic);

      this.callbacks?.onJointInput?.(name, parsed);

      await this.callbacks?.onJointCommitted?.(name, parsed);
    });

    const controls = document.createElement("div");
    controls.className = "joint-controls";

    controls.appendChild(slider);
    controls.appendChild(input);

    top.appendChild(nameEl);
    top.appendChild(valueEl);

    card.appendChild(top);
    card.appendChild(controls);

    return card;
  }

  /* ---------------- range ---------------- */

  #getRange(joint) {
    const type = joint.jointType;

    if (type === "continuous") return CONTINUOUS_RANGE;

    const fallback =
      type === "prismatic"
        ? PRISMATIC_FALLBACK_RANGE
        : ROTARY_FALLBACK_RANGE;

    let min = fallback.min;
    let max = fallback.max;
    const step = fallback.step;

    if (joint.limit) {
      if (Number.isFinite(joint.limit.lower)) min = joint.limit.lower;
      if (Number.isFinite(joint.limit.upper)) max = joint.limit.upper;
    }

    return { min, max, step };
  }
}