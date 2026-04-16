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

    // slider UI uses degrees while internal value is radians
    const toDeg = (r) => (r * 180) / Math.PI;
    ui.slider.value = String(toDeg(value));
    ui.value.textContent = formatJointValue(value, isPrismatic);
    // update progress fill if present (map by degrees)
    const card = ui.slider.closest('.joint-card');
    if (card) {
      const range = this.#getRange(joint);
      const degMin = toDeg(range.min);
      const degMax = toDeg(range.max);
      const pct = (toDeg(value) - degMin) / (degMax - degMin || 1);
      const fill = card.querySelector('.progress-fill');
      if (fill) fill.style.width = `${Math.max(0, Math.min(1, pct)) * 100}%`;
    }
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
    // For prismatic joints, keep meters; for rotary use degrees in UI.
    const deg = (v) => (v * 180) / Math.PI;
    const rad = (d) => (d * Math.PI) / 180;
    let sliderMin, sliderMax, sliderStep, sliderValue;
    if (isPrismatic) {
      sliderMin = range.min;
      sliderMax = range.max;
      sliderStep = range.step || 0.001;
      sliderValue = clamp(current, range.min, range.max);
    } else {
      sliderMin = deg(range.min);
      sliderMax = deg(range.max);
      sliderStep = deg(range.step) || 0.1;
      sliderValue = deg(clamp(current, range.min, range.max));
    }
    slider.min = String(sliderMin);
    slider.max = String(sliderMax);
    // use 'any' to avoid browser HTML5 validation tooltips when value/step
    // floating-point rounding causes mismatches. keep original step on data attribute.
    slider.setAttribute('data-step', String(sliderStep));
    slider.step = 'any';
    slider.value = String(Number.isFinite(sliderValue) ? sliderValue : 0);

    /* ---------- cache UI ---------- */

    this.uiMap[name] = {
      slider,
      value: valueEl,
    };

    /* ---------- slider input ---------- */

    slider.addEventListener("input", (e) => {
      const raw = parseFloat(e.target.value);
      const valueRad = isPrismatic ? raw : rad(raw);

      this.setJointValue(name, valueRad, false);

      // update displayed value and progress fill
      valueEl.textContent = formatJointValue(valueRad, isPrismatic);
      const fill = card.querySelector('.progress-fill');
      if (fill) {
        const a = isPrismatic ? sliderMin : sliderMin; // already in UI units
        const b = isPrismatic ? sliderMax : sliderMax;
        const pct = (raw - a) / (b - a || 1);
        fill.style.width = `${Math.max(0, Math.min(1, pct)) * 100}%`;
      }

      // callbacks receive radians (internal units)
      this.callbacks?.onJointInput?.(name, valueRad);
    });

    /* ---------- slider commit ---------- */

    slider.addEventListener("change", async (e) => {
      const raw = parseFloat(e.target.value);
      const valueRad = isPrismatic ? raw : rad(raw);
      await this.callbacks?.onJointCommitted?.(name, valueRad);
    });

    // numeric input removed — changes are handled via slider

    const controls = document.createElement("div");
    controls.className = "joint-controls";

    // build progress preview (left) and large slider (right)
    const progressWrap = document.createElement('div');
    progressWrap.className = 'progress-wrap';

      // show joint current value above the progress track
      progressWrap.appendChild(valueEl);

      const progressTrack = document.createElement('div');
      progressTrack.className = 'progress-track';
      const progressFill = document.createElement('div');
      progressFill.className = 'progress-fill';
      const pctInit = (sliderValue - sliderMin) / (sliderMax - sliderMin || 1);
      progressFill.style.width = `${Math.max(0, Math.min(1, pctInit)) * 100}%`;
      progressTrack.appendChild(progressFill);

      // labels row below the track: min left, max right
      const minLabel = document.createElement('div');
      minLabel.className = 'range-label range-label-min';
      minLabel.textContent = String(Math.round(isPrismatic ? sliderMin : sliderMin));
      const maxLabel = document.createElement('div');
      maxLabel.className = 'range-label range-label-max';
      maxLabel.textContent = String(Math.round(isPrismatic ? sliderMax : sliderMax));

      const labelsRow = document.createElement('div');
      labelsRow.className = 'range-labels';
      labelsRow.appendChild(minLabel);
      labelsRow.appendChild(maxLabel);

      progressWrap.appendChild(progressTrack);
      progressWrap.appendChild(labelsRow);

    const sliderWrap = document.createElement('div');
    sliderWrap.className = 'slider-wrap';
    slider.className = 'slider-large';
    sliderWrap.appendChild(slider);

    controls.appendChild(progressWrap);
    controls.appendChild(sliderWrap);

    top.appendChild(nameEl);

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