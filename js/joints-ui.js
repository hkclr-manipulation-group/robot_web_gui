import {
  CONTINUOUS_RANGE,
  ROTARY_FALLBACK_RANGE,
} from "./config.js";

import {
  clamp,
  cssSafe,
  formatJointValue,
} from "./utils.js";

export class JointsUI {
  constructor(container, countElement, callbacks, options = {}) {
    this.container = container;
    this.countElement = countElement;
    this.callbacks = callbacks;

    // 可配置参数
    this.intervalMs = options.intervalMs ?? 500; // 连续调节的时间间隔（毫秒），默认500ms
    this.stepDeg = options.stepDeg ?? 1; // 每次步进的角度，默认1度

    this.jointMap = {};
    this.uiMap = {}; // 缓存DOM
    this.jointNames = [];
    this.interactingJoints = new Set(); // 记录正在交互的关节
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
      return type === "revolute" || type === "continuous";
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

  /**
   * @param {boolean} applyUrdfJoint 若为 false：只刷新面板数值/进度条，不改写机器人关节角度（仿真臂可由指令单独驱动）
   */
  setJointValue(name, value, updateUi = true, applyUrdfJoint = true) {
    const joint = this.jointMap[name];
    const ui = this.uiMap[name];

    if (!joint) return;

    if (applyUrdfJoint) {
      joint.setVal = value;
      if (typeof joint.setJointValue === "function") {
        joint.setJointValue(value);
      } else {
        joint.angle = value;
      }
    }

    if (!updateUi || !ui) return;

    // 更新数值显示
    ui.value.textContent = formatJointValue(value, false);

    // 更新进度条：映射到实际关节限位范围
    const card = ui.slider.closest(".joint-card");
    if (card) {
      const range = this.#getRange(joint);
      const toDeg = (r) => (r * 180) / Math.PI;
      const degValue = toDeg(value);
      const degMin = toDeg(range.min);
      const degMax = toDeg(range.max);
      const pct = (degValue - degMin) / (degMax - degMin || 1);

      const fill = card.querySelector(".progress-fill");
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

  /**
   * @param {number[]} q 遥测关节向量（与 jointNames 同序）
   * @param {{ updateGhostUrdfJoints?: boolean }} options
   *        updateGhostUrdfJoints=false 时：只更新面板反映真实硬件，不挪动仿真 URDF（用于滞后对比）
   */
  syncFromStreamData(q, options = {}) {
    const updateGhostUrdfJoints =
      options.updateGhostUrdfJoints !== false;

    this.jointNames.forEach((name, index) => {
      if (this.interactingJoints.has(name)) {
        return;
      }

      const value = q[index];

      this.setJointValue(name, value, true, updateGhostUrdfJoints);

      if (this.uiMap[name]) {
        this.uiMap[name].baseValue = value;
      }
    });
  }

  /* ---------------- card creation ---------------- */

  #createJointCard(name, joint) {
    const type = joint.jointType || "unknown";
    const range = this.#getRange(joint);
    const current = Number.isFinite(joint.angle) ? joint.angle : 0;

    const card = document.createElement("div");
    card.className = "joint-card";

    const top = document.createElement("div");
    top.className = "joint-top";

    const nameEl = document.createElement("div");
    nameEl.className = "joint-name";
    nameEl.textContent = name;

    const valueEl = document.createElement("div");
    valueEl.className = "joint-value";
    valueEl.textContent = formatJointValue(current, false);

    const slider = document.createElement("input");
    slider.type = "range";
    slider.min = "-1";
    slider.max = "1";
    slider.step = "0.1"; // 改为0.1以确保平滑拖拽和事件触发
    slider.value = "0";
    
    // 禁用默认过渡效果以实现即时响应
    slider.style.transition = 'none';

    /* ---------- cache UI ---------- */
    this.uiMap[name] = {
      slider,
      value: valueEl,
      baseValue: current, // 存储基础值用于增量计算
      intervalId: null,   // 用于连续调节的定时器
      lastDir: 0          // 初始化最后方向
    };

    /* ---------- slider events (Incremental Mode) ---------- */
    const rad = (d) => (d * Math.PI) / 180;
    
    // 核心动作：应用增量并累加，直接实时下发
    const applyAndAccumulate = (deltaDeg) => {
      const ui = this.uiMap[name];
      if (!ui) {
        console.error(`[applyAndAccumulate] ${name}: UI not found!`);
        return;
      }
      
      let currentBase = ui.baseValue ?? 0;
      let newValue = currentBase + rad(deltaDeg);
      
      // 获取关节范围并进行边界限制
      const joint = this.jointMap[name];
      if (joint) {
        const range = this.#getRange(joint);
        // 对于连续关节，不进行限制
        if (joint.jointType !== "continuous") {
          newValue = clamp(newValue, range.min, range.max);
        }
      }
      
      console.log(`[applyAndAccumulate] ${name}: delta=${deltaDeg}°, base=${currentBase.toFixed(6)}rad, new=${newValue.toFixed(6)}rad (${(newValue * 180 / Math.PI).toFixed(2)}°)`);
      
      // 更新显示
      ui.value.textContent = formatJointValue(newValue, false);
      
      // 更新 baseValue 以便下一次累加
      ui.baseValue = newValue;
      
      // 更新进度条
      const card = ui.slider.closest('.joint-card');
      if (card) {
        const joint = this.jointMap[name];
        if (joint) {
          const range = this.#getRange(joint);
          const toDeg = (r) => (r * 180) / Math.PI;
          const degValue = toDeg(newValue);
          const degMin = toDeg(range.min);
          const degMax = toDeg(range.max);
          const pct = (degValue - degMin) / (degMax - degMin || 1);
          
          const fill = card.querySelector('.progress-fill');
          if (fill) {
            fill.style.width = `${Math.max(0, Math.min(1, pct)) * 100}%`;
          }
        }
      }
      
      // 直接实时下发指令（不再区分预览和提交）
      console.log(`[applyAndAccumulate] ${name}: Directly sending to robot with value=${newValue.toFixed(6)}`);
      this.callbacks?.onJointInput?.(name, newValue);
    };

    // 开始移动（启动定时器）
    const startMoving = (direction) => {
       const ui = this.uiMap[name];
       if (!ui) {
         console.error(`[startMoving] ${name}: UI not found!`);
         return;
       }

       console.log(`[startMoving] ${name}: direction=${direction}, current baseValue=${ui.baseValue?.toFixed(6) || 'undefined'}`);

       // 如果方向没变，不要重启定时器，避免闪烁
       if (ui.lastDir === direction && ui.intervalId) {
         console.log(`[startMoving] ${name}: Same direction, skipping restart`);
         return;
       }

       // 标记该关节正在被交互
       this.interactingJoints.add(name);
       console.log(`[startMoving] ${name}: Added to interactingJoints`);

       // 清除旧的
       if (ui.intervalId) {
         console.log(`[startMoving] ${name}: Clearing old interval`);
         clearInterval(ui.intervalId);
       }
       
       // 立即执行一次
       console.log(`[startMoving] ${name}: Executing first step`);
       applyAndAccumulate(direction * this.stepDeg);
       
       // 记录方向
       ui.lastDir = direction;

       // 连续执行 (使用可配置的时间间隔)
       console.log(`[startMoving] ${name}: Starting interval (${this.intervalMs}ms)`);
       ui.intervalId = setInterval(() => {
         applyAndAccumulate(direction * this.stepDeg);
       }, this.intervalMs); 
    };

    // 停止移动
    const stopMoving = () => {
      const ui = this.uiMap[name];
      if (!ui) return;

      // 如果没有正在运行的定时器，说明已经停止过了
      if (!ui.intervalId && ui.lastDir === 0) {
        console.log(`[stopMoving] ${name}: Already stopped, skipping`);
        return;
      }

      console.log(`[stopMoving] ${name}: Stopping, final baseValue=${ui.baseValue?.toFixed(6) || 'undefined'}`);

      if (ui.intervalId) {
        clearInterval(ui.intervalId);
        ui.intervalId = null;
        console.log(`[stopMoving] ${name}: Interval cleared`);
      }
      
      // 清除交互锁标记
      this.interactingJoints.delete(name);
      console.log(`[stopMoving] ${name}: Removed from interactingJoints`);
      
      // 重置方向
      ui.lastDir = 0;
            
      // 滑块视觉上回中 (延迟一点，确保 input 事件处理完)
      setTimeout(() => {
        if (ui.slider) {
          ui.slider.value = "0";
          console.log(`[stopMoving] ${name}: Slider reset to 0`);
        }
      }, 10);
    };

    // 监听 input 事件 - 使用阈值判断方向
    slider.addEventListener("input", (e) => {
      const rawVal = parseFloat(e.target.value);
      console.log(`[Slider Input] ${name}: rawVal=${rawVal}, type=${typeof rawVal}`);
      
      // 阈值判断：大于0.5视为向右，小于-0.5视为向左
      if (rawVal > 0.5) {
        console.log(`[Slider] ${name}: Moving RIGHT`);
        startMoving(1); // 向右
      } else if (rawVal < -0.5) {
        console.log(`[Slider] ${name}: Moving LEFT, value=${rawVal}`);
        startMoving(-1); // 向左
      } else {
        console.log(`[Slider] ${name}: Stopped (center), value=${rawVal}`);
        // 在中间区域，停止任何连续移动
        const ui = this.uiMap[name];
        if (ui && ui.intervalId) {
          stopMoving();
        }
      }
    });
    
    // 监听结束事件，确保停止
    const handleEnd = () => {
      stopMoving();
    };

    slider.addEventListener("change", handleEnd);
    slider.addEventListener("mouseup", handleEnd);
    slider.addEventListener("touchend", handleEnd);
    // 防止鼠标移出滑块区域后仍然持续移动
    slider.addEventListener("mouseleave", () => {
      // 鼠标移出时停止移动
      stopMoving();
    });

    /* ---------- Layout Construction ---------- */
    const controls = document.createElement("div");
    controls.className = "joint-controls";

    // 1. Progress Preview (Left)
    const progressWrap = document.createElement('div');
    progressWrap.className = 'progress-wrap';
    progressWrap.appendChild(valueEl);

    const progressTrack = document.createElement('div');
    progressTrack.className = 'progress-track';
    const progressFill = document.createElement('div');
    progressFill.className = 'progress-fill';
    
    // 初始化进度条：映射到实际关节限位
    const toDeg = (r) => (r * 180) / Math.PI;
    const degCurrent = toDeg(current);
    const degMin = toDeg(range.min);
    const degMax = toDeg(range.max);
    const pctInit = (degCurrent - degMin) / (degMax - degMin || 1);
    progressFill.style.width = `${Math.max(0, Math.min(1, pctInit)) * 100}%`;
    
    progressTrack.appendChild(progressFill);

    // 标签：显示实际限位
    const labelsRow = document.createElement('div');
    labelsRow.className = 'range-labels';
    
    const minLabel = document.createElement('div');
    minLabel.className = 'range-label range-label-min';
    minLabel.textContent = String(Math.round(toDeg(range.min))) + "°";
    
    const maxLabel = document.createElement('div');
    maxLabel.className = 'range-label range-label-max';
    maxLabel.textContent = String(Math.round(toDeg(range.max))) + "°";
    
    labelsRow.appendChild(minLabel);
    labelsRow.appendChild(maxLabel);

    progressWrap.appendChild(progressTrack);
    progressWrap.appendChild(labelsRow);

    // 2. Slider (Right)
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
    const fallback = ROTARY_FALLBACK_RANGE;
    let min = fallback.min;
    let max = fallback.max;

    if (joint.limit) {
      if (Number.isFinite(joint.limit.lower)) min = joint.limit.lower;
      if (Number.isFinite(joint.limit.upper)) max = joint.limit.upper;
    }

    if (joint.jointType === "continuous") {
      return CONTINUOUS_RANGE;
    }

    return { min, max, step: fallback.step };
  }
}