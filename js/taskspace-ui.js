import * as THREE from "three";
import { TASK_LIMITS } from "./config.js";
import { clamp } from "./utils.js";

const AXES = [
  { key: "x", label: "X", unit: "m", angle: false },
  { key: "y", label: "Y", unit: "m", angle: false },
  { key: "z", label: "Z", unit: "m", angle: false },
  { key: "rx", label: "RX", unit: "deg", angle: true },
  { key: "ry", label: "RY", unit: "deg", angle: true },
  { key: "rz", label: "RZ", unit: "deg", angle: true },
];

export class TaskSpaceUI {
  constructor(container, callbacks, options = {}) {
    this.container = container;
    this.callbacks = callbacks;

    // 可配置参数
    this.intervalMs = options.intervalMs ?? 500; // 连续调节的时间间隔（毫秒），默认500ms
    this.stepTrans = options.stepTrans ?? 0.01; // 平移每次步进的米数，默认0.01m
    this.stepRot = options.stepRot ?? 1; // 旋转每次步进的角度，默认1度

    this.uiMap = {}; // 缓存DOM
    this.currentPose = { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 };
    this.interactingAxes = new Set(); // 记录正在交互的轴
    
    // 新增：控制模式（绝对位姿 / 增量位姿）
    this.controlMode = options.controlMode ?? 1; // 0: incremental, 1: absolute (默认绝对位姿)
  }

  /* ---------------- clear ---------------- */

  clear() {
    if (this.container) this.container.innerHTML = "";
    this.uiMap = {};
  }

  /* ---------------- build ---------------- */

  build(initialPose = null) {
    this.clear();

    if (!this.container) return;

    const fragment = document.createDocumentFragment();

    // 控制模式切换开关
    // const modeSwitch = this.#createModeSwitch();
    // fragment.appendChild(modeSwitch);

    AXES.forEach((axis) => {
      const card = this.#createAxisCard(axis);
      fragment.appendChild(card);
    });

    this.container.appendChild(fragment);

    if (initialPose) this.setPose(initialPose);
  }

  /* ---------------- state access ---------------- */

  getPose() {
    return { ...this.currentPose };
  }

  getPoseVector() {
    const pose = this.getPose();
    return [pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz];
  }

  /* ---------------- set values ---------------- */

  setPose(pose) {
    if (!pose) return;

    this.currentPose = {
      x: pose.x ?? this.currentPose.x,
      y: pose.y ?? this.currentPose.y,
      z: pose.z ?? this.currentPose.z,
      rx: pose.rx ?? this.currentPose.rx,
      ry: pose.ry ?? this.currentPose.ry,
      rz: pose.rz ?? this.currentPose.rz,
    };

    // 更新所有UI
    AXES.forEach((axis) => {
      this.#updateAxisUI(axis.key);
    });
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

  syncFromStreamData(pose) {
    // 如果有正在被用户交互，跳过同步以避免冲突
    if (this.interactingAxes.size > 0) {
      return;
    }

    this.setPose(pose);
  }

  /* ---------------- card creation ---------------- */

  #createModeSwitch() {
    const container = document.createElement("div");
    container.className = "mode-switch-container";
    container.style.cssText = `
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 12px 16px;
      margin-bottom: 16px;
      background: rgba(255, 255, 255, 0.05);
      border-radius: 8px;
      border: 1px solid rgba(255, 255, 255, 0.1);
    `;

    const label = document.createElement("span");
    label.textContent = "控制模式:";
    label.style.cssText = `
      color: #e0e0e0;
      font-size: 14px;
      font-weight: 500;
    `;

    const switchContainer = document.createElement("div");
    switchContainer.className = "switch-wrapper";
    switchContainer.style.cssText = `
      display: flex;
      gap: 8px;
    `;

    // 绝对位姿按钮
    const absoluteBtn = document.createElement("button");
    absoluteBtn.textContent = "绝对位姿";
    absoluteBtn.className = `mode-btn ${this.controlMode === 1 ? 'active' : ''}`;
    absoluteBtn.style.cssText = `
      padding: 6px 12px;
      border: 1px solid ${this.controlMode === 1 ? '#4CAF50' : 'rgba(255, 255, 255, 0.2)'};
      background: ${this.controlMode === 1 ? '#4CAF50' : 'transparent'};
      color: ${this.controlMode === 1 ? 'white' : '#e0e0e0'};
      border-radius: 4px;
      cursor: pointer;
      font-size: 13px;
      transition: all 0.2s;
    `;
    absoluteBtn.addEventListener("click", () => {
      this.setControlMode(1);
    });

    // 增量位姿按钮
    const incrementalBtn = document.createElement("button");
    incrementalBtn.textContent = "增量位姿";
    incrementalBtn.className = `mode-btn ${this.controlMode === 0 ? 'active' : ''}`;
    incrementalBtn.style.cssText = `
      padding: 6px 12px;
      border: 1px solid ${this.controlMode === 0 ? '#FF9800' : 'rgba(255, 255, 255, 0.2)'};
      background: ${this.controlMode === 0 ? '#FF9800' : 'transparent'};
      color: ${this.controlMode === 0 ? 'white' : '#e0e0e0'};
      border-radius: 4px;
      cursor: pointer;
      font-size: 13px;
      transition: all 0.2s;
    `;
    incrementalBtn.addEventListener("click", () => {
      this.setControlMode(0);
    });

    switchContainer.appendChild(absoluteBtn);
    switchContainer.appendChild(incrementalBtn);

    container.appendChild(label);
    container.appendChild(switchContainer);

    return container;
  }

  setControlMode(mode) {
    if (mode !== 0 && mode !== 1) {
      console.error(`[TaskSpaceUI] Invalid control mode: ${mode} (expected 0 or 1)`);
      return;
    }

    const modeName = mode === 0 ? 'incremental' : 'absolute';
    console.log(`[TaskSpaceUI] Switching control mode from ${this.controlMode === 0 ? 'incremental' : 'absolute'} to ${modeName}`);
    this.controlMode = mode;

    // 更新按钮状态
    const container = this.container.querySelector('.mode-switch-container');
    if (container) {
      const buttons = container.querySelectorAll('.mode-btn');
      buttons.forEach(btn => {
        const isAbsolute = btn.textContent === '绝对位姿';
        const isActive = (isAbsolute && mode === 1) || (!isAbsolute && mode === 0);
        
        btn.style.borderColor = isActive ? (isAbsolute ? '#4CAF50' : '#FF9800') : 'rgba(255, 255, 255, 0.2)';
        btn.style.background = isActive ? (isAbsolute ? '#4CAF50' : '#FF9800') : 'transparent';
        btn.style.color = isActive ? 'white' : '#e0e0e0';
      });
    }

    setStatus(`已切换到${mode === 0 ? '增量位姿' : '绝对位姿'}模式`, "ok");
  }

  #createAxisCard(axis) {
    const limits = TASK_LIMITS[axis.key];
    const currentValue = this.currentPose[axis.key] || 0;

    const card = document.createElement("div");
    card.className = "joint-card";

    const top = document.createElement("div");
    top.className = "joint-top";

    const nameEl = document.createElement("div");
    nameEl.className = "joint-name";
    nameEl.textContent = `${axis.label} (${axis.unit})`;

    const valueEl = document.createElement("div");
    valueEl.className = "joint-value";
    valueEl.textContent = this.#formatValue(currentValue, axis.angle);

    const slider = document.createElement("input");
    slider.type = "range";
    slider.min = "-1";
    slider.max = "1";
    slider.step = "0.1"; // 以确保平滑拖拽和事件触发
    slider.value = "0";
    
    // 禁用默认过渡效果以实现即时响应
    slider.style.transition = 'none';

    /* ---------- cache UI ---------- */
    this.uiMap[axis.key] = {
      slider,
      value: valueEl,
      baseValue: currentValue, // 存储基础值用于增量计算
      intervalId: null,   // 用于连续调节的定时器
      lastDir: 0          // 初始化最后方向
    };

    /* ---------- slider events (Incremental Mode) ---------- */
    
    // 核心动作：应用增量并累加，直接实时下发
    const applyAndAccumulate = (delta) => {
      const ui = this.uiMap[axis.key];
      if (!ui) {
        console.error(`[applyAndAccumulate] ${axis.key}: UI not found!`);
        return;
      }
      
      let currentBase = ui.baseValue ?? 0;
      let newValue = currentBase + delta;
      
      // 获取范围并进行边界限制
      const range = TASK_LIMITS[axis.key];
      newValue = clamp(newValue, range.min, range.max);
      
      console.log(`[applyAndAccumulate] ${axis.key}: delta=${delta}, base=${currentBase.toFixed(6)}, new=${newValue.toFixed(6)}`);
      
      // 更新显示
      ui.value.textContent = this.#formatValue(newValue, axis.angle);
      
      // 更新 baseValue 以便下一次累加
      ui.baseValue = newValue;
      
      // 更新 currentPose
      this.currentPose[axis.key] = newValue;
      
      // 更新进度条
      this.#updateProgressBar(axis.key, newValue);
      
      // 根据控制模式下发指令
      if (this.controlMode === 0) {
        // 增量模式：只发送本次的单次增量 delta，不发送累计值
        // 构建一个只有当前轴有增量的对象，其他轴为 0
        const singleDelta = {};
        AXES.forEach(axis => {
          singleDelta[axis.key] = 0;  // 先初始化为 0
        });
        singleDelta[axis.key] = delta;  // 设置当前轴的增量
        
        console.log(`[applyAndAccumulate] ${axis.key}: Incremental mode - sending single delta=${delta.toFixed(6)}`);
        this.callbacks?.onMoveIncremental?.(singleDelta);
      } else {
        // 绝对位姿模式：发送当前绝对位姿
        console.log(`[applyAndAccumulate] ${axis.key}: Absolute mode - sending pose=${newValue.toFixed(6)}`);
        this.callbacks?.onMove?.(this.getPose());
      }
    };

    // 开始移动（启动定时器）
    const startMoving = (direction) => {
       const ui = this.uiMap[axis.key];
       if (!ui) {
         console.error(`[startMoving] ${axis.key}: UI not found!`);
         return;
       }

       console.log(`[startMoving] ${axis.key}: direction=${direction}, current baseValue=${ui.baseValue?.toFixed(6) || 'undefined'}`);

       // 如果方向没变，不要重启定时器，避免闪烁
       if (ui.lastDir === direction && ui.intervalId) {
         console.log(`[startMoving] ${axis.key}: Same direction, skipping restart`);
         return;
       }

       // 标记该轴正在被交互
       this.interactingAxes.add(axis.key);
       console.log(`[startMoving] ${axis.key}: Added to interactingAxes`);

       // 清除旧的
       if (ui.intervalId) {
         console.log(`[startMoving] ${axis.key}: Clearing old interval`);
         clearInterval(ui.intervalId);
       }
       
       // 立即执行一次
       console.log(`[startMoving] ${axis.key}: Executing first step`);
       const stepSize = axis.angle ? this.stepRot : this.stepTrans;
       applyAndAccumulate(direction * stepSize);
       
       // 记录方向
       ui.lastDir = direction;

       // 连续执行 (使用可配置的时间间隔)
       console.log(`[startMoving] ${axis.key}: Starting interval (${this.intervalMs}ms)`);
       ui.intervalId = setInterval(() => {
         applyAndAccumulate(direction * stepSize);
       }, this.intervalMs); 
    };

    // 停止移动
    const stopMoving = () => {
      const ui = this.uiMap[axis.key];
      if (!ui) return;

      // 如果没有正在运行的定时器，说明已经停止过了
      if (!ui.intervalId && ui.lastDir === 0) {
        console.log(`[stopMoving] ${axis.key}: Already stopped, skipping`);
        return;
      }

      console.log(`[stopMoving] ${axis.key}: Stopping, final baseValue=${ui.baseValue?.toFixed(6) || 'undefined'}`);

      if (ui.intervalId) {
        clearInterval(ui.intervalId);
        ui.intervalId = null;
        console.log(`[stopMoving] ${axis.key}: Interval cleared`);
      }
      
      // 清除交互锁标记
      this.interactingAxes.delete(axis.key);
      console.log(`[stopMoving] ${axis.key}: Removed from interactingAxes`);
      
      // 重置方向
      ui.lastDir = 0;
                  
      // 滑块视觉上回中 (延迟一点，确保 input 事件处理完)
      setTimeout(() => {
        if (ui.slider) {
          ui.slider.value = "0";
          console.log(`[stopMoving] ${axis.key}: Slider reset to 0`);
        }
      }, 10);
    };

    // 监听 input 事件 - 使用阈值判断方向
    slider.addEventListener("input", (e) => {
      const rawVal = parseFloat(e.target.value);
      console.log(`[Slider Input] ${axis.key}: rawVal=${rawVal}, type=${typeof rawVal}`);
      
      // 阈值判断：大于0.5视为向右/向上，小于-0.5视为向左/向下
      if (rawVal > 0.5) {
        console.log(`[Slider] ${axis.key}: Moving POSITIVE`);
        startMoving(1); // 正向
      } else if (rawVal < -0.5) {
        console.log(`[Slider] ${axis.key}: Moving NEGATIVE, value=${rawVal}`);
        startMoving(-1); // 负向
      } else {
        console.log(`[Slider] ${axis.key}: Stopped (center), value=${rawVal}`);
        // 在中间区域，停止任何连续移动
        const ui = this.uiMap[axis.key];
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
    
    // 初始化进度条
    const pctInit = (currentValue - limits.min) / (limits.max - limits.min || 1);
    progressFill.style.width = `${Math.max(0, Math.min(1, pctInit)) * 100}%`;
    
    progressTrack.appendChild(progressFill);

    // 标签：显示实际限位
    const labelsRow = document.createElement('div');
    labelsRow.className = 'range-labels';
    
    const minLabel = document.createElement('div');
    minLabel.className = 'range-label range-label-min';
    minLabel.textContent = String(limits.min) + (axis.angle ? "°" : "");
    
    const maxLabel = document.createElement('div');
    maxLabel.className = 'range-label range-label-max';
    maxLabel.textContent = String(limits.max) + (axis.angle ? "°" : "");
    
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

  /* ---------------- helpers ---------------- */

  #formatValue(value, isAngle) {
    if (isAngle) {
      return `${value.toFixed(1)}°`;
    } else {
      return value.toFixed(4);
    }
  }

  #updateAxisUI(axisKey) {
    const ui = this.uiMap[axisKey];
    if (!ui) return;

    const value = this.currentPose[axisKey];
    ui.value.textContent = this.#formatValue(value, AXES.find(a => a.key === axisKey).angle);
    ui.baseValue = value;
    
    this.#updateProgressBar(axisKey, value);
  }

  #updateProgressBar(axisKey, value) {
    const ui = this.uiMap[axisKey];
    if (!ui) return;

    const card = ui.slider.closest('.joint-card');
    if (!card) return;

    const limits = TASK_LIMITS[axisKey];
    const pct = (value - limits.min) / (limits.max - limits.min || 1);
    
    const fill = card.querySelector('.progress-fill');
    if (fill) {
      fill.style.width = `${Math.max(0, Math.min(1, pct)) * 100}%`;
    }
  }
}