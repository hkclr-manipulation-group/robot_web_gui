import {
  CONTINUOUS_RANGE,
  PRISMATIC_FALLBACK_RANGE,
  ROTARY_FALLBACK_RANGE,
} from './config.js';

export class JointsUI {
  constructor(container, countElement, callbacks) {
    // 确保 container 和 countElement 被正确初始化
    if (!container || !countElement) {
      console.error('Container or CountElement is null or undefined.');
      return;
    }

    this.container = container;
    this.countElement = countElement;
    this.callbacks = callbacks;
    this.jointMap = {};
  }

  clear() {
    if (this.container) {
      this.container.innerHTML = '';
    }
    if (this.countElement) {
      this.countElement.textContent = '0 joints';
    }
    this.jointMap = {};
  }

  build(robot) {
    this.clear();
    const joints = robot.joints || {};
    const names = Object.keys(joints).filter((name) => {
      const jt = joints[name]?.jointType;
      return jt === 'revolute' || jt === 'continuous' || jt === 'prismatic';
    });

    if (this.countElement) {
      this.countElement.textContent = `${names.length} joints`;
    }

    if (names.length === 0) {
      if (this.container) {
        this.container.innerHTML = '<div class="muted">No controllable joints found.</div>';
      }
      return;
    }

    names.forEach((name) => {
      const joint = joints[name];
      this.jointMap[name] = joint;
      const card = this.#createJointCard(name, joint);
      if (this.container) {
        this.container.appendChild(card);
      }
    });
  }

  setJointValue(name, value, updateUi = true) {
    const joint = this.jointMap[name];
    if (!joint) return;

    if (typeof joint.setJointValue === 'function') joint.setJointValue(value);
    else joint.angle = value;

    if (updateUi) {
      const slider = document.getElementById(`slider-${cssSafe(name)}`);
      const valueEl = document.getElementById(`value-${cssSafe(name)}`);
      const isPrismatic = joint.jointType === 'prismatic';
      if (slider) slider.value = String(value);
      if (valueEl) valueEl.textContent = formatJointValue(value, isPrismatic);
    }
  }

  zeroAll() {
    Object.keys(this.jointMap).forEach((name) => this.setJointValue(name, 0, true));
  }

  #createJointCard(name, joint) {
    const jointType = joint.jointType || 'unknown';
    const isPrismatic = jointType === 'prismatic';
    const range = this.#getRange(joint);
    const current = Number.isFinite(joint.angle) ? joint.angle : 0;

    const card = document.createElement('div');
    card.className = 'joint-card';

    const top = document.createElement('div');
    top.className = 'joint-top';

    const nameEl = document.createElement('div');
    nameEl.className = 'joint-name';
    nameEl.textContent = `${name} (${jointType})`;

    const valueEl = document.createElement('div');
    valueEl.className = 'joint-value';
    valueEl.id = `value-${cssSafe(name)}`;
    valueEl.textContent = formatJointValue(current, isPrismatic);

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.id = `slider-${cssSafe(name)}`;
    slider.min = String(range.min);
    slider.max = String(range.max);
    slider.step = String(range.step);
    slider.value = String(clamp(current, range.min, range.max));

    slider.addEventListener('input', (event) => {
      const value = parseFloat(event.target.value);
      this.setJointValue(name, value, false);
      valueEl.textContent = formatJointValue(value, isPrismatic);
    });

    slider.addEventListener('change', async (event) => {
      const value = parseFloat(event.target.value);
      await this.callbacks?.onJointCommitted?.(name, value);
    });

    top.appendChild(nameEl);
    top.appendChild(valueEl);
    card.appendChild(top);
    card.appendChild(slider);
    return card;
  }

  #getRange(joint) {
    const type = joint.jointType;

    if (type === 'continuous') return CONTINUOUS_RANGE;

    const fallback = type === 'prismatic' ? PRISMATIC_FALLBACK_RANGE : ROTARY_FALLBACK_RANGE;

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

// 使用时确保 container 和 countElement 不为 null
document.addEventListener('DOMContentLoaded', () => {
  const container = document.getElementById("yourContainerId");
  const countElement = document.getElementById("yourCountElementId");

  if (container && countElement) {
    const jointsUI = new JointsUI(container, countElement, {
      onJointCommitted: async (name, value) => {
        console.log(`Joint ${name} committed with value: ${value}`);
      }
    });

    // 假设机器人对象
    const robot = {
      joints: {
        joint1: { jointType: 'revolute', angle: 0 },
        joint2: { jointType: 'prismatic', angle: 10 },
        joint3: { jointType: 'continuous', angle: 5 }
      }
    };

    jointsUI.build(robot);
  } else {
    console.error('Container or CountElement not found.');
  }
});