import { CONTINUOUS_RANGE, PRISMATIC_FALLBACK_RANGE, ROTARY_FALLBACK_RANGE } from './config.js';

export class JointsUI {
  constructor(container, countElement, callbacks) {
    this.container = container;
    this.countElement = countElement;
    this.callbacks = callbacks;
    this.jointMap = {};
  }

  clear() {
    if (!this.container) return;
    this.container.innerHTML = '';
    this.countElement.textContent = '0 joints';
  }

  build(robot) {
    this.clear();
    const joints = robot.joints || {};
    const names = Object.keys(joints);
    this.countElement.textContent = names.length + " joints";

    names.forEach((name) => {
      const joint = joints[name];
      const slider = document.createElement("input");
      slider.type = "range";
      slider.min = -Math.PI;
      slider.max = Math.PI;
      slider.step = 0.01;
      slider.addEventListener("change", () => {
        this.callbacks.onJointCommitted(name, parseFloat(slider.value));
      });
      this.container.appendChild(slider);
    });
  }
}
