import { TASK_LIMITS } from "./config.js";
import { normalizeDisplayZero } from "./utils.js";

const PARTIAL_NUMBER_RE = /^-?(\d+)?(\.\d*)?$/;

const AXES = [
  { key: "x", label: "X", unit: "m", angle: false, decimals: 4 },
  { key: "y", label: "Y", unit: "m", angle: false, decimals: 4 },
  { key: "z", label: "Z", unit: "m", angle: false, decimals: 4 },
  { key: "rx", label: "RX", unit: "deg", angle: true, decimals: 2 },
  { key: "ry", label: "RY", unit: "deg", angle: true, decimals: 2 },
  { key: "rz", label: "RZ", unit: "deg", angle: true, decimals: 2 },
];

function formatAxisValue(value, decimals) {
  return String(normalizeDisplayZero(Number(value), decimals).toFixed(decimals));
}

/**
 * Absolute task-space pose input (base-frame EE pose: m + deg).
 * Frame dropdown (gizmo) is independent — numbers stay base absolute (plan A).
 */
export class TaskInputUI {
  constructor(options = {}) {
    this.listEl = options.listEl;
    this.moveBtn = options.moveBtn;
    this.keypadEl = options.keypadEl;
    this.callbacks = options.callbacks || {};

    this.axes = AXES;
    this.inputs = {};
    this.activeKey = null;
    this.dirty = false;
    this._bound = false;
  }

  clear() {
    if (this.listEl) this.listEl.innerHTML = "";
    this.inputs = {};
    this.activeKey = null;
    this.dirty = false;
  }

  build() {
    this.clear();
    if (!this.listEl) return;

    const fragment = document.createDocumentFragment();
    this.axes.forEach((axis) => {
      const row = document.createElement("div");
      row.className = "joint-input-row";

      const label = document.createElement("div");
      label.className = "joint-input-label";
      label.textContent = `${axis.label} (${axis.unit})`;

      const input = document.createElement("input");
      input.type = "text";
      input.inputMode = "decimal";
      input.autocomplete = "off";
      input.spellcheck = false;
      input.className = "joint-input-field";
      input.dataset.axis = axis.key;
      input.value = formatAxisValue(0, axis.decimals);

      input.addEventListener("focus", () => {
        this.activeKey = axis.key;
      });
      input.addEventListener("input", () => {
        this.dirty = true;
        this.#sanitizeLiveInput(input, axis.decimals);
        input.classList.remove("is-invalid");
        this.callbacks.onDraftChange?.();
      });
      input.addEventListener("keydown", (e) => {
        if (e.key === "Enter") {
          e.preventDefault();
          this.callbacks.onMoveRequest?.();
        }
      });

      this.inputs[axis.key] = input;
      row.appendChild(label);
      row.appendChild(input);
      fragment.appendChild(row);
    });

    this.listEl.appendChild(fragment);
    this.#bindChromeOnce();
  }

  /**
   * @param {{x,y,z,rx,ry,rz}} pose UI units (m / deg)
   * @param {{force?: boolean}} options
   */
  seedFromPose(pose, { force = true } = {}) {
    if (!force && this.dirty) return false;
    if (!pose) return false;

    this.axes.forEach((axis) => {
      const input = this.inputs[axis.key];
      if (!input) return;
      if (!force && document.activeElement === input) return;
      const raw = pose[axis.key];
      const value = Number.isFinite(raw) ? raw : 0;
      const next = formatAxisValue(value, axis.decimals);
      if (input.value !== next) input.value = next;
      input.classList.remove("is-invalid");
    });

    if (force) this.dirty = false;
    this.callbacks.onDraftChange?.();
    return true;
  }

  syncFromPose(pose) {
    return this.seedFromPose(pose, { force: false });
  }

  /**
   * @returns {{ ok: true, pose: object } | { ok: false, message: string }}
   */
  validateForMove() {
    const pose = {};
    for (const axis of this.axes) {
      const input = this.inputs[axis.key];
      const raw = (input?.value ?? "").trim();
      const parsed = this.#parseComplete(raw);
      if (parsed == null) {
        input?.classList.add("is-invalid");
        return { ok: false, message: `Invalid value for ${axis.label}.` };
      }
      const limits = TASK_LIMITS[axis.key];
      if (parsed < limits.min || parsed > limits.max) {
        input?.classList.add("is-invalid");
        return {
          ok: false,
          message: `${axis.label} must be within [${limits.min}, ${limits.max}] ${axis.unit}.`,
        };
      }
      input?.classList.remove("is-invalid");
      pose[axis.key] = parsed;
    }
    return { ok: true, pose };
  }

  getDraftPose() {
    const result = this.validateForMove();
    return result.ok ? result.pose : null;
  }

  #bindChromeOnce() {
    if (this._bound) return;
    this._bound = true;

    this.moveBtn?.addEventListener("click", () => {
      this.callbacks.onMoveRequest?.();
    });
    this.keypadEl?.addEventListener("click", (e) => {
      const btn = e.target.closest("button[data-key]");
      if (!btn) return;
      e.preventDefault();
      this.#applyKey(btn.dataset.key);
    });
  }

  #applyKey(key) {
    const axisKey = this.activeKey || this.axes[0]?.key;
    const axis = this.axes.find((a) => a.key === axisKey);
    const input = this.inputs[axisKey];
    if (!input || !axis) return;

    this.activeKey = axisKey;
    input.focus();

    let next = input.value ?? "";
    if (key === "backspace") {
      next = next.slice(0, -1);
    } else if (key === "-") {
      next = next.startsWith("-") ? next.slice(1) : `-${next}`;
    } else if (key === ".") {
      if (!next.includes(".")) {
        next = next === "" || next === "-" ? `${next}0.` : `${next}.`;
      }
    } else if (/^\d$/.test(key)) {
      next = `${next}${key}`;
    } else {
      return;
    }

    if (next !== "" && !PARTIAL_NUMBER_RE.test(next)) return;

    next = this.#limitFractionDigits(next, axis.decimals);
    this.dirty = true;
    input.value = next;
    input.classList.remove("is-invalid");
    this.callbacks.onDraftChange?.();
  }

  #sanitizeLiveInput(input, maxFrac) {
    let v = input.value.replace(/[^\d.\-]/g, "");
    v = v.replace(/(?!^)-/g, "");
    const firstDot = v.indexOf(".");
    if (firstDot !== -1) {
      v = v.slice(0, firstDot + 1) + v.slice(firstDot + 1).replace(/\./g, "");
    }
    if (v !== "" && !PARTIAL_NUMBER_RE.test(v)) {
      while (v.length && !PARTIAL_NUMBER_RE.test(v)) {
        v = v.slice(0, -1);
      }
    }
    v = this.#limitFractionDigits(v, maxFrac);
    if (input.value !== v) input.value = v;
  }

  #limitFractionDigits(value, maxFrac) {
    const m = String(value).match(/^(-?\d*)(\.(\d*))?$/);
    if (!m) return value;
    const intPart = m[1] ?? "";
    if (m[2] == null) return value;
    const frac = (m[3] ?? "").slice(0, maxFrac);
    if (m[2] === "." && frac === "") return `${intPart}.`;
    return `${intPart}.${frac}`;
  }

  #parseComplete(raw) {
    if (raw === "" || raw === "-" || raw === "." || raw === "-.") return null;
    if (!/^-?\d+(\.\d+)?$/.test(raw)) return null;
    const n = Number(raw);
    return Number.isFinite(n) ? n : null;
  }
}
