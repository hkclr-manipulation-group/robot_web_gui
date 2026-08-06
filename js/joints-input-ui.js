import {
  CONTINUOUS_RANGE,
  INITIAL_POSE_MATCH_TOLERANCE_DEG,
  ROTARY_FALLBACK_RANGE,
} from "./config.js";

import {
  formatJointInput,
  parseJointInput,
} from "./utils.js";

const PARTIAL_NUMBER_RE = /^-?(\d+)?(\.\d*)?$/;
const TOL_RAD = (INITIAL_POSE_MATCH_TOLERANCE_DEG * Math.PI) / 180;

/**
 * Absolute joint-angle input panel (Input mode): drafts, keypad, Move/Save enable.
 * Display order matches JointsUI (tip→base). Storage / command maps use caller order.
 */
export class JointsInputUI {
  constructor(options = {}) {
    this.listEl = options.listEl;
    this.moveBtn = options.moveBtn;
    this.saveBtn = options.saveBtn;
    this.saveIconEl = options.saveIconEl;
    this.keypadEl = options.keypadEl;
    this.callbacks = options.callbacks || {};

    this.jointNames = []; // display order (reversed)
    this.limitsDeg = {};
    this.inputs = {};
    this.activeName = null;
    /** When true, live actual sync is paused so the user can edit a Move target. */
    this.dirty = false;
    this._bound = false;
  }

  clear() {
    if (this.listEl) this.listEl.innerHTML = "";
    this.jointNames = [];
    this.limitsDeg = {};
    this.inputs = {};
    this.activeName = null;
    this.dirty = false;
    this.setSaveEnabled(false);
  }

  build(robot) {
    this.clear();
    if (!this.listEl || !robot) return;

    const joints = robot.joints || {};
    const names = Object.keys(joints).filter((name) => {
      const type = joints[name]?.jointType;
      return type === "revolute" || type === "continuous";
    });

    // Same display order as JointsUI: tip → base
    names.reverse();
    this.jointNames = names;

    const fragment = document.createDocumentFragment();
    names.forEach((name) => {
      const joint = joints[name];
      const range = this.#getRange(joint);
      const toDeg = (r) => (r * 180) / Math.PI;
      this.limitsDeg[name] = {
        min: toDeg(range.min),
        max: toDeg(range.max),
        continuous: joint.jointType === "continuous",
      };

      const row = document.createElement("div");
      row.className = "joint-input-row";

      const label = document.createElement("div");
      label.className = "joint-input-label";
      label.textContent = name;

      const input = document.createElement("input");
      input.type = "text";
      input.inputMode = "decimal";
      input.autocomplete = "off";
      input.spellcheck = false;
      input.className = "joint-input-field";
      input.dataset.joint = name;
      input.value = formatJointInput(joint.angle ?? 0, false);

      input.addEventListener("focus", () => {
        this.activeName = name;
      });
      input.addEventListener("input", () => {
        this.dirty = true;
        this.#sanitizeLiveInput(input);
        this.#refreshFieldValidity(name);
        this.callbacks.onDraftChange?.();
      });
      input.addEventListener("keydown", (e) => {
        if (e.key === "Enter") {
          e.preventDefault();
          this.callbacks.onMoveRequest?.();
        }
      });

      this.inputs[name] = input;
      row.appendChild(label);
      row.appendChild(input);
      fragment.appendChild(row);
    });

    this.listEl.appendChild(fragment);
    this.#bindChromeOnce();
    this.setSaveEnabled(false);
  }

  /**
   * Write values into inputs. Use force=true after Home / Initial / mode enter / Move.
   * Without force, skips when the user has a dirty draft (editing a Move target).
   */
  seedFromRadians(map, { force = true } = {}) {
    if (!force && this.dirty) return false;

    this.jointNames.forEach((name) => {
      const input = this.inputs[name];
      if (!input) return;
      // Don't clobber the field the user is actively typing into unless forced.
      if (!force && document.activeElement === input) return;
      const rad = map?.[name];
      const value = Number.isFinite(rad) ? rad : 0;
      const next = formatJointInput(value, false);
      if (input.value !== next) {
        input.value = next;
      }
      input.classList.remove("is-invalid");
    });

    if (force) this.dirty = false;
    this.callbacks.onDraftChange?.();
    return true;
  }

  /** Live sync from actual joints (same role as slider value readout). */
  syncFromRadians(map) {
    return this.seedFromRadians(map, { force: false });
  }

  markClean() {
    this.dirty = false;
  }

  /**
   * @returns {{ ok: true, map: Record<string, number> } | { ok: false, message: string }}
   */
  validateForMove() {
    const map = {};
    for (const name of this.jointNames) {
      const input = this.inputs[name];
      const raw = (input?.value ?? "").trim();
      const parsed = this.#parseCompleteDeg(raw);
      if (parsed == null) {
        input?.classList.add("is-invalid");
        return { ok: false, message: `Invalid angle for ${name}.` };
      }
      const limits = this.limitsDeg[name];
      if (!limits.continuous && (parsed < limits.min || parsed > limits.max)) {
        input?.classList.add("is-invalid");
        return {
          ok: false,
          message: `${name} must be within [${limits.min.toFixed(0)}°, ${limits.max.toFixed(0)}°].`,
        };
      }
      input?.classList.remove("is-invalid");
      map[name] = parseJointInput(String(parsed), false);
    }
    return { ok: true, map };
  }

  getDraftRadians() {
    const result = this.validateForMove();
    return result.ok ? result.map : null;
  }

  /**
   * Enable Save when every draft is valid and |draft − actual| ≤ tolerance.
   * @param {Record<string, number>} actualRadMap
   */
  updateMatchState(actualRadMap) {
    let allMatch = true;
    for (const name of this.jointNames) {
      const input = this.inputs[name];
      const raw = (input?.value ?? "").trim();
      const deg = this.#parseCompleteDeg(raw);
      if (deg == null) {
        allMatch = false;
        break;
      }
      const limits = this.limitsDeg[name];
      if (!limits.continuous && (deg < limits.min || deg > limits.max)) {
        allMatch = false;
        break;
      }
      const draftRad = parseJointInput(String(deg), false);
      const actual = actualRadMap?.[name];
      if (!Number.isFinite(actual) || Math.abs(draftRad - actual) > TOL_RAD) {
        allMatch = false;
        break;
      }
    }
    this.setSaveEnabled(allMatch);
    return allMatch;
  }

  setSaveEnabled(enabled) {
    if (this.saveBtn) {
      this.saveBtn.disabled = !enabled;
      this.saveBtn.classList.toggle("is-disabled", !enabled);
      this.saveBtn.setAttribute("aria-disabled", enabled ? "false" : "true");
    }
    if (this.saveIconEl) {
      this.saveIconEl.src = enabled
        ? "./image/Group 228.svg"
        : "./image/Group 229.svg";
    }
  }

  #bindChromeOnce() {
    if (this._bound) return;
    this._bound = true;

    this.moveBtn?.addEventListener("click", () => {
      this.callbacks.onMoveRequest?.();
    });
    this.saveBtn?.addEventListener("click", () => {
      if (this.saveBtn.disabled) return;
      this.callbacks.onSaveRequest?.();
    });
    this.keypadEl?.addEventListener("click", (e) => {
      const btn = e.target.closest("button[data-key]");
      if (!btn) return;
      e.preventDefault();
      this.#applyKey(btn.dataset.key);
    });
  }

  #applyKey(key) {
    const name = this.activeName || this.jointNames[0];
    const input = this.inputs[name];
    if (!input) return;

    this.activeName = name;
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

    if (next !== "" && !PARTIAL_NUMBER_RE.test(next)) {
      return;
    }

    // Cap fractional digits to 2 while typing (keep trailing "." / incomplete ok)
    next = this.#limitFractionDigits(next, 2);
    this.dirty = true;
    input.value = next;
    this.#refreshFieldValidity(name);
    this.callbacks.onDraftChange?.();
  }

  #sanitizeLiveInput(input) {
    let v = input.value.replace(/[^\d.\-]/g, "");
    // Keep only leading minus
    v = v.replace(/(?!^)-/g, "");
    // Single dot
    const firstDot = v.indexOf(".");
    if (firstDot !== -1) {
      v =
        v.slice(0, firstDot + 1) +
        v.slice(firstDot + 1).replace(/\./g, "");
    }
    if (v !== "" && !PARTIAL_NUMBER_RE.test(v)) {
      // Revert to last good partial by trimming last char
      while (v.length && !PARTIAL_NUMBER_RE.test(v)) {
        v = v.slice(0, -1);
      }
    }
    v = this.#limitFractionDigits(v, 2);
    if (input.value !== v) input.value = v;
  }

  #limitFractionDigits(value, maxFrac) {
    const m = String(value).match(/^(-?\d*)(\.(\d*))?$/);
    if (!m) return value;
    const intPart = m[1] ?? "";
    if (m[2] == null) return value;
    const frac = (m[3] ?? "").slice(0, maxFrac);
    // Preserve trailing dot when user just typed it
    if (m[2] === "." && frac === "") return `${intPart}.`;
    return `${intPart}.${frac}`;
  }

  #parseCompleteDeg(raw) {
    if (raw === "" || raw === "-" || raw === "." || raw === "-.") return null;
    if (!/^-?\d+(\.\d+)?$/.test(raw)) return null;
    const n = Number(raw);
    return Number.isFinite(n) ? n : null;
  }

  #refreshFieldValidity(name) {
    const input = this.inputs[name];
    if (!input) return;
    const raw = input.value.trim();
    if (
      raw === "" ||
      raw === "-" ||
      raw === "." ||
      raw === "-." ||
      PARTIAL_NUMBER_RE.test(raw)
    ) {
      // Partial: don't mark invalid yet unless clearly over limit with complete number
      const deg = this.#parseCompleteDeg(raw);
      if (deg == null) {
        input.classList.remove("is-invalid");
        return;
      }
      const limits = this.limitsDeg[name];
      if (!limits.continuous && (deg < limits.min || deg > limits.max)) {
        // Soft: allow display, light mark optional — keep soft (no hard invalid while typing)
        input.classList.remove("is-invalid");
      } else {
        input.classList.remove("is-invalid");
      }
    }
  }

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
