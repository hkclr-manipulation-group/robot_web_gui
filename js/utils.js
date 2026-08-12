import * as THREE from 'three';

export function cssSafe(name) {
  return String(name).replace(/[^a-zA-Z0-9_-]/g, '_');
}

export function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

/** Snap tiny values and -0 to 0 so UI does not flicker between "0.00" and "-0.00". */
export function normalizeDisplayZero(value, displayDecimals = 2) {
  const n = Number(value);
  if (!Number.isFinite(n)) return 0;
  const epsilon = 0.5 * 10 ** -displayDecimals;
  if (Math.abs(n) <= epsilon) return 0;
  return Object.is(n, -0) ? 0 : n;
}

export function formatJointValue(value, isPrismatic = false) {
  if (isPrismatic) {
    const n = normalizeDisplayZero(Number(value), 4);
    return `${n.toFixed(4)} m`;
  }
  const deg = normalizeDisplayZero(THREE.MathUtils.radToDeg(Number(value)), 2);
  return `${deg.toFixed(2)}°`;
}

export function formatJointInput(value, isPrismatic = false) {
  if (isPrismatic) {
    return String(normalizeDisplayZero(Number(value), 4).toFixed(4));
  }
  return String(normalizeDisplayZero(THREE.MathUtils.radToDeg(Number(value)), 2).toFixed(2));
}

export function formatTaskSpaceValue(value, isAngle = false) {
  if (isAngle) {
    const deg = normalizeDisplayZero(Number(value), 1);
    return `${deg.toFixed(1)}°`;
  }
  return normalizeDisplayZero(Number(value), 4).toFixed(4);
}

/** End Effector Pose card (meters, 4 decimals). */
export function formatEePoseValue(value) {
  return formatTaskSpaceValue(value, false);
}

export function parseJointInput(value, isPrismatic = false) {
  const num = parseFloat(value || 0);
  return isPrismatic ? num : THREE.MathUtils.degToRad(num);
}

export function formatPoseText(pose) {
  return `x=${formatEePoseValue(pose.x)} y=${formatEePoseValue(pose.y)} z=${formatEePoseValue(pose.z)} `;
}

export function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

export function poseToObject(position, euler) {
  return { x: position.x, y: position.y, z: position.z, rx: euler.x, ry: euler.y, rz: euler.z };
}

export function matrixToPose(matrix) {
  const position = new THREE.Vector3();
  const quaternion = new THREE.Quaternion();
  const scale = new THREE.Vector3();
  matrix.decompose(position, quaternion, scale);
  const euler = new THREE.Euler().setFromQuaternion(quaternion, 'XYZ');
  return poseToObject(position, euler);
}

export function quaternionToPose(position, quaternion) {
  const quat = new THREE.Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
  const euler = new THREE.Euler().setFromQuaternion(quat, 'XYZ');
  const pos = new THREE.Vector3(position[0], position[1], position[2]);
  return poseToObject(pos, euler);
}

const _streamTempQuat = new THREE.Quaternion();
const _streamTempEuler = new THREE.Euler();
/** Reject per-frame Euler branch flips when orientation barely changed (Qt uses quat in state panel). */
const STREAM_ORIENTATION_STABLE_RAD = THREE.MathUtils.degToRad(0.05);

/** Keep Euler degrees continuous vs a prior UI pose (±360 wraps, same orientation). */
export function unwrapEulerDeg(next, prev) {
  const unwrap = (value, reference) => {
    let v = value;
    while (v - reference > 180) v -= 360;
    while (v - reference < -180) v += 360;
    return v;
  };
  return {
    rx: unwrap(next.rx, prev.rx),
    ry: unwrap(next.ry, prev.ry),
    rz: unwrap(next.rz, prev.rz),
  };
}

/**
 * Build a stream helper that converts gateway EE telemetry into stable TaskSpaceUI degrees.
 * Gateway format: [x, y, z, qw, qx, qy, qz].
 */
export function createStreamEulerStabilizer() {
  let lastQuat = null;
  let lastEulerDeg = null;

  function taskPoseDegFromStream(eePose) {
    const x = eePose[0];
    const y = eePose[1];
    const z = eePose[2];
    let rx = 0;
    let ry = 0;
    let rz = 0;

    if (eePose.length >= 7) {
      _streamTempQuat.set(eePose[4], eePose[5], eePose[6], eePose[3]);

      if (lastQuat) {
        const dot = Math.abs(_streamTempQuat.dot(lastQuat));
        const angle = 2 * Math.acos(Math.min(1, dot));
        if (angle < STREAM_ORIENTATION_STABLE_RAD && lastEulerDeg) {
          return { x, y, z, ...lastEulerDeg };
        }
      }

      _streamTempEuler.setFromQuaternion(_streamTempQuat, 'XYZ');
      const next = {
        rx: THREE.MathUtils.radToDeg(_streamTempEuler.x),
        ry: THREE.MathUtils.radToDeg(_streamTempEuler.y),
        rz: THREE.MathUtils.radToDeg(_streamTempEuler.z),
      };
      const stable = lastEulerDeg ? unwrapEulerDeg(next, lastEulerDeg) : next;
      lastQuat = _streamTempQuat.clone();
      lastEulerDeg = stable;
      rx = stable.rx;
      ry = stable.ry;
      rz = stable.rz;
    } else if (eePose.length >= 6) {
      const next = {
        rx: THREE.MathUtils.radToDeg(eePose[3]),
        ry: THREE.MathUtils.radToDeg(eePose[4]),
        rz: THREE.MathUtils.radToDeg(eePose[5]),
      };
      const stable = lastEulerDeg ? unwrapEulerDeg(next, lastEulerDeg) : next;
      lastEulerDeg = stable;
      rx = stable.rx;
      ry = stable.ry;
      rz = stable.rz;
      lastQuat = null;
    }

    return { x, y, z, rx, ry, rz };
  }

  function reset() {
    lastQuat = null;
    lastEulerDeg = null;
  }

  return { taskPoseDegFromStream, reset };
}

export function quaternionError(targetQ, currentQ) {

  const qErr = targetQ.clone().multiply(currentQ.clone().invert());

  const sign = qErr.w >= 0 ? 1 : -1;

  return new THREE.Vector3(
    2 * sign * qErr.x,
    2 * sign * qErr.y,
    2 * sign * qErr.z
  );

}

export function vectorNorm(arr) {
  return Math.sqrt(arr.reduce((sum, v) => sum + v * v, 0));
}

export function transpose(mat) {
  return mat[0].map((_, col) => mat.map((row) => row[col]));
}

export function matMul(A, B) {
  const rows = A.length;
  const cols = B[0].length;
  const inner = B.length;
  const out = Array.from({ length: rows }, () => Array(cols).fill(0));
  for (let i = 0; i < rows; i++) {
    for (let k = 0; k < inner; k++) {
      for (let j = 0; j < cols; j++) out[i][j] += A[i][k] * B[k][j];
    }
  }
  return out;
}

export function matVecMul(A, v) {
  return A.map((row) => row.reduce((sum, a, i) => sum + a * v[i], 0));
}

export function identity(n) {
  return Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
}

export function inverse(matrix) {
  const n = matrix.length;
  const A = matrix.map((row) => [...row]);
  const I = identity(n);
  for (let i = 0; i < n; i++) {
    let pivotRow = i;
    for (let r = i + 1; r < n; r++) {
      if (Math.abs(A[r][i]) > Math.abs(A[pivotRow][i])) pivotRow = r;
    }
    if (Math.abs(A[pivotRow][i]) < 1e-12) throw new Error('Matrix inversion failed');
    [A[i], A[pivotRow]] = [A[pivotRow], A[i]];
    [I[i], I[pivotRow]] = [I[pivotRow], I[i]];
    const pivot = A[i][i];
    for (let j = 0; j < n; j++) {
      A[i][j] /= pivot;
      I[i][j] /= pivot;
    }
    for (let r = 0; r < n; r++) {
      if (r === i) continue;
      const factor = A[r][i];
      for (let c = 0; c < n; c++) {
        A[r][c] -= factor * A[i][c];
        I[r][c] -= factor * I[i][c];
      }
    }
  }
  return I;
}

export function dampedLeastSquares(J, error, damping = 0.1) {
  const JT = transpose(J);
  const JJT = matMul(J, JT);
  const lambda2 = damping * damping;
  for (let i = 0; i < JJT.length; i++) JJT[i][i] += lambda2;
  const inv = inverse(JJT);
  const temp = matVecMul(inv, error);
  return matVecMul(JT, temp);
}

export function lerp(a, b, t) {
  return a + (b - a) * t;
}

export function downloadTextFile(filename, text, mime = 'application/json') {
  const blob = new Blob([text], { type: mime });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = filename;
  a.click();
  URL.revokeObjectURL(a.href);
}

export function readFileAsText(file) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve(String(reader.result));
    reader.onerror = reject;
    reader.readAsText(file);
  });
}
