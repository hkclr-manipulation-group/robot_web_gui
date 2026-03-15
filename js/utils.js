import * as THREE from 'three';

export function cssSafe(name) {
  return String(name).replace(/[^a-zA-Z0-9_-]/g, '_');
}

export function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

export function formatJointValue(value, isPrismatic = false) {
  return isPrismatic ? `${Number(value).toFixed(4)} m` : `${THREE.MathUtils.radToDeg(Number(value)).toFixed(2)}°`;
}

export function formatJointInput(value, isPrismatic = false) {
  return isPrismatic ? Number(value).toFixed(4) : THREE.MathUtils.radToDeg(Number(value)).toFixed(2);
}

export function parseJointInput(value, isPrismatic = false) {
  const num = parseFloat(value || 0);
  return isPrismatic ? num : THREE.MathUtils.degToRad(num);
}

export function formatPoseText(pose) {
  return `x=${pose.x.toFixed(3)} y=${pose.y.toFixed(3)} z=${pose.z.toFixed(3)} r=${THREE.MathUtils.radToDeg(pose.rx).toFixed(1)} p=${THREE.MathUtils.radToDeg(pose.ry).toFixed(1)} y=${THREE.MathUtils.radToDeg(pose.rz).toFixed(1)}`;
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
