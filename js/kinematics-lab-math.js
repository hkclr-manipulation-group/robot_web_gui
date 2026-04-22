import * as THREE from "three";

export function degToRad(value) {
  return THREE.MathUtils.degToRad(Number(value) || 0);
}

export function radToDeg(value) {
  return THREE.MathUtils.radToDeg(Number(value) || 0);
}

export function mmToMeters(value) {
  return (Number(value) || 0) / 1000;
}

export function wrapAngleDeg(value) {
  const normalized = ((Number(value) || 0) + 180) % 360;
  return normalized < 0 ? normalized + 180 : normalized - 180;
}

export function round(value, digits = 3) {
  return Number(Number(value) || 0).toFixed(digits);
}

export function cloneJointVector(values) {
  return values.map((value) => Number(value) || 0);
}

export function jointVectorDegToRad(values) {
  return values.map((value) => degToRad(value));
}

export function jointVectorRadToDeg(values) {
  return values.map((value) => radToDeg(value));
}

export function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

export function poseMmDegToMatrix(pose) {
  const matrix = new THREE.Matrix4();
  const quaternion = new THREE.Quaternion().setFromEuler(
    new THREE.Euler(
      degToRad(pose.roll),
      degToRad(pose.pitch),
      degToRad(pose.yaw),
      "XYZ"
    )
  );

  matrix.compose(
    new THREE.Vector3(Number(pose.x) || 0, Number(pose.y) || 0, Number(pose.z) || 0),
    quaternion,
    new THREE.Vector3(1, 1, 1)
  );

  return matrix;
}

export function matrixToPoseMmDeg(matrix) {
  const position = new THREE.Vector3();
  const quaternion = new THREE.Quaternion();
  const scale = new THREE.Vector3();
  matrix.decompose(position, quaternion, scale);
  const euler = new THREE.Euler().setFromQuaternion(quaternion, "XYZ");

  return {
    x: position.x,
    y: position.y,
    z: position.z,
    roll: radToDeg(euler.x),
    pitch: radToDeg(euler.y),
    yaw: radToDeg(euler.z),
  };
}

export function matrixToRows(matrix) {
  const e = matrix.elements;
  return [
    [e[0], e[4], e[8], e[12]],
    [e[1], e[5], e[9], e[13]],
    [e[2], e[6], e[10], e[14]],
    [e[3], e[7], e[11], e[15]],
  ];
}

export function matrixFromFlatRowMajor(values) {
  if (!Array.isArray(values) || values.length !== 16) {
    throw new Error("矩阵输入必须包含 16 个数值。");
  }

  return new THREE.Matrix4().set(
    values[0],
    values[1],
    values[2],
    values[3],
    values[4],
    values[5],
    values[6],
    values[7],
    values[8],
    values[9],
    values[10],
    values[11],
    values[12],
    values[13],
    values[14],
    values[15]
  );
}

export function computeMdhTransform(thetaRad, alphaRad, aMm, dMm) {
  const cAlpha = Math.cos(alphaRad);
  const sAlpha = Math.sin(alphaRad);
  const cTheta = Math.cos(thetaRad);
  const sTheta = Math.sin(thetaRad);

  return new THREE.Matrix4().set(
    cTheta,
    -sTheta,
    0,
    aMm,
    sTheta * cAlpha,
    cTheta * cAlpha,
    -sAlpha,
    -sAlpha * dMm,
    sTheta * sAlpha,
    cTheta * sAlpha,
    cAlpha,
    cAlpha * dMm,
    0,
    0,
    0,
    1
  );
}

export function forwardKinematicsMDH(mdh, jointRad) {
  const transform = new THREE.Matrix4().identity();

  for (let index = 0; index < mdh.aMm.length; index += 1) {
    const theta = (jointRad[index] || 0) + (mdh.offsetRad[index] || 0);
    transform.multiply(
      computeMdhTransform(theta, mdh.alphaRad[index], mdh.aMm[index], mdh.dMm[index])
    );
  }

  return {
    matrix: transform,
    pose: matrixToPoseMmDeg(transform),
  };
}

export function poseToViewerPoint(pose) {
  return {
    x: mmToMeters(pose.x),
    y: mmToMeters(pose.y),
    z: mmToMeters(pose.z),
  };
}

export function posesToViewerPoints(poses) {
  return poses.map((pose) => poseToViewerPoint(pose));
}

export function computePoseError(studentMatrix, standardMatrix) {
  const studentPose = matrixToPoseMmDeg(studentMatrix);
  const standardPose = matrixToPoseMmDeg(standardMatrix);

  const studentQuaternion = new THREE.Quaternion();
  const standardQuaternion = new THREE.Quaternion();
  const studentPosition = new THREE.Vector3();
  const standardPosition = new THREE.Vector3();
  const scale = new THREE.Vector3();

  studentMatrix.decompose(studentPosition, studentQuaternion, scale);
  standardMatrix.decompose(standardPosition, standardQuaternion, scale);

  const dx = studentPose.x - standardPose.x;
  const dy = studentPose.y - standardPose.y;
  const dz = studentPose.z - standardPose.z;
  const dRoll = wrapAngleDeg(studentPose.roll - standardPose.roll);
  const dPitch = wrapAngleDeg(studentPose.pitch - standardPose.pitch);
  const dYaw = wrapAngleDeg(studentPose.yaw - standardPose.yaw);

  return {
    standardPose,
    studentPose,
    dx,
    dy,
    dz,
    dRoll,
    dPitch,
    dYaw,
    positionNormMm: Math.sqrt(dx * dx + dy * dy + dz * dz),
    orientationNormDeg: radToDeg(studentQuaternion.angleTo(standardQuaternion)),
  };
}

export function isPoseErrorPassing(error, thresholds) {
  return (
    error.positionNormMm < thresholds.positionMm &&
    error.orientationNormDeg < thresholds.orientationDeg
  );
}

export function parseNumericCsv(text) {
  const lines = String(text || "")
    .split(/\r?\n/)
    .map((line) => line.trim())
    .filter(Boolean);

  const rows = [];
  for (let index = 0; index < lines.length; index += 1) {
    const line = lines[index];
    const tokens = line.split(/[\s,;\t]+/).filter(Boolean);
    const numeric = tokens.map((token) => Number.parseFloat(token));
    const hasInvalid = numeric.some((value) => !Number.isFinite(value));

    if (hasInvalid) {
      if (index === 0) {
        continue;
      }
      throw new Error(`CSV 第 ${index + 1} 行包含非法数字。`);
    }

    rows.push(numeric);
  }

  if (!rows.length) {
    throw new Error("CSV 中没有可解析的数值行。");
  }

  return rows;
}

export function inferRowType(row) {
  if (!row) return null;
  if (row.length === 6) return "pose";
  if (row.length === 16) return "matrix";
  return null;
}

export function buildStudentMatrixFromRow(row) {
  const rowType = inferRowType(row);
  if (rowType === "pose") {
    return poseMmDegToMatrix({
      x: row[0],
      y: row[1],
      z: row[2],
      roll: row[3],
      pitch: row[4],
      yaw: row[5],
    });
  }

  if (rowType === "matrix") {
    return matrixFromFlatRowMajor(row);
  }

  throw new Error("CSV 每行必须是 6 列位姿或 16 列矩阵。");
}

export function randomJointVectorDeg(limitsRad) {
  return limitsRad.min.map((minValue, index) => {
    const minDeg = radToDeg(minValue);
    const maxDeg = radToDeg(limitsRad.max[index]);
    return minDeg + Math.random() * (maxDeg - minDeg);
  });
}

export function buildBatchJointSets(limitsRad, count = 10) {
  return Array.from({ length: count }, () => randomJointVectorDeg(limitsRad));
}

export function buildCircleTrajectory() {
  const points = [];
  const center = { x: 420, y: 0, z: 240 };
  const radius = 55;
  const samples = 20;

  for (let index = 0; index < samples; index += 1) {
    const angle = (Math.PI * 2 * index) / samples;
    points.push({
      x: center.x,
      y: center.y + Math.cos(angle) * radius,
      z: center.z + Math.sin(angle) * radius,
      roll: 90,
      pitch: 0,
      yaw: 0,
    });
  }

  return points;
}

export function buildMultiPointTrajectory() {
  return [
    { x: 360, y: -80, z: 220, roll: 90, pitch: 0, yaw: -45 },
    { x: 420, y: -20, z: 260, roll: 90, pitch: 10, yaw: -10 },
    { x: 460, y: 40, z: 250, roll: 90, pitch: -10, yaw: 20 },
    { x: 400, y: 90, z: 210, roll: 90, pitch: 0, yaw: 50 },
  ];
}
