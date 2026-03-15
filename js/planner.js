import * as THREE from 'three';
import { lerp } from './utils.js';

export function planJointTrajectory(startMap, goalMap, steps = 50) {
  const keys = Object.keys(goalMap);
  const trajectory = [];
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const pose = {};
    keys.forEach((key) => {
      const a = startMap[key] ?? 0;
      const b = goalMap[key] ?? a;
      pose[key] = lerp(a, b, t);
    });
    trajectory.push(pose);
  }
  return trajectory;
}

function slerpPose(a, b, t) {
  const qa = new THREE.Quaternion().setFromEuler(new THREE.Euler(a.rx, a.ry, a.rz, 'XYZ'));
  const qb = new THREE.Quaternion().setFromEuler(new THREE.Euler(b.rx, b.ry, b.rz, 'XYZ'));
  const q = qa.clone().slerp(qb, t);
  const e = new THREE.Euler().setFromQuaternion(q, 'XYZ');
  return {
    x: lerp(a.x, b.x, t),
    y: lerp(a.y, b.y, t),
    z: lerp(a.z, b.z, t),
    rx: e.x,
    ry: e.y,
    rz: e.z,
  };
}

export function planCartesianTrajectory(kinematics, startPose, goalPose, steps = 30) {
  const trajectory = [];
  let qSeed = kinematics.getCurrentJointVector();
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const pose = slerpPose(startPose, goalPose, t);
    const result = kinematics.solveIK(pose, qSeed);
    qSeed = result.q;
    trajectory.push(kinematics.getJointNames().reduce((acc, name, idx) => {
      acc[name] = qSeed[idx];
      return acc;
    }, {}));
  }
  return trajectory;
}
