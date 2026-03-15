import { lerp } from './utils.js';

export function planJointTrajectory(startMap, goalMap, steps = 50) {
  const jointNames = Object.keys(startMap);
  const out = [];
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const q = {};
    jointNames.forEach((name) => {
      q[name] = lerp(startMap[name], goalMap[name], t);
    });
    out.push(q);
  }
  return out;
}

export function planCartesianTrajectory(kinematics, startPose, goalPose, steps = 50) {
  const out = [];
  let qSeed = kinematics.getCurrentJointVector();
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const pose = {
      x: lerp(startPose.x, goalPose.x, t),
      y: lerp(startPose.y, goalPose.y, t),
      z: lerp(startPose.z, goalPose.z, t),
      rx: lerp(startPose.rx, goalPose.rx, t),
      ry: lerp(startPose.ry, goalPose.ry, t),
      rz: lerp(startPose.rz, goalPose.rz, t),
    };
    const result = kinematics.solveIK(pose, qSeed);
    qSeed = result.q;
    const map = {};
    kinematics.getJointNames().forEach((name, idx) => { map[name] = result.q[idx]; });
    out.push(map);
  }
  return out;
}
