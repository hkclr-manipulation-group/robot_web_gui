import * as THREE from 'three';
import { IK_DEFAULTS } from './config.js';
import { dampedLeastSquares, matrixToPose, quaternionError, vectorNorm } from './utils.js';

function getChain(robot) {
  const joints = Object.entries(robot.joints || {})
    .filter(([, joint]) => ['revolute', 'continuous', 'prismatic'].includes(joint.jointType))
    .map(([name, joint]) => ({ name, joint }));

  joints.sort((a, b) => {
    const da = a.joint.parent ? a.joint.parent.matrixWorld.elements[13] : 0;
    const db = b.joint.parent ? b.joint.parent.matrixWorld.elements[13] : 0;
    return da - db;
  });

  return joints;
}

function getTipObject(robot, chain) {
  if (robot.links && robot.links.tool0) return robot.links.tool0;
  if (robot.links && robot.links.ee_link) return robot.links.ee_link;
  if (robot.links && robot.links.tcp) return robot.links.tcp;
  const lastJoint = chain[chain.length - 1]?.joint;
  if (lastJoint?.children?.length) return lastJoint.children[0];
  let last = robot;
  robot.traverse((obj) => { if (obj.isURDFLink || obj.isObject3D) last = obj; });
  return last;
}

function applyJointVector(chain, q) {
  chain.forEach((item, index) => {
    const value = q[index];
    if (typeof item.joint.setJointValue === 'function') item.joint.setJointValue(value);
    else item.joint.angle = value;
  });
}

function getJointVector(chain) {
  return chain.map((item) => Number.isFinite(item.joint.angle) ? item.joint.angle : 0);
}

function computePoseError(currentMatrix, targetPose) {
  const currentPos = new THREE.Vector3();
  const currentQuat = new THREE.Quaternion();
  const scale = new THREE.Vector3();
  currentMatrix.decompose(currentPos, currentQuat, scale);

  const targetPos = new THREE.Vector3(targetPose.x, targetPose.y, targetPose.z);
  const targetQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(targetPose.rx, targetPose.ry, targetPose.rz, 'XYZ'));

  const posErr = targetPos.sub(currentPos);
  const rotErr = quaternionError(targetQuat, currentQuat);
  return [posErr.x, posErr.y, posErr.z, rotErr.x, rotErr.y, rotErr.z];
}

function computeNumericJacobian(robot, chain, tip, q, delta = 1e-4) {
  const basePose = tip.matrixWorld.clone();
  const basePos = new THREE.Vector3();
  const baseQuat = new THREE.Quaternion();
  const scale = new THREE.Vector3();
  basePose.decompose(basePos, baseQuat, scale);

  const cols = [];
  for (let i = 0; i < q.length; i++) {
    const qPerturbed = [...q];
    qPerturbed[i] += delta;
    applyJointVector(chain, qPerturbed);
    robot.updateMatrixWorld(true);

    const pertPos = new THREE.Vector3();
    const pertQuat = new THREE.Quaternion();
    tip.matrixWorld.decompose(pertPos, pertQuat, scale);

    const dp = pertPos.sub(basePos).multiplyScalar(1 / delta);
    const dq = quaternionError(pertQuat, baseQuat).multiplyScalar(1 / delta);
    cols.push([dp.x, dp.y, dp.z, dq.x, dq.y, dq.z]);
  }

  applyJointVector(chain, q);
  robot.updateMatrixWorld(true);
  return Array.from({ length: 6 }, (_, row) => cols.map((col) => col[row]));
}

export class RobotKinematics {
  constructor(robot) {
    this.robot = robot;
    this.chain = getChain(robot);
    this.tip = getTipObject(robot, this.chain);
    this.baseLinkName = this.chain[0]?.joint?.parent?.name || robot.name || 'base';
    this.tipLinkName = this.tip?.name || 'tip';
  }

  getJointNames() {
    return this.chain.map((item) => item.name);
  }

  getCurrentJointVector() {
    return getJointVector(this.chain);
  }

  getCurrentJointMap() {
    const out = {};
    this.chain.forEach((item) => { out[item.name] = Number.isFinite(item.joint.angle) ? item.joint.angle : 0; });
    return out;
  }

  setJointVector(q) {
    applyJointVector(this.chain, q);
    this.robot.updateMatrixWorld(true);
  }

  setJointMap(map) {
    this.chain.forEach((item) => {
      const value = map[item.name];
      if (value !== undefined) {
        if (typeof item.joint.setJointValue === 'function') item.joint.setJointValue(value);
        else item.joint.angle = value;
      }
    });
    this.robot.updateMatrixWorld(true);
  }

  getEndEffectorPose() {
    this.robot.updateMatrixWorld(true);
    return matrixToPose(this.tip.matrixWorld);
  }

  solveIK(targetPose, q0 = null, options = {}) {
    const opts = { ...IK_DEFAULTS, ...options };
    let q = q0 ? [...q0] : this.getCurrentJointVector();
    this.setJointVector(q);

    for (let i = 0; i < opts.maxIterations; i++) {
      const current = this.tip.matrixWorld.clone();
      const err = computePoseError(current, targetPose);
      const posNorm = vectorNorm(err.slice(0, 3));
      const rotNorm = vectorNorm(err.slice(3, 6));
      if (posNorm < opts.positionTolerance && rotNorm < opts.orientationTolerance) {
        return { success: true, q, iterations: i, error: err };
      }
      const J = computeNumericJacobian(this.robot, this.chain, this.tip, q);
      const dq = dampedLeastSquares(J, err, opts.damping).map((v) => v * opts.stepScale);
      q = q.map((v, idx) => v + dq[idx]);
      this.setJointVector(q);
    }

    const finalError = computePoseError(this.tip.matrixWorld.clone(), targetPose);
    return { success: false, q, iterations: opts.maxIterations, error: finalError };
  }
}
