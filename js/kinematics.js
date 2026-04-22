import * as THREE from 'three';
import { IK_DEFAULTS } from './config.js';
import { dampedLeastSquares, matrixToPose, quaternionError, vectorNorm } from './utils.js';

function getChain(robot) {
  const jointsDict = robot.joints || {};
  const movable = Object.keys(jointsDict)
    .filter((name) => {
      const type = jointsDict[name]?.jointType;
      return type === 'revolute' || type === 'continuous' || type === 'prismatic';
    })
    .map((name) => ({ name, joint: jointsDict[name] }));

  if (movable.length <= 1) return movable;

  // Build a serial chain by URDF topology: parent-link -> joint -> child-link.
  const childLinks = new Set();
  const jointsByParentLink = new Map();
  movable.forEach((item) => {
    const parentLinkName = item.joint?.parent?.name;
    const childLinkName = item.joint?.children?.[0]?.name;
    if (parentLinkName) {
      if (!jointsByParentLink.has(parentLinkName)) jointsByParentLink.set(parentLinkName, []);
      jointsByParentLink.get(parentLinkName).push(item);
    }
    if (childLinkName) childLinks.add(childLinkName);
  });

  const head = movable.find((item) => {
    const parentLinkName = item.joint?.parent?.name;
    return parentLinkName && !childLinks.has(parentLinkName);
  }) || movable[0];

  const ordered = [];
  const used = new Set();
  let current = head;
  while (current && !used.has(current.name)) {
    ordered.push(current);
    used.add(current.name);
    const nextParent = current.joint?.children?.[0]?.name;
    const next = nextParent
      ? (jointsByParentLink.get(nextParent) || []).find((item) => !used.has(item.name))
      : null;
    if (!next) break;
    current = next;
  }

  if (ordered.length === movable.length) return ordered;
  return [...ordered, ...movable.filter((item) => !used.has(item.name))];
}

function getTipObject(robot, chain) {
  if (robot.links && robot.links.arm_end_effector) return robot.links.arm_end_effector;
  if (robot.links && robot.links.tool0) return robot.links.tool0;
  if (robot.links && robot.links.ee_link) return robot.links.ee_link;
  if (robot.links && robot.links.tcp) return robot.links.tcp;
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
    console.log(this.tip);
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
    this.chain.forEach((item) => {
      const j = item.joint;
      const v = Number.isFinite(j.angle)
        ? j.angle
        : (Number.isFinite(j.setVal) ? j.setVal : 0);
      out[item.name] = v;
    });
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

  getEndEffectorMatrix() {
    this.robot.updateMatrixWorld(true);
    return this.tip.matrixWorld.clone();
  }

  /** Tip origin in scene meters, matching the FK reference frame. */
  getEndEffectorPositionMeters() {
    this.robot.updateMatrixWorld(true);
    const p = new THREE.Vector3();
    this.tip.getWorldPosition(p);
    return { x: p.x, y: p.y, z: p.z };
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
