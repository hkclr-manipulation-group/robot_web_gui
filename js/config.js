export const DEFAULT_ROBOTS = [
  { id: 'spark2', name: 'spark2_v2_2', mode: 'wifi', dof: 6 },
  { id: 'spark2', name: 'spark2_v2_2_with_gripper', mode: 'wifi', dof: 6, hasGripper: true },
  { id: 'spark2', name: 'spark2_v2', mode: 'wifi', dof: 6 },
  { id: 'spark2', name: 'spark2_v1', mode: 'wifi', dof: 6 },
];

export const GRIPPER = {
  jointName: 'gripper_J1',
  min: -0.03,
  max: 0.01,
  uiMin: 0,
  uiMax: 1,
  step: 0.01,
  default: 0,
  speed: 50,
  accTime: 0,
};

/** URDF path derived from robot `name` (folder under `./urdf/`). */
export function getUrdfPathForRobot(robot) {
  return `./urdf/${robot.name}/robot.urdf`;
}

export function findRobotByName(name) {
  return DEFAULT_ROBOTS.find((item) => item.name === name) || DEFAULT_ROBOTS[0];
}

export const DEFAULT_URDF_PATH = getUrdfPathForRobot(DEFAULT_ROBOTS[0]);

export const STORAGE_KEYS = {
  lastTrajectory: 'robot-web-gui.lastTrajectory',
  gatewayUrl: 'robot-web-gui.gatewayUrl',
  /** Persists selected robot `name` (URDF variant), not gateway `id`. */
  robotId: 'robot-web-gui.robotId',
};

export const CONTINUOUS_RANGE = { min: -Math.PI, max: Math.PI, step: 0.0025 };
export const ROTARY_FALLBACK_RANGE = { min: -Math.PI, max: Math.PI, step: 0.0025 };
export const PRISMATIC_FALLBACK_RANGE = { min: -0.2, max: 0.2, step: 0.0005 };

export const TASK_LIMITS = {
  x: { min: -1.2, max: 1.2, step: 0.001 },
  y: { min: -1.2, max: 1.2, step: 0.001 },
  z: { min: -0.2, max: 1.8, step: 0.001 },
  rx: { min: -180, max: 180, step: 0.5 },
  ry: { min: -180, max: 180, step: 0.5 },
  rz: { min: -180, max: 180, step: 0.5 },
};

export const IK_DEFAULTS = {
  maxIterations: 80,
  positionTolerance: 1e-3,
  orientationTolerance: 1e-2,
  damping: 0.14,
  stepScale: 0.65,
};

export const PATH_DEFAULTS = {
  steps: 50,
  /** Delay between trajectory waypoints during play (`executeTrajectory` / Play Delay UI fallback). */
  delayMs: 100,
};

/** 3D viewer display (see `main.js`, `viewer.js`). */
export const VIEWER = {
  /** Show ghost (command / IK preview) robot. When false, only the hardware arm is shown when telemetry is available. */
  showGhostRobot: true,
  /** RGB axes on the tip link (X red / Y green / Z blue). */
  showEndEffectorAxes: false,
  /**
   * When gateway URL is set, hide 3D task gizmo (TransformControls) on Joint / Teach tabs.
   * Task tab keeps gizmo; preview mode (no gateway) always shows it.
   */
  hideTaskGizmoOnJointTeachInGateway: true,
};

/** Teach-mode timing (see `teach.js`). */
export const TEACH = {
  /** Interval (ms) between UI joint samples while recording (preview only; RT records internally). */
  recordSampleIntervalMs: 100,
};

/** Slider / jog control (joint and task space). Edit here; not read from config.yaml. */
export const SLIDER_CONTROL = {
  mode: 'jog', // 'incremental' | 'jog'
  jogIntervalMs: 40,
  jogSpeed: 50,
  jogAccTime: 0,
};

/**
 * Real-time joint interpolation for the RT gateway (`/home`, `/move_joint`, teach playback).
 * Unspecified timing fields are resolved by backend defaults from robot config yaml.
 * For `interpolation_type: 'NONE'`, you do not need `interpolation_acc_time` (omitted in the request body).
 */
export const RT_INTERPOLATION = {
  home: {
    interpolation_type: 'COS',
    interpolation_acc_time: 5,
  },
  moveJoint: {
    interpolation_type: 'COS',
    interpolation_acc_time: 5,
  },
  /**
   * Legacy teach waypoint timing (unused by SDK teach/playback; kept for executeTrajectory).
   * Waypoint index 0 uses `first`; index >= 1 uses `rest`.
   */
  teach: {
    first: {
      interpolation_type: 'COS',
      interpolation_acc_time: 5,
    },
    rest: {
      interpolation_type: 'COS',
      interpolation_acc_time: 1,
    },
  },
};

function isNoneInterpolationType(profile) {
  const t = profile.interpolation_type ?? profile.interpolation;
  if (t === 4) return true;
  if (typeof t === 'string' && t.toUpperCase() === 'NONE') return true;
  return false;
}

/**
 * Body fields for `sendHomeCommand` / `sendJointCommand` (optional third argument).
 * Keeps `profile` fields as-is; omitted fields fall back to backend yaml defaults.
 * For NONE interpolation, `interpolation_acc_time` is not added unless you set it explicitly on `profile`.
 */
export function rtInterpolationPayload(profile) {
  const out = { ...profile };
  if (
    isNoneInterpolationType(profile) &&
    !Object.prototype.hasOwnProperty.call(profile, 'interpolation_acc_time')
  ) {
    delete out.interpolation_acc_time;
  }
  return out;
}
