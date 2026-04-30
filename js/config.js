export const DEFAULT_URDF_PATH = './urdf/lrlita/robot.urdf';

export const STORAGE_KEYS = {
  lastUrdfPath: 'robot-web-gui.lastUrdfPath',
  lastTrajectory: 'robot-web-gui.lastTrajectory',
  gatewayUrl: 'robot-web-gui.gatewayUrl',
  robotId: 'robot-web-gui.robotId',
};

export const DEFAULT_ROBOTS = [
  { id: 'preview-arm', name: 'Preview Arm', mode: 'preview', ip: '-', dof: 6 },
  // { id: 'left-arm', name: 'Left Arm', mode: 'wifi', ip: '192.168.1.10', dof: 6 },
  // { id: 'right-arm', name: 'Right Arm', mode: 'wifi', ip: '192.168.1.11', dof: 6 },
  { id: 'arm_v1', name: 'Arm V1', mode: 'wifi', ip: '192.168.1.10', dof: 6 },
];

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
  delayMs: 10,
};

/** Teach-mode timing (see `teach.js`). */
export const TEACH = {
  /** Interval (ms) between recorded joint samples while recording a path. */
  recordSampleIntervalMs: 10,
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
    interpolation_acc_time: 1,
  },
  /**
   * Teach playback (`executeTrajectory`): waypoint index 0 uses `first`; index >= 1 uses `rest`.
   * A single-point path never reads `rest` — only `first` applies.
   */
  teach: {
    first: {
      interpolation_type: 'COS',
      interpolation_acc_time: 5,
    },
    rest: {
      interpolation_type: 'None',
      interpolation_acc_time: 0,
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
