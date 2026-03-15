export const DEFAULT_URDF_PATH = './urdf/robot.urdf';

export const STORAGE_KEYS = {
  lastUrdfPath: 'robot-web-gui.lastUrdfPath',
  lastTrajectory: 'robot-web-gui.lastTrajectory',
  gatewayUrl: 'robot-web-gui.gatewayUrl',
  robotId: 'robot-web-gui.robotId',
};

export const DEFAULT_ROBOTS = [
  { id: 'preview-arm', name: 'Preview Arm', mode: 'preview', ip: '-', dof: 6 },
  { id: 'left-arm', name: 'Left Arm', mode: 'wifi', ip: '192.168.1.10', dof: 6 },
  { id: 'right-arm', name: 'Right Arm', mode: 'wifi', ip: '192.168.1.11', dof: 6 },
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
  delayMs: 40,
};
