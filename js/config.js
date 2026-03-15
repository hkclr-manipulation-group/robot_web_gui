export const DEFAULT_URDF_PATH = './assets/urdf/simple6dof/simple6dof.urdf';

export const CONTINUOUS_RANGE = { min: -Math.PI, max: Math.PI, step: 0.001 };
export const ROTARY_FALLBACK_RANGE = { min: -Math.PI, max: Math.PI, step: 0.001 };
export const PRISMATIC_FALLBACK_RANGE = { min: -0.25, max: 0.25, step: 0.0005 };

export const IK_DEFAULTS = {
  maxIterations: 120,
  positionTolerance: 1e-3,
  orientationTolerance: 2e-2,
  damping: 0.15,
  stepScale: 0.7,
};

export const PATH_DEFAULTS = {
  steps: 50,
  delayMs: 40,
};

export const STORAGE_KEYS = {
  lastUrdfPath: 'robot_web_gui_last_urdf_path',
  lastTrajectory: 'robot_web_gui_last_trajectory',
};
