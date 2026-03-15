export const DEFAULT_URDF_PATH = "./urdf/robot.urdf";

export const CONTINUOUS_RANGE = {
  min: -Math.PI,
  max: Math.PI
};

export const STORAGE_KEYS = {
  keypoints: "robot_gui_keypoints_v1",
  path: "robot_gui_path_v1",
};

export const PATH_DEFAULTS = {
  interpolationSteps: 20,
  runDelayMs: 80,
};

export const API_ENDPOINTS = {
  joint: "/api/joint",
  home: "/api/home",
  stop: "/api/stop",
  zero: "/api/zero",
  movePose: "/api/move_pose",
  getCurrentPose: "/api/current_pose"
};