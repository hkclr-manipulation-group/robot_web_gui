const DEG = Math.PI / 180;

export const VALIDATION_THRESHOLDS = {
  positionMm: 2.0,
  orientationDeg: 1.0,
};

export const STANDARD_MDH = {
  alphaRad: [0, 90, 0, 0, 90, 90].map((value) => value * DEG),
  aMm: [0.0, 96.5, 346.5, 150.0, 0.0, 0.0],
  dMm: [96.2, 0.0, 0.0, 0.0, 96.5, 185.0],
  offsetRad: [0, 90, 0, 0, 90, 90].map((value) => value * DEG),
  limitsRad: {
    min: [-3.14159265, -1.57079632, -3.14159265, -3.92699, -3.14159265, -6.2831853],
    max: [3.14159265, 1.57079632, 3.14159265, 0.872664, 3.14159265, 6.2831853],
  },
};

export const DH_PRESET_CASES = [
  {
    id: "zero-pose",
    label: "Zero Pose",
    jointDeg: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  },
  {
    id: "typical-pose",
    label: "Typical Pose",
    jointDeg: [25.0, -35.0, 55.0, 15.0, 35.0, -20.0],
  },
  {
    id: "check-pose",
    label: "Check Pose",
    jointDeg: [-40.0, 30.0, -60.0, 20.0, -30.0, 45.0],
  },
];

export const FK_BATCH_CASES = [
  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  [25.0, -35.0, 55.0, 15.0, 35.0, -20.0],
  [-40.0, 30.0, -60.0, 20.0, -30.0, 45.0],
  [15.0, 20.0, -35.0, -40.0, 25.0, 60.0],
  [-75.0, -20.0, 40.0, -90.0, 10.0, -120.0],
];
