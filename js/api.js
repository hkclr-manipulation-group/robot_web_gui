export async function sendJointCommand(jointNames, jointValues) {
  return { ok: true, mode: 'local-preview', jointNames, jointValues };
}

export async function sendPoseCommand(pose) {
  return { ok: true, mode: 'local-preview', pose };
}

export async function sendStopCommand() {
  return { ok: true, mode: 'local-preview' };
}

export async function sendHomeCommand() {
  return { ok: true, mode: 'local-preview' };
}

export async function sendZeroCommand() {
  return { ok: true, mode: 'local-preview' };
}

export async function getCurrentPose() {
  return null;
}
