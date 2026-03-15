async function postJson(url, payload = {}) {
  const response = await fetch(url, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(payload),
  });

  if (!response.ok) {
    throw new Error(`HTTP ${response.status} ${response.statusText}`);
  }

  const text = await response.text();
  if (!text) return {};
  try {
    return JSON.parse(text);
  } catch {
    return { raw: text };
  }
}

export async function sendJointCommand(name, value) {
  return postJson("/api/joint", { joint: name, value });
}

export async function sendZeroCommand() {
  return postJson("/api/zero");
}

export async function sendHomeCommand() {
  return postJson("/api/home");
}

export async function sendStopCommand() {
  return postJson("/api/stop");
}

export async function sendPoseCommand(pose) {
  return postJson("/api/move_pose", pose);
}

export async function getCurrentPose() {
  const response = await fetch("/api/current_pose", {
    method: "GET",
  });

  if (!response.ok) {
    throw new Error(`HTTP ${response.status} ${response.statusText}`);
  }

  return response.json();
}