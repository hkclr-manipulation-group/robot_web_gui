import { API_ENDPOINTS } from "./config.js";

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
  return postJson(API_ENDPOINTS.joint, {
    joint: name,
    value,
  });
}

export async function sendZeroCommand() {
  return postJson(API_ENDPOINTS.zero, {});
}

export async function sendHomeCommand() {
  return postJson(API_ENDPOINTS.home, {});
}

export async function sendStopCommand() {
  return postJson(API_ENDPOINTS.stop, {});
}

export async function sendPoseCommand(pose) {
  return postJson(API_ENDPOINTS.movePose, pose);
}

export async function getCurrentPose() {
  const response = await fetch(API_ENDPOINTS.getCurrentPose, {
    method: "GET",
  });

  if (!response.ok) {
    throw new Error(`HTTP ${response.status} ${response.statusText}`);
  }

  return response.json();
}