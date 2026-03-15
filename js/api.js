async function postJson(url, payload) {
  const response = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload),
  });

  if (!response.ok) {
    throw new Error(`HTTP ${response.status}`);
  }

  return response.json().catch(() => ({}));
}

export async function sendJointCommand(joint, value) {
  return postJson('/api/joint', { joint, value });
}

export async function sendHomeCommand() {
  return postJson('/api/home', {});
}

export async function sendStopCommand() {
  return postJson('/api/stop', {});
}

export async function sendZeroCommand() {
  return postJson('/api/zero', {});
}
