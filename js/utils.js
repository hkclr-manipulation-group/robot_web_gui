export function cssSafe(str) {
  return str.replace(/[^a-zA-Z0-9_-]/g, '_');
}

export function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

export function formatJointValue(value, isPrismatic) {
  return isPrismatic ? `${value.toFixed(4)} m` : `${value.toFixed(3)} rad`;
}

export function getBaseDir(path) {
  const idx = path.lastIndexOf('/');
  return idx === -1 ? './' : path.slice(0, idx + 1);
}
