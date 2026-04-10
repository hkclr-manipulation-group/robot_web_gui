import { downloadTextFile, readFileAsText } from './utils.js';

export function saveTrajectoryToFile(trajectory, filename = 'trajectory.json') {
  downloadTextFile(filename, JSON.stringify(trajectory, null, 2));
}

export async function loadTrajectoryFromFile(file) {
  const text = await readFileAsText(file);
  return JSON.parse(text);
}
