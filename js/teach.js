export class TeachSystem {
  constructor() {
    this.poses = [];
  }

  record(jointMap) {
    this.poses.push({ ...jointMap });
  }

  clear() {
    this.poses = [];
  }

  replaceAll(poses) {
    this.poses = poses.map((p) => ({ ...p }));
  }

  getPath() {
    return this.poses.map((p) => ({ ...p }));
  }

  get count() {
    return this.poses.length;
  }
}
