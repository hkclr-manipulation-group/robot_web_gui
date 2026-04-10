export class TeachSystem {
  constructor() {
    this.poses = [];
  }

  record(jointMap) {
    this.poses.push({ ...jointMap });
  }

  replaceAll(poses) {
    this.poses = poses.map((item) => ({ ...item }));
  }

  clear() {
    this.poses = [];
  }

  getPath() {
    return this.poses.map((item) => ({ ...item }));
  }

  get count() {
    return this.poses.length;
  }
}
