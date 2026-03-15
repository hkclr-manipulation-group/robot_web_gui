import { URDFLoader } from "urdf-loader";

export async function loadRobotFromUrdf(path) {
  const loader = new URDFLoader();
  return new Promise((resolve, reject) => {
    loader.load(path, (robot) => {
      resolve(robot);
    });
  });
}
