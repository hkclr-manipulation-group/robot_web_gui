import * as THREE from "three";
import URDFLoader from "urdf-loader";

export async function loadRobotFromUrdf(url) {
  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  loader.packages = url.includes("/") ? url.slice(0, url.lastIndexOf("/")) : ".";

  return await new Promise((resolve, reject) => {
    loader.load(
      url,
      (robot) => {
        robot.traverse((obj) => {
          if (obj.isMesh) {
            obj.castShadow = true;
            obj.receiveShadow = true;
            if (obj.material && obj.material.metalness !== undefined) {
              obj.material.metalness = 0.15;
            }
          }
        });

        const robotRoot = new THREE.Group();
        robotRoot.name = "robot-root";

        // 统一坐标修正
        robotRoot.rotation.x = 0;

        robotRoot.add(robot);

        resolve({
          robot,
          robotRoot,
        });
      },
      undefined,
      (error) => reject(error)
    );
  });
}