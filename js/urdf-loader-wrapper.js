import * as THREE from 'three';
import { URDFLoader } from 'urdf-loader';

export async function loadRobotFromUrdf(url) {
  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  loader.packages = url.includes('/') ? url.slice(0, url.lastIndexOf('/')) : '.';
  return await new Promise((resolve, reject) => {
    loader.load(
      url,
      (robot) => {
        robot.traverse((obj) => {
          if (obj.isMesh) {
            obj.castShadow = true;
            obj.receiveShadow = true;
            if (obj.material && obj.material.metalness !== undefined) obj.material.metalness = 0.15;
          }
        });
        resolve(robot);
      },
      undefined,
      (error) => reject(error)
    );
  });
}
