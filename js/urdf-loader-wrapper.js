import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';
import URDFLoader from 'https://unpkg.com/urdf-loader@0.12.4/src/URDFLoader.js';
import { getBaseDir } from './utils.js';

export async function loadRobotFromUrdf(urdfPath) {
  return new Promise((resolve, reject) => {
    const manager = new THREE.LoadingManager();
    const stlLoader = new STLLoader(manager);
    const colladaLoader = new ColladaLoader(manager);
    const loader = new URDFLoader(manager);

    loader.packages = '';
    loader.workingPath = getBaseDir(urdfPath);

    loader.loadMeshCb = (path, managerArg, done) => {
      const lower = path.toLowerCase();

      if (lower.endsWith('.stl')) {
        stlLoader.load(
          path,
          (geometry) => {
            geometry.computeVertexNormals();
            const mesh = new THREE.Mesh(
              geometry,
              new THREE.MeshStandardMaterial({
                color: 0x9a9a9a,
                metalness: 0.12,
                roughness: 0.72,
              })
            );
            done(mesh);
          },
          undefined,
          (err) => done(null, err)
        );
        return;
      }

      if (lower.endsWith('.dae')) {
        colladaLoader.load(path, (dae) => done(dae.scene), undefined, (err) => done(null, err));
        return;
      }

      done(null, new Error(`Unsupported mesh format: ${path}`));
    };

    loader.load(urdfPath, resolve, undefined, reject);
  });
}
