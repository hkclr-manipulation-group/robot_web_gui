import * as THREE from "three";
import URDFLoader from "urdf-loader";

export function cloneMaterialsPerMesh(robot) {
  robot?.traverse?.((obj) => {
    if (!obj?.isMesh || !obj.material) return;
    if (Array.isArray(obj.material)) {
      obj.material = obj.material.map((m) => m?.clone?.() ?? m);
    } else if (obj.material?.clone) {
      obj.material = obj.material.clone();
    }
  });
}

function createGhostArmMaterial() {
  const orange = new THREE.Color(0xff8f4a);
  return new THREE.MeshStandardMaterial({
    color: orange,
    metalness: 0.06,
    roughness: 0.78,
    envMapIntensity: 0,
    emissive: new THREE.Color(0xd65a08),
    emissiveIntensity: 0.12,
    transparent: true,
    opacity: 0.54,
    depthWrite: false,
    polygonOffset: true,
    polygonOffsetFactor: 1,
    polygonOffsetUnits: 1,
  });
}

export function applyHardwareContrastStyle(robot) {
  const warmSteel = new THREE.Color(0xd5d0c8);
  robot?.traverse?.((obj) => {
    if (!obj?.isMesh || !obj.material) return;
    const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
    mats.forEach((mat) => {
      if (!mat?.color) return;
      mat.color.lerp(warmSteel, 0.14);
      if (mat.metalness !== undefined) mat.metalness = Math.max(mat.metalness, 0.2);
      if (mat.isMeshPhongMaterial) {
        mat.specular.setHex(0xc0b8b0);
        mat.shininess = 28;
      }
      mat.needsUpdate = true;
    });
  });
}

/**
 * 仿真臂：橙色半透明 MeshStandard。
 * 与实景金属臂区分，且不依赖环境高光。
 */
export function applyGhostVisualStyle(robot) {
  robot?.traverse?.((obj) => {
    if (!obj?.isMesh) return;

    const disposeMat = (m) => {
      if (!m) return;
      try {
        m.dispose?.();
      } catch {
        /* noop */
      }
    };

    if (Array.isArray(obj.material)) {
      obj.material.forEach(disposeMat);
      obj.material = obj.material.map(() => createGhostArmMaterial());
      return;
    }

    disposeMat(obj.material);
    obj.material = createGhostArmMaterial();
  });
}

function applyUrdfMeshesShadowMetal(robot) {
  robot?.traverse?.((obj) => {
    if (!obj?.isMesh) return;
    obj.castShadow = true;
    obj.receiveShadow = true;

    const applyMetal = (mat) => {
      if (!mat) return;
      if (mat.metalness !== undefined) mat.metalness = 0.15;
    };

    if (Array.isArray(obj.material)) obj.material.forEach(applyMetal);
    else applyMetal(obj.material);
  });
}

export async function loadRobotFromUrdf(url) {
  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  loader.packages = url.includes("/") ? url.slice(0, url.lastIndexOf("/")) : ".";

  return await new Promise((resolve, reject) => {
    let captured = null;

    manager.onLoad = () => {
      if (!captured) {
        reject(
          new Error(
            `URDF: LoadingManager finished before URDF loader callback (${url}).`
          )
        );
        return;
      }
      applyUrdfMeshesShadowMetal(captured);
      resolve(captured);
    };

    manager.onError = (itemUrl) => {
      reject(new Error(`Failed to load URDF dependency: ${itemUrl}`));
    };

    loader.load(
      url,
      (robot) => {
        captured = robot;
      },
      undefined,
      (error) => reject(error)
    );
  });
}
