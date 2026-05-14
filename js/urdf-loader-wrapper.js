import * as THREE from "three";
import { MTLLoader } from "three/addons/loaders/MTLLoader.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import URDFLoader from "urdf-loader";

/** OBJ+MTL 网格不打硬件金属覆盖，见 applyUrdfMeshesShadowMetal / applyHardwareContrastStyle */
const PRESERVE_OBJ_MATERIAL_KEY = "preserveObjMaterial";

/** OBJ 里声明的材质库（Three OBJLoader 不会自动加载 .mtl，必须 MTLLoader + setMaterials）。 */
function extractMtllibFiles(objSource) {
  const files = [];
  const seen = new Set();
  for (const line of objSource.split(/\r?\n/)) {
    const t = line.trim();
    if (!t || t.startsWith("#")) continue;
    const m = /^mtllib\s+(.+)$/i.exec(t);
    if (!m) continue;
    for (const part of m[1].trim().split(/\s+/)) {
      if (part && !seen.has(part)) {
        seen.add(part);
        files.push(part);
      }
    }
  }
  return files;
}

function tagObjMeshesPreserved(object) {
  object?.traverse?.((child) => {
    if (child.isMesh) child.userData[PRESERVE_OBJ_MATERIAL_KEY] = true;
  });
}

/**
 * urdf-loader 默认只注册 STL / Collada；URDF 里用 .obj 时必须扩展 loadMeshCb。
 */
function loadMeshWithObjSupport(urdfLoader, path, manager, done) {
  if (/\.obj$/i.test(path)) {
    const baseUrl = THREE.LoaderUtils.extractUrlBase(path);
    const fileLoader = new THREE.FileLoader(manager);

    fileLoader.load(
      path,
      (text) => {
        const mtls = extractMtllibFiles(text);
        const objLoader = new OBJLoader(manager);

        const parseObj = (materialCreator) => {
          if (materialCreator) {
            materialCreator.preload();
            objLoader.setMaterials(materialCreator);
          }
          try {
            const object = objLoader.parse(text);
            tagObjMeshesPreserved(object);
            done(object);
          } catch (err) {
            done(null, err);
          }
        };

        if (mtls.length === 0) {
          parseObj(null);
          return;
        }

        if (mtls.length > 1) {
          console.warn(
            `[OBJ] multiple mtllib in ${path}; using ${mtls[0]}, ignoring:`,
            mtls.slice(1),
          );
        }

        const mtlLoader = new MTLLoader(manager);
        mtlLoader.setPath(baseUrl);
        mtlLoader.setResourcePath(baseUrl);

        mtlLoader.load(
          mtls[0],
          (creator) => parseObj(creator),
          undefined,
          () => {
            console.warn(`[OBJ] failed to load MTL ${mtls[0]} for ${path}`);
            parseObj(null);
          },
        );
      },
      undefined,
      (err) => done(null, err),
    );
    return;
  }
  URDFLoader.prototype.defaultMeshLoader.call(urdfLoader, path, manager, done);
}

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
    if (obj.userData[PRESERVE_OBJ_MATERIAL_KEY]) return;
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

    if (obj.userData[PRESERVE_OBJ_MATERIAL_KEY]) return;

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
  loader.loadMeshCb = (path, mgr, done) =>
    loadMeshWithObjSupport(loader, path, mgr, done);

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
