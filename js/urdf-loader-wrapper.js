import * as THREE from "three";
import { MTLLoader } from "three/addons/loaders/MTLLoader.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
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

function meshShortName(path) {
  try {
    return decodeURIComponent(String(path).split("/").pop() || path);
  } catch {
    return String(path);
  }
}

/**
 * urdf-loader 默认只注册 STL / Collada；URDF 里用 .obj 时必须扩展 loadMeshCb。
 * @param {{ inFlight?: Set<string> } | null} track optional in-flight mesh set for stall logs
 */
function loadMeshWithObjSupport(urdfLoader, path, manager, done, track = null) {
  const short = meshShortName(path);
  const t0 = performance.now();
  track?.inFlight?.add(path);
  console.log(`[URDF][mesh] start ${short}`);

  const finishMesh = (object, err) => {
    track?.inFlight?.delete(path);
    const ms = (performance.now() - t0).toFixed(1);
    if (err) {
      console.warn(`[URDF][mesh] FAIL ${short} ${ms}ms`, err);
      done(null, err);
      return;
    }
    let tris = 0;
    object?.traverse?.((child) => {
      if (!child?.isMesh) return;
      const idx = child.geometry?.index;
      const pos = child.geometry?.attributes?.position;
      tris += idx ? idx.count / 3 : pos ? pos.count / 3 : 0;
    });
    console.log(
      `[URDF][mesh] done  ${short} ${ms}ms` +
        (tris ? ` ~${Math.round(tris)} tris` : ""),
    );
    done(object);
  };

  if (/\.obj$/i.test(path)) {
    const baseUrl = THREE.LoaderUtils.extractUrlBase(path);
    const fileLoader = new THREE.FileLoader(manager);

    fileLoader.load(
      path,
      (text) => {
        const fetchMs = (performance.now() - t0).toFixed(1);
        const bytes =
          typeof text === "string" ? text.length : text?.byteLength ?? "?";
        console.log(`[URDF][obj] fetched ${short} ${fetchMs}ms (${bytes} chars)`);

        const mtls = extractMtllibFiles(text);
        const objLoader = new OBJLoader(manager);

        const parseObj = (materialCreator) => {
          const parseT0 = performance.now();
          if (materialCreator) {
            materialCreator.preload();
            objLoader.setMaterials(materialCreator);
          }
          try {
            const object = objLoader.parse(text);
            tagObjMeshesPreserved(object);
            console.log(
              `[URDF][obj] parsed ${short} ${(performance.now() - parseT0).toFixed(1)}ms` +
                (mtls[0] ? ` (mtl=${mtls[0]})` : " (no mtl)"),
            );
            finishMesh(object);
          } catch (err) {
            finishMesh(null, err);
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
        const mtlT0 = performance.now();

        mtlLoader.load(
          mtls[0],
          (creator) => {
            console.log(
              `[URDF][obj] mtl ok ${short}/${mtls[0]} ${(performance.now() - mtlT0).toFixed(1)}ms`,
            );
            parseObj(creator);
          },
          undefined,
          () => {
            console.warn(
              `[OBJ] failed to load MTL ${mtls[0]} for ${path} after ${(performance.now() - mtlT0).toFixed(1)}ms`,
            );
            parseObj(null);
          },
        );
      },
      undefined,
      (err) => finishMesh(null, err),
    );
    return;
  }

  // STL: split network vs parse (defaultMeshLoader hides which is slow).
  if (/\.stl$/i.test(path)) {
    const fileLoader = new THREE.FileLoader(manager);
    fileLoader.setResponseType("arraybuffer");
    fileLoader.load(
      path,
      (buffer) => {
        const fetchMs = performance.now() - t0;
        const bytes = buffer?.byteLength ?? 0;
        const parseT0 = performance.now();
        try {
          const geom = new STLLoader().parse(buffer);
          const parseMs = performance.now() - parseT0;
          console.log(
            `[URDF][stl] ${short} fetch=${fetchMs.toFixed(1)}ms parse=${parseMs.toFixed(1)}ms (${(bytes / 1024).toFixed(1)} KB)`,
          );
          finishMesh(new THREE.Mesh(geom, new THREE.MeshPhongMaterial()));
        } catch (err) {
          finishMesh(null, err);
        }
      },
      undefined,
      (err) => finishMesh(null, err),
    );
    return;
  }

  URDFLoader.prototype.defaultMeshLoader.call(
    urdfLoader,
    path,
    manager,
    (object, err) => finishMesh(object, err),
  );
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
  const tLoad0 = performance.now();
  const track = { inFlight: new Set() };
  console.log(`[URDF] loadRobotFromUrdf start: ${url}`);

  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  loader.packages = url.includes("/") ? url.slice(0, url.lastIndexOf("/")) : ".";
  // Viewer does not need collision meshes; keep false so we never double-fetch heavy assets.
  loader.parseCollision = false;
  loader.loadMeshCb = (path, mgr, done) =>
    loadMeshWithObjSupport(loader, path, mgr, done, track);

  return await new Promise((resolve, reject) => {
    let captured = null;
    let settled = false;
    let robotCbAt = null;
    let lastProgress = { loaded: 0, total: 0, url: "" };

    const finish = (fn, value) => {
      if (settled) return;
      settled = true;
      clearInterval(stallTimer);
      fn(value);
    };

    manager.onStart = (itemUrl, loaded, total) => {
      console.log(
        `[URDF][mgr] onStart first=${meshShortName(itemUrl)} (${loaded}/${total}) +${(performance.now() - tLoad0).toFixed(0)}ms`,
      );
    };

    manager.onProgress = (itemUrl, loaded, total) => {
      lastProgress = { loaded, total, url: itemUrl };
      console.log(
        `[URDF][mgr] ${loaded}/${total} ${meshShortName(itemUrl)} +${(performance.now() - tLoad0).toFixed(0)}ms`,
      );
    };

    manager.onLoad = () => {
      const mgrMs = (performance.now() - tLoad0).toFixed(1);
      console.log(
        `[URDF][mgr] onLoad +${mgrMs}ms (robotCb=${robotCbAt != null ? "yes" : "NO"}, inFlightMeshes=${track.inFlight.size})`,
      );
      if (!captured) {
        finish(
          reject,
          new Error(
            `URDF: LoadingManager finished before URDF loader callback (${url}).`
          )
        );
        return;
      }
      const styleT0 = performance.now();
      applyUrdfMeshesShadowMetal(captured);
      console.log(
        `[URDF] shadow/metal style ${(performance.now() - styleT0).toFixed(1)}ms; total ${(performance.now() - tLoad0).toFixed(1)}ms`,
      );
      finish(resolve, captured);
    };

    manager.onError = (itemUrl) => {
      console.warn(
        `[URDF] dependency failed (continuing): ${itemUrl} +${(performance.now() - tLoad0).toFixed(0)}ms`,
      );
    };

    // If LoadingManager never reaches onLoad, surface what's still pending.
    const stallTimer = setInterval(() => {
      if (settled) return;
      const elapsed = ((performance.now() - tLoad0) / 1000).toFixed(1);
      const meshes = [...track.inFlight].map(meshShortName);
      console.warn(
        `[URDF] STILL LOADING ${elapsed}s — robotCb=${robotCbAt != null ? `${robotCbAt.toFixed(0)}ms` : "no"}, mgr=${lastProgress.loaded}/${lastProgress.total}, inFlightMeshes(${meshes.length}):`,
        meshes.length ? meshes : "(none — hung before/after mesh, or waiting on non-mesh item)",
      );
    }, 5000);

    loader.load(
      url,
      (robot) => {
        robotCbAt = performance.now() - tLoad0;
        const links = robot?.links ? Object.keys(robot.links).length : "?";
        const joints = robot?.joints ? Object.keys(robot.joints).length : "?";
        console.log(
          `[URDF] loader.load robot callback +${robotCbAt.toFixed(1)}ms (links=${links}, joints=${joints})`,
        );
        captured = robot;
      },
      undefined,
      (error) => {
        console.error(
          `[URDF] loader.load error +${(performance.now() - tLoad0).toFixed(1)}ms`,
          error,
        );
        finish(
          reject,
          error instanceof Error ? error : new Error(String(error)),
        );
      },
    );
  });
}
