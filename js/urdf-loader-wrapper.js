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

/**
 * CAD 导出的红漆薄片常法线朝内，FrontSide 会被剔掉。
 * 仅对偏红材质开 DoubleSide，不影响钢体/灰色 OBJ。
 */
function isRedPaintMaterial(mat) {
  if (!mat) return false;
  const name = String(mat.name || "");
  if (/red/i.test(name) || /226\s*,\s*23\s*,\s*23/.test(name)) return true;
  const c = mat.color;
  if (!c) return false;
  // 高 R、低 G/B：红漆；排除灰/钢（接近等通道）
  return c.r > 0.7 && c.g < 0.45 && c.b < 0.35 && c.r - Math.max(c.g, c.b) > 0.35;
}

function fixRedPaintStripeVisibility(object) {
  object?.traverse?.((child) => {
    if (!child?.isMesh || !child.material) return;
    const mats = Array.isArray(child.material) ? child.material : [child.material];
    mats.forEach((mat) => {
      if (!isRedPaintMaterial(mat)) return;
      mat.side = THREE.DoubleSide;
      mat.needsUpdate = true;
    });
  });
}

function tagObjMeshesPreserved(object) {
  object?.traverse?.((child) => {
    if (child.isMesh) child.userData[PRESERVE_OBJ_MATERIAL_KEY] = true;
  });
  fixRedPaintStripeVisibility(object);
}

function meshShortName(path) {
  try {
    return decodeURIComponent(String(path).split("/").pop() || path);
  } catch {
    return String(path);
  }
}

const ON_GITHUB_PAGES =
  typeof location !== "undefined" && /\.github\.io$/i.test(location.hostname);

/**
 * Pages origin is slow for multi‑MB STL; pull binaries from jsDelivr (same git repo).
 * Concurrency 2 is a balance: faster than serial, less HTTP/2 breakage than 6+.
 */
const MESH_FETCH_CONCURRENCY = ON_GITHUB_PAGES ? 2 : 4;

const meshFetchWaiters = [];
let meshFetchActive = 0;

function runMeshFetchTask(task) {
  return new Promise((resolve, reject) => {
    meshFetchWaiters.push({ task, resolve, reject });
    pumpMeshFetchQueue();
  });
}

function pumpMeshFetchQueue() {
  while (meshFetchActive < MESH_FETCH_CONCURRENCY && meshFetchWaiters.length) {
    const { task, resolve, reject } = meshFetchWaiters.shift();
    meshFetchActive += 1;
    Promise.resolve()
      .then(task)
      .then(resolve, reject)
      .finally(() => {
        meshFetchActive -= 1;
        pumpMeshFetchQueue();
      });
  }
}

function sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
}

/**
 * Project Pages URL `/<repo>/urdf/...` → jsDelivr `gh/<owner>/<repo>@main/urdf/...`
 * Localhost / non-Pages hosts keep the original relative path.
 */
function toCdnMeshUrl(path) {
  if (!ON_GITHUB_PAGES || typeof location === "undefined") return null;
  try {
    const abs = new URL(path, location.href);
    const parts = abs.pathname.split("/").filter(Boolean);
    if (parts.length < 2) return null;
    const repo = parts[0];
    const repoPath = parts.slice(1).join("/");
    const owner = location.hostname.split(".")[0];
    if (!owner || !repo || !repoPath) return null;
    return `https://cdn.jsdelivr.net/gh/${owner}/${repo}@main/${repoPath}`;
  } catch {
    return null;
  }
}

function absolutePageUrl(path) {
  if (typeof location === "undefined") return path;
  try {
    return new URL(path, location.href).href;
  } catch {
    return path;
  }
}

/**
 * fetch + retry: survives GitHub Pages HTTP/2 glitches that leave XHR hung.
 * Always settles so LoadingManager can itemEnd (avoids eternal "Loading URDF...").
 */
async function fetchBufferWithRetry(url, { short, retries = 3, timeoutMs = 90000 } = {}) {
  let lastErr;
  for (let attempt = 1; attempt <= retries; attempt++) {
    const t0 = performance.now();
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeoutMs);
    try {
      const sep = url.includes("?") ? "&" : "?";
      const reqUrl =
        attempt === 1 ? url : `${url}${sep}_urdf_retry=${attempt}&_t=${Date.now()}`;
      const res = await fetch(reqUrl, {
        signal: controller.signal,
        cache: attempt === 1 ? "default" : "reload",
      });
      if (!res.ok) {
        throw new Error(`HTTP ${res.status} ${res.statusText}`);
      }
      const buffer = await res.arrayBuffer();
      console.log(
        `[URDF][fetch] ${short} ok #${attempt} ${(performance.now() - t0).toFixed(1)}ms (${(buffer.byteLength / 1024).toFixed(1)} KB) ← ${url}`,
      );
      return buffer;
    } catch (err) {
      lastErr = err;
      console.warn(
        `[URDF][fetch] ${short} fail #${attempt}/${retries} ${(performance.now() - t0).toFixed(1)}ms ← ${url}`,
        err?.message || err,
      );
      if (attempt < retries) await sleep(300 * attempt);
    } finally {
      clearTimeout(timer);
    }
  }
  throw lastErr instanceof Error
    ? lastErr
    : new Error(String(lastErr ?? "fetch failed"));
}

/** Prefer CDN on GitHub Pages; fall back to Pages origin if CDN fails. */
async function fetchMeshBuffer(path, { short } = {}) {
  const pageUrl = absolutePageUrl(path);
  const cdnUrl = toCdnMeshUrl(path);
  const sources = cdnUrl && cdnUrl !== pageUrl ? [cdnUrl, pageUrl] : [pageUrl];

  let lastErr;
  for (let i = 0; i < sources.length; i++) {
    const url = sources[i];
    if (i === 0 && cdnUrl) {
      console.log(`[URDF][cdn] ${short} try ${url}`);
    } else if (i > 0) {
      console.warn(`[URDF][cdn] ${short} fallback → ${url}`);
    }
    try {
      return await fetchBufferWithRetry(url, { short, retries: 3, timeoutMs: 90000 });
    } catch (err) {
      lastErr = err;
    }
  }
  throw lastErr instanceof Error
    ? lastErr
    : new Error(String(lastErr ?? "mesh fetch failed"));
}

/**
 * urdf-loader 默认只注册 STL / Collada；URDF 里用 .obj 时必须扩展 loadMeshCb。
 * @param {{ inFlight?: Set<string> } | null} track optional in-flight mesh set for stall logs
 */
function loadMeshWithObjSupport(urdfLoader, path, manager, done, track = null) {
  const short = meshShortName(path);
  const t0 = performance.now();
  track?.inFlight?.add(path);
  track?.onMeshChange?.();
  console.log(`[URDF][mesh] start ${short}`);

  const finishMesh = (object, err) => {
    track?.inFlight?.delete(path);
    track?.onMeshChange?.();
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

  /** Manual LoadingManager bookkeeping (we use fetch, not FileLoader). */
  const withManagerItem = (url, promise) => {
    const resolved = manager.resolveURL(url);
    manager.itemStart(resolved);
    return promise.then(
      (value) => {
        manager.itemEnd(resolved);
        return value;
      },
      (err) => {
        manager.itemError(resolved);
        manager.itemEnd(resolved);
        throw err;
      },
    );
  };

  if (/\.obj$/i.test(path)) {
    // Prefer CDN directory so relative .mtl / textures also hit the fast host.
    const assetBase =
      toCdnMeshUrl(path)?.replace(/[^/]+$/, "") ||
      THREE.LoaderUtils.extractUrlBase(absolutePageUrl(path));

    withManagerItem(
      path,
      runMeshFetchTask(async () => {
        const buffer = await fetchMeshBuffer(path, { short });
        return new TextDecoder().decode(buffer);
      }),
    )
      .then((text) => {
        const fetchMs = (performance.now() - t0).toFixed(1);
        console.log(
          `[URDF][obj] fetched ${short} ${fetchMs}ms (${text.length} chars)`,
        );

        const mtls = extractMtllibFiles(text);
        const objLoader = new OBJLoader();

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

        // MTLLoader still uses FileLoader+manager for the .mtl (small text).
        const mtlLoader = new MTLLoader(manager);
        mtlLoader.setPath(assetBase);
        mtlLoader.setResourcePath(assetBase);
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
      })
      .catch((err) => finishMesh(null, err));
    return;
  }

  // STL via queued CDN fetch+retry (Pages origin is too slow for multi‑MB binaries).
  if (/\.stl$/i.test(path)) {
    withManagerItem(
      path,
      runMeshFetchTask(async () => {
        const buffer = await fetchMeshBuffer(path, { short });
        const parseT0 = performance.now();
        const geom = new STLLoader().parse(buffer);
        console.log(
          `[URDF][stl] ${short} parse=${(performance.now() - parseT0).toFixed(1)}ms (${(buffer.byteLength / 1024).toFixed(1)} KB)`,
        );
        return new THREE.Mesh(geom, new THREE.MeshPhongMaterial());
      }),
    )
      .then((mesh) => finishMesh(mesh))
      .catch((err) => finishMesh(null, err));
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
    // Cast onto ground only — self-receive causes shadow-acne stripes on link surfaces.
    obj.castShadow = true;
    obj.receiveShadow = false;

    if (obj.userData[PRESERVE_OBJ_MATERIAL_KEY]) return;

    const applyMetal = (mat) => {
      if (!mat) return;
      if (mat.metalness !== undefined) mat.metalness = 0.15;
    };

    if (Array.isArray(obj.material)) obj.material.forEach(applyMetal);
    else applyMetal(obj.material);
  });
}

/**
 * @param {string} url
 * @param {{ onProgress?: (info: {
 *   phase: string,
 *   loaded: number,
 *   total: number,
 *   file?: string,
 *   downloading?: string[],
 * }) => void }} [options]
 */
export async function loadRobotFromUrdf(url, { onProgress } = {}) {
  const tLoad0 = performance.now();
  let lastProgress = { loaded: 0, total: 0, url: "" };

  const emitProgress = (phase = "assets") => {
    if (!onProgress) return;
    try {
      onProgress({
        phase,
        loaded: lastProgress.loaded,
        total: lastProgress.total,
        file: lastProgress.url ? meshShortName(lastProgress.url) : "",
        downloading: [...track.inFlight].map(meshShortName),
      });
    } catch {
      /* UI callback must not break load */
    }
  };

  const track = {
    inFlight: new Set(),
    onMeshChange: () => emitProgress("assets"),
  };

  console.log(
    `[URDF] loadRobotFromUrdf start: ${url} (meshConcurrency=${MESH_FETCH_CONCURRENCY}, host=${typeof location !== "undefined" ? location.hostname : "?"}, cdnMeshes=${ON_GITHUB_PAGES})`,
  );

  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  loader.packages = url.includes("/") ? url.slice(0, url.lastIndexOf("/")) : ".";
  // Viewer does not need collision meshes; keep false so we never double-fetch heavy assets.
  loader.parseCollision = false;
  loader.loadMeshCb = (path, mgr, done) =>
    loadMeshWithObjSupport(loader, path, mgr, done, track);

  emitProgress("start");

  return await new Promise((resolve, reject) => {
    let captured = null;
    let settled = false;
    let robotCbAt = null;

    const finish = (fn, value) => {
      if (settled) return;
      settled = true;
      clearInterval(stallTimer);
      fn(value);
    };

    manager.onStart = (itemUrl, loaded, total) => {
      lastProgress = { loaded, total, url: itemUrl };
      console.log(
        `[URDF][mgr] onStart first=${meshShortName(itemUrl)} (${loaded}/${total}) +${(performance.now() - tLoad0).toFixed(0)}ms`,
      );
      emitProgress("assets");
    };

    manager.onProgress = (itemUrl, loaded, total) => {
      lastProgress = { loaded, total, url: itemUrl };
      console.log(
        `[URDF][mgr] ${loaded}/${total} ${meshShortName(itemUrl)} +${(performance.now() - tLoad0).toFixed(0)}ms`,
      );
      emitProgress("assets");
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
