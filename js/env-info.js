import { APP_VERSION, GIT_COMMIT } from './build-info.js';

const NA = 'n/a';

function na(value) {
  if (value === undefined || value === null || value === '') return NA;
  return String(value);
}

/** Best-effort browser brand/version from classic User-Agent string. */
function parseBrowserFromUa(ua) {
  const s = String(ua || '');
  const patterns = [
    [/Edg\/([\d.]+)/, 'Edge'],
    [/OPR\/([\d.]+)/, 'Opera'],
    [/Chrome\/([\d.]+)/, 'Chrome'],
    [/Firefox\/([\d.]+)/, 'Firefox'],
    [/Version\/([\d.]+).*Safari/, 'Safari'],
    [/Safari\/([\d.]+)/, 'Safari'],
  ];
  for (const [re, name] of patterns) {
    const m = s.match(re);
    if (m) return `${name} ${m[1]}`;
  }
  return NA;
}

async function getUaHints() {
  const uad = navigator.userAgentData;
  if (!uad) return null;
  try {
    const high = await uad.getHighEntropyValues([
      'architecture',
      'bitness',
      'model',
      'platformVersion',
      'fullVersionList',
      'uaFullVersion',
    ]);
    return { brands: uad.brands, mobile: uad.mobile, platform: uad.platform, ...high };
  } catch {
    return {
      brands: uad.brands,
      mobile: uad.mobile,
      platform: uad.platform,
    };
  }
}

function formatBrowser(hints, ua) {
  if (hints?.fullVersionList?.length) {
    const list = hints.fullVersionList
      .filter((b) => b.brand && !/not.?a.?brand/i.test(b.brand))
      .map((b) => `${b.brand} ${b.version}`);
    if (list.length) return list.join(', ');
  }
  if (hints?.brands?.length) {
    const list = hints.brands
      .filter((b) => b.brand && !/not.?a.?brand/i.test(b.brand))
      .map((b) => `${b.brand} ${b.version}`);
    if (list.length) return list.join(', ');
  }
  return parseBrowserFromUa(ua);
}

function formatPlatform(hints) {
  if (hints?.platform) {
    const ver = hints.platformVersion ? ` ${hints.platformVersion}` : '';
    const arch = hints.architecture ? ` (${hints.architecture}${hints.bitness ? `/${hints.bitness}` : ''})` : '';
    return `${hints.platform}${ver}${arch}`.trim();
  }
  return na(navigator.platform);
}

function probeWebGl() {
  const canvas = document.createElement('canvas');
  let gl = null;
  let api = NA;
  try {
    gl = canvas.getContext('webgl2');
    if (gl) api = 'WebGL2';
    else {
      gl = canvas.getContext('webgl') || canvas.getContext('experimental-webgl');
      if (gl) api = 'WebGL';
    }
  } catch {
    return { version: NA, renderer: NA };
  }
  if (!gl) return { version: NA, renderer: NA };

  let version = api;
  try {
    const v = gl.getParameter(gl.VERSION);
    if (v) version = `${api} (${v})`;
  } catch {
    /* keep api label */
  }

  let renderer = NA;
  try {
    const ext = gl.getExtension('WEBGL_debug_renderer_info');
    if (ext) {
      const vendor = gl.getParameter(ext.UNMASKED_VENDOR_WEBGL);
      const gpu = gl.getParameter(ext.UNMASKED_RENDERER_WEBGL);
      renderer = [vendor, gpu].filter(Boolean).join(' / ') || NA;
    }
  } catch {
    renderer = NA;
  }

  try {
    const lose = gl.getExtension('WEBGL_lose_context');
    lose?.loseContext();
  } catch {
    /* ignore */
  }

  return { version, renderer };
}

/**
 * Collect environment diagnostics as an ordered key/value map.
 * @returns {Promise<Record<string, string>>}
 */
export async function collectEnvInfo() {
  const ua = navigator.userAgent || '';
  const hints = await getUaHints();
  const gl = probeWebGl();
  const touchPoints = Number(navigator.maxTouchPoints) || 0;
  const now = new Date();

  return {
    '浏览器及版本': formatBrowser(hints, ua),
    'User-Agent': na(ua),
    '操作系统平台': formatPlatform(hints),
    '屏幕尺寸': `${screen.width} × ${screen.height}`,
    '页面可用尺寸': `${window.innerWidth} × ${window.innerHeight}`,
    'DPR/显示缩放参考值': na(window.devicePixelRatio),
    '是否支持触摸': touchPoints > 0 ? `yes (${touchPoints})` : 'no (0)',
    'CPU逻辑线程参考值': na(navigator.hardwareConcurrency),
    '内存参考值': navigator.deviceMemory != null ? `${navigator.deviceMemory} GiB` : NA,
    'WebGL版本': gl.version,
    'GPU渲染器': gl.renderer,
    '测试日期时间': `${now.toISOString()} (local ${now.toString()})`,
    '前端版本': APP_VERSION,
    'Git Commit': GIT_COMMIT,
  };
}

/** Format env info as plain text lines for pasting into issues/chat. */
export function formatEnvInfoText(info) {
  return Object.entries(info)
    .map(([key, value]) => `${key}: ${value}`)
    .join('\n');
}

function copyViaTextarea(text) {
  const ta = document.createElement('textarea');
  ta.value = text;
  ta.setAttribute('readonly', '');
  ta.style.position = 'fixed';
  ta.style.left = '-9999px';
  ta.style.top = '0';
  document.body.appendChild(ta);
  ta.select();
  ta.setSelectionRange(0, text.length);
  const ok = document.execCommand('copy');
  document.body.removeChild(ta);
  if (!ok) throw new Error('Clipboard copy failed.');
}

/**
 * Collect env info and copy plain text to the clipboard.
 * @returns {Promise<string>} The copied text.
 */
export async function copyEnvInfoToClipboard() {
  const info = await collectEnvInfo();
  const text = formatEnvInfoText(info);

  if (navigator.clipboard?.writeText) {
    try {
      await navigator.clipboard.writeText(text);
      return text;
    } catch {
      /* fall through to execCommand */
    }
  }

  copyViaTextarea(text);
  return text;
}
