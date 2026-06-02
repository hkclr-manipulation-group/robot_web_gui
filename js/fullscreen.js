/**
 * Fullscreen toggle for mobile/tablet browsers without a native fullscreen control.
 * Uses the Fullscreen API when available, with a CSS fallback otherwise.
 */

function getFullscreenElement() {
  return (
    document.fullscreenElement
    || document.webkitFullscreenElement
    || document.mozFullScreenElement
    || document.msFullscreenElement
    || null
  );
}

function requestFullscreen(el) {
  const options = { navigationUI: "hide" };

  if (el.requestFullscreen) {
    return el.requestFullscreen(options);
  }
  if (el.webkitRequestFullscreen) {
    el.webkitRequestFullscreen();
    return Promise.resolve();
  }
  if (el.mozRequestFullScreen) {
    el.mozRequestFullScreen();
    return Promise.resolve();
  }
  if (el.msRequestFullscreen) {
    el.msRequestFullscreen();
    return Promise.resolve();
  }
  return null;
}

function exitNativeFullscreen() {
  if (document.exitFullscreen) {
    return document.exitFullscreen();
  }
  if (document.webkitExitFullscreen) {
    document.webkitExitFullscreen();
    return Promise.resolve();
  }
  if (document.mozCancelFullScreen) {
    document.mozCancelFullScreen();
    return Promise.resolve();
  }
  if (document.msExitFullscreen) {
    document.msExitFullscreen();
    return Promise.resolve();
  }
  return null;
}

export function initFullscreen() {
  const btn = document.getElementById("fullscreenBtn");
  if (!btn) return;

  const root = document.documentElement;

  function isPseudoFullscreen() {
    return document.body.classList.contains("is-pseudo-fullscreen");
  }

  function isFullscreen() {
    return Boolean(getFullscreenElement()) || isPseudoFullscreen();
  }

  function enterPseudoFullscreen() {
    document.body.classList.add("is-pseudo-fullscreen");
    updateButtonState();
    window.dispatchEvent(new Event("resize"));
  }

  function exitPseudoFullscreen() {
    document.body.classList.remove("is-pseudo-fullscreen");
    updateButtonState();
    window.dispatchEvent(new Event("resize"));
  }

  function enterFullscreen() {
    const request = requestFullscreen(root);
    if (request instanceof Promise) {
      request.catch(() => enterPseudoFullscreen());
      return;
    }
    enterPseudoFullscreen();
  }

  function exitFullscreen() {
    if (isPseudoFullscreen()) {
      exitPseudoFullscreen();
      return;
    }
    const request = exitNativeFullscreen();
    if (request instanceof Promise) {
      request.catch(() => {});
    }
  }

  function updateButtonState() {
    const active = isFullscreen();
    btn.classList.toggle("is-active", active);
    btn.setAttribute("aria-pressed", active ? "true" : "false");
    btn.setAttribute("aria-label", active ? "Exit fullscreen" : "Enter fullscreen");
    btn.title = active ? "Exit fullscreen" : "Fullscreen";
  }

  btn.addEventListener("click", (event) => {
    event.stopPropagation();
    if (isFullscreen()) {
      exitFullscreen();
    } else {
      enterFullscreen();
    }
  });

  [
    "fullscreenchange",
    "webkitfullscreenchange",
    "mozfullscreenchange",
    "MSFullscreenChange",
  ].forEach((eventName) => {
    document.addEventListener(eventName, updateButtonState);
  });

  updateButtonState();
}
