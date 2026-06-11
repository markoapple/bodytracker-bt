import assert from "node:assert/strict";
import fs from "node:fs";
import http from "node:http";
import { createRequire } from "node:module";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const repoRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const uiRoot = path.join(repoRoot, "src", "ui", "app");
let uiUrl = "";
const require = createRequire(import.meta.url);

function contentType(file) {
  if (file.endsWith(".html")) return "text/html; charset=utf-8";
  if (file.endsWith(".js")) return "text/javascript; charset=utf-8";
  if (file.endsWith(".css")) return "text/css; charset=utf-8";
  if (file.endsWith(".json")) return "application/json; charset=utf-8";
  return "application/octet-stream";
}

async function startUiServer() {
  const server = http.createServer((request, response) => {
    try {
      const url = new URL(request.url || "/", "http://127.0.0.1");
      const decoded = decodeURIComponent(url.pathname === "/" ? "/index.html" : url.pathname);
      const normalized = path.normalize(decoded).replace(/^([\\/])+/, "");
      const filePath = path.resolve(uiRoot, normalized);
      if (!filePath.startsWith(path.resolve(uiRoot)) || !fs.existsSync(filePath) || !fs.statSync(filePath).isFile()) {
        response.writeHead(404, { "content-type": "text/plain; charset=utf-8" });
        response.end("not found");
        return;
      }
      response.writeHead(200, { "content-type": contentType(filePath), "cache-control": "no-store" });
      response.end(fs.readFileSync(filePath));
    } catch (error) {
      response.writeHead(500, { "content-type": "text/plain; charset=utf-8" });
      response.end(String(error?.stack || error));
    }
  });
  await new Promise((resolve, reject) => {
    server.once("error", reject);
    server.listen(0, "127.0.0.1", resolve);
  });
  const address = server.address();
  return { server, url: `http://127.0.0.1:${address.port}/index.html` };
}
const commandIds = [
  "scanCameras",
  "enablePhoneWebCamera",
  "openPhoneWebCamera",
  "disablePhoneWebCamera",
  "openModelsFolder",
  "openCalibrationFolder",
  "createCalibrationTemplate",
  "openBuildFolder",
  "prepareDeployFolder",
  "rescanModel",
  "saveConfig",
  "startRuntime",
  "stopRuntime",
  "setCamera",
  "refreshCameraPreview",
  "calibrateFloorGeometryBackend",
  "steamVrAlignmentStart",
  "steamVrAlignmentRecord",
  "steamVrAlignmentFinish",
  "steamVrAlignmentClear"
];

const bridgeButtonIds = [
  "scanCams",
  "phoneSiteEnable",
  "phoneSiteOpen",
  "phoneSiteDisable",
  "openModels",
  "openCalib",
  "createCalib",
  "openBuild",
  "prepDeploy",
  "rescanModel",
  "applyInferenceDevice",
  "startRuntime",
  "stopRuntime",
  "saveConfig",
  "saveAdvanced",
  "refreshPreview",
  "floorManualApply",
  "wallApplySelected",
  "steamVrAlignStart",
  "steamVrAlignLeftFoot",
  "steamVrAlignRightFoot",
  "steamVrAlignPelvis",
  "steamVrAlignFloor",
  "steamVrAlignForward",
  "steamVrAlignChest",
  "steamVrAlignLeftElbow",
  "steamVrAlignRightElbow",
  "steamVrAlignLeftKnee",
  "steamVrAlignRightKnee",
  "steamVrAlignFinish",
  "steamVrAlignClear"
];

async function loadPlaywright() {
  const candidates = [
    "playwright-core",
    process.env.BT_PLAYWRIGHT_CORE,
    process.env.TEMP ? path.join(process.env.TEMP, "bt-ui-harness", "node_modules", "playwright-core") : ""
  ].filter(Boolean);
  for (const candidate of candidates) {
    try {
      return require(candidate);
    } catch (_) {}
  }
  try {
    return await import("playwright-core");
  } catch (error) {
    throw new Error(`playwright-core is required for UI sweep: ${error.message}`);
  }
}

function browserPath() {
  const candidates = [
    process.env.BT_UI_BROWSER,
    "C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe",
    "C:/Program Files/Microsoft/Edge/Application/msedge.exe",
    "C:/Program Files/BraveSoftware/Brave-Browser/Application/brave.exe",
    "C:/Program Files/Google/Chrome/Application/chrome.exe"
  ].filter(Boolean);
  const found = candidates.find((candidate) => fs.existsSync(candidate));
  if (!found) throw new Error("No Chromium browser found; set BT_UI_BROWSER to msedge.exe/brave.exe/chrome.exe");
  return found;
}

function merge(base, patch) {
  if (!patch || typeof patch !== "object") return base;
  const out = Array.isArray(base) ? base.slice() : { ...base };
  for (const [key, value] of Object.entries(patch)) {
    if (value && typeof value === "object" && !Array.isArray(value)) {
      out[key] = merge(base?.[key] || {}, value);
    } else {
      out[key] = value;
    }
  }
  return out;
}

const baseState = {
  config_path: "config/default.json",
  model: { exists: true, path: "models/rtmw-dw-x-l-cocktail14-384x288.onnx", folder: "models", active_device: "directml" },
  runtime: { running: false, last_error: null, last_exit_code: 0 },
  config: {
    inference: { device: "directml" },
    app: { log_file: "bodytracker.log", recording_dir: "recordings" },
    camera_a: { device_index: 0, source: "opencv", initial_roi_enabled: false, initial_roi: [0, 0, 1, 1] },
    camera_b: { device_index: 1, source: "opencv" },
    tracking: {
      mode: "stereo",
      model_path: "models/rtmw-dw-x-l-cocktail14-384x288.onnx",
      calibration_path: "calib/default.json",
      stale_frame_timeout_ms: 250,
      latest_frame_skew_tolerance_ms: 18,
      max_frame_skew_ms: 30,
      min_triangulated_seed_count: 3,
      max_mean_reprojection_error_px: 45,
      stereo_monocular_fallback_enabled: false,
      body_calibration: { enabled: false, auto_persist: true, required_seconds: 2.5, min_overall_confidence: 0.55, max_segment_cv: 0.12 },
      monocular: { image_width: 640, image_height: 360, horizontal_fov_deg: 70, user_height_m: 1.75, camera_height_m: 1.2, default_depth_m: 2.4, depth_confidence_scale: 0.65, min_keypoint_confidence: 0.2, min_seed_count: 8 },
      motion_consistency: { enabled: true, contact_root_correction_gain: 0.2, contact_root_max_correction_m: 0.25, contact_root_max_residual_m: 0.4, contact_root_min_support_confidence: 0.45 },
      steamvr_tracker_bridge: { enabled: true, host: "127.0.0.1", port: 39570, min_confidence: 0, send_chest: true, send_elbows: true, send_knees: false },
      osc: { enabled: false, host: "127.0.0.1", port: 9000, min_confidence: 0.2, rotations_enabled: true, trackers: { pelvis: 1, left_foot: 2, right_foot: 3, chest: 4, left_elbow: 5, right_elbow: 6, left_knee: 0, right_knee: 0 } },
      hmd: { mode: "null", json_path: "" },
      tracker_space: { valid: false, offset: [0, 0, 0], rotation: [0, 0, 0, 1], scale: 1 }
    }
  },
  calibration: { tracking_ready: true },
  cameras: { items: [{ index: 0, opened: true, backend: "opencv", width: 640, height: 360, fps: 30, preview: "" }, { index: 1, opened: true, backend: "opencv", width: 640, height: 360, fps: 30, preview: "" }, { index: 2, opened: false, backend: "opencv" }] },
  phone_site: { enabled: false, status: "disabled", url: "", apk: true, launcher_available: true, stop_available: true, updated: "mock" },
  debug: { osc: { enabled: false, status: "disabled" }, body_state: { diagnostics: {} }, tracker_roles: [], steamvr_bridge: { enabled: true, status: "idle", active_min_confidence: 0 } },
  steamvr_alignment: { provider: { status: "ready", reason: "", left_controller_tracked: true, right_controller_tracked: true }, session: { active: false }, samples: [], solve: { valid: false, status: "idle", required_samples_present: 0, required_samples_complete: false }, transform: { valid: false } },
  tracker_space: { status: "missing", reason: "no transform" }
};

function steamVrState({ active = false, accepted = [], solved = false } = {}) {
  const required = ["left_foot", "right_foot", "pelvis", "floor"];
  return {
    provider: { status: "ready", reason: "", left_controller_tracked: true, right_controller_tracked: true },
    session: { active },
    samples: accepted.map((key) => ({ landmark_key: key, landmark: key, accepted: true, controller: "left_hand", age_ms: 0 })),
    solve: { valid: solved, status: solved ? "valid" : "pending", reason: solved ? "mock solved" : "mock pending", required_samples_present: accepted.filter((key) => required.includes(key)).length, required_samples_complete: required.every((key) => accepted.includes(key)) },
    transform: { valid: solved }
  };
}

async function installBridge(page) {
  await page.addInitScript(() => {
    const listeners = [];
    const commands = [];
    const previewSvg = '<svg xmlns="http://www.w3.org/2000/svg" width="640" height="360"><rect width="640" height="360" fill="#dfddd5"/><path d="M60 260 L580 235" stroke="#ff006e" stroke-width="10"/><path d="M70 310 L600 285" stroke="#040404" stroke-width="8"/></svg>';
    function mockSteamVrState({ active = false, accepted = [], solved = false } = {}) {
      const required = ["left_foot", "right_foot", "pelvis", "floor"];
      return {
        provider: { status: "ready", reason: "", left_controller_tracked: true, right_controller_tracked: true },
        session: { active },
        samples: accepted.map((key) => ({ landmark_key: key, landmark: key, accepted: true, controller: "left_hand", age_ms: 0 })),
        solve: { valid: solved, status: solved ? "valid" : "pending", reason: solved ? "mock solved" : "mock pending", required_samples_present: accepted.filter((key) => required.includes(key)).length, required_samples_complete: required.every((key) => accepted.includes(key)) },
        transform: { valid: solved }
      };
    }
    function replyFor(message) {
      const result = { ok: true, status: "ok", status_class: "ok" };
      if (message.command === "refreshCameraPreview") {
        result.preview = "data:image/svg+xml;base64," + btoa(previewSvg);
        result.full_preview = result.preview;
        result.camera = 0;
        result.width = 640;
        result.height = 360;
      }
      if (message.command === "calibrateFloorGeometryBackend") {
        result.floor_geometry = { valid: true, backend_owned: true, source: "manual_plank_outline", floor_type: "manual_plank", image_width: 640, image_height: 360, homography_valid: true, two_axis_grid_valid: true, metric_scale_confidence: 0.8, family_a: { valid: true, metric_spacing_valid: true, spacing_m: 0.12, spacing_px: 40, confidence: 0.8 }, manual_plank: { valid: true } };
      }
      if (message.command === "steamVrAlignmentStart") result.steamvr_alignment = mockSteamVrState({ active: true });
      if (message.command === "steamVrAlignmentRecord") result.steamvr_alignment = mockSteamVrState({ active: true, accepted: [message.payload?.landmark_key || message.payload?.landmark || "left_foot"] });
      if (message.command === "steamVrAlignmentFinish") {
        result.steamvr_alignment = mockSteamVrState({ active: false, accepted: ["left_foot", "right_foot", "pelvis", "floor"], solved: true });
        result.tracker_space = { status: "valid", reason: "mock solved" };
      }
      if (message.command === "steamVrAlignmentClear") result.steamvr_alignment = mockSteamVrState({ active: false });
      return result;
    }
    const bridgeObject = { webview: {
      addEventListener(type, handler) { if (type === "message") listeners.push(handler); },
      postMessage(message) {
        commands.push(message);
        setTimeout(() => listeners.forEach((handler) => handler({ data: { type: "reply", id: message.id, ok: true, result: replyFor(message) } })), 25);
      }
    } };
    window.chrome = bridgeObject;
    window.__postedCommands = commands;
    window.__lastBtState = null;
    window.__sendState = (state) => {
      window.__lastBtState = state;
      listeners.forEach((handler) => handler({ data: { type: "state", state } }));
    };
    window.__setBridgeOnline = (online) => {
      window.chrome = online ? bridgeObject : {};
      if (typeof window.render === "function" && window.__lastBtState) window.render(window.__lastBtState);
      if (typeof window.updateActionPresentation === "function") window.updateActionPresentation();
    };
  });
}

async function assertHealthy(page, context) {
  const status = await page.locator("#commandStatus").textContent();
  assert(!status.includes("JS error"), `${context}: ${status}`);
  assert(!status.includes("Unhandled promise rejection"), `${context}: ${status}`);
  assert(!status.includes("�"), `${context}: replacement glyph leaked into status: ${status}`);
  for (const command of commandIds) assert(!status.includes(command), `${context}: raw command id leaked into status: ${status}`);
  const badButtons = await page.$$eval("button", (buttons) => buttons
    .map((button, index) => ({ index, id: button.id, text: button.textContent.trim(), title: button.title || "", state: button.dataset.state || "", disabled: button.disabled, busy: button.getAttribute("aria-busy") === "true" }))
    .filter((button) => !button.state || (button.disabled && !["disabled", "pending"].includes(button.state)) || (!button.disabled && button.state === "disabled") || (button.disabled && button.state === "disabled" && !button.title) || button.text.includes("�") || button.title.includes("�")));
  assert.deepEqual(badButtons, [], `${context}: every button needs coherent visual state, readable reason, and clean glyphs`);
  const rawLeaks = await page.$$eval("button,[title],#commandStatus", (nodes, rawCommands) => nodes
    .map((node) => ({ id: node.id || node.tagName, text: node.textContent?.trim() || "", title: node.title || "" }))
    .filter((item) => rawCommands.some((command) => item.text.includes(command) || item.title.includes(command))), commandIds);
  assert.deepEqual(rawLeaks, [], `${context}: raw command ids must not leak into visible text or titles`);
}

async function auditAllButtons(page, label, options = {}) {
  const buttons = await page.$$eval("button", (nodes) => nodes.map((button, index) => ({
    index,
    id: button.id,
    text: button.textContent.trim(),
    title: button.title || "",
    disabled: button.disabled,
    state: button.dataset.state || "",
    actionId: button.dataset.actionId || "",
    busy: button.getAttribute("aria-busy") === "true"
  })));
  assert.equal(buttons.length, 65, `${label}: expected to audit all 65 buttons`);
  for (const button of buttons) {
    assert(button.state, `${label}: ${button.id || button.text} missing data-state`);
    if (button.disabled && button.state === "disabled") {
      assert(button.title, `${label}: disabled ${button.id || button.text} needs a user-readable reason`);
    }
    assert(!button.text.includes("�") && !button.title.includes("�"), `${label}: malformed glyph on ${button.id || button.text}`);
  }
  if (options.probeDisabled) {
    for (const button of buttons.filter((item) => item.disabled)) {
      const before = await page.evaluate(() => window.__postedCommands.length);
      await page.evaluate((index) => document.querySelectorAll("button")[index].click(), button.index);
      await page.waitForTimeout(25);
      const after = await page.evaluate(() => window.__postedCommands.length);
      assert.equal(after, before, `${label}: disabled ${button.id || button.text} still posted a backend command`);
      await assertHealthy(page, `${label}:disabled:${button.id || button.text}`);
    }
  }
  return { total: buttons.length, disabledAudited: buttons.filter((button) => button.disabled).length };
}

async function assertBridgeButtonsDisabled(page, label) {
  const states = await page.$$eval("button", (nodes, ids) => Object.fromEntries(nodes
    .filter((button) => ids.includes(button.id))
    .map((button) => [button.id, { disabled: button.disabled, state: button.dataset.state || "", title: button.title || "" }])), bridgeButtonIds);
  for (const id of bridgeButtonIds) {
    assert(states[id], `${label}: missing bridge action button ${id}`);
    assert(states[id].disabled, `${label}: ${id} must be disabled while bridge is offline`);
    assert(states[id].title, `${label}: ${id} needs an offline reason`);
  }
}

async function assertDisabledButtonsDoNotLookLive(page, label) {
  const liveLooking = await page.$$eval("button.primary:disabled,button.danger:disabled", (buttons) => buttons
    .map((button) => {
      const style = getComputedStyle(button);
      return { id: button.id, text: button.textContent.trim(), color: style.color, background: style.backgroundColor, transition: style.transitionProperty };
    })
    .filter((button) => button.background === "rgb(4, 4, 4)" || button.background === "rgb(255, 75, 31)" || button.color === "rgb(255, 255, 255)"));
  assert.deepEqual(liveLooking, [], `${label}: disabled primary/danger buttons must not retain live colors`);
}

async function sendState(page, statePatch = {}) {
  await page.evaluate((state) => window.__sendState(state), merge(baseState, statePatch));
  await page.waitForTimeout(80);
}

async function clickAllEnabled(page, label) {
  const auditBefore = await auditAllButtons(page, label, { probeDisabled: true });
  const total = await page.locator("button").count();
  let clicked = 0;
  for (let i = 0; i < total; i += 1) {
    const button = page.locator("button").nth(i);
    const meta = await button.evaluate((node) => ({ disabled: node.disabled, id: node.id, text: node.textContent.trim() }));
    if (meta.disabled) continue;
    await button.scrollIntoViewIfNeeded();
    await button.click();
    clicked += 1;
    await page.waitForTimeout(70);
    await assertHealthy(page, `${label}:${meta.id || meta.text || i}`);
  }
  const auditAfter = await auditAllButtons(page, `${label}:after`, { probeDisabled: true });
  return { total, clicked, disabledAuditedBefore: auditBefore.disabledAudited, disabledAuditedAfter: auditAfter.disabledAudited };
}

async function dragPreviewLine(page, from, to) {
  const preview = page.locator("#floorAssistPreview");
  await preview.scrollIntoViewIfNeeded();
  const box = await preview.boundingBox();
  assert(box, "preview box missing");
  await page.mouse.move(box.x + from[0] * box.width, box.y + from[1] * box.height);
  await page.mouse.down();
  await page.mouse.move(box.x + to[0] * box.width, box.y + to[1] * box.height);
  await page.mouse.up();
  await page.waitForTimeout(60);
}

async function clickPreviewPoint(page, point) {
  const preview = page.locator("#floorAssistPreview");
  await preview.scrollIntoViewIfNeeded();
  const box = await preview.boundingBox();
  assert(box, "preview box missing");
  await page.mouse.click(box.x + point[0] * box.width, box.y + point[1] * box.height);
  await page.waitForTimeout(50);
}

async function exerciseDrawingStates(page) {
  await auditAllButtons(page, "preview missing", { probeDisabled: true });
  assert.equal(await page.locator("#cropDraw").evaluate((node) => node.disabled), true, "crop draw should stay disabled before preview refresh");
  assert.equal(await page.locator("#floorMarkStart").evaluate((node) => node.disabled), true, "plank drawing should stay disabled before preview refresh");
  assert.equal(await page.locator("#floorWallStart").evaluate((node) => node.disabled), true, "wall drawing should stay disabled before preview refresh");

  await page.locator("#refreshPreview").click();
  await page.waitForTimeout(120);
  await assertHealthy(page, "refresh preview");

  await page.locator("#cropDraw").click();
  await dragPreviewLine(page, [0.18, 0.20], [0.75, 0.62]);
  assert.equal(await page.locator("#cropFull").evaluate((node) => node.disabled), false, "crop full should become actionable after crop draw");
  await page.locator("#cropFull").click();

  await page.locator("#floorMarkStart").click();
  await dragPreviewLine(page, [0.12, 0.70], [0.88, 0.65]);
  assert.equal(await page.locator("#floorManualApply").evaluate((node) => node.disabled), true, "manual plank apply should stay disabled after one line");
  await dragPreviewLine(page, [0.14, 0.82], [0.90, 0.77]);
  assert.equal(await page.locator("#floorManualApply").evaluate((node) => node.disabled), true, "manual plank apply should stay disabled after two lines");
  await dragPreviewLine(page, [0.14, 0.70], [0.14, 0.82]);
  assert.equal(await page.locator("#floorManualApply").evaluate((node) => node.disabled), false, "manual plank apply should unlock after three drawn lines");
  await page.locator("#floorManualApply").click();
  await page.waitForTimeout(120);
  await assertHealthy(page, "manual plank apply");

  await page.locator("#floorWallStart").click();
  for (const point of [[0.30, 0.25], [0.55, 0.25], [0.55, 0.52]]) await clickPreviewPoint(page, point);
  assert.equal(await page.locator("#wallApplySelected").evaluate((node) => node.disabled), true, "wall sample solve should stay disabled before four corners");
  await clickPreviewPoint(page, [0.30, 0.52]);
  assert.equal(await page.locator("#wallApplySelected").evaluate((node) => node.disabled), false, "wall sample solve should unlock after four corners");
  await page.locator("#wallApplySelected").click();
  await page.waitForTimeout(120);
  await assertHealthy(page, "wall sample apply");
}

const { chromium } = await loadPlaywright();
const uiServer = await startUiServer();
uiUrl = uiServer.url;
const browser = await chromium.launch({ executablePath: browserPath(), headless: true });
const offlineBootPage = await browser.newPage({ viewport: { width: 1440, height: 1800 } });
await offlineBootPage.goto(uiUrl, { waitUntil: "load" });
await offlineBootPage.waitForTimeout(80);
assert.match(await offlineBootPage.locator("#commandStatus").textContent(), /bridge is unavailable/i, "offline boot must not report READY");
await assertBridgeButtonsDisabled(offlineBootPage, "offline boot");
await assertDisabledButtonsDoNotLookLive(offlineBootPage, "offline boot");
await assertHealthy(offlineBootPage, "offline boot");
await offlineBootPage.close();

const page = await browser.newPage({ viewport: { width: 1440, height: 1800 } });
await installBridge(page);
await page.goto(uiUrl, { waitUntil: "load" });

const results = [];
await sendState(page);
await assertHealthy(page, "stopped render");
results.push(["stopped", await clickAllEnabled(page, "stopped")]);

await sendState(page, { runtime: { running: true }, debug: { total_ms: 16.2 } });
await assertHealthy(page, "running render");
results.push(["running", await clickAllEnabled(page, "running")]);

await sendState(page, { phone_site: { enabled: true, status: "enabled", url: "https://127.0.0.1:39443/", apk: true, launcher_available: true, stop_available: true, updated: "mock" } });
await assertHealthy(page, "phone render");
results.push(["phone", await clickAllEnabled(page, "phone")]);

await sendState(page, { steamvr_alignment: steamVrState({ active: true, accepted: ["left_foot", "right_foot", "pelvis", "floor"], solved: false }) });
await assertHealthy(page, "steamvr ready render");
results.push(["steamvr", await clickAllEnabled(page, "steamvr")]);

await sendState(page, { steamvr_alignment: steamVrState({ active: true, accepted: [], solved: false }) });
await assertHealthy(page, "steamvr active no samples render");
results.push(["steamvr_active_no_samples", await clickAllEnabled(page, "steamvr_active_no_samples")]);

await sendState(page, {
  config: { tracking: { steamvr_tracker_bridge: { send_chest: false, send_elbows: false, send_knees: false }, osc: { trackers: { chest: 0, left_elbow: 0, right_elbow: 0, left_knee: 0, right_knee: 0 } } } },
  steamvr_alignment: steamVrState({ active: true, accepted: ["left_foot"], solved: false })
});
await assertHealthy(page, "steamvr optional disabled render");
results.push(["steamvr_optional_disabled", await clickAllEnabled(page, "steamvr_optional_disabled")]);

await sendState(page, { steamvr_alignment: { provider: { status: "ready", reason: "mock waiting for controllers", left_controller_tracked: false, right_controller_tracked: false }, session: { active: true }, samples: [], solve: { valid: false, status: "pending", required_samples_present: 0, required_samples_complete: false }, transform: { valid: false } } });
await assertHealthy(page, "steamvr controller missing render");
results.push(["steamvr_controller_missing", await clickAllEnabled(page, "steamvr_controller_missing")]);

await sendState(page);
await page.evaluate(() => window.__setBridgeOnline(false));
await page.waitForTimeout(80);
await assertBridgeButtonsDisabled(page, "bridge offline");
await assertDisabledButtonsDoNotLookLive(page, "bridge offline");
await auditAllButtons(page, "bridge offline", { probeDisabled: true });
await assertHealthy(page, "bridge offline render");
await page.evaluate(() => window.__setBridgeOnline(true));
await page.waitForTimeout(80);

await sendState(page);
await exerciseDrawingStates(page);
await browser.close();
await new Promise((resolve) => uiServer.server.close(resolve));

console.log(JSON.stringify({ ok: true, scenarios: results }, null, 2));
