import { htmlEscape, metricCellHtml, esc, fmt } from "./js/format.js";
import { UIStore, configEventStore } from "./js/store.js";
import { webviewBridge } from "./js/bridge.js";
import {
  commandLabels, commandLabel, actionIntents, registerActionIntent,
  humanizeCommandText, actionIntentFor, actionIntentForButton,
  actionLabel, actionProgressText, buttonActionLabel, CommandBus
} from "./js/command-bus.js";

const uiStore = new UIStore();
const commandBus = new CommandBus({
  bridge: webviewBridge,
  onStatus: (message, ok = true) => setCommandStatus(message, ok),
  onBridge: (message, ok = true) => setBridge(message, ok),
  onPendingChange: (snapshot) => {
    uiStore.dispatch({ type: "PENDING_COMMANDS", payload: snapshot });
    if (current && Object.keys(el || {}).length) render(current);
  }
});

const ids = [
  "runtimePill", "bridgePill", "configPath", "overview", "modelBox", "runtimeBox",
  "phoneSiteBox", "phoneSiteUrl", "phoneSiteEnable", "phoneSiteOpen", "phoneSiteDisable",
  "trackingMode", "cameraA", "cameraB", "modeA", "modeB", "cameraStatus", "cameraRows",
  "cropToggle", "cropX", "cropY", "cropW", "cropH", "cropDraw", "cropFull", "cropStatus",
  "telemetry", "solverMetrics", "trackerRoleDetails", "inferenceDevice", "applyInferenceDevice", "modelPath", "calibrationPath", "logFile",
  "recordingDir", "replayLogPath", "staleTimeout", "latestSkew", "maxSkew", "minSeeds",
  "maxReproj", "bodyCalToggle", "bodyCalAutoPersistToggle", "bodyCalRequiredSeconds",
  "bodyCalMinConfidence", "bodyCalMaxCv", "bodyCalReadout", "roomDepthMapToggle", "roomDepthMapCollectOnlyToggle",
  "roomDepthMapFrames", "roomDepthMapStatus", "monocularImageWidth", "monocularImageHeight", "monocularFov",
  "monocularUserHeight", "monocularCameraHeight", "monocularDefaultDepth",
  "monocularDepthConfidence", "monocularMinKeypointConfidence", "monocularMinSeeds",
  "floorAssistToggle", "floorCameraSlot", "floorSpacingM", "floorSpacingPx", "floorReferenceY",
  "floorReferenceM", "floorPlankLengthM", "floorConfidence", "floorAssistStatus", "floorAssistPreview",
  "wallRectSlot", "wallRectWidthM", "wallRectHeightM", "wallRectAspect", "wallRectStatus",
  "wallSlotButtons", "wallSlot0", "wallSlot1", "wallSlot2",
          "floorManualApply", "refreshPreview", "floorMarkStart", "floorWallStart", "wallApplySelected", "wallClearSelected",
  "floorMarkClear", "floorMarkReadout", "floorMarkOverlay", "bodyPoseOverlay",
  "legacySolverToggle", "replayToggle", "motionToggle", "stereoFallbackToggle",
  "contactRootGain", "contactRootMaxCorrection", "contactRootMaxResidual",
  "contactRootMinSupport", "steamVrBridgeToggle", "steamVrBridgeHost", "steamVrBridgePort", "steamVrBridgeMinConfidence", "steamVrBridgeChestToggle", "steamVrBridgeElbowsToggle", "steamVrBridgeKneesToggle", "steamVrBridgeStatus", "oscHost",
  "oscPort", "oscMinConfidence", "pelvisTracker", "leftFootTracker",
  "rightFootTracker", "chestTracker", "leftElbowTracker", "rightElbowTracker",
  "leftKneeTracker", "rightKneeTracker", "hmdMode", "hmdPath", "trackerOffsetX", "trackerOffsetY",
  "trackerOffsetZ", "trackerRotX", "trackerRotY", "trackerRotZ", "trackerRotW",
  "trackerScale", "trackerSpaceValidToggle", "trackerSpaceStatus",
  "trackerSpaceIdentity", "trackerSpaceValidate", "trackerSpaceReadout",
  "steamVrAlignStart", "steamVrAlignLeftFoot", "steamVrAlignRightFoot", "steamVrAlignPelvis",
  "steamVrAlignFloor", "steamVrAlignForward", "steamVrAlignChest", "steamVrAlignLeftElbow",
  "steamVrAlignRightElbow", "steamVrAlignLeftKnee", "steamVrAlignRightKnee",
  "steamVrAlignFinish", "steamVrAlignClear", "steamVrAlignStatus", "steamVrAlignMetrics", "steamVrAlignSamples",
  "steamVrWizProgress", "steamVrCalibPrompt", "steamVrPromptMain", "steamVrPromptDetail",
  "oscToggle", "oscRotToggle", "scanCams", "openModels", "openCalib", "createCalib",
  "openBuild", "prepDeploy", "rescanModel", "startRuntime", "stopRuntime",
  "saveConfig", "saveAdvanced", "desktopPet", "petBubble", "petMood", "petLine",
  "petAction", "petBody", "commandStatus", "log"
];

const el = Object.fromEntries(ids.map((id) => [id, document.querySelector(`#${id}`)]));
let current = uiStore.getState().backend || {};
let syncInputs = true;
let localDraftDirty = false;
let metricHistory = uiStore.getState().metricHistory || {};
let floorMarking = false;
let floorMarkMode = "plank";
let floorLineStart = null;
let floorLineEnd = null;
let floorMarks = [];
const wallRectangleSlotCount = 3;
let activeWallSlot = 0;
let wallRectangles = Array.from({ length: wallRectangleSlotCount }, (_, index) => ({
  index,
  corners: [],
  geometry: null,
  status: "empty"
}));
let wallCorners = wallRectangles[0].corners;
let cropDrawing = false;
let cropStart = null;
let cropEnd = null;
let cropAdjust = null;
let floorMarkImageWidth = 0;
let floorMarkImageHeight = 0;
let floorGeometryAuto = null;
let floorGeometryDraft = null;
let floorGeometryByCamera = { camera_a: null, camera_b: null };
let wallRectanglesByCamera = { camera_a: null, camera_b: null };
let activeWallCameraKey = activeFloorCameraKey();
let floorGeometryCleared = false;
let floorCalibrationRequestSeq = 0;
const logLines = [];
const transientTimers = new WeakMap();

function activeFloorCameraKey() {
  return el.floorCameraSlot?.value === "camera_b" ? "camera_b" : "camera_a";
}

function activeFloorCameraLabel() {
  return activeFloorCameraKey() === "camera_b" ? "Camera B" : "Camera A";
}

function cameraConfigForFloorSlot(key = activeFloorCameraKey()) {
  const config = key === "camera_b" ? (current.config?.camera_b || {}) : (current.config?.camera_a || {});
  const selectedIndex = Number(config.device_index ?? (key === "camera_b" ? el.cameraB?.value : el.cameraA?.value));
  const listed = (current.cameras?.items || []).find((camera) => Number(camera.index) === selectedIndex);
  return listed || config || {};
}

function cameraIndexForFloorSlot(key = activeFloorCameraKey()) {
  const cfg = cameraConfigForFloorSlot(key);
  return Number(cfg?.device_index ?? (key === "camera_b" ? el.cameraB?.value : el.cameraA?.value));
}

function activeFloorGeometry() {
  const key = activeFloorCameraKey();
  return floorGeometryByCamera[key] || (key === "camera_a" ? floorGeometryAuto : null);
}

function setActiveFloorGeometry(geometry) {
  const key = activeFloorCameraKey();
  floorGeometryByCamera[key] = geometry || null;
  if (key === "camera_a") floorGeometryAuto = geometry || null;
}

function emptyWallSlots() {
  return Array.from({ length: wallRectangleSlotCount }, (_, index) => ({
    index,
    corners: [],
    geometry: null,
    status: "empty"
  }));
}

function cloneWallSlots(slots) {
  const cloned = emptyWallSlots();
  for (const slot of slots || []) {
    const index = Math.max(0, Math.min(wallRectangleSlotCount - 1, Number(slot?.index) || 0));
    cloned[index] = {
      index,
      corners: Array.isArray(slot?.corners) ? slot.corners.map((p) => ({ ...p })) : [],
      geometry: slot?.geometry ? JSON.parse(JSON.stringify(slot.geometry)) : null,
      status: slot?.status || "empty"
    };
  }
  return cloned;
}

function saveActiveWallSlots() {
  const key = activeWallCameraKey || activeFloorCameraKey();
  wallRectanglesByCamera[key] = cloneWallSlots(wallRectangles);
}

function loadActiveWallSlots() {
  ensureWallRectangles();
  const key = activeFloorCameraKey();
  const saved = wallRectanglesByCamera[key];
  wallRectangles = saved ? cloneWallSlots(saved) : emptyWallSlots();
  activeWallCameraKey = key;
  activeWallSlot = Math.max(0, Math.min(wallRectangleSlotCount - 1, activeWallSlot));
  wallCorners = wallRectangles[activeWallSlot].corners;
}

// SteamVR calibration wizard state. The backend's machine keys are the state
// owner; display labels and button state are derived from these keys every render.
const steamVrAlignmentLandmarks = [
  { key: "left_foot",   required: true,  buttonId: "steamVrAlignLeftFoot",   label: "Left ankle",   short: "L ankle", icon: "[L]", main: "Hold controller against your LEFT ankle",        detail: "Use either hand. Press the controller firmly against the outside of your left ankle bone, then click the button or press trigger.",      hand: "EITHER HAND -> LEFT ANKLE" },
  { key: "right_foot",  required: true,  buttonId: "steamVrAlignRightFoot",  label: "Right ankle",  short: "R ankle", icon: "[R]", main: "Hold controller against your RIGHT ankle",       detail: "Use either hand. Press the controller firmly against the outside of your right ankle bone, then click the button or press trigger.",     hand: "EITHER HAND -> RIGHT ANKLE" },
  { key: "pelvis",      required: true,  buttonId: "steamVrAlignPelvis",     label: "Pelvis",       short: "Pelvis",  icon: "[P]", main: "Hold controller at your pelvis / hip center",    detail: "Place the controller against your belly button or belt line, centered. Use either hand.",                                                hand: "EITHER HAND -> HIP CENTER" },
  { key: "floor",       required: true,  buttonId: "steamVrAlignFloor",      label: "Floor",        short: "Floor",   icon: "[_]", main: "Place controller flat on the FLOOR",             detail: "Required vertical scope: set the controller down on the floor between your feet. This uses SteamVR floor height even when stereo floor calibration is missing.", hand: "SET DOWN ON FLOOR" },
  { key: "forward",     required: false, buttonId: "steamVrAlignForward",    label: "Forward",      short: "Forward", icon: "[^]", main: "Point controller FORWARD toward your cameras",   detail: "Optional yaw check: hold a controller at arm's length in front of you, pointing straight toward your camera setup.",                    hand: "OPTIONAL -> ARM'S LENGTH FORWARD" },
  { key: "chest",       required: false, buttonId: "steamVrAlignChest",      label: "Chest",        short: "Chest",   icon: "[C]", main: "Hold controller against your upper chest",        detail: "Optional upper-body offset: hold the controller against your sternum/chest tracker location.",                                           hand: "OPTIONAL -> CHEST" },
  { key: "left_elbow",  required: false, buttonId: "steamVrAlignLeftElbow",  label: "Left elbow",   short: "L elbow", icon: "[E]", main: "Hold controller against your LEFT elbow",        detail: "Optional upper-body offset: touch the controller to the left elbow tracker landmark.",                                                   hand: "OPTIONAL -> LEFT ELBOW" },
  { key: "right_elbow", required: false, buttonId: "steamVrAlignRightElbow", label: "Right elbow",  short: "R elbow", icon: "[E]", main: "Hold controller against your RIGHT elbow",       detail: "Optional upper-body offset: touch the controller to the right elbow tracker landmark.",                                                  hand: "OPTIONAL -> RIGHT ELBOW" },
  { key: "left_knee",   required: false, buttonId: "steamVrAlignLeftKnee",   label: "Left knee",    short: "L knee",  icon: "[K]", main: "Hold controller against your LEFT knee",         detail: "Optional lower-body offset only when knee trackers are mapped in OSC.",                                                                  hand: "OPTIONAL -> LEFT KNEE" },
  { key: "right_knee",  required: false, buttonId: "steamVrAlignRightKnee",  label: "Right knee",   short: "R knee",  icon: "[K]", main: "Hold controller against your RIGHT knee",        detail: "Optional lower-body offset only when knee trackers are mapped in OSC.",                                                                  hand: "OPTIONAL -> RIGHT KNEE" }
];

const wizardLandmarks = [
  { key: "start", landmark: null, icon: "[+]", main: "Press \"Start calibration\" to begin", detail: "Stand in your play space facing your cameras. Left ankle, right ankle, pelvis, and floor unlock Solve. Upper-body samples refine role offsets.", hand: null },
  ...steamVrAlignmentLandmarks.map((step) => ({ ...step, landmark: step.key })),
  { key: "solve", landmark: null, icon: "[ok]", main: "Required samples captured - solve now, or capture optional role offsets first", detail: "Solve computes the camera-to-VR transform. Required body+floor samples solve tracker-space. Optional forward/chest/elbow/knee samples refine yaw and tracker attachment offsets.", hand: null }
];
let wizardStep = 0;
let steamVrTriggerRecordPending = false;
let petAction = null;
let petDrag = null;

const steamVrLandmarkByKey = new Map(steamVrAlignmentLandmarks.map((item) => [item.key, item]));
const steamVrLandmarkButtons = Object.fromEntries(steamVrAlignmentLandmarks.map((item) => [item.key, item.buttonId]));
const steamVrRequiredLandmarkKeys = steamVrAlignmentLandmarks.filter((item) => item.required).map((item) => item.key);
function commandOk(reply) {
  return !!reply && reply.ok !== false && reply.result?.ok !== false;
}

function baseButtonLabel(button) {
  if (!button) return "";
  if (!button.dataset.baseLabel) button.dataset.baseLabel = button.textContent;
  return button.dataset.baseLabel;
}

function buttonIsBusy(button) {
  return button?.dataset?.busy === "true";
}

function flashNode(node, className) {
  if (!node) return;
  node.classList.remove(className);
  // Force restart so two fast clicks still visibly acknowledge the action.
  void node.offsetWidth;
  node.classList.add(className);
}

function setButtonBusy(button, label = "Working") {
  if (!button) return;
  label = buttonActionLabel(label);
  window.clearTimeout(transientTimers.get(button));
  button.dataset.busy = "true";
  button.dataset.feedback = "busy";
  button.dataset.state = "pending";
  button.dataset.statusText = label;
  button.textContent = baseButtonLabel(button);
  button.setAttribute("aria-busy", "true");
  button.title = `${label} pending`;
}

function setButtonResult(button, ok, text = "done") {
  if (!button) return;
  text = humanizeCommandText(text);
  const base = baseButtonLabel(button);
  button.dataset.busy = "false";
  button.dataset.feedback = ok ? "ok" : "fail";
  button.removeAttribute("aria-busy");
  button.dataset.statusText = ok ? "done" : "failed";
  button.textContent = base;
  button.title = text;
  flashNode(button, "tap-flash");
  const timer = window.setTimeout(() => {
    button.dataset.feedback = "";
    button.textContent = base;
    button.title = button.dataset.actionReason || "";
    updateActionPresentation();
    renderSteamVrAlignment();
  }, ok ? 900 : 1500);
  transientTimers.set(button, timer);
}

async function withButtonFeedback(button, pendingText, work) {
  if (buttonIsBusy(button)) return { ok: false, error: "already pending" };
  setButtonBusy(button, pendingText);
  try {
    const reply = await work();
    setButtonResult(button, commandOk(reply), commandOk(reply) ? "done" : (reply?.error || reply?.result?.status || "failed"));
    return reply;
  } catch (error) {
    setButtonResult(button, false, error?.message || "failed");
    throw error;
  }
}

function markControlTouched(node) {
  if (!node) return;
  node.dataset.touched = "true";
  flashNode(node, "tap-flash");
  window.setTimeout(() => { node.dataset.touched = ""; }, 750);
}



function setBridge(text, ok = true) {
  if (!el.bridgePill) return;
  el.bridgePill.textContent = `BRIDGE: ${text}`;
  el.bridgePill.className = ok ? "good" : "bad";
}

function appendLog(message) {
  const line = `[${new Date().toLocaleTimeString()}] ${message}`;
  logLines.unshift(line);
  while (logLines.length > 80) logLines.pop();
  if (el.log) {
    const stateLines = [
      current.runtime?.last_error,
      current.debug?.last_error,
      current.calibration?.summary
    ].filter(Boolean);
    el.log.textContent = [...logLines, ...stateLines].join("\n");
  }
}

function setCommandStatus(message, ok = true) {
  if (!el.commandStatus) return;
  message = humanizeCommandText(message);
  const state = ok === "warn" ? "warn" : (ok ? "ok" : "fail");
  el.commandStatus.textContent = message;
  el.commandStatus.className = `command-status ${state}`;
  el.commandStatus.dataset.state = state;
  flashNode(el.commandStatus, "status-pulse");
  appendLog(message);
  renderPet(current);
}

function checked(id) {
  return el[id]?.getAttribute("aria-pressed") === "true";
}

function setPressed(id, value) {
  if (el[id]) el[id].setAttribute("aria-pressed", String(!!value));
}

function ensureWallRectangles() {
  if (!Array.isArray(wallRectangles)) wallRectangles = [];
  while (wallRectangles.length < wallRectangleSlotCount) {
    wallRectangles.push({ index: wallRectangles.length, corners: [], geometry: null, status: "empty" });
  }
  wallRectangles = wallRectangles.slice(0, wallRectangleSlotCount).map((slot, index) => ({
    index,
    corners: Array.isArray(slot?.corners) ? slot.corners : [],
    geometry: slot?.geometry || null,
    status: slot?.status || "empty"
  }));
  activeWallSlot = Math.max(0, Math.min(wallRectangleSlotCount - 1, Number(activeWallSlot) || 0));
  wallCorners = wallRectangles[activeWallSlot].corners;
}

function wallSlotLabel(index = activeWallSlot) {
  return `Sample ${Number(index) + 1}`;
}

function currentWallRectangle() {
  ensureWallRectangles();
  return wallRectangles[activeWallSlot];
}

function setActiveWallSlot(index) {
  ensureWallRectangles();
  activeWallSlot = Math.max(0, Math.min(wallRectangleSlotCount - 1, Number(index) || 0));
  wallCorners = wallRectangles[activeWallSlot].corners;
  if (el.wallRectSlot) el.wallRectSlot.value = String(activeWallSlot);
  updateWallRectangleStatus();
  updateFloorMarkOverlay();
}

function clearWallRectangle(index = activeWallSlot) {
  ensureWallRectangles();
  const slot = wallRectangles[Math.max(0, Math.min(wallRectangleSlotCount - 1, Number(index) || 0))];
  slot.corners = [];
  slot.geometry = null;
  slot.status = "empty";
  if (slot.index === activeWallSlot) wallCorners = slot.corners;
  updateWallRectangleStatus();
  updateFloorMarkOverlay();
}

function clearAllWallRectangles() {
  ensureWallRectangles();
  for (const slot of wallRectangles) {
    slot.corners = [];
    slot.geometry = null;
    slot.status = "empty";
  }
  wallCorners = wallRectangles[activeWallSlot].corners;
  updateWallRectangleStatus();
}

function pointFromSavedWallCorner(corner) {
  if (Array.isArray(corner) && corner.length >= 2) {
    return { x: Number(corner[0]), y: Number(corner[1]) };
  }
  if (corner && typeof corner === "object") {
    return { x: Number(corner.x), y: Number(corner.y) };
  }
  return null;
}

function wallCornersFromGeometry(geometry) {
  const raw = Array.isArray(geometry?.image_corners) ? geometry.image_corners : [];
  return raw.map(pointFromSavedWallCorner).filter((p) => Number.isFinite(p?.x) && Number.isFinite(p?.y)).slice(0, 4);
}

function wallSlotElements() {
  return [el.wallSlot0, el.wallSlot1, el.wallSlot2].filter(Boolean);
}

function wallSlotIndexFromGeometry(geometry) {
  const source = String(geometry?.source || "");
  const match = source.match(/wall_rectangle_(\d+)/);
  if (!match) return null;
  const index = Number(match[1]) - 1;
  return Number.isInteger(index) && index >= 0 && index < wallRectangleSlotCount ? index : null;
}

function wallSampleHasDepth(geometry) {
  return !!(geometry?.usable_for_depth_assist && geometry?.wall_depth_valid &&
    Number.isFinite(Number(geometry?.wall_center_depth_m)) && Number(geometry?.wall_center_depth_m) > 0);
}

function wallSampleHasOrientation(geometry) {
  return !!(geometry?.usable_for_orientation && geometry?.wall_orientation_valid &&
    Number(geometry?.wall_orientation_confidence || 0) > 0);
}

function wallSampleCapabilityLabel(geometry) {
  const depth = wallSampleHasDepth(geometry);
  const orientation = wallSampleHasOrientation(geometry);
  if (depth && orientation) return "depth + angle";
  if (depth) return "depth";
  if (orientation) return "angle";
  if (geometry?.metric_scale_valid || geometry?.usable_for_metric_scale) return "scale";
  if (geometry?.valid) return "candidate";
  return "";
}

function wallCandidateSummary(key) {
  const slots = wallRectanglesByCamera[key] || [];
  const candidates = slots.filter((s) => s.geometry?.valid).length;
  const depth = slots.filter((s) => wallSampleHasDepth(s.geometry)).length;
  const orientation = slots.filter((s) => wallSampleHasOrientation(s.geometry)).length;
  const parts = [];
  if (candidates > 0) parts.push(`${candidates} candidate${candidates === 1 ? "" : "s"}`);
  if (depth > 0) parts.push(`${depth} depth`);
  if (orientation > 0) parts.push(`${orientation} angle`);
  return parts.length ? parts.join(" / ") : "no samples";
}

function updateWallRectangleStatus() {
  ensureWallRectangles();
  for (const button of wallSlotElements()) {
    const index = Number(button?.dataset?.wallSlot ?? -1);
    const slot = wallRectangles[index];
    if (!button || !slot) continue;
    const count = slot.corners.length;
    const solved = !!slot.geometry?.valid;
    const drawn = count >= 4 && !solved;
    const capability = wallSampleCapabilityLabel(slot.geometry);
    button.classList.toggle("active", index === activeWallSlot);
    button.classList.toggle("solved", solved);
    button.classList.toggle("drawn", drawn);
    button.classList.toggle("has-depth", wallSampleHasDepth(slot.geometry));
    button.classList.toggle("has-angle", wallSampleHasOrientation(slot.geometry));
    const label = button.querySelector("span");
    if (label) {
      if (solved) {
        label.textContent = capability || "candidate";
      } else if (drawn) {
        label.textContent = "ready to solve";
      } else if (count > 0) {
        label.textContent = `${count}/4 corners`;
      } else {
        label.textContent = index === 0 ? "start here" : "optional";
      }
    }
  }
  if (!el.wallRectStatus) {
    updateActionPresentation();
    return;
  }
  const slot = currentWallRectangle();
  const done = slot.corners.length >= 4;
  const solved = !!slot.geometry?.valid;
  const width = Number(el.wallRectWidthM?.value || 0);
  const height = Number(el.wallRectHeightM?.value || 0);
  const aspect = Number(el.wallRectAspect?.value || 0);
  const dims = width > 0 || height > 0 || aspect > 0
    ? ` · size hint ${width > 0 ? fmt(width, 3) + "m W" : "auto W"} / ${height > 0 ? fmt(height, 3) + "m H" : "auto H"}${aspect > 0 ? " · aspect " + fmt(aspect, 3) : ""}`
    : "";
  const capability = wallSampleCapabilityLabel(slot.geometry);
  const state = done ? (solved ? (capability || "candidate") : "ready to solve") : `${slot.corners.length}/4 corners`;
  const stateClass = solved ? "good" : (done ? "warn" : "muted");
  el.wallRectStatus.innerHTML = `${esc(activeFloorCameraLabel())} ${esc(wallSlotLabel())}: <b class="${stateClass}">${esc(state)}</b>${dims} <span class="wall-bank">A ${esc(wallCandidateSummary("camera_a"))} · B ${esc(wallCandidateSummary("camera_b"))}</span>`;
  updateActionPresentation();
}
function controlLabel(node) {
  if (!node) return "control";
  const explicit = node.querySelector?.(":scope > span")?.textContent?.trim();
  if (explicit) return explicit;
  const labelled = node.getAttribute?.("aria-label") || node.title;
  if (labelled) return labelled.trim();
  return baseButtonLabel(node).replace(/\s+/g, " ").trim();
}

function resetFloorMarks(clearGeometry = false) {
  floorCalibrationRequestSeq++;
  floorMarking = false;
  floorMarkMode = "plank";
  floorLineStart = null;
  floorLineEnd = null;
  floorMarks = [];
  floorMarkImageWidth = 0;
  floorMarkImageHeight = 0;
  el.floorAssistPreview?.classList.remove("is-marking", "line-started");
  if (clearGeometry) {
    floorGeometryAuto = null;
    floorGeometryDraft = null;
    floorGeometryCleared = true;
    uiStore.dispatch({ type: "FLOOR_GEOMETRY", patch: { auto: null, draft: null, cleared: true } });
    configEventStore.append("FLOOR_GEOMETRY_CLEARED", {});
  }
  updateFloorMarkOverlay();
  updateFloorMarkReadout();
}


function editingFormControl() {
  const active = document.activeElement;
  return !!active?.matches?.("input,select,textarea");
}

function markDraftDirty() {
  const firstDirty = !localDraftDirty;
  localDraftDirty = true;
  syncInputs = false;
  uiStore.dispatch({ type: "DRAFT_DIRTY" });
  if (firstDirty) {
    configEventStore.append("CONFIG_CHANGED", { dirty: true });
  }
  document.body.classList.add("draft-dirty");
  updateActionPresentation();
  if (firstDirty) {
    setCommandStatus("Draft changed · save config to apply");
  }
}

function clearDraftDirty() {
  localDraftDirty = false;
  syncInputs = !editingFormControl();
  uiStore.dispatch({ type: "DRAFT_CLEAN" });
  configEventStore.append("CONFIG_SAVED", { dirty: false });
  document.body.classList.remove("draft-dirty");
  updateActionPresentation();
}

function toggle(id) {
  setPressed(id, !checked(id));
}

function visibleActionButton(id) {
  return el[id] && !el[id].disabled ? el[id] : null;
}

function scrollToNode(node) {
  if (!node) return false;
  node.scrollIntoView({ behavior: "smooth", block: "center" });
  flashNode(node, "tap-flash");
  return true;
}

function petHint(state) {
  const modelOk = !!state.model?.exists;
  const running = !!state.runtime?.running;
  const mode = trackingMode();
  const cameraNeed = mode === "monocular" ? 1 : 2;
  const cameraOpen = Number(state.cameras?.open_count || 0);
  const setupOk = mode === "monocular" ? true : !!state.calibration?.tracking_ready;
  const osc = state.debug?.osc || {};
  const oscEnabled = osc.enabled ?? state.config?.osc?.enabled;
  const tracker = state.tracker_space || {};
  const bodyDiag = state.debug?.body_state?.diagnostics || {};
  const totalMs = state.debug?.total_ms ?? state.debug?.pipeline_ms;

  if (!webviewAvailable()) {
    return { mood: "OFFLINE", line: "Bridge offline. UI cannot command backend.", action: "Log", target: "log", tone: "bad" };
  }
  if (localDraftDirty) {
    return { mood: "SAVE", line: "Draft changed. Save before runtime trust.", action: "Save", target: "saveConfig", tone: "warn" };
  }
  if (!modelOk) {
    return { mood: "MODEL", line: "ONNX missing. Put model in folder, rescan.", action: "Models", target: "openModels", tone: "bad" };
  }
  if (cameraOpen < cameraNeed) {
    return { mood: "CAM", line: `${cameraOpen}/${cameraNeed} cameras open. Scan signals.`, action: "Scan", target: "scanCams", tone: "warn" };
  }
  if (!setupOk) {
    return { mood: "CALIB", line: "Stereo calibration not ready. Check calib.", action: "Calib", target: "openCalib", tone: "warn" };
  }
  if (!running) {
    return { mood: "READY", line: "Setup looks usable. Start runtime.", action: "Start", target: "startRuntime", tone: "good" };
  }
  if (oscEnabled && tracker.osc_blocked) {
    return { mood: "OSC", line: `OSC blocked: ${tracker.reason || tracker.status || "tracker space"}.`, action: "Align", target: "trackerSpaceStatus", tone: "warn" };
  }
  if (bodyDiag.degraded || state.debug?.degradation_mode) {
    return { mood: "WATCH", line: `Tracking degraded: ${state.debug?.degradation_mode || bodyDiag.reason || "check metrics"}.`, action: "Metrics", target: "solverMetrics", tone: "warn" };
  }
  return {
    mood: "GOOD",
    line: `Tracking live${Number.isFinite(Number(totalMs)) ? ` · ${fmt(totalMs, 1)} ms` : ""}.`,
    action: "Telemetry",
    target: "telemetry",
    tone: "good"
  };
}

function runPetAction() {
  if (!petAction) return;
  const button = visibleActionButton(petAction.target);
  if (button) {
    button.click();
    return;
  }
  if (scrollToNode(el[petAction.target])) return;
  if (petAction.target === "log") scrollToNode(el.log);
}

function renderPet(state = current) {
  if (!el.desktopPet || !el.petMood || !el.petLine || !el.petAction) return;
  const hint = petHint(state || {});
  petAction = hint;
  el.desktopPet.dataset.mood = hint.tone;
  el.petMood.textContent = hint.mood;
  el.petLine.textContent = hint.line;
  el.petAction.textContent = hint.action;
  el.petAction.disabled = !hint.target;
}

function setValue(id, value) {
  const node = el[id];
  if (!node || document.activeElement === node) return;
  node.value = value ?? "";
}

function setDisabled(id, value) {
  if (el[id]) el[id].disabled = !!value;
}

function pendingCommandsSnapshot() {
  return uiStore.getState().pendingCommands || {};
}

function commandPending(...commands) {
  const wanted = new Set(commands.flat().filter(Boolean));
  if (!wanted.size) return Object.keys(pendingCommandsSnapshot()).length > 0;
  return Object.values(pendingCommandsSnapshot()).some((entry) => wanted.has(entry?.command));
}

function normalizeButtonVisualStates(root = document) {
  for (const button of root.querySelectorAll?.("button") || []) {
    const busy = buttonIsBusy(button) || button.getAttribute("aria-busy") === "true";
    const pressed = button.getAttribute("aria-pressed");
    const done = button.classList.contains("is-done");
    const warn = button.classList.contains("is-warn");
    const active = button.classList.contains("is-active") || (button.classList.contains("switch") && pressed === "true");
    const state = busy
      ? "pending"
      : (button.disabled
        ? "disabled"
        : (done ? "done" : (warn ? "warn" : (active ? "active" : "ready"))));
    button.dataset.state = state;
    if (!button.dataset.baseLabel) button.dataset.baseLabel = button.textContent;
  }
}

function setActionControl(buttonOrId, options = {}) {
  const button = typeof buttonOrId === "string" ? el[buttonOrId] : buttonOrId;
  if (!button) return;
  const {
    enabled = true,
    pending = false,
    active = false,
    done = false,
    warn = false,
    reason = "",
    label = null
  } = options;
  const locallyBusy = buttonIsBusy(button);
  const busy = locallyBusy || !!pending;
  const hasTransientFeedback = !!button.dataset.feedback && button.dataset.feedback !== "busy";
  button.disabled = !!(!enabled || busy);
  button.classList.toggle("is-active", !!active && !done && !!enabled && !busy);
  button.classList.toggle("is-done", !!done && !busy);
  button.classList.toggle("is-warn", !!warn && !done && !!enabled && !busy);
  button.dataset.state = busy ? "pending" : (!enabled ? "disabled" : (done ? "done" : (warn ? "warn" : (active ? "active" : "ready"))));
  if (busy) {
    button.setAttribute("aria-busy", "true");
  } else if (!locallyBusy) {
    button.removeAttribute("aria-busy");
  }
  if (!locallyBusy && !hasTransientFeedback) {
    button.textContent = label || baseButtonLabel(button);
    button.title = reason || "";
    button.dataset.actionReason = reason || "";
  } else if (reason && !locallyBusy) {
    button.title = reason;
    button.dataset.actionReason = reason;
  }
}

function floorPreviewAvailable() {
  return !!el.floorAssistPreview?.getAttribute("src");
}

function cropRectIsFullFrame(rect = currentCropRect()) {
  return rect.x <= 0.001 && rect.y <= 0.001 && rect.w >= 0.999 && rect.h >= 0.999;
}

function wallSlotHasData(slot = currentWallRectangle()) {
  return !!(slot?.geometry?.valid || (Array.isArray(slot?.corners) && slot.corners.length > 0));
}

function anyWallSlotHasData() {
  ensureWallRectangles();
  return wallRectangles.some((slot) => wallSlotHasData(slot));
}

function setBridgeAction(id, command, options = {}) {
  const bridgeReady = webviewAvailable();
  const intent = registerActionIntent(id, { command, label: options.label || actionLabel(command, id) });
  const reason = !bridgeReady
    ? intent.unavailableLabel
    : (options.reason || intent.label);
  setActionControl(el[id], Object.assign({}, options, {
    enabled: bridgeReady && options.enabled !== false,
    pending: commandPending(command),
    reason
  }));
}

function updateActionPresentation() {
  const bridgeReady = webviewAvailable();
  const previewReady = floorPreviewAvailable();
  const crop = currentCropRect();
  const cropFull = cropRectIsFullFrame(crop) && !checked("cropToggle");
  const wallSlot = currentWallRectangle();
  const wallReady = Array.isArray(wallSlot?.corners) && wallSlot.corners.length >= 4;
  const wallSolved = !!wallSlot?.geometry?.valid;
  const wallHasData = wallSlotHasData(wallSlot);
  const floorReady = floorMarks.length >= 3;
  const floorAccepted = acceptedProjectiveFloorGeometry() || acceptedScalarFloorGeometry() || acceptedManualPlankGeometry();
  const anyDrawing = cropDrawing || cropAdjust || floorMarking || floorMarks.length > 0 || anyWallSlotHasData();

  setBridgeAction("openModels", "openModelsFolder", { reason: "Open model asset folder" });
  setBridgeAction("rescanModel", "rescanModel", { reason: "Reload model asset and device status" });
  setBridgeAction("openCalib", "openCalibrationFolder", { reason: "Open calibration folder" });
  setBridgeAction("createCalib", "createCalibrationTemplate", { reason: "Create a calibration template" });
  setBridgeAction("openBuild", "openBuildFolder", { reason: "Open build output folder" });
  setBridgeAction("prepDeploy", "prepareDeployFolder", { reason: "Prepare deploy folder" });

  const selectedDevice = el.inferenceDevice?.value || "";
  const activeDevice = current.config?.inference?.device || current.model?.active_device || "";
  setActionControl(el.applyInferenceDevice, {
    enabled: bridgeReady && !commandPending("saveConfig", "stopRuntime", "startRuntime", "rescanModel"),
    pending: commandPending("saveConfig", "stopRuntime", "startRuntime", "rescanModel"),
    active: selectedDevice !== "" && selectedDevice !== activeDevice,
    done: selectedDevice !== "" && selectedDevice === activeDevice,
    reason: bridgeReady ? "Save selected inference device and refresh runtime model state" : "Backend bridge is unavailable"
  });

  setBridgeAction("refreshPreview", "refreshCameraPreview", {
    active: !previewReady,
    reason: previewReady ? "Refresh the current camera setup preview" : `Refresh ${activeFloorCameraLabel()} preview before drawing`
  });
  setActionControl(el.cropDraw, {
    enabled: previewReady && !cropDrawing,
    active: cropDrawing,
    reason: previewReady ? "Draw Camera A crop rectangle on the preview" : "Refresh preview before drawing crop"
  });
  setActionControl(el.cropFull, {
    enabled: !cropFull,
    done: cropFull,
    reason: cropFull ? "Camera A already uses full frame" : "Reset Camera A crop to full frame"
  });
  setBridgeAction("floorManualApply", "calibrateFloorGeometryBackend", {
    enabled: bridgeReady && floorReady,
    active: floorReady && !floorAccepted,
    done: floorAccepted && !floorMarking,
    reason: floorReady ? "Solve drawn plank geometry" : "Draw two long plank edges plus one short end cap first"
  });
  setActionControl(el.floorMarkStart, {
    enabled: previewReady,
    active: floorMarking && floorMarkMode === "plank",
    reason: previewReady ? "Draw one visible floor plank" : `Refresh ${activeFloorCameraLabel()} preview before drawing`
  });
  setActionControl(el.floorWallStart, {
    enabled: previewReady,
    active: floorMarking && floorMarkMode === "wall",
    reason: previewReady ? "Draw four corners of one rectangular wall object" : `Refresh ${activeFloorCameraLabel()} preview before drawing`
  });
  setActionControl(el.wallApplySelected, {
    enabled: bridgeReady && wallReady,
    active: wallReady && !wallSolved,
    done: wallSolved,
    reason: wallReady ? `Solve ${wallSlotLabel()} wall sample` : "Click four corners before solving this wall sample"
  });
  setActionControl(el.wallClearSelected, {
    enabled: wallHasData,
    warn: wallHasData,
    reason: wallHasData ? `Clear ${wallSlotLabel()}` : `${wallSlotLabel()} has no marks to clear`
  });
  setActionControl(el.floorMarkClear, {
    enabled: anyDrawing || floorAccepted,
    warn: anyDrawing || floorAccepted,
    reason: anyDrawing || floorAccepted ? "Clear current drawing and local geometry hints" : "No drawing or local geometry to clear"
  });
  setActionControl(el.trackerSpaceIdentity, {
    enabled: true,
    reason: "Load an identity transform draft into tracker-space fields"
  });
  const trackerSpaceState = trackerSpaceLocalStatus();
  setActionControl(el.trackerSpaceValidate, {
    enabled: true,
    active: trackerSpaceState.status !== "valid",
    done: trackerSpaceState.status === "valid" && checked("trackerSpaceValidToggle"),
    reason: "Validate tracker-space numbers locally before saving"
  });
  normalizeButtonVisualStates();
}

function positiveIndex(value) {
  const n = Number(value);
  return Number.isFinite(n) && n > 0;
}

function oscTrackerIndexForLandmark(key) {
  switch (key) {
    case "pelvis": return Number(current.config?.osc?.pelvis_tracker_index ?? 0);
    case "left_foot": return Number(current.config?.osc?.left_foot_tracker_index ?? 0);
    case "right_foot": return Number(current.config?.osc?.right_foot_tracker_index ?? 0);
    case "chest": return Number(current.config?.osc?.chest_tracker_index ?? 0);
    case "left_elbow": return Number(current.config?.osc?.left_elbow_tracker_index ?? 0);
    case "right_elbow": return Number(current.config?.osc?.right_elbow_tracker_index ?? 0);
    case "left_knee": return Number(current.config?.osc?.left_knee_tracker_index ?? 0);
    case "right_knee": return Number(current.config?.osc?.right_knee_tracker_index ?? 0);
    default: return 1;
  }
}

function landmarkEnabledByConfig(key) {
  if (key === "forward" || key === "floor") return true;
  const meta = steamVrLandmarkByKey.get(key);
  if (meta?.required) return true;
  const bridge = current.config?.steamvr_tracker_bridge || {};
  if (current.config?.steamvr_tracker_bridge?.enabled !== false) {
    if (key === "chest") return bridge.send_chest !== false;
    if (key === "left_elbow" || key === "right_elbow") return !!bridge.send_elbows;
    if (key === "left_knee" || key === "right_knee") return !!bridge.send_knees;
  }
  return positiveIndex(oscTrackerIndexForLandmark(key));
}

function canonicalSteamVrLandmarkKey(raw) {
  const value = String(raw || "").trim().toLowerCase();
  if (!value) return "";
  const aliases = {
    left_foot_marker: "left_foot",
    right_foot_marker: "right_foot",
    pelvis_marker: "pelvis",
    floor_marker: "floor",
    forward_marker: "forward",
    chest_marker: "chest",
    left_elbow_marker: "left_elbow",
    right_elbow_marker: "right_elbow",
    left_knee_marker: "left_knee",
    right_knee_marker: "right_knee",
    leftfoot: "left_foot",
    rightfoot: "right_foot",
    leftelbow: "left_elbow",
    rightelbow: "right_elbow",
    leftknee: "left_knee",
    rightknee: "right_knee"
  };
  return aliases[value] || value;
}

function steamVrSampleKey(sample) {
  return canonicalSteamVrLandmarkKey(sample?.landmark_key || sample?.landmark);
}

function steamVrAcceptedSampleKeys(samples = []) {
  const accepted = new Set();
  for (const sample of samples || []) {
    const key = steamVrSampleKey(sample);
    if (key && sample?.accepted) accepted.add(key);
  }
  return accepted;
}

function steamVrSamplesByKey(samples = []) {
  const byKey = new Map();
  for (const sample of samples || []) {
    const key = steamVrSampleKey(sample);
    if (!key) continue;
    if (!byKey.has(key)) byKey.set(key, []);
    byKey.get(key).push(sample);
  }
  return byKey;
}

function steamVrRequiredComplete(acceptedKeys) {
  return steamVrRequiredLandmarkKeys.every((key) => acceptedKeys.has(key));
}

function steamVrRequiredAcceptedCount(acceptedKeys) {
  return steamVrRequiredLandmarkKeys.filter((key) => acceptedKeys.has(key)).length;
}

function firstMissingRequiredSteamVrLandmark(acceptedKeys) {
  return steamVrAlignmentLandmarks.find((item) => item.required && !acceptedKeys.has(item.key)) || null;
}

function stepIndexForSteamVrLandmark(key) {
  return wizardLandmarks.findIndex((step) => step.landmark === key);
}

function solveWizardStepIndex() {
  return wizardLandmarks.length - 1;
}

function steamVrControllerReady(provider = {}) {
  return !!(provider.left_controller_tracked || provider.right_controller_tracked ||
    Number(provider.controller_device_count || 0) > 0);
}

function steamVrProviderStructurallyUnavailable(provider = {}) {
  const reason = String(provider.reason || provider.status || "").toLowerCase();
  return !!(provider.compile_disabled || provider.hard_unavailable ||
    reason.includes("compile_disabled") || reason.includes("not_compiled") ||
    reason.includes("not compiled"));
}

function steamVrNextWizardStepForState(session = {}, acceptedKeys = new Set()) {
  if (!session.active && !acceptedKeys.size) return 0;
  const currentStep = wizardLandmarks[wizardStep];
  if (session.active && currentStep?.landmark && !acceptedKeys.has(currentStep.landmark)) {
    return wizardStep;
  }
  const missing = firstMissingRequiredSteamVrLandmark(acceptedKeys);
  if (missing) return stepIndexForSteamVrLandmark(missing.key);
  return solveWizardStepIndex();
}

function renderSteamVrWizardProgress(acceptedKeys = new Set()) {
  if (!el.steamVrWizProgress) return;
  if (el.steamVrWizProgress.dataset.dynamic !== "true") {
    el.steamVrWizProgress.innerHTML = wizardLandmarks.map((step, index) => {
      const optional = step.landmark && !steamVrLandmarkByKey.get(step.landmark)?.required;
      return `<div class="wiz-step${optional ? " optional" : ""}" data-wiz="${esc(step.key)}"><div class="wiz-num">${index + 1}</div><span class="wiz-label">${esc(step.short || step.label || step.key)}</span></div>`;
    }).join("");
    el.steamVrWizProgress.dataset.dynamic = "true";
  }
  const steps = el.steamVrWizProgress.querySelectorAll(".wiz-step");
  const requiredComplete = steamVrRequiredComplete(acceptedKeys);
  steps.forEach((node, i) => {
    const step = wizardLandmarks[i];
    const done = !!step?.landmark && acceptedKeys.has(step.landmark);
    const optional = !!step?.landmark && !steamVrLandmarkByKey.get(step.landmark)?.required;
    node.classList.toggle("done", done || (step?.key === "start" && (wizardStep > 0 || acceptedKeys.size > 0)) || (step?.key === "solve" && requiredComplete));
    node.classList.toggle("active", i === wizardStep);
    node.classList.toggle("optional", optional);
    node.classList.toggle("blocked", optional && !landmarkEnabledByConfig(step.landmark));
  });
}

function trackingMode() {
  return el.trackingMode?.value === "monocular" ? "monocular" : "stereo";
}

function option(value, label) {
  return `<option value="${esc(value)}">${esc(label)}</option>`;
}

function statusClass(ok, warn = false) {
  return ok ? "good" : (warn ? "warn" : "bad");
}

function phoneSiteStatusClass(status, enabled) {
  const value = String(status || "").toLowerCase();
  if (enabled && value === "enabled") return "good";
  if (value === "disabled") return "muted";
  if (value.includes("missing") || value.includes("unreadable") || value.includes("failed")) return "bad";
  return "warn";
}

function floorStatusClass(status) {
  if (status === "active") return "good";
  if (status === "standby" || status === "weak" || status === "waiting") return "warn";
  if (status === "disabled" || status === "inactive") return "muted";
  return "bad";
}

function trackerSpaceStatusClass(status) {
  if (status === "valid") return "good";
  // "stale" means degraded output: alignment is old but OSC continues with last numeric transform.
  if (status === "weak" || status === "pending" || status === "sampling" || status === "degraded" || status === "stale") return "warn";
  // "missing" and "idle" mean blocked: no tracking data is sent via OSC.
  if (status === "missing" || status === "idle") return "bad";
  return "bad";
}

function solveStatusClass(status) {
  if (status === "active") return "good";
  if (status === "degraded" || status === "occlusion" || status === "uncertain") return "warn";
  return "bad";
}

function bodyCalibrationPersistClass(status) {
  if (status === "saved_on_shutdown" || status === "saved_this_frame" || status === "persisted") return "good";
  if (status === "not_complete" || status === "complete_pending_persist") return "warn";
  if (status === "disabled" || status === "auto_persist_disabled") return "muted";
  return "bad";
}

function trackerSpaceLocalStatus() {
  const values = [
    Number(el.trackerOffsetX?.value),
    Number(el.trackerOffsetY?.value),
    Number(el.trackerOffsetZ?.value),
    Number(el.trackerRotX?.value),
    Number(el.trackerRotY?.value),
    Number(el.trackerRotZ?.value),
    Number(el.trackerRotW?.value),
    Number(el.trackerScale?.value)
  ];
  if (!values.every(Number.isFinite)) {
    return { status: "invalid", reason: "all offset, rotation, and scale fields must be finite numbers" };
  }
  const [,, , rx, ry, rz, rw, scale] = values;
  const qLen2 = rx * rx + ry * ry + rz * rz + rw * rw;
  if (!Number.isFinite(qLen2) || qLen2 < 1e-12) {
    return { status: "invalid", reason: "rotation must be a non-zero quaternion" };
  }
  if (!Number.isFinite(scale) || scale <= 0) {
    return { status: "invalid", reason: "scale must be positive" };
  }
  if (!checked("trackerSpaceValidToggle")) {
    return { status: "pending", reason: "transform values look valid, but the transform is not marked valid yet" };
  }
  return { status: "valid", reason: "manual tracker-space numbers are finite and marked valid; physical alignment is user-provided" };
}

function renderSteamVrOutputStatus() {
  const bridge = current.config?.steamvr_tracker_bridge || {};
  const debugBridge = current.debug?.steamvr_bridge || {};
  const enabled = bridge.enabled !== false;
  const host = bridge.target_address || "127.0.0.1";
  const port = bridge.target_port ?? 39560;
  const min = debugBridge.min_confidence ?? bridge.min_confidence ?? 0.2;
  if (el.steamVrBridgeStatus) {
    el.steamVrBridgeStatus.innerHTML = `SteamVR trackers: <b class="${enabled ? "good" : "bad"}">${enabled ? "ON" : "OFF"}</b> · driver bridge ${esc(host)}:${esc(port)} · active min conf ${esc(min)}`;
  }
}

function renderTrackerSpace() {
  const saved = current.tracker_space || {};
  const local = trackerSpaceLocalStatus();
  const useSaved = syncInputs;
  const status = useSaved ? (saved.status || local.status) : local.status;
  const reason = useSaved ? (saved.reason || local.reason) : local.reason;
  if (el.trackerSpaceStatus) {
    const source = useSaved ? (saved.source || "manual") : "manual";
    const blocked = useSaved
      ? !!saved.osc_blocked
      : (checked("oscToggle") && status !== "valid");
    let annotation = "";
    if (status === "stale") annotation = " (degraded output — alignment is old, OSC continues)";
    else if (status === "missing" || status === "idle") annotation = " (OSC blocked — no tracking data sent)";
    const text = `${status} · ${source}${blocked ? " · OSC blocked" : ""}${annotation}`;
    el.trackerSpaceStatus.innerHTML = `tracker space: <b class="${trackerSpaceStatusClass(status)}">${esc(text)}</b>`;
  }
  if (el.trackerSpaceReadout) {
    const action = saved.action || "Set offset, rotation, and scale, validate the numbers, mark the manual transform valid only after physical alignment, then save before enabling OSC.";
    const roleOffsetSource = (saved.source === "steamvr_controller_alignment" || saved.source === "steamvr_controller_alignment_stale")
      ? "SteamVR controller alignment"
      : (saved.manual_fallback_active ? "manual fallback" : "manual config");
    el.trackerSpaceReadout.textContent = `${reason}. ${action} Role offsets: ${roleOffsetSource}.`;
  }
}


function updateWizardPrompt(stepIndex) {
  wizardStep = Math.max(0, Math.min(Number(stepIndex) || 0, wizardLandmarks.length - 1));
  const step = wizardLandmarks[wizardStep] || wizardLandmarks[0];
  if (el.steamVrPromptMain) el.steamVrPromptMain.textContent = step.main;
  if (el.steamVrPromptDetail) el.steamVrPromptDetail.textContent = step.detail;
  const prompt = el.steamVrCalibPrompt;
  if (prompt) {
    const oldHand = prompt.querySelector(".prompt-hand");
    if (oldHand) oldHand.remove();
    if (step.hand) {
      const badge = document.createElement("div");
      badge.className = "prompt-hand";
      badge.textContent = step.hand;
      prompt.appendChild(badge);
    }
  }
  const samples = current.steamvr_alignment?.samples || [];
  renderSteamVrWizardProgress(steamVrAcceptedSampleKeys(samples));
}

function advanceWizardAfterSample(landmark) {
  const key = canonicalSteamVrLandmarkKey(landmark);
  const samples = current.steamvr_alignment?.samples || [];
  const accepted = steamVrAcceptedSampleKeys(samples);
  if (key) accepted.add(key);
  const next = steamVrNextWizardStepForState(current.steamvr_alignment?.session || {}, accepted);
  updateWizardPrompt(next);
}

function advanceWizardAfterSampleFailure(landmark, reply) {
  const key = canonicalSteamVrLandmarkKey(landmark);
  const samples = current.steamvr_alignment?.samples || [];
  const accepted = steamVrAcceptedSampleKeys(samples);
  const requiredComplete = steamVrRequiredComplete(accepted);
  let nextIndex = requiredComplete ? solveWizardStepIndex() : -1;
  if (nextIndex < 0) {
    const required = steamVrAlignmentLandmarks.filter((item) => item.required);
    const currentIndex = Math.max(0, required.findIndex((item) => item.key === key));
    const after = required.slice(currentIndex + 1).find((item) => !accepted.has(item.key));
    const before = required.slice(0, currentIndex).find((item) => !accepted.has(item.key));
    const next = after || before || firstMissingRequiredSteamVrLandmark(accepted);
    nextIndex = next ? stepIndexForSteamVrLandmark(next.key) : solveWizardStepIndex();
  }
  updateWizardPrompt(nextIndex);
  const error = reply?.result?.status || reply?.result?.reason || reply?.error || "sample rejected";
  setCommandStatus(`SteamVR sample rejected: ${error}; moved to another calibration step`, false);
}

async function recordSteamVrWizardLandmark(landmark, controller, button) {
  const key = canonicalSteamVrLandmarkKey(landmark);
  const stepIndex = stepIndexForSteamVrLandmark(key);
  if (stepIndex >= 0) updateWizardPrompt(stepIndex);
  const reply = await sendCommand("steamVrAlignmentRecord", { landmark: key, controller }, button, { actionId: button?.dataset?.actionId || steamVrLandmarkButtons[key] });
  if (commandOk(reply)) advanceWizardAfterSample(key);
  else advanceWizardAfterSampleFailure(key, reply);
  return reply;
}

function currentSteamVrWizardLandmarkKey() {
  return canonicalSteamVrLandmarkKey(wizardLandmarks[wizardStep]?.landmark);
}

function maybeRecordSteamVrTrigger(provider, session) {
  if (steamVrTriggerRecordPending || !session.active || commandPending("steamVrAlignmentRecord", "steamVrAlignmentFinish")) return;
  const landmark = currentSteamVrWizardLandmarkKey();
  if (!landmark || !landmarkEnabledByConfig(landmark)) return;

  const leftEdge = !!provider.left_trigger_pressed_edge;
  const rightEdge = !!provider.right_trigger_pressed_edge;
  if (!leftEdge && !rightEdge) return;

  const controller = leftEdge ? "left_hand" : "right_hand";
  const button = el[steamVrLandmarkButtons[landmark]];
  steamVrTriggerRecordPending = true;
  recordSteamVrWizardLandmark(landmark, controller, button)
    .finally(() => { steamVrTriggerRecordPending = false; });
}

function renderSteamVrAlignmentButtons(provider, session, acceptedKeys, samplesByKey, requiredMet) {
  const bridgeReady = webviewAvailable();
  const providerStructural = steamVrProviderStructurallyUnavailable(provider);
  const sessionActive = !!session.active;
  const startPending = commandPending("steamVrAlignmentStart");
  const recordPending = commandPending("steamVrAlignmentRecord") || steamVrTriggerRecordPending;
  const finishPending = commandPending("steamVrAlignmentFinish");
  const clearPending = commandPending("steamVrAlignmentClear");
  const busy = startPending || recordPending || finishPending || clearPending;
  const providerRecoverable = !providerStructural;
  const controllerReady = steamVrControllerReady(provider);
  const acceptedCount = acceptedKeys.size;

  setActionControl(el.steamVrAlignStart, {
    enabled: bridgeReady && !sessionActive && providerRecoverable && !busy,
    pending: startPending,
    active: bridgeReady && !sessionActive && providerRecoverable,
    done: sessionActive,
    reason: !bridgeReady
      ? actionIntents.steamVrAlignStart.unavailableLabel
      : (sessionActive
        ? "Calibration session is already active"
        : (providerStructural ? (provider.reason || "SteamVR provider is unavailable in this build") : "Begin a SteamVR controller alignment session")),
    label: sessionActive ? "Session active" : null
  });

  const firstMissingRequired = firstMissingRequiredSteamVrLandmark(acceptedKeys);
  const canRecord = bridgeReady && sessionActive && !recordPending && !finishPending && !startPending && providerRecoverable && controllerReady;
  for (const item of steamVrAlignmentLandmarks) {
    const done = acceptedKeys.has(item.key);
    const keySamples = samplesByKey.get(item.key) || [];
    const rejected = keySamples.some((sample) => sample && !sample.accepted);
    const enabledByConfig = landmarkEnabledByConfig(item.key);
    const promptedLandmark = currentSteamVrWizardLandmarkKey();
    const active = promptedLandmark
      ? promptedLandmark === item.key
      : (!!firstMissingRequired && firstMissingRequired.key === item.key);
    const reason = !bridgeReady
      ? actionIntents[item.buttonId].unavailableLabel
      : (!sessionActive
        ? "Start calibration first"
        : (!providerRecoverable
          ? (provider.reason || "SteamVR provider is unavailable")
          : (!controllerReady
            ? "Waiting for a tracked SteamVR controller"
            : (!enabledByConfig
              ? "This optional tracker role is disabled: enable SteamVR trackers or map this legacy OSC role"
              : (done
                ? "Sample accepted; click again to replace it"
                : item.detail)))));
    setActionControl(item.buttonId, {
      enabled: canRecord && enabledByConfig,
      pending: recordPending && active,
      active: active && !done,
      done,
      warn: rejected && !done,
      reason,
      label: done ? `${item.label} ✓` : item.label
    });
  }

  const requiredMissing = Math.max(0, steamVrRequiredLandmarkKeys.length - steamVrRequiredAcceptedCount(acceptedKeys));
  setActionControl(el.steamVrAlignFinish, {
    enabled: bridgeReady && sessionActive && requiredMet && !busy,
    pending: finishPending,
    active: bridgeReady && sessionActive && requiredMet,
    reason: !bridgeReady ? actionIntents.steamVrAlignFinish.unavailableLabel : (requiredMet
      ? "Solve alignment and save it"
      : `Need ${requiredMissing} more required sample(s) before solving`)
  });

  const hasSomethingToClear = sessionActive || acceptedCount > 0 || !!(current.steamvr_alignment?.transform?.valid);
  setActionControl(el.steamVrAlignClear, {
    enabled: bridgeReady && hasSomethingToClear && !busy,
    pending: clearPending,
    warn: bridgeReady && hasSomethingToClear,
    reason: !bridgeReady ? actionIntents.steamVrAlignClear.unavailableLabel : (hasSomethingToClear ? "Clear SteamVR alignment session and samples" : "No SteamVR alignment data to clear")
  });
}

function renderSteamVrAlignment() {
  const s = current.steamvr_alignment || {};
  const provider = s.provider || {};
  const session = s.session || {};
  const transform = s.transform || {};
  const active = s.active_transform || {};
  const context = s.context || {};
  const samples = s.samples || [];
  const acceptedKeys = steamVrAcceptedSampleKeys(samples);
  const samplesByKey = steamVrSamplesByKey(samples);
  const requiredTotal = steamVrRequiredLandmarkKeys.length;
  const requiredCount = Math.max(steamVrRequiredAcceptedCount(acceptedKeys), Number(session.required_samples_present ?? 0));
  const requiredMet = !!session.required_samples_complete || steamVrRequiredComplete(acceptedKeys) ||
    requiredCount >= requiredTotal;
  const state = transform.state || "missing";
  const reason = transform.reason || provider.reason || "idle";
  if (el.steamVrAlignStatus) {
    let providerText;
    let providerClass = "muted";
    if (provider.runtime_initialized) {
      providerText = `${provider.status || "connected"} · controllers ${provider.controller_device_count ?? 0}`;
      providerClass = steamVrControllerReady(provider) ? "good" : "warn";
    } else {
      const isStructural = steamVrProviderStructurallyUnavailable(provider);
      providerText = isStructural
        ? `${provider.status || "unavailable"} · ${provider.reason || "not available this session"}`
        : `${provider.status || "unavailable"} · ${provider.reason || "may recover"}`;
      providerClass = isStructural ? "bad" : "warn";
    }
    const sessionText = session.active ? "session active" : "session idle";
    el.steamVrAlignStatus.innerHTML = `SteamVR alignment: <b class="${trackerSpaceStatusClass(state === "stale" ? "invalid" : state)}">${esc(state)}</b> · ${esc(reason)} · <span class="${providerClass}">${esc(providerText)}</span> · <b class="${session.active ? "good" : "muted"}">${esc(sessionText)}</b>`;
  }
  if (el.steamVrAlignMetrics) {
    el.steamVrAlignMetrics.innerHTML =
      `<div><span>samples</span><b>${requiredCount}/${requiredTotal} required · ${esc(session.accepted_sample_count ?? acceptedKeys.size)} accepted</b></div>` +
      `<div><span>transform</span><b>${transform.valid ? "VALID" : "NO"} · stale ${transform.stale ? "YES" : "NO"}</b></div>` +
      `<div><span>residual</span><b>${fmt(transform.residual_m, 3)}m · floor ${fmt(transform.floor_residual_m, 3)}m</b></div>` +
      `<div><span>source</span><b>${esc(active.source || "none")}</b></div>` +
      `<div><span>context</span><b>body ${context.body_calibration_valid ? "OK" : "NO"} · floor ${context.floor_calibration_valid ? "OK" : "NO"} · live ${context.body_state_stable ? "OK" : "NO"}</b></div>` +
      `<div><span>confidence</span><b>${fmt(transform.confidence, 2)}</b></div>`;
  }
  if (el.steamVrAlignSamples) {
    el.steamVrAlignSamples.innerHTML = samples.length
      ? samples.map((sample) => {
          const key = steamVrSampleKey(sample);
          const meta = steamVrLandmarkByKey.get(key);
          const label = meta?.label || key || sample.landmark || "unknown";
          const required = meta?.required ? "required" : "optional";
          return `<div class="sample-row ${sample.accepted ? "accepted" : "rejected"}"><span>${esc(label)}</span><span>${esc(sample.controller)}</span><span class="${sample.accepted ? "good" : "bad"}">${sample.accepted ? "accepted" : "rejected"}</span><span>${esc(required)} · ${esc(sample.reason || sample.reason_code || "")}</span></div>`;
        }).join("")
      : `<div class="sample-row muted"><span>no samples recorded</span><span></span><span></span><span></span></div>`;
  }

  const desiredStep = steamVrNextWizardStepForState(session, acceptedKeys);
  if (wizardStep !== desiredStep) updateWizardPrompt(desiredStep);
  else renderSteamVrWizardProgress(acceptedKeys);
  renderSteamVrAlignmentButtons(provider, session, acceptedKeys, samplesByKey, requiredMet);
}

function selectedCameraA() {
  const items = current.cameras?.items || [];
  const selectedIndex = Number(el.cameraA?.value ?? current.config?.camera_a?.device_index ?? 0);
  return items.find((camera) => camera.index === selectedIndex) || current.config?.camera_a || {};
}

function stereoFallbackActive() {
  const mode = String(current.debug?.degradation_mode || current.debug?.stereo_fallback_mode || "");
  return mode.startsWith("stereo_monocular_fallback:") || mode === "stereo_fallback_active";
}

function floorAssistApplies(mode) {
  return mode === "monocular" || (mode === "stereo" && checked("stereoFallbackToggle"));
}

function floorAssistReferenceUsable(spacingPx, referenceY) {
  // Just require a finite positive reference Y. No assumption about camera mounting.
  // principalY was only correct for ceiling-mounted cameras — removed.
  if (!Number.isFinite(referenceY) || referenceY <= 0) return false;
  const referenceM = Number(el.floorReferenceM?.value);
  if (Number.isFinite(referenceM) && referenceM > 0) return true;
  // Fallback: require that the reference is not trivially close to origin
  return Number.isFinite(spacingPx) && spacingPx > 0 && referenceY > 20;
}

function floorGeometryMatchesCurrentImageSize(g) {
  if (!g) return false;
  const geometryWidth = Number(g.image_width || 0);
  const geometryHeight = Number(g.image_height || 0);
  if (!Number.isFinite(geometryWidth) || !Number.isFinite(geometryHeight) || geometryWidth <= 0 || geometryHeight <= 0) {
    return false;
  }
  const { imageWidth, imageHeight } = previewImagePixelSize();
  return Math.round(geometryWidth) === Math.round(imageWidth) &&
    Math.round(geometryHeight) === Math.round(imageHeight);
}

function projectiveFloorGeometryAccepted(g) {
  return !!(g?.valid && g?.backend_owned !== false &&
    floorGeometryMatchesCurrentImageSize(g) &&
    g?.homography_valid &&
    Number(g?.metric_scale_confidence || 0) > 0);
}

function scalarFloorGeometryAccepted(g) {
  return !!(g?.backend_owned !== false &&
    floorGeometryMatchesCurrentImageSize(g) &&
    g?.family_a?.valid &&
    g?.family_a?.metric_spacing_valid &&
    Number(g?.family_a?.spacing_m || 0) > 0 &&
    Number(g?.family_a?.spacing_px || 0) > 0 &&
    Number(g?.metric_scale_confidence || g?.family_a?.confidence || 0) > 0);
}

function manualPlankFloorGeometryAccepted(g) {
  return !!(g?.valid && g?.backend_owned !== false &&
    floorGeometryMatchesCurrentImageSize(g) &&
    (g?.source === "manual_plank_outline" || g?.floor_type === "manual_plank"));
}

function floorGeometryAcceptedForRuntime(g) {
  return projectiveFloorGeometryAccepted(g) || scalarFloorGeometryAccepted(g) || manualPlankFloorGeometryAccepted(g);
}

function acceptedProjectiveFloorGeometry() {
  if (floorGeometryCleared) return false;
  const runtime = current.debug?.floor_geometry;
  const saved = floorGeometryAuto;
  return projectiveFloorGeometryAccepted(runtime?.valid ? runtime : saved);
}

function acceptedScalarFloorGeometry() {
  if (floorGeometryCleared) return false;
  const runtime = current.debug?.floor_geometry;
  const saved = floorGeometryAuto;
  return scalarFloorGeometryAccepted(runtime?.valid ? runtime : saved);
}

function acceptedManualPlankGeometry() {
  if (floorGeometryCleared) return false;
  const runtime = current.debug?.floor_geometry;
  const saved = floorGeometryAuto;
  return manualPlankFloorGeometryAccepted(runtime?.valid ? runtime : saved);
}

function currentFloorAssistStatus(mode) {
  if (!floorAssistApplies(mode)) return "disabled";
  const floor = current.floor_assist || {};
  const runtimeStatus = syncInputs ? floor.status : "";
  const geometryAccepted = acceptedProjectiveFloorGeometry() || acceptedScalarFloorGeometry() || acceptedManualPlankGeometry();
  const geometryAvailableButWrongImage = !!(floorGeometryAuto?.valid && !floorGeometryMatchesCurrentImageSize(floorGeometryAuto));
  if (geometryAccepted) {
    return mode === "stereo" && !stereoFallbackActive() ? "standby" : "active";
  }
  if (runtimeStatus && runtimeStatus !== "disabled" && runtimeStatus !== "inactive") {
    return runtimeStatus;
  }
  if (geometryAvailableButWrongImage) return "invalid";
  if (!checked("floorAssistToggle")) return "disabled";
  const spacingM = Number(el.floorSpacingM?.value);
  const spacingPx = Number(el.floorSpacingPx?.value);
  const referenceY = Number(el.floorReferenceY?.value);
  const confidence = Number(el.floorConfidence?.value);
  if (!Number.isFinite(spacingM) || spacingM <= 0 ||
      !Number.isFinite(spacingPx) || spacingPx <= 0 ||
      !floorAssistReferenceUsable(spacingPx, referenceY)) return "invalid";
  if (spacingPx < 8 || !Number.isFinite(confidence)) return "invalid";

  if (runtimeStatus && runtimeStatus !== "disabled") return runtimeStatus;
  return mode === "stereo" && !stereoFallbackActive() ? "standby" : "active";
}

function renderFloorAssist(mode) {
  const floor = current.floor_assist || {};
  const solver = current.debug?.solver || {};
  const usedSource = floor.used_source || solver.monocular_scale_source || "";
  const status = currentFloorAssistStatus(mode);
  if (el.floorAssistStatus) {
    const projectiveAccepted = acceptedProjectiveFloorGeometry();
    const scalarAccepted = acceptedScalarFloorGeometry();
    const manualPlankAccepted = acceptedManualPlankGeometry();
    const acceptedSource = projectiveAccepted ? "floor_projective" : (scalarAccepted ? "floor_spacing" : "--");
    const source = acceptedSource !== "--" ? acceptedSource : (manualPlankAccepted ? "floor_orientation" : (floor.source || solver.monocular_scale_source || "--"));
    const acceptedConfidence = Number((current.debug?.floor_geometry?.valid ? current.debug.floor_geometry : floorGeometryAuto)?.metric_scale_confidence || 0);
    const depth = Number(floor.depth_m ?? solver.monocular_floor_assist_depth_m);
    const confidence = Number(floor.confidence ?? solver.monocular_floor_assist_confidence ?? acceptedConfidence);
    const usedThisFrame = usedSource === "floor_projective" || usedSource === "floor_spacing" || usedSource === "wall_depth" || floor.floor_geometry_used;
    const geometryImageMismatch = !projectiveAccepted && !scalarAccepted && !!(floorGeometryAuto?.valid && !floorGeometryMatchesCurrentImageSize(floorGeometryAuto));
    let detail = status === "disabled"
        ? "disabled"
        : (geometryImageMismatch
        ? `invalid · saved floor geometry image size ${esc(floorGeometryAuto.image_width)}x${esc(floorGeometryAuto.image_height)} does not match current ${esc(previewImagePixelSize().imageWidth)}x${esc(previewImagePixelSize().imageHeight)}`
        : (projectiveAccepted
          ? `active · projective metric scale · ${esc(floorGeometryAuto?.camera_height_m ? (floorGeometryAuto.camera_height_m.toFixed(2) + "m cam") : "unknown cam")}`
          : (scalarAccepted
            ? `active · scalar metric depth ${esc(depth.toFixed(2))}m`
            : (manualPlankAccepted
              ? `active · orientation hint · ${esc(floorGeometryAuto?.camera_yaw_rad ? (floorGeometryAuto.camera_yaw_rad.toFixed(2) + "rad yaw") : "")}`
              : (usedSource === "wall_depth"
                ? `active · wall sample depth ${esc(depth.toFixed(2))}m`
                : `active · fallback runtime depth ${esc(depth.toFixed(2))}m`)))));
    if (status !== "disabled" && status !== "invalid" && !projectiveAccepted && !scalarAccepted && !manualPlankAccepted && usedSource === "floor_spacing" && !usedThisFrame) {
      detail += ` · unused`;
    }
    if (floorMarking && floorMarkMode === "wall") {
      const slot = currentWallRectangle();
      detail = `drawing ${wallSlotLabel().toLowerCase()} · ${slot.corners.length}/4 corners · saved samples stay in the bank`;
    } else if (floorMarking) {
      detail = `drawing new plank · ${floorMarks.length}/4 marks · existing accepted geometry stays active until Apply`;
    }
    if (mode !== "monocular") {
      if (!checked("stereoFallbackToggle")) {
        detail = "disabled · stereo uses triangulated calibration; single-camera fallback is off";
      } else if (!checked("floorAssistToggle") && !projectiveAccepted && !scalarAccepted) {
        detail = "disabled · stereo fallback is enabled; scalar floor assist is off and no projective geometry is accepted";
      } else if (status === "standby") {
        detail = `standby · Camera A fallback has accepted runtime geometry ${source} · ${usedThisFrame ? "used this frame" : "not used this frame"} · conf ${fmt(confidence || acceptedConfidence, 2)}`;
      } else if (status === "active") {
        detail = `active · stereo fallback is using ${source} · depth ${fmt(depth, 2)}m · conf ${fmt(confidence || acceptedConfidence, 2)}`;
      } else {
        detail = `${status} · stereo fallback will inherit Camera A floor assist after the inputs are valid`;
      }
    }
    el.floorAssistStatus.innerHTML = `floor assist: <b class="${floorStatusClass(status)}">${esc(detail)}</b>`;
  }

  const camA = cameraConfigForFloorSlot();
  if (el.floorAssistPreview) {
    const preview = displayPreviewUrl(camA);
    if (el.floorAssistPreview.getAttribute("src") !== preview) {
      el.floorAssistPreview.setAttribute("src", preview);
      // A new preview may have a different aspect/dimension binding. Keep solved
      // backend geometry, but do not keep unsaved drawn marks on the wrong image.
      if (floorMarks.length > 0) {
        resetFloorMarks(false);
        setCommandStatus("Camera preview changed; redraw the manual plank before applying", false);
      }
    }
    el.floorAssistPreview.title = preview
      ? `${activeFloorCameraLabel()} setup preview: draw plank lines or wall samples`
      : `Refresh preview for ${activeFloorCameraLabel()} before drawing`;
    // Show/hide placeholder message in the preview wrapper
    const wrap = el.floorAssistPreview.closest(".floor-preview-wrap");
    if (wrap) {
      wrap.classList.toggle("no-preview", !preview);
      wrap.classList.toggle("is-cropped-preview", !!preview && !cropEditPreviewActive() && checked("cropToggle"));
      wrap.classList.toggle("is-crop-editing", cropEditPreviewActive());
    }
    updateWallRectangleStatus();
    updateFloorMarkOverlay();
    updateBodyPoseOverlay();
    updateActionPresentation();
  }

}

function previewImagePixelSize() {
  const img = el.floorAssistPreview;
  // cropEditPreviewActive only covers active drawing/editing; also check saved crop toggle
  const cropActive = cropEditPreviewActive() || (checked("cropToggle") && currentCropRect().w < 0.99);
  const cam = cropActive ? selectedCameraA() : cameraConfigForFloorSlot();
  const imageWidth = Number(cam.preview_frame_width || cam.width || el.monocularImageWidth?.value || current.config?.camera_a?.width || img?.naturalWidth || 1);
  const imageHeight = Number(cam.preview_frame_height || cam.height || el.monocularImageHeight?.value || current.config?.camera_a?.height || img?.naturalHeight || 1);
  return {
    imageWidth: Math.max(1, imageWidth),
    imageHeight: Math.max(1, imageHeight)
  };
}

function cropEditPreviewActive() {
  return !!(cropDrawing || cropAdjust);
}

function previewSourceRect() {
  const { imageWidth, imageHeight } = previewImagePixelSize();
  const cam = cropEditPreviewActive() ? selectedCameraA() : cameraConfigForFloorSlot();
  if (cropEditPreviewActive()) {
    return { x: 0, y: 0, width: imageWidth, height: imageHeight, frameWidth: imageWidth, frameHeight: imageHeight };
  }
  const sourceWidth = Number(cam.preview_source_width || cam.width || imageWidth);
  const sourceHeight = Number(cam.preview_source_height || cam.height || imageHeight);
  return {
    x: Number(cam.preview_source_x || 0),
    y: Number(cam.preview_source_y || 0),
    width: Math.max(1, Number.isFinite(sourceWidth) ? sourceWidth : imageWidth),
    height: Math.max(1, Number.isFinite(sourceHeight) ? sourceHeight : imageHeight),
    frameWidth: imageWidth,
    frameHeight: imageHeight
  };
}

function displayPreviewUrl(cam) {
  if (cropEditPreviewActive() && cam.full_preview) return cam.full_preview;
  return cam.preview || cam.full_preview || "";
}

function clamp01(value, fallback = 0) {
  const n = Number(value);
  if (!Number.isFinite(n)) return fallback;
  return Math.max(0, Math.min(1, n));
}

function currentCropRect() {
  const x = clamp01(el.cropX?.value, 0);
  const y = clamp01(el.cropY?.value, 0);
  const w = Math.max(0.01, Math.min(1 - x, clamp01(el.cropW?.value, 1)));
  const h = Math.max(0.01, Math.min(1 - y, clamp01(el.cropH?.value, 1)));
  return { x, y, w, h };
}

function setCropRect(rect, enabled = true) {
  const x = clamp01(rect?.x, 0);
  const y = clamp01(rect?.y, 0);
  const w = Math.max(0.01, Math.min(1 - x, clamp01(rect?.w ?? rect?.width, 1)));
  const h = Math.max(0.01, Math.min(1 - y, clamp01(rect?.h ?? rect?.height, 1)));
  setValue("cropX", x.toFixed(3));
  setValue("cropY", y.toFixed(3));
  setValue("cropW", w.toFixed(3));
  setValue("cropH", h.toFixed(3));
  setPressed("cropToggle", enabled && !(x === 0 && y === 0 && w === 1 && h === 1));
  renderCropStatus();
  updateFloorMarkOverlay();
}

function cropFromPoints(a, b) {
  if (!a || !b) return null;
  const x1 = Math.max(0, Math.min(a.x, b.x));
  const y1 = Math.max(0, Math.min(a.y, b.y));
  const x2 = Math.min(a.imageWidth, Math.max(a.x, b.x));
  const y2 = Math.min(a.imageHeight, Math.max(a.y, b.y));
  const w = x2 - x1;
  const h = y2 - y1;
  if (w < 8 || h < 8) return null;
  return {
    x: x1 / Math.max(1, a.imageWidth),
    y: y1 / Math.max(1, a.imageHeight),
    w: w / Math.max(1, a.imageWidth),
    h: h / Math.max(1, a.imageHeight)
  };
}

function cropHitTest(point) {
  if (!point || !checked("cropToggle")) return null;
  const rect = currentCropRect();
  const imageWidth = Math.max(1, point.imageWidth);
  const imageHeight = Math.max(1, point.imageHeight);
  const x1 = rect.x * imageWidth;
  const y1 = rect.y * imageHeight;
  const x2 = (rect.x + rect.w) * imageWidth;
  const y2 = (rect.y + rect.h) * imageHeight;
  const threshold = Math.max(14, Math.min(imageWidth, imageHeight) * 0.025);
  const near = (x, y) => Math.hypot(point.x - x, point.y - y) <= threshold;
  if (near(x1, y1)) return "nw";
  if (near(x2, y1)) return "ne";
  if (near(x1, y2)) return "sw";
  if (near(x2, y2)) return "se";
  if (point.x >= x1 && point.x <= x2 && point.y >= y1 && point.y <= y2) return "move";
  return null;
}

function applyCropAdjustment(point) {
  if (!cropAdjust || !point) return;
  const imageWidth = Math.max(1, cropAdjust.imageWidth);
  const imageHeight = Math.max(1, cropAdjust.imageHeight);
  const dx = (point.x - cropAdjust.start.x) / imageWidth;
  const dy = (point.y - cropAdjust.start.y) / imageHeight;
  let { x, y, w, h } = cropAdjust.original;
  const minSize = 0.03;
  if (cropAdjust.mode === "move") {
    x = Math.max(0, Math.min(1 - w, x + dx));
    y = Math.max(0, Math.min(1 - h, y + dy));
  } else {
    let x1 = x;
    let y1 = y;
    let x2 = x + w;
    let y2 = y + h;
    if (cropAdjust.mode.includes("w")) x1 = Math.max(0, Math.min(x2 - minSize, x1 + dx));
    if (cropAdjust.mode.includes("e")) x2 = Math.min(1, Math.max(x1 + minSize, x2 + dx));
    if (cropAdjust.mode.includes("n")) y1 = Math.max(0, Math.min(y2 - minSize, y1 + dy));
    if (cropAdjust.mode.includes("s")) y2 = Math.min(1, Math.max(y1 + minSize, y2 + dy));
    x = x1;
    y = y1;
    w = x2 - x1;
    h = y2 - y1;
  }
  setCropRect({ x, y, w, h }, true);
}

function renderCropStatus() {
  if (!el.cropStatus) return;
  const rect = currentCropRect();
  const enabled = checked("cropToggle");
  el.cropStatus.innerHTML = enabled
    ? `crop: <b class="good">${fmt(rect.x, 3)}, ${fmt(rect.y, 3)} / ${fmt(rect.w, 3)} x ${fmt(rect.h, 3)}</b>`
    : `crop: <b class="muted">full frame</b>`;
}

function previewContentRect() {
  const img = el.floorAssistPreview;
  if (!img || !img.getAttribute("src")) return null;
  const rect = img.getBoundingClientRect();
  if (rect.width <= 0 || rect.height <= 0) return null;
  const source = previewSourceRect();
  const imageWidth = source.width;
  const imageHeight = source.height;
  const scale = Math.min(rect.width / imageWidth, rect.height / imageHeight);
  const width = imageWidth * scale;
  const height = imageHeight * scale;
  return {
    left: rect.left + (rect.width - width) * 0.5,
    top: rect.top + (rect.height - height) * 0.5,
    width,
    height,
    imageWidth: source.frameWidth,
    imageHeight: source.frameHeight,
    sourceX: source.x,
    sourceY: source.y,
    sourceWidth: source.width,
    sourceHeight: source.height
  };
}

function cameraPointFromPreviewEvent(event) {
  const content = previewContentRect();
  if (!content) return null;
  const x = event.clientX - content.left;
  const y = event.clientY - content.top;
  if (x < 0 || y < 0 || x > content.width || y > content.height) return null;
  return {
    x: content.sourceX + (x / Math.max(1, content.width)) * content.sourceWidth,
    y: content.sourceY + (y / Math.max(1, content.height)) * content.sourceHeight,
    imageWidth: content.imageWidth,
    imageHeight: content.imageHeight,
    sourceX: content.sourceX,
    sourceY: content.sourceY,
    sourceWidth: content.sourceWidth,
    sourceHeight: content.sourceHeight
  };
}

function updateFloorMarkOverlay() {
  const overlay = el.floorMarkOverlay;
  if (!overlay) return;
  const content = previewContentRect();
  if (!content) {
    overlay.innerHTML = "";
    return;
  }
  const wrap = overlay.parentElement?.getBoundingClientRect();
  if (wrap) {
    overlay.style.left = `${content.left - wrap.left}px`;
    overlay.style.top = `${content.top - wrap.top}px`;
    overlay.style.width = `${content.width}px`;
    overlay.style.height = `${content.height}px`;
  }
  const imageWidth = content.imageWidth;
  const imageHeight = content.imageHeight;
  overlay.setAttribute("viewBox", `${content.sourceX} ${content.sourceY} ${content.sourceWidth} ${content.sourceHeight}`);
  const committed = floorMarks.map((mark, index) => {
    const [x1, y1] = mark.a || [0, 0];
    const [x2, y2] = mark.b || [0, 0];
    const label = index === 0 ? "L1" : (index === 1 ? "L2" : (index === 2 ? "CAP" : "CAP2"));
    return `<line x1="${esc(x1)}" y1="${esc(y1)}" x2="${esc(x2)}" y2="${esc(y2)}"></line><text x="${esc((x1 + x2) * 0.5)}" y="${esc((y1 + y2) * 0.5)}">${label}</text>`;
  }).join("");
  ensureWallRectangles();
  const wallMarkup = wallRectangles.map((slot, slotIndex) => {
    const corners = slot.corners || [];
    if (!corners.length) return "";
    const active = slotIndex === activeWallSlot;
    const closed = corners.length >= 4;
    const points = `${corners.map((p) => `${esc(p.x)},${esc(p.y)}`).join(" ")}${closed ? ` ${esc(corners[0].x)},${esc(corners[0].y)}` : ""}`;
    const center = corners.reduce((acc, p) => ({ x: acc.x + Number(p.x || 0), y: acc.y + Number(p.y || 0) }), { x: 0, y: 0 });
    center.x /= Math.max(1, corners.length);
    center.y /= Math.max(1, corners.length);
    const klass = `wall-rect${active ? " active" : ""}${slot.geometry?.valid ? " solved" : ""}`;
    const poly = `<polyline class="${klass}" points="${points}"></polyline><text class="wall-label" x="${esc(center.x)}" y="${esc(center.y)}">${esc(wallSlotLabel(slotIndex))}</text>`;
    const dots = corners.map((p, index) =>
      `<circle class="wall-corner${active ? " active" : ""}" cx="${esc(p.x)}" cy="${esc(p.y)}" r="${active ? 8 : 6}"></circle><text class="wall-corner-label" x="${esc(p.x + 8)}" y="${esc(p.y - 8)}">${esc(slotIndex + 1)}.${esc(index + 1)}</text>`
    ).join("");
    return poly + dots;
  }).join("");
  const cropRect = (() => {
    const activeCrop = cropDrawing && cropStart && cropEnd
      ? cropFromPoints(cropStart, cropEnd)
      : (checked("cropToggle") ? currentCropRect() : null);
    if (!activeCrop) return "";
    const x = activeCrop.x * imageWidth;
    const y = activeCrop.y * imageHeight;
    const w = activeCrop.w * imageWidth;
    const h = activeCrop.h * imageHeight;
    if (w <= 0 || h <= 0) return "";
    return [
      `<rect class="crop-shade" x="0" y="0" width="${esc(imageWidth)}" height="${esc(y)}"></rect>`,
      `<rect class="crop-shade" x="0" y="${esc(y + h)}" width="${esc(imageWidth)}" height="${esc(Math.max(0, imageHeight - y - h))}"></rect>`,
      `<rect class="crop-shade" x="0" y="${esc(y)}" width="${esc(x)}" height="${esc(h)}"></rect>`,
      `<rect class="crop-shade" x="${esc(x + w)}" y="${esc(y)}" width="${esc(Math.max(0, imageWidth - x - w))}" height="${esc(h)}"></rect>`,
      `<rect class="crop-rect" x="${esc(x)}" y="${esc(y)}" width="${esc(w)}" height="${esc(h)}"></rect>`,
      `<circle class="crop-handle" cx="${esc(x)}" cy="${esc(y)}" r="9"></circle>`,
      `<circle class="crop-handle" cx="${esc(x + w)}" cy="${esc(y)}" r="9"></circle>`,
      `<circle class="crop-handle" cx="${esc(x)}" cy="${esc(y + h)}" r="9"></circle>`,
      `<circle class="crop-handle" cx="${esc(x + w)}" cy="${esc(y + h)}" r="9"></circle>`
    ].join("");
  })();
  const preview = floorLineStart && floorLineEnd
    ? `<line class="preview-line" x1="${esc(floorLineStart.x)}" y1="${esc(floorLineStart.y)}" x2="${esc(floorLineEnd.x)}" y2="${esc(floorLineEnd.y)}"></line>`
    : "";
  overlay.innerHTML = cropRect + committed + wallMarkup + preview;
}


function roleDisplayName(role) {
  return String(role || "--").replace(/_/g, " ");
}

function roleStatusClass(status) {
  if (status === "sent" || status === "measured" || status === "anchored") return "good";
  if (status === "degraded" || status === "predicted" || status === "blocked") return "warn";
  if (status === "unmapped" || status === "disabled") return "muted";
  return "bad";
}

function classifyOutputRoleStatus(outputRole = {}, bodyRole = {}) {
  if (!outputRole.configured && !outputRole.enabled) return "disabled";
  if (outputRole.sent) return outputRole.degraded ? "degraded" : "sent";
  const reason = String(outputRole.reason || "");
  if (reason === "disabled_by_config" || reason === "unmapped") return "disabled";
  if (reason === "predicted_only_tracker_evidence") return "predicted";
  if (reason === "blocked_tracker_space") return "blocked";
  if (bodyRole?.measured) return "measured_not_sent";
  if (bodyRole?.evidence?.source === "anchor_held" || bodyRole?.visibility === "anchored") return "anchored";
  if (bodyRole?.predicted) return "predicted";
  return outputRole.valid ? "valid_not_sent" : "invalid";
}

function renderTrackerRoleDetails() {
  const host = el.trackerRoleDetails;
  if (!host) return;
  const bridgeEnabled = current.config?.steamvr_tracker_bridge?.enabled !== false;
  const bridgeRoles = Array.isArray(current.debug?.steamvr_bridge?.roles) ? current.debug.steamvr_bridge.roles : [];
  const oscRoles = Array.isArray(current.debug?.osc?.roles) ? current.debug.osc.roles : [];
  const outputRoles = bridgeEnabled && bridgeRoles.length ? bridgeRoles : oscRoles;
  const output = bridgeEnabled && bridgeRoles.length ? current.debug.steamvr_bridge : current.debug?.osc;
  const outputName = bridgeEnabled && bridgeRoles.length ? "SteamVR" : "Legacy OSC";
  const bodyRoles = Array.isArray(current.debug?.body_state?.roles) ? current.debug.body_state.roles : [];
  const bodyByRole = new Map(bodyRoles.map((role) => [String(role.role || ""), role]));
  if (!outputRoles.length) {
    host.innerHTML = `<div class="tracker-role-empty">No tracker role report yet. Start runtime from this menu to populate per-role send/evidence state.</div>`;
    return;
  }
  const rows = outputRoles.map((outputRole) => {
    const bodyRole = bodyByRole.get(String(outputRole.role || "")) || {};
    const status = classifyOutputRoleStatus(outputRole, bodyRole);
    const cls = roleStatusClass(status);
    const bodyConf = Number(bodyRole.confidence ?? 0);
    const outputConf = Number(outputRole.confidence ?? bodyConf);
    const evidence = bodyRole.evidence?.source || bodyRole.reason || "--";
    const reason = outputRole.reason || bodyRole.reason || "--";
    const sent = outputRole.sent ? "YES" : "NO";
    const valid = outputRole.valid ? "YES" : "NO";
    const mapped = outputName === "SteamVR"
      ? (outputRole.enabled ? "driver" : "off")
      : (outputRole.configured ? `#${outputRole.tracker_index}` : "off");
    return `<div class="tracker-role-row ${cls}">` +
      `<b>${esc(roleDisplayName(outputRole.role))}</b>` +
      `<span class="tracker-role-pill ${cls}">${esc(status)}</span>` +
      `<span>${esc(mapped)}</span>` +
      `<span>${esc(outputName)} sent ${sent} / valid ${valid}</span>` +
      `<span>conf ${fmt(outputConf, 2)}</span>` +
      `<span>${esc(evidence)}</span>` +
      `<span>${esc(reason)}</span>` +
    `</div>`;
  }).join("");
  const bodyDiag = current.debug?.body_state?.diagnostics || {};
  const sent = Number(output?.sent_tracker_count || 0);
  const skipped = Number(output?.skipped_tracker_count || 0);
  host.innerHTML = `<div class="tracker-role-title"><span>${esc(outputName)} tracker roles</span><b>${sent} sent / ${skipped} skipped</b><small>${esc(bodyDiag.measured_role_count ?? 0)} measured, ${esc(bodyDiag.predicted_joint_count ?? 0)} predicted, ${esc(bodyDiag.invalid_role_count ?? 0)} invalid body roles</small></div>` +
    `<div class="tracker-role-grid"><div class="tracker-role-head"><b>role</b><b>state</b><b>map</b><b>output</b><b>body</b><b>evidence</b><b>reason</b></div>${rows}</div>`;
}

const lastBodyOverlayBySlot = new Map();
const kBodyOverlayHoldMs = 1200;

function overlaySlotKey(view) {
  return String(view?.slot || "active");
}

function bodyOverlayForCameraIndex(index) {
  const overlay = current.body_overlay || current.debug?.body_overlay || {};
  if (Number(index) === Number(current.config?.camera_a?.device_index ?? 0)) return overlay.camera_a || null;
  if (Number(index) === Number(current.config?.camera_b?.device_index ?? 1)) return overlay.camera_b || null;
  return null;
}

function bodyOverlaySvgMarkup(view, extraClass = "", options = {}) {
  const slot = overlaySlotKey(view);
  const now = Date.now();
  const width = Math.max(1, Number(view?.preview_source_width || view?.preview_width || 1));
  const height = Math.max(1, Number(view?.preview_source_height || view?.preview_height || 1));
  const sourceX = Number(view?.preview_source_x || 0);
  const sourceY = Number(view?.preview_source_y || 0);
  const keypoints = (view?.keypoints || []).filter((kp) => kp?.present && Number(kp.confidence || 0) > 0.05);
  const usable = !!(view?.pose_available && view?.preview_width && view?.preview_height && keypoints.length >= 2);
  if (!usable) {
    const cached = lastBodyOverlayBySlot.get(slot);
    if (cached && now - cached.time <= (options.holdMs ?? kBodyOverlayHoldMs)) {
      return cached.markup.replace('body-pose-inline ', 'body-pose-inline is-held ');
    }
    return "";
  }
  const byIndex = new Map(keypoints.map((kp) => [Number(kp.index), kp]));
  const edgeMarkup = (view.skeleton || []).map((edge) => {
    const a = byIndex.get(Number(edge?.[0]));
    const b = byIndex.get(Number(edge?.[1]));
    if (!a || !b) return "";
    const conf = Math.min(Number(a.confidence || 0), Number(b.confidence || 0));
    const opacity = Math.max(0.20, Math.min(0.95, conf));
    return `<line x1="${fmt(a.x, 2)}" y1="${fmt(a.y, 2)}" x2="${fmt(b.x, 2)}" y2="${fmt(b.y, 2)}" style="opacity:${fmt(opacity, 2)}"></line>`;
  }).join("");
  const dots = keypoints.map((kp) => {
    const r = Math.max(2.5, Math.min(6, 2.5 + Number(kp.confidence || 0) * 3));
    return `<circle cx="${fmt(kp.x, 2)}" cy="${fmt(kp.y, 2)}" r="${fmt(r, 1)}"><title>${esc(kp.name || "keypoint")} ${fmt(kp.confidence, 2)}</title></circle>`;
  }).join("");
  const bbox = view.bbox?.valid
    ? `<rect x="${fmt(view.bbox.x, 2)}" y="${fmt(view.bbox.y, 2)}" width="${fmt(view.bbox.width, 2)}" height="${fmt(view.bbox.height, 2)}"></rect>`
    : "";
  const staleClass = view.stale ? " is-stale" : "";
  const markup = `<svg class="body-pose-inline ${extraClass}${staleClass}" viewBox="${sourceX} ${sourceY} ${width} ${height}" preserveAspectRatio="none" aria-hidden="true">${bbox}${edgeMarkup}${dots}</svg>`;
  lastBodyOverlayBySlot.set(slot, { time: now, markup });
  return markup;
}

function updateBodyPoseOverlay() {
  const svg = el.bodyPoseOverlay;
  if (!svg) return;
  const view = bodyOverlayForCameraIndex(cameraIndexForFloorSlot(activeFloorCameraKey()));
  const img = el.floorAssistPreview;
  const wrap = svg.parentElement?.getBoundingClientRect();
  const content = previewContentRect();
  if (!img?.getAttribute("src") || !wrap || content.width <= 0 || content.height <= 0) {
    svg.innerHTML = "";
    svg.removeAttribute("viewBox");
    return;
  }
  svg.style.left = `${content.left - wrap.left}px`;
  svg.style.top = `${content.top - wrap.top}px`;
  svg.style.width = `${content.width}px`;
  svg.style.height = `${content.height}px`;
  svg.setAttribute("viewBox", `${content.sourceX} ${content.sourceY} ${content.sourceWidth} ${content.sourceHeight}`);
  const markup = bodyOverlaySvgMarkup(view || { slot: activeFloorCameraKey() }, "floor-view");
  const held = markup.includes("is-held");
  svg.classList.toggle("is-stale", !!view?.stale || held);
  svg.classList.toggle("is-held", held);
  svg.innerHTML = markup.replace(/^<svg[^>]*>|<\/svg>$/g, "");
}

function updateFloorMarkReadout() {
  if (!el.floorMarkReadout) return;
  if (floorMarks.length === 0) {
    el.floorMarkReadout.textContent = "Draw plank lines for floor geometry, or draw one wall sample around a rectangular object. A known object size makes that sample a depth candidate.";
  } else if (floorMarks.length === 1) {
    el.floorMarkReadout.textContent = "long edge 1 captured · drag the opposite long edge of the same plank";
  } else if (floorMarks.length === 2) {
    el.floorMarkReadout.textContent = "both long edges captured · drag a short end cap across the plank";
  } else if (floorMarks.length === 3) {
    el.floorMarkReadout.textContent = "short end cap captured · optional: drag the opposite end cap for projective plank geometry, or click Apply";
  }
}

function setFloorMarkFailureReadout(message, nextAction = "") {
  if (!el.floorMarkReadout) return;
  const suffix = nextAction || (floorMarks.length >= 3
    ? "Adjust the last plank line or click Clear lines and redraw."
    : "Click Draw one plank or Draw wall sample to try a different calibration route.");
  el.floorMarkReadout.textContent = `${message}. ${suffix}`;
}

function addManualFloorLine(a, b) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const length = Math.hypot(dx, dy);
  if (!Number.isFinite(length) || length < 12) {
    setCommandStatus("Manual plank line too short; drag along the seam, not just a point", false);
    return false;
  }
  if (floorMarks.length === 0) {
    floorMarkImageWidth = a.imageWidth;
    floorMarkImageHeight = a.imageHeight;
  } else if (floorMarkImageWidth !== a.imageWidth || floorMarkImageHeight !== a.imageHeight) {
    setCommandStatus("Manual plank image size changed while drawing; clear and redraw on the current preview", false);
    return false;
  }
  const line = {
    a: [Number(a.x.toFixed(2)), Number(a.y.toFixed(2))],
    b: [Number(b.x.toFixed(2)), Number(b.y.toFixed(2))],
    strength: 1
  };
  floorMarks.push(line);
  configEventStore.append("FLOOR_GEOMETRY_MARKED", { line, image_width: floorMarkImageWidth, image_height: floorMarkImageHeight });
  updateFloorMarkOverlay();
  updateFloorMarkReadout();
  if (floorMarks.length === 1) {
    setCommandStatus("Long edge 1 captured; draw the opposite long edge");
    return true;
  }
  if (floorMarks.length === 2) {
    setCommandStatus("Plank width captured; draw one short end cap");
    return true;
  }
  if (floorMarks.length === 3) {
    setCommandStatus("Plank outline has enough data; optional opposite end cap improves projective geometry");
    applyManualFloorLines("Manual plank outline", { keepDrawing: true, newlyAdded: true });
    return true;
  }
  applyManualFloorLines("Manual plank quad", { keepDrawing: false, newlyAdded: true });
  return true;
}

async function applyManualFloorLines(sourceLabel = "Manual plank outline", options = {}) {
  const button = options.button || null;
  if (buttonIsBusy(button)) return false;
  const spacingA = Number(el.floorSpacingM?.value || 0);
  const spacingB = Number(el.floorPlankLengthM?.value || 0);
  if (floorMarks.length < 3) {
    setCommandStatus(`${sourceLabel} failed: draw two long plank edges plus one short end cap first`, false);
    setButtonResult(button, false, "draw at least 3 lines first");
    return false;
  }
  const img = el.floorAssistPreview;
  const cam = cameraConfigForFloorSlot();
  const imageWidth = Number(floorMarkImageWidth || cam.width || img?.naturalWidth || el.monocularImageWidth?.value || 0);
  const imageHeight = Number(floorMarkImageHeight || cam.height || img?.naturalHeight || el.monocularImageHeight?.value || 0);
  if (!Number.isFinite(imageWidth) || imageWidth <= 0 || !Number.isFinite(imageHeight) || imageHeight <= 0) {
    setCommandStatus(`${sourceLabel} failed: preview image size is unknown`, false);
    setButtonResult(button, false, "preview image size unknown");
    return false;
  }

  const requestSeq = ++floorCalibrationRequestSeq;
  const requestMarkCount = floorMarks.length;
  setButtonBusy(button, "calibrate floor geometry");
  const payload = {
    camera_slot: activeFloorCameraKey(),
    lines: floorMarks.slice(0, 8),
    image_width: imageWidth,
    image_height: imageHeight,
    floor_type: "manual_plank",
    family_a_spacing_m: Number.isFinite(spacingA) && spacingA > 0 ? spacingA : 0,
    family_b_spacing_m: Number.isFinite(spacingB) && spacingB > 0 ? spacingB : 0,
    camera_height_m: Number(el.monocularCameraHeight?.value || current.config?.tracking?.monocular?.camera_height_m || 0),
    horizontal_fov_deg: Number(el.monocularFov?.value || current.config?.tracking?.monocular?.horizontal_fov_deg || 0),
    floor_confidence: Number(el.floorConfidence?.value || 1)
  };
  const reply = await sendCommand("calibrateFloorGeometryBackend", payload, null, { actionId: button?.id || "floorManualApply", silentReply: true });
  if (reply?.error === "already pending") {
    setButtonResult(button, false, "calibration already pending");
    return false;
  }
  if (requestSeq !== floorCalibrationRequestSeq) {
    setButtonResult(button, false, "ignored stale solve");
    return true;
  }
  if (requestMarkCount !== floorMarks.length && requestMarkCount < 4) {
    setButtonResult(button, false, "ignored older partial solve");
    return true;
  }
  const ok = applyBackendFloorGeometry(reply?.result || reply, sourceLabel);
  const runtimeAccepted = ok && floorGeometryAcceptedForRuntime(floorGeometryAuto);
  setButtonResult(button, ok, ok
    ? (runtimeAccepted ? "accepted runtime geometry" : "captured degraded metric geometry")
    : (reply?.result?.status || reply?.error || "backend returned no usable floor geometry"));
  if (!ok) {
    const failure = reply?.result?.status || reply?.error || "backend returned no usable floor geometry";
    if (options.newlyAdded) {
      floorMarks.pop();
      updateFloorMarkOverlay();
      updateFloorMarkReadout();
    }
    setFloorMarkFailureReadout(`${sourceLabel} failed: ${failure}`, "Continue drawing a cleaner plank line, or switch to Draw wall sample for scoped depth evidence.");
    return false;
  }
  floorMarking = !!options.keepDrawing && floorMarks.length < 4;
  el.floorAssistPreview?.classList.toggle("is-marking", floorMarking);
  el.floorAssistPreview?.classList.remove("line-started");
  updateFloorMarkOverlay();
  if (floorMarking) {
    setCommandStatus(`${sourceLabel}: metric plank evidence captured; draw the opposite end cap only if you want projective geometry`);
  } else if (runtimeAccepted) {
    setCommandStatus(`${sourceLabel}: backend accepted as runtime floor geometry`);
  } else {
    setCommandStatus(`${sourceLabel}: evidence captured as draft geometry`);
  }
  return true;
}

async function refreshManualPlankBeforeSave(sourceLabel = "Manual plank refresh before save") {
  if (floorGeometryCleared || floorMarks.length < 3) return true;
  return applyManualFloorLines(sourceLabel, { keepDrawing: floorMarking && floorMarks.length < 4 });
}

async function applyManualWallRectangle(sourceLabel = "Wall sample") {
  const slot = currentWallRectangle();
  const corners = slot.corners || [];
  if (corners.length < 4) {
    setCommandStatus(`${sourceLabel} failed: click four corners of one rectangular object`, false);
    return false;
  }
  const img = el.floorAssistPreview;
  const cam = cameraConfigForFloorSlot();
  const imageWidth = Number(floorMarkImageWidth || cam.width || img?.naturalWidth || el.monocularImageWidth?.value || 0);
  const imageHeight = Number(floorMarkImageHeight || cam.height || img?.naturalHeight || el.monocularImageHeight?.value || 0);
  const wallWidth = Number(el.wallRectWidthM?.value || 0);
  const wallHeight = Number(el.wallRectHeightM?.value || 0);
  const wallAspect = Number(el.wallRectAspect?.value || 0);
  const payload = {
    camera_slot: activeFloorCameraKey(),
    wall_corners: corners.slice(0, 4).map((p) => ({ x: Number(p.x.toFixed(2)), y: Number(p.y.toFixed(2)) })),
    image_width: imageWidth,
    image_height: imageHeight,
    source: `manual_${activeFloorCameraKey()}_wall_rectangle_${activeWallSlot + 1}`,
    wall_width_m: Number.isFinite(wallWidth) && wallWidth > 0 ? wallWidth : 0,
    wall_height_m: Number.isFinite(wallHeight) && wallHeight > 0 ? wallHeight : 0,
    wall_aspect_ratio: Number.isFinite(wallAspect) && wallAspect > 0 ? wallAspect : 0,
    horizontal_fov_deg: Number(el.monocularFov?.value || current.config?.tracking?.monocular?.horizontal_fov_deg || 0)
  };
  const reply = await sendCommand("calibrateFloorGeometryBackend", payload, el.wallApplySelected, { actionId: "wallApplySelected", silentReply: true });
  const result = reply?.result || reply || {};
  const ok = !!result.ok || !!result.wall_geometry?.valid;
  slot.geometry = result.wall_geometry || null;
  slot.status = ok ? "solved" : "rejected";
  saveActiveWallSlots();
  const wallMessage = ok
    ? `${sourceLabel}: ${result.capability_reason || "wall evidence captured"}`
    : `${sourceLabel} failed: ${result.status || result.reason || reply?.error || "backend rejected wall sample"}`;
  setCommandStatus(wallMessage, ok);
  floorMarking = false;
  floorMarkMode = "plank";
  el.floorAssistPreview?.classList.remove("is-marking", "line-started");
  updateWallRectangleStatus();
  updateFloorMarkOverlay();
  if (!ok) {
    setFloorMarkFailureReadout(wallMessage, "Switched out of wall marking. Click Draw wall sample to retry this sample, or Draw one plank to use floor edges instead.");
  } else {
    updateFloorMarkReadout();
  }
  return ok;
}

async function saveVisibleConfig(sourceLabel = "Manual plank refresh before save") {
  const refreshed = await refreshManualPlankBeforeSave(sourceLabel);
  if (!refreshed) {
    setCommandStatus(`${sourceLabel}: draft plank refresh failed; saving last committed config`, false);
  }
  return sendCommand("saveConfig", readPayload(), null, { actionId: "saveConfig" });
}

function handleFloorPreviewPointerDown(event) {
  if (cropDrawing) {
    const point = cameraPointFromPreviewEvent(event);
    if (!point) {
      setCommandStatus("Crop draw failed: click inside the visible preview image", false);
      return;
    }
    cropStart = point;
    cropEnd = point;
    try { el.floorAssistPreview?.setPointerCapture?.(event.pointerId); } catch (_) {}
    el.floorAssistPreview?.classList.add("line-started");
    updateFloorMarkOverlay();
    event.preventDefault();
    return;
  }
  if (!floorMarking && checked("cropToggle")) {
    const point = cameraPointFromPreviewEvent(event);
    const mode = cropHitTest(point);
    if (mode) {
      cropAdjust = {
        mode,
        start: point,
        original: currentCropRect(),
        imageWidth: point.imageWidth,
        imageHeight: point.imageHeight
      };
      try { el.floorAssistPreview?.setPointerCapture?.(event.pointerId); } catch (_) {}
      el.floorAssistPreview?.classList.add("line-started");
      renderFloorAssist(trackingMode());
      setCommandStatus(mode === "move" ? "Moving Camera A crop" : "Resizing Camera A crop");
      event.preventDefault();
      return;
    }
  }
  if (!floorMarking) return;
  const point = cameraPointFromPreviewEvent(event);
  if (!point) {
    setCommandStatus("Manual plank line failed: click inside the visible preview image", false);
    return;
  }
  if (floorMarkMode === "wall") {
    const slot = currentWallRectangle();
    if (slot.corners.length === 0) {
      floorMarkImageWidth = point.imageWidth;
      floorMarkImageHeight = point.imageHeight;
    } else if (floorMarkImageWidth !== point.imageWidth || floorMarkImageHeight !== point.imageHeight) {
      setCommandStatus("Wall sample image size changed; clear and redraw on the current preview", false);
      return;
    }
    if (slot.corners.length >= 4) {
      slot.corners = [];
      slot.geometry = null;
      slot.status = "empty";
      wallCorners = slot.corners;
      floorMarkImageWidth = point.imageWidth;
      floorMarkImageHeight = point.imageHeight;
    }
    slot.corners.push({ x: Number(point.x.toFixed(2)), y: Number(point.y.toFixed(2)) });
    slot.status = slot.corners.length >= 4 ? "drawn" : "drawing";
    wallCorners = slot.corners;
    updateWallRectangleStatus();
    updateFloorMarkOverlay();
    setCommandStatus(slot.corners.length < 4
      ? `${wallSlotLabel()} corner ${slot.corners.length}/4 captured`
      : `${wallSlotLabel()} corners captured; click Solve sample to test this wall geometry`);
    updateActionPresentation();
    event.preventDefault();
    return;
  }
  if (floorMarks.length > 0 && (floorMarkImageWidth !== point.imageWidth || floorMarkImageHeight !== point.imageHeight)) {
    setCommandStatus("Manual plank image size changed; clear and redraw on the current preview", false);
    return;
  }
  floorLineStart = point;
  floorLineEnd = point;
  updateFloorMarkOverlay();
  try { el.floorAssistPreview?.setPointerCapture?.(event.pointerId); } catch (_) {}
  el.floorAssistPreview?.classList.add("line-started");
  setCommandStatus("Line started · release at the other end");
  event.preventDefault();
}

function handleFloorPreviewPointerMove(event) {
  if (cropDrawing && cropStart) {
    const point = cameraPointFromPreviewEvent(event);
    if (!point) return;
    cropEnd = point;
    updateFloorMarkOverlay();
    return;
  }
  if (cropAdjust) {
    const point = cameraPointFromPreviewEvent(event);
    if (!point) return;
    applyCropAdjustment(point);
    return;
  }
  if (!floorMarking || floorMarkMode === "wall" || !floorLineStart) return;
  const point = cameraPointFromPreviewEvent(event);
  if (!point) return;
  floorLineEnd = point;
  updateFloorMarkOverlay();
}

function handleFloorPreviewPointerUp(event) {
  if (cropDrawing && cropStart) {
    const point = cameraPointFromPreviewEvent(event);
    const start = cropStart;
    cropStart = null;
    cropEnd = null;
    cropDrawing = false;
    el.floorAssistPreview?.classList.remove("is-marking", "line-started");
    try { el.floorAssistPreview?.releasePointerCapture?.(event.pointerId); } catch (_) {}
    renderFloorAssist(trackingMode());
    const rect = cropFromPoints(start, point);
    if (!rect) {
      setCommandStatus("Crop draw failed: rectangle too small", false);
      updateFloorMarkOverlay();
      return;
    }
    setCropRect(rect, true);
    markDraftDirty();
    setCommandStatus("Camera A crop drawn · save config/restart runtime to apply");
    event.preventDefault();
    return;
  }
  if (cropAdjust) {
    const point = cameraPointFromPreviewEvent(event);
    if (point) applyCropAdjustment(point);
    cropAdjust = null;
    el.floorAssistPreview?.classList.remove("line-started");
    try { el.floorAssistPreview?.releasePointerCapture?.(event.pointerId); } catch (_) {}
    renderFloorAssist(trackingMode());
    markDraftDirty();
    setCommandStatus("Camera A crop resized · save config/restart runtime to apply");
    event.preventDefault();
    return;
  }
  if (!floorMarking || floorMarkMode === "wall" || !floorLineStart) return;
  const point = cameraPointFromPreviewEvent(event);
  const start = floorLineStart;
  floorLineStart = null;
  floorLineEnd = null;
  el.floorAssistPreview?.classList.remove("line-started");
  try { el.floorAssistPreview?.releasePointerCapture?.(event.pointerId); } catch (_) {}
  if (!point) {
    setCommandStatus("Manual plank line failed: release inside the visible preview image", false);
    updateFloorMarkOverlay();
    return;
  }
  addManualFloorLine(start, point);
  event.preventDefault();
}


function cancelFloorPreviewLine() {
  if (cropAdjust) {
    cropAdjust = null;
    el.floorAssistPreview?.classList.remove("line-started");
    renderFloorAssist(trackingMode());
    updateFloorMarkOverlay();
    setCommandStatus("Camera A crop resize cancelled");
    return;
  }
  if (!floorLineStart) return;
  floorLineStart = null;
  floorLineEnd = null;
  el.floorAssistPreview?.classList.remove("line-started");
  updateFloorMarkOverlay();
  setCommandStatus("Manual plank line cancelled");
}


// Final floor-geometry calibration is backend-owned. The UI only sends capture/spacing hints,
// displays the backend evidence, and saves backend-owned calibration payloads.

function floorReferenceYFromFamily(family, imageWidth, imageHeight) {
  if (!family?.valid) return 0;
  const orientation = Number(family.orientation_rad || 0);
  const nx = -Math.sin(orientation);
  const ny = Math.cos(orientation);
  if (!Number.isFinite(ny) || Math.abs(ny) < 0.20) return 0;
  const centerX = 0.5 * Math.max(1, Number(imageWidth || current.config?.tracking?.monocular?.image_width || current.config?.camera_a?.width || 1280));
  const y = (Number(family.reference_rho_px || 0) - centerX * nx) / ny;
  return Math.max(0, Math.min(Math.max(1, Number(imageHeight || current.config?.tracking?.monocular?.image_height || current.config?.camera_a?.height || 720)), y));
}

function hydrateFloorGeometryFromStatus() {
  if (floorGeometryCleared) return;
  const byCamera = current.calibration?.floor_geometry_by_camera || {};
  for (const key of ["camera_a", "camera_b"]) {
    const saved = byCamera[key] || (key === "camera_a" ? current.calibration?.floor_geometry : null);
    if (!saved || floorGeometryByCamera[key]) continue;
    const g = JSON.parse(JSON.stringify(saved));
    if (!g.valid) continue;
    g.backend_owned = true;
    floorGeometryByCamera[key] = g;
    if (key === "camera_a") floorGeometryAuto = g;
  }
}

function hydrateWallRectanglesFromStatus() {
  const byCamera = current.calibration?.wall_rectangles_by_camera || {};
  for (const key of ["camera_a", "camera_b"]) {
    if (wallRectanglesByCamera[key]) continue;
    const saved = Array.isArray(byCamera[key]) ? byCamera[key] : (key === "camera_a" && Array.isArray(current.calibration?.wall_rectangles) ? current.calibration.wall_rectangles : []);
    const slots = emptyWallSlots();
    let nextCompactIndex = 0;
    for (const geometry of saved) {
      const hintedIndex = wallSlotIndexFromGeometry(geometry);
      let index = hintedIndex;
      if (index === null || slots[index]?.geometry) {
        while (nextCompactIndex < wallRectangleSlotCount && slots[nextCompactIndex]?.geometry) nextCompactIndex++;
        index = nextCompactIndex;
      }
      if (index < 0 || index >= wallRectangleSlotCount) continue;
      const corners = wallCornersFromGeometry(geometry);
      slots[index] = {
        index,
        corners,
        geometry: geometry ? JSON.parse(JSON.stringify(geometry)) : null,
        status: geometry?.valid ? "solved" : (corners.length >= 4 ? "drawn" : "empty")
      };
      nextCompactIndex = Math.max(nextCompactIndex, index + 1);
    }
    wallRectanglesByCamera[key] = slots;
  }
  loadActiveWallSlots();
  updateWallRectangleStatus();
  updateFloorMarkOverlay();
}
function applyBackendFloorGeometry(result, sourceLabel) {
  if (!result?.ok || !result.floor_geometry) {
    const msg = `${sourceLabel} failed: ${result?.status || result?.error || "backend returned no floor geometry"}`;
    setCommandStatus(msg, false);
    setFloorMarkFailureReadout(msg, "Try wall sample if plank edges keep failing, or clear and redraw the plank with longer edge lines.");
    return false;
  }
  const g = JSON.parse(JSON.stringify(result.floor_geometry));
  g.source = g.source || result.source || "manual_plank_outline";
  g.image_width = Number(result.image_width || g.image_width || 0);
  g.image_height = Number(result.image_height || g.image_height || 0);
  g.manual_plank = result.manual_plank || g.manual_plank || null;
  g.backend_owned = true;
  const runtimeAccepted = floorGeometryAcceptedForRuntime(g);
  const activeStillRuntime = floorGeometryAcceptedForRuntime(activeFloorGeometry());
  if (!runtimeAccepted && activeStillRuntime) {
    floorGeometryDraft = g;
    uiStore.dispatch({ type: "FLOOR_GEOMETRY", patch: { draft: g } });
    configEventStore.append("FLOOR_GEOMETRY_DRAFTED", { status: g.status || g.floor_type || "draft" });
  } else {
    setActiveFloorGeometry(g);
    floorGeometryDraft = null;
    floorGeometryCleared = false;
    uiStore.dispatch({ type: "FLOOR_GEOMETRY", patch: { auto: g, draft: null, cleared: false } });
    configEventStore.append("FLOOR_GEOMETRY_APPLIED", { camera_slot: activeFloorCameraKey(), status: g.status || g.floor_type || "active", confidence: g.metric_scale_confidence });
  }

  const family = g.family_a?.valid ? g.family_a : (g.family_b?.valid ? g.family_b : null);
  if (family) {
    setValue("floorSpacingPx", Number(family.spacing_px || 0));
    setValue("floorReferenceY", floorReferenceYFromFamily(family, g.image_width, g.image_height));
  }
  const displayedConfidence = Number(g.metric_scale_confidence || g.family_a?.confidence || g.pattern?.confidence || 0);
  setValue("floorConfidence", displayedConfidence);
  setPressed("floorAssistToggle", scalarFloorSpacingAllowed(g) && !!g.family_a?.metric_spacing_valid);
  markDraftDirty();
  const seamCount = Number(result.candidate_count || 0);
  const homography = g.homography_valid ? `projective ${fmt(g.homography_reprojection_error_px, 2)}px` : (g.homography_reason || "no projective homography");
  if (el.floorMarkReadout) {
    const hasMetricWidth = Number(g.family_a?.spacing_m || result.spacing_m || 0) > 0;
    const metric = g.family_a?.metric_spacing_valid
      ? `${fmt(g.family_a?.spacing_px || result.spacing_px || 0, 1)} px per ${fmt(g.family_a?.spacing_m || result.spacing_m || 0, 3)} m`
      : (hasMetricWidth
        ? `${fmt(g.family_a?.spacing_px || result.spacing_px || 0, 1)} px wide · metric width captured; scalar assist not used`
        : `${fmt(g.family_a?.spacing_px || result.spacing_px || 0, 1)} px wide · metric scale not set`);
    el.floorMarkReadout.textContent = `manual plank: ${g.family_count || 0} families · ${seamCount} drawn edges · ${metric} · ${homography}`;
  }
  setCommandStatus(`${activeFloorCameraLabel()} ${sourceLabel}: ${result.status || g.reason || "backend floor geometry ready"}`);
  renderFloorAssist(trackingMode());
  return true;
}


function scalarFloorSpacingAllowed(g) {
  return !!(g?.family_a?.valid &&
    Number(g.family_a?.spacing_m || 0) > 0 &&
    Number(g.family_a?.spacing_px || 0) > 0);
}

function savedManualPlankEditable(g) {
  return !!(g?.manual_plank?.valid ||
    g?.source === "manual_plank_outline" ||
    g?.floor_type === "manual_plank");
}

function buildFloorGeometryPayload() {
  const active = activeFloorGeometry();
  if (!active || active.backend_owned !== true) return null;
  const g = JSON.parse(JSON.stringify(active));
  const editableManualOutline = floorMarks.length >= 3 || savedManualPlankEditable(g);
  const spacingA = Number(el.floorSpacingM?.value);
  const scalarAllowed = scalarFloorSpacingAllowed(g);
  const projectiveQuad = !!(g.homography_valid && g.two_axis_grid_valid);
  if (editableManualOutline && Number.isFinite(spacingA) && spacingA > 0 && g.family_a?.valid) {
    g.family_a.spacing_m = spacingA;
    g.family_a.metric_spacing_valid = true;
    if (g.manual_plank) {
      g.manual_plank.width_m = spacingA;
      g.manual_plank.width_metric_valid = true;
      g.manual_plank.scalar_floor_spacing_usable = true;
    }
    const editedConfidence = Number(el.floorConfidence?.value || 0);
    const familyConfidence = Number(g.family_a?.confidence || 0);
    if (scalarAllowed || projectiveQuad || savedManualPlankEditable(g)) {
      g.metric_scale_confidence = Math.max(Number(g.metric_scale_confidence || 0), editedConfidence, familyConfidence);
    }
    g.planted_drift_axis_confidence = Number(g.metric_scale_confidence || 0);
  }
  const spacingB = Number(el.floorPlankLengthM?.value || current.config?.tracking?.monocular?.floor_second_axis_spacing_m || 0);
  if (editableManualOutline && Number.isFinite(spacingB) && spacingB > 0 && g.family_b?.valid && Number(g.family_b.spacing_px || 0) > 0 && g.two_axis_grid_valid) {
    g.family_b.spacing_m = spacingB;
    g.family_b.metric_spacing_valid = true;
    if (g.manual_plank) {
      g.manual_plank.length_m = spacingB;
      g.manual_plank.length_metric_valid = true;
    }
  }
  g.distortion ||= { available: false, valid: false, confidence: 0, reason: "manual_preview_edges_do_not_estimate_lens_distortion_projective_perspective_is_homography" };
  return g;
}

function buildFloorGeometryByCameraPayload() {
  const activeKey = activeFloorCameraKey();
  if (activeKey === "camera_a") floorGeometryByCamera.camera_a = floorGeometryAuto;
  const out = {};
  for (const key of ["camera_a", "camera_b"]) {
    const geometry = floorGeometryByCamera[key];
    if (geometry?.backend_owned === true) {
      out[key] = key === activeKey ? buildFloorGeometryPayload() : JSON.parse(JSON.stringify(geometry));
    }
  }
  return Object.keys(out).length ? out : null;
}

function buildWallRectanglesPayload() {
  saveActiveWallSlots();
  ensureWallRectangles();
  const slots = wallRectanglesByCamera.camera_a || wallRectangles;
  return slots
    .filter((slot) => slot.geometry?.valid)
    .map((slot) => {
      const geometry = JSON.parse(JSON.stringify(slot.geometry));
      geometry.source = geometry.source || `manual_wall_rectangle_${Number(slot.index || 0) + 1}`;
      geometry.image_corners = wallCornersFromGeometry(geometry).length >= 4
        ? geometry.image_corners
        : (slot.corners || []).slice(0, 4).map((p) => ({ x: Number(p.x), y: Number(p.y) }));
      geometry.applied_to_runtime = false;
      return geometry;
    });
}

function buildWallRectanglesByCameraPayload() {
  saveActiveWallSlots();
  const out = {};
  for (const key of ["camera_a", "camera_b"]) {
    const slots = wallRectanglesByCamera[key] || [];
    out[key] = slots
      .filter((slot) => slot.geometry?.valid)
      .map((slot) => {
        const geometry = JSON.parse(JSON.stringify(slot.geometry));
        geometry.source = geometry.source || `manual_${key}_wall_rectangle_${Number(slot.index || 0) + 1}`;
        geometry.image_corners = wallCornersFromGeometry(geometry).length >= 4
          ? geometry.image_corners
          : (slot.corners || []).slice(0, 4).map((p) => ({ x: Number(p.x), y: Number(p.y) }));
        geometry.applied_to_runtime = false;
        return geometry;
      });
  }
  return out;
}

function modeText(camera) {
  if (!camera) return "--";
  return `${camera.width || "--"}x${camera.height || "--"} @ ${fmt(camera.fps, 1)} / ${(camera.backend || "auto").toUpperCase()}`;
}

function pushMetric(key, value) {
  const number = Number(value);
  if (!Number.isFinite(number)) return;
  uiStore.dispatch({ type: "METRIC_PUSH", key, value: number });
  metricHistory = uiStore.getState().metricHistory || {};
}

function percentile(values, q) {
  if (!values?.length) return 0;
  const sorted = [...values].sort((a, b) => a - b);
  const index = Math.min(sorted.length - 1, Math.max(0, Math.round((sorted.length - 1) * q)));
  return sorted[index];
}

function average(values) {
  return values?.length ? values.reduce((sum, value) => sum + value, 0) / values.length : 0;
}

function sparkline(values, width = 96, height = 22) {
  if (!values?.length) return "";
  const min = Math.min(...values);
  const max = Math.max(...values);
  const range = Math.max(1e-6, max - min);
  const step = values.length > 1 ? width / (values.length - 1) : width;
  const points = values.map((value, index) => {
    const x = index * step;
    const y = height - ((value - min) / range) * height;
    return `${fmt(x, 1)},${fmt(y, 1)}`;
  }).join(" ");
  return `<svg class="sparkline" viewBox="0 0 ${width} ${height}" preserveAspectRatio="none" aria-hidden="true"><polyline points="${points}"></polyline></svg>`;
}

function metricCell(label, key, now, decimals = 2) {
  pushMetric(key, now);
  const values = metricHistory[key] || [];
  const rendered = metricCellHtml({ label, value: Number(now), decimals, history: values });
  return rendered.replace("</div>", `${sparkline(values)}<span>max ${fmt(Math.max(0, ...values), decimals)}</span></div>`);
}

function webviewAvailable() {
  return commandBus.available();
}

function sendCommand(command, payload = {}, button = null, options = {}) {
  const intent = actionIntentForButton(button, command, options);
  if (buttonIsBusy(button)) {
    return Promise.resolve({ ok: false, error: "already pending", actionLabel: intent?.label });
  }
  if (button && intent?.id) button.dataset.actionId = intent.id;
  if (button) setButtonBusy(button, options.pendingLabel || intent?.pendingLabel || command);
  return commandBus.send(command, payload, {
    button,
    actionId: intent?.id || options.actionId || null,
    actionLabel: intent?.label || options.actionLabel || commandLabel(command),
    silentReply: !!options.silentReply,
    timeoutMs: options.timeoutMs,
    maxRetries: options.maxRetries,
    retryable: options.retryable
  }).then((reply) => {
    if (reply?.ok === false && !reply?.result && button) {
      setButtonResult(button, false, reply.error || "failed");
    }
    return reply;
  });
}

function readPayload() {
  return {
    tracking_mode: trackingMode(),
    camera_a_source: current.config?.camera_a?.source || "opencv",
    camera_b_source: current.config?.camera_b?.source || "opencv",
    camera_a: Number(el.cameraA?.value),
    camera_b: Number(el.cameraB?.value),
    camera_a_roi_enabled: checked("cropToggle"),
    camera_a_roi_normalized: true,
    camera_a_roi: [currentCropRect().x, currentCropRect().y, currentCropRect().w, currentCropRect().h],
    inference_device: ["cpu", "directml", "directml_strict"].includes(el.inferenceDevice?.value) ? el.inferenceDevice.value : "directml",
    model_path: el.modelPath?.value ?? "",
    calibration_path: el.calibrationPath?.value ?? "",
    log_file: el.logFile?.value ?? "",
    recording_dir: el.recordingDir?.value ?? "",
    replay_log_path: el.replayLogPath?.value ?? "",
    stale_frame_timeout_ms: Number(el.staleTimeout?.value),
    latest_frame_skew_tolerance_ms: Number(el.latestSkew?.value),
    max_frame_skew_ms: Number(el.maxSkew?.value),
    min_triangulated_seed_count: Number(el.minSeeds?.value),
    max_mean_reprojection_error_px: Number(el.maxReproj?.value),
    stereo_monocular_fallback_enabled: checked("stereoFallbackToggle"),
    body_calibration_enabled: checked("bodyCalToggle"),
    body_calibration_auto_persist: checked("bodyCalAutoPersistToggle"),
    body_calibration_required_seconds: Number(el.bodyCalRequiredSeconds?.value),
    body_calibration_min_overall_confidence: Number(el.bodyCalMinConfidence?.value),
    body_calibration_max_segment_cv: Number(el.bodyCalMaxCv?.value),
    room_depth_map_enabled: checked("roomDepthMapToggle"),
    room_depth_map_collect_only: checked("roomDepthMapCollectOnlyToggle"),
    room_depth_map_min_accepted_frames_before_active: Number(el.roomDepthMapFrames?.value),
    monocular_image_width: Number(el.monocularImageWidth?.value),
    monocular_image_height: Number(el.monocularImageHeight?.value),
    monocular_horizontal_fov_deg: Number(el.monocularFov?.value),
    monocular_user_height_m: Number(el.monocularUserHeight?.value),
    monocular_camera_height_m: Number(el.monocularCameraHeight?.value),
    monocular_default_depth_m: Number(el.monocularDefaultDepth?.value),
    monocular_depth_confidence_scale: Number(el.monocularDepthConfidence?.value),
    monocular_min_keypoint_confidence: Number(el.monocularMinKeypointConfidence?.value),
    monocular_min_seed_count: Number(el.monocularMinSeeds?.value),
    monocular_floor_scale_assist_enabled: checked("floorAssistToggle"),
    monocular_floor_depth_line_spacing_m: Number(el.floorSpacingM?.value),
    monocular_floor_depth_line_spacing_px: Number(el.floorSpacingPx?.value),
    monocular_floor_depth_reference_y_px: Number(el.floorReferenceY?.value),
    monocular_floor_depth_reference_m: Number(el.floorReferenceM?.value),
    monocular_floor_depth_confidence: Number(el.floorConfidence?.value),
    monocular_floor_geometry_calibration_enabled: floorGeometryCleared
      ? false
      : (!!floorGeometryAuto || !!current.config?.tracking?.monocular?.floor_geometry_calibration_enabled),
    monocular_floor_geometry_type: floorGeometryAuto?.floor_type || current.config?.tracking?.monocular?.floor_geometry_type || "unknown",
    monocular_floor_second_axis_spacing_m: Number(el.floorPlankLengthM?.value || current.config?.tracking?.monocular?.floor_second_axis_spacing_m || 0),
    monocular_floor_geometry_confidence: Number(floorGeometryAuto?.metric_scale_confidence ?? current.config?.tracking?.monocular?.floor_geometry_confidence ?? 0),
    floor_geometry_auto: (buildFloorGeometryByCameraPayload() || {}).camera_a || null,
    floor_geometry_by_camera: buildFloorGeometryByCameraPayload(),
    floor_geometry_clear: floorGeometryCleared,
    wall_rectangles_auto: buildWallRectanglesPayload(),
    wall_rectangles_by_camera: buildWallRectanglesByCameraPayload(),
    use_legacy_solver: checked("legacySolverToggle"),
    enable_replay_recording: checked("replayToggle"),
    motion_consistency_enabled: checked("motionToggle"),
    // One-Euro fields are sealed internal knobs with no UI controls.
    // Pass-through from config so saves don't silently zero them.
    motion_one_euro_enabled: current.config?.tracking?.motion_consistency?.one_euro_enabled ?? true,
    motion_one_euro_min_cutoff_hz: current.config?.tracking?.motion_consistency?.one_euro_min_cutoff_hz ?? 1.2,
    motion_one_euro_beta: current.config?.tracking?.motion_consistency?.one_euro_beta ?? 0.018,
    motion_one_euro_d_cutoff_hz: current.config?.tracking?.motion_consistency?.one_euro_d_cutoff_hz ?? 1.0,
    contact_root_correction_gain: Number(el.contactRootGain?.value),
    contact_root_max_correction_m: Number(el.contactRootMaxCorrection?.value),
    contact_root_max_residual_m: Number(el.contactRootMaxResidual?.value),
    contact_root_min_support_confidence: Number(el.contactRootMinSupport?.value),
    osc_enabled: checked("oscToggle"),
    osc_host: el.oscHost?.value ?? "",
    osc_port: Number(el.oscPort?.value),
    osc_send_rotations: checked("oscRotToggle"),
    osc_min_confidence: Number(el.oscMinConfidence?.value),
    steamvr_tracker_bridge_enabled: checked("steamVrBridgeToggle"),
    steamvr_tracker_bridge_host: el.steamVrBridgeHost?.value ?? "127.0.0.1",
    steamvr_tracker_bridge_port: Number(el.steamVrBridgePort?.value),
    steamvr_tracker_bridge_min_confidence: Number(el.steamVrBridgeMinConfidence?.value),
    steamvr_tracker_bridge_send_chest: checked("steamVrBridgeChestToggle"),
    steamvr_tracker_bridge_send_elbows: checked("steamVrBridgeElbowsToggle"),
    steamvr_tracker_bridge_send_knees: checked("steamVrBridgeKneesToggle"),
    pelvis_tracker_index: Number(el.pelvisTracker?.value),
    left_foot_tracker_index: Number(el.leftFootTracker?.value),
    right_foot_tracker_index: Number(el.rightFootTracker?.value),
    chest_tracker_index: Number(el.chestTracker?.value),
    left_elbow_tracker_index: Number(el.leftElbowTracker?.value),
    right_elbow_tracker_index: Number(el.rightElbowTracker?.value),
    left_knee_tracker_index: Number(el.leftKneeTracker?.value),
    right_knee_tracker_index: Number(el.rightKneeTracker?.value),
    hmd_mode: ["null", "json_file"].includes(el.hmdMode?.value)
      ? el.hmdMode.value
      : "null",
    hmd_path: el.hmdPath?.value ?? "",
    tracker_space_transform_valid: checked("trackerSpaceValidToggle"),
    tracker_space_offset_x: Number(el.trackerOffsetX?.value),
    tracker_space_offset_y: Number(el.trackerOffsetY?.value),
    tracker_space_offset_z: Number(el.trackerOffsetZ?.value),
    tracker_space_rotation_x: Number(el.trackerRotX?.value),
    tracker_space_rotation_y: Number(el.trackerRotY?.value),
    tracker_space_rotation_z: Number(el.trackerRotZ?.value),
    tracker_space_rotation_w: Number(el.trackerRotW?.value),
    tracker_space_scale: Number(el.trackerScale?.value)
  };
}

function render(state) {
  current = state || {};
  const bridgeReady = webviewAvailable();
  setBridge(bridgeReady ? "CONNECTED" : "FAILED", bridgeReady);

  const modelOk = !!current.model?.exists;
  const running = !!current.runtime?.running;
  const savedMode = current.config?.tracking?.mode === "monocular" ? "monocular" : "stereo";
  const selectedMode = el.trackingMode?.value === "monocular" ? "monocular" : "stereo";
  const mode = syncInputs ? savedMode : selectedMode;
  const monocularMode = mode === "monocular";
  el.runtimePill.textContent = running ? "RUNNING" : "STOPPED";
  el.runtimePill.className = running ? "good" : (modelOk ? "warn" : "bad");
  el.configPath.textContent = current.config_path || "";

  const cameraNeed = monocularMode ? 1 : 2;
  const cameraLabel = monocularMode
    ? `${current.config?.camera_a?.device_index ?? "--"}`
    : `${current.config?.camera_a?.device_index ?? "--"} / ${current.config?.camera_b?.device_index ?? "--"}`;
  const setupOk = monocularMode ? true : !!current.calibration?.tracking_ready;
  const calibrationLabel = monocularMode ? "MONO SCALE" : (current.calibration?.tracking_ready ? "STEREO READY" : "STEREO CALIB NEEDED");
  const oscDebug = current.debug?.osc || {};
  const oscEnabled = oscDebug.enabled ?? current.config?.osc?.enabled;
  const oscStatus = oscDebug.status || (oscEnabled ? "enabled" : "disabled");
  const oscClass = !oscEnabled ? "muted" : (oscDebug.last_send_ok === false ? "bad" : (oscDebug.open ? "good" : "warn"));
  const bodyStateDiag = current.debug?.body_state?.diagnostics || {};
  const solveStatus = bodyStateDiag.active
    ? (bodyStateDiag.degraded ? "degraded" : "active")
    : (running ? "inactive" : "stopped");
  const identityStatus = bodyStateDiag.left_right_identity_uncertain
    ? "uncertain"
    : (bodyStateDiag.left_right_identity_stable ? "stable" : "waiting");
  const occlusionStatus = bodyStateDiag.occlusion_prediction_active ? "occlusion" : "clear";
  const contactStatus = bodyStateDiag.contact_lock_active ? "locked" : "free";
  const activeDevice = current.model?.active_device || current.debug?.model_active_device || current.config?.inference?.device || "cpu";
  const epFallback = !!(current.model?.ep_fallback || current.debug?.model_ep_fallback);
  const depthPostprocess = !!current.config?.tracking?.depth_postprocess_enabled;
  const depthPostprocessNote = depthPostprocess ? `<span>3D depth postprocess</span><b class="warn">ON (config-only)</b>` : "";
  el.modelBox.innerHTML = `<span>state</span><b class="${modelOk ? "good" : "bad"}">${modelOk ? "found" : "missing"}</b><span>device</span><b class="${epFallback ? "warn" : "good"}">${esc(activeDevice)}${epFallback ? " fallback" : ""}</b><span>path</span><b>${esc(current.model?.path)}</b><span>folder</span><b>${esc(current.model?.folder)}</b>${depthPostprocessNote}`;
  el.runtimeBox.innerHTML = `<span>state</span><b class="${running ? "good" : "warn"}">${running ? "running" : "stopped"}</b><span>last error</span><b>${esc(current.runtime?.last_error || current.debug?.last_error || "none")}</b><span>last exit</span><b>${esc(current.runtime?.last_exit_code ?? "--")}</b>`;
  const phoneSite = current.phone_site || {};
  const phoneSiteUrl = typeof phoneSite.url === "string" ? phoneSite.url : "";
  const phoneSiteStatus = String(phoneSite.status || (phoneSite.enabled === true ? "enabled" : "disabled"));
  const phoneSiteStatusKey = phoneSiteStatus.toLowerCase();
  const phoneSiteOn = phoneSite.enabled === true && !!phoneSiteUrl;
  const phoneTarget = phoneSite.target || phoneSite.configured_target || "camera A network listener";
  const phoneUpdated = phoneSite.updated || "--";
  el.phoneSiteBox.innerHTML = `<span>state</span><b class="${phoneSiteStatusClass(phoneSiteStatus, phoneSiteOn)}">${esc(phoneSiteStatus)}</b><span>url</span><b>${esc(phoneSiteUrl || "--")}</b><span>target</span><b>${esc(phoneTarget)}</b><span>apk</span><b class="${phoneSite.apk ? "good" : "warn"}">${phoneSite.apk ? "available" : "missing"}</b><span>updated</span><b>${esc(phoneUpdated)}</b>`;
  el.phoneSiteUrl.textContent = phoneSiteUrl ? `Open this on iPhone/Android: ${phoneSiteUrl}` : `site: ${phoneSiteStatus}`;
  const phoneEnablePending = commandPending("enablePhoneWebCamera");
  const phoneOpenPending = commandPending("openPhoneWebCamera");
  const phoneDisablePending = commandPending("disablePhoneWebCamera");
  setActionControl(el.phoneSiteEnable, {
    enabled: bridgeReady && phoneSite.launcher_available !== false && phoneSiteStatusKey !== "enabled",
    pending: phoneEnablePending,
    done: phoneSiteOn,
    reason: !bridgeReady
      ? actionIntents.phoneSiteEnable.unavailableLabel
      : (phoneSiteOn
        ? "Phone camera site is already enabled"
        : (phoneSite.launcher_available === false ? "Phone camera launcher is unavailable" : "Enable the phone camera web site"))
  });
  setActionControl(el.phoneSiteOpen, {
    enabled: bridgeReady && !!phoneSiteUrl,
    pending: phoneOpenPending,
    active: !!phoneSiteUrl && !phoneSiteOn,
    reason: !bridgeReady ? actionIntents.phoneSiteOpen.unavailableLabel : (phoneSiteUrl ? "Open the phone camera web site" : "Enable the phone camera site first")
  });
  setActionControl(el.phoneSiteDisable, {
    enabled: bridgeReady && phoneSite.stop_available !== false && phoneSiteStatusKey !== "disabled" && !phoneDisablePending,
    pending: phoneDisablePending,
    warn: phoneSiteOn,
    reason: !bridgeReady ? actionIntents.phoneSiteDisable.unavailableLabel : (phoneSiteStatusKey === "disabled" ? "Phone camera site is already disabled" : "Stop the phone camera web site")
  });

  if (syncInputs) {
    const scanned = current.cameras?.items || [];
    const opts = scanned.length
      ? scanned.map((camera) => option(camera.index, `Camera ${camera.index} / ${camera.opened ? "OPEN" : "NO SIGNAL"}`)).join("")
      : [0, 1, 2, 3, 4, 5, 6, 7, 8, 9].map((index) => option(index, `Camera ${index}`)).join("");
    el.cameraA.innerHTML = opts;
    el.cameraB.innerHTML = opts;
    setValue("trackingMode", mode);
    setValue("cameraA", current.config?.camera_a?.device_index ?? 0);
    setValue("cameraB", current.config?.camera_b?.device_index ?? 1);
    const crop = current.config?.camera_a?.initial_roi || [0, 0, 1, 1];
    setCropRect({
      x: Number(crop[0] ?? 0),
      y: Number(crop[1] ?? 0),
      w: Number(crop[2] ?? 1),
      h: Number(crop[3] ?? 1)
    }, current.config?.camera_a?.initial_roi_enabled === true);
    setValue("inferenceDevice", ["cpu", "directml", "directml_strict"].includes(current.config?.inference?.device) ? current.config.inference.device : "directml");
    setValue("modelPath", current.config?.tracking?.model_path ?? "models/rtmw-dw-x-l-cocktail14-384x288.onnx");
    setValue("calibrationPath", current.config?.tracking?.calibration_path ?? "calib/default.json");
    setValue("logFile", current.config?.app?.log_file ?? "bodytracker.log");
    setValue("recordingDir", current.config?.app?.recording_dir ?? "recordings");
    setValue("replayLogPath", current.config?.debug?.replay_log_path ?? "");
    setValue("staleTimeout", current.config?.tracking?.stale_frame_timeout_ms ?? 250);
    setValue("latestSkew", current.config?.tracking?.latest_frame_skew_tolerance_ms ?? current.config?.tracking?.max_frame_skew_ms ?? 18);
    setValue("maxSkew", current.config?.tracking?.max_frame_skew_ms ?? current.config?.tracking?.latest_frame_skew_tolerance_ms ?? 18);
    setValue("minSeeds", current.config?.tracking?.min_triangulated_seed_count ?? 3);
    setValue("maxReproj", current.config?.tracking?.max_mean_reprojection_error_px ?? 45);
    setPressed("stereoFallbackToggle", current.config?.tracking?.stereo_monocular_fallback_enabled === true);
    const bodyCal = current.config?.tracking?.body_calibration || {};
    setPressed("bodyCalToggle", bodyCal.enabled === true);
    setPressed("bodyCalAutoPersistToggle", bodyCal.auto_persist !== false);
    setValue("bodyCalRequiredSeconds", bodyCal.required_seconds ?? 2.5);
    setValue("bodyCalMinConfidence", bodyCal.min_overall_confidence ?? 0.55);
    setValue("bodyCalMaxCv", bodyCal.max_segment_cv ?? 0.12);
    const roomMapConfig = current.config?.tracking?.room_depth_map || {};
    setPressed("roomDepthMapToggle", roomMapConfig.enabled !== false);
    setPressed("roomDepthMapCollectOnlyToggle", roomMapConfig.collect_only !== false);
    setValue("roomDepthMapFrames", roomMapConfig.min_accepted_frames_before_active ?? 1000);
    const roomMap = current.debug?.solver?.room_depth_map || current.debug?.room_depth_map || {};
    const roomState = roomMap.state || (roomMapConfig.enabled === false ? "disabled" : "warming_up");
    const accepted = Number(roomMap.accepted_frames ?? 0);
    const rejected = Number(roomMap.rejected_frames ?? 0);
    const coverage = Number(roomMap.coverage ?? 0);
    const targetFrames = Number(roomMapConfig.min_accepted_frames_before_active ?? 1000);
    const progressText = Number.isFinite(accepted) && Number.isFinite(targetFrames)
      ? `${accepted}/${targetFrames} accepted`
      : "-- accepted";
    if (el.roomDepthMapStatus) {
      el.roomDepthMapStatus.innerHTML = `room map: <b class="${roomState === "active" || roomState === "collect_only_active" ? "good" : (roomState === "disabled" ? "warn" : "")}">${esc(roomState)}</b> · ${esc(progressText)} · rejected ${esc(rejected)} · coverage ${fmt(coverage * 100, 1)}%${roomMap.last_rejection_reason ? ` · last ${esc(roomMap.last_rejection_reason)}` : ""}`;
    }
    const mono = current.config?.tracking?.monocular || {};
    if (current.calibration?.floor_geometry) {
      hydrateFloorGeometryFromStatus();
    } else {
      floorGeometryAuto = null;
      floorGeometryDraft = null;
      floorGeometryCleared = false;
    }
    hydrateWallRectanglesFromStatus();
    setValue("monocularImageWidth", mono.image_width ?? current.config?.camera_a?.width ?? 1280);
    setValue("monocularImageHeight", mono.image_height ?? current.config?.camera_a?.height ?? 720);
    setValue("monocularFov", mono.horizontal_fov_deg ?? 70);
    setValue("monocularUserHeight", mono.user_height_m ?? 1.70);
    setValue("monocularCameraHeight", mono.camera_height_m ?? 1.20);
    setValue("monocularDefaultDepth", mono.default_depth_m ?? 2.20);
    setValue("monocularDepthConfidence", mono.depth_confidence_scale ?? 0.55);
    setValue("monocularMinKeypointConfidence", mono.min_keypoint_confidence ?? 0.05);
    setValue("monocularMinSeeds", mono.min_seed_count ?? 4);
    setPressed("floorAssistToggle", !!mono.floor_scale_assist_enabled);
    setValue("floorSpacingM", mono.floor_depth_line_spacing_m ?? 0);
    setValue("floorPlankLengthM", mono.floor_second_axis_spacing_m ?? 0);
    setValue("floorSpacingPx", mono.floor_depth_line_spacing_px ?? 0);
    setValue("floorReferenceY", mono.floor_depth_reference_y_px ?? 0);
    setValue("floorReferenceM", mono.floor_depth_reference_m ?? 0);
    setValue("floorConfidence", mono.floor_depth_confidence ?? 0.65);
    if (floorGeometryAuto?.backend_owned) {
      const family = floorGeometryAuto.family_a?.valid ? floorGeometryAuto.family_a : (floorGeometryAuto.family_b?.valid ? floorGeometryAuto.family_b : null);
      if (family) {
        setValue("floorSpacingPx", Number(family.spacing_px || mono.floor_depth_line_spacing_px || 0));
        setValue("floorReferenceY", floorReferenceYFromFamily(family, mono.image_width, mono.image_height));
      }
      if (Number(floorGeometryAuto.family_a?.spacing_m || 0) > 0) {
        setValue("floorSpacingM", Number(floorGeometryAuto.family_a.spacing_m));
      }
      if (Number(floorGeometryAuto.family_b?.spacing_m || 0) > 0) {
        setValue("floorPlankLengthM", Number(floorGeometryAuto.family_b.spacing_m));
      }
      setValue("floorConfidence", Number(floorGeometryAuto.metric_scale_confidence ?? mono.floor_geometry_confidence ?? mono.floor_depth_confidence ?? 0.65));
      setPressed("floorAssistToggle", scalarFloorSpacingAllowed(floorGeometryAuto) && !!floorGeometryAuto.family_a?.metric_spacing_valid);
    }
    setValue("steamVrBridgeHost", current.config?.steamvr_tracker_bridge?.target_address ?? "127.0.0.1");
    setValue("steamVrBridgePort", current.config?.steamvr_tracker_bridge?.target_port ?? 39560);
    setValue("steamVrBridgeMinConfidence", current.config?.steamvr_tracker_bridge?.min_confidence ?? 0.20);
    setPressed("steamVrBridgeChestToggle", current.config?.steamvr_tracker_bridge?.send_chest !== false);
    setPressed("steamVrBridgeElbowsToggle", current.config?.steamvr_tracker_bridge?.send_elbows !== false);
    setPressed("steamVrBridgeKneesToggle", !!current.config?.steamvr_tracker_bridge?.send_knees);
    setValue("oscHost", current.config?.osc?.target_address ?? "127.0.0.1");
    setValue("oscPort", current.config?.osc?.target_port ?? 9000);
    setValue("oscMinConfidence", current.config?.osc?.min_confidence ?? 0.20);
    setValue("pelvisTracker", current.config?.osc?.pelvis_tracker_index ?? 1);
    setValue("leftFootTracker", current.config?.osc?.left_foot_tracker_index ?? 2);
    setValue("rightFootTracker", current.config?.osc?.right_foot_tracker_index ?? 3);
    setValue("chestTracker", current.config?.osc?.chest_tracker_index ?? 4);
    setValue("leftElbowTracker", current.config?.osc?.left_elbow_tracker_index ?? 5);
    setValue("rightElbowTracker", current.config?.osc?.right_elbow_tracker_index ?? 6);
    setValue("leftKneeTracker", current.config?.osc?.left_knee_tracker_index ?? 0);
    setValue("rightKneeTracker", current.config?.osc?.right_knee_tracker_index ?? 0);
    const configuredHmdMode = current.config?.hmd?.mode;
    setValue("hmdMode", configuredHmdMode === "json_file" ? "json_file" : "null");
    setValue("hmdPath", current.config?.hmd?.pose_json_path ?? "hmd_pose.json");
    const osc = current.config?.osc || {};
    const trackerSpace = current.tracker_space || {};
    const showingManualFallback = trackerSpace.manual_fallback_valid &&
      (trackerSpace.source === "manual" || trackerSpace.source === "manual_json_file");
    const offset = showingManualFallback
      ? (trackerSpace.position_offset || trackerSpace.manual_fallback_position_offset || [0, 0, 0])
      : (osc.tracker_space_position_offset || [0, 0, 0]);
    const rotation = showingManualFallback
      ? (trackerSpace.rotation || trackerSpace.manual_fallback_rotation || [0, 0, 0, 1])
      : (osc.tracker_space_rotation || [0, 0, 0, 1]);
    setValue("trackerOffsetX", offset[0] ?? 0);
    setValue("trackerOffsetY", offset[1] ?? 0);
    setValue("trackerOffsetZ", offset[2] ?? 0);
    setValue("trackerRotX", rotation[0] ?? 0);
    setValue("trackerRotY", rotation[1] ?? 0);
    setValue("trackerRotZ", rotation[2] ?? 0);
    setValue("trackerRotW", rotation[3] ?? 1);
    setValue("trackerScale", showingManualFallback
      ? (trackerSpace.scale ?? trackerSpace.manual_fallback_scale ?? 1.0)
      : (osc.tracker_space_scale ?? 1.0));
    setPressed("trackerSpaceValidToggle", showingManualFallback
      ? !!trackerSpace.manual_fallback_valid
      : !!osc.tracker_space_transform_valid);
    setPressed("steamVrBridgeToggle", current.config?.steamvr_tracker_bridge?.enabled !== false);
    setPressed("oscToggle", !!current.config?.osc?.enabled);
    setPressed("oscRotToggle", current.config?.osc?.send_rotations !== false);
    setPressed("legacySolverToggle", !!current.config?.tracking?.use_legacy_solver);
    setPressed("replayToggle", !!current.config?.tracking?.enable_replay_recording);
    const motion = current.config?.tracking?.motion_consistency || {};
    setValue("contactRootGain", motion.contact_root_correction_gain ?? 0.20);
    setValue("contactRootMaxCorrection", motion.contact_root_max_correction_m ?? 0.015);
    setValue("contactRootMaxResidual", motion.contact_root_max_residual_m ?? 0.035);
    setValue("contactRootMinSupport", motion.contact_root_min_support_confidence ?? 0.75);
    setPressed("motionToggle", motion.enabled !== false);
  }

  setDisabled("cameraB", monocularMode);
  renderFloorAssist(mode);
  renderTrackerSpace();
  renderSteamVrOutputStatus();
  renderSteamVrAlignment();

  const floorOverviewStatus = currentFloorAssistStatus(mode);
  el.overview.innerHTML = [
    ["mode", mode.toUpperCase(), monocularMode ? "warn" : "good"],
    ["model", modelOk ? "FOUND" : "MISSING", statusClass(modelOk)],
    ["cameras", cameraLabel, current.cameras?.open_count >= cameraNeed ? "good" : "warn"],
    ["calibration", calibrationLabel, setupOk ? "good" : "warn"],
    ["3d solve", solveStatus, solveStatusClass(solveStatus)],
    ["identity", identityStatus, identityStatus === "stable" ? "good" : (identityStatus === "uncertain" ? "warn" : "muted")],
    ["occlusion", occlusionStatus, occlusionStatus === "occlusion" ? "warn" : "good"],
    ["contact", contactStatus, contactStatus === "locked" ? "good" : "muted"],
    ["floor", floorOverviewStatus, floorStatusClass(floorOverviewStatus)],
    ["tracker space", current.tracker_space?.status || "pending", trackerSpaceStatusClass(current.tracker_space?.status || "pending")],
    ["tracking", running ? (current.debug?.degradation_mode || "RUNNING") : "STOPPED", running ? "good" : "warn"],
    ["osc", oscStatus, oscClass]
  ].map((item) => `<div><span>${item[0]}</span><b class="${item[2]}">${esc(item[1])}</b></div>`).join("");

  const bodyCalibrationStatus = current.debug?.body_calibration || {};
  if (el.bodyCalReadout) {
    const autoPersist = current.config?.tracking?.body_calibration?.auto_persist !== false;
    const persistStatus = bodyCalibrationStatus.enabled
      ? (bodyCalibrationStatus.persist_status || (bodyCalibrationStatus.saved_this_frame ? "saved_this_frame" : "not_complete"))
      : "disabled";
    const completeNoAutoPersist = bodyCalibrationStatus.enabled && bodyCalibrationStatus.complete && !autoPersist;
    const calibrationStatus = bodyCalibrationStatus.enabled
      ? (completeNoAutoPersist
        ? `${bodyCalibrationStatus.complete ? "complete" : "collecting"} · ${persistStatus} · ${esc(bodyCalibrationStatus.accepted_samples ?? 0)} samples · conf ${fmt(bodyCalibrationStatus.overall_confidence, 2)} · save config manually to persist (auto-persist is off)`
        : `${bodyCalibrationStatus.complete ? "complete" : "collecting"} · ${persistStatus} · ${esc(bodyCalibrationStatus.accepted_samples ?? 0)} samples · conf ${fmt(bodyCalibrationStatus.overall_confidence, 2)}`)
      : "disabled";
    const displayPersistClass = completeNoAutoPersist ? "warn" : bodyCalibrationPersistClass(persistStatus);
    el.bodyCalReadout.innerHTML = `body calibration: <b class="${displayPersistClass}">${calibrationStatus}</b>`;
  }

  const items = current.cameras?.items || [];
  const camA = items.find((camera) => camera.index === Number(el.cameraA.value)) || current.config?.camera_a;
  const camB = items.find((camera) => camera.index === Number(el.cameraB.value)) || current.config?.camera_b;
  el.modeA.textContent = modeText(camA);
  el.modeB.textContent = monocularMode ? "disabled in monocular mode" : modeText(camB);
  el.cameraStatus.textContent = current.cameras?.scanning ? "scanning cameras..." : (current.cameras?.status || "camera scan ready");
  const cameraSetPending = !bridgeReady || commandPending("saveConfig", "scanCameras", "startRuntime");
  el.cameraRows.innerHTML = items.map((camera) => {
    const isA = Number(el.cameraA.value) === camera.index;
    const isB = Number(el.cameraB.value) === camera.index;
    const overlay = bodyOverlaySvgMarkup(bodyOverlayForCameraIndex(camera.index), "camera-row");
    const preview = camera.preview
      ? `<span class="preview-stack"><img class="preview-img" src="${esc(camera.preview)}" alt="">${overlay}</span>`
      : "--";
    const assignment = `${isA ? "CAM A " : ""}${(!monocularMode && isB) ? "CAM B" : ""}`.trim() || "--";
    const disableA = cameraSetPending || !camera.opened || isA;
    const disableB = cameraSetPending || monocularMode || !camera.opened || isB;
    const rowClass = `${isA ? "selected-a" : ""} ${(!monocularMode && isB) ? "selected-b" : ""}`.trim();
    const titleA = !bridgeReady ? "Camera assignment needs the desktop backend bridge" : (!camera.opened ? "Camera has no signal" : (isA ? "Camera is already assigned to A" : "Assign this camera to A"));
    const titleB = !bridgeReady ? "Camera assignment needs the desktop backend bridge" : (monocularMode ? "Camera B is disabled in monocular mode" : (!camera.opened ? "Camera has no signal" : (isB ? "Camera is already assigned to B" : "Assign this camera to B")));
    return `<div class="row ${esc(rowClass)}"><span>${camera.index}</span><span class="pill"><i class="dot ${camera.opened ? "on" : ""}"></i>${camera.opened ? "OPEN" : "NO SIGNAL"}</span><span>${preview}</span><span>${esc((camera.backend || "--").toUpperCase())}</span><span>${camera.opened ? `${camera.width}x${camera.height} @ ${fmt(camera.fps, 1)}` : "--"}</span><span>${esc(assignment)}</span><button class="${isA ? "is-done" : ""}" ${disableA ? "disabled" : ""} title="${esc(titleA)}" data-a="${camera.index}" type="button">${isA ? "A active" : "Use A"}</button><button class="${(!monocularMode && isB) ? "is-done" : ""}" ${disableB ? "disabled" : ""} title="${esc(titleB)}" data-b="${camera.index}" type="button">${(!monocularMode && isB) ? "B active" : "Use B"}</button></div>`;
  }).join("");

  el.telemetry.innerHTML = [
    metricCell("capture ms", "capture", current.debug?.capture_ms, 2),
    metricCell("pair ms", "pair", current.debug?.frame_pair_ms, 2),
    metricCell("preprocess ms", "preprocess", current.debug?.preprocess_ms, 2),
    metricCell("onnx ms", "onnx", current.debug?.onnx_ms ?? current.debug?.inference_ms, 2),
    metricCell("decode ms", "decode", current.debug?.decode_ms, 2),
    metricCell("solver ms", "solver", current.debug?.solver_ms ?? current.debug?.pipeline_ms, 2),
    metricCell("osc ms", "osc", current.debug?.osc_ms, 2),
    metricCell("total ms", "total", current.debug?.total_ms, 2)
  ].join("");

  const dbg = current.debug || {};
  const leftSupport = dbg.support?.left_foot || {};
  const rightSupport = dbg.support?.right_foot || {};
  const motionFilter = dbg.motion_filter || {};
  const contactRoot = motionFilter.contact_root || {};
  const solver = dbg.solver || {};
  const constraints = solver.support_constraints || {};
  const leftConstraints = constraints.left_foot || {};
  const rightConstraints = constraints.right_foot || {};
  const epipolar = solver.epipolar || dbg.stereo?.epipolar || {};
  const epipolarStatus = epipolar.status || solver.epipolar_status || "--";
  const epipolarChecked = epipolar.checked_count ?? solver.epipolar_checked_count ?? 0;
  const epipolarHardMismatch = epipolar.hard_mismatch_count ?? solver.epipolar_hard_mismatch_count ?? 0;
  const epipolarRejected = epipolar.pair_rejected_count ?? solver.epipolar_pair_rejected_count ?? 0;
  const epipolarSoftened = epipolar.degraded_pair_softened_count ?? solver.epipolar_degraded_pair_softened_count ?? 0;
  const epipolarMeanError = epipolar.mean_error_px ?? solver.mean_epipolar_error_px;
  const epipolarMeanErrorAnisotropic = epipolar.mean_error_px_anisotropic ?? solver.mean_epipolar_error_px_anisotropic;
  const epipolarMeanConfidence = epipolar.mean_confidence ?? solver.mean_epipolar_confidence;
  const floorGeometry = dbg.floor_geometry || {};
  const bodyCalibration = dbg.body_calibration || {};
  const bodyState = dbg.body_state || {};
  const bodyDiag = bodyState.diagnostics || {};
  const bodyPersistStatus = bodyCalibration.persist_status || (bodyCalibration.saved_this_frame ? "saved_this_frame" : "--");
  const oscRoles = Array.isArray(oscDebug.active_roles) && oscDebug.active_roles.length
    ? oscDebug.active_roles.join(", ")
    : "--";
  const oscTarget = `${oscDebug.target_address || current.config?.osc?.target_address || "--"}:${oscDebug.target_port || current.config?.osc?.target_port || "--"}`;
  const oscOpenAttempts = Array.isArray(oscDebug.open_attempts) && oscDebug.open_attempts.length
    ? oscDebug.open_attempts.slice(-2).join(" | ")
    : "--";
  const profiler = dbg.profiler || {};
  const perfStages = profiler.stages || {};
  const perfStage = (name) => {
    const stage = perfStages[name] || {};
    const label = `${fmt(stage.p95_ms, 2)} / ${fmt(stage.budget_ms, 2)}ms`;
    return stage.over_budget ? `OVER ${label}` : label;
  };
  el.solverMetrics.innerHTML =
    `<div class="mini-metrics"><div><span>objective evals</span><b>${esc(dbg.objective_evaluations ?? 0)}</b></div><div><span>passes</span><b>${esc(dbg.coordinate_passes ?? 0)}</b></div><div><span>early stop</span><b>${dbg.optimizer_early_stopped ? "YES" : "NO"}</b></div><div><span>solve split</span><b>${fmt(dbg.preliminary_solve_ms, 2)} / ${fmt(dbg.final_solve_ms, 2)}</b></div></div>` +
    `<div class="mini-metrics"><div><span>left support</span><b>${esc(leftSupport.phase || leftSupport.type || "--")} / ${fmt(leftSupport.support_confidence, 2)}</b></div><div><span>right support</span><b>${esc(rightSupport.phase || rightSupport.type || "--")} / ${fmt(rightSupport.support_confidence, 2)}</b></div><div><span>contact root</span><b>${esc(contactRoot.reason || "--")} / ${fmt(contactRoot.correction_m, 3)}m</b></div><div><span>foot evidence L/R</span><b>${fmt(solver.left_foot_contact_confidence, 2)} / ${fmt(solver.right_foot_contact_confidence, 2)}</b></div></div>` +
    `<div class="mini-metrics"><div><span>motion root</span><b>${esc(motionFilter.root?.decision || "--")} / ${esc(motionFilter.root?.reason || "--")}</b></div><div><span>motion left</span><b>${esc(motionFilter.left_foot?.decision || "--")} / ${esc(motionFilter.left_foot?.reason || "--")}</b></div><div><span>motion right</span><b>${esc(motionFilter.right_foot?.decision || "--")} / ${esc(motionFilter.right_foot?.reason || "--")}</b></div><div><span>root residual</span><b>${fmt(constraints.root_support?.residual, 3)}m × ${fmt(constraints.root_support?.weight, 2)}</b></div></div>` +
    `<div class="mini-metrics"><div><span>bone residual</span><b>${fmt(constraints.bone_length?.residual, 3)}m × ${fmt(constraints.bone_length?.weight, 2)}</b></div><div><span>left slide</span><b>${fmt(leftConstraints.sliding_velocity?.residual, 3)}m/s × ${fmt(leftConstraints.sliding_velocity?.weight, 2)}</b></div><div><span>right slide</span><b>${fmt(rightConstraints.sliding_velocity?.residual, 3)}m/s × ${fmt(rightConstraints.sliding_velocity?.weight, 2)}</b></div><div><span>transition L/R</span><b>${fmt(leftConstraints.transition_quality, 2)} / ${fmt(rightConstraints.transition_quality, 2)}</b></div></div>` +
    `<div class="mini-metrics"><div><span>epipolar status</span><b>${esc(epipolarStatus)}</b></div><div><span>epipolar checked/mismatch</span><b>${esc(epipolarChecked)} / ${esc(epipolarHardMismatch)}</b></div><div><span>epipolar rejected/softened</span><b>${esc(epipolarRejected)} / ${esc(epipolarSoftened)}</b></div><div><span>epipolar iso/aniso/conf</span><b>${fmt(epipolarMeanError, 2)}px / ${fmt(epipolarMeanErrorAnisotropic, 2)}px / ${fmt(epipolarMeanConfidence, 2)}</b></div></div>` +
    `<div class="mini-metrics"><div><span>floor assist</span><b>${esc(current.floor_assist?.status || "--")}</b></div><div><span>floor source</span><b>${esc(current.floor_assist?.source || solver.monocular_scale_source || "--")}</b></div><div><span>floor depth</span><b>${fmt(current.floor_assist?.depth_m ?? solver.monocular_floor_assist_depth_m, 2)}m</b></div><div><span>floor confidence</span><b>${fmt(current.floor_assist?.confidence ?? solver.monocular_floor_assist_confidence, 2)}</b></div></div>` +
    `<div class="mini-metrics"><div><span>body state</span><b>${bodyDiag.active ? "ACTIVE" : "--"} / ${fmt(bodyDiag.role_output_confidence, 2)}</b></div><div><span>identity conf</span><b>${fmt(bodyDiag.identity_confidence, 2)}</b></div><div><span>role truth</span><b>${esc(bodyDiag.measured_role_count ?? 0)} measured / ${esc(bodyDiag.predicted_joint_count ?? 0)} predicted / ${esc(bodyDiag.anchored_role_count ?? 0)} anchored</b></div><div><span>degraded/stale/invalid</span><b>${esc(bodyDiag.degraded_role_count ?? 0)} / ${esc(bodyDiag.stale_aged_role_count ?? 0)} / ${esc(bodyDiag.invalid_role_count ?? 0)}</b></div></div>` +
    `<div class="mini-metrics"><div><span>floor geometry</span><b>${floorGeometry.valid ? "VALID" : "--"} / ${esc(floorGeometry.family_count ?? 0)} families</b></div><div><span>homography</span><b>${floorGeometry.homography_valid ? "YES" : "NO"} / ${fmt(floorGeometry.homography_reprojection_error_px, 2)}px</b></div><div><span>calib conf</span><b>${fmt(constraints.floor_calibration_weight, 2)} floor / ${fmt(constraints.leg_length_weight, 2)} leg</b></div><div><span>body persist</span><b>${esc(bodyPersistStatus)}</b></div></div>` +
    `<div class="mini-metrics"><div><span>tracker space</span><b>${esc(current.tracker_space?.status || "--")}</b></div><div><span>tracker source</span><b>${esc(current.tracker_space?.source || "--")}</b></div><div><span>osc blocked</span><b>${current.tracker_space?.osc_blocked ? "YES" : "NO"}</b></div><div><span>scale</span><b>${fmt(current.tracker_space?.scale, 3)}</b></div></div>` +
    `<div class="mini-metrics"><div><span>steamvr bridge</span><b>${esc(current.debug?.steamvr_bridge?.status || "--")}</b></div><div><span>steamvr sent</span><b>${esc(current.debug?.steamvr_bridge?.sent_tracker_count ?? 0)} trackers / ${esc(current.debug?.steamvr_bridge?.sent_message_count ?? 0)} msg</b></div><div><span>steamvr target</span><b>${esc((current.debug?.steamvr_bridge?.target_address || "--") + ":" + (current.debug?.steamvr_bridge?.target_port || "--"))}</b></div><div><span>steamvr error</span><b>${esc(current.debug?.steamvr_bridge?.last_error || "--")}</b></div></div>` +
    `<div class="mini-metrics"><div><span>legacy osc status</span><b>${esc(oscStatus)}</b></div><div><span>legacy osc target</span><b>${esc(oscTarget)}</b></div><div><span>legacy osc sent</span><b>${esc(oscDebug.sent_tracker_count ?? 0)} trackers / ${esc(oscDebug.sent_message_count ?? 0)} msg</b></div><div><span>legacy osc roles</span><b>${esc(oscRoles)}</b></div></div>` +
    `<div class="mini-metrics"><div><span>osc open attempts</span><b>${esc(oscOpenAttempts)}</b></div><div><span>osc last error</span><b>${esc(oscDebug.last_error || "--")}</b></div><div><span>osc tracker space</span><b>${esc(oscDebug.tracker_space_state || "--")}</b></div><div><span>osc stale</span><b>${oscDebug.tracker_space_stale ? "YES" : "NO"}</b></div></div>` +
    `<div class="mini-metrics"><div><span>perf samples</span><b>${esc(profiler.sample_count ?? 0)}</b></div><div><span>bottleneck</span><b>${esc(profiler.bottleneck_stage || "none")} × ${fmt(profiler.bottleneck_ratio, 2)}</b></div><div><span>total p95/budget</span><b>${esc(perfStage("total"))}</b></div><div><span>pipeline p95/budget</span><b>${esc(perfStage("pipeline"))}</b></div></div>` +
    `<div class="mini-metrics"><div><span>inference p95/budget</span><b>${esc(perfStage("inference"))}</b></div><div><span>solver p95/budget</span><b>${esc(perfStage("solver"))}</b></div><div><span>osc p95/budget</span><b>${esc(perfStage("osc"))}</b></div><div><span>ui p95/budget</span><b>${esc(perfStage("ui_publish"))}</b></div></div>`;
  renderTrackerRoleDetails();

  const stateLines = [
    current.runtime?.last_error,
    current.debug?.last_error,
    current.calibration?.summary
  ].filter(Boolean);
  el.log.textContent = [...logLines, ...stateLines].join("\n") || JSON.stringify(current.debug || {}, null, 2);
  const runtimeStartPending = commandPending("startRuntime", "saveConfig");
  const runtimeStopPending = commandPending("stopRuntime");
  // Start saves the visible draft before launching. Do not block that path using
  // the old saved model state; a stopped user may be fixing a missing/bad model
  // path right now. If the draft path is still bad, startRuntime reports the
  // real model-load error after the save.
  const runtimeStartBlocked = !modelOk && !localDraftDirty;
  setActionControl(el.startRuntime, {
    enabled: bridgeReady && !running && !runtimeStartBlocked && !runtimeStopPending,
    pending: runtimeStartPending,
    active: bridgeReady && !running && !runtimeStartBlocked,
    done: running,
    reason: !bridgeReady
      ? actionIntents.startRuntime.unavailableLabel
      : (running
        ? "Runtime is already running"
        : (runtimeStartBlocked ? "Model asset is missing" : (setupOk ? "Save visible config, then start runtime" : "Start runtime; stereo calibration is not ready yet"))),
    label: running ? "Runtime running" : "Start"
  });
  setActionControl(el.stopRuntime, {
    enabled: bridgeReady && running && !runtimeStartPending,
    pending: runtimeStopPending,
    active: running,
    reason: !bridgeReady ? actionIntents.stopRuntime.unavailableLabel : (running ? "Stop runtime" : "Runtime is already stopped")
  });
  setActionControl(el.scanCams, {
    enabled: bridgeReady && !commandPending("scanCameras") && !running,
    pending: commandPending("scanCameras"),
    reason: !bridgeReady ? actionIntents.scanCams.unavailableLabel : (running ? "Stop runtime before rescanning cameras" : "Scan cameras")
  });
  setActionControl(el.saveConfig, {
    enabled: bridgeReady && !commandPending("saveConfig"),
    pending: commandPending("saveConfig"),
    active: bridgeReady && localDraftDirty,
    done: bridgeReady && !localDraftDirty,
    reason: !bridgeReady ? actionIntents.saveConfig.unavailableLabel : (localDraftDirty ? "Save modified visible configuration" : "Visible configuration matches backend state")
  });
  setActionControl(el.saveAdvanced, {
    enabled: bridgeReady && !commandPending("saveConfig"),
    pending: commandPending("saveConfig"),
    active: bridgeReady && localDraftDirty,
    done: bridgeReady && !localDraftDirty,
    reason: !bridgeReady ? actionIntents.saveAdvanced.unavailableLabel : (localDraftDirty ? "Save modified advanced configuration" : "Advanced configuration matches backend state")
  });
  renderPet(current);
  updateActionPresentation();
}



function mergeSteamVrReplyState(result) {
  if (!result || (!result.steamvr_alignment && !result.tracker_space)) return false;
  const merged = Object.assign({}, current || {});
  if (result.steamvr_alignment) merged.steamvr_alignment = result.steamvr_alignment;
  if (result.tracker_space) merged.tracker_space = result.tracker_space;
  const next = uiStore.dispatch({ type: "STATE_RECEIVED", state: merged });
  render(next.backend);
  const svr = next.backend?.steamvr_alignment || {};
  maybeRecordSteamVrTrigger(svr.provider || {}, svr.session || {});
  return true;
}

function handleReply(message) {
  const handled = commandBus.handleReply(message);
  if (!handled) return;
  const { entry, result, ok } = handled;
  const envelope = entry.envelope || {};
  const statusClass = result.status_class || (ok ? "ok" : "failed_fatal");
  const replyLabel = envelope.actionLabel || actionLabel(entry.command, envelope.actionId);
  const replyDetail = humanizeCommandText(result.warning || result.status || message.error || statusClass || (ok ? "ok" : "failed"));
  const text = `${replyLabel}: ${replyDetail}`;
  const statusState = (statusClass === "warning" || statusClass === "degraded") ? "warn" : ok;
  setBridge(ok ? "CONNECTED" : "ERROR", ok);
  if (!envelope.silentReply) {
    setCommandStatus(text, statusState);
    setButtonResult(envelope.button, ok, text);
  }
  if (ok && entry.command === "saveConfig") {
    const clearedFloorGeometry = floorGeometryCleared;
    clearDraftDirty();
    if (clearedFloorGeometry) {
      floorGeometryCleared = false;
      floorGeometryAuto = null;
      floorGeometryDraft = null;
      uiStore.dispatch({ type: "FLOOR_GEOMETRY", patch: { auto: null, draft: null, cleared: false } });
    }
  }
  if (entry.command && entry.command.startsWith("steamVrAlignment")) {
    mergeSteamVrReplyState(result);
  }
}

function handleMessage(event) {
  const message = typeof event.data === "string" ? JSON.parse(event.data) : event.data;
  if (message?.type === "state") {
    const next = uiStore.dispatch({ type: "STATE_RECEIVED", state: message.state });
    render(next.backend);
    const svr = message.state?.steamvr_alignment || {};
    maybeRecordSteamVrTrigger(svr.provider || {}, svr.session || {});
  } else if (message?.type === "reply") {
    handleReply(message);
  }
}

function bindAction(id, definition = {}) {
  const node = el[id];
  if (!node) {
    appendLog(`Missing UI node #${id}`);
    return;
  }
  const intent = registerActionIntent(id, definition);
  node.dataset.actionId = intent.id;
  node.addEventListener("click", () => sendCommand(intent.command, definition.payload?.() || {}, node, { actionId: intent.id }));
}

function bindClick(id, command, payload = () => ({})) {
  bindAction(id, { command, payload });
}

function bindToggle(id) {
  const node = el[id];
  if (!node) return appendLog(`Missing UI node #${id}`);
  node.addEventListener("click", () => {
    toggle(id);
    markControlTouched(node);
    markDraftDirty();
    const label = controlLabel(node);
    setCommandStatus(`${label}: ${checked(id) ? "ON" : "OFF"} · save config to apply`);
    if (id === "steamVrBridgeToggle" || id === "steamVrBridgeChestToggle" || id === "steamVrBridgeElbowsToggle" || id === "steamVrBridgeKneesToggle") renderSteamVrOutputStatus();
    if (id === "trackerSpaceValidToggle" || id === "oscToggle") renderTrackerSpace();
    if (id === "floorAssistToggle" || id === "stereoFallbackToggle") renderFloorAssist(trackingMode());
  });
}

function bindEvents() {
  for (const [id, intent] of Object.entries(actionIntents)) {
    if (el[id]) el[id].dataset.actionId = intent.id;
  }
  el.petAction?.addEventListener("click", runPetAction);
  el.petBody?.addEventListener("click", (event) => {
    if (petDrag?.moved) return;
    event.preventDefault();
    el.petBubble?.classList.toggle("open");
    renderPet(current);
  });
  el.petBody?.addEventListener("pointerdown", (event) => {
    if (!el.desktopPet) return;
    const rect = el.desktopPet.getBoundingClientRect();
    petDrag = {
      pointerId: event.pointerId,
      offsetX: event.clientX - rect.left,
      offsetY: event.clientY - rect.top,
      moved: false
    };
    el.petBody.setPointerCapture?.(event.pointerId);
  });
  el.petBody?.addEventListener("pointermove", (event) => {
    if (!petDrag || petDrag.pointerId !== event.pointerId || !el.desktopPet) return;
    const x = Math.max(8, Math.min(window.innerWidth - el.desktopPet.offsetWidth - 8, event.clientX - petDrag.offsetX));
    const y = Math.max(8, Math.min(window.innerHeight - el.desktopPet.offsetHeight - 8, event.clientY - petDrag.offsetY));
    el.desktopPet.style.left = `${x}px`;
    el.desktopPet.style.top = `${y}px`;
    el.desktopPet.style.right = "auto";
    el.desktopPet.style.bottom = "auto";
    petDrag.moved = true;
  });
  el.petBody?.addEventListener("pointerup", (event) => {
    if (petDrag?.pointerId === event.pointerId) {
      window.setTimeout(() => { petDrag = null; }, 0);
    }
  });
  el.petBody?.addEventListener("pointercancel", () => { petDrag = null; });

  document.addEventListener("focusin", (event) => {
    if (event.target?.matches?.("input,select,textarea")) {
      syncInputs = false;
    }
  }, true);
  document.addEventListener("input", (event) => {
    if (event.target?.matches?.("input,select,textarea")) {
      markControlTouched(event.target);
      markDraftDirty();
    }
  }, true);
  document.addEventListener("change", (event) => {
    if (event.target?.matches?.("input,select,textarea")) {
      markControlTouched(event.target);
      markDraftDirty();
    }
  }, true);
  document.addEventListener("focusout", () => {
    window.requestAnimationFrame(() => {
      syncInputs = !localDraftDirty && !editingFormControl();
      if (syncInputs) render(current);
    });
  }, true);

  bindAction("scanCams");
  bindAction("phoneSiteEnable");
  bindAction("phoneSiteOpen");
  bindAction("phoneSiteDisable");
  bindAction("openModels");
  bindAction("openCalib");
  bindAction("createCalib");
  bindAction("openBuild");
  bindAction("prepDeploy");
  bindAction("rescanModel");
  el.startRuntime?.addEventListener("click", async () => {
    await withButtonFeedback(el.startRuntime, "startRuntime", async () => {
      setCommandStatus("Saving config before runtime start");
      const saved = await saveVisibleConfig("Manual plank refresh before runtime start");
      if (saved?.ok === false || saved?.result?.ok === false) return saved;
      return sendCommand("startRuntime", {}, null, { actionId: "startRuntime" });
    });
  });
  bindAction("stopRuntime");
  el.saveConfig?.addEventListener("click", () => withButtonFeedback(el.saveConfig, "saveConfig", () => saveVisibleConfig("Manual plank refresh before save")));
  el.saveAdvanced?.addEventListener("click", () => withButtonFeedback(el.saveAdvanced, "saveConfig", () => saveVisibleConfig("Manual plank refresh before advanced save")));

  for (const id of ["steamVrBridgeToggle", "steamVrBridgeChestToggle", "steamVrBridgeElbowsToggle", "steamVrBridgeKneesToggle", "oscToggle", "oscRotToggle", "legacySolverToggle", "replayToggle", "motionToggle", "stereoFallbackToggle", "bodyCalToggle", "bodyCalAutoPersistToggle", "roomDepthMapToggle", "roomDepthMapCollectOnlyToggle", "floorAssistToggle", "trackerSpaceValidToggle", "cropToggle"]) {
    bindToggle(id);
  }
  el.cropToggle?.addEventListener("click", () => {
    renderCropStatus();
    renderFloorAssist(trackingMode());
    updateFloorMarkOverlay();
  });
  for (const id of ["cropX", "cropY", "cropW", "cropH"]) {
    el[id]?.addEventListener("input", () => {
      setPressed("cropToggle", true);
      renderCropStatus();
      renderFloorAssist(trackingMode());
      updateFloorMarkOverlay();
    });
  }
  el.cropFull?.addEventListener("click", () => {
    setCropRect({ x: 0, y: 0, w: 1, h: 1 }, false);
    markDraftDirty();
    setCommandStatus("Camera A crop cleared · save config/restart runtime to apply");
  });
  el.cropDraw?.addEventListener("click", () => {
    if (!el.floorAssistPreview?.getAttribute("src")) {
      setButtonResult(el.cropDraw, false, "refresh preview first");
      setCommandStatus("Draw crop failed: refresh Camera A preview first", false);
      return;
    }
    cropDrawing = true;
    cropStart = null;
    cropEnd = null;
    floorMarking = false;
    el.floorAssistPreview?.classList.add("is-marking");
    renderFloorAssist(trackingMode());
    updateFloorMarkOverlay();
    setCommandStatus("Crop draw mode on · drag a rectangle on the Camera A preview");
  });

  el.cameraRows?.addEventListener("click", (event) => {
    const button = event.target.closest("button[data-a],button[data-b]");
    if (!button) return;
    if (button.dataset.a !== undefined) {
      el.cameraA.value = button.dataset.a;
      markControlTouched(button);
      markDraftDirty();
      setCommandStatus(`Camera A set to ${button.dataset.a} locally · sending backend update`);
      renderFloorAssist(trackingMode());
      sendCommand("setCamera", { slot: "a", index: Number(button.dataset.a) }, button, { actionLabel: "Set Camera A" });
    }
    if (button.dataset.b !== undefined) {
      el.cameraB.value = button.dataset.b;
      markControlTouched(button);
      markDraftDirty();
      setCommandStatus(`Camera B set to ${button.dataset.b} locally · sending backend update`);
      sendCommand("setCamera", { slot: "b", index: Number(button.dataset.b) }, button, { actionLabel: "Set Camera B" });
    }
  });

  el.trackingMode?.addEventListener("change", () => render(current));
  el.inferenceDevice?.addEventListener("change", () => {
    markDraftDirty();
    render(current);
  });
  for (const input of [el.modelPath, el.calibrationPath, el.logFile, el.recordingDir, el.replayLogPath]) {
    input?.addEventListener("input", () => {
      markDraftDirty();
      render(current);
    });
    input?.addEventListener("change", () => {
      markDraftDirty();
      render(current);
    });
  }
  el.applyInferenceDevice?.addEventListener("click", async () => {
    await withButtonFeedback(el.applyInferenceDevice, "applyInferenceDevice", async () => {
      const wasRunning = !!current.runtime?.running;
      setCommandStatus(wasRunning ? "Saving device and restarting runtime" : "Saving inference device");
      const saved = await sendCommand("saveConfig", readPayload(), null, { actionId: "applyInferenceDevice" });
      if (saved?.ok === false || saved?.result?.ok === false) return saved;
      if (wasRunning) {
        const stopped = await sendCommand("stopRuntime", {}, null, { actionId: "stopRuntime" });
        if (stopped?.ok === false || stopped?.result?.ok === false) return stopped;
        return sendCommand("startRuntime", {}, null, { actionId: "startRuntime" });
      }
      return sendCommand("rescanModel", {}, null, { actionId: "rescanModel" });
    });
  });
  el.cameraA?.addEventListener("change", () => {
    markControlTouched(el.cameraA);
    setCommandStatus(`Camera A set to ${el.cameraA.value} locally · sending backend update`);
    renderFloorAssist(trackingMode());
    sendCommand("setCamera", { slot: "a", index: Number(el.cameraA.value) }, null, { actionLabel: "Set Camera A" });
  });
  el.cameraB?.addEventListener("change", () => {
    markControlTouched(el.cameraB);
    setCommandStatus(`Camera B set to ${el.cameraB.value} locally · sending backend update`);
    sendCommand("setCamera", { slot: "b", index: Number(el.cameraB.value) }, null, { actionLabel: "Set Camera B" });
  });

  el.floorCameraSlot?.addEventListener("change", () => {
    saveActiveWallSlots();
    floorCalibrationRequestSeq++;
    resetFloorMarks(false);
    loadActiveWallSlots();
    floorGeometryAuto = floorGeometryByCamera.camera_a;
    updateWallRectangleStatus();
    renderFloorAssist(trackingMode());
    setCommandStatus(`${activeFloorCameraLabel()} wall/plank setup selected`);
  });

  el.floorManualApply?.addEventListener("click", () => {
    applyManualFloorLines("Manual plank lines", { button: el.floorManualApply });
  });
  el.refreshPreview?.addEventListener("click", async () => {
    if (buttonIsBusy(el.refreshPreview)) return;
    setButtonBusy(el.refreshPreview, "refresh preview");
    const cameraSlot = activeFloorCameraKey();
    const reply = await sendCommand("refreshCameraPreview", { camera_a: cameraIndexForFloorSlot(cameraSlot), camera_slot: cameraSlot }, null, { actionId: "refreshPreview", silentReply: true });
    const result = reply?.result || {};
    if (reply?.ok && result.preview) {
      const cameraIndex = Number(result.camera ?? el.cameraA?.value);
      const items = current.cameras?.items || [];
      let camera = items.find((item) => Number(item.index) === cameraIndex);
      if (!camera) {
        current.cameras ||= {};
        current.cameras.items ||= [];
        camera = { index: cameraIndex };
        current.cameras.items.push(camera);
      }
      camera.opened = true;
      camera.preview = result.preview;
      camera.full_preview = result.full_preview || result.preview;
      camera.width = Number(result.width || camera.width || 0);
      camera.height = Number(result.height || camera.height || 0);
      camera.preview_source_x = Number(result.preview_source_x || 0);
      camera.preview_source_y = Number(result.preview_source_y || 0);
      camera.preview_source_width = Number(result.preview_source_width || camera.width || 0);
      camera.preview_source_height = Number(result.preview_source_height || camera.height || 0);
      camera.preview_frame_width = Number(result.preview_frame_width || camera.width || 0);
      camera.preview_frame_height = Number(result.preview_frame_height || camera.height || 0);
      resetFloorMarks(false);
      renderFloorAssist(trackingMode());
      setButtonResult(el.refreshPreview, true, "preview refreshed");
      setCommandStatus(`Preview refreshed for ${activeFloorCameraLabel()} ${cameraIndex}`);
    } else if (reply?.ok && !result.preview) {
      setButtonResult(el.refreshPreview, false, "camera opened but no preview image returned");
      setCommandStatus("Refresh preview failed: camera opened but no preview image returned", false);
    } else {
      setButtonResult(el.refreshPreview, false, reply?.error || result.status || "refresh failed");
      setCommandStatus(`Refresh preview failed: ${reply?.error || result.status || "backend failed"}`, false);
    }
  });
  el.floorMarkStart?.addEventListener("click", () => {
    markControlTouched(el.floorMarkStart);
    if (!el.floorAssistPreview?.getAttribute("src")) {
      setButtonResult(el.floorMarkStart, false, "refresh preview first");
      setCommandStatus(`Draw one plank failed: refresh ${activeFloorCameraLabel()} preview first`, false);
      return;
    }
    floorCalibrationRequestSeq++;
    floorLineStart = null;
    floorLineEnd = null;
    floorMarks = [];
    floorMarkImageWidth = 0;
    floorMarkImageHeight = 0;
    cropDrawing = false;
    cropStart = null;
    cropEnd = null;
    cropAdjust = null;
    floorGeometryCleared = false;
    floorMarking = true;
    floorMarkMode = "plank";
    markDraftDirty();
    el.floorAssistPreview?.classList.remove("line-started");
    el.floorAssistPreview?.classList.add("is-marking");
    updateFloorMarkOverlay();
    setButtonResult(el.floorMarkStart, true, "drawing mode on");
    setCommandStatus(`${activeFloorCameraLabel()} drawing mode on - drag the first long plank edge`);
    renderFloorAssist(trackingMode());
    if (el.floorMarkReadout) el.floorMarkReadout.textContent = `${activeFloorCameraLabel()} drawing mode active - drag along the first long edge of one plank`;
  });
  el.wallRectSlot?.addEventListener("change", () => {
    setActiveWallSlot(Number(el.wallRectSlot.value));
    setCommandStatus(`${wallSlotLabel()} selected`);
  });
  el.wallSlotButtons?.addEventListener("click", (event) => {
    const button = event.target.closest("button[data-wall-slot]");
    if (!button) return;
    markControlTouched(button);
    setActiveWallSlot(Number(button.dataset.wallSlot));
    setCommandStatus(`${wallSlotLabel()} selected`);
  });
  el.floorWallStart?.addEventListener("click", () => {
    markControlTouched(el.floorWallStart);
    if (!el.floorAssistPreview?.getAttribute("src")) {
      setButtonResult(el.floorWallStart, false, "refresh preview first");
      setCommandStatus(`Draw wall sample failed: refresh ${activeFloorCameraLabel()} preview first`, false);
      return;
    }
    floorCalibrationRequestSeq++;
    floorLineStart = null;
    floorLineEnd = null;
    floorMarks = [];
    floorMarkImageWidth = 0;
    floorMarkImageHeight = 0;
    cropDrawing = false;
    cropStart = null;
    cropEnd = null;
    cropAdjust = null;
    clearWallRectangle(activeWallSlot);
    floorGeometryCleared = false;
    floorMarking = true;
    floorMarkMode = "wall";
    markDraftDirty();
    el.floorAssistPreview?.classList.remove("line-started");
    el.floorAssistPreview?.classList.add("is-marking");
    updateFloorMarkOverlay();
    setButtonResult(el.floorWallStart, true, `${wallSlotLabel()} draw on`);
    setCommandStatus(`${wallSlotLabel()} draw mode on - click four corners of one rectangular object`);
    renderFloorAssist(trackingMode());
    if (el.floorMarkReadout) el.floorMarkReadout.textContent = `${wallSlotLabel()} active - click four visible corners around one rectangular object; first trace the measured width edge, then continue around it`;
  });
  el.wallApplySelected?.addEventListener("click", () => {
    markControlTouched(el.wallApplySelected);
    applyManualWallRectangle(wallSlotLabel());
  });
  el.wallClearSelected?.addEventListener("click", () => {
    markControlTouched(el.wallClearSelected);
    clearWallRectangle(activeWallSlot);
    floorMarking = false;
    el.floorAssistPreview?.classList.remove("is-marking", "line-started");
    markDraftDirty();
    setButtonResult(el.wallClearSelected, true, `${wallSlotLabel()} cleared`);
    setCommandStatus(`${wallSlotLabel()} cleared`);
  });
  el.floorMarkClear?.addEventListener("click", () => {
    markControlTouched(el.floorMarkClear);
    resetFloorMarks(true);
    setPressed("floorAssistToggle", false);
    setValue("floorSpacingPx", 0);
    setValue("floorReferenceY", 0);
    markDraftDirty();
    setButtonResult(el.floorMarkClear, true, "cleared locally");
    setCommandStatus("Manual plank outline and floor-geometry activation cleared; save config to disable it");
    renderFloorAssist(trackingMode());
  });
  el.floorAssistPreview?.addEventListener("pointerdown", handleFloorPreviewPointerDown);
  el.floorAssistPreview?.addEventListener("pointermove", handleFloorPreviewPointerMove);
  el.floorAssistPreview?.addEventListener("pointerup", handleFloorPreviewPointerUp);
  el.floorAssistPreview?.addEventListener("pointercancel", cancelFloorPreviewLine);
  for (const id of ["floorSpacingM", "floorSpacingPx", "floorReferenceY", "floorReferenceM", "floorPlankLengthM", "floorConfidence"]) {
    el[id]?.addEventListener("input", () => {
      markDraftDirty();
      renderFloorAssist(trackingMode());
    });
  }
  for (const id of ["wallRectWidthM", "wallRectHeightM", "wallRectAspect"]) {
    el[id]?.addEventListener("input", () => {
      markDraftDirty();
      updateWallRectangleStatus();
    });
  }



  el.steamVrAlignStart?.addEventListener("click", async () => {
    const reply = await sendCommand("steamVrAlignmentStart", {}, el.steamVrAlignStart, { actionId: "steamVrAlignStart" });
    if (commandOk(reply)) {
      updateWizardPrompt(steamVrNextWizardStepForState({ active: true }, steamVrAcceptedSampleKeys(current.steamvr_alignment?.samples || [])));
    }
  });
  function bindSteamVrSample(buttonId, landmark) {
    el[buttonId]?.addEventListener("click", async () => {
      const key = canonicalSteamVrLandmarkKey(landmark);
      if (!landmarkEnabledByConfig(key)) {
        setButtonResult(el[buttonId], false, "tracker role disabled: enable SteamVR trackers or map this legacy OSC role");
        setCommandStatus(`${steamVrLandmarkByKey.get(key)?.label || key}: optional tracker role is disabled: enable SteamVR trackers or map this legacy OSC role`, "warn");
        return;
      }
      await recordSteamVrWizardLandmark(key, "unknown", el[buttonId]);
    });
  }
  for (const item of steamVrAlignmentLandmarks) {
    bindSteamVrSample(item.buttonId, item.key);
  }
  el.steamVrAlignFinish?.addEventListener("click", async () => {
    const reply = await sendCommand("steamVrAlignmentFinish", {}, el.steamVrAlignFinish, { actionId: "steamVrAlignFinish" });
    if (commandOk(reply)) updateWizardPrompt(solveWizardStepIndex());
  });
  el.steamVrAlignClear?.addEventListener("click", async () => {
    const reply = await sendCommand("steamVrAlignmentClear", {}, el.steamVrAlignClear, { actionId: "steamVrAlignClear" });
    if (commandOk(reply)) updateWizardPrompt(0);
  });
  el.trackerSpaceIdentity?.addEventListener("click", () => {
    setValue("trackerOffsetX", 0);
    setValue("trackerOffsetY", 0);
    setValue("trackerOffsetZ", 0);
    setValue("trackerRotX", 0);
    setValue("trackerRotY", 0);
    setValue("trackerRotZ", 0);
    setValue("trackerRotW", 1);
    setValue("trackerScale", 1);
    setPressed("trackerSpaceValidToggle", false);
    markDraftDirty();
    setCommandStatus("Identity tracker-space draft loaded; validate the numbers, then only mark valid if it is physically aligned");
    renderTrackerSpace();
  });
  el.trackerSpaceValidate?.addEventListener("click", () => {
    const result = trackerSpaceLocalStatus();
    if (result.status === "invalid") {
      setPressed("trackerSpaceValidToggle", false);
      markDraftDirty();
      setCommandStatus(`Tracker-space validation failed: ${result.reason}`, false);
    } else {
      setPressed("trackerSpaceValidToggle", true);
      markDraftDirty();
      setCommandStatus("Tracker-space numbers validated; manual physical alignment is not proven. Save only if the transform is physically correct");
    }
    renderTrackerSpace();
  });
  for (const id of ["trackerOffsetX", "trackerOffsetY", "trackerOffsetZ", "trackerRotX", "trackerRotY", "trackerRotZ", "trackerRotW", "trackerScale"]) {
    el[id]?.addEventListener("input", renderTrackerSpace);
  }
}

window.bodyTrackerUi = Object.assign(window.bodyTrackerUi || {}, {
  render,
  updateActionPresentation,
  getState: () => current
});
window.render = render;
window.updateActionPresentation = updateActionPresentation;

window.addEventListener("error", (event) => {
  setBridge("JS ERROR", false);
  setCommandStatus(`JS error: ${event.message}`, false);
});

window.addEventListener("unhandledrejection", (event) => {
  setBridge("JS ERROR", false);
  setCommandStatus(`Unhandled promise rejection: ${event.reason}`, false);
});

const initialBridgeReady = webviewAvailable();
if (initialBridgeReady) {
  webviewBridge.addMessageListener(handleMessage);
  setBridge("WAITING");
} else {
  setBridge("FAILED", false);
}

bindEvents();
render(current);
setCommandStatus(initialBridgeReady ? "READY" : "WebView2 bridge unavailable", initialBridgeReady);
