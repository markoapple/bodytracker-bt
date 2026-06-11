import { titleCaseCommand } from "./format.js";

export const commandLabels = {
  scanCameras: "Scan cameras",
  enablePhoneWebCamera: "Enable phone site",
  openPhoneWebCamera: "Open phone site",
  disablePhoneWebCamera: "Disable phone site",
  openModelsFolder: "Open models folder",
  openCalibrationFolder: "Open calib folder",
  createCalibrationTemplate: "Create calib template",
  openBuildFolder: "Open build folder",
  prepareDeployFolder: "Prepare deploy",
  rescanModel: "Rescan model",
  saveConfig: "Save config",
  startRuntime: "Start runtime",
  stopRuntime: "Stop runtime",
  setCamera: "Set camera",
  refreshCameraPreview: "Refresh preview",
  calibrateFloorGeometryBackend: "Solve floor geometry",
  steamVrAlignmentStart: "Start SteamVR alignment",
  steamVrAlignmentRecord: "Record SteamVR sample",
  steamVrAlignmentFinish: "Solve SteamVR alignment",
  steamVrAlignmentClear: "Clear SteamVR alignment"
};

export function commandLabel(command) {
  return commandLabels[command] || titleCaseCommand(command);
}

export const actionIntents = Object.create(null);

export function registerActionIntent(id, definition = {}) {
  if (!id) return null;
  const currentIntent = actionIntents[id] || {};
  const command = definition.command || currentIntent.command || id;
  const label = definition.label || currentIntent.label || commandLabel(command);
  const intent = Object.assign({}, currentIntent, definition, {
    id,
    command,
    label,
    pendingLabel: definition.pendingLabel || currentIntent.pendingLabel || `${label}...`,
    successLabel: definition.successLabel || currentIntent.successLabel || `${label}: ok`,
    unavailableLabel: definition.unavailableLabel || currentIntent.unavailableLabel || `${label} needs the desktop backend bridge`
  });
  actionIntents[id] = intent;
  return intent;
}

function escapeRegex(value) {
  return String(value).replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
}

export function humanizeCommandText(value) {
  let text = String(value ?? "");
  for (const [command, label] of Object.entries(commandLabels)) {
    text = text.replace(new RegExp(`\b${escapeRegex(command)}\b`, "g"), label);
  }
  text = text.replace(/command circuit breaker open/gi, "backend command channel is temporarily paused");
  text = text.replace(/bridge unavailable/gi, "backend bridge is unavailable");
  return text;
}

export function actionIntentFor(actionId, command = null) {
  if (actionId && actionIntents[actionId]) return actionIntents[actionId];
  const match = Object.values(actionIntents).find((intent) => intent.command === command);
  if (match) return match;
  if (command) return { id: actionId || command, command, label: commandLabel(command), pendingLabel: `${commandLabel(command)}...` };
  return null;
}

export function actionIntentForButton(button, command = null, options = {}) {
  return actionIntentFor(options.actionId || button?.dataset?.actionId || button?.id, command);
}

export function actionLabel(command, actionId = null) {
  return actionIntentFor(actionId, command)?.label || commandLabel(command);
}

export function actionProgressText(command, actionId = null) {
  return `${actionLabel(command, actionId)}...`;
}

export function buttonActionLabel(value) {
  if (actionIntents[value]) return actionIntents[value].label;
  return commandLabels[value] || String(value || "Working");
}

const staticActionIntents = {
  scanCams: { command: "scanCameras", label: "Scan cameras" },
  phoneSiteEnable: { command: "enablePhoneWebCamera", label: "Enable phone site" },
  phoneSiteOpen: { command: "openPhoneWebCamera", label: "Open phone site" },
  phoneSiteDisable: { command: "disablePhoneWebCamera", label: "Disable phone site" },
  openModels: { command: "openModelsFolder", label: "Open models folder" },
  openCalib: { command: "openCalibrationFolder", label: "Open calibration folder" },
  createCalib: { command: "createCalibrationTemplate", label: "Create calibration template" },
  openBuild: { command: "openBuildFolder", label: "Open build folder" },
  prepDeploy: { command: "prepareDeployFolder", label: "Prepare deploy folder" },
  rescanModel: { command: "rescanModel", label: "Rescan model" },
  applyInferenceDevice: { command: "saveConfig", label: "Apply inference device", pendingLabel: "Applying inference device..." },
  startRuntime: { command: "startRuntime", label: "Start runtime" },
  stopRuntime: { command: "stopRuntime", label: "Stop runtime" },
  saveConfig: { command: "saveConfig", label: "Save config" },
  saveAdvanced: { command: "saveConfig", label: "Save advanced config" },
  refreshPreview: { command: "refreshCameraPreview", label: "Refresh preview" },
  floorManualApply: { command: "calibrateFloorGeometryBackend", label: "Apply drawn plank" },
  wallApplySelected: { command: "calibrateFloorGeometryBackend", label: "Solve wall sample" },
  steamVrAlignStart: { command: "steamVrAlignmentStart", label: "Start SteamVR calibration" },
  steamVrAlignFinish: { command: "steamVrAlignmentFinish", label: "Solve SteamVR alignment" },
  steamVrAlignClear: { command: "steamVrAlignmentClear", label: "Clear SteamVR alignment" },
  steamVrAlignLeftFoot: { command: "steamVrAlignmentRecord", label: "Record left ankle sample" },
  steamVrAlignRightFoot: { command: "steamVrAlignmentRecord", label: "Record right ankle sample" },
  steamVrAlignPelvis: { command: "steamVrAlignmentRecord", label: "Record pelvis sample" },
  steamVrAlignFloor: { command: "steamVrAlignmentRecord", label: "Record floor sample" },
  steamVrAlignForward: { command: "steamVrAlignmentRecord", label: "Record forward sample" },
  steamVrAlignChest: { command: "steamVrAlignmentRecord", label: "Record chest sample" },
  steamVrAlignLeftElbow: { command: "steamVrAlignmentRecord", label: "Record left elbow sample" },
  steamVrAlignRightElbow: { command: "steamVrAlignmentRecord", label: "Record right elbow sample" },
  steamVrAlignLeftKnee: { command: "steamVrAlignmentRecord", label: "Record left knee sample" },
  steamVrAlignRightKnee: { command: "steamVrAlignmentRecord", label: "Record right knee sample" }
};
Object.entries(staticActionIntents).forEach(([id, definition]) => registerActionIntent(id, definition));

export class CommandBus {
  constructor(options = {}) {
    this.bridge = options.bridge;
    this.onStatus = options.onStatus || (() => {});
    this.onBridge = options.onBridge || (() => {});
    this.onPendingChange = options.onPendingChange || (() => {});
    this.now = options.now || (() => Date.now());
    this.pending = new Map();
    this.nextId = 1;
    this.retryConfig = { maxRetries: 1, baseDelayMs: 150, maxDelayMs: 1000, backoffMultiplier: 2 };
    this.retryableCommands = new Set(["scanCameras", "refreshCameraPreview", "rescanModel"]);
    this.circuitBreaker = { failures: 0, threshold: 5, timeoutMs: 30000, state: "CLOSED", nextAttempt: 0 };
  }

  available() { return !!this.bridge?.available?.(); }

  pendingSnapshot() {
    const out = {};
    for (const [id, entry] of this.pending.entries()) {
      out[id] = {
        command: entry.command,
        retries: entry.envelope.retries,
        actionId: entry.envelope.actionId || null,
        actionLabel: entry.envelope.actionLabel || commandLabel(entry.command)
      };
    }
    return out;
  }

  notifyPending() { this.onPendingChange(this.pendingSnapshot()); }

  send(command, payload = {}, options = {}) {
    const actionId = options.actionId || options.button?.dataset?.actionId || null;
    const label = options.actionLabel || actionLabel(command, actionId);
    if (!this.available()) {
      this.onBridge("FAILED", false);
      this.onStatus(`${label} needs the desktop backend bridge`, false);
      return Promise.resolve({ ok: false, error: "backend bridge unavailable", actionLabel: label });
    }
    if (this.circuitBreaker.state === "OPEN") {
      if (this.now() < this.circuitBreaker.nextAttempt) {
        this.onBridge("CIRCUIT OPEN", false);
        this.onStatus(`${label} paused: backend command channel is recovering after repeated failures`, "warn");
        return Promise.resolve({ ok: false, error: "backend command channel is temporarily paused", actionLabel: label });
      }
      this.circuitBreaker.state = "HALF_OPEN";
    }

    const timeoutMs = options.timeoutMs ?? (command === "calibrateFloorGeometryBackend" ? 20000 : 5000);
    const retryable = options.retryable ?? this.retryableCommands.has(command);
    const maxRetries = options.maxRetries ?? (retryable ? this.retryConfig.maxRetries : 0);
    const envelope = {
      id: this.nextId++, command, payload, retries: 0, maxRetries, timeoutMs,
      createdAt: this.now(), deadline: this.now() + timeoutMs,
      silentReply: !!options.silentReply, button: options.button || null,
      actionId, actionLabel: label
    };

    this.onStatus(actionProgressText(command, actionId));
    return new Promise((resolve, reject) => {
      this.pending.set(envelope.id, { command, resolve, reject, envelope, timeout: null });
      this.notifyPending();
      this.execute(envelope);
    });
  }

  execute(envelope) {
    const entry = this.pending.get(envelope.id);
    if (!entry) return;
    if (entry.timeout) window.clearTimeout(entry.timeout);
    entry.timeout = window.setTimeout(() => this.handleTimeout(envelope.id), envelope.timeoutMs);
    try {
      this.bridge.postMessage({ id: envelope.id, command: envelope.command, payload: envelope.payload });
    } catch (error) {
      this.fail(envelope.id, error?.message || "bridge post failed");
    }
  }

  handleTimeout(id) {
    const entry = this.pending.get(id);
    if (!entry) return;
    const envelope = entry.envelope;
    if (envelope.retries < envelope.maxRetries) {
      envelope.retries += 1;
      const delay = Math.min(this.retryConfig.maxDelayMs, this.retryConfig.baseDelayMs * Math.pow(this.retryConfig.backoffMultiplier, envelope.retries - 1));
      this.onStatus(`${envelope.actionLabel || commandLabel(envelope.command)} timed out; retry ${envelope.retries}/${envelope.maxRetries}`, "warn");
      window.setTimeout(() => this.execute(envelope), delay);
      this.notifyPending();
      return;
    }
    this.fail(id, `timeout after ${Math.round(envelope.timeoutMs / 1000)}s`);
  }

  fail(id, error) {
    const entry = this.pending.get(id);
    if (!entry) return;
    const errorText = String(error?.message || error || "command failed");
    if (entry.timeout) window.clearTimeout(entry.timeout);
    this.pending.delete(id);
    this.circuitBreaker.failures += 1;
    if (this.circuitBreaker.failures >= this.circuitBreaker.threshold) {
      this.circuitBreaker.state = "OPEN";
      this.circuitBreaker.nextAttempt = this.now() + this.circuitBreaker.timeoutMs;
    }
    this.onBridge(errorText.includes("timeout") ? "TIMEOUT" : "ERROR", false);
    this.onStatus(`${entry.envelope?.actionLabel || commandLabel(entry.command)}: ${humanizeCommandText(errorText)}`, false);
    entry.resolve({ ok: false, error: errorText });
    this.notifyPending();
  }

  handleReply(message) {
    const entry = this.pending.get(message?.id);
    if (!entry) return null;
    if (entry.timeout) window.clearTimeout(entry.timeout);
    this.pending.delete(message.id);
    const result = message.result || {};
    const ok = !!message.ok && result.ok !== false;
    if (ok) {
      this.circuitBreaker.failures = 0;
      if (this.circuitBreaker.state === "HALF_OPEN") this.circuitBreaker.state = "CLOSED";
    } else {
      this.circuitBreaker.failures += 1;
    }
    entry.resolve(message);
    this.notifyPending();
    return { entry, result, ok };
  }
}
