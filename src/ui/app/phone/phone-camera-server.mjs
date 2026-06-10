import http from "node:http";
import https from "node:https";
import fs from "node:fs";
import net from "node:net";
import os from "node:os";
import path from "node:path";
import { createHash } from "node:crypto";
import { fileURLToPath } from "node:url";

const root = path.dirname(fileURLToPath(import.meta.url));
const indexHtml = fs.readFileSync(path.join(root, "phone-camera.html"), "utf8");
const statePath = path.join(root, "phone-site.json");

const host = process.env.BT_PHONE_HOST || "0.0.0.0";
const webPort = Number(process.env.BT_PHONE_WEB_PORT || 39443);
const targetHost = process.env.BT_PHONE_TARGET_HOST || "127.0.0.1";
const targetPort = Number(process.env.BT_PHONE_TARGET_PORT || 39555);
const certPath = process.env.BT_PHONE_CERT || path.join(root, "certs", "bodytracker-phone-camera.crt");
const keyPath = process.env.BT_PHONE_KEY || path.join(root, "certs", "bodytracker-phone-camera.key");
const pfxPath = process.env.BT_PHONE_PFX || path.join(root, "certs", "bodytracker-phone-camera.pfx");
const pfxPassphrase = process.env.BT_PHONE_PFX_PASSPHRASE || "bodytracker";
const maxFrameBytes = 64 * 1024 * 1024;
const maxPendingFrameBytes = 64 * 1024 * 1024;
const backendConnectRetryMs = 400;
const backendConnectDeadlineMs = 30000;

const apkCandidates = [
  path.join(root, "app-debug.apk"),
  path.join(root, "..", "..", "android", "FBTPhoneCamera", "app", "build", "outputs", "apk", "debug", "app-debug.apk"),
  path.join(root, "..", "..", "..", "android", "FBTPhoneCamera", "app", "build", "outputs", "apk", "debug", "app-debug.apk"),
  path.join(root, "..", "..", "..", "..", "android", "FBTPhoneCamera", "app", "build", "outputs", "apk", "debug", "app-debug.apk")
];

const stats = {
  startedAt: new Date().toISOString(),
  wsConnections: 0,
  activeWs: 0,
  backendConnections: 0,
  backendConnectFailures: 0,
  framesFromPhone: 0,
  framesToTracker: 0,
  bytesToTracker: 0,
  lastWsAt: "",
  lastBackendAt: "",
  lastFrameAt: "",
  lastError: ""
};

function apkPath() {
  return apkCandidates.map((p) => path.resolve(p)).find((p) => fs.existsSync(p));
}

function adapterScore(name, address) {
  const label = String(name || "").toLowerCase();
  const virtual = /vethernet|hyper-v|vmware|virtualbox|docker|wsl|loopback|npcap|zerotier|tailscale|wireguard|tap|tunnel/.test(label);
  const privateLan = /^(192\.168\.|10\.|172\.(1[6-9]|2\d|3[0-1])\.)/.test(address);
  let score = 0;
  if (!privateLan) score += 20;
  if (virtual) score += 100;
  return score;
}

function localIps() {
  const entries = [];
  for (const [name, addresses] of Object.entries(os.networkInterfaces())) {
    for (const i of addresses || []) {
      if (!i || i.family !== "IPv4" || i.internal || i.address.startsWith("169.254.")) continue;
      entries.push({ name, address: i.address, score: adapterScore(name, i.address) });
    }
  }
  entries.sort((a, b) => a.score - b.score || a.name.localeCompare(b.name) || a.address.localeCompare(b.address));
  return [...new Set(entries.map((i) => i.address))];
}

function serverUrls(scheme) {
  const ips = host === "0.0.0.0" || host === "::" ? localIps() : [host];
  const targets = ips.length ? ips : ["127.0.0.1"];
  return [...new Set(targets.map((ip) => `${scheme}://${ip}:${webPort}/`))];
}

function writeState(fields = {}) {
  const scheme = fields.scheme || (tlsReady ? "https" : "http");
  const urls = fields.urls || serverUrls(scheme);
  const enabled = Boolean(fields.enabled);
  const state = {
    enabled,
    status: fields.status || (enabled ? "enabled" : "disabled"),
    url: enabled ? (fields.url || urls[0] || `${scheme}://127.0.0.1:${webPort}/`) : "",
    urls: enabled ? urls : [],
    web_port: webPort,
    target_host: targetHost,
    target_port: targetPort,
    target: `${targetHost}:${targetPort}`,
    pc_ips: localIps(),
    apk: Boolean(apkPath()),
    secure: scheme === "https",
    tls: tlsReady,
    bridge: stats,
    updated: new Date().toISOString()
  };
  try {
    fs.writeFileSync(statePath, `${JSON.stringify(state)}\n`);
  } catch (error) {
    console.error(`failed to write phone-site.json: ${error.message}`);
  }
}

function acceptKey(key) {
  return createHash("sha1").update(key.trim() + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").digest("base64");
}

function wsFrame(opcode, payload = Buffer.alloc(0)) {
  const data = Buffer.isBuffer(payload) ? payload : Buffer.from(String(payload), "utf8");
  if (data.length < 126) {
    return Buffer.concat([Buffer.from([0x80 | opcode, data.length]), data]);
  }
  if (data.length <= 0xffff) {
    const header = Buffer.allocUnsafe(4);
    header[0] = 0x80 | opcode;
    header[1] = 126;
    header.writeUInt16BE(data.length, 2);
    return Buffer.concat([header, data]);
  }
  const header = Buffer.allocUnsafe(10);
  header[0] = 0x80 | opcode;
  header[1] = 127;
  header.writeBigUInt64BE(BigInt(data.length), 2);
  return Buffer.concat([header, data]);
}

function sendText(socket, message) {
  try {
    if (!socket.destroyed) socket.write(wsFrame(0x1, message));
  } catch {
    socket.destroy();
  }
}

function closeWebSocket(socket, code = 1011, reason = "bodytracker phone camera bridge closed") {
  const text = String(reason).slice(0, 123);
  const reasonBytes = Buffer.from(text, "utf8").subarray(0, 123);
  const payload = Buffer.allocUnsafe(2 + reasonBytes.length);
  payload.writeUInt16BE(code, 0);
  reasonBytes.copy(payload, 2);
  try {
    if (!socket.destroyed) socket.end(wsFrame(0x8, payload));
  } catch {
    socket.destroy();
  }
}

function writeTrackerFrame(socket, frame) {
  const header = Buffer.allocUnsafe(12);
  header.writeUInt32BE(frame.length, 0);
  header.writeBigUInt64BE(process.hrtime.bigint(), 4);
  socket.write(header);
  socket.write(frame);
}

function bridgeWebSocket(req, socket, head = Buffer.alloc(0)) {
  const key = req.headers["sec-websocket-key"];
  if (!key) {
    socket.destroy();
    return;
  }

  socket.write([
    "HTTP/1.1 101 Switching Protocols",
    "Upgrade: websocket",
    "Connection: Upgrade",
    `Sec-WebSocket-Accept: ${acceptKey(key)}`,
    "",
    ""
  ].join("\r\n"));

  stats.wsConnections += 1;
  stats.activeWs += 1;
  stats.lastWsAt = new Date().toISOString();
  writeState({ enabled: true, status: "enabled" });
  sendText(socket, "bridge_open");

  const pendingFrames = [];
  let pendingBytes = 0;
  let camera = null;
  let cameraReady = false;
  let closing = false;
  let messageOpcode = null;
  let messageParts = [];
  let messageBytes = 0;
  let buffer = Buffer.alloc(0);
  const connectDeadline = Date.now() + backendConnectDeadlineMs;

  function closeBridge(reason = "bridge closed") {
    if (closing) return;
    closing = true;
    stats.lastError = reason;
    writeState({ enabled: true, status: `bridge error: ${reason}` });
    closeWebSocket(socket, 1011, reason);
    if (camera) camera.destroy();
  }

  function flushPending() {
    if (!cameraReady || !camera || !camera.writable) return;
    for (const frame of pendingFrames.splice(0)) {
      writeTrackerFrame(camera, frame);
      stats.framesToTracker += 1;
      stats.bytesToTracker += frame.length;
      stats.lastFrameAt = new Date().toISOString();
      if (stats.framesToTracker === 1 || stats.framesToTracker % 30 === 0) {
        sendText(socket, `frames_forwarded:${stats.framesToTracker}`);
      }
    }
    pendingBytes = 0;
    writeState({ enabled: true, status: "streaming" });
  }

  function forwardFrame(payload) {
    stats.framesFromPhone += 1;
    if (!cameraReady) {
      if (pendingBytes + payload.length > maxPendingFrameBytes) {
        closeBridge("phone is sending frames but BodyTracker has not accepted the backend socket yet");
        return;
      }
      pendingFrames.push(payload);
      pendingBytes += payload.length;
      return;
    }
    if (!camera || !camera.writable) {
      closeBridge("BodyTracker backend socket became unavailable");
      return;
    }
    writeTrackerFrame(camera, payload);
    stats.framesToTracker += 1;
    stats.bytesToTracker += payload.length;
    stats.lastFrameAt = new Date().toISOString();
    if (stats.framesToTracker === 1 || stats.framesToTracker % 30 === 0) {
      sendText(socket, `frames_forwarded:${stats.framesToTracker}`);
      writeState({ enabled: true, status: "streaming" });
    }
  }

  function connectCamera() {
    if (closing || cameraReady || camera) return;

    let settled = false;
    const candidate = net.createConnection({ host: targetHost, port: targetPort });
    camera = candidate;
    candidate.setNoDelay(true);
    candidate.setTimeout(3000);

    function fail(reason) {
      if (settled) return;
      settled = true;
      stats.backendConnectFailures += 1;
      stats.lastError = reason;
      if (camera === candidate) camera = null;
      candidate.destroy();
      if (!closing && Date.now() < connectDeadline) {
        setTimeout(connectCamera, backendConnectRetryMs).unref();
      } else {
        closeBridge(reason);
      }
    }

    candidate.on("connect", () => {
      settled = true;
      cameraReady = true;
      stats.backendConnections += 1;
      stats.lastBackendAt = new Date().toISOString();
      stats.lastError = "";
      candidate.write("BTMJPEG1\n", "ascii");
      sendText(socket, "backend_connected");
      writeState({ enabled: true, status: "backend connected" });
      flushPending();
    });
    candidate.on("timeout", () => {
      fail(`BodyTracker did not accept ${targetHost}:${targetPort} within 3s`);
    });
    candidate.on("error", (error) => {
      if (!cameraReady) {
        fail(`BodyTracker camera input ${targetHost}:${targetPort} failed: ${error?.code || error?.message || "connection error"}`);
      } else {
        closeBridge(`BodyTracker camera input ${targetHost}:${targetPort} failed: ${error?.code || error?.message || "connection error"}`);
      }
    });
    candidate.on("close", () => {
      if (camera === candidate) camera = null;
      if (cameraReady && !closing) {
        cameraReady = false;
        closeBridge(`BodyTracker camera input ${targetHost}:${targetPort} closed`);
      } else if (!settled) {
        fail(`BodyTracker camera input ${targetHost}:${targetPort} closed before connect`);
      }
    });
  }

  function appendMessageChunk(opcode, payload, fin) {
    if (opcode === 0x2) {
      if (messageOpcode !== null) {
        closeBridge("received a new binary websocket message before the previous one finished");
        return;
      }
      if (fin) {
        if (payload.length > 0) forwardFrame(payload);
        return;
      }
      messageOpcode = opcode;
      messageParts = [payload];
      messageBytes = payload.length;
      return;
    }

    if (opcode === 0x0) {
      if (messageOpcode !== 0x2) {
        closeBridge("received websocket continuation without a binary message");
        return;
      }
      messageParts.push(payload);
      messageBytes += payload.length;
      if (messageBytes > maxFrameBytes) {
        closeBridge("websocket binary message exceeded max frame size");
        return;
      }
      if (fin) {
        const frame = Buffer.concat(messageParts, messageBytes);
        messageOpcode = null;
        messageParts = [];
        messageBytes = 0;
        if (frame.length > 0) forwardFrame(frame);
      }
      return;
    }

    if (opcode === 0x1) {
      // Text from the browser is diagnostic-only. Ignore it.
      return;
    }

    closeBridge(`unsupported websocket opcode ${opcode}`);
  }

  function consume(chunk) {
    buffer = Buffer.concat([buffer, chunk]);
    while (buffer.length >= 2 && !closing) {
      const b0 = buffer[0];
      const b1 = buffer[1];
      const fin = (b0 & 0x80) !== 0;
      const rsv = b0 & 0x70;
      const opcode = b0 & 0x0f;
      let len = b1 & 0x7f;
      const masked = (b1 & 0x80) !== 0;
      let offset = 2;

      if (rsv !== 0) {
        closeBridge("compressed websocket frames are not supported");
        return;
      }
      if (len === 126) {
        if (buffer.length < 4) return;
        len = buffer.readUInt16BE(2);
        offset = 4;
      } else if (len === 127) {
        if (buffer.length < 10) return;
        const wide = buffer.readBigUInt64BE(2);
        if (wide > BigInt(Number.MAX_SAFE_INTEGER)) {
          closeBridge("websocket frame too large");
          return;
        }
        len = Number(wide);
        offset = 10;
      }
      if (!masked) {
        closeBridge("unmasked websocket frame from browser");
        return;
      }
      if (len > maxFrameBytes) {
        closeBridge("websocket frame exceeded max frame size");
        return;
      }
      if (buffer.length < offset + 4 + len) return;

      const mask = buffer.subarray(offset, offset + 4);
      offset += 4;
      const payload = Buffer.from(buffer.subarray(offset, offset + len));
      buffer = buffer.subarray(offset + len);
      for (let i = 0; i < payload.length; i += 1) payload[i] ^= mask[i & 3];

      if (opcode === 0x8) {
        closing = true;
        try { socket.end(wsFrame(0x8, payload.subarray(0, 125))); } catch { socket.destroy(); }
        if (camera) camera.destroy();
        return;
      }
      if (opcode === 0x9) {
        try { socket.write(wsFrame(0xA, payload.subarray(0, 125))); } catch { socket.destroy(); }
        continue;
      }
      if (opcode === 0xA) {
        continue;
      }
      appendMessageChunk(opcode, payload, fin);
    }
  }

  socket.on("data", consume);
  socket.on("close", () => {
    closing = true;
    stats.activeWs = Math.max(0, stats.activeWs - 1);
    writeState({ enabled: true, status: stats.activeWs > 0 ? "enabled" : "enabled" });
    if (camera) camera.destroy();
  });
  socket.on("error", () => {
    closing = true;
    stats.activeWs = Math.max(0, stats.activeWs - 1);
    if (camera) camera.destroy();
  });

  connectCamera();
  if (head.length) consume(head);
}

function requestHandler(req, res) {
  const requestUrl = new URL(req.url || "/", "http://bodytracker.local");
  if (requestUrl.pathname === "/") {
    res.writeHead(200, {
      "content-type": "text/html; charset=utf-8",
      "cache-control": "no-store, max-age=0"
    });
    res.end(req.method === "HEAD" ? undefined : indexHtml);
    return;
  }
  if (requestUrl.pathname === "/healthz") {
    const scheme = tlsReady ? "https" : "http";
    const file = apkPath();
    const body = JSON.stringify({
      ok: true,
      targetHost,
      targetPort,
      urls: serverUrls(scheme),
      secure: scheme === "https",
      apk: Boolean(file),
      apkBytes: file ? fs.statSync(file).size : 0,
      bridge: stats
    });
    res.writeHead(200, {
      "content-type": "application/json",
      "cache-control": "no-store, max-age=0"
    });
    res.end(req.method === "HEAD" ? undefined : body);
    return;
  }
  if (requestUrl.pathname === "/app-debug.apk") {
    const file = apkPath();
    if (!file) {
      res.writeHead(404);
      res.end("apk not found");
      return;
    }
    const stat = fs.statSync(file);
    res.writeHead(200, {
      "content-type": "application/vnd.android.package-archive",
      "content-disposition": "attachment; filename=bodytracker-phone-camera.apk",
      "cache-control": "no-store, max-age=0",
      "content-length": stat.size
    });
    if (req.method === "HEAD") res.end();
    else fs.createReadStream(file).pipe(res);
    return;
  }
  res.writeHead(404);
  res.end("not found");
}

const pfxReady = fs.existsSync(pfxPath);
const pemReady = fs.existsSync(certPath) && fs.existsSync(keyPath);
const tlsReady = pfxReady || pemReady;
const server = tlsReady
  ? https.createServer(
      pfxReady
        ? { pfx: fs.readFileSync(pfxPath), passphrase: pfxPassphrase }
        : { cert: fs.readFileSync(certPath), key: fs.readFileSync(keyPath) },
      requestHandler)
  : http.createServer(requestHandler);

server.on("upgrade", (req, socket, head) => {
  const requestUrl = new URL(req.url || "/", "http://bodytracker.local");
  if (requestUrl.pathname === "/stream") bridgeWebSocket(req, socket, head);
  else socket.destroy();
});

server.on("error", (error) => {
  stats.lastError = error.message;
  writeState({ enabled: false, status: `failed: ${error.message}` });
  console.error(error.message);
  process.exitCode = 1;
});

server.listen(webPort, host, () => {
  const scheme = tlsReady ? "https" : "http";
  const urls = serverUrls(scheme);
  writeState({ enabled: true, status: "enabled", scheme, urls, url: urls[0] });
  console.log(`BodyTracker phone web camera -> ${targetHost}:${targetPort}`);
  console.log(`Open on phone: ${urls.join("  ") || `${scheme}://127.0.0.1:${webPort}/`}`);
});

function shutdown() {
  writeState({ enabled: false, status: "stopped" });
  server.close(() => process.exit(0));
  setTimeout(() => process.exit(0), 1000).unref();
}

process.once("SIGINT", shutdown);
process.once("SIGTERM", shutdown);