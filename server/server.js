/**
 * server.js — WebSocket relay for dual ESP32-CAM viewer
 *
 * ESP32 cameras connect on  ws://<host>:3000/ws   (binary + text)
 * Browsers      connect on  ws://<host>:3000/ws   (register with "browser")
 *
 * Flow:
 *   1. Each ESP32 sends  register:camX:<token>:<firmwareVersion>
 *   2. Server periodically sends "capture" to every ESP32
 *   3. ESP32 replies with binary:  "camX:" + JPEG
 *   4. Server relays that binary verbatim to every browser client
 */

const express = require('express');
const http    = require('http');
const WebSocket = require('ws');
const path    = require('path');
const { spawnSync } = require('child_process');
const crypto  = require('crypto');
const fs      = require('fs');
const os      = require('os');
const multer  = require('multer');

// ─── Express static file server ───
const app    = express();
app.set('trust proxy', 1);
const REGISTER_TOKEN = process.env.REGISTER_TOKEN;
const server = http.createServer(app);
app.use(express.static(path.join(__dirname, 'public')));

// ─── Stereo depth map endpoint ────────────────────────────────────────────────
const upload     = multer({ storage: multer.memoryStorage(), limits: { fileSize: 10 * 1024 * 1024 } });
const DEPTHMAP_PY = path.join(__dirname, 'depthmap.py');
const CALIB_JSON  = path.join(__dirname, 'calibration.json');

app.post('/api/depthmap', upload.fields([{ name: 'cam1', maxCount: 1 }, { name: 'cam2', maxCount: 1 }]), (req, res) => {
  const files = req.files;
  if (!files || !files.cam1 || !files.cam2) {
    return res.status(400).json({ error: 'Both cam1 and cam2 files are required' });
  }

  const requestedMode = typeof req.body?.viewMode === 'string'
    ? req.body.viewMode.trim().toLowerCase()
    : 'depth';
  if (!['depth', 'undistort', 'undistort_cam1', 'undistort_cam2'].includes(requestedMode)) {
    return res.status(400).json({ error: 'Invalid viewMode. Expected depth, undistort, undistort_cam1 or undistort_cam2' });
  }

  // Write uploaded buffers to unique temp files
  const id      = crypto.randomUUID();
  const tmpDir  = os.tmpdir();
  const img1    = path.join(tmpDir, `${id}_cam1.jpg`);
  const img2    = path.join(tmpDir, `${id}_cam2.jpg`);
  const outPng  = path.join(tmpDir, `${id}_depth.png`);

  const cleanup = () => {
    for (const f of [img1, img2, outPng]) {
      try { fs.unlinkSync(f); } catch { /* already gone */ }
    }
  };

  try {
    fs.writeFileSync(img1, files.cam1[0].buffer);
    fs.writeFileSync(img2, files.cam2[0].buffer);
  } catch (e) {
    cleanup();
    return res.status(500).json({ error: 'Failed to write temp files: ' + e.message });
  }

  const args = [DEPTHMAP_PY, img1, img2, outPng];
  if (fs.existsSync(CALIB_JSON)) args.push(CALIB_JSON);
  args.push(requestedMode);

  const result = spawnSync('python3', args, { timeout: 60_000, encoding: 'utf8' });

  if (result.error) {
    cleanup();
    return res.status(500).json({ error: 'Failed to spawn python3: ' + result.error.message });
  }

  let pyOut = {};
  try { pyOut = JSON.parse((result.stdout || '').trim()); } catch { /* ignore */ }

  if (result.status !== 0 || !pyOut.success) {
    cleanup();
    const errMsg = pyOut.error || result.stderr || 'depthmap.py failed';
    console.error('[depthmap]', errMsg);
    return res.status(500).json({ error: errMsg });
  }

  const outputMode = typeof pyOut.mode === 'string' ? pyOut.mode : requestedMode;
  console.log(`[depthmap] ${outputMode} (${pyOut.calibrated ? 'calibrated' : 'uncalibrated'}) written to ${outPng}`);
  let fileName = 'depthmap.png';
  if (outputMode === 'undistort') fileName = 'undistorted_preview.png';
  if (outputMode === 'undistort_cam1') fileName = 'undistorted_cam1.png';
  if (outputMode === 'undistort_cam2') fileName = 'undistorted_cam2.png';
  res.download(outPng, fileName, (err) => {
    cleanup();
    if (err && !res.headersSent) res.status(500).json({ error: 'Download failed' });
  });
});

// ─── WebSocket server on /ws ───
const wss = new WebSocket.Server({ server, path: '/ws'/*, perMessageDeflate: false*/});

// Bookkeeping
const espClients     = new Map();    // camId → ws
const browserClients = new Set();

// Image cache so late-joining browsers get the last frame immediately
const latestImages = { cam1: null, cam2: null };
const firmwareVersions = { cam1: null, cam2: null };

let captureTimer = null;
let masterOTAPending   = false;
let slaveOTAPending = false;
const CAPTURE_INTERVAL_MS = 2000;  // default auto-capture rate

// ─── Connection handler ───
wss.on('connection', (ws, req) => {
  const ip = req.socket.remoteAddress;
  console.log(`+ connection from ${ip}`);

  ws.isAlive    = true;
  ws.clientType = null;   // 'esp' | 'browser'
  ws.cameraId   = null;

  // Close connections that never identify themselves within 10 seconds
  const authTimeout = setTimeout(() => {
    if (!ws.clientType) {
      console.warn(`  Auth timeout — closing unidentified connection from ${ip}`);
      ws.terminate();
    }
  }, 10_000);

  ws.on('pong', () => { ws.isAlive = true; });
  ws.on('close', () => clearTimeout(authTimeout));

  ws.on('message', (data, isBinary) => {
    // ── Text messages ──
    if (!isBinary) {
      const text = data.toString();

      // ESP32 registration
      if (text.startsWith('register:')) {
        const parts  = text.split(':');   // ['register', camId, token, firmwareVersion]
        const camId  = parts[1] ?? '';
        const token  = parts[2] ?? '';
        const fwVersion = parts[3] ?? null;
        if (REGISTER_TOKEN && token !== REGISTER_TOKEN) {
          console.warn(`  Rejected ESP registration from ${ip} — bad token`);
          ws.terminate();
          return;
        }
        ws.clientType = 'esp';
        ws.cameraId   = camId;
        ws.firmwareVersion = fwVersion;
        espClients.set(camId, ws);
        if (camId === 'cam1' || camId === 'cam2') {
          firmwareVersions[camId] = fwVersion;
        }
        console.log(`  ESP registered: ${camId}  (total ${espClients.size})`);
        if (masterOTAPending) {
          if(camId === "cam1"){
            ws.send('master_ota_update');
            masterOTAPending = false;
            console.log(`  OTA update command sent to ${camId}`);
          }
        }
        if (slaveOTAPending){
          if (camId === "cam2"){
            ws.send('slave_ota_update');
            slaveOTAPending = false;
            console.log(`  OTA update command sent to ${camId}`);
          }
        }
        maybeStartCapture();
        broadcastStatus();
        return;
      }

      // Browser registration
      if (text === 'browser') {
        ws.clientType = 'browser';
        browserClients.add(ws);
        console.log(`  Browser joined (total ${browserClients.size})`);

        // Send status
        sendStatus(ws);

        // Send cached frames
        for (const camId of ['cam1', 'cam2']) {
          if (latestImages[camId]) {
            const hdr = Buffer.from(`${camId}:`, 'utf-8');
            ws.send(Buffer.concat([hdr, latestImages[camId]]));
          }
        }
        return;
      }

      // Browser commands
      if (text === 'trigger_capture') { triggerCapture(); return; }
      if (text === 'start_auto')      { startCapture();   return; }
      if (text === 'stop_auto')       { stopCapture();    return; }
      if (text === 'request_ota')     { masterOTAPending = true; slaveOTAPending = true; broadcastStatus(); return; }
      return;
    }

    // ── Binary messages (image from ESP) ──
    const buf       = Buffer.from(data);
 
    const camId = buf.subarray(0, 8).toString('utf8').replace(/\0/g, '');
    const timestamp = buf.readBigUInt64LE(8);
    const dataLen = buf.readUInt32LE(16);
    const jpegData = buf.subarray(24, 24+dataLen);

    console.log(`Image ${camId}, ts=${timestamp}, dataLen =${dataLen}, size=${jpegData.length}`);

    latestImages[camId] = jpegData;

    // Build normalized frame: "camId:" + JPEG (consistent format for browsers)
    // Build normalized frame: "camId:timestamp:" + JPEG
    const hdr = Buffer.from(`${camId}:${timestamp.toString()}:`, 'utf-8');
    const browserFrame = Buffer.concat([hdr, jpegData]);

    // Relay to browsers
    for (const b of browserClients) {
      if (b.readyState === WebSocket.OPEN) b.send(browserFrame);
    }
  });

  ws.on('close', () => {
    if (ws.clientType === 'esp') {
      espClients.delete(ws.cameraId);
      latestImages[ws.cameraId] = null;
      if (ws.cameraId === 'cam1' || ws.cameraId === 'cam2') {
        firmwareVersions[ws.cameraId] = null;
      }
      console.log(`- ESP disconnected: ${ws.cameraId}`);
      if (espClients.size === 0) stopCapture();
      broadcastStatus();
    } else if (ws.clientType === 'browser') {
      browserClients.delete(ws);
      console.log(`- Browser left (total ${browserClients.size})`);
    }
  });

  ws.on('error', (e) => console.error('ws error:', e.message));
});

// ─── Capture helpers ───
function triggerCapture() {
  for (const [id, ws] of espClients) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send('capture');
    }
  }
}

function startCapture() {
  // if (captureTimer) return;
  // captureTimer = setInterval(triggerCapture, CAPTURE_INTERVAL_MS);
  // console.log(`Auto-capture ON  (${CAPTURE_INTERVAL_MS} ms)`);
  // broadcastStatus();
}

function stopCapture() {
  // if (!captureTimer) return;
  // clearInterval(captureTimer);
  // captureTimer = null;
  // console.log('Auto-capture OFF');
  // broadcastStatus();
}

function maybeStartCapture() {
  //if (espClients.size >= 1 && !captureTimer) startCapture();
}

// ─── Status helpers ───
function makeStatus() {
  return JSON.stringify({
    type: 'status',
    cameras: Array.from(espClients.keys()),
    captureActive: captureTimer !== null,
    masterOTAPending,
    slaveOTAPending,
    firmwareVersions
  });
}

function sendStatus(ws) {
  if (ws.readyState === WebSocket.OPEN) ws.send(makeStatus());
}

function broadcastStatus() {
  const msg = makeStatus();
  for (const b of browserClients) {
    if (b.readyState === WebSocket.OPEN) b.send(msg);
  }
}

// ─── Heartbeat (detect dead sockets) ───
const heartbeat = setInterval(() => {
  for (const ws of wss.clients) {
    if (!ws.isAlive) return ws.terminate();
    ws.isAlive = false;
    ws.ping();
  }
}, 30_000);

wss.on('close', () => clearInterval(heartbeat));

// ─── Start ───
const PORT = 3000;
server.listen(PORT, '0.0.0.0', () => {
  console.log(`\n🟢  Server listening on http://0.0.0.0:${PORT}`);
  console.log('    Waiting for ESP32-CAM connections …\n');
});
