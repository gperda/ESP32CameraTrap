/**
 * server.js — WebSocket relay for dual ESP32-CAM viewer
 *
 * ESP32 cameras connect on  ws://<host>:3000/ws   (binary + text)
 * Browsers      connect on  ws://<host>:3000/ws   (register with "browser")
 *
 * Flow:
 *   1. Each ESP32 sends  register:cam1  or  register:cam2
 *   2. Server periodically sends "capture" to every ESP32
 *   3. ESP32 replies with binary:  "camX:" + JPEG
 *   4. Server relays that binary verbatim to every browser client
 */

const express = require('express');
const http    = require('http');
const WebSocket = require('ws');
const path    = require('path');
const { buffer } = require('stream/consumers');

// ─── Express static file server ───
const app    = express();
const server = http.createServer(app);
app.use(express.static(path.join(__dirname, 'public')));

// ─── WebSocket server on /ws ───
const wss = new WebSocket.Server({ server, path: '/ws' });

// Bookkeeping
const espClients     = new Map();    // camId → ws
const browserClients = new Set();

// Image cache so late-joining browsers get the last frame immediately
const latestImages = { cam1: null, cam2: null };

let captureTimer = null;
const CAPTURE_INTERVAL_MS = 2000;  // default auto-capture rate

// ─── Connection handler ───
wss.on('connection', (ws, req) => {
  const ip = req.socket.remoteAddress;
  console.log(`+ connection from ${ip}`);
  

  ws.isAlive    = true;
  ws.clientType = null;   // 'esp' | 'browser'
  ws.cameraId   = null;

  ws.on('pong', () => { ws.isAlive = true; });

  ws.on('message', (data, isBinary) => {
    // ── Text messages ──
    if (!isBinary) {
      const text = data.toString();

      // ESP32 registration
      if (text.startsWith('register:')) {
        const camId = text.slice(9);
        ws.clientType = 'esp';
        ws.cameraId   = camId;
        espClients.set(camId, ws);
        console.log(`  ESP registered: ${camId}  (total ${espClients.size})`);
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
    const browserFrame = Buffer.concat([
      Buffer.from(`${camId}:`, 'utf-8'),
      jpegData
    ]);

    // Relay to browsers
    for (const b of browserClients) {
      if (b.readyState === WebSocket.OPEN) b.send(browserFrame);
    }
  });

  ws.on('close', () => {
    if (ws.clientType === 'esp') {
      espClients.delete(ws.cameraId);
      latestImages[ws.cameraId] = null;
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
    captureActive: captureTimer !== null
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
