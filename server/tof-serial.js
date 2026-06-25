/**
 * tof-serial.js  —  VL53L5CX Serial → WebSocket relay
 *
 * Reads newline-delimited JSON from tof_sensor.ino over USB Serial and
 * forwards each validated frame to every browser client connected to the
 * existing camera server at ws://localhost:3000/ws.
 *
 * server.js is NOT modified.  This process connects to it as a WebSocket
 * client tagged "tof_relay".  The only required addition to server.js is a
 * single fallthrough at the end of the text-message handler so that messages
 * from unrecognised client types reach the browsers:
 *
 *   ─── server.js  (inside the `if (!isBinary)` block, after all existing checks) ───
 *   // Relay text from auxiliary clients (e.g. tof_relay) to all browsers
 *   if (ws.clientType === null || ws.clientType === 'tof_relay') {
 *     for (const b of browserClients) {
 *       if (b.readyState === WebSocket.OPEN) b.send(text);
 *     }
 *   }
 *   ────────────────────────────────────────────────────────────────────────────────
 *
 * Usage:
 *   node tof-serial.js [serialPort] [baudRate]
 *
 *   serialPort  default: /dev/ttyUSB0   (Windows: COM3  macOS: /dev/tty.usbserial-*)
 *   baudRate    default: 115200
 *
 * Install deps once:
 *   npm install serialport @serialport/parser-readline ws
 */

'use strict';

const { SerialPort }     = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const WebSocket          = require('ws');

// ─── Configuration ────────────────────────────────────────────────────────────
const SERIAL_PORT     = process.argv[2] ?? '/dev/ttyUSB0';
const BAUD_RATE       = parseInt(process.argv[3] ?? '115200', 10);
const WS_URL          = 'ws://localhost:3000/ws';
const WS_RETRY_MS     = 3_000;
const SERIAL_RETRY_MS = 5_000;
const QUEUE_MAX       = 10;    // max frames buffered while WS reconnects

// VL53L5CX constants (match firmware)
const VALID_STATUS    = 5;
const PROXIMITY_MM    = 500;
const PROXIMITY_RATIO = 0.5;

// ─── Per-frame statistics ─────────────────────────────────────────────────────
// Computed here so the browser receives ready-made numbers without extra JS.
function computeStats(distances, statuses) {
  const valid = [], close = [];
  for (let i = 0; i < 64; i++) {
    if (statuses[i] === VALID_STATUS) {
      valid.push(distances[i]);
      if (distances[i] < PROXIMITY_MM) close.push(distances[i]);
    }
  }
  if (!valid.length) return { min: null, max: null, mean: null, validZones: 0, closeZones: 0 };
  const sum = valid.reduce((a, b) => a + b, 0);
  return {
    min:        Math.min(...valid),
    max:        Math.max(...valid),
    mean:       Math.round(sum / valid.length),
    validZones: valid.length,
    closeZones: close.length,
  };
}

// ─── WebSocket relay ──────────────────────────────────────────────────────────
let wsRelay = null;
let wsReady = false;
const queue = [];

function connectWS() {
  console.log(`[WS]  Connecting to ${WS_URL} …`);
  wsRelay = new WebSocket(WS_URL);

  wsRelay.on('open', () => {
    console.log('[WS]  Connected to camera server');
    wsRelay.send('tof_relay');
    wsReady = true;
    while (queue.length) wsRelay.send(queue.shift());
  });

  wsRelay.on('close', () => {
    console.warn(`[WS]  Disconnected — retry in ${WS_RETRY_MS} ms`);
    wsReady = false;
    setTimeout(connectWS, WS_RETRY_MS);
  });

  wsRelay.on('error', (err) => console.error('[WS]  Error:', err.message));
}

function relayFrame(json) {
  if (wsReady && wsRelay.readyState === WebSocket.OPEN) {
    wsRelay.send(json);
  } else {
    if (queue.length < QUEUE_MAX) queue.push(json);
  }
}

// ─── Serial reader ────────────────────────────────────────────────────────────
function openSerial() {
  console.log(`[Serial]  Opening ${SERIAL_PORT} @ ${BAUD_RATE} baud …`);

  let port;
  try {
    port = new SerialPort({ path: SERIAL_PORT, baudRate: BAUD_RATE });
  } catch (err) {
    console.error('[Serial]  Could not open port:', err.message);
    setTimeout(openSerial, SERIAL_RETRY_MS);
    return;
  }

  const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));

  port.on('open',  ()    => console.log('[Serial]  Port open'));
  port.on('error', (err) => console.error('[Serial]  Error:', err.message));
  port.on('close', ()    => {
    console.warn(`[Serial]  Closed — retry in ${SERIAL_RETRY_MS} ms`);
    setTimeout(openSerial, SERIAL_RETRY_MS);
  });

  parser.on('data', (raw) => {
    const line = raw.trim();
    if (!line.startsWith('{')) return;       // skip boot messages

    let obj;
    try { obj = JSON.parse(line); }
    catch { console.warn('[Serial]  Bad JSON:', line.slice(0, 60)); return; }

    // Only forward frames that carry distance data
    if (!Array.isArray(obj.distances) || obj.distances.length !== 64) return;

    const statuses = Array.isArray(obj.status) ? obj.status : new Array(64).fill(VALID_STATUS);
    const stats    = computeStats(obj.distances, statuses);

    // Derive alert robustly (handles older firmware that may omit the field)
    const proximityAlert = obj.proximity_alert ??
      (stats.validZones > 0 && (stats.closeZones / stats.validZones) > PROXIMITY_RATIO);

    const frame = JSON.stringify({
      type:            'tof_frame',
      distances:       obj.distances,
      status:          statuses,
      proximity_alert: proximityAlert,
      stats,
      v:  obj.v  ?? '?',
      ts: Date.now(),
    });

    relayFrame(frame);

    console.log(
      `[ToF]  alert=${String(proximityAlert).padEnd(5)}` +
      `  valid=${String(stats.validZones).padStart(2)}` +
      `  close=${String(stats.closeZones).padStart(2)}` +
      `  min=${String(stats.min ?? '—').padStart(5)} mm` +
      `  mean=${String(stats.mean ?? '—').padStart(5)} mm`
    );
  });
}

// ─── Boot ─────────────────────────────────────────────────────────────────────
connectWS();
openSerial();
console.log('ToF Serial relay started.  Press Ctrl-C to stop.\n');
