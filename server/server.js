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
const DATA_ROOT = process.env.DATA_ROOT;
const server = http.createServer(app);
app.use(express.static(path.join(__dirname, 'public')));

// ─── Stereo depth map endpoint ────────────────────────────────────────────────
const upload     = multer({ storage: multer.memoryStorage(), limits: { fileSize: 10 * 1024 * 1024 } });
const DEPTHMAP_PY      = path.join(__dirname, 'depthmap.py');
const TRIANGULATION_PY = path.join(__dirname, 'triangulation.py');
const DETECTION_PY     = path.join(__dirname, 'detection_segmentation.py');
const CALIB_JSON       = path.join(__dirname, 'calibration.json');
const FRAME_ROOT       = path.join(DATA_ROOT, 'frames');
const FRAME_INDEX_FILE = path.join(DATA_ROOT, 'frame_index.ndjson');
const TOF_DUMP_ROOT       = path.join(DATA_ROOT, 'tofdumps');
const PROCESSED_ROOT        = path.join(DATA_ROOT, 'processed');
const PROCESSED_INDEX_FILE  = path.join(DATA_ROOT, 'processed_index.ndjson');
const HISTORY_DEFAULT_LIMIT = 200;
const HISTORY_MAX_LIMIT = 2000;

// Process enum — all valid process identifiers for the artifact pipeline
const VALID_PROCESSES = new Set([
  'undistort', 'undistort_cam1', 'undistort_cam2', 'depthmap', 'detection', 'segmentation',
]);
const HISTORY_PROCESS_BUTTONS = ['depthmap', 'undistort', 'detection'];

const persistedFrames = [];

function parseTimestampMs(tsStr) {
  if (typeof tsStr !== 'string' || !/^\d+$/.test(tsStr)) return null;
  const n = Number(tsStr);
  if (!Number.isFinite(n)) return null;
  if (tsStr.length >= 16) return Math.floor(n / 1000);
  if (tsStr.length === 13) return n;
  if (tsStr.length === 10) return n * 1000;
  return n;
}

function parseJsonFromStdout(stdoutText) {
  const text = String(stdoutText || '').trim();
  if (!text) return null;

  try {
    return JSON.parse(text);
  } catch {
    // Some Python tools print extra logs; use the last valid JSON line.
    const lines = text.split(/\r?\n/);
    for (let i = lines.length - 1; i >= 0; i -= 1) {
      const line = lines[i].trim();
      if (!line) continue;
      if (!line.startsWith('{') && !line.startsWith('[')) continue;
      try {
        return JSON.parse(line);
      } catch {
        // Keep scanning older lines.
      }
    }
  }

  return null;
}

function ensurePersistenceStorage() {
  fs.mkdirSync(FRAME_ROOT, { recursive: true });
  if (!fs.existsSync(FRAME_INDEX_FILE)) {
    fs.writeFileSync(FRAME_INDEX_FILE, '');
  }
}

function loadPersistedFramesFromIndex() {
  let raw = '';
  try {
    raw = fs.readFileSync(FRAME_INDEX_FILE, 'utf8');
  } catch (e) {
    console.warn('[history] Failed to read frame index:', e.message);
    return;
  }
  for (const line of raw.split('\n')) {
    if (!line.trim()) continue;
    try {
      const parsed = JSON.parse(line);
      if (!parsed || typeof parsed !== 'object') continue;
      if (typeof parsed.camId !== 'string') continue;
      if (typeof parsed.tsStr !== 'string') continue;
      if (typeof parsed.relPath !== 'string') continue;
      parsed.tsMs = parseTimestampMs(parsed.tsStr);
      if (!Number.isFinite(parsed.tsMs)) continue;
      persistedFrames.push(parsed);
    } catch {
      // Keep startup resilient even if one index line is malformed.
    }
  }
}

function makeHistoryImageUrl(camId, tsStr) {
  return `/api/history/image?camId=${encodeURIComponent(camId)}&ts=${encodeURIComponent(tsStr)}`;
}

function makeHistoryTofDumpUrl(tsStr) {
  return `/api/history/tofdump?ts=${encodeURIComponent(tsStr)}`;
}

function makeTofDumpAbsPath(tsStr) {
  return path.join(TOF_DUMP_ROOT, `${tsStr}.json`);
}

function extractTimestampFromTofDumpName(fileName) {
  const base = path.basename(String(fileName || '').trim());
  const ext = path.extname(base);
  const stem = ext ? base.slice(0, -ext.length) : base;
  if (!/^\d+$/.test(stem)) return null;
  return stem;
}

function ensureTofDumpStorage() {
  fs.mkdirSync(TOF_DUMP_ROOT, { recursive: true });
}

function persistIncomingTofDump(tsStr, payload, source = 'unknown') {
  const absPath = makeTofDumpAbsPath(tsStr);
  const jsonText = JSON.stringify(payload);

  fs.writeFile(absPath, jsonText, 'utf8', (writeErr) => {
    if (writeErr) {
      console.warn('[tofdump] write failed:', writeErr.message);
      return;
    }

    console.log(`[tofdump] persisted ${tsStr}.json (${source})`);
  });
}

function persistIncomingFrame(camId, tsStr, jpegData) {
  const safeCamId = String(camId || 'unknown').replace(/[^a-zA-Z0-9_-]/g, '_');
  const camDir = path.join(FRAME_ROOT, safeCamId);
  const fileName = `${tsStr}.jpg`;
  const absPath = path.join(camDir, fileName);
  const relPath = path.relative(__dirname, absPath).split(path.sep).join('/');

  fs.mkdir(camDir, { recursive: true }, (mkdirErr) => {
    if (mkdirErr) {
      console.warn('[history] mkdir failed:', mkdirErr.message);
      return;
    }

    fs.writeFile(absPath, jpegData, (writeErr) => {
      if (writeErr) {
        console.warn('[history] frame write failed:', writeErr.message);
        return;
      }

      const entry = {
        camId,
        tsStr,
        tsMs: parseTimestampMs(tsStr),
        sizeBytes: jpegData.length,
        relPath,
        storedAtMs: Date.now(),
      };

      if (!Number.isFinite(entry.tsMs)) return;
      persistedFrames.push(entry);
      fs.appendFile(FRAME_INDEX_FILE, JSON.stringify(entry) + '\n', (appendErr) => {
        if (appendErr) {
          console.warn('[history] frame index append failed:', appendErr.message);
        }
      });
    });
  });
}

ensurePersistenceStorage();
loadPersistedFramesFromIndex();
ensureTofDumpStorage();

// ─── Processed artifact helpers ───────────────────────────────────────────────
function sanitizeProcess(proc) {
  if (typeof proc !== 'string') return null;
  const p = proc.trim().toLowerCase();
  return VALID_PROCESSES.has(p) ? p : null;
}

function buildArtifactFileName(tsStr, proc, ext) {
  return `${tsStr}_${proc}.${ext}`;
}

function parseForceFlag(value) {
  if (typeof value === 'boolean') return value;
  if (typeof value === 'number') return value === 1;
  if (typeof value !== 'string') return false;
  const normalized = value.trim().toLowerCase();
  return normalized === '1' || normalized === 'true' || normalized === 'yes' || normalized === 'on';
}

function makeProcessedArtifactUrl(tsStr, process, ext) {
  const fileName = buildArtifactFileName(tsStr, process, ext || 'png');
  return `/api/processed/${encodeURIComponent(tsStr)}/${encodeURIComponent(fileName)}`;
}

const processedArtifacts = [];

function ensureProcessedStorage() {
  fs.mkdirSync(PROCESSED_ROOT, { recursive: true });
  if (!fs.existsSync(PROCESSED_INDEX_FILE)) {
    fs.writeFileSync(PROCESSED_INDEX_FILE, '');
  }
}

function loadProcessedArtifactsFromIndex() {
  let raw = '';
  try { raw = fs.readFileSync(PROCESSED_INDEX_FILE, 'utf8'); }
  catch (e) { console.warn('[processed] Failed to read index:', e.message); return; }
  for (const line of raw.split('\n')) {
    if (!line.trim()) continue;
    try {
      const parsed = JSON.parse(line);
      if (!parsed || typeof parsed !== 'object') continue;
      if (typeof parsed.tsStr !== 'string' || typeof parsed.process !== 'string') continue;
      processedArtifacts.push(parsed);
    } catch { /* skip malformed lines */ }
  }
}

function registerProcessedArtifact(record) {
  const idx = processedArtifacts.findIndex(r => r.tsStr === record.tsStr && r.process === record.process);
  if (idx !== -1) {
    processedArtifacts[idx] = record;
    // Full rewrite for overwrite (deterministic reference)
    const content = processedArtifacts.map(r => JSON.stringify(r)).join('\n') + '\n';
    fs.writeFile(PROCESSED_INDEX_FILE, content, (err) => {
      if (err) console.warn('[processed] index rewrite failed:', err.message);
    });
  } else {
    processedArtifacts.push(record);
    fs.appendFile(PROCESSED_INDEX_FILE, JSON.stringify(record) + '\n', (err) => {
      if (err) console.warn('[processed] index append failed:', err.message);
    });
  }
}

function getLatestArtifact(tsStr, proc) {
  for (let i = processedArtifacts.length - 1; i >= 0; i--) {
    const r = processedArtifacts[i];
    if (r.tsStr === tsStr && r.process === proc) return r;
  }
  return null;
}

ensureProcessedStorage();
loadProcessedArtifactsFromIndex();

app.get('/api/history', (req, res) => {
  const startMsRaw = req.query.startMs;
  const endMsRaw = req.query.endMs;
  const offsetRaw = req.query.offset;
  const limitRaw = req.query.limit;

  const startMs = startMsRaw == null || startMsRaw === '' ? null : Number(startMsRaw);
  const endMs = endMsRaw == null || endMsRaw === '' ? null : Number(endMsRaw);
  const offset = Math.max(0, Number(offsetRaw || 0) || 0);
  const requestedLimit = Number(limitRaw || HISTORY_DEFAULT_LIMIT) || HISTORY_DEFAULT_LIMIT;
  const limit = Math.min(HISTORY_MAX_LIMIT, Math.max(1, requestedLimit));

  if (startMs !== null && !Number.isFinite(startMs)) {
    return res.status(400).json({ error: 'startMs must be a number' });
  }
  if (endMs !== null && !Number.isFinite(endMs)) {
    return res.status(400).json({ error: 'endMs must be a number' });
  }
  if (startMs !== null && endMs !== null && startMs > endMs) {
    return res.status(400).json({ error: 'startMs cannot be greater than endMs' });
  }

  const groupedByTs = new Map();
  for (let i = persistedFrames.length - 1; i >= 0; i -= 1) {
    const entry = persistedFrames[i];
    if (!entry || !Number.isFinite(entry.tsMs)) continue;
    if (startMs !== null && entry.tsMs < startMs) continue;
    if (endMs !== null && entry.tsMs > endMs) continue;
    if (!groupedByTs.has(entry.tsStr)) {
      groupedByTs.set(entry.tsStr, {
        tsStr: entry.tsStr,
        tsMs: entry.tsMs,
        cam1: null,
        cam2: null,
        tofDump: null,
      });
    }
    const row = groupedByTs.get(entry.tsStr);
    if (entry.camId !== 'cam1' && entry.camId !== 'cam2') continue;
    if (row[entry.camId]) continue;

    row[entry.camId] = {
      camId: entry.camId,
      sizeBytes: entry.sizeBytes,
      sizeKB: (entry.sizeBytes / 1024).toFixed(1),
      url: makeHistoryImageUrl(entry.camId, entry.tsStr),
      storedAtMs: entry.storedAtMs,
    };
  }

  const rows = Array.from(groupedByTs.values()).sort((a, b) => b.tsMs - a.tsMs);
  const tsSet = new Set(rows.map(r => r.tsStr));
  const processByTs = new Map();

  for (let i = processedArtifacts.length - 1; i >= 0; i -= 1) {
    const record = processedArtifacts[i];
    if (!record || !tsSet.has(record.tsStr)) continue;
    if (!HISTORY_PROCESS_BUTTONS.includes(record.process)) continue;

    if (!processByTs.has(record.tsStr)) processByTs.set(record.tsStr, {});
    const rowProcess = processByTs.get(record.tsStr);
    if (rowProcess[record.process]) continue;

    rowProcess[record.process] = {
      available: true,
      url: makeProcessedArtifactUrl(record.tsStr, record.process, record.ext),
      createdAtMs: Number.isFinite(record.createdAtMs) ? record.createdAtMs : null,
    };
  }

  for (const row of rows) {
    const tofDumpAbsPath = makeTofDumpAbsPath(row.tsStr);
    if (fs.existsSync(tofDumpAbsPath)) {
      let dumpStats = null;
      try {
        dumpStats = fs.statSync(tofDumpAbsPath);
      } catch {
        dumpStats = null;
      }
      row.tofDump = {
        available: true,
        tsStr: row.tsStr,
        fileName: `${row.tsStr}.json`,
        sizeBytes: dumpStats ? Number(dumpStats.size) || 0 : null,
        storedAtMs: dumpStats ? Number(dumpStats.mtimeMs) || null : null,
        url: makeHistoryTofDumpUrl(row.tsStr),
      };
    } else {
      row.tofDump = {
        available: false,
        tsStr: row.tsStr,
        fileName: null,
        sizeBytes: null,
        storedAtMs: null,
        url: null,
      };
    }

    const processOutputs = {};
    const existing = processByTs.get(row.tsStr) || {};
    for (const processName of HISTORY_PROCESS_BUTTONS) {
      processOutputs[processName] = existing[processName] || {
        available: false,
        url: null,
        createdAtMs: null,
      };
    }
    row.processOutputs = processOutputs;
  }

  const paged = rows.slice(offset, offset + limit);

  res.json({
    items: paged,
    total: rows.length,
    offset,
    limit,
  });
});

app.get('/api/history/image', (req, res) => {
  const camId = String(req.query.camId || '');
  const tsStr = String(req.query.ts || '');
  if (!camId || !tsStr) {
    return res.status(400).json({ error: 'camId and ts are required' });
  }

  let matched = null;
  for (let i = persistedFrames.length - 1; i >= 0; i -= 1) {
    const entry = persistedFrames[i];
    if (entry.camId === camId && entry.tsStr === tsStr) {
      matched = entry;
      break;
    }
  }
  if (!matched) {
    return res.status(404).json({ error: 'Image not found' });
  }

  const absPath = path.join(__dirname, matched.relPath);
  if (!fs.existsSync(absPath)) {
    return res.status(404).json({ error: 'Image file is missing on disk' });
  }

  res.setHeader('Content-Type', 'image/jpeg');
  res.setHeader('Cache-Control', 'private, max-age=60');
  res.sendFile(absPath);
});

app.get('/api/history/tofdump', (req, res) => {
  const tsStr = String(req.query.ts || '').trim();
  if (!tsStr || !/^\d+$/.test(tsStr)) {
    return res.status(400).json({ error: 'ts is required and must be numeric' });
  }

  const absPath = makeTofDumpAbsPath(tsStr);
  if (!fs.existsSync(absPath)) {
    return res.status(404).json({ error: 'ToF dump file is missing on disk' });
  }

  let stats = null;
  try {
    stats = fs.statSync(absPath);
  } catch {
    stats = null;
  }

  let payload;
  try {
    payload = JSON.parse(fs.readFileSync(absPath, 'utf8'));
  } catch (e) {
    return res.status(500).json({ error: `Failed to parse TOF dump JSON: ${e.message}` });
  }

  res.json({
    tsStr,
    fileName: `${tsStr}.json`,
    storedAtMs: stats ? Number(stats.mtimeMs) || null : null,
    source: null,
    payload,
  });
});

app.post('/api/depthmap', upload.fields([{ name: 'cam1', maxCount: 1 }, { name: 'cam2', maxCount: 1 }]), (req, res) => {
  const files = req.files;

  const requestedMode = typeof req.body?.viewMode === 'string'
    ? req.body.viewMode.trim().toLowerCase()
    : 'depth';
  if (!['depth', 'undistort', 'undistort_cam1', 'undistort_cam2'].includes(requestedMode)) {
    return res.status(400).json({ error: 'Invalid viewMode. Expected depth, undistort, undistort_cam1 or undistort_cam2' });
  }

  const tsStr = typeof req.body?.tsStr === 'string' ? req.body.tsStr.trim() : null;
  const forceRecompute = parseForceFlag(req.body?.forceRecompute);
  if (tsStr !== null && !/^\d+$/.test(tsStr)) {
    return res.status(400).json({ error: 'Invalid tsStr format: must be numeric' });
  }

  const requestedProcessName = requestedMode === 'depth' ? 'depthmap' : requestedMode;
  if (tsStr && !forceRecompute) {
    const existing = getLatestArtifact(tsStr, requestedProcessName);
    if (existing && existing.absPath && fs.existsSync(existing.absPath)) {
      return res.json({
        success: true,
        cached: true,
        tsStr,
        process: requestedProcessName,
        url: makeProcessedArtifactUrl(tsStr, requestedProcessName, existing.ext),
        relPath: existing.relPath,
        calibrated: !!(existing.metadata && existing.metadata.calibrated),
      });
    }
  }

  // Determine input buffers
  let cam1Buffer = null;
  let cam2Buffer = null;

  if (tsStr && !files?.cam1 && !files?.cam2) {
    // Chain mode: resolve raw frames from persisted history
    const cam1Frame = persistedFrames.find(f => f.tsStr === tsStr && f.camId === 'cam1');
    const cam2Frame = persistedFrames.find(f => f.tsStr === tsStr && f.camId === 'cam2');
    if (!cam1Frame || !cam2Frame) {
      return res.status(404).json({ error: `No cam1/cam2 frames found in history for tsStr=${tsStr}` });
    }
    try {
      cam1Buffer = fs.readFileSync(path.join(__dirname, cam1Frame.relPath));
      cam2Buffer = fs.readFileSync(path.join(__dirname, cam2Frame.relPath));
    } catch (e) {
      return res.status(404).json({ error: 'Frame files not found on disk' });
    }
  } else {
    if (!files || !files.cam1 || !files.cam2) {
      return res.status(400).json({ error: 'Both cam1 and cam2 files are required' });
    }
    cam1Buffer = files.cam1[0].buffer;
    cam2Buffer = files.cam2[0].buffer;
  }

  // Write uploaded buffers to unique temp files
  const id      = crypto.randomUUID();
  const tmpDir  = os.tmpdir();
  const img1    = path.join(tmpDir, `${id}_cam1.jpg`);
  const img2    = path.join(tmpDir, `${id}_cam2.jpg`);
  const outPng  = tsStr
    ? path.join(PROCESSED_ROOT, tsStr, buildArtifactFileName(tsStr, requestedProcessName, 'png'))
    : path.join(tmpDir, `${id}_depth.png`);

  const cleanup = () => {
    const tempFiles = [img1, img2];
    if (!tsStr) tempFiles.push(outPng);
    for (const f of tempFiles) {
      try { fs.unlinkSync(f); } catch { /* already gone */ }
    }
  };

  if (tsStr) {
    try { fs.mkdirSync(path.dirname(outPng), { recursive: true }); }
    catch (e) {
      return res.status(500).json({ error: 'Failed to create output dir: ' + e.message });
    }
  }

  try {
    fs.writeFileSync(img1, cam1Buffer);
    fs.writeFileSync(img2, cam2Buffer);
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

  const pyOut = parseJsonFromStdout(result.stdout) || {};

  if (result.status !== 0 || !pyOut.success) {
    cleanup();
    if (tsStr) {
      try { fs.unlinkSync(outPng); } catch { /* ignore partial output */ }
    }
    const errMsg = pyOut.error || result.stderr || 'depthmap.py failed';
    console.error('[depthmap]', errMsg);
    return res.status(500).json({ error: errMsg });
  }

  const outputMode = typeof pyOut.mode === 'string' ? pyOut.mode : requestedMode;
  console.log(`[depthmap] ${outputMode} (${pyOut.calibrated ? 'calibrated' : 'uncalibrated'}) written to ${outPng}`);

  if (tsStr) {
    // Persist output with tsStr-based naming
    const processName = requestedProcessName;
    const outFileName = buildArtifactFileName(tsStr, processName, 'png');
    const persistedAbsPath = outPng;
    const relPath = path.relative(__dirname, persistedAbsPath).split(path.sep).join('/');
    if (!fs.existsSync(persistedAbsPath)) {
      cleanup();
      return res.status(500).json({ error: 'Expected output file missing after processing' });
    }

    const record = {
      tsStr,
      process: processName,
      ext: 'png',
      relPath,
      absPath: persistedAbsPath,
      sourceProcess: null,
      sourceRelPath: null,
      createdAtMs: Date.now(),
      metadata: { calibrated: pyOut.calibrated || false },
    };
    registerProcessedArtifact(record);
    cleanup();

    return res.json({
      success: true,
      cached: false,
      tsStr,
      process: processName,
      url: `/api/processed/${encodeURIComponent(tsStr)}/${encodeURIComponent(outFileName)}`,
      relPath,
      calibrated: pyOut.calibrated || false,
    });
  }

  // Legacy mode (no tsStr): return blob download
  let fileName = 'depthmap.png';
  if (outputMode === 'undistort') fileName = 'undistorted_preview.png';
  if (outputMode === 'undistort_cam1') fileName = 'undistorted_cam1.png';
  if (outputMode === 'undistort_cam2') fileName = 'undistorted_cam2.png';
  res.download(outPng, fileName, (err) => {
    cleanup();
    if (err && !res.headersSent) res.status(500).json({ error: 'Download failed' });
  });
});

// ─── Stereo triangulation endpoint ────────────────────────────────────────────
// Accepts two pixel coordinates (already in the rectified undistort-preview
// frame) and returns the triangulated 3-D point in the cam1 rectified frame.
app.post('/api/triangulate', express.json({ limit: '4kb' }), (req, res) => {
  const { point_cam1, point_cam2 } = req.body || {};

  const isFiniteNonNeg = v => typeof v === 'number' && isFinite(v) && v >= 0;
  if (
    !point_cam1 || !point_cam2 ||
    !isFiniteNonNeg(point_cam1.x) || !isFiniteNonNeg(point_cam1.y) ||
    !isFiniteNonNeg(point_cam2.x) || !isFiniteNonNeg(point_cam2.y)
  ) {
    return res.status(400).json({
      error: 'point_cam1 and point_cam2 must each have finite non-negative x and y numbers',
    });
  }

  if (!fs.existsSync(CALIB_JSON)) {
    return res.status(500).json({ error: 'calibration.json not found on server' });
  }

  const args = [
    TRIANGULATION_PY,
    String(point_cam1.x), String(point_cam1.y),
    String(point_cam2.x), String(point_cam2.y),
    CALIB_JSON,
  ];

  const result = spawnSync('python3', args, { timeout: 15_000, encoding: 'utf8' });

  if (result.error) {
    return res.status(500).json({ error: 'Failed to spawn python3: ' + result.error.message });
  }

  const pyOut = parseJsonFromStdout(result.stdout) || {};

  if (result.status !== 0 || !pyOut.success) {
    const errMsg = pyOut.error || result.stderr || 'triangulation.py failed';
    console.error('[triangulate]', errMsg);
    return res.status(500).json({ error: errMsg });
  }

  console.log(`[triangulate] XYZ=${JSON.stringify(pyOut.xyz_mm)}`);
  res.json(pyOut);
});

// ─── Detection/Segmentation endpoint ─────────────────────────────────────────────────────
app.post('/api/detect', upload.fields([{ name: 'cam1', maxCount: 1 }]), (req, res) => {
  const files = req.files;

  const tsStr = typeof req.body?.tsStr === 'string' ? req.body.tsStr.trim() : null;
  const forceRecompute = parseForceFlag(req.body?.forceRecompute);
  if (tsStr !== null && !/^\d+$/.test(tsStr)) {
    return res.status(400).json({ error: 'Invalid tsStr format: must be numeric' });
  }

  if (tsStr && !forceRecompute) {
    const existing = getLatestArtifact(tsStr, 'detection');
    if (existing && existing.absPath && fs.existsSync(existing.absPath)) {
      return res.json({
        success: true,
        cached: true,
        tsStr,
        process: 'detection',
        url: makeProcessedArtifactUrl(tsStr, 'detection', existing.ext),
        relPath: existing.relPath,
        detections: Number(existing.metadata?.detections || 0),
      });
    }
  }

  const requestedSource = typeof req.body?.sourceProcess === 'string'
    ? sanitizeProcess(req.body.sourceProcess)
    : null;

  let inputBuffer = null;
  let resolvedSourceProcess = null;

  if (tsStr && !files?.cam1) {
    // Chain mode: resolve prior artifact from registry
    const preferredSource = requestedSource || 'depthmap';
    const sourceArt = getLatestArtifact(tsStr, preferredSource);
    if (!sourceArt) {
      return res.status(404).json({
        error: `No prior artifact found for tsStr=${tsStr}, process=${preferredSource}. Run ${preferredSource} first or upload cam1 directly.`,
      });
    }
    try {
      inputBuffer = fs.readFileSync(sourceArt.absPath);
    } catch (e) {
      return res.status(404).json({ error: 'Prior artifact file not found on disk' });
    }
    resolvedSourceProcess = preferredSource;
  } else {
    if (!files || !files.cam1) {
      return res.status(400).json({ error: 'cam1 file is required' });
    }
    inputBuffer = files.cam1[0].buffer;
  }

  const id     = crypto.randomUUID();
  const tmpDir = os.tmpdir();
  const img1   = path.join(tmpDir, `${id}_cam1.jpg`);
  const outPng = tsStr
    ? path.join(PROCESSED_ROOT, tsStr, buildArtifactFileName(tsStr, 'detection', 'png'))
    : path.join(tmpDir, `${id}_detect.png`);

  const cleanup = () => {
    const tempFiles = [img1];
    if (!tsStr) tempFiles.push(outPng);
    for (const f of tempFiles) {
      try { fs.unlinkSync(f); } catch { /* already gone */ }
    }
  };

  if (tsStr) {
    try { fs.mkdirSync(path.dirname(outPng), { recursive: true }); }
    catch (e) {
      return res.status(500).json({ error: 'Failed to create output dir: ' + e.message });
    }
  }

  try {
    fs.writeFileSync(img1, inputBuffer);
  } catch (e) {
    cleanup();
    return res.status(500).json({ error: 'Failed to write temp file: ' + e.message });
  }

  const result = spawnSync('python3', [DETECTION_PY, img1, outPng], { timeout: 120_000, encoding: 'utf8' });

  if (result.error) {
    cleanup();
    return res.status(500).json({ error: 'Failed to spawn python3: ' + result.error.message });
  }

  const pyOut = parseJsonFromStdout(result.stdout) || {};
  console.log(pyOut);

  if (result.status !== 0 || !pyOut.success) {
    cleanup();
    if (tsStr) {
      try { fs.unlinkSync(outPng); } catch { /* ignore partial output */ }
    }
    const errMsg = pyOut.error || result.stderr || 'detection.py failed';
    console.error('[detect]', errMsg);
    return res.status(500).json({ error: errMsg });
  }

  console.log(`[detect] ${pyOut.detections} detection(s) written to ${outPng}`);

  if (tsStr) {
    const outFileName = buildArtifactFileName(tsStr, 'detection', 'png');
    const persistedAbsPath = outPng;
    const relPath = path.relative(__dirname, persistedAbsPath).split(path.sep).join('/');
    if (!fs.existsSync(persistedAbsPath)) {
      cleanup();
      return res.status(500).json({ error: 'Expected output file missing after processing' });
    }

    const sourceArtRecord = resolvedSourceProcess ? getLatestArtifact(tsStr, resolvedSourceProcess) : null;
    const record = {
      tsStr,
      process: 'detection',
      ext: 'png',
      relPath,
      absPath: persistedAbsPath,
      sourceProcess: resolvedSourceProcess,
      sourceRelPath: sourceArtRecord ? sourceArtRecord.relPath : null,
      createdAtMs: Date.now(),
      metadata: { detections: pyOut.detections || 0 },
    };
    registerProcessedArtifact(record);
    cleanup();

    return res.json({
      success: true,
      cached: false,
      tsStr,
      process: 'detection',
      url: `/api/processed/${encodeURIComponent(tsStr)}/${encodeURIComponent(outFileName)}`,
      relPath,
      detections: pyOut.detections || 0,
    });
  }

  // Legacy mode (no tsStr): return blob download
  res.download(outPng, 'detection.png', (err) => {
    cleanup();
    if (err && !res.headersSent) res.status(500).json({ error: 'Download failed' });
  });
});

// ─── Serve persisted processed files ─────────────────────────────────────────
app.get('/api/processed/:tsStr/:filename', (req, res) => {
  const tsStr    = req.params.tsStr;
  const filename = req.params.filename;
  if (!/^\d+$/.test(tsStr) || !/^[\w.\-]+$/.test(filename)) {
    return res.status(400).json({ error: 'Invalid path parameters' });
  }
  const absPath = path.join(PROCESSED_ROOT, tsStr, filename);
  if (!fs.existsSync(absPath)) {
    return res.status(404).json({ error: 'Processed file not found' });
  }
  res.sendFile(absPath);
});

// ─── Artifact lookup endpoints ─────────────────────────────────────────────────
app.get('/api/artifact', (req, res) => {
  const tsStr = String(req.query.tsStr || '');
  const proc  = sanitizeProcess(req.query.process);
  if (!tsStr || !/^\d+$/.test(tsStr)) {
    return res.status(400).json({ error: 'Invalid or missing tsStr' });
  }
  if (!proc) {
    return res.status(400).json({ error: `Invalid process. Valid values: ${[...VALID_PROCESSES].join(', ')}` });
  }
  const artifact = getLatestArtifact(tsStr, proc);
  if (!artifact) {
    return res.status(404).json({ error: `No artifact found for tsStr=${tsStr}, process=${proc}` });
  }
  res.json(artifact);
});

app.get('/api/artifacts', (req, res) => {
  const tsStr = String(req.query.tsStr || '');
  if (!tsStr || !/^\d+$/.test(tsStr)) {
    return res.status(400).json({ error: 'Invalid or missing tsStr' });
  }
  const artifacts = processedArtifacts.filter(r => r.tsStr === tsStr);
  res.json({ tsStr, artifacts });
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
  ws.pendingTofDumpFile = null;

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

      // Handle pending ToF dump payload after a tofdump:<filename> header.
      if (ws.pendingTofDumpFile) {
        const pendingFile = ws.pendingTofDumpFile;
        ws.pendingTofDumpFile = null;
        const tsStr = extractTimestampFromTofDumpName(pendingFile);
        if (!tsStr) {
          console.warn(`[tofdump] Filename must contain numeric timestamp: ${pendingFile}`);
          return;
        }

        let payload;
        try {
          payload = JSON.parse(text);
        } catch (e) {
          console.warn(`[tofdump] Invalid JSON payload for ${pendingFile}: ${e.message}`);
          return;
        }

        persistIncomingTofDump(tsStr, payload, ws.cameraId || ws.clientType || 'unknown');

        const msg = JSON.stringify({
          type: 'tofdump',
          tsStr,
          filename: `${tsStr}.json`,
          url: makeHistoryTofDumpUrl(tsStr),
          payload,
          ts: parseTimestampMs(tsStr) || Date.now(),
          source: ws.cameraId || ws.clientType || 'unknown',
        });

        for (const b of browserClients) {
          if (b.readyState === WebSocket.OPEN) b.send(msg);
        }
        return;
      }

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


      // Auxiliary relay registration
      if (text === 'tof_relay') {
        ws.clientType = 'tof_relay';
        ws.cameraId = null;
        console.log(`  Auxiliary relay registered from ${ip}`);
        return;
      }

      // Per-file ToF dump framing from ESP:
      // first frame: "tofdump:<filename>", second frame: raw JSON payload.
      if (text.startsWith('tofdump:')) {
        const filename = text.slice('tofdump:'.length).trim();
        if (!filename) {
          console.warn('[tofdump] Empty filename header, ignoring');
          ws.pendingTofDumpFile = null;
          return;
        }

        if (!extractTimestampFromTofDumpName(filename)) {
          console.warn('[tofdump] Ignoring non-timestamp filename header:', filename);
          ws.pendingTofDumpFile = null;
          return;
        }

        ws.pendingTofDumpFile = filename;
        return;
      }

      // Browser registration
      if (text === 'browser') {
        ws.clientType = 'browser';
        browserClients.add(ws);
        console.log(`  Browser joined (total ${browserClients.size})`);

        // Send status
        sendStatus(ws);
        return;
      }

      // Browser commands
      if (text === 'trigger_capture') { triggerCapture(); return; }
      if (text === 'start_auto')      { startCapture();   return; }
      if (text === 'stop_auto')       { stopCapture();    return; }
      if (text === 'request_ota')     { masterOTAPending = true; slaveOTAPending = true; broadcastStatus(); return; }

      if (ws.clientType === 'tof_relay') {
        for (const b of browserClients) {
          if (b.readyState === WebSocket.OPEN) b.send(text);
        }
        return;
      }

      return;
    }

    // ── Binary messages (image from ESP) ──
    const buf       = Buffer.from(data);
 
    const camId = buf.subarray(0, 8).toString('utf8').replace(/\0/g, '');
    const timestamp = buf.readBigUInt64LE(8);
    const dataLen = buf.readUInt32LE(16);
    const jpegData = buf.subarray(24, 24+dataLen);
    const tsStr = timestamp.toString();

    console.log(`Image ${camId}, ts=${timestamp}, dataLen =${dataLen}, size=${jpegData.length}`);

    latestImages[camId] = jpegData;
    persistIncomingFrame(camId, tsStr, jpegData);

    // Build normalized frame: "camId:" + JPEG (consistent format for browsers)
    // Build normalized frame: "camId:timestamp:" + JPEG
    const hdr = Buffer.from(`${camId}:${tsStr}:`, 'utf-8');
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
