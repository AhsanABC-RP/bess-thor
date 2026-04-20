const $ = (id) => document.getElementById(id);

let _recording = false;  // tracked from ws state to gate preview-switch calls

async function loadPresets() {
  const r = await fetch('/presets');
  const j = await r.json();
  const sel = $('preset');
  const prev = sel.value;
  sel.innerHTML = '';
  // Prefer selecting a live preset on fresh load so the UX doesn't default to
  // a dark camera (e.g. the absent A6701).
  let firstLive = null;
  for (const [k, v] of Object.entries(j)) {
    const opt = document.createElement('option');
    opt.value = k;
    const badge = v.live ? '✓' : '✗ (no publisher)';
    opt.textContent = `${k}  ${badge}  (${v.image_topic})`;
    opt.disabled = false;  // keep selectable so user can switch back once hardware comes back
    if (!v.live) opt.style.opacity = '0.5';
    sel.appendChild(opt);
    if (v.live && firstLive === null) firstLive = k;
  }
  // Restore prior selection if still present, else jump to first live
  if (prev && j[prev]) sel.value = prev;
  else if (firstLive) sel.value = firstLive;
}

// Re-poll live-status every 4s so presets that come back online re-enable.
setInterval(() => { loadPresets().catch(() => {}); }, 4000);

function isoDate() {
  const d = new Date();
  const pad = (n) => String(n).padStart(2, '0');
  return `${d.getFullYear()}-${pad(d.getMonth()+1)}-${pad(d.getDate())}`;
}

// Auto-filled pattern we own: <preset>_intrinsics_YYYY-MM-DD
// Replace the session name iff it matches our pattern OR is empty; leave custom names alone.
const AUTO_SESSION_RE = /^[a-z0-9_]+_intrinsics_\d{4}-\d{2}-\d{2}$/;

function autoSession() {
  const preset = $('preset').value;
  const existing = $('session').value.trim();
  if (!existing || AUTO_SESSION_RE.test(existing)) {
    $('session').value = `${preset}_intrinsics_${isoDate()}`;
  }
}

async function previewCurrent() {
  const preset = $('preset').value;
  try {
    await fetch('/preview', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ preset }),
    });
  } catch (e) { console.error('preview failed', e); }
}

$('preset').addEventListener('change', async () => {
  autoSession();
  // Don't flip the subscription mid-recording — would lose frames + corrupt the bag
  if (!_recording) await previewCurrent();
});

$('target').addEventListener('change', async () => {
  const preset = $('preset').value;
  const target = $('target').value;
  try {
    const r = await fetch('/target', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ preset, target }),
    });
    const j = await r.json();
    if (!j.ok) alert(`Target set failed: ${j.err || 'unknown'}`);
  } catch (e) { console.error('target set failed', e); }
});

$('btn-start').addEventListener('click', async () => {
  const preset = $('preset').value;
  const sessionName = $('session').value.trim();
  if (!sessionName) { alert('set a session name'); return; }
  const r = await fetch('/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ preset, session: sessionName }),
  });
  console.log(await r.json());
});

$('btn-stop').addEventListener('click', async () => {
  const r = await fetch('/stop', { method: 'POST' });
  console.log(await r.json());
});

$('btn-calib').addEventListener('click', async () => {
  const r = await fetch('/calibrate', { method: 'POST' });
  console.log(await r.json());
});

async function fetchRectified() {
  const errEl = $('rect-err');
  const imgEl = $('rect-img');
  errEl.textContent = '';
  imgEl.removeAttribute('src');
  // Cache-bust so repeat-clicks show a fresh frame even though the URL is static
  const url = `/rectified.jpg?ts=${Date.now()}`;
  try {
    const r = await fetch(url);
    if (!r.ok) {
      let msg = `HTTP ${r.status}`;
      try { const j = await r.json(); if (j.err) msg = `${msg}: ${j.err}`; } catch {}
      errEl.textContent = msg;
      return;
    }
    const blob = await r.blob();
    imgEl.src = URL.createObjectURL(blob);
  } catch (e) {
    errEl.textContent = String(e);
  }
}

$('btn-rect').addEventListener('click', () => {
  $('rect-modal').classList.remove('hidden');
  fetchRectified();
});
$('btn-rect-refresh').addEventListener('click', fetchRectified);
$('btn-rect-close').addEventListener('click', () => {
  $('rect-modal').classList.add('hidden');
});
// Click outside the inner panel also closes
$('rect-modal').addEventListener('click', (e) => {
  if (e.target.id === 'rect-modal') $('rect-modal').classList.add('hidden');
});

$('btn-drawer').addEventListener('click', () => {
  $('drawer').classList.toggle('hidden');
  if (window._lastGrid) drawGridSidebar(window._lastGrid);
});

$('btn-rotate').addEventListener('click', async () => {
  // Cycle 0→90→180→270→0; server-side persists per preset
  const r = await fetch('/rotate', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({}),
  });
  if (!r.ok) console.error('rotate failed', await r.text());
});

async function applyExposure(expUs, gainDb) {
  const preset = $('preset').value;
  const r = await fetch('/exposure', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ preset, exposure_us: expUs, gain_db: gainDb }),
  });
  const j = await r.json();
  console.log('exposure', j);
  if (!j.ok) alert(`Set failed: ${j.err || 'unknown'}`);
}

$('btn-apply-exp').addEventListener('click', () => {
  const exp = parseFloat($('inp-exp').value);
  const gain = parseFloat($('inp-gain').value);
  applyExposure(exp, gain);
});

$('btn-exp-auto').addEventListener('click', () => {
  $('inp-exp').value = -1;
  $('inp-gain').value = -1;
  applyExposure(-1, -1);
});

// Render coverage grid into the sidebar canvas. Aspect follows grid shape (cols/rows)
// rather than the video, so it stays legible regardless of per-camera grid size.
function drawGridSidebar(grid) {
  const canvas = $('grid-sidebar');
  if (!canvas) return;
  if (!Array.isArray(grid) || !grid.length || !Array.isArray(grid[0])) return;

  const rows = grid.length, cols = grid[0].length;
  const dims = $('grid-dims');
  if (dims) dims.textContent = `${cols}×${rows}`;

  // Size the backing store to the CSS box with DPR for crisp lines.
  const cssW = canvas.clientWidth || 340;
  const cssH = Math.round(cssW * rows / cols);
  canvas.style.height = cssH + 'px';
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(cssW * dpr);
  canvas.height = Math.round(cssH * dpr);

  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cssW, cssH);

  const cw = cssW / cols, ch = cssH / rows;
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const hit = !!grid[r][c];
      ctx.fillStyle = hit ? 'rgba(26,139,58,0.55)' : 'rgba(200,50,50,0.35)';
      ctx.fillRect(c*cw, r*ch, cw, ch);
      ctx.strokeStyle = 'rgba(20,30,60,0.9)';
      ctx.lineWidth = 1;
      ctx.strokeRect(c*cw + 0.5, r*ch + 0.5, cw - 1, ch - 1);
    }
  }
}

window.addEventListener('resize', () => { if (window._lastGrid) drawGridSidebar(window._lastGrid); });

function ws() {
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  const s = new WebSocket(`${scheme}://${location.host}/ws`);
  // On (re)connect, re-arm the server-side ROS subscription. Server process
  // restarts drop the subscription; WS will reconnect via the onclose retry
  // below but the preview stays dead without an explicit /preview POST,
  // leaving the UI showing "nothing detected" even when the board is in view.
  s.onopen = () => { if (!_recording) previewCurrent().catch(() => {}); };
  s.onmessage = (e) => {
    const st = JSON.parse(e.data);
    _recording = !!st.recording;
    if (st.grid) { window._lastGrid = st.grid; drawGridSidebar(st.grid); }

    $('p-grid').value = Math.round(st.grid_pct * 100);
    $('t-grid').textContent = `${Math.round(st.grid_pct*100)}%`;
    $('p-dist').value = st.dist_hit;  $('t-dist').textContent = `${st.dist_hit}/${st.min_dist}`;
    $('p-tilt').value = st.tilt_hit;  $('t-tilt').textContent = `${st.tilt_hit}/${st.min_tilt}`;
    $('p-roll').value = st.roll_hit;  $('t-roll').textContent = `${st.roll_hit}/${st.min_roll}`;
    $('p-frames').max = st.target_frames;
    $('p-frames').value = Math.min(st.frame_detected, st.target_frames);
    $('t-frames').textContent = `${st.frame_detected}`;

    const pill = $('status');
    if (st.recording) { pill.className = 'pill rec'; pill.textContent = `REC · ${st.session}`; }
    else if (st.ready_to_finish) { pill.className = 'pill ready'; pill.textContent = 'ready'; }
    else { pill.className = 'pill'; pill.textContent = st.preset || 'idle'; }

    // Target dropdown — populate once, sync value each tick
    const tsel = $('target');
    if (Array.isArray(st.targets_available) && tsel.options.length !== st.targets_available.length) {
      tsel.innerHTML = '';
      for (const t of st.targets_available) {
        const opt = document.createElement('option');
        opt.value = t; opt.textContent = t;
        tsel.appendChild(opt);
      }
    }
    if (st.target && tsel.value !== st.target) tsel.value = st.target;

    $('btn-start').disabled = st.recording;
    $('btn-stop').disabled = !st.recording;
    $('btn-calib').disabled = st.recording || !st.session;
    // Rectified view only makes sense once a calibration yaml exists for the session
    $('btn-rect').disabled = !st.kalibr_result;
    $('t-rot').textContent = `${st.rotation || 0}°`;

    const info = [
      `preset:   ${st.preset || '-'}`,
      `topic:    ${st.image_topic || '-'}`,
      `resolution: ${st.image_width}x${st.image_height}`,
      `frames total/detected: ${st.frame_total} / ${st.frame_detected}`,
      `last tag count: ${st.last_tag_count}`,
      `dist bins: ${JSON.stringify(st.dist_bins)}`,
      `tilt bins: ${JSON.stringify(st.tilt_bins)}`,
      `roll bins: ${JSON.stringify(st.roll_bins)}`,
      `ready: ${st.ready_to_finish}`,
    ].join('\n');
    $('status-txt').textContent = info;

    if (st.kalibr_log_tail && st.kalibr_log_tail.length) {
      $('kalibr-log').textContent = st.kalibr_log_tail.join('\n');
    } else if (st.kalibr_result) {
      $('kalibr-log').textContent = st.kalibr_result;
    }
  };
  s.onclose = () => setTimeout(ws, 1000);
}

loadPresets().then(async () => {
  autoSession();
  // Kick off MJPEG stream subscription to the first camera in the list so the
  // video pane is populated on first load without the user clicking anything.
  await previewCurrent();
  ws();
});
