import { SerialTransport, BleTransport, ReplayTransport } from './transports.js';
import { SosCoordinator, dispatch, resolveLocation, mapsLink } from './emergency.js';

const $ = (id) => document.getElementById(id);
const OFFLINE_MS = 3000;

// ---------------------------------------------------------------- state ----
const S = {
  transport: null, replay: false, device: null, cfg: null, tel: null, telAt: 0, lastRx: 0,
  state: null, stateMsg: null, det: null, detAt: 0, phoneFix: null, sos: null, online: false,
};

const STATE_INFO = {
  BOOT: ['offline', 'Starting up…'],
  CALIBRATING: ['warn', 'Calibrating sensor — keep the bike still.'],
  SAFE: ['safe', 'Normal operation. Monitoring motion continuously.'],
  MONITORING: ['safe', 'Analysing an unusual motion event (no alarm).'],
  POSSIBLE_CRASH: ['warn', 'Suspicious motion detected — still evaluating.'],
  CRASH_DETECTED: ['alarm', 'Possible accident detected — emergency countdown running.'],
  USER_CANCELLED: ['safe', 'Rider cancelled the alarm. No SOS sent.'],
  SOS_SENT: ['alarm', 'Countdown expired — SOS issued.'],
  RECOVERY: ['warn', 'After an incident. Explicit reset required before normal operation.'],
  MANUAL_TEST: ['warn', 'TEST / MANUAL TRIGGER — simulated alarm, not a real crash.'],
  IMU_FAULT: ['alarm', 'IMU DISCONNECTED — crash detection is NOT active.'],
};
const LABEL = { POSSIBLE_CRASH: 'POSSIBLE CRASH', CRASH_DETECTED: 'CRASH DETECTED', SOS_SENT: 'SOS SENT', USER_CANCELLED: 'CANCELLED', MANUAL_TEST: 'TEST / MANUAL TRIGGER', IMU_FAULT: 'IMU DISCONNECTED' };

// ------------------------------------------------------------- settings ----
const SKEY = 'bsb.settings';
function loadSettings() {
  try { return { rider: '', notes: '', contacts: [], ntfy: '', webhook: '', phoneLoc: false, ...JSON.parse(localStorage.getItem(SKEY) || '{}') }; }
  catch { return { rider: '', notes: '', contacts: [], ntfy: '', webhook: '', phoneLoc: false }; }
}
let settings = loadSettings();
function saveSettings() { try { localStorage.setItem(SKEY, JSON.stringify(settings)); } catch {} }

// ------------------------------------------------------------ incidents ----
const IKEY = 'bsb.incidents';
let incidents = (() => { try { return JSON.parse(localStorage.getItem(IKEY) || '{}'); } catch { return {}; } })();
function saveIncidents() { try { localStorage.setItem(IKEY, JSON.stringify(incidents)); } catch {} }

function upsertIncident(m) {
  const key = `${S.device?.dev || 'dev'}#${m.id}`;
  const prev = incidents[key] || {};
  const rec = { ...prev, ...m, device: S.device?.dev, key, replay: S.replay || prev.replay };
  if (!rec.wall && m.cur && S.tel) rec.wall = Date.now() - Math.max(0, S.tel.ms - m.up);   // uptime -> wall clock
  if (!rec.wall && m.cur && !S.tel) rec.wall = Date.now();
  if (S.sos?.id === m.id && S.sos.loc?.ok && !m.gpsOk) rec.phoneLoc = S.sos.loc;
  incidents[key] = rec;
  saveIncidents();
  renderIncidents();
}

// ------------------------------------------------------------- console -----
const consoleLines = [];
function logConsole(line, dir = '') {
  if (!$('showJson').checked && line.startsWith('{') && dir !== '>') return;
  consoleLines.push((dir ? dir + ' ' : '') + line);
  if (consoleLines.length > 600) consoleLines.splice(0, consoleLines.length - 600);
  const el = $('console');
  const atBottom = el.scrollTop + el.clientHeight >= el.scrollHeight - 20;
  el.textContent = consoleLines.join('\n');
  if (atBottom) el.scrollTop = el.scrollHeight;
}

// ----------------------------------------------------------- transport -----
async function send(cmd) {
  if (!S.transport || S.replay) return false;
  logConsole(cmd, '>');
  return S.transport.send(cmd);
}

let pingTimer = null;
async function connect(kind) {
  await disconnect();
  const onLine = handleLine;
  const onClose = (why) => { logConsole(`-- ${why}`); S.transport = null; stopPing(); render(); };
  try {
    if (kind === 'USB') {
      if (!SerialTransport.supported()) return alert('Web Serial is not supported in this browser. Use Chrome or Edge on a computer, or Bluetooth on Android.');
      S.transport = new SerialTransport(onLine, onClose);
    } else if (kind === 'BLE') {
      if (!BleTransport.supported()) return alert('Web Bluetooth is not supported in this browser. Use Chrome on Android or desktop.');
      S.transport = new BleTransport(onLine, onClose);
    } else {
      S.transport = new ReplayTransport(onLine, onClose, kind);
    }
    S.replay = S.transport.kind === 'REPLAY';
    await S.transport.connect();
    logConsole(`-- connected (${S.transport.kind})`);
    if (!S.replay) {
      await send('HELLO');
      pingTimer = setInterval(() => S.transport && S.transport.send('PING'), 1000);
    }
  } catch (e) {
    logConsole(`-- connection failed: ${e.message}`);
    S.transport = null;
  }
  render();
}
function stopPing() { clearInterval(pingTimer); pingTimer = null; }
async function disconnect() {
  stopPing();
  if (S.transport) { const t = S.transport; S.transport = null; await t.disconnect(); }
  S.replay = false; S.online = false;
  render();
}

// -------------------------------------------------------------- protocol ---
function handleLine(line) {
  S.lastRx = Date.now();
  if (!line.startsWith('{')) { logConsole(line); return; }
  let m;
  try { m = JSON.parse(line); } catch { logConsole(line); return; }
  logConsole(line);
  switch (m.t) {
    case 'hello': S.device = m; break;
    case 'cfg': S.cfg = m; renderProfile(); break;
    case 'tel': S.tel = m; S.telAt = Date.now(); if (m.st !== S.state) S.state = m.st; break;
    case 'state': S.state = m.st; S.stateMsg = m; break;
    case 'det': if (m.ev !== 'candidate' || !S.det || Date.now() - S.detAt > 4000) { S.det = m; S.detAt = Date.now(); renderDet(); } break;
    case 'inc': upsertIncident(m); break;
    case 'sos': sos.handle(m, { phoneFix: S.phoneFix, replay: S.replay, device: S.device?.dev }); break;
    default: break;
  }
  scheduleRender();
}

const sos = new SosCoordinator(() => settings, (c) => send(c), (st) => { S.sos = st; renderOverlay(); });

// --------------------------------------------------------------- render ----
let renderQueued = false;
function scheduleRender() { if (!renderQueued) { renderQueued = true; requestAnimationFrame(() => { renderQueued = false; render(); }); } }

const fmt = (v, d = 1, unit = '') => (v === null || v === undefined || Number.isNaN(v) ? '—' : `${Number(v).toFixed(d)}${unit}`);

function leanWords(roll) {
  if (roll === null || roll === undefined) return 'No data';
  const a = Math.abs(roll);
  const side = roll > 0 ? 'right' : 'left';
  if (a < 3) return 'Upright';
  const n = S.cfg?.leanN ?? 45, c = S.cfg?.leanC ?? 65;
  if (a < n) return `Leaning ${side} — within normal riding range (≤ ${n}°)`;
  if (a < c) return `Leaning ${side} — beyond normal range for this profile`;
  return `Leaning ${side} — abnormal (≥ ${c}°), bike may be down`;
}

function render() {
  const now = Date.now();
  S.online = !!S.transport && now - S.lastRx < OFFLINE_MS;
  const pill = $('linkPill');
  if (S.replay) { pill.className = 'pill pill-replay'; pill.textContent = 'REPLAY'; }
  else if (S.online) { pill.className = 'pill pill-online'; pill.textContent = `ONLINE · ${S.transport.kind}`; }
  else { pill.className = 'pill pill-offline'; pill.textContent = 'DEVICE OFFLINE'; }
  $('btnDisconnect').classList.toggle('hidden', !S.transport);
  $('deviceInfo').textContent = S.device ? `${S.device.dev} · fw ${S.device.fw} · ${S.device.board} · boot #${S.device.boot}` : 'No device';
  $('simBanner').classList.toggle('hidden', !(S.replay || (S.tel && S.tel.sim)));

  // Status card
  const card = $('statusCard');
  let tone, title, desc;
  if (!S.online && !S.replay) {
    tone = 'offline'; title = 'DEVICE OFFLINE';
    desc = S.transport ? `No data from the device for ${Math.round((now - S.lastRx) / 1000)} s — communication failure. Detection may still run on the device, but alerts cannot reach this app.` : 'Connect the Bike Safety Belt over USB or Bluetooth.';
  } else {
    const st = S.state || 'BOOT';
    [tone, desc] = STATE_INFO[st] || ['offline', st];
    title = LABEL[st] || st;
    if (st === 'SAFE' && S.tel?.ab) desc = 'Bike is not upright (static lean). Not an alarm by itself.';
    if (st === 'SAFE' && S.tel && !S.tel.arm) desc += ' Detection re-arms once the bike is upright.';
  }
  card.dataset.tone = tone;
  $('statusValue').textContent = title;
  $('statusDesc').textContent = desc;
  const meta = [];
  if (S.tel?.sim) meta.push(`Simulation ${S.tel.sim} running`);
  if (S.cfg) meta.push(`Profile: ${S.cfg.profile}`);
  $('statusMeta').textContent = meta.join(' · ');
  $('btnCancelPossible').classList.toggle('hidden', !(S.online && S.state === 'POSSIBLE_CRASH'));

  // Live values
  const t = S.tel;
  const stale = !t || now - S.telAt > 2000 || !t.imu;
  document.querySelectorAll('#view-live .panel').forEach((p, i) => i < 4 && p.classList.toggle('stale', stale));
  const roll = t?.r ?? null;
  $('leanVal').textContent = roll === null ? '—' : `${Math.abs(roll).toFixed(1)}° ${roll > 0.5 ? 'R' : roll < -0.5 ? 'L' : ''}`;
  $('leanText').textContent = t && !t.imu ? 'IMU DISCONNECTED' : leanWords(roll);
  $('gaugeNeedle').style.transform = `rotate(${Math.max(-90, Math.min(90, roll || 0))}deg)`;
  $('rollVal').textContent = fmt(t?.r, 1, '°');
  $('pitchVal').textContent = fmt(t?.p, 1, '°');
  $('tiltVal').textContent = fmt(t?.tl, 1, '°');
  $('refVal').textContent = t ? ({ 1: 'Accelerometer + gyro', 2: 'Cornering kinematics', 3: 'Gyro (dynamic motion)' }[t.ref] || '—') : '—';
  $('accVal').textContent = fmt(t?.a, 2, ' g');
  $('accPeak').textContent = fmt(t?.ap, 2, ' g');
  $('gyrVal').textContent = fmt(t?.g, 0, ' °/s');
  $('gyrPeak').textContent = fmt(t?.gp, 0, ' °/s');
  const ap = t?.ap ?? 0, gp = t?.gp ?? 0;
  $('motionText').textContent = !t ? '' : ap > (S.cfg?.accC ?? 4.5) ? 'Severe acceleration spike' : ap > (S.cfg?.accN ?? 2.2) ? 'Acceleration spike (bump/pothole range)' : gp > (S.cfg?.gyrN ?? 120) ? 'Fast rotation' : 'Normal motion';
  const conf = t?.c ?? 0;
  $('confVal').textContent = t ? `${conf}%` : '—';
  const fill = $('confFill');
  fill.style.width = `${conf}%`;
  fill.style.background = conf >= (S.cfg?.crash ?? 65) ? 'var(--red)' : conf >= (S.cfg?.possible ?? 30) ? 'var(--amber)' : 'var(--green)';
  $('markPossible').style.left = `${S.cfg?.possible ?? 30}%`;
  $('markCrash').style.left = `${S.cfg?.crash ?? 65}%`;

  renderHealth();
  renderOverlay();
}

function renderGauge() {
  const n = 45, c = 90;
  const arc = (a0, a1) => {
    const p = (a) => { const r = (a - 90) * Math.PI / 180; return [100 + 85 * Math.cos(r), 105 + 85 * Math.sin(r)]; };
    const [x0, y0] = p(a0), [x1, y1] = p(a1);
    return `M ${x0} ${y0} A 85 85 0 0 1 ${x1} ${y1}`;
  };
  const leanN = S.cfg?.leanN ?? n;
  $('gaugeArcNorm').setAttribute('d', arc(-leanN, leanN));
  $('gaugeArcAbn').setAttribute('d', arc(-c, c));
}

function renderHealth() {
  const t = S.tel, cfg = S.cfg, dev = S.device;
  const items = [];
  const add = (lvl, name, text) => items.push(`<li><i class="dot ${lvl}"></i><div><b>${name}</b><span>${text}</span></div></li>`);
  if (!S.online && !S.replay) add('bad', 'App link', S.transport ? 'DEVICE OFFLINE — no data' : 'Not connected');
  else add('ok', 'App link', S.replay ? 'Replaying recorded data' : `Connected via ${S.transport.kind}`);
  if (!t) add('', 'Sensor (MPU-6050)', 'Unknown');
  else if (!t.imu) add('bad', 'Sensor (MPU-6050)', 'IMU DISCONNECTED — detection inactive');
  else add('ok', 'Sensor (MPU-6050)', t.sim ? `Simulated input (test ${t.sim})` : 'Connected, data valid');
  if (t) add(['warn', 'ok', 'warn', 'bad'][t.cal] || '', 'Calibration', ['In progress — keep still', 'Gyro & accel offsets OK', 'Degraded — using saved offsets', 'Not calibrated'][t.cal] || '?');
  if (t) add(t.lvl ? 'ok' : 'warn', 'Riding orientation', t.lvl ? 'Calibrated' : 'Not calibrated (Settings → Calibration)');
  if (t) add(t.arm ? 'ok' : 'warn', 'Crash detection', t.arm ? 'Armed' : 'Re-arms when the bike is upright');
  // Location
  if (t?.gps === 2) add('ok', 'Location', `Device GPS ${t.lat.toFixed(5)}, ${t.lon.toFixed(5)} (±${t.hacc} m est., ${t.sat} sat)`);
  else if (settings.phoneLoc && S.phoneFix) add('warn', 'Location', `Device: ${t?.gps === 1 ? 'no GPS fix' : 'no GPS module'}. Phone location ±${Math.round(S.phoneFix.acc)} m`);
  else add('bad', 'Location', `LOCATION UNAVAILABLE (${t?.gps === 1 ? 'no valid GPS fix' : 'no GPS module installed'})`);
  // Battery
  if (dev && !dev.batHw) add('', 'Device battery', 'Not available (no battery sensing hardware)');
  else if (t && t.bat > 0) add('ok', 'Device battery', `${(t.bat / 1000).toFixed(2)} V`);
  // Alerts
  const ch = [settings.ntfy && 'ntfy', settings.webhook && 'webhook'].filter(Boolean);
  add(ch.length ? 'ok' : 'warn', 'Emergency delivery', ch.length ? `Automatic: ${ch.join(' + ')}; ${settings.contacts.length} contact(s)` : 'No automatic channel configured (Settings)');
  $('healthList').innerHTML = items.join('');
}

const IND = [
  ['Abnormal lean', 'pkT', '°', 'leanN', 'leanC'],
  ['Angular velocity', 'pkG', '°/s', 'gyrN', 'gyrC'],
  ['Impact / acceleration', 'pkA', ' g', 'accN', 'accC'],
  ['Sudden orientation change', 'dT', '° / 0.5 s', 'tcN', 'tcC'],
  ['Post-impact orientation', 'rest', '°', 'restN', 'restC'],
];

function explain(d) {
  if (!d) return '<span class="muted">No motion event evaluated yet.</span>';
  const c = S.cfg || {};
  const v = d.verdict;
  const cls = v === 'CRASH' ? 'crash' : d.ev === 'possible' ? 'poss' : 'ok';
  const head = v === 'CRASH' ? `CRASH LIKELY — confidence ${d.conf}% (${d.act} of 5 indicators active)`
    : v === 'DISMISSED' ? `Not a crash — confidence ${d.conf}% (${d.act} of 5 indicators active)`
    : `Evaluating… confidence ${d.conf}%`;
  const rows = IND.map(([name, key, unit, nk, ck], i) => {
    const s = d.s[i];
    const lvl = s >= 60 ? 2 : s >= 25 ? 1 : 0;
    let val = `${fmt(d[key], key === 'pkA' ? 2 : 0)}${unit}`;
    if (i === 4 && !d.post) val = 'not counted';
    return `<div class="ind"><div class="ind-name">${name}<small>${val} · normal ≤ ${c[nk] ?? '?'}, crash ≥ ${c[ck] ?? '?'}</small></div>
      <div class="ind-bar"><div class="lvl-${lvl}" style="width:${s}%"></div></div><div class="ind-pct">${s}%</div></div>`;
  }).join('');
  const reasons = [];
  if (d.s[0] >= 50) reasons.push(`Lean of ${fmt(d.pkT, 0)}° is beyond the normal cornering range (${c.leanN}°).`);
  if (d.s[1] >= 50) reasons.push(`Rotation peaked at ${fmt(d.pkG, 0)}°/s — faster than normal riding.`);
  if (d.s[2] >= 50) reasons.push(`Impact of ${fmt(d.pkA, 1)} g detected.`);
  if (d.vert) reasons.push('Impact was mostly vertical — typical of a speed breaker or pothole, so it was down-weighted.');
  if (d.s[3] >= 50) reasons.push(`Orientation changed ${fmt(d.dT, 0)}° within 0.5 s (or rotated fast for ${d.sus} ms).`);
  if (d.post && d.s[4] >= 50) reasons.push(`After the event the bike ${d.set ? 'came to rest' : 'stayed'} at ${fmt(d.rest, 0)}° — lying on its side.`);
  if (!d.post && v) reasons.push('No major motion event, so the resting orientation was not counted.');
  if (d.park) reasons.push('The bike was parked and completely still before the event — confidence reduced (no SOS for a parked bike being knocked over).');
  if (v === 'DISMISSED' && d.act < (c.minInd ?? 3)) reasons.push(`Fewer than ${c.minInd ?? 3} indicators were abnormal — a single abnormal reading never triggers an alarm.`);
  return `<p class="verdict ${cls}">${head}${d.sim ? ' · SIMULATION' : ''}</p>${rows}<ul class="reasons">${reasons.map((r) => `<li>${r}</li>`).join('')}</ul>`;
}
function renderDet() {
  const html = explain(S.det);
  $('detPanel').innerHTML = html;
  $('testExplain').innerHTML = html;
  $('detWhen').textContent = S.det ? `· ${new Date(S.detAt).toLocaleTimeString()}` : '';
}

// ------------------------------------------------------------- overlay -----
let lastOverlayKey = '';
function renderOverlay() {
  const ov = $('overlay');
  const st = S.online || S.replay ? S.state : null;
  const t = S.tel;
  let key = st;
  if (st === 'CRASH_DETECTED') key += t?.cd;
  if (st === 'SOS_SENT' || st === 'RECOVERY') key += JSON.stringify(S.sos?.results || []) + (S.sos?.dispatching ? 'd' : '');
  if (key === lastOverlayKey) return;
  lastOverlayKey = key;

  const show = ['CRASH_DETECTED', 'SOS_SENT', 'RECOVERY', 'MANUAL_TEST'].includes(st);
  ov.classList.toggle('hidden', !show);
  if (!show) return;
  const sim = S.replay || (t && t.sim) || S.sos?.simulated;
  const set = (k, q, title, count, body, actions, tone) => {
    $('ovKicker').textContent = k; $('ovQuestion').textContent = q; $('ovTitle').textContent = title;
    $('ovCount').textContent = count; $('ovBody').innerHTML = body; ov.dataset.tone = tone;
    $('ovActions').innerHTML = '';
    actions.forEach(([label, cls, fn]) => { const b = document.createElement('button'); b.className = `btn ${cls}`; b.textContent = label; b.onclick = fn; $('ovActions').appendChild(b); });
  };
  if (st === 'MANUAL_TEST') {
    set('TEST / MANUAL TRIGGER', 'This is NOT a real crash event.', 'Test alarm active', '', 'Triggered by the physical push button. No SOS will be sent.<br>Press the button again or clear it here.', [['CLEAR TEST', 'btn-warn', () => send('CANCEL')]], 'test');
  } else if (st === 'CRASH_DETECTED') {
    set(sim ? 'SIMULATION · POSSIBLE ACCIDENT DETECTED' : 'POSSIBLE ACCIDENT DETECTED', 'Are you okay?', 'POSSIBLE ACCIDENT DETECTED', t?.cd ?? '', `If you do not respond, an SOS will be sent when the countdown reaches zero.${sim ? '<br><b>Simulation:</b> contacts will not be notified.' : ''}`,
      [["I'M OKAY — CANCEL SOS", 'btn-ok', () => send('CANCEL')]], 'alarm');
  } else {
    const s = S.sos;
    const loc = s?.loc;
    const parts = [];
    parts.push(loc?.ok ? `<b>Location:</b> <a href="${mapsLink(loc)}" target="_blank" rel="noopener">${loc.lat.toFixed(6)}, ${loc.lon.toFixed(6)}</a> (${loc.source}${loc.acc ? `, ±${Math.round(loc.acc)} m` : ''})` : '<b>LOCATION UNAVAILABLE</b> — no GPS fix; no coordinates were sent.');
    if (!s) parts.push('Waiting for the SOS message from the device…');
    else if (s.simulated) parts.push('<b>SIMULATION / REPLAY</b> — no alert was sent to emergency contacts.');
    else if (s.duplicate) parts.push('This SOS was already handled earlier (duplicate suppressed).');
    else if (s.dispatching) parts.push('Sending alert…');
    else if (!s.results.length) parts.push('<b>NO AUTOMATIC DELIVERY CONFIGURED.</b> Use the buttons below to notify contacts manually.');
    if (s?.results?.length) parts.push('<ul>' + s.results.map((r) => `<li>${r.channel}: ${r.ok ? 'delivered' : 'FAILED'} (${r.detail})</li>`).join('') + '</ul>');
    if (s && !s.simulated && s.results.some((r) => !r.ok)) parts.push('Delivery failed on at least one channel — notify contacts manually.');
    const actions = [];
    if (s && !s.simulated) {
      const phones = settings.contacts.map((c) => c.phone).join(',');
      if (phones) actions.push(['Send SMS to contacts', 'btn-danger', () => { location.href = `sms:${phones}?body=${encodeURIComponent(s.text)}`; }]);
      if (navigator.share) actions.push(['Share alert…', 'small', () => navigator.share({ title: 'Emergency', text: s.text }).catch(() => {})]);
    }
    actions.push(['Reset after incident (rider & bike checked)', 'small', () => { if (confirm('Reset the system to normal operation?')) send('RESET'); }]);
    actions.push(['Hide', 'small btn-ghost', () => ov.classList.add('hidden')]);
    set(sim ? 'SIMULATION' : 'EMERGENCY', st === 'RECOVERY' ? 'Recovery — explicit reset required' : 'The rider did not cancel the alarm.', 'SOS ACTIVATED', '', parts.join('<br>'), actions, 'sos');
  }
}

// ------------------------------------------------------------ incidents ----
function incidentTime(r) {
  if (r.wall) return new Date(r.wall).toLocaleString();
  const s = Math.floor(r.up / 1000);
  return `Boot #${r.boot} +${Math.floor(s / 3600)}h${String(Math.floor(s / 60) % 60).padStart(2, '0')}m${String(s % 60).padStart(2, '0')}s`;
}
function renderIncidents() {
  const list = Object.values(incidents).sort((a, b) => (b.boot - a.boot) || (b.id - a.id));
  if (!list.length) { $('incBody').innerHTML = '<tr><td colspan="10" class="muted">No incidents recorded.</td></tr>'; return; }
  const tag = (txt, c) => `<span class="tag ${c}">${txt}</span>`;
  $('incBody').innerHTML = list.map((r) => {
    const type = r.type === 'TEST' ? tag('MANUAL TEST', 'amber') : r.type === 'SIM' || r.replay ? tag('SIMULATION', 'grey') : tag((r.flags & 2) ? 'AUTO · CRASH DETECTED' : 'AUTO · SUSPECTED', (r.flags & 2) ? 'red' : 'amber');
    const out = { CANCELLED: tag('Rider cancelled', 'green'), DISMISSED: tag('Cleared by detector', 'green'), SOS: tag('SOS sent', 'red'), RESOLVED: tag('SOS sent · reset', 'red'), TEST_CLEARED: tag('Test cleared', 'grey'), OPEN: tag('Open', 'amber') }[r.out] || r.out;
    const loc = r.gpsOk ? `<a href="https://maps.google.com/?q=${r.lat},${r.lon}" target="_blank" rel="noopener">GPS</a>` : r.phoneLoc ? `<a href="${mapsLink(r.phoneLoc)}" target="_blank" rel="noopener">Phone</a>` : '<span class="muted">Unavailable</span>';
    const sosTxt = (r.flags & 8) ? ((r.flags & 16) ? 'Sent · app received' : 'Sent · not confirmed') : '—';
    return `<tr><td>${incidentTime(r)}</td><td>#${r.id}</td><td>${type}</td><td>${out}</td><td>${r.conf}%</td><td>${fmt(r.acc, 2, ' g')}</td><td>${r.gyr} °/s</td><td>${fmt(r.lean, 0, '°')}</td><td>${loc}</td><td>${sosTxt}</td></tr>`;
  }).join('');
}
function download(name, text, type) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type }));
  a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

// ------------------------------------------------------------ test mode ----
const TESTS = [
  [1, 'Normal stationary state', 'Bike upright, engine idling.', 'SAFE — no event.'],
  [2, 'Normal lean', 'Side stand and walking the bike at 12–25°.', 'SAFE — lean alone never alarms.'],
  [3, 'Simulated cornering', 'Coordinated turns to 40–45° + chicane.', 'SAFE — no crash.'],
  [4, 'Sudden braking', '1 g braking with ABS judder, then hard acceleration.', 'SAFE — no event.'],
  [5, 'Speed breaker', '3 g vertical spike, front then rear wheel.', 'MONITORING briefly, cleared (vertical bump pattern).'],
  [6, 'Sudden rotation', 'Emergency swerve, ~170°/s.', 'MONITORING briefly, cleared.'],
  [7, 'Crash-like combined motion', 'Low-side: lean + rotation + impact + slide + rests on its side.', 'POSSIBLE CRASH → CRASH DETECTED → countdown.'],
  ['BTN', 'Manual button trigger', 'Same as pressing the physical button (TEST / MANUAL TRIGGER).', 'MANUAL TEST, never SOS. Press again to clear.'],
  [9, 'Sensor disconnect', 'IMU reads fail for 6 s.', 'IMU DISCONNECTED, then recovers to SAFE.'],
  ['GPS', 'GPS unavailable', 'Forces "no fix" for 30 s (or reports that no GPS module exists).', 'LOCATION UNAVAILABLE — no fake coordinates.'],
  ['COMMS', 'Communication failure', 'Device stops sending data for 8 s.', 'App shows DEVICE OFFLINE, then reconnects.'],
  ['CANCEL', 'Countdown cancellation', 'Runs the crash scenario; tap I\'M OKAY during the countdown.', 'USER CANCELLED → SAFE, no SOS.'],
  ['EXPIRE', 'Countdown expiry', 'Runs the crash scenario; let the countdown run out.', 'SOS SENT (simulation: contacts not notified) → RECOVERY.'],
  [14, 'Pothole (extra)', 'Two sharp 4 g vertical impacts.', 'Cleared — bump pattern.'],
  [15, 'Parked bike knocked over (extra)', '35 s parked & still, then tips over.', 'POSSIBLE CRASH only — no countdown for a parked bike.'],
  [16, 'High-side crash (extra)', 'Violent roll, airborne, 9 g impact, tumble.', 'CRASH DETECTED → countdown.'],
];
function renderTests() {
  $('testList').innerHTML = TESTS.map(([id, name, desc, exp], i) => `<div class="test"><h3>${i < 13 ? `${i + 1}. ` : ''}${name}</h3><p>${desc}</p><div class="expect">Expected: ${exp}</div><button class="btn" data-test="${id}">Run</button></div>`).join('');
  $('testList').querySelectorAll('button').forEach((b) => b.onclick = () => runTest(b.dataset.test));
}
function runTest(id) {
  if (!S.transport || S.replay) return alert('Connect a device first (tests run on the device firmware).');
  const map = { BTN: 'BTN', GPS: 'GPSLOSS 30', COMMS: 'COMMS 8', CANCEL: 'SIM 7', EXPIRE: 'SIM 7' };
  send(map[id] || `SIM ${id}`);
  document.querySelector('[data-tab="live"]').click();
}

// --------------------------------------------------------------- settings --
function renderSettings() {
  $('setRider').value = settings.rider; $('setNotes').value = settings.notes;
  $('setNtfy').value = settings.ntfy; $('setWebhook').value = settings.webhook;
  $('setPhoneLoc').checked = settings.phoneLoc;
  $('contactList').innerHTML = settings.contacts.length ? settings.contacts.map((c, i) => `<div class="contact"><span>${escapeHtml(c.name)} · ${escapeHtml(c.phone)}</span><button class="btn btn-ghost" data-rm="${i}">Remove</button></div>`).join('') : '<p class="sub">No contacts yet.</p>';
  $('contactList').querySelectorAll('[data-rm]').forEach((b) => b.onclick = () => { settings.contacts.splice(Number(b.dataset.rm), 1); saveSettings(); renderSettings(); });
}
function renderProfile() {
  const c = S.cfg;
  if (!c) return;
  $('setProfile').value = c.profile;
  $('profileInfo').textContent = `Normal lean ≤ ${c.leanN}°, abnormal ≥ ${c.leanC}°, rotation ${c.gyrN}–${c.gyrC} °/s, impact ${c.accN}–${c.accC} g. Countdown threshold ${c.crash}% with ≥ ${c.minInd} indicators.`;
  renderGauge();
}
function escapeHtml(s) { return String(s).replace(/[&<>"']/g, (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch])); }

let geoWatch = null;
function updateGeo() {
  if (settings.phoneLoc && !geoWatch && 'geolocation' in navigator) {
    geoWatch = navigator.geolocation.watchPosition(
      (p) => { S.phoneFix = { lat: p.coords.latitude, lon: p.coords.longitude, acc: p.coords.accuracy, ts: Date.now() }; },
      () => { S.phoneFix = null; }, { enableHighAccuracy: true, maximumAge: 10000 });
  } else if (!settings.phoneLoc && geoWatch !== null) {
    navigator.geolocation.clearWatch(geoWatch); geoWatch = null; S.phoneFix = null;
  }
}

// ------------------------------------------------------------------ wiring --
document.querySelectorAll('.tab').forEach((t) => t.onclick = () => {
  document.querySelectorAll('.tab').forEach((x) => x.classList.toggle('active', x === t));
  document.querySelectorAll('.view').forEach((v) => v.classList.toggle('active', v.id === `view-${t.dataset.tab}`));
});
$('btnSerial').onclick = () => connect('USB');
$('btnBle').onclick = () => connect('BLE');
$('btnDisconnect').onclick = () => disconnect();
$('btnDemo').onclick = async () => {
  try { const r = await fetch('samples/demo-session.log'); if (!r.ok) throw new Error(r.status); connect(await r.text()); }
  catch { $('replayFile').click(); }
};
$('replayFile').onchange = async (e) => { const f = e.target.files[0]; if (f) connect(await f.text()); };
$('btnCancelPossible').onclick = () => send('CANCEL');
$('btnSyncLog').onclick = () => send('LOG');
$('btnClearLog').onclick = () => { if (confirm('Clear the incident history stored in this browser?')) { incidents = {}; saveIncidents(); renderIncidents(); } };
$('btnExportJson').onclick = () => download('bsb-incidents.json', JSON.stringify(Object.values(incidents), null, 2), 'application/json');
$('btnExportCsv').onclick = () => {
  const cols = ['id', 'device', 'boot', 'up', 'wall', 'type', 'out', 'conf', 'acc', 'gyr', 'lean', 'gpsOk', 'lat', 'lon', 'flags'];
  const rows = Object.values(incidents).map((r) => cols.map((c) => JSON.stringify(c === 'wall' && r.wall ? new Date(r.wall).toISOString() : r[c] ?? '')).join(','));
  download('bsb-incidents.csv', [cols.join(','), ...rows].join('\n'), 'text/csv');
};
['setRider', 'setNotes', 'setNtfy', 'setWebhook'].forEach((id) => $(id).onchange = () => {
  settings.rider = $('setRider').value.trim(); settings.notes = $('setNotes').value.trim();
  settings.ntfy = $('setNtfy').value.trim(); settings.webhook = $('setWebhook').value.trim();
  saveSettings(); render();
});
$('setPhoneLoc').onchange = () => { settings.phoneLoc = $('setPhoneLoc').checked; saveSettings(); updateGeo(); render(); };
$('btnAddContact').onclick = () => {
  const name = $('newContactName').value.trim(), phone = $('newContactPhone').value.trim();
  if (!phone) return;
  settings.contacts.push({ name: name || phone, phone }); saveSettings();
  $('newContactName').value = ''; $('newContactPhone').value = ''; renderSettings(); render();
};
$('btnTestAlert').onclick = async () => {
  if (!settings.ntfy && !settings.webhook) { $('testAlertResult').textContent = 'Configure ntfy or a webhook first.'; return; }
  $('testAlertResult').textContent = 'Sending…';
  const fake = { id: 0, dev: S.device?.dev, conf: 0, acc: 0, gyr: 0, lean: 0, gps: S.tel?.gps ?? 0 };
  const { results } = await dispatch(fake, resolveLocation(fake, S.phoneFix, settings), settings, S.device?.dev, true);
  $('testAlertResult').textContent = results.map((r) => `${r.channel}: ${r.ok ? 'OK' : 'FAILED'} (${r.detail})`).join(' · ');
};
$('setProfile').onchange = () => send(`PROFILE ${$('setProfile').value}`);
document.querySelectorAll('[data-cmd]').forEach((b) => b.onclick = () => send(b.dataset.cmd));
$('btnSendCmd').onclick = () => { const v = $('cmdInput').value.trim(); if (v) { send(v); $('cmdInput').value = ''; } };
$('cmdInput').onkeydown = (e) => { if (e.key === 'Enter') $('btnSendCmd').click(); };

renderGauge(); renderTests(); renderSettings(); renderIncidents(); updateGeo(); render();
setInterval(render, 500);
