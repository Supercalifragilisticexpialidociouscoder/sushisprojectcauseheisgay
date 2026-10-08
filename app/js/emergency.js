// SOS handling on the phone/PC side.
//
// The device has no GSM module, so this page is the only path to emergency
// contacts. Rules:
//   * one dispatch per incident id (the device re-sends until we acknowledge)
//   * simulated / replayed incidents are NEVER dispatched
//   * only configured channels are used; nothing calls emergency services
//   * never invent a location: device GPS > phone location (if enabled) > UNAVAILABLE

const HANDLED_KEY = 'bsb.sosHandled';

export function loadHandled() {
  try { return new Set(JSON.parse(localStorage.getItem(HANDLED_KEY) || '[]')); } catch { return new Set(); }
}
function saveHandled(set) {
  try { localStorage.setItem(HANDLED_KEY, JSON.stringify([...set].slice(-100))); } catch {}
}

export function resolveLocation(sos, phoneFix, settings) {
  if (sos.gps === 2 && typeof sos.lat === 'number') {
    return { ok: true, source: 'Device GPS', lat: sos.lat, lon: sos.lon, acc: sos.hacc > 0 ? sos.hacc : null, time: sos.utc || null };
  }
  if (settings.phoneLoc && phoneFix && Date.now() - phoneFix.ts < 120000) {
    return { ok: true, source: 'Phone location', lat: phoneFix.lat, lon: phoneFix.lon, acc: phoneFix.acc, time: new Date(phoneFix.ts).toISOString() };
  }
  return { ok: false, reason: sos.gps === 0 ? 'no GPS module on the device' : 'no valid GPS fix' };
}

export function mapsLink(loc) {
  return `https://maps.google.com/?q=${loc.lat.toFixed(7)},${loc.lon.toFixed(7)}`;
}

export function composeMessage(sos, loc, settings, device) {
  const when = new Date().toLocaleString();
  const lines = [
    'EMERGENCY ALERT - Bike Safety Belt',
    `Rider: ${settings.rider || 'not set'} (device ${sos.dev || device || '?'})`,
    'POSSIBLE ACCIDENT DETECTED automatically. The rider did not cancel the alarm.',
    'This is an automatic detection, not a confirmed accident.',
    `Time: ${when}`,
    loc.ok
      ? `Location: ${mapsLink(loc)} (${loc.source}${loc.acc ? `, approx. +/-${Math.round(loc.acc)} m` : ', accuracy unknown'})`
      : `LOCATION UNAVAILABLE (${loc.reason})`,
    `Crash confidence: ${sos.conf}% (impact ${sos.acc} g, rotation ${sos.gyr} deg/s, lean ${sos.lean} deg)`,
  ];
  if (settings.notes) lines.push(`Notes: ${settings.notes}`);
  return lines.join('\n');
}

async function postWithRetry(url, options, tries = 3) {
  let last = '';
  for (let i = 0; i < tries; i++) {
    try {
      const r = await fetch(url, options);
      if (r.ok) return { ok: true, detail: `HTTP ${r.status}` };
      last = `HTTP ${r.status}`;
    } catch (e) { last = e.message || 'network error'; }
    await new Promise((res) => setTimeout(res, 1500 * (i + 1)));
  }
  return { ok: false, detail: last };
}

// Returns a list of {channel, ok, detail}.
export async function dispatch(sos, loc, settings, device, isTest = false) {
  const text = composeMessage(sos, loc, settings, device);
  const results = [];
  const jobs = [];
  if (settings.ntfy) {
    jobs.push(postWithRetry(`https://ntfy.sh/${encodeURIComponent(settings.ntfy)}`, {
      method: 'POST', body: text,
      headers: { Title: isTest ? 'TEST - Bike Safety Belt' : 'EMERGENCY - possible accident', Priority: isTest ? 'default' : 'urgent', Tags: 'rotating_light' },
    }).then((r) => results.push({ channel: 'ntfy.sh', ...r })));
  }
  if (settings.webhook) {
    jobs.push(postWithRetry(settings.webhook, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ type: isTest ? 'test' : 'sos', incident: sos.id, device: sos.dev || device, rider: settings.rider,
        confidence: sos.conf, location: loc.ok ? { lat: loc.lat, lon: loc.lon, accuracy_m: loc.acc, source: loc.source, maps: mapsLink(loc) } : null,
        contacts: settings.contacts, message: text, sent_at: new Date().toISOString() }),
    }).then((r) => results.push({ channel: 'Webhook', ...r })));
  }
  await Promise.all(jobs);
  return { text, results };
}

export class SosCoordinator {
  constructor(getSettings, sendCmd, onUpdate) {
    this.handled = loadHandled();
    this.getSettings = getSettings;
    this.sendCmd = sendCmd;
    this.onUpdate = onUpdate;
    this.current = null;
  }
  // Called for every {"t":"sos"} message from the device.
  async handle(sos, ctx) {
    this.sendCmd(`SOSACK ${sos.id}`);   // device stops re-sending (receipt, not delivery!)
    if (this.current?.id === sos.id) return;
    const settings = this.getSettings();
    const loc = resolveLocation(sos, ctx.phoneFix, settings);
    const simulated = !!sos.sim || ctx.replay;
    const st = { id: sos.id, sos, loc, simulated, text: composeMessage(sos, loc, settings, ctx.device), results: [], dispatching: false, duplicate: this.handled.has(sos.id) };
    this.current = st;
    this.onUpdate(st);
    if (simulated || st.duplicate) return;
    this.handled.add(sos.id);
    saveHandled(this.handled);
    if (!settings.ntfy && !settings.webhook) return;
    st.dispatching = true;
    this.onUpdate(st);
    const { results } = await dispatch(sos, loc, settings, ctx.device);
    st.results = results;
    st.dispatching = false;
    this.onUpdate(st);
  }
}
