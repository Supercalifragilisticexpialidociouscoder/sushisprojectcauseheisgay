// Device transports. Each one delivers text lines via onLine(line) and accepts
// commands via send(text). The rest of the app never knows which one is used.

class LineSplitter {
  constructor(onLine) { this.buf = ''; this.onLine = onLine; }
  push(text) {
    this.buf += text;
    let i;
    while ((i = this.buf.indexOf('\n')) >= 0) {
      const line = this.buf.slice(0, i).replace(/\r$/, '');
      this.buf = this.buf.slice(i + 1);
      if (line.length) this.onLine(line);
    }
    if (this.buf.length > 4096) this.buf = '';   // garbage protection
  }
}

// ---- USB serial (Web Serial API: Chrome / Edge desktop) ---------------------
export class SerialTransport {
  constructor(onLine, onClose) { this.kind = 'USB'; this.onClose = onClose; this.split = new LineSplitter(onLine); }
  static supported() { return 'serial' in navigator; }
  async connect() {
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate: 115200 });
    this.encoder = new TextEncoder();
    this.writer = this.port.writable.getWriter();
    this.closed = false;
    this.readLoop();
  }
  async readLoop() {
    const decoder = new TextDecoder();
    try {
      while (this.port.readable && !this.closed) {
        this.reader = this.port.readable.getReader();
        try {
          for (;;) {
            const { value, done } = await this.reader.read();
            if (done) break;
            this.split.push(decoder.decode(value, { stream: true }));
          }
        } finally { this.reader.releaseLock(); }
      }
    } catch (e) { /* device unplugged */ }
    if (!this.closed) { this.closed = true; this.onClose('USB connection lost'); }
  }
  async send(text) {
    if (!this.writer || this.closed) return false;
    try { await this.writer.write(this.encoder.encode(text + '\n')); return true; } catch { return false; }
  }
  async disconnect() {
    this.closed = true;
    try { await this.reader?.cancel(); } catch {}
    try { this.writer?.releaseLock(); } catch {}
    try { await this.port?.close(); } catch {}
  }
}

// ---- Bluetooth LE, Nordic UART Service (ESP32) --------------------------------
const NUS = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const NUS_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';
const NUS_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';

export class BleTransport {
  constructor(onLine, onClose) { this.kind = 'BLE'; this.onClose = onClose; this.split = new LineSplitter(onLine); this.queue = Promise.resolve(); }
  static supported() { return 'bluetooth' in navigator; }
  async connect() {
    this.device = await navigator.bluetooth.requestDevice({ filters: [{ namePrefix: 'BSB-' }], optionalServices: [NUS] });
    this.device.addEventListener('gattserverdisconnected', () => { if (!this.closed) { this.closed = true; this.onClose('Bluetooth connection lost'); } });
    const server = await this.device.gatt.connect();
    const svc = await server.getPrimaryService(NUS);
    this.rx = await svc.getCharacteristic(NUS_RX);
    const tx = await svc.getCharacteristic(NUS_TX);
    const decoder = new TextDecoder();
    tx.addEventListener('characteristicvaluechanged', (e) => this.split.push(decoder.decode(e.target.value)));
    await tx.startNotifications();
    this.closed = false;
  }
  send(text) {
    // Serialise writes; 20-byte chunks fit the default BLE MTU.
    const data = new TextEncoder().encode(text + '\n');
    this.queue = this.queue.then(async () => {
      for (let i = 0; i < data.length; i += 20) await this.rx.writeValue(data.slice(i, i + 20));
      return true;
    }).catch(() => false);
    return this.queue;
  }
  async disconnect() { this.closed = true; try { this.device?.gatt.disconnect(); } catch {} }
}

// ---- Replay of a recorded session ("<ms>\t<line>" per line) -------------------
export class ReplayTransport {
  constructor(onLine, onClose, text) {
    this.kind = 'REPLAY'; this.onLine = onLine; this.onClose = onClose;
    this.items = text.split('\n').filter(Boolean).map((l) => {
      const tab = l.indexOf('\t');
      return tab > 0 ? { t: Number(l.slice(0, tab)), line: l.slice(tab + 1) } : { t: NaN, line: l };
    });
  }
  async connect() {
    this.closed = false;
    const t0 = this.items.find((x) => !isNaN(x.t))?.t ?? 0;
    const start = performance.now();
    let i = 0;
    const step = () => {
      if (this.closed) return;
      const now = performance.now() - start;
      while (i < this.items.length && (isNaN(this.items[i].t) || this.items[i].t - t0 <= now)) this.onLine(this.items[i++].line);
      if (i >= this.items.length) { this.closed = true; this.onClose('Replay finished'); return; }
      this.timer = setTimeout(step, 20);
    };
    step();
  }
  async send() { return true; }   // commands are ignored during replay
  async disconnect() { this.closed = true; clearTimeout(this.timer); }
}
