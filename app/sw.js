// Offline support for the dashboard (GitHub Pages / any static host).
// Network-first: always tries the latest version, falls back to the cached copy
// when offline. Only same-origin GET requests are handled — alert deliveries
// (ntfy / webhook POSTs) always go straight to the network.
const CACHE = 'bsb-v1';
const SHELL = [
  './', 'index.html', 'css/style.css', 'js/main.js', 'js/transports.js', 'js/emergency.js',
  'manifest.webmanifest', 'icon.svg', 'icon-192.png', 'icon-512.png', 'samples/demo-session.log',
];

self.addEventListener('install', (e) => {
  e.waitUntil(caches.open(CACHE).then((c) => c.addAll(SHELL)).then(() => self.skipWaiting()));
});

self.addEventListener('activate', (e) => {
  e.waitUntil(caches.keys()
    .then((keys) => Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k))))
    .then(() => self.clients.claim()));
});

self.addEventListener('fetch', (e) => {
  const req = e.request;
  if (req.method !== 'GET' || new URL(req.url).origin !== self.location.origin) return;
  e.respondWith(
    fetch(req)
      .then((res) => {
        if (res.ok) { const copy = res.clone(); caches.open(CACHE).then((c) => c.put(req, copy)); }
        return res;
      })
      .catch(() => caches.match(req, { ignoreSearch: true }).then((hit) => hit || caches.match('index.html'))),
  );
});
