// Meridian Tablet GCS — Service Worker
//
// Caches the tablet GCS shell (mission.html + manifest + icons) plus the
// Leaflet runtime so the tablet works offline once you've loaded it once.
// Runtime cache for tile images so already-viewed map regions are still
// available when the tablet is out of range of the ship's wifi.
//
// Versioning: bump CACHE_VERSION when you ship a new mission.html — old
// caches are cleared on activation.

const CACHE_VERSION = 'meridian-gcs-v18';

const SHELL = [
    '/',
    '/mission.html',
    '/test-console.html',
    '/manifest.json',
    '/icons/icon-72.png',
    '/icons/icon-96.png',
    '/icons/icon-128.png',
    '/icons/icon-144.png',
    '/icons/icon-152.png',
    '/icons/icon-192.png',
    '/icons/icon-384.png',
    '/icons/icon-512.png',
    'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css',
    'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js',
];

self.addEventListener('install', (event) => {
    event.waitUntil(
        caches.open(CACHE_VERSION).then((cache) =>
            // addAll fails atomically if any one resource 404s; use Promise.allSettled
            // pattern by adding individually so a single missing icon doesn't block install.
            Promise.allSettled(SHELL.map((url) =>
                fetch(url, { mode: 'no-cors' })
                    .then((res) => cache.put(url, res))
                    .catch(() => null)
            ))
        ).then(() => self.skipWaiting())
    );
});

self.addEventListener('activate', (event) => {
    event.waitUntil(
        caches.keys().then((keys) =>
            Promise.all(keys.filter((k) => k !== CACHE_VERSION).map((k) => caches.delete(k)))
        ).then(() => self.clients.claim())
    );
});

self.addEventListener('fetch', (event) => {
    const req = event.request;
    const url = new URL(req.url);

    // Never cache the WebSocket upgrade or other non-GET
    if (req.method !== 'GET') return;

    // Map tiles: cache-first with network fallback (so out-of-range tablet
    // can still pan around recently viewed area).
    const isTile = /tile\.openstreetmap|cartocdn|mapbox|stamen-tiles|tile\.openseamap/i.test(url.host);
    if (isTile) {
        event.respondWith(
            caches.match(req).then((cached) => cached || fetch(req).then((res) => {
                const clone = res.clone();
                caches.open(CACHE_VERSION + '-tiles').then((c) => c.put(req, clone));
                return res;
            }).catch(() => cached))
        );
        return;
    }

    // Same-origin shell + Leaflet: cache-first, network update in background.
    event.respondWith(
        caches.match(req).then((cached) => {
            const fetchPromise = fetch(req).then((res) => {
                if (res && res.status === 200) {
                    const clone = res.clone();
                    caches.open(CACHE_VERSION).then((c) => c.put(req, clone));
                }
                return res;
            }).catch(() => cached);
            return cached || fetchPromise;
        })
    );
});
