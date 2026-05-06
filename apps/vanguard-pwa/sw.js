// VANGUARD Mesh Relay — Service Worker
// Caches everything for full offline operation

const CACHE = 'vanguard-v1';
const FONTS = 'vanguard-fonts-v1';

const CORE_ASSETS = [
  '/',
  '/index.html',
  '/manifest.json',
  '/icons/icon-192.png',
  '/icons/icon-512.png'
];

const FONT_URLS = [
  'https://fonts.googleapis.com/css2?family=Orbitron:wght@400;500;700;900&family=DM+Mono:wght@300;400;500&family=DM+Sans:wght@300;400;500&display=swap'
];

// ── INSTALL: cache core assets ──
self.addEventListener('install', e => {
  e.waitUntil(
    Promise.all([
      caches.open(CACHE).then(c => c.addAll(CORE_ASSETS)),
      caches.open(FONTS).then(c => c.addAll(FONT_URLS))
    ]).then(() => self.skipWaiting())
  );
});

// ── ACTIVATE: clean old caches ──
self.addEventListener('activate', e => {
  e.waitUntil(
    caches.keys().then(keys =>
      Promise.all(
        keys.filter(k => k !== CACHE && k !== FONTS)
            .map(k => caches.delete(k))
      )
    ).then(() => self.clients.claim())
  );
});

// ── FETCH: cache-first for assets, network-first for API ──
self.addEventListener('fetch', e => {
  const url = new URL(e.request.url);

  // Google Fonts — cache first
  if (url.hostname.includes('fonts.g') || url.hostname.includes('fonts.')) {
    e.respondWith(
      caches.open(FONTS).then(c =>
        c.match(e.request).then(cached =>
          cached || fetch(e.request).then(res => {
            c.put(e.request, res.clone());
            return res;
          })
        )
      )
    );
    return;
  }

  // Meridian API calls — network only, never cache
  if (url.pathname.startsWith('/api/') || url.port === '5760') {
    e.respondWith(fetch(e.request).catch(() =>
      new Response(JSON.stringify({ error: 'offline', message: 'No connection to Meridian bridge' }),
        { headers: { 'Content-Type': 'application/json' } })
    ));
    return;
  }

  // Everything else — cache first, fallback to network, update cache
  e.respondWith(
    caches.open(CACHE).then(c =>
      c.match(e.request).then(cached => {
        const networkFetch = fetch(e.request).then(res => {
          if (res.ok) c.put(e.request, res.clone());
          return res;
        });
        return cached || networkFetch;
      })
    )
  );
});

// ── BACKGROUND SYNC: queue commands when offline ──
self.addEventListener('sync', e => {
  if (e.tag === 'meridian-commands') {
    e.waitUntil(flushCommandQueue());
  }
});

async function flushCommandQueue() {
  const db = await openDB();
  const commands = await db.getAll('pending-commands');
  for (const cmd of commands) {
    try {
      await fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(cmd)
      });
      await db.delete('pending-commands', cmd.id);
    } catch (err) {
      console.warn('Command flush failed, will retry:', cmd);
    }
  }
}

// ── PUSH: alert notifications (e-stop alerts, battery warnings) ──
self.addEventListener('push', e => {
  const data = e.data ? e.data.json() : {};
  const options = {
    body: data.body || 'Vanguard alert',
    icon: '/icons/icon-192.png',
    badge: '/icons/icon-72.png',
    vibrate: data.critical ? [200, 100, 200, 100, 200] : [200],
    data: { url: data.url || '/' },
    actions: data.actions || [],
    requireInteraction: data.critical || false,
    tag: data.tag || 'vanguard-alert'
  };
  e.waitUntil(
    self.registration.showNotification(
      data.title || 'VANGUARD · ' + (data.critical ? 'CRITICAL ALERT' : 'Mission Update'),
      options
    )
  );
});

self.addEventListener('notificationclick', e => {
  e.notification.close();
  e.waitUntil(clients.openWindow(e.notification.data.url || '/'));
});

// Simple IndexedDB helper
function openDB() {
  return new Promise((resolve, reject) => {
    const req = indexedDB.open('vanguard-db', 1);
    req.onupgradeneeded = e => e.target.result.createObjectStore('pending-commands', { keyPath: 'id', autoIncrement: true });
    req.onsuccess = e => resolve({
      getAll: store => new Promise((r,j) => { const t=e.target.result.transaction(store,'readonly'); const req=t.objectStore(store).getAll(); req.onsuccess=()=>r(req.result); req.onerror=j; }),
      delete: (store, id) => new Promise((r,j) => { const t=e.target.result.transaction(store,'readwrite'); const req=t.objectStore(store).delete(id); req.onsuccess=r; req.onerror=j; })
    });
    req.onerror = reject;
  });
}
