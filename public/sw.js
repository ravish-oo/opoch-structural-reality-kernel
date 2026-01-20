const CACHE_NAME = 'opoch-v1';
const OFFLINE_URL = '/offline.html';

// Resources to cache
const urlsToCache = [
  '/',
  '/moonshots',
  '/updates',
  '/index.css',
  OFFLINE_URL
];

// Install event
self.addEventListener('install', event => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then(cache => cache.addAll(urlsToCache))
      .then(() => self.skipWaiting())
  );
});

// Activate event
self.addEventListener('activate', event => {
  event.waitUntil(
    caches.keys().then(cacheNames => {
      return Promise.all(
        cacheNames
          .filter(cacheName => cacheName !== CACHE_NAME)
          .map(cacheName => caches.delete(cacheName))
      );
    }).then(() => self.clients.claim())
  );
});

// Fetch event - Network first, fall back to cache
self.addEventListener('fetch', event => {
  if (event.request.method !== 'GET') return;
  
  // Ignore non-http(s) requests (like chrome-extension://)
  const url = new URL(event.request.url);
  if (url.protocol !== 'http:' && url.protocol !== 'https:') {
    return;
  }
  
  // Don't cache API requests or external resources
  const isAPI = url.hostname.includes('supabase.co') || 
                url.hostname.includes('supabase.io') ||
                url.hostname.includes('googleapis.com') ||
                url.pathname.startsWith('/api/');
  
  if (isAPI || url.origin !== self.location.origin) {
    return; // Let these requests pass through normally
  }

  event.respondWith(
    fetch(event.request)
      .then(response => {
        // Only cache successful responses from our own origin
        if (response.status === 200 && event.request.url.startsWith(self.location.origin)) {
          const responseToCache = response.clone();
          caches.open(CACHE_NAME)
            .then(cache => {
              cache.put(event.request, responseToCache);
            });
        }
        
        return response;
      })
      .catch(() => {
        // Network request failed, try to get from cache
        return caches.match(event.request)
          .then(response => {
            if (response) {
              return response;
            }
            // If requesting an HTML page and it's not in cache, show offline page
            if (event.request.headers.get('accept')?.includes('text/html')) {
              return caches.match(OFFLINE_URL);
            }
          });
      })
  );
});