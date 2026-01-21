import { writeFileSync, readFileSync } from 'fs';

// Load moonshot slugs from JSON file
const moonshotSlugs = JSON.parse(readFileSync('./scripts/moonshots-slugs.json', 'utf8'));

const DOMAIN = 'https://www.opoch.com';
const currentDate = new Date().toISOString().split('T')[0];

// Define all routes with priorities and change frequencies
const routes = [
  // Main pages - highest priority
  { url: '/', priority: '1.0', changefreq: 'daily' },
  
  // Major sections - high priority
  { url: '/moonshots', priority: '0.9', changefreq: 'weekly' },
  { url: '/updates', priority: '0.8', changefreq: 'weekly' },
  
  // Authentication - medium-low priority
  { url: '/auth/callback', priority: '0.3', changefreq: 'never' },
  
  // Admin routes - low priority, restricted access
  { url: '/admin/leads', priority: '0.2', changefreq: 'monthly' },
  { url: '/admin/emails', priority: '0.2', changefreq: 'monthly' },
  
  // Legal pages - low priority
  { url: '/privacy', priority: '0.3', changefreq: 'monthly' },
  { url: '/terms', priority: '0.3', changefreq: 'monthly' },
];

// Generate dynamic moonshot routes with high priority
const moonshotRoutes = moonshotSlugs.map(slug => ({
  url: `/moonshots/${slug}`,
  priority: '0.8',
  changefreq: 'monthly',
  lastmod: currentDate // Could be more specific per moonshot
}));

// Future: Generate dynamic update routes (when implemented)
// const updateRoutes = updates.map(update => ({
//   url: `/updates/${update.slug}`,
//   priority: '0.7',
//   changefreq: 'never', // Updates don't change once published
//   lastmod: update.publishedAt?.split('T')[0] || currentDate
// }));

// Combine all routes
const allRoutes = [
  ...routes.map(route => ({ ...route, lastmod: currentDate })),
  ...moonshotRoutes,
  // ...updateRoutes
];

// Generate comprehensive sitemap XML with all namespaces
const sitemap = `<?xml version="1.0" encoding="UTF-8"?>
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9"
        xmlns:news="http://www.google.com/schemas/sitemap-news/0.9"
        xmlns:xhtml="http://www.w3.org/1999/xhtml"
        xmlns:image="http://www.google.com/schemas/sitemap-image/1.1"
        xmlns:video="http://www.google.com/schemas/sitemap-video/1.1">
${allRoutes.map(route => `  <url>
    <loc>${DOMAIN}${route.url}</loc>
    <lastmod>${route.lastmod}</lastmod>
    <changefreq>${route.changefreq}</changefreq>
    <priority>${route.priority}</priority>
  </url>`).join('\n')}
</urlset>`;

// Write sitemap to public directory
writeFileSync('./public/sitemap.xml', sitemap);
console.log(`✅ Sitemap generated with ${allRoutes.length} URLs!`);

// Generate comprehensive robots.txt with proper directives
const robotsTxt = `# Robots.txt for Opoch - Age of Truth
# https://www.opoch.com/robots.txt

User-agent: *
Allow: /
Disallow: /admin/
Disallow: /api/

# Allow specific search engines
User-agent: Googlebot
Allow: /

User-agent: Bingbot
Allow: /

User-agent: facebookexternalhit
Allow: /

User-agent: Twitterbot
Allow: /

# Block AI training crawlers (optional)
User-agent: GPTBot
Disallow: /

User-agent: Google-Extended
Disallow: /

User-agent: CCBot
Disallow: /

# Sitemap location
Sitemap: ${DOMAIN}/sitemap.xml

# Crawl delay for respectful crawling
Crawl-delay: 1

# Host directive
Host: ${DOMAIN}
`;

writeFileSync('./public/robots.txt', robotsTxt);
console.log('✅ Robots.txt updated with comprehensive directives!');