import type { Moonshot } from '../data/moonshots'

export interface PageMetadata {
  title: string
  description: string
  image: string
  url: string
  type: 'website' | 'article'
  keywords?: string[]
  article?: {
    publishedTime?: string
    author?: string
    tags?: string[]
    section?: string
  }
  breadcrumbs?: Array<{
    name: string
    url: string
  }>
}

const SITE_URL = 'https://www.opoch.com'
const DEFAULT_IMAGE = `${SITE_URL}/og-image.jpg`

// Base site metadata matching the "Age of Truth" brand
export const SITE_METADATA = {
  siteName: 'Opoch',
  tagline: 'Age of Truth',
  baseTitle: 'Opoch: Age of Truth',
  description: 'Deep tech consulting without the fluff. We architect, design, and implement solutions that work. From AI/ML systems to distributed architectures, we deliver clarity and results.',
  url: SITE_URL,
  image: DEFAULT_IMAGE,
  address: {
    street: '300 4th St',
    city: 'San Francisco',
    state: 'CA',
    zip: '94107',
    country: 'US'
  },
  contact: {
    email: 'hello@opoch.com',
    twitter: '@opochhq'
  },
  membership: {
    price: '11111.00',
    currency: 'USD',
    billing: 'monthly'
  }
}

// Page-specific metadata configurations
export const PAGE_METADATA: Record<string, PageMetadata> = {
  home: {
    title: SITE_METADATA.baseTitle,
    description: 'Get unstuck. Build faster. Decide with confidence. Technical consulting for ambitious builders who can\'t afford wrong answers. On-site sessions @ 300 4th St, San Francisco. $11,111/month membership.',
    image: DEFAULT_IMAGE,
    url: SITE_URL,
    type: 'website',
    keywords: [
      'tech consulting',
      'deep tech',
      'AI/ML',
      'distributed systems', 
      'architecture',
      'engineering',
      'San Francisco',
      'membership',
      '300 4th St',
      'Age of Truth',
      'technical consulting'
    ]
  },
  
  moonshots: {
    title: 'Moonshots | Opoch',
    description: 'Explore ambitious technical challenges we\'re tackling. From quantum error correction to fusion control, see what\'s possible in the Age of Truth.',
    image: DEFAULT_IMAGE,
    url: `${SITE_URL}/moonshots`,
    type: 'website',
    keywords: [
      'moonshots',
      'deep tech',
      'research',
      'innovation',
      'technical challenges',
      'breakthrough',
      'physics',
      'engineering',
      'Age of Truth'
    ],
    breadcrumbs: [
      { name: 'Home', url: SITE_URL },
      { name: 'Moonshots', url: `${SITE_URL}/moonshots` }
    ]
  },
  
  updates: {
    title: 'Updates | Opoch',
    description: 'Latest insights, breakthroughs, and technical updates from Opoch. Stay informed about the cutting edge of deep tech consulting.',
    image: DEFAULT_IMAGE,
    url: `${SITE_URL}/updates`,
    type: 'website',
    keywords: [
      'updates',
      'insights',
      'tech news',
      'breakthroughs',
      'consulting',
      'engineering updates',
      'deep tech news'
    ],
    breadcrumbs: [
      { name: 'Home', url: SITE_URL },
      { name: 'Updates', url: `${SITE_URL}/updates` }
    ]
  },

  admin: {
    title: 'Admin Dashboard | Opoch',
    description: 'Administrative dashboard for Opoch operations and management.',
    image: DEFAULT_IMAGE,
    url: `${SITE_URL}/admin`,
    type: 'website'
  }
}

// Generate metadata for moonshot detail pages
export function generateMoonshotMetadata(moonshot: Moonshot): PageMetadata {
  // Create unique, descriptive titles that capture the essence of each moonshot
  const domainContext = {
    physics: 'Physics Breakthrough',
    energy: 'Energy Innovation', 
    bio: 'Biophysics Research',
    infra: 'Infrastructure Solution',
    robotics: 'Robotics Advancement'
  }
  
  const statusContext = {
    open: 'Open Challenge',
    'in-progress': 'Active Research',
    partners: 'Partnership Opportunity'
  }
  
  // Enhanced title with context
  const title = `${moonshot.title}: ${moonshot.tagline} | ${domainContext[moonshot.domain]} | Age of Truth`
  
  // Rich description with problem context
  const description = `${moonshot.tagline} - ${moonshot.excerpt} ${statusContext[moonshot.status]} in ${moonshot.domain}. Timeline: ${moonshot.timeline}. Complexity: ${moonshot.complexity}/5. Join Opoch in the Age of Truth.`
  
  // Use custom OG image, cover image, or fall back to default
  const moonshotImage = moonshot.ogImage || moonshot.coverImage || DEFAULT_IMAGE
  
  return {
    title,
    description,
    image: moonshotImage, // Will fall back to DEFAULT_IMAGE if custom image doesn't exist
    url: `${SITE_URL}/moonshots/${moonshot.slug}`,
    type: 'article',
    keywords: [
      moonshot.domain,
      'moonshot',
      'technical challenge',
      'deep tech',
      'research',
      moonshot.title.toLowerCase().replace(/\s+/g, ' '),
      'opoch',
      'Age of Truth',
      `${moonshot.domain} moonshot`,
      'breakthrough technology',
      moonshot.tagline.toLowerCase(),
      `${moonshot.domain} research`,
      statusContext[moonshot.status].toLowerCase(),
      `complexity ${moonshot.complexity}`,
      moonshot.timeline.toLowerCase()
    ],
    article: {
      publishedTime: new Date().toISOString(),
      author: 'Opoch Team',
      tags: [moonshot.domain, 'moonshot', 'technical-challenge', moonshot.status, `complexity-${moonshot.complexity}`],
      section: `${domainContext[moonshot.domain]} Moonshots`
    },
    breadcrumbs: [
      { name: 'Home', url: SITE_URL },
      { name: 'Moonshots', url: `${SITE_URL}/moonshots` },
      { name: moonshot.title, url: `${SITE_URL}/moonshots/${moonshot.slug}` }
    ]
  }
}

// Generate metadata for update detail pages (for future use)
export function generateUpdateMetadata(update: any): PageMetadata {
  const title = `${update.title} | Updates | Opoch`
  const description = update.excerpt || `Latest update from Opoch: ${update.title}`
  
  return {
    title,
    description,
    image: update.coverImage || DEFAULT_IMAGE,
    url: `${SITE_URL}/updates/${update.slug}`,
    type: 'article',
    keywords: [
      'update',
      'insight',
      'tech',
      'consulting',
      'opoch',
      'deep tech',
      'breakthrough'
    ],
    article: {
      publishedTime: update.publishedAt || new Date().toISOString(),
      author: update.author || 'Opoch Team',
      tags: update.tags || ['update'],
      section: 'Updates'
    },
    breadcrumbs: [
      { name: 'Home', url: SITE_URL },
      { name: 'Updates', url: `${SITE_URL}/updates` },
      { name: update.title, url: `${SITE_URL}/updates/${update.slug}` }
    ]
  }
}

// Generate admin page metadata
export function generateAdminMetadata(pageType: 'leads' | 'emails'): PageMetadata {
  const titles = {
    leads: 'Lead Management | Admin | Opoch',
    emails: 'Email Management | Admin | Opoch'
  }
  
  const descriptions = {
    leads: 'Manage and track potential client leads and applications.',
    emails: 'Manage email templates and communications.'
  }
  
  return {
    title: titles[pageType],
    description: descriptions[pageType],
    image: DEFAULT_IMAGE,
    url: `${SITE_URL}/admin/${pageType}`,
    type: 'website'
  }
}

// Main function to get metadata for any page
export function getPageMetadata(pageKey: string, data?: any): PageMetadata {
  // Handle moonshot detail pages
  if (pageKey.startsWith('moonshot-') && data) {
    return generateMoonshotMetadata(data)
  }
  
  // Handle update detail pages
  if (pageKey.startsWith('update-') && data) {
    return generateUpdateMetadata(data)
  }
  
  // Handle admin pages
  if (pageKey.startsWith('admin-')) {
    const adminType = pageKey.replace('admin-', '') as 'leads' | 'emails'
    return generateAdminMetadata(adminType)
  }
  
  // Return predefined page metadata or fallback to home
  return PAGE_METADATA[pageKey] || PAGE_METADATA.home
}

// Helper function to generate structured data for organization
export function getOrganizationStructuredData() {
  return {
    '@context': 'https://schema.org',
    '@type': 'Organization',
    name: SITE_METADATA.siteName,
    url: SITE_METADATA.url,
    logo: `${SITE_METADATA.url}/Opoch Assets/Opoch Logo.png`,
    description: SITE_METADATA.description,
    slogan: SITE_METADATA.tagline,
    address: {
      '@type': 'PostalAddress',
      streetAddress: SITE_METADATA.address.street,
      addressLocality: SITE_METADATA.address.city,
      addressRegion: SITE_METADATA.address.state,
      postalCode: SITE_METADATA.address.zip,
      addressCountry: SITE_METADATA.address.country
    },
    contactPoint: {
      '@type': 'ContactPoint',
      email: SITE_METADATA.contact.email,
      contactType: 'sales',
      areaServed: 'Worldwide',
      availableLanguage: 'English'
    },
    foundingDate: '2023',
    sameAs: [
      'https://x.com/opochhq',
      'https://github.com/opoch'
    ],
    hasOfferCatalog: {
      '@type': 'OfferCatalog',
      name: 'Deep Tech Consulting Services',
      itemListElement: [
        {
          '@type': 'Offer',
          name: 'Technical Consulting Membership',
          description: 'Monthly membership for deep tech consulting with on-site sessions in San Francisco',
          price: SITE_METADATA.membership.price,
          priceCurrency: SITE_METADATA.membership.currency,
          priceSpecification: {
            '@type': 'RecurringPriceSpecification',
            price: SITE_METADATA.membership.price,
            priceCurrency: SITE_METADATA.membership.currency,
            billingDuration: 'P1M',
            billingIncrement: 1
          },
          availability: 'InStock',
          areaServed: {
            '@type': 'Place',
            name: 'Worldwide'
          }
        }
      ]
    },
    serviceArea: {
      '@type': 'Place',
      name: 'Worldwide'
    }
  }
}
