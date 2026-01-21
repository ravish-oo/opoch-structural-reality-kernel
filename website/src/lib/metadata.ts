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

const SITE_URL = 'https://opoch.com'
// Use root path for better social media compatibility (no URL encoding needed)
const DEFAULT_IMAGE = `${SITE_URL}/og-image.jpg`

// TODO: Create page-specific OG images and place them in /public/
// Required images (1200x630 for OG):
// - /public/og-home.png - Home page with Ω logo + "Quantum Gravity • Consciousness • Arrow of Time"
// - /public/twitter-home.png - Twitter card version
// - /public/og-falsify.png - "Attempts: 10+" "Successful: 0"
// - /public/twitter-falsify.png - Twitter card version
// - /public/logo.png - Opoch logo
// Once created, update the image paths in PAGE_METADATA below

// Base site metadata matching the "Age of Truth" brand
export const SITE_METADATA = {
  siteName: 'Opoch',
  tagline: 'Age of Truth',
  baseTitle: 'Opoch: Age of Truth',
  description: 'The Theory of Everything is here. All of physics, mathematics, and consciousness, solved and derived. Chat with GPT-5 powered by 122MB verified TOE corpus.',
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
    title: 'The End of Probabilistic Guessing | Opoch',
    description: 'Reasoning is not statistical. It is a strict binary: either derive the proof, or halt and identify the gap. Meet GITM—the first AI architecture that compiles truth instead of predicting tokens.',
    image: DEFAULT_IMAGE,
    url: SITE_URL,
    type: 'website',
    keywords: [
      'opoch',
      'GITM',
      'gauge-invariant truth machine',
      'null-state logic',
      'derivational intelligence',
      'AI reasoning',
      'critpt benchmark',
      'logic engine',
      'truth compiler',
      'proof',
      'reasoning'
    ]
  },

  toe: {
    title: 'The Theory of Everything | Opoch',
    description: 'The Theory of Everything is here. Quantum gravity. Consciousness. Arrow of time. All solved, all verified. Test any claim yourself.',
    image: DEFAULT_IMAGE,
    url: `${SITE_URL}/toe`,
    type: 'website',
    keywords: [
      'theory of everything',
      'TOE',
      'quantum gravity',
      'consciousness',
      'unified theory',
      'physics',
      'mathematics',
      'AI verification',
      'verified corpus',
      'arrow of time',
      'quantum mechanics',
      'general relativity',
      'thermodynamics',
      'fundamental physics'
    ],
    breadcrumbs: [
      { name: 'Home', url: SITE_URL },
      { name: 'TOE', url: `${SITE_URL}/toe` }
    ]
  },

  falsify: {
    title: 'Falsify the Theory of Everything | Opoch',
    description: 'Try to falsify the Theory of Everything. Break one axiom, find one contradiction. 10+ attempts, 0 successful. Submit your counterexample.',
    image: DEFAULT_IMAGE, // TODO: Replace with og-falsify.png when created
    url: `${SITE_URL}/falsify`,
    type: 'website',
    keywords: [
      'falsify TOE',
      'falsification',
      'scientific method',
      'theory testing',
      'counterexample',
      'axioms',
      'verification',
      'theory of everything',
      'TOE',
      'scientific rigor'
    ],
    breadcrumbs: [
      { name: 'Home', url: SITE_URL },
      { name: 'Falsify', url: `${SITE_URL}/falsify` }
    ]
  },

  paper: {
    title: 'The Theory of Everything - Complete Paper | Opoch',
    description: 'The complete Theory of Everything in plain English and full technical PDF. Three axioms, exact receipts, verifiable predictions.',
    image: DEFAULT_IMAGE,
    url: `${SITE_URL}/paper`,
    type: 'article',
    keywords: [
      'theory of everything',
      'TOE',
      'complete paper',
      'technical PDF',
      'plain English',
      'three axioms',
      'exact receipts',
      'verifiable predictions',
      'quantum mechanics',
      'relativity',
      'consciousness',
      'quantum gravity',
      'unified theory'
    ],
    breadcrumbs: [
      { name: 'Home', url: SITE_URL },
      { name: 'Paper', url: `${SITE_URL}/paper` }
    ]
  },

  simpleQuestions: {
    title: 'Simple Questions About the Theory of Everything | Opoch',
    description: 'What is truth? What is time? What is consciousness? Plain answers to the biggest questions—no math, no jargon.',
    image: DEFAULT_IMAGE,
    url: `${SITE_URL}/simple-questions`,
    type: 'website',
    keywords: [
      'theory of everything',
      'TOE',
      'simple questions',
      'FAQ',
      'what is truth',
      'what is time',
      'what is consciousness',
      'plain English',
      'no math',
      'no jargon',
      'physics explained',
      'philosophy'
    ],
    breadcrumbs: [
      { name: 'Home', url: SITE_URL },
      { name: 'Simple Questions', url: `${SITE_URL}/simple-questions` }
    ]
  },

  consulting: {
    title: 'Solve Impossible Problems | Opoch Consulting',
    description: 'We solve impossible problems using fundamental physics. Ruthlessly Basic Truth approach. $11,111/month for founders and researchers.',
    image: DEFAULT_IMAGE,
    url: `${SITE_URL}/consulting`,
    type: 'website',
    keywords: [
      'impossible problems',
      'fundamental physics',
      'Ruthlessly Basic Truth',
      'RBT',
      'theory of everything consulting',
      'TOE consulting',
      'founders',
      'researchers',
      'engineers',
      'real applications',
      'verified results',
      'deep tech',
      'technical consulting',
      'San Francisco'
    ],
    breadcrumbs: [
      { name: 'Home', url: SITE_URL },
      { name: 'Consulting', url: `${SITE_URL}/consulting` }
    ]
  },

  moonshots: {
    title: 'Moonshot Problems | Opoch',
    description: 'High-impact problems the Theory of Everything can solve: fusion containment, room-temp superconductors, protein folding, and more.',
    image: DEFAULT_IMAGE,
    url: `${SITE_URL}/moonshots`,
    type: 'website',
    keywords: [
      'moonshot problems',
      'fusion containment',
      'room-temp superconductors',
      'protein folding',
      'longevity',
      'carbon capture',
      'theory of everything',
      'TOE applications',
      'verified methods',
      'impossible problems',
      'breakthrough technology'
    ],
    breadcrumbs: [
      { name: 'Home', url: SITE_URL },
      { name: 'Moonshots', url: `${SITE_URL}/moonshots` }
    ]
  },

  verifyIndependently: {
    title: 'Verify Independently | Opoch',
    description: 'Don\'t trust our verification? Test the Theory of Everything yourself with your own AI. Step-by-step instructions.',
    image: DEFAULT_IMAGE,
    url: `${SITE_URL}/verify-independently`,
    type: 'website',
    keywords: [
      'verify independently',
      'independent verification',
      'test TOE yourself',
      'ChatGPT',
      'Claude',
      'Grok',
      'AI verification',
      'no cherry-picking',
      'scientific method',
      'theory of everything',
      'TOE',
      'transparent verification'
    ],
    breadcrumbs: [
      { name: 'Home', url: SITE_URL },
      { name: 'Verify Independently', url: `${SITE_URL}/verify-independently` }
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
    logo: `${SITE_METADATA.url}/Opoch%20Assets/Opoch%20Typelogo.png`,
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
