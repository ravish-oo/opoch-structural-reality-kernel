import { Helmet } from 'react-helmet-async'
import { 
  SITE_METADATA, 
  getOrganizationStructuredData, 
  type PageMetadata 
} from '../lib/metadata'

interface SEOHeadProps extends Partial<PageMetadata> {
  metadata?: PageMetadata
}

const DEFAULT_TITLE = SITE_METADATA.baseTitle
const DEFAULT_DESCRIPTION = SITE_METADATA.description
const DEFAULT_IMAGE = SITE_METADATA.image
const SITE_URL = SITE_METADATA.url

export default function SEOHead({ 
  metadata,
  title,
  description,
  image,
  url,
  type = 'website',
  keywords,
  article,
  breadcrumbs
}: SEOHeadProps) {
  // Use provided metadata or fallback to individual props
  const meta = metadata || {
    title: title || DEFAULT_TITLE,
    description: description || DEFAULT_DESCRIPTION,
    image: image || DEFAULT_IMAGE,
    url: url || SITE_URL,
    type,
    keywords,
    article,
    breadcrumbs
  }
  
  // Generate full title with proper branding
  const fullTitle = meta.title === SITE_METADATA.baseTitle ? meta.title : 
                   meta.title.includes('Opoch') ? meta.title : `${meta.title} | Opoch`
  
  // Get organization structured data from metadata engine
  const structuredData = getOrganizationStructuredData()

  // Generate article structured data if this is an article
  const articleStructuredData = meta.article ? {
    '@context': 'https://schema.org',
    '@type': 'Article',
    headline: fullTitle,
    description: meta.description,
    image: meta.image,
    datePublished: meta.article.publishedTime,
    dateModified: meta.article.publishedTime,
    author: {
      '@type': 'Organization',
      name: meta.article.author || 'Opoch Team',
      url: SITE_URL
    },
    publisher: {
      '@type': 'Organization',
      name: SITE_METADATA.siteName,
      logo: {
        '@type': 'ImageObject',
        url: `${SITE_URL}/Opoch Assets/Opoch Logo.png`,
        width: 600,
        height: 200
      },
      url: SITE_URL
    },
    mainEntityOfPage: meta.url,
    articleSection: meta.article.section || 'Technology',
    keywords: meta.article.tags?.join(', '),
    inLanguage: 'en-US'
  } : null

  // Generate breadcrumb structured data if breadcrumbs exist
  const breadcrumbStructuredData = meta.breadcrumbs ? {
    '@context': 'https://schema.org',
    '@type': 'BreadcrumbList',
    itemListElement: meta.breadcrumbs.map((crumb, index) => ({
      '@type': 'ListItem',
      position: index + 1,
      name: crumb.name,
      item: crumb.url
    }))
  } : null

  return (
    <Helmet>
      {/* Basic Meta Tags */}
      <title>{fullTitle}</title>
      <meta name="description" content={meta.description} />
      <meta name="author" content="Opoch" />
      <link rel="canonical" href={meta.url} />
      
      {/* Keywords */}
      {meta.keywords && meta.keywords.length > 0 && (
        <meta name="keywords" content={meta.keywords.join(', ')} />
      )}

      {/* Open Graph / Facebook Tags */}
      <meta property="og:type" content={meta.type} />
      <meta property="og:title" content={fullTitle} />
      <meta property="og:description" content={meta.description} />
      <meta property="og:image" content={meta.image} />
      <meta property="og:image:secure_url" content={meta.image} />
      <meta property="og:image:type" content="image/jpeg" />
      <meta property="og:image:width" content="1200" />
      <meta property="og:image:height" content="630" />
      <meta property="og:image:alt" content={`${fullTitle} - ${SITE_METADATA.tagline}`} />
      <meta property="og:url" content={meta.url} />
      <meta property="og:site_name" content={SITE_METADATA.siteName} />
      <meta property="og:locale" content="en_US" />

      {/* Twitter Card Tags */}
      <meta name="twitter:card" content="summary_large_image" />
      <meta name="twitter:title" content={fullTitle} />
      <meta name="twitter:description" content={meta.description} />
      <meta name="twitter:image" content={meta.image} />
      <meta name="twitter:site" content={SITE_METADATA.contact.twitter} />
      <meta name="twitter:creator" content={SITE_METADATA.contact.twitter} />

      {/* Article specific tags */}
      {meta.article && (
        <>
          <meta property="article:published_time" content={meta.article.publishedTime} />
          <meta property="article:author" content={meta.article.author || 'Opoch Team'} />
          <meta property="article:section" content={meta.article.section || 'Technology'} />
          {meta.article.tags?.map(tag => (
            <meta key={tag} property="article:tag" content={tag} />
          ))}
        </>
      )}

      {/* Structured Data - Organization */}
      <script type="application/ld+json">
        {JSON.stringify(structuredData)}
      </script>
      
      {/* Structured Data - Article */}
      {articleStructuredData && (
        <script type="application/ld+json">
          {JSON.stringify(articleStructuredData)}
        </script>
      )}

      {/* Structured Data - Breadcrumbs */}
      {breadcrumbStructuredData && (
        <script type="application/ld+json">
          {JSON.stringify(breadcrumbStructuredData)}
        </script>
      )}

      {/* Additional SEO Tags */}
      <meta name="robots" content="index, follow, max-image-preview:large, max-snippet:-1, max-video-preview:-1" />
      <meta name="googlebot" content="index, follow, max-video-preview:-1, max-image-preview:large, max-snippet:-1" />
      <meta name="viewport" content="width=device-width, initial-scale=1" />
      <meta httpEquiv="Content-Type" content="text/html; charset=utf-8" />
      <meta name="language" content="English" />
      <meta name="revisit-after" content="7 days" />
      <meta name="theme-color" content="#000000" />
      
      {/* Additional Meta Tags for Better Indexing */}
      <meta name="distribution" content="global" />
      <meta name="rating" content="general" />
      <meta name="referrer" content="no-referrer-when-downgrade" />
    </Helmet>
  )
}