import { track as vercelTrack } from '@vercel/analytics'

// Analytics event types
export type AnalyticsEvent = 
  | 'lead_submitted'
  | 'query_submitted'
  | 'cta_opened'
  | 'moonshot_viewed'
  | 'map_clicked'
  | 'apply_modal_opened'
  | 'auth_modal_opened'
  | 'sign_in_attempted'
  | 'sign_in_completed'
  | 'update_viewed'
  | 'newsletter_subscribed'

interface EventProperties {
  source?: string
  moonshot_slug?: string
  update_slug?: string
  provider?: string
  [key: string]: any
}

// Track custom events
export function track(event: AnalyticsEvent, properties?: EventProperties) {
  if (typeof window !== 'undefined') {
    vercelTrack(event, properties)
  }
  
  // Log in development
  if (import.meta.env.DEV) {
    console.log('📊 Analytics:', event, properties)
  }
}

// Convenience functions for common events
export const trackLeadSubmitted = (source: string) => 
  track('lead_submitted', { source })

export const trackQuerySubmitted = (source: string) => 
  track('query_submitted', { source })

export const trackCTAOpened = (source: string) => 
  track('cta_opened', { source })

export const trackMoonshotViewed = (slug: string) => 
  track('moonshot_viewed', { moonshot_slug: slug })

export const trackMapClicked = () => 
  track('map_clicked')

export const trackApplyModalOpened = (source: string) => 
  track('apply_modal_opened', { source })

export const trackAuthModalOpened = () => 
  track('auth_modal_opened')

export const trackSignInAttempted = (provider: string) => 
  track('sign_in_attempted', { provider })

export const trackSignInCompleted = (provider: string) => 
  track('sign_in_completed', { provider })

export const trackUpdateViewed = (slug: string) => 
  track('update_viewed', { update_slug: slug })

export const trackNewsletterSubscribed = (source: string) => 
  track('newsletter_subscribed', { source })