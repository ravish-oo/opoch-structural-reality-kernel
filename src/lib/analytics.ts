import { track as vercelTrack } from '@vercel/analytics'

// Analytics event types
export type AnalyticsEvent =
  // Global events
  | 'lead_submitted'
  | 'query_submitted'
  | 'moonshot_viewed'
  | 'map_clicked'
  | 'apply_modal_opened'
  | 'auth_modal_opened'
  | 'sign_in_attempted'
  | 'sign_in_completed'
  | 'update_viewed'
  | 'newsletter_subscribed'
  // Homepage events (GITM/Reasoning page - current homepage)
  | 'home_page_viewed'
  | 'home_hero_cta_clicked'
  | 'home_falsify_cta_clicked'
  | 'home_kernel_docs_clicked'
  | 'home_critpt_link_clicked'
  | 'home_proof_bundle_clicked'
  | 'home_reasoning_prompt_clicked'
  | 'home_apple_puzzles_clicked'
  | 'home_solves_at_scale_clicked'
  | 'home_derivation_clicked'
  | 'home_section_viewed'
  // ToE page events (Theory of Everything - old homepage at /toe)
  | 'toe_cta_clicked'
  | 'toe_comparison_table_expanded'
  | 'toe_comparison_table_collapsed'
  | 'toe_verify_clicked'
  | 'toe_frontiers_expanded'
  | 'toe_frontiers_collapsed'
  | 'toe_impact_expanded'
  | 'toe_impact_collapsed'
  | 'toe_easter_egg_revealed'
  | 'toe_bounties_expanded'
  | 'toe_bounties_collapsed'
  | 'toe_bounty_details_clicked'
  | 'toe_bounty_chat_clicked'
  // Falsify page events
  | 'falsify_page_viewed'
  | 'falsify_section_expanded'
  | 'falsify_section_collapsed'
  | 'falsify_email_clicked'
  | 'falsify_back_clicked'
  // Verify page events
  | 'verify_page_viewed'
  | 'verify_prompt_copied'
  | 'verify_chat_clicked'
  | 'verify_faq_expanded'
  | 'verify_faq_collapsed'
  | 'verify_falsify_link_clicked'
  | 'verify_back_clicked'
  // Simple Questions page events
  | 'simple_questions_page_viewed'
  | 'simple_questions_section_expanded'
  | 'simple_questions_section_collapsed'
  | 'simple_questions_chat_clicked'
  | 'simple_questions_paper_clicked'
  | 'simple_questions_back_clicked'
  // Paper page events
  | 'paper_page_viewed'
  | 'paper_pdf_download_clicked'
  | 'paper_pdf_view_clicked'
  | 'paper_citation_copied'
  | 'paper_related_link_clicked'
  | 'paper_back_clicked'
  // Navigation events
  | 'nav_link_clicked'
  | 'nav_sign_in_clicked'
  | 'nav_mobile_menu_opened'
  | 'nav_mobile_menu_closed'

interface EventProperties {
  source?: string
  moonshot_slug?: string
  update_slug?: string
  provider?: string
  problem?: string
  section?: string
  [key: string]: string | number | boolean | undefined
}

// Track custom events
export function track(event: AnalyticsEvent, properties?: EventProperties) {
  if (typeof window !== 'undefined') {
    // Filter out undefined values for vercel analytics
    const filteredProperties = properties
      ? (Object.fromEntries(
          Object.entries(properties).filter(([, v]) => v !== undefined)
        ) as Record<string, string | number | boolean>)
      : undefined
    vercelTrack(event, filteredProperties)
  }

  // Log in development
  if (import.meta.env.DEV) {
    console.log('Analytics:', event, properties)
  }
}

// ============================================
// Global Events
// ============================================

export const trackLeadSubmitted = (source: string) =>
  track('lead_submitted', { source })

export const trackQuerySubmitted = (source: string) =>
  track('query_submitted', { source })

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

// ============================================
// Homepage Events (GITM/Reasoning - current homepage)
// ============================================

export const trackHomePageViewed = () =>
  track('home_page_viewed')

export const trackHomeHeroCTAClicked = (source: 'hero_desktop' | 'mobile_sticky') =>
  track('home_hero_cta_clicked', { source })

export const trackHomeFalsifyCTAClicked = (source: 'hero_desktop' | 'mobile_sticky') =>
  track('home_falsify_cta_clicked', { source })

export const trackHomeKernelDocsClicked = () =>
  track('home_kernel_docs_clicked')

export const trackHomeCritPtLinkClicked = () =>
  track('home_critpt_link_clicked')

export const trackHomeProofBundleClicked = () =>
  track('home_proof_bundle_clicked')

export const trackHomeReasoningPromptClicked = () =>
  track('home_reasoning_prompt_clicked')

export const trackHomeApplePuzzlesClicked = () =>
  track('home_apple_puzzles_clicked')

export const trackHomeSolvesAtScaleClicked = () =>
  track('home_solves_at_scale_clicked')

export const trackHomeDerivationClicked = () =>
  track('home_derivation_clicked')

export const trackHomeSectionViewed = (section: '0th_principle' | 'comparison' | 'orc_engine' | 'vision') =>
  track('home_section_viewed', { section })

// ============================================
// ToE Page Events (Theory of Everything - /toe)
// ============================================

export const trackToeCTAClicked = (source: string) =>
  track('toe_cta_clicked', { source })

export const trackToeComparisonTableExpanded = () =>
  track('toe_comparison_table_expanded')

export const trackToeComparisonTableCollapsed = () =>
  track('toe_comparison_table_collapsed')

export const trackToeVerifyClicked = (problem: string) =>
  track('toe_verify_clicked', { problem })

export const trackToeFrontiersExpanded = () =>
  track('toe_frontiers_expanded')

export const trackToeFrontiersCollapsed = () =>
  track('toe_frontiers_collapsed')

export const trackToeImpactExpanded = () =>
  track('toe_impact_expanded')

export const trackToeImpactCollapsed = () =>
  track('toe_impact_collapsed')

export const trackToeEasterEggRevealed = () =>
  track('toe_easter_egg_revealed')

export const trackToeBountiesExpanded = () =>
  track('toe_bounties_expanded')

export const trackToeBountiesCollapsed = () =>
  track('toe_bounties_collapsed')

export const trackToeBountyDetailsClicked = (bountyId: string) =>
  track('toe_bounty_details_clicked', { problem: bountyId })

export const trackToeBountyChatClicked = (bountyId: string, bountyName: string) =>
  track('toe_bounty_chat_clicked', { problem: bountyId, source: bountyName })

// ============================================
// Falsify Page Events
// ============================================

export const trackFalsifyPageViewed = () =>
  track('falsify_page_viewed')

export const trackFalsifySectionExpanded = (sectionId: string) =>
  track('falsify_section_expanded', { section: sectionId })

export const trackFalsifySectionCollapsed = (sectionId: string) =>
  track('falsify_section_collapsed', { section: sectionId })

export const trackFalsifyEmailClicked = () =>
  track('falsify_email_clicked')

export const trackFalsifyBackClicked = () =>
  track('falsify_back_clicked')

// ============================================
// Verify Page Events
// ============================================

export const trackVerifyPageViewed = () =>
  track('verify_page_viewed')

export const trackVerifyPromptCopied = (source: string) =>
  track('verify_prompt_copied', { source })

export const trackVerifyChatClicked = () =>
  track('verify_chat_clicked')

export const trackVerifyFAQExpanded = (question: string) =>
  track('verify_faq_expanded', { question })

export const trackVerifyFAQCollapsed = (question: string) =>
  track('verify_faq_collapsed', { question })

export const trackVerifyFalsifyLinkClicked = (source: string) =>
  track('verify_falsify_link_clicked', { source })

export const trackVerifyBackClicked = () =>
  track('verify_back_clicked')

// ============================================
// Simple Questions Page Events
// ============================================

export const trackSimpleQuestionsPageViewed = () =>
  track('simple_questions_page_viewed')

export const trackSimpleQuestionsSectionExpanded = (sectionId: string) =>
  track('simple_questions_section_expanded', { section: sectionId })

export const trackSimpleQuestionsSectionCollapsed = (sectionId: string) =>
  track('simple_questions_section_collapsed', { section: sectionId })

export const trackSimpleQuestionsChatClicked = () =>
  track('simple_questions_chat_clicked')

export const trackSimpleQuestionsPaperClicked = () =>
  track('simple_questions_paper_clicked')

export const trackSimpleQuestionsBackClicked = () =>
  track('simple_questions_back_clicked')

// ============================================
// Paper Page Events
// ============================================

export const trackPaperPageViewed = () =>
  track('paper_page_viewed')

export const trackPaperPDFDownloadClicked = () =>
  track('paper_pdf_download_clicked')

export const trackPaperPDFViewClicked = () =>
  track('paper_pdf_view_clicked')

export const trackPaperCitationCopied = () =>
  track('paper_citation_copied')

export const trackPaperRelatedLinkClicked = (destination: string) =>
  track('paper_related_link_clicked', { source: destination })

export const trackPaperBackClicked = () =>
  track('paper_back_clicked')

// ============================================
// Navigation Events
// ============================================

export const trackNavLinkClicked = (linkName: string, currentPage: string, isMobile: boolean = false) =>
  track('nav_link_clicked', {
    link_name: linkName,
    current_page: currentPage,
    is_mobile: isMobile
  })

export const trackNavSignInClicked = (currentPage: string, isMobile: boolean = false) =>
  track('nav_sign_in_clicked', {
    current_page: currentPage,
    is_mobile: isMobile
  })

export const trackNavMobileMenuOpened = (currentPage: string) =>
  track('nav_mobile_menu_opened', { current_page: currentPage })

export const trackNavMobileMenuClosed = (currentPage: string) =>
  track('nav_mobile_menu_closed', { current_page: currentPage })
