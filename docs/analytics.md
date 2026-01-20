# Analytics Event Documentation

This document describes all analytics events tracked across the Opoch website using Vercel Analytics.

## Overview

We use `@vercel/analytics` for event tracking. All events are defined in `src/lib/analytics.ts` with TypeScript types for safety.

### Usage

```typescript
import { track } from '../lib/analytics'

// Basic usage
track('event_name', { property: 'value' })

// Or use convenience functions
import { trackHomeCTAClicked } from '../lib/analytics'
trackHomeCTAClicked('hero_desktop')
```

---

## Event Naming Convention

Events follow this pattern: `{page}_{action}_{element}`

- **page**: `home`, `toe`, `falsify`, `verify`, `paper`, `simple_questions`, `nav`
- **action**: `clicked`, `viewed`, `expanded`, `collapsed`, `submitted`, `copied`
- **element**: specific UI element (e.g., `cta`, `section`, `link`)

---

## Homepage Events (Current - GITM/Reasoning)

Page: `/` (main homepage)

| Event Name | Description | Properties |
|------------|-------------|------------|
| `home_page_viewed` | User lands on homepage | - |
| `home_hero_cta_clicked` | "Try for Yourself" button clicked | `source: "hero_desktop" \| "mobile_sticky"` |
| `home_falsify_cta_clicked` | "Falsify Us" button clicked | `source: "hero_desktop" \| "mobile_sticky"` |
| `home_kernel_docs_clicked` | Announcement pill (0th Principle link) | - |
| `home_critpt_link_clicked` | "View benchmark" link | - |
| `home_proof_bundle_clicked` | "View Proof Bundle" link | - |
| `home_reasoning_prompt_clicked` | "The reasoning prompt" link | - |
| `home_apple_puzzles_clicked` | Apple Puzzles arxiv link | - |
| `home_solves_at_scale_clicked` | "solves at scale" link | - |
| `home_derivation_clicked` | "Read the Nothingness Derivation" link | - |
| `home_section_viewed` | Section scrolled into view | `section: "0th_principle" \| "comparison" \| "orc_engine" \| "vision"` |

---

## ToE Page Events (Old Homepage - Theory of Everything)

Page: `/toe`

| Event Name | Description | Properties |
|------------|-------------|------------|
| `toe_cta_clicked` | CTA button clicked | `source: "hero_test_yourself" \| "hero_falsify" \| "hero_ai_verification_{name}" \| "hero_paper_link"` |
| `toe_comparison_table_expanded` | Comparison table expanded | - |
| `toe_comparison_table_collapsed` | Comparison table collapsed | - |
| `toe_verify_clicked` | Verify link on a problem | `problem: string` |
| `toe_frontiers_expanded` | Frontier problems section expanded | - |
| `toe_frontiers_collapsed` | Frontier problems section collapsed | - |
| `toe_impact_expanded` | Impact section expanded | - |
| `toe_impact_collapsed` | Impact section collapsed | - |
| `toe_easter_egg_revealed` | Easter egg interaction | - |
| `toe_bounties_expanded` | Bounties section expanded | - |
| `toe_bounties_collapsed` | Bounties section collapsed | - |
| `toe_bounty_details_clicked` | Bounty details clicked | `problem: string` |
| `toe_bounty_chat_clicked` | Bounty chat button clicked | `problem: string, source: string` |

---

## Falsify Page Events

Page: `/falsify`

| Event Name | Description | Properties |
|------------|-------------|------------|
| `falsify_page_viewed` | User lands on falsify page | - |
| `falsify_section_expanded` | FAQ/section expanded | `section: string` |
| `falsify_section_collapsed` | FAQ/section collapsed | `section: string` |
| `falsify_email_clicked` | Email link clicked | - |
| `falsify_back_clicked` | Back button clicked | - |

---

## Verify Page Events

Page: `/verify`

| Event Name | Description | Properties |
|------------|-------------|------------|
| `verify_page_viewed` | User lands on verify page | - |
| `verify_prompt_copied` | Verification prompt copied | `source: string` |
| `verify_chat_clicked` | Chat link clicked | - |
| `verify_faq_expanded` | FAQ question expanded | `question: string` |
| `verify_faq_collapsed` | FAQ question collapsed | `question: string` |
| `verify_falsify_link_clicked` | Link to falsify page | `source: string` |
| `verify_back_clicked` | Back button clicked | - |

---

## Simple Questions Page Events

Page: `/questions`

| Event Name | Description | Properties |
|------------|-------------|------------|
| `simple_questions_page_viewed` | User lands on questions page | - |
| `simple_questions_section_expanded` | Section expanded | `section: string` |
| `simple_questions_section_collapsed` | Section collapsed | `section: string` |
| `simple_questions_chat_clicked` | Chat link clicked | - |
| `simple_questions_paper_clicked` | Paper link clicked | - |
| `simple_questions_back_clicked` | Back button clicked | - |

---

## Paper Page Events

Page: `/paper`

| Event Name | Description | Properties |
|------------|-------------|------------|
| `paper_page_viewed` | User lands on paper page | - |
| `paper_pdf_download_clicked` | PDF download clicked | - |
| `paper_pdf_view_clicked` | PDF view clicked | - |
| `paper_citation_copied` | Citation copied | - |
| `paper_related_link_clicked` | Related link clicked | `source: string` |
| `paper_back_clicked` | Back button clicked | - |

---

## Navigation Events

Global navigation

| Event Name | Description | Properties |
|------------|-------------|------------|
| `nav_link_clicked` | Nav link clicked | `link_name: string, current_page: string, is_mobile: boolean` |
| `nav_sign_in_clicked` | Sign in clicked | `current_page: string, is_mobile: boolean` |
| `nav_mobile_menu_opened` | Mobile menu opened | `current_page: string` |
| `nav_mobile_menu_closed` | Mobile menu closed | `current_page: string` |

---

## Global Events

| Event Name | Description | Properties |
|------------|-------------|------------|
| `lead_submitted` | Lead form submitted | `source: string` |
| `query_submitted` | Query submitted | `source: string` |
| `moonshot_viewed` | Moonshot detail viewed | `moonshot_slug: string` |
| `map_clicked` | Map interaction | - |
| `apply_modal_opened` | Apply modal opened | `source: string` |
| `auth_modal_opened` | Auth modal opened | - |
| `sign_in_attempted` | Sign in attempted | `provider: string` |
| `sign_in_completed` | Sign in completed | `provider: string` |
| `update_viewed` | Update page viewed | `update_slug: string` |
| `newsletter_subscribed` | Newsletter signup | `source: string` |

---

## Adding New Events

1. Add the event type to `AnalyticsEvent` union in `src/lib/analytics.ts`
2. Create a convenience function if needed
3. Document the event in this file
4. Implement tracking in the component

### Example:

```typescript
// In analytics.ts
export type AnalyticsEvent =
  | 'existing_event'
  | 'new_feature_clicked'  // Add new event

// Add convenience function
export const trackNewFeatureClicked = (source: string) =>
  track('new_feature_clicked', { source })

// In component
import { trackNewFeatureClicked } from '../lib/analytics'

const handleClick = () => {
  trackNewFeatureClicked('button_location')
}
```

---

## Viewing Analytics

Analytics data is available in the Vercel dashboard:
1. Go to your project in Vercel
2. Navigate to Analytics tab
3. View Custom Events section

Filter by:
- Event name
- Time range
- Properties
