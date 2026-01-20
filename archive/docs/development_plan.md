# Opoch Development Plan

## Overview
This document outlines the complete development plan for Opoch's website, tracking implementation progress and next steps.

## Current Status
- **Phase 0**: ✅ Completed
- **Phase 1**: ✅ Completed
- **Phase 2**: 🚧 In Progress

---

## Phase 0 — Instant Polish ✅

### 0.1 Set the new OG image ✅
- Kept existing optimized og-image.jpg (62KB)
- Maintained current metadata as per user preference

### 0.2 Copy tweaks ✅
- Updated SF address to full format with Google Maps link
- Changed: "300 4th St, San Francisco" → "300 4th St, San Francisco, CA 94107"

### 0.3 RBT section link ✅
- Updated "Technical paper & verifier coming soon" to link to PDF
- Added: "Read the theoretical RBT paper →" with external link

### 0.4 "Verify a run (local)" → "Ask your query" ✅
- Replaced JSON verifier with AskBox component
- Added rotating prompt examples
- Saves queries to Supabase when configured

---

## Phase 1 — Lead Capture System ✅

### 1.1 Supabase schema ✅
- Created SQL schema file at `supabase/schema.sql`
- Defined `leads` and `queries` tables with RLS policies
- Added indexes for performance

### 1.2 Apply modal ✅
- Built ApplyModal component with:
  - Zod validation
  - React Hook Form
  - Success states
  - Source attribution tracking
- Fields: Name, Email, Phone, Designation, Organization, Reason, Research Query

### 1.3 "Ask your query" ✅
- Created AskBox component with rotating hints
- Graceful handling when Supabase not configured
- Integrated into RBT section

### 1.4 Integration Points ✅
- Nav bar Apply button (desktop + mobile)
- Hero Apply CTA
- Pricing section Apply button
- Footer CTA section
- Floating mobile Apply button

---

## Phase 2 — Moonshots (Hero Sales Surface) ✅

### 2.1 /moonshots landing
**Status**: Completed

**Requirements**:
- Headline + looping Lottie animation
- Grid of 15 moonshot cards with status chips
- Domain filters (physics/energy/bio/infra)
- Apply CTA with source tracking

**Implementation Plan**:
1. Install react-router-dom for routing ✅
2. Create moonshots data structure
3. Build MoonshotsGrid component
4. Add domain filtering logic
5. Integrate Lottie animations

### 2.2 Page template /moonshots/[slug]
**Status**: Completed

**Sections**:
1. Problem (stakes, constraints)
2. Our approach (high-level)
3. What we need from you
4. What you get
5. Verification
6. Potential impact
7. Work with us CTA + FAQ

### 2.3 Content model
**Status**: Completed

```typescript
export type Moonshot = {
  slug: string;
  title: string;
  domain: "physics"|"energy"|"bio"|"infra"|"robotics";
  status: "open"|"in-progress"|"partners";
  excerpt: string;
  heroLottie?: string;
  sections: { id:string; title:string; body:string }[];
  ctas?: { label:string; href:string }[];
};
```

### 2.4 Lottie integration
**Status**: Partially Complete
- Install lottie-react ✅
- Implement performance optimizations (pending)
- Add prefers-reduced-motion support (pending)

---

## Phase 3 — Email + Updates
**Status**: Completed ✓

### Goals
- Email provider integration (Resend/Postmark) ✅
- Email templates system ✅
- Updates page with MDX support ✅
- Email tracking database ✅

### Tasks
- [x] Choose and integrate email provider (Resend)
- [x] Create email templates (5 templates created)
- [x] Build /updates page with newsletter signup
- [x] Set up email_sent tracking table
- [x] Create admin dashboard for email monitoring

---

## Phase 4 — Auth (Supabase)

### Goals
- SSO with Google/GitHub
- User profiles linked to leads
- Gated dashboard access

### Tasks
- [ ] Configure Supabase Auth
- [ ] Create profiles table
- [ ] Build auth UI components
- [ ] Implement protected routes

---

## Phase 5 — Analytics, SEO, Accessibility

### Analytics Events
- [ ] lead_submitted
- [ ] query_submitted
- [ ] cta_opened
- [ ] moonshot_viewed
- [ ] map_clicked

### SEO
- [ ] Generate sitemap.xml
- [ ] Add structured data
- [ ] Meta tag optimization

### Accessibility
- [ ] Keyboard navigation
- [ ] ARIA labels
- [ ] Color contrast audit
- [ ] Screen reader testing

---

## Phase 6 — Performance & Reliability

### Tasks
- [ ] Image optimization
- [ ] Font preloading
- [ ] CSP headers
- [ ] Rate limiting
- [ ] Error boundaries

---

## Phase 7 — Ops & BD

### Tasks
- [ ] Pipedrive integration
- [ ] Analytics dashboard
- [ ] Press kit creation
- [ ] Team documentation

---

## Technical Debt & Issues

### Current Issues
1. **Vercel Deployment Black Screen**
   - Fixed: Supabase client now handles missing env vars gracefully
   - Site works without database connection

2. **Development Server Port**
   - Fixed: Killed process on port 3000
   - Dev server runs on port 3000

### Upcoming Considerations
- MDX integration for Vite (not Next.js)
- Client-side routing with React Router
- Static site generation limitations
- Bundle size optimization

---

## Next Steps

1. **Complete Phase 2.1**: Build moonshots landing page
2. **Set up routing**: Configure React Router for SPA
3. **Create moonshot data**: Define 15 moonshots with content
4. **Build individual pages**: Implement moonshot detail views

---

## Dependencies Added

```json
{
  "@hookform/resolvers": "^5.2.1",
  "@radix-ui/react-dialog": "^1.1.15",
  "@supabase/supabase-js": "^2.56.0",
  "@vercel/analytics": "^1.5.0",
  "@vercel/speed-insights": "^1.2.0",
  "lottie-react": "^2.4.1",
  "react-hook-form": "^7.62.0",
  "react-router-dom": "^7.8.2",
  "zod": "^4.1.4"
}
```