# Homepage V2 - Implementation Clarity & Anchor Document

**Purpose**: This document anchors all technical context, implementation decisions, and execution strategy for building Homepage V2. Reference this to stay aligned and avoid drift.

**Last Updated**: 2025-10-22

---

## 1. TECHNICAL CONTEXT

### 1.1 Repository Setup

**Current State**:
- **Repo**: `/Users/ravishq/code/opoch-website`
- **Tech Stack**: Vite + React 18 + TypeScript + Tailwind CSS
- **Framework**: SPA with React Router v6
- **Deployment**: Vercel (automatic on push to `main`)
- **Domain**: www.opoch.com
- **Dev Server**: Port 5173 (or 5174 if 5173 occupied)

**Related Repositories**:
- **Chat App**: `/Users/ravishq/code/opoch-chat-test`
  - Tech: Next.js 15 + React 19
  - Domain: chat.opoch.com
  - See: `/Users/ravishq/code/opoch-chat-test/docs/README.md`

---

### 1.2 Authentication & Cookie Sharing

**Supabase Auth Setup** (Shared across subdomains):
- **Provider**: Supabase Auth
- **Methods**: Google OAuth, GitHub OAuth
- **Cookie Domain**: `.opoch.com` (leading dot enables subdomain sharing)
- **Library**: `@supabase/ssr` with `createBrowserClient`

**How It Works**:
1. User signs in on www.opoch.com
2. Supabase stores session in cookies with `domain: '.opoch.com'`
3. Cookies automatically accessible to chat.opoch.com
4. No re-login needed when navigating between subdomains

**Key Files**:
- **Website Auth**: `src/lib/supabase.ts`, `src/contexts/AuthContext.tsx`
- **Chat Middleware**: `/Users/ravishq/code/opoch-chat-test/middleware.ts`

**Documentation**:
- Full details: `/Users/ravishq/code/opoch-chat-test/docs/02-TWO-APP-ARCHITECTURE.md`

**Current Setup**: ✅ **WORKING** - No changes needed for Homepage V2

---

### 1.3 Chat Integration

**Chat Application**:
- **URL**: https://chat.opoch.com
- **Function**: AI chat powered by GPT-5 with UNIVERSE-TECH vector store
- **Vector Store ID**: `vs_68f191698c10819183cad7bec284c47a`
- **Integration**: Already complete, cookies shared automatically

**Primary CTA Flow**:
1. User clicks "Test It Yourself" on Homepage V2
2. If not authenticated → Show login modal (existing `AuthModal.tsx`)
3. After auth → Redirect to `https://chat.opoch.com`
4. Chat app validates session via shared cookies

**No Additional Work Required**: Auth/chat integration is ready to use

---

### 1.4 Available Resources

**Vercel Blob Storage**:
- **Purpose**: Store/serve assets, images, files
- **Access**: Available via Vercel CLI or API
- **Use Cases**: AI logos, OG images, downloadable packs, etc.
- **Documentation**: https://vercel.com/docs/storage/vercel-blob

**Analytics**:
- **Vercel Analytics**: Already integrated (`src/lib/analytics.ts`)
- **Custom Events**: Use `track(event, properties)` function
- **Available Events**: See `AnalyticsEvent` type in `src/lib/analytics.ts`

**Assets**:
- **Location**: `/public/Opoch Assets/`
- **Existing**: Logos, OG images, favicon
- **Format**: SVG, PNG

---

## 2. IMPLEMENTATION STRATEGY

### 2.1 Build Approach

**Core Principle**: Build all pages first, swap routes at the end

**Steps**:
1. ✅ **Create clean structure** (folders, pages, components)
2. 🔄 **Build sections progressively** (Hero → Section 2 → Section 3...)
3. ⏳ **Test each section independently** (screenshot, iterate)
4. ⏳ **Integrate all sections into Homepage V2**
5. ⏳ **QA complete homepage** at `/home-v2`
6. ⏳ **Swap routes** when ready:
   - `/ ` → HomeV2 (new)
   - `/classic` → OpochLanding (old, for rollback)

**Why This Approach**:
- Existing site stays functional throughout development
- Can test Homepage V2 at `/home-v2` without affecting production
- Easy rollback if needed
- Progressive iteration without risk

---

### 2.2 Homepage V2 Structure

**Reference Documents**:
- **High-Level**: `docs/homepage-v2/homepage-structure.md`
- **Section Specs**: `docs/homepage-v2/SEC-01-HERO.md`, etc. (as created)

**8 Main Sections**:
1. **HERO** - Bold claim, Test It Yourself CTA, AI verification strip
2. **COMPARISON TABLE** - Objective comparison (String Theory vs Ω, etc.)
3. **VERIFY WITH YOUR OWN AI** - Ultimate skeptic hook
4. **BOUNTIES** - Real money, credibility signal
5. **FALSIFY RECORD** - Transparency, challenge invitation
6. **IMMEDIATE IMPACT** - Practical value showcase
7. **FUTURE PROGRESSION** - Vision timeline (Kardashev → easter eggs)
8. **FOOTER** - Standard nav, acknowledgments, legal

**Build Order** (Phased):
- **Phase 1**: Hero + Comparison Table ← **START HERE**
- **Phase 2**: Verify Yourself + Bounties
- **Phase 3**: Falsify Record
- **Phase 4**: Impact + Future sections
- **Phase 5**: Polish, optimize, test

---

### 2.3 Design Philosophy

**Target Audience**:
- Deep tech founders, VCs, researchers, mathematicians, physicists
- High IQ, technically literate, skeptical by nature
- Need objective proof, not marketing claims

**Design Principles**:
- **Visual**: Clean, minimal, high contrast, sober "lab-grade" aesthetic
- **Copy**: Concise, technical but accessible, action-oriented
- **Interactions**: Fast, responsive, obvious affordances
- **Accessibility**: WCAG AA minimum, keyboard navigable

**Brand Colors** (from `tailwind.config.js`):
- **Primary**: Black background
- **Text**: White (primary), white/60 (secondary), white/40 (tertiary)
- **Accent**: Opoch Cyan #1CE4E6
- **Additional**: Blue shades, tertiary colors (green, pink, yellow, orange, purple)

**Typography**:
- **Font Stack**: Gotham HTF, Whitney, Verdana, system-ui
- **Hierarchy**: Clear H1/H2/body/caption distinctions
- **Readability**: Line height 1.4-1.6, sufficient spacing

---

## 3. FILE ORGANIZATION

### 3.1 New Directory Structure

```
src/
├── components/
│   ├── v2/                    # Homepage V2 specific components
│   │   ├── Nav.tsx           # New navigation for V2
│   │   ├── Footer.tsx        # Footer for V2
│   │   └── ...
│   └── ui/                   # Existing shared UI components
│       ├── button.tsx
│       ├── card.tsx
│       └── ...
├── sections/
│   └── v2/                   # Homepage V2 sections
│       ├── Hero.tsx
│       ├── ComparisonTable.tsx
│       ├── VerifyYourself.tsx
│       ├── Bounties.tsx
│       ├── FalsifyRecord.tsx
│       ├── ImmediateImpact.tsx
│       ├── FutureProgression.tsx
│       └── constants.ts      # All copy text
├── pages/
│   ├── HomeV2.tsx            # Main Homepage V2 page
│   ├── OpochLanding.tsx      # Current homepage (will become /classic)
│   └── ...
└── data/
    └── v2/                   # Homepage V2 data
        ├── comparison-table.ts
        ├── bounties.ts
        ├── falsify-stats.ts
        └── ...
```

**Naming Convention**:
- Use `v2/` subfolder for Homepage V2 specific code
- Keeps existing code untouched
- Clear separation between old and new

---

### 3.2 Placeholder Strategy

**For Missing Assets**:
- **AI Logos**: Use text placeholders in bordered boxes (e.g., "GPT-5")
- **Verification URLs**: Use `#` placeholders, add real URLs later
- **Content**: Use placeholder text from specs, mark as "[TBD]" if uncertain

**For Incomplete Sections**:
- Build structure/layout first
- Use realistic placeholder content
- Screenshot for review
- Iterate based on feedback

**Philosophy**: Get structure/spacing/hierarchy right with placeholders, then refine content

---

## 4. DECISION LOG

### 4.1 Key Technical Decisions

**Decision**: Use existing brand colors (black, white, cyan)
**Rationale**: Matches current site, "sober lab-grade" aesthetic, no need for new color system
**Status**: ✅ Confirmed

**Decision**: Build at `/home-v2`, swap routes at the end
**Rationale**: Zero risk to existing site, easy rollback, progressive testing
**Status**: ✅ Confirmed

**Decision**: Create new `v2/` folders for components/sections
**Rationale**: Clean separation, no conflicts with existing code
**Status**: ✅ Confirmed

**Decision**: Reuse existing UI components where possible
**Rationale**: Consistency, faster development, proven components
**Status**: ✅ Confirmed

**Decision**: Progressive section-by-section build
**Rationale**: Iterative feedback, manageable scope, clear milestones
**Status**: ✅ Confirmed

---

### 4.2 What We're NOT Changing

✅ **Keep Existing**:
- Current authentication setup (Supabase, cookies, AuthContext)
- Chat integration (chat.opoch.com with shared cookies)
- Existing routes (`/moonshots`, `/updates`, `/admin/*`, etc.)
- Current Tailwind config and theme
- Analytics setup
- Deployment workflow

❌ **Do Not Touch**:
- `/src/OpochLanding.tsx` (current homepage - becomes `/classic` later)
- `/src/components/landing/*` (current landing components)
- `/src/lib/supabase.ts` (auth setup)
- `/src/contexts/AuthContext.tsx` (unless absolutely necessary)

---

## 5. EXECUTION WORKFLOW

### 5.1 Per-Section Process

For each new section:

1. **Read Spec** - Review `SEC-XX-[NAME].md` thoroughly
2. **Create Constants** - Add copy to `src/sections/v2/constants.ts`
3. **Build Component** - Create `src/sections/v2/[Name].tsx`
4. **Screenshot** - Use Chrome MCP to capture result
5. **Review** - Get feedback, iterate if needed
6. **Integrate** - Add to `HomeV2.tsx`
7. **Test** - Verify functionality, links, responsive behavior
8. **Document** - Update relevant docs if needed

---

### 5.2 Quality Checkpoints

**Before Moving to Next Section**:
- [ ] All copy matches spec exactly
- [ ] Spacing/sizing matches spec (desktop + mobile)
- [ ] All links work (even if placeholders)
- [ ] Responsive behavior correct
- [ ] Accessibility: semantic HTML, ARIA labels, keyboard nav
- [ ] No console errors
- [ ] Screenshot reviewed and approved
- [ ] **Analytics**: Event tracking added for key interactions
- [ ] **Performance**: Section lazy loaded, React.memo/useCallback applied, Suspense boundary added
- [ ] **SEO**: Metadata configured (if section-specific metadata needed)

**Before Final Launch**:
- [ ] All 8 sections complete
- [ ] End-to-end flow tested
- [ ] All placeholder content replaced with real content
- [ ] All verification URLs added
- [ ] Cross-browser testing
- [ ] Mobile testing (iOS + Android)
- [ ] Lighthouse scores: 95/95/100/100 (perf/a11y/best practices/SEO)
- [ ] **Analytics**: All events firing correctly, verified in Vercel Analytics dashboard
- [ ] **Performance**: 100/100 Vercel Speed Insights score maintained
- [ ] **SEO**: Homepage V2 metadata finalized and tested (og:image, meta description, structured data)

---

## 6. PERFORMANCE OPTIMIZATION

### 6.1 Current Performance Achievement: 100/100 Speed Insights

**Verified Status**: All current pages (homepage, moonshots, updates) achieve **100/100** on Vercel Speed Insights.

**How It's Achieved**:

The current site uses a comprehensive performance strategy that Homepage V2 MUST maintain:

#### 1. **Vercel Analytics & Speed Insights Integration**
   - **Implementation**: `src/main.tsx:1-2, 10-11`
   ```typescript
   import { Analytics } from '@vercel/analytics/react'
   import { SpeedInsights } from '@vercel/speed-insights/react'

   <Analytics />
   <SpeedInsights />
   ```
   - **Purpose**: Automatic performance tracking, real-time insights
   - **Homepage V2**: Include both components in HomeV2.tsx

#### 2. **Performance Monitoring Initialization**
   - **Implementation**: `src/main.tsx:8, 14`
   ```typescript
   import { initPerformance } from './lib/performance'
   initPerformance()
   ```
   - **What it does** (`src/lib/performance.ts`):
     - Measures page load metrics (DNS, TCP, TTFB, DOM load, full load)
     - Monitors Web Vitals (CLS, INP, FCP, LCP, TTFB) via `web-vitals` library
     - Adds resource hints (DNS prefetch, preconnect) for external resources
     - Creates lazy load observer for images/components
   - **Homepage V2**: Call initPerformance() on mount (already in main.tsx, no change needed)

#### 3. **Code Splitting with React.lazy()**
   - **Implementation**: `src/OpochLanding.tsx:16-26`
   ```typescript
   // Lazy load components for better performance
   const Nav = lazy(() => import("./components/landing/Nav"))
   const Hero = lazy(() => import("./components/landing/Hero"))
   const Outcomes = lazy(() => import("./components/landing/Outcomes"))
   // ... all major sections lazy loaded

   // Lazy load modals only when needed
   const ApplyModal = lazy(() => import("./components/ApplyModal"))
   const AuthModal = lazy(() => import("./components/AuthModal"))
   ```
   - **Benefits**: Reduces initial bundle size, faster First Contentful Paint (FCP)
   - **Homepage V2**: Lazy load ALL sections (Hero, ComparisonTable, Verify, Bounties, etc.)

#### 4. **React.Suspense with Loading States**
   - **Implementation**: `src/OpochLanding.tsx:29-33, 61-107`
   ```typescript
   const SectionLoader = () => (
     <div className="flex justify-center items-center py-24">
       <div className="animate-pulse text-white/40">Loading...</div>
     </div>
   )

   <Suspense fallback={<SectionLoader />}>
     <Hero openApply={() => openApplyModal("hero")} />
   </Suspense>
   ```
   - **Benefits**: Prevents layout shift (CLS), graceful loading UX
   - **Homepage V2**: Wrap all lazy-loaded sections in Suspense

#### 5. **React.memo for Inline Components**
   - **Implementation**: `src/OpochLanding.tsx:122, 154, 181, 251`
   ```typescript
   const Pricing = React.memo(({ openApply }: { openApply: () => void }) => {
     // ... component logic
   })
   ```
   - **Benefits**: Prevents unnecessary re-renders, optimizes runtime performance
   - **Homepage V2**: Use React.memo for smaller inline sections

#### 6. **useCallback for Event Handlers**
   - **Implementation**: `src/OpochLanding.tsx:49-53`
   ```typescript
   const openApplyModal = useCallback((source: string) => {
     setApplySource(source)
     setApplyModalOpen(true)
     trackApplyModalOpened(source)
   }, [])
   ```
   - **Benefits**: Prevents function recreation, reduces child re-renders
   - **Homepage V2**: Use useCallback for all callbacks passed to child components

#### 7. **Resource Hints**
   - **Implementation**: `src/lib/performance.ts:50-60`
   ```typescript
   function addResourceHints() {
     const hints = [
       { rel: 'dns-prefetch', href: 'https://fonts.googleapis.com' },
       { rel: 'preconnect', href: 'https://fonts.googleapis.com', crossorigin: true }
     ]
     // ... adds to document head
   }
   ```
   - **Benefits**: Early DNS resolution, faster external resource loading
   - **Homepage V2**: Add resource hints for any external resources (chat.opoch.com, AI logos if hosted externally)

#### 8. **Lazy Load Observer for Images**
   - **Implementation**: `src/lib/performance.ts:39-48`
   ```typescript
   export function createLazyLoadObserver(callback: (entry: IntersectionObserverEntry) => void) {
     return new IntersectionObserver(
       (entries) => { entries.forEach(callback) },
       { rootMargin: '50px', threshold: 0.01 }
     )
   }
   ```
   - **Benefits**: Images load only when near viewport (50px buffer)
   - **Homepage V2**: Use for all images, especially in Comparison Table, Bounties, Impact sections

#### 9. **Vite Build Optimization**
   - **Automatic**: Tree shaking, minification, code splitting (built-in)
   - **Config**: `vite.config.ts` (no special config needed, defaults are optimal)
   - **Homepage V2**: No changes needed, Vite handles it automatically

---

### 6.2 Homepage V2 Performance Strategy

**Target**: Maintain **100/100** Vercel Speed Insights score

**Implementation Checklist**:

**Required for Every Section**:
- [ ] Section component exported via React.lazy()
- [ ] Section wrapped in Suspense with fallback
- [ ] All images use lazy loading (Intersection Observer)
- [ ] Event handlers wrapped in useCallback
- [ ] Smaller inline components use React.memo

**Required for Overall Homepage V2**:
- [ ] Analytics and SpeedInsights components included
- [ ] initPerformance() called (already in main.tsx)
- [ ] All routes lazy loaded in App.tsx
- [ ] Resource hints added for external resources
- [ ] No render-blocking resources in <head>
- [ ] Images optimized (WebP preferred, fallback to PNG/SVG)
- [ ] Proper image sizing (width/height attributes)
- [ ] Font loading optimized (font-display: swap)

**Performance Testing During Development**:
1. Run Lighthouse audit after each section: `npm run build` then test production build
2. Check Web Vitals in browser console (logged by performance.ts)
3. Verify Speed Insights in Vercel dashboard after deployment
4. Target metrics:
   - **LCP** (Largest Contentful Paint): < 2.5s
   - **FCP** (First Contentful Paint): < 1.8s
   - **CLS** (Cumulative Layout Shift): < 0.1
   - **INP** (Interaction to Next Paint): < 200ms
   - **TTFB** (Time to First Byte): < 800ms

---

### 6.3 Performance Anti-Patterns to Avoid

**❌ DO NOT**:
- Import sections directly (always use React.lazy)
- Render all sections at once without Suspense
- Load images eagerly (use Intersection Observer)
- Create inline functions in render (use useCallback)
- Use large unoptimized images (compress and use modern formats)
- Add external scripts without resource hints
- Block rendering with synchronous operations
- Skip React.memo for components that re-render frequently

**✅ DO**:
- Lazy load everything below the fold
- Use Suspense boundaries for graceful loading
- Implement lazy loading for all images
- Memoize callbacks and components
- Optimize and compress all assets
- Add preconnect/dns-prefetch for external resources
- Keep render cycles fast and non-blocking
- Profile and test performance continuously

---

### 6.4 Performance Validation Before Launch

**Pre-Launch Checklist**:
- [ ] Run `npm run build` successfully
- [ ] Test production build locally: `npm run preview`
- [ ] Run Lighthouse audit (target: 95+ performance, 95+ accessibility, 100 best practices, 100 SEO)
- [ ] Check Vercel Speed Insights after deployment (target: 100/100)
- [ ] Test on slow 3G connection (Chrome DevTools throttling)
- [ ] Verify all images load lazily (Network tab)
- [ ] Check bundle size: `npm run build` output (should be similar to current site)
- [ ] Verify no console errors or warnings
- [ ] Test Web Vitals across 5 page loads (consistent good scores)

**If Performance Drops Below 100**:
1. Check bundle size: Is it significantly larger than current site?
2. Check images: Are they optimized and lazy loaded?
3. Check render blocking: Any synchronous scripts in <head>?
4. Check re-renders: Use React DevTools Profiler to find bottlenecks
5. Check network: Are external resources being prefetched?

---

### 6.5 Key Files Reference

**Performance Implementation**:
- `src/main.tsx` - Analytics, SpeedInsights, initPerformance()
- `src/lib/performance.ts` - Performance utilities, Web Vitals, lazy load observer
- `src/OpochLanding.tsx` - Example of lazy loading, Suspense, React.memo patterns
- `src/App.tsx` - Route-level code splitting with React.lazy()

**Performance Monitoring**:
- Vercel Dashboard → Speed Insights tab
- Browser Console → Web Vitals logs (in dev mode)
- Chrome DevTools → Lighthouse tab

---

## 7. KNOWN CONSTRAINTS & LIMITATIONS

### 7.1 Current Limitations

**Missing Assets**:
- AI logos (GPT-5, Claude, Grok, DeepSeek) - need SVG/PNG files
- Public verification chat URLs - need real links
- Downloadable verification pack content - need actual prompts

**Incomplete Content**:
- Comparison table data (theories, problems, solutions)
- Bounty details (prize amounts, criteria, links)
- Falsify attempt records (if any exist)
- Impact section content (specific applications)

**Approach**: Use placeholders, build structure first, add real content progressively

---

### 7.2 Technical Constraints

**Dev Server Port**: May use 5174 instead of 5173 if occupied
**Build Time**: TypeScript compilation + Vite bundling ~5-10 seconds
**Hot Reload**: Available, but full page refresh may be needed for route changes

---

## 8. SUCCESS METRICS

### 8.1 Development Metrics

**Velocity**:
- Target: 1 section per iteration
- Hero section: Baseline for complexity estimation

**Quality**:
- Zero console errors
- All specs matched exactly
- Positive review feedback

---

### 8.2 Launch Metrics (Post-Release)

**Primary**: Click "Test It Yourself" CTA
**Secondary**:
- Time on page (target: 60+ seconds)
- Scroll depth (target: reach comparison table)
- Click verification links
- Download verification pack
- Browse bounties

**Analytics Setup**: Use existing `track()` function in `src/lib/analytics.ts`

---

## 9. REFERENCE QUICK LINKS

### 9.1 Key Documents

- **High-Level Structure**: `docs/homepage-v2/homepage-structure.md`
- **Hero Spec**: `docs/homepage-v2/SEC-01-HERO.md`
- **Original Plan** (deprecated): `docs/homepage-v2/plan.md` ← Use new structure instead
- **Context Index**: `docs/context_index.md`
- **Chat Architecture**: `/Users/ravishq/code/opoch-chat-test/docs/02-TWO-APP-ARCHITECTURE.md`

---

### 9.2 Key Files

**Authentication**:
- `src/lib/supabase.ts` - Supabase client with cookie config
- `src/contexts/AuthContext.tsx` - Auth state management
- `src/components/AuthModal.tsx` - Sign-in modal

**Routing**:
- `src/App.tsx` - Route definitions
- `src/pages/HomeV2.tsx` - New homepage (when created)
- `src/OpochLanding.tsx` - Current homepage (will become /classic)

**Styling**:
- `tailwind.config.js` - Brand colors, theme
- `src/index.css` - Global styles, CSS variables

**Analytics**:
- `src/lib/analytics.ts` - Event tracking

---

## 10. TROUBLESHOOTING

### 10.1 Common Issues

**Issue**: Port 5173 already in use
**Solution**: Dev server auto-selects 5174, no action needed

**Issue**: HMR not working after file changes
**Solution**: Full page refresh (Cmd+R), check console for errors

**Issue**: Route not found after adding new page
**Solution**: Verify lazy import in App.tsx, check route path spelling

**Issue**: Cookies not sharing between domains
**Solution**: Verify `.opoch.com` domain in supabase.ts, check browser cookies in DevTools

---

### 10.2 Debug Tools

**Chrome MCP**: Screenshots, browser automation, testing
**Console Logs**: Check for errors, verify analytics events
**React DevTools**: Component hierarchy, props inspection
**Network Tab**: Verify API calls, check cookie headers

---

## 11. NEXT STEPS

### 11.1 Immediate Actions

✅ **COMPLETED**:
- Reverted all experimental code changes
- Created this implementation clarity document
- Clean repo state achieved
- Updated `context_index.md` with homepage-v2 reference
- Documented performance optimization strategy (100/100 Speed Insights)

🔄 **READY TO START**:
- Verified clean state (no build errors)

⏳ **NEXT**:
- Build Hero section per `SEC-01-HERO.md` spec
- Screenshot and review
- Iterate if needed
- Move to Section 2 (Comparison Table)

---

## 12. TEAM COMMUNICATION

### 12.1 When to Ask for Clarification

**Always ask before proceeding if**:
- Spec has conflicting information
- Missing critical content (copy, URLs, assets)
- Uncertain about design choice
- Unsure about technical approach

**Don't guess**: Better to ask and stay anchored than drift and rebuild

---

### 12.2 How to Stay Anchored

1. **Reference this doc** before starting each section
2. **Follow the spec exactly** - don't improvise unless explicitly needed
3. **Screenshot and review** - visual confirmation prevents drift
4. **Update this doc** if new decisions are made
5. **Mark ambiguities** and resolve before coding

---

**END OF IMPLEMENTATION CLARITY DOCUMENT**

*This is a living document. Update as decisions are made and context changes.*
