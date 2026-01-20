# Repository Context Index

> **Purpose**: Navigation map for AI agents and developers to quickly locate critical files and understand repository structure. This is a "what and where" guide, not a "how-to" guide.

---

## 🏗️ Architecture & Configuration

### Core Config Files
- **`package.json`** - Dependencies, scripts, Node.js requirements
- **`vite.config.ts`** - Build configuration, port (5173), path aliases (@/)
- **`tailwind.config.js`** - Design system, brand colors (opoch.cyan, etc.), theme tokens
- **`src/index.css`** - CSS variables, global styles, dark theme defaults
- **`.claude/settings.local.json`** - MCP servers, permissions, enabled features

### Architecture Documentation
- **`docs/architecture.md`** - Tech stack decisions, component architecture, deployment strategy
- **`docs/dependencies.md`** - Full dependency list with versions, compatibility notes, upgrade recommendations
- **`README.md`** - Project overview, setup instructions, deployment guide

---

## 🎯 Application Entry Points

### Main Entry
- **`src/main.tsx`** - ReactDOM root, Analytics/SpeedInsights setup, dev debug tools
- **`src/App.tsx`** - Router setup, route definitions, lazy loading, AuthProvider/HelmetProvider wrappers

### Routes Map
```
/                      → src/OpochLanding.tsx
/auth/callback         → src/pages/AuthCallbackPage.tsx
/moonshots             → src/pages/MoonshotsPage.tsx
/moonshots/:slug       → src/pages/MoonshotDetailPage.tsx
/updates               → src/pages/UpdatesPage.tsx
/updates/:slug         → src/pages/UpdateDetailPage.tsx
/admin/emails          → src/pages/AdminEmailsPage.tsx (protected)
/admin/leads           → src/pages/AdminLeadsPage.tsx (protected)
```

---

## 🎨 Design System & UI

### Brand Identity
- **`tailwind.config.js`** - Opoch color palette (cyan #1CE4E6, blue shades, tertiary colors)
- **`src/index.css`** - CSS variables (--background, --foreground, etc.), typography (Gotham HTF)

### UI Components
- **`src/components/ui/`** - Base components (button, card, dialog, input, textarea, phone-input)
- **`src/lib/utils.ts`** - `cn()` utility for className merging

### Landing Page Sections
- **`src/components/landing/Hero.tsx`** - Hero section
- **`src/components/landing/Moonshots.tsx`** - Moonshots showcase
- **`src/components/landing/Process.tsx`** - Process explanation
- **`src/components/landing/Outcomes.tsx`** - Outcomes section
- **`src/components/landing/Who.tsx`** - Who we serve
- **`src/components/landing/FAQ.tsx`** - FAQ accordion
- **`src/components/landing/Nav.tsx`** - Navigation bar
- **`src/components/landing/ReceiptBadge.tsx`** - Receipt badge component

---

## 🔐 Authentication & User Management

### Core Auth
- **`src/contexts/AuthContext.tsx`** - Auth state, session management, user profile
- **`src/lib/supabase.ts`** - Supabase client initialization
- **`src/components/AuthModal.tsx`** - Sign-in modal (Google/GitHub SSO)
- **`src/components/UserMenu.tsx`** - User dropdown menu
- **`src/components/ProtectedRoute.tsx`** - Route protection wrapper

### Auth Flow
1. User clicks sign-in → `AuthModal.tsx`
2. OAuth redirect → `/auth/callback` → `AuthCallbackPage.tsx`
3. Session stored → `AuthContext.tsx` updates
4. Protected routes check → `ProtectedRoute.tsx`

### Related Docs
- **`docs/auth-debugging-guide.md`** - Auth troubleshooting
- **`AUTH_DEBUG_GUIDE.md`** - Root-level auth debug guide

---

## 📊 Data & Content

### Static Data
- **`src/data/moonshots.ts`** - 15 moonshot challenges (physics, energy, bio, infra, robotics)
  - Type definitions: `Moonshot`, `MoonshotStatus`, `MoonshotDomain`
  - Helper functions: `getMoonshotBySlug()`, `getMoonshotsByDomain()`, `getMoonshotsByStatus()`

### Database Schema
- **`supabase/schema.sql`** - Initial database schema
- **`supabase/schema_phase3.sql`** - Phase 3 additions
- **`supabase/fix_backend.sql`** - Email templates, RLS policies
- **`supabase/create-profiles-table.sql`** - User profiles table
- **`supabase/fix-rls-step-by-step.sql`** - RLS policy fixes

### Database Tables (Supabase)
- **`leads`** - Form submissions from apply modal
- **`queries`** - User questions from ExploreWithOpoch
- **`profiles`** - User profile data (name, email, phone, company)
- **`email_templates`** - Email content templates

---

## 📝 Forms & Lead Capture

### Form Components
- **`src/components/ApplyModal.tsx`** - Main application form, profile pre-fill
- **`src/components/ExploreWithOpoch.tsx`** - Query submission, auth-aware
- **`src/components/AskBox.tsx`** - Legacy ask box (authenticated)
- **`src/components/AskBoxTeaser.tsx`** - Legacy ask box teaser (unauthenticated)

### Server Endpoints (Vercel Functions)
- **`api/submit-lead.ts`** - Process lead submissions, bypass RLS
- **`api/submit-query.ts`** - Process query submissions, bypass RLS
- **`api/get-profile.ts`** - Fetch user profile data
- **`api/ping-supabase.ts`** - Health check endpoint

### Form Flow
1. User opens form → Pre-fills from `profiles` table if authenticated
2. User submits → Server endpoint (`/api/submit-*`)
3. Data saved to Supabase → Email sent via Resend
4. Profile persisted → Next visit auto-fills

---

## 📧 Email System

### Email Configuration
- **`src/lib/email.ts`** - Email utilities, templates
- **Resend API** - Email delivery service
- **Reply-to**: `hello@opoch.com`

### Email Templates (in Supabase)
- Welcome email (sent once on first login)
- Lead submission confirmation
- Query submission confirmation

### Assets
- **`/Opoch Assets/Opoch Typelogo.png`** - PNG logo for email compatibility

### Related Docs
- **`docs/email-testing-guide.md`** - Email testing instructions

---

## 🚀 Deployment & DevOps

### Deployment
- **Platform**: Vercel
- **Trigger**: Push to `main` branch → Auto-deploy
- **Scripts**: `./scripts/deploy.sh` (preview/production)

### Environment Variables
**Client-side (VITE_ prefix):**
- `VITE_SUPABASE_URL`
- `VITE_SUPABASE_ANON_KEY`

**Server-side (Vercel):**
- `SUPABASE_SERVICE_ROLE_KEY`
- `RESEND_API_KEY`

### Related Docs
- **`docs/deployment-guide.md`** - Deployment checklist, RLS setup, testing
- **`docs/vercel-env-setup.md`** - Environment variable setup
- **`VERCEL_ENV_SETUP.md`** - Root-level env setup

---

## 🧪 Testing & Quality

### Test Setup
- **`src/test/setup.ts`** - Vitest configuration
- **`src/components/OpochLogo.test.tsx`** - Example component test
- **`src/tests/basic.test.tsx`** - Basic test suite

### Scripts
- `npm test` - Run tests
- `npm run test:ui` - Test UI
- `npm run test:coverage` - Coverage report
- `npm run lint` - ESLint

### Related Docs
- **`docs/backend-testing-guide.md`** - Backend testing
- **`TEST_SUMMARY.md`** - Test summary

---

## 🛠️ Utilities & Helpers

### Core Utilities
- **`src/lib/utils.ts`** - `cn()` className utility
- **`src/lib/analytics.ts`** - Event tracking utilities
- **`src/lib/performance.ts`** - Performance monitoring
- **`src/lib/metadata.ts`** - SEO metadata helpers
- **`src/lib/markdown.ts`** - Markdown processing (MDX support)
- **`src/lib/error-handler.ts`** - Error handling utilities

### Custom Hooks
- **`src/hooks/use-toast.tsx`** - Toast notifications
- **`src/hooks/use-async.ts`** - Async operation wrapper
- **`src/hooks/use-debounce.ts`** - Debounce utility
- **`src/hooks/use-local-storage.ts`** - Local storage hook
- **`src/hooks/use-click-outside.ts`** - Outside click detection
- **`src/hooks/useDebouncedValue.ts`** - Debounced value
- **`src/hooks/usePerformanceMonitor.ts`** - Performance monitoring

### Debug Tools
- **`src/utils/debug-supabase.ts`** - Supabase debug utilities (dev only)
- **Available in console**: `debugSupabase()` function

---

## 🤖 MCP Servers & Integrations

### Configured MCPs
1. **Chrome Browser MCP** - Web automation, screenshots, testing
   - Setup: `docs/chrome-mcp-setup.md`
   - Usage: `docs/chrome-mcp-usage.md`

2. **Linear MCP** - Issue tracking, project management
   - Setup: `linear-mcp-setup.md`
   - API Key: Stored in setup doc

### MCP Config
- **`.claude/settings.local.json`** - Enabled servers, permissions

---

## 📚 Documentation Map

### Implementation Guides
- **`docs/implementation-summary.md`** - All implemented features overview
- **`docs/backend-update-summary.md`** - Backend updates
- **`docs/fix-signout-and-forms.md`** - Sign-out and form fixes

### Setup & Configuration
- **`docs/deployment-guide.md`** - Deployment process
- **`docs/vercel-env-setup.md`** - Vercel environment setup
- **`docs/chrome-mcp-setup.md`** - Chrome MCP installation
- **`docs/chrome-mcp-usage.md`** - Chrome MCP usage examples
- **`linear-mcp-setup.md`** - Linear MCP installation

### Debugging & Troubleshooting
- **`docs/auth-debugging-guide.md`** - Auth issues
- **`docs/email-testing-guide.md`** - Email testing
- **`docs/backend-testing-guide.md`** - Backend testing
- **`docs/rls-fix-instructions.md`** - RLS policy fixes

### Root-Level Docs (Legacy/Specific)
- **`PRODUCTION_FIXES.md`** - Production issue fixes
- **`MOBILE_HEADER_FIXES.md`** - Mobile header issues
- **`PHONE_INPUT_FIX.md`** - Phone input fixes
- **`PHONE_INPUT_FINAL_FIX.md`** - Final phone input solution
- **`MOONSHOT_IMAGES_COMPLETE.md`** - Moonshot images work
- **`MOONSHOT_METADATA_ENHANCEMENTS.md`** - Metadata improvements
- **`LINEAR_INTEGRATION_SUMMARY.md`** - Linear integration details
- **`SUPABASE_ISSUE_REPORT.md`** - Supabase issues
- **`SUPABASE_SQL_SETUP.md`** - SQL setup instructions
- **`SECURITY.md`** - Security best practices

### Future Work
- **`docs/homepage-v2/plan.md`** - Homepage V2 planning document

---

## 🎯 Common Tasks → File Locations

### Adding a New Route
1. Create page in `src/pages/`
2. Add lazy import in `src/App.tsx`
3. Add route in `<Routes>` section

### Modifying Design System
1. Colors → `tailwind.config.js` (opoch.* colors)
2. CSS variables → `src/index.css`
3. UI components → `src/components/ui/`

### Auth Issues
1. Check `src/contexts/AuthContext.tsx`
2. Debug with `src/utils/debug-supabase.ts`
3. Review `docs/auth-debugging-guide.md`

### Form Changes
1. Modal → `src/components/ApplyModal.tsx`
2. Query box → `src/components/ExploreWithOpoch.tsx`
3. Server logic → `api/submit-*.ts`
4. Database → `supabase/*.sql`

### Email Issues
1. Templates → Supabase `email_templates` table
2. Sending logic → `api/submit-*.ts` endpoints
3. Testing → `docs/email-testing-guide.md`

### Adding Moonshots
1. Edit `src/data/moonshots.ts`
2. Add images to `/moonshots/:slug/` folder
3. Update types if needed

### Database Changes
1. Create SQL file in `supabase/`
2. Run in Supabase SQL Editor
3. Update TypeScript types in `src/types/index.ts`
4. Document in relevant guide

### Deployment Issues
1. Check `docs/deployment-guide.md`
2. Verify env vars in Vercel dashboard
3. Review `docs/vercel-env-setup.md`

---

## 📐 Type Definitions

### Core Types
- **`src/types/index.ts`** - Shared TypeScript types
- **`src/data/moonshots.ts`** - Moonshot types
- **`src/vite-env.d.ts`** - Vite environment types

### Key Interfaces
- `Moonshot` - Moonshot data structure
- `MoonshotStatus` - "open" | "in-progress" | "partners"
- `MoonshotDomain` - "physics" | "energy" | "bio" | "infra" | "robotics"

---

## 🚧 Homepage V2 Development

### Overview
New homepage for Theory of Everything (TOE) platform, targeting deep tech founders, VCs, researchers.

### Documentation
- **Implementation Guide**: `docs/homepage-v2/implementation-clarity.md` ← **START HERE**
- **High-Level Structure**: `docs/homepage-v2/homepage-structure.md`
- **Section Specs**: `docs/homepage-v2/SEC-01-HERO.md`, etc.

### Build Approach
- Build sections at `/home-v2` first (staging route)
- Test iteratively, section by section
- Swap `/ ` → HomeV2 when complete (old becomes `/classic`)
- Existing site stays functional throughout development

### Key Files (When Created)
- `src/pages/HomeV2.tsx` - New homepage
- `src/sections/v2/` - Section components
- `src/components/v2/` - V2-specific components
- `src/data/v2/` - V2 data files

### Technical Context
- Auth/cookies: ✅ Already working (Supabase, `.opoch.com` domain)
- Chat integration: ✅ Ready (chat.opoch.com with shared cookies)
- Brand colors: Use existing (black, white, cyan #1CE4E6)
- No changes needed to existing routes/pages

---

## 🔍 Quick Reference

### Important Constants
- **Dev server port**: 5173 (vite.config.ts)
- **Node version**: 20.19.4 LTS (package.json engines)
- **Main branch**: `main`
- **Default theme**: Dark mode
- **Primary brand color**: #1CE4E6 (opoch-cyan)
- **Font stack**: Gotham HTF, Whitney, Verdana

### Key Commands
```bash
npm run dev          # Start dev server (port 5173)
npm run build        # Production build
npm test             # Run tests
npm run lint         # Lint code
```

### Asset Locations
- **Brand assets**: `/Opoch Assets/`
- **Favicon**: `/Opoch Assets/favicon.svg`
- **OG image**: `/Opoch Assets/og-image.jpg`
- **Email logo**: `/Opoch Assets/Opoch Typelogo.png`

---

## 🚦 Status & Health

### Working Features ✅
- Authentication (Google/GitHub SSO)
- Lead capture forms
- Query submission
- Moonshots catalog
- Updates/blog pages
- Admin dashboards
- Email notifications
- Profile persistence
- RLS policies
- Service worker

### Known Issues
- Check `docs/deployment-guide.md` for current fixes
- Review root-level `*_FIXES.md` files for specific issues

---

**Last Updated**: 2025-10-22
**Maintained By**: Claude Code AI Agent
**For Questions**: Refer to specific doc files or use MCP tools to explore code
