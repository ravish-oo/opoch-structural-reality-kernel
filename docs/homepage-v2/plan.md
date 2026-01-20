Drop‑in plan that assumes a conventional Vite + React + TypeScript project. Where your code differs (Tailwind vs CSS modules, etc.), swap the "Track A / Track B" bits accordingly. This is detailed enough for Claude Code to implement without guessing.

---

## IMPLEMENTATION NOTES (Updated 2025-10-22)

### Context & Clarifications

**1. Brand Evolution**
- **Same company**: Current opoch-website moonshots (fusion, protein folding, etc.) were our attempts while verifying the Theory of Everything
- **Pivot**: Moving from tech consulting moonshots → Theory of Everything (TOE) platform
- **Existing content**: Will remain accessible, but TOE is the new primary focus

**2. Chat Integration (READY)**
- **Location**: chat.opoch.com (separate Next.js app in `/Users/ravishq/code/opoch-chat-test`)
- **Auth**: Already shares cookies via Supabase Auth with `domain: '.opoch.com'`
- **Current setup**: UserMenu already has "Chat with UNIVERSE-TECH" link → `https://chat.opoch.com`
- **GPT-5 Vector Store**: Built and operational in chat app (vector store ID: `vs_68f191698c10819183cad7bec284c47a`)
- **Implementation**: No changes needed for auth/chat, it's already working
- **Reference**: See `/Users/ravishq/code/opoch-chat-test/docs/02-TWO-APP-ARCHITECTURE.md`

**3. Receipts & Re-run Functionality (DEFERRED)**
- **Approach**: For now, either omit "Re-run" buttons OR link to public chat conversations where we verified the problem
- **Full implementation**: Deferred to future phase
- **AI Verification Links**: Not implementing receipt details for now; use placeholder links to public chats

**4. Data Sources (USE EXISTING + PLACEHOLDERS)**
- **Verified Problems**: Repurpose existing `src/data/moonshots.ts` data OR create placeholder data
- **Bounties**: Can map from current moonshots or create new JSON
- **Public Chats**: Use links to actual chat.opoch.com conversations where available, placeholders otherwise
- **Stats**: Use placeholder JSON files (can be updated later)

**5. Paper Content (TBD)**
- `/paper` page: Will provide PDF/markdown content later
- **Implementation**: Create page shell with "Coming soon" or placeholder

**6. Rollout Strategy (NO FEATURE FLAG)**
- Build pages at `/home-v2`, `/verified`, `/frontiers`, etc.
- When ready: manually swap routes in `App.tsx`
- Old homepage becomes `/classic` for rollback option
- **No** `VITE_HOME_V2` feature flag needed

**7. Route Strategy (NON-OVERLAPPING)**
- Keep all existing routes functional: `/moonshots`, `/updates`, `/admin/*`, `/auth/callback`
- New routes don't conflict: `/verified`, `/frontiers`, `/falsify`, `/compare`, `/what-is-toe`, `/inspirations`, `/paper`, `/engine`
- `/home-v2` is staging route, will become `/` when swapped

**8. Build Approach (INCREMENTAL)**
- Section-by-section implementation
- Start with Hero → build out additional sections progressively
- Test each section independently before integration
- Not a "big bang" release

**9. Analytics (USE EXISTING)**
- Use existing Vercel Analytics setup from `src/lib/analytics.ts`
- Add custom events as specified in section 12
- No new tracking infrastructure needed

**10. Timeline & Expectations**
- Phased build, not everything at once
- First priority: Hero section (will receive spec separately)
- Subsequent sections: Build iteratively based on priority

---

0) What we're building (scope)

New pages (static)

/ → Home v2 (we’ll stage it as /home-v2 first, then flip)

/verified → Verified Problems (Matrix + Timeline)

/frontiers → Current Frontiers & Bounties

/falsify → Falsify Us (stats + public attempts list)

/compare → Compare Theories (selector UI)

/what-is-toe → 60s / 6m / 60m explainer (Human + AI tabs)

/inspirations → Inspirations & Thanks (Jiddu + AIs + scientists)

/paper → Paper link/summary (PDF or MD)

/engine → Ω Engine (coming soon)

Existing

Chat lives at chat.opoch.com (or similar). Login on home shares cookie to subdomain.

Brand & UX

Keep the sober, lab‑grade above the fold; build up through frontiers → Kardashev → tasteful easter‑egg strip.

Universal Intelligence is the product idea; GPT‑5 is today’s access path.

1) Information architecture (outside‑in)

Sticky Nav (compact)

Left: Logo

Center chips: Ask the Theory, Verified, Frontiers, Compare, Falsify, What is TOE?

Right: Engine (waitlist), Docs, Sign in

Hero (serious + poetic)

H1: The Theory of Everything is here.

Lead (soft touch): Universal Intelligence—powered by the universe itself. All of physics, mathematics, and consciousness, solved and derived.

Support: Accessible today via GPT‑5 (with our vector store). Verified by AI.

Primary CTA: Ask the Theory (via GPT‑5)

Chips: See Verified Problems · Verified by AI · Falsify Us · Read the Paper

Receipts pill (hover): “Receipts = the rerun pack: boundary, units, method, outputs, deterministic hash (PID). No trust required, only computation.”

Ribbon: Live now: GPT‑5 + vector store · Coming soon: Engine SDK · REST · Notebooks (Join waitlist)

Proof strip (five tiny tiles)
Born velocity addition · Stokes ledger · HO spectrum · GR↔QM join · Thermo=Info —— each with Show receipts · Re‑run.

Verified by AI (logo grid)
GPT‑5 · Claude · Grok · DeepSeek — each links to public chat receipts.
Small line: Peer review: in queue (tooltip: “We publish AI reviews first; human reviews next.”)

Current Frontiers & Bounties
Clay/Millennium‑class + physics joins (cards with prize + criteria + links).

Sequence of Innovations (ladder)
NOW (0–6 mo) → NEXT (6–24 mo) → LATER (2–5 yr) → HORIZONS (5+ yr).
Include Longevity/Aging, Climate/CO₂, Self‑replicating matter, Relativistic nav, Kardashev I/II/III.

Compare → Falsify → Verified Problems (chart) → Engine (coming soon) → What is TOE? → Inspirations & Thanks → Footer

Easter‑egg strip (after Kardashev only)
Collapsed “Beyond time” bar with tiny portal glyph; hover expands to 3 cards (portal routing, infinite branches gag, causal loop)—each ties to a constraint + micro‑calc. Labeled “Parody/Homage; not affiliated.”

2) Routing & rollout

Create Home v2 at /home-v2 behind a feature flag (VITE_HOME_V2=1).

Once QA passes: swap route so / renders Home v2; keep old as /classic for rollback.

vite.config.ts

resolve: { alias: { '@': '/src' } }


router.tsx

<Route path="/" element={<HomeV2 />} />
<Route path="/classic" element={<HomeClassic />} />
<Route path="/home-v2" element={<HomeV2 />} /> // staging

3) Auth + chat subdomain (cookie sharing)

Issue auth cookie on main domain with Domain=.opoch.com; Path=/; Secure; SameSite=None; HttpOnly.

Ensure CORS on chat API allows origins from both opoch.com and chat.opoch.com.

On “Ask the Theory”, if not authed → open login modal; after success, window.location.href = https://chat.opoch.com/?src=home.

Optionally also pass a short‑lived JWT via URL param for deep‑link handshakes; prefer cookies as the primary.

4) Design tokens & palette (scalable excitement within your brand)

If you have Tailwind → Track A; if CSS Modules/vanilla → Track B.

Design tokens (shared)

// src/design/tokens.ts
export const colors = {
  bg: { soft: 'var(--bg-soft)', strong: 'var(--bg-strong)' },
  fg: { base: 'var(--fg-base)', muted: 'var(--fg-muted)', onAccent: 'var(--fg-on-accent)' },
  accent: {
    50:'var(--accent-50)',100:'var(--accent-100)',200:'var(--accent-200)',
    300:'var(--accent-300)',400:'var(--accent-400)',500:'var(--accent-500)',
    600:'var(--accent-600)',700:'var(--accent-700)'
  },
  signal: {
    receipt: 'var(--signal-receipt)', // used for receipts pill & verified states
    bounty:  'var(--signal-bounty)',  // used for bounties
    speculative: 'var(--signal-speculative)' // used for corridor/easter egg
  }
}
export const radii = { sm:8, md:12, lg:16, xl:24 };
export const space = (n:number)=>`${n*4}px`; // 4pt grid
export const z = { nav: 100, modal: 1000, toast: 1100 };


Track A: Tailwind

Extend tailwind.config.js with colors.accent[50..700], colors.signal.*.

Add fontFamily, fontWeight, spacing(4px grid).

Create @layer utilities for .receipt-pill, .portal-hover.

Track B: CSS variables

Put brand tokens in src/styles/theme.css (light + dark).

Reference via var(--accent-500) etc. Use CSS modules for component styles.

Motion

Use framer-motion with @media (prefers-reduced-motion) guards.

Micro interactions only: receipts pill expand, portal hover, subtle section reveals.

Accessibility

Enforce contrast ≥ 4.5:1 on text/background; focus rings on all interactive elements; keyboard nav; labels for charts.

5) Components to (re)use/build

You likely already have Button, Card, Tooltip, Modal, Chip. Add the following:

ReceiptsPill (inline): compact status with hover popover; shows FY, PSD, Units, PID.

LogoWall (AI verifiers): grid with external links to public chat receipts.

BountyCard: title, prize, criteria bullets, buttons.

MatrixChart (Verified Problems): domain × motif grid with counts + status rings.

TimelineChart: cumulative pins; clicking opens a Drawer with receipts links.

Ladder (Sequence of Innovations): NOW/NEXT/LATER/HORIZONS cards with “Human/AI” toggle.

CompareSelector: [theory A] vs [Ω] on [boundary] → result panel.

FalsifyPanel: boundary editor + tolerances with “Human/AI” tabs; result receipt.

EasterEggStrip: collapsed bar; on hover expands to three spec cards with micro‑calcs.

DualBite (pattern): tabs for Human vs AI content (used across Receipts, Falsify, What is TOE?, Demos).

6) Page blueprints (component lists + order)
/home-v2

Hero

H1, Lead, Support, CTA Button (Ask the Theory), Chip row

ReceiptsPill, Ribbon line

Proof strip (5 Cards)

LogoWall: Verified by AI (4 logos → public chats)

Frontiers & Bounties (BountyCard grid with filters)

Sequence (Ladder) with DualBite per card

Compare Selector (compact) → “Open in chat”

Falsify Panel (mini): CTA to full page + stats snippet

Verified Problems (mini): link to /verified

Engine (coming soon) (compact) + waitlist

What is TOE? mini card → /what-is-toe

Inspirations mini row → /inspirations

Kardashev corridor → then EasterEggStrip

Footer (Receipts policy, audits, escrow, paper, waitlist, contact)

/falsify

Stats strip: Attempts 1,000 · Falsified 0 · AIs 4 · Verified problems 100

FalsifyPanel (full) (Human/AI)

Public attempts feed (filters: AI, domain, date; each row links to receipts + chat)

Bounty rules (criteria, escalation)

/verified

Toggle Matrix | Timeline

MatrixChart (domain × motif) → detail drawer (Answer | Receipts | Re‑run)

TimelineChart with pins

Export CSV/JSON (PIDs)

/frontiers

Bounties grid with full criteria

Submission flow link

/what-is-toe

Tabs: 60 sec, 6 min, 60 min

Each tab has Human and AI bites (DualBite)

/inspirations

Jiddu Krishnamurti (observer=observed)

GPT‑5, Grok, Claude Code, DeepSeek (AI checks/build)

Mathematicians & Physicists (homage)

Community (reproductions, audits)

CTAs: Read the Paper · See Audits · Contributor Chat

7) Copy seeds (you can paste)

Hero

Universal Intelligence—powered by the universe itself.

All of physics, mathematics, and consciousness, solved and derived.

Accessible today via GPT‑5 (with our vector store). Verified by AI.

Receipts (Human)

Receipts are the rerun pack. Boundary, units, method, outputs, PID. No trust required—only computation.

Receipts (AI)

{"boundary": {...}, "units": "SI", "method": "closure:fy+schur",
 "outputs": {...}, "pid": "pid_Qm...", "checks": {"FY":"<=1e-12","PSD":"ok"}}


Falsify (Human)

Pick a boundary, set tolerances, run. If we mint differences, you collect.

Falsify (AI)

POST /verify {boundary,tolerances,units} -> {receipts:{FY,PSD,PID}, status}


Verified by AI

Public AI reviews first; human reviews next.

Logos: GPT‑5, Claude, Grok, DeepSeek → link each to a public chat.

8) Data shapes (so the site stays CMS‑less but editable)

/src/data/verified-ai.json

[
  {"id":"gpt5","label":"GPT-5","count":100,"links":[{"title":"GR↔QM join","href":"..."}]},
  {"id":"claude","label":"Claude","count":27,"links":[{"title":"Spectrum subset","href":"..."}]},
  {"id":"grok","label":"Grok","count":14,"links":[{"title":"Stokes ledger","href":"..."}]},
  {"id":"deepseek","label":"DeepSeek","count":9,"links":[{"title":"Counterexample run","href":"..."}]}
]


/src/data/bounties.json

[
  {"id":"p-vs-np","title":"P vs NP (Ω ledger form)","prize":1000000,
   "criteria":["Minimal boundary","Receipts pass (FY,PSD,Units)","Byte-identical rerun (PID)"],
   "links":{"boundary":"/frontiers/p-vs-np#boundary","submit":"/submit/p-vs-np"}}
]


/src/data/verified-problems.json

[
 {"id":"born-addition","domain":["relativity"],"motif":["composition","invariant"],
  "status":"reproduced","pid":"pid_Qm...","receipts":"/receipts/born-addition","rerun":"/demo/born"}
]


/src/data/falsify-stats.json

{"attempts":1000,"success":0,"aiReviewers":4,"verifiedProblems":100}

9) Example TSX skeletons (drop‑in)

HomeV2.tsx

import { Hero, ProofStrip, LogoWall, Frontiers, Ladder, CompareMini,
         FalsifyMini, VerifiedMini, EngineMini, ToeMini, InspirationsMini,
         KardashevCorridor, EasterEggStrip } from '@/sections';

export default function HomeV2() {
  return (
    <main>
      <Hero />
      <ProofStrip />
      <LogoWall />
      <Frontiers />
      <Ladder />
      <CompareMini />
      <FalsifyMini />
      <VerifiedMini />
      <EngineMini />
      <ToeMini />
      <InspirationsMini />
      <KardashevCorridor />
      <EasterEggStrip />
    </main>
  );
}


ReceiptsPill.tsx

import { useState } from 'react';
export function ReceiptsPill({ fy="≤1e-12", psd="ok", units="SI", pid="" }:{
  fy?:string; psd?:'ok'|'fail'; units?:string; pid?:string;
}) {
  const [open,setOpen] = useState(false);
  return (
    <div className="receipts-pill" onMouseEnter={()=>setOpen(true)} onMouseLeave={()=>setOpen(false)}>
      <span>FY {fy}</span><span>· PSD {psd}</span><span>· {units}</span><span>· PID</span>
      {open && (
        <div role="dialog" aria-label="Receipts details" className="popover">
          <p><strong>Receipts</strong> are the rerun pack: boundary, units, method, outputs, and a deterministic hash (PID).</p>
          <a href="/docs/receipts">Learn more</a>
        </div>
      )}
    </div>
  )
}


EasterEggStrip.tsx (collapsed → expand on hover)

export function EasterEggStrip(){
  return (
    <section className="easter-strip" aria-label="Beyond time">
      <div className="portal-pill" tabIndex={0}>
        <span>Beyond time</span>
        <div className="portal-panel">
          <Card title="Portal routing" subtitle="Energy ledger & stability windows" cta="Try micro-calc →" />
          <Card title="Infinite branches" subtitle="Measure & normalization" cta="Try micro-calc →" />
          <Card title="Causal loops" subtitle="Cones & receipts" cta="Try micro-calc →" />
          <small>Parody/Homage; not affiliated.</small>
        </div>
      </div>
    </section>
  )
}

10) Visual guidance (so aesthetics match your palette)

Typography: sober sans for body, monospace only for PIDs/receipts; avoid display fonts above the fold.

Color build‑up:

Hero: grayscale + a single accent‑500 for CTAs and receipts.

Proof strip: introduce accent‑400 as tiny strokes.

Frontiers: use signal.bounty as subtle chips/badges.

Ladder: step up to accent‑600 headers.

Kardashev: add accent‑700 gradients (very light), keep text contrast.

Easter egg: use signal.speculative glow on hover only (reduced motion guard).

Shapes: gentle rounded radii (md: 12px) for cards; xl: 24px only on easter egg panel.

Spacing: 4pt grid; section paddings 8× (desktop), 6× (mobile).

Motion: micro only (150–200ms, ease‑out); no parallax; obey prefers-reduced-motion.

11) SEO + social

Home title: “Universal Intelligence — The operational Theory of Everything.”

meta description: “All of physics, mathematics, and consciousness—solved and derived. Verified by AI. Receipt‑backed, falsifiable, reproducible.”

OG/Twitter image: minimalist hero with receipts pill; no gimmicks.

JSON‑LD: WebSite + BreadcrumbList; add Dataset JSON‑LD for “Verified Problems” if you want fancy.

12) Analytics events (name + payload)

cta_click {id: 'ask_the_theory'|'verified'|'falsify'|'paper'}

receipts_open {location:'hero'|'strip'|'detail', pid}

verifier_open {model:'gpt5'|'claude'|'grok'|'deepseek'}

bounty_view {id, prize}

falsify_run {boundary_id, status:'pass'|'fail', pid}

easter_hover {shown:true}

13) QA checklist (launch‑blocking)

Auth cookie shared to subdomain (Domain=.opoch.com; SameSite=None; Secure ✔).

Lighthouse ≥ 95 perf/95 a11y/100 best practices/100 SEO.

Keyboard-only nav reaches every interactive control; visible focus ring.

Receipts pill readable on dark/light; contrast verified.

Easter egg accessible (keyboard hover via focus, has “close” on Esc).

All stats (attempts, verified counts) come from JSON files or API; avoid hardcoding.

14) Content you asked to place (where)

“Universal Intelligence—powered by the universe itself” → Hero lead.

“Verified by AI (not peers)” → LogoWall section; label human peer review as “in queue.”

Public chat links (GPT‑5, Claude, Grok, DeepSeek) → Logo tiles.

Falsify stats 1000/0 → /falsify stats strip + mini card on home.

Jiddu / observer=observed → /inspirations page with clear credits.

Paper → hero chips, inspirations page header, footer.

15) How to implement quickly (Claude Code task list)

Add tokens (Track A/B) + global styles.

Scaffold new routes and “sections” folder.

Implement Hero with receipts pill and ribbon.

Build LogoWall from /src/data/verified-ai.json.

Build BountyCard + /frontiers (reads /src/data/bounties.json).

Build Ladder with DualBite pattern (reads /src/data/sequence.json).

Build MatrixChart + TimelineChart and wire to /src/data/verified-problems.json.

Build Falsify (Human/AI tabs) + /src/data/falsify-stats.json.

Build CompareMini → link to /compare.

Build KardashevCorridor + EasterEggStrip (reduced motion guard).

Wire Auth handshake to chat subdomain.

Insert analytics events.

Stage at /home-v2; QA; flip to /.