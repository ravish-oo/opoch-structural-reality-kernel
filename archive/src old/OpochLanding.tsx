import React, { useState, useEffect } from "react";
import { Link, useSearchParams } from "react-router-dom";
import { motion } from "framer-motion";
import { ArrowRight, Building2, CheckCircle2, ChevronDown, HelpCircle, FileCheck, FlaskConical, LineChart, Network, Rocket, ShieldCheck, Sparkles, Target } from "lucide-react";
import { Button } from "./components/ui/button";
import { Card, CardContent } from "./components/ui/card";
import OpochLogo from "./components/OpochLogo";
import { AskBox } from "./components/AskBox";
import ApplyModal from "./components/ApplyModal";
import UserMenu from "./components/UserMenu";
import AuthModal from "./components/AuthModal";
import { useAuth } from "./contexts/AuthContext";
import { trackApplyModalOpened } from "./lib/analytics";
import SEOHead from "./components/SEOHead";
import { getPageMetadata } from "./lib/metadata";

function classNames(...xs: (string | false | undefined)[]) { return xs.filter(Boolean).join(" "); }

function Tag({ children }: { children: React.ReactNode }) {
  return (<span className="inline-flex items-center gap-2 rounded-full border border-white/10 bg-white/5 px-3 py-1 text-xs text-white/80 backdrop-blur">{children}</span>);
}

function ReceiptBadge({ state = "PASS" as "PASS" | "Ø" | "Ø_∞" }) {
  const color = state === "PASS" ? "bg-emerald-500/20 text-emerald-300" : state === "Ø" ? "bg-amber-500/20 text-amber-300" : "bg-sky-500/20 text-sky-300";
  return <span className={classNames("rounded-md px-2 py-0.5 text-xs", color)}>{state}</span>;
}

const outcomes = [
  { icon: LineChart, title: "Decisions you can defend", blurb: "Clear yes/no, trade‑offs, and next steps you can stand behind." },
  { icon: Target, title: "Faster problem unblocking", blurb: "We diagnose in hours, plot the path in days, and get you moving." },
  { icon: ShieldCheck, title: "Reduced risk", blurb: "No vague promises. We show what’s possible now and what it costs to get the rest." },
];

const who = [
  { icon: Rocket, title: "Deep‑tech founders", blurb: "You need the short path to truth, not another deck." },
  { icon: FlaskConical, title: "PI‑level researchers", blurb: "Tighter experiments. Fewer dead ends. Better funding shots." },
  { icon: Network, title: "Staff+ engineers", blurb: "Hard systems decisions with high confidence and a clear plan." },
];

const moonshots: { title: string; tagline: string; emoji: string }[] = [
  { title: "Alpha Fixed Point", tagline: "metrology under control", emoji: "🧭" },
  { title: "H(z) + Cosmic Time", tagline: "universe clock", emoji: "⏳" },
  { title: "Rotation/Lensing", tagline: "dark matter residuals", emoji: "🕳️" },
  { title: "QEC Thresholds", tagline: "logical error bounds", emoji: "🧪" },
  { title: "Fold/Bind Explorer", tagline: "biophysics short path", emoji: "🧬" },
  { title: "Fusion Control", tagline: "minimum paid work", emoji: "⚡" },
  { title: "Solid‑State Battery", tagline: "capacity proofs", emoji: "🔋" },
  { title: "Grid Phasor Flow", tagline: "routing receipts", emoji: "🌐" },
  { title: "Robotics Control", tagline: "Z₄ rhythm", emoji: "🤖" },
  { title: "Climate MRV", tagline: "measure what matters", emoji: "🌿" },
  { title: "Port Logistics", tagline: "throughput lift", emoji: "🚢" },
  { title: "Deep Space Traj", tagline: "Δv budgets", emoji: "🛰️" },
  { title: "Safety Refuters", tagline: "mint/commit changes", emoji: "🛡️" },
  { title: "Phase Maps (CKL)", tagline: "holonomy collapse", emoji: "🧊" },
  { title: "Causal Route‑Finding", tagline: "policy proofs", emoji: "🧠" },
];

export default function OpochLanding() {
  const [applyModalOpen, setApplyModalOpen] = useState(false);
  const [applySource, setApplySource] = useState("hero");
  const [authModalOpen, setAuthModalOpen] = useState(false);
  const [searchParams] = useSearchParams();
  
  useEffect(() => {
    if (searchParams.get('auth') === 'required') {
      setAuthModalOpen(true);
    }
  }, [searchParams]);
  
  const openApplyModal = (source: string) => {
    setApplySource(source);
    setApplyModalOpen(true);
    trackApplyModalOpened(source);
  };

  const homeMetadata = getPageMetadata('home')
  
  return (
    <>
      <SEOHead metadata={homeMetadata} />
      <div className="min-h-screen bg-black text-white">
        <Nav openApply={() => openApplyModal("header-cta")} />
      <Hero openApply={() => openApplyModal("hero")} />
      <Outcomes />
      <Process />
      <Who />
      <Moonshots />
      <Pricing openApply={() => openApplyModal("pricing")} />
      <SF />
      <RBTSection />
      <FAQ />
      <Footer openApply={() => openApplyModal("footer")} />
      
      <ApplyModal
        open={applyModalOpen}
        onOpenChange={setApplyModalOpen}
        source={applySource}
      />
      
      <AuthModal
        open={authModalOpen}
        onOpenChange={setAuthModalOpen}
      />
      </div>
    </>
  );
}

function Nav({ openApply }: { openApply: () => void }) {
  const { user } = useAuth();
  
  return (
    <div className="sticky top-0 z-40 border-b border-white/10 bg-black/80 backdrop-blur">
      <div className="mx-auto flex max-w-7xl items-center justify-between px-4 py-4">
        <div className="flex items-center gap-3">
          <OpochLogo width={100} height={32} />
          <Tag><Sparkles className="h-3 w-3" /> Age of Truth</Tag>
        </div>
        <div className="hidden items-center gap-6 md:flex">
          <a className="text-sm text-white/80 hover:text-white" href="#process">How it works</a>
          <a className="text-sm text-white/80 hover:text-white" href="#who">Who it's for</a>
          <Link to="/moonshots" className="text-sm text-white/80 hover:text-white">Moonshots</Link>
          <a className="text-sm text-white/80 hover:text-white" href="#rbt">RBT</a>
          <Button className="rounded-2xl bg-white text-black hover:bg-white/90" onClick={openApply}>Apply — $11,111/mo</Button>
          {user && <UserMenu />}
        </div>
        <div className="flex items-center gap-2 md:hidden">
          {user && <UserMenu />}
          <Button className="rounded-2xl bg-white text-black hover:bg-white/90" size="sm" onClick={openApply}>Apply</Button>
        </div>
      </div>
    </div>
  );
}

function Hero({ openApply }: { openApply: () => void }) {
  return (
    <section className="relative overflow-hidden">
      <div className="pointer-events-none absolute inset-0 -z-10 bg-[radial-gradient(80%_50%_at_50%_-10%,rgba(27,205,255,0.28),transparent)]" />
      <div className="mx-auto max-w-7xl px-4 py-20">
        <motion.h1 initial={{ opacity: 0, y: 10 }} animate={{ opacity: 1, y: 0 }} transition={{ duration: 0.6 }} className="max-w-4xl text-balance text-5xl font-semibold leading-tight tracking-tight md:text-6xl">
          Get unstuck. Build faster. <span className="bg-gradient-to-r from-opoch-blue to-opoch-cyan-light bg-clip-text text-transparent">Decide with confidence</span>.
        </motion.h1>
        <motion.p initial={{ opacity: 0, y: 10 }} animate={{ opacity: 1, y: 0 }} transition={{ delay: 0.1, duration: 0.6 }} className="mt-5 max-w-2xl text-lg text-white/80">
          For deep‑tech founders, PI‑level researchers, and staff+ engineers who can't afford wrong answers. Bring your hardest problem—we'll give you a plan you can act on.
        </motion.p>
        <div className="mt-8 flex flex-wrap items-center gap-4">
          <Button className="rounded-2xl bg-white text-black hover:bg-white/90" onClick={openApply}><span className="flex items-center gap-2">Apply now <ArrowRight className="h-4 w-4" /></span></Button>
          <Button variant="outline" className="rounded-2xl border-white/20 bg-white/5 text-white hover:bg-white/10" asChild><a href="#process" className="flex items-center gap-2">See how it works</a></Button>
        </div>
      </div>
    </section>
  );
}

function Outcomes() {
  return (
    <section className="border-t border-white/10 bg-white/5" id="outcomes">
      <div className="mx-auto max-w-7xl px-4 py-14">
        <div className="grid gap-6 md:grid-cols-3">
          {outcomes.map((o, i) => (
            <div key={i} className="rounded-2xl border border-white/10 bg-black p-6">
              <div className="flex items-center gap-3"><o.icon className="h-5 w-5 text-opoch-cyan-light" /><p className="font-medium">{o.title}</p></div>
              <p className="mt-2 text-sm text-white/70">{o.blurb}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function Process() {
  const steps = [
    { n: 1, title: "Apply", blurb: "Tell us who you are, the problem, and constraints." },
    { n: 2, title: "Deep dive (SF or remote)", blurb: "We map variables, invariants, and success conditions—in plain language." },
    { n: 3, title: "Delivery", blurb: "Action plan, timelines, and working artifacts. If blocked, we show the fastest way to unblock." },
  ];
  return (
    <section id="process" className="border-t border-white/10">
      <div className="mx-auto max-w-7xl px-4 py-16">
        <h2 className="text-3xl font-semibold">How it works</h2>
        <div className="mt-8 grid gap-6 md:grid-cols-3">
          {steps.map((s) => (
            <div key={s.n} className="rounded-2xl border border-white/10 bg-white/5 p-6">
              <div className="flex items-center gap-3"><div className="flex h-7 w-7 items-center justify-center rounded-full bg-white/10 text-sm">{s.n}</div><p className="font-medium">{s.title}</p></div>
              <p className="mt-2 text-sm text-white/70">{s.blurb}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function Who() {
  return (
    <section id="who" className="border-t border-white/10 bg-white/5">
      <div className="mx-auto max-w-7xl px-4 py-16">
        <h2 className="text-3xl font-semibold">Who it’s for</h2>
        <div className="mt-8 grid gap-6 md:grid-cols-3">
          {who.map((w, i) => (
            <div key={i} className="rounded-2xl border border-white/10 bg-black p-6">
              <div className="flex items-center gap-3"><w.icon className="h-5 w-5 text-opoch-blue" /><p className="font-medium">{w.title}</p></div>
              <p className="mt-2 text-sm text-white/70">{w.blurb}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function Moonshots() {
  return (
    <section id="moonshots" className="border-t border-white/10">
      <div className="mx-auto max-w-7xl px-4 py-16">
        <div className="flex items-end justify-between">
          <h2 className="text-3xl font-semibold">15 Moonshots</h2>
          <Link to="/moonshots" className="text-sm text-white/70 hover:text-white">See all →</Link>
        </div>
        <div className="mt-8 grid gap-4 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4">
          {moonshots.map((m, i) => (
            <motion.div key={i} initial={{ opacity: 0, y: 8 }} whileInView={{ opacity: 1, y: 0 }} viewport={{ once: true }} transition={{ duration: 0.4, delay: i * 0.02 }} className="group rounded-2xl border border-white/10 bg-white/5 p-5 hover:border-white/20">
              <div className="flex items-center justify-between">
                <span className="text-2xl">{m.emoji}</span>
                <ReceiptBadge state="PASS" />
              </div>
              <h3 className="mt-3 text-lg font-semibold tracking-tight">{m.title}</h3>
              <p className="text-sm text-white/70">{m.tagline}</p>
              <div className="mt-4">
                <Button variant="ghost" className="text-white/80 hover:text-white" asChild>
                  <a href={`/moonshots/${encodeURIComponent(m.title.toLowerCase().replace(/\\s+/g, "-"))}`}>Open case <ArrowRight className="ml-1 inline h-4 w-4" /></a>
                </Button>
              </div>
            </motion.div>
          ))}
        </div>
      </div>
    </section>
  );
}

function Pricing({ openApply }: { openApply: () => void }) {
  return (
    <section id="apply" className="border-t border-white/10 bg-white/5">
      <div className="mx-auto max-w-7xl px-4 py-16">
        <div className="grid items-center gap-10 md:grid-cols-2">
          <div>
            <h2 className="text-3xl font-semibold">Membership</h2>
            <p className="mt-2 max-w-xl text-white/80">$11,111/month — for people who are deep in the work. You get a private research seat, on‑site hours in San Francisco, and answers you can act on.</p>
            <ul className="mt-6 grid gap-3 text-white/80">
              <li className="flex items-center gap-2"><CheckCircle2 className="h-4 w-4 text-emerald-300" /> A precise plan you can defend</li>
              <li className="flex items-center gap-2"><CheckCircle2 className="h-4 w-4 text-emerald-300" /> Shortest path to unblock or ship</li>
              <li className="flex items-center gap-2"><CheckCircle2 className="h-4 w-4 text-emerald-300" /> Option to bring your data under NDA</li>
              <li className="flex items-center gap-2"><CheckCircle2 className="h-4 w-4 text-emerald-300" /> On‑site sessions @ 300 4th St, San Francisco</li>
            </ul>
            <div className="mt-8 flex flex-wrap gap-3">
              <Button className="rounded-2xl bg-white text-black hover:bg-white/90" onClick={openApply}>Apply now</Button>
              <Button variant="outline" className="rounded-2xl border-white/20 bg-[#0B0F1A] text-white hover:bg-white/10" asChild><a href="#sf" className="flex items-center gap-2">Visit HQ <Building2 className="h-4 w-4" /></a></Button>
            </div>
          </div>
          <Card className="rounded-2xl border-white/10 bg-[#0B0F1A]">
            <CardContent className="p-6">
              <p className="text-sm text-white/70">What we promise</p>
              <h3 className="mt-1 text-xl font-semibold">You will leave with clarity</h3>
              <p className="mt-2 text_WHITE/70">If the problem is solvable, we show the fastest path. If something blocks you, we point at it and show the cost of removing it. No wasted cycles.</p>
            </CardContent>
          </Card>
        </div>
      </div>
    </section>
  );
}

function SF() {
  return (
    <section id="sf" className="border-t border-white/10">
      <div className="mx-auto max-w-7xl px-4 py-16">
        <div className="grid items-center gap-8 md:grid-cols-2">
          <div>
            <h2 className="text-3xl font-semibold">Downtown SF — 300 4th St</h2>
            <p className="mt-2 max-w-xl text-white/70">Walk in with your dataset. Walk out with a plan. Weekly Opoch Hours: open lab, live refuters, verified runs. Zo Zo Zo.</p>
            <div className="mt-6 grid grid-cols-2 gap-4 text-sm text-white/80">
              <div><p className="text-white/60">Hours</p><p>Thu 6–9 PM • Sat 2–6 PM</p></div>
              <div><p className="text-white/60">Contact</p><p>hello@opoch.com</p></div>
            </div>
          </div>
          <div className="rounded-2xl border border-white/10 bg-white/5 p-6">
            <div className="grid grid-cols-2 gap-4 text-sm text-white/80">
              <div><p className="text-white/60">Address</p><p><a href="https://maps.app.goo.gl/HUUtFwq5LBBjfqDD6" target="_blank" rel="noopener noreferrer" className="underline underline-offset-2 hover:text-white">300 4th St, San Francisco, CA 94107</a></p></div>
              <div><p className="text-white/60">Bring</p><p>asks • data • constraints</p></div>
              <div><p className="text-white/60">Seats</p><p>limited • by application</p></div>
              <div><p className="text-white/60">Vibe</p><p>smart • fast • kind</p></div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function RBTSection() {
  const [open, setOpen] = useState(false);
  return (
    <section id="rbt" className="border-t border-white/10 bg-white/5">
      <div className="mx-auto max-w-7xl px-4 py-16">
        <div className="flex items-center justify-between">
          <h2 className="text-3xl font-semibold">RBT — how it works (for the curious)</h2>
          <button onClick={() => setOpen(!open)} className="inline-flex items-center gap-2 rounded-2xl border border-white/10 bg-white/10 px-3 py-2 text-sm text-white/80 hover:bg-white/20">
            {open ? "Hide details" : "Show details"} <ChevronDown className={classNames("h-4 w-4 transition", open && "rotate-180")} />
          </button>
        </div>
        {open && (
          <div className="mt-8 grid gap-6 md:grid-cols-2">
            <Card className="rounded-2xl border-white/10 bg-[#0B0F1A]">
              <CardContent className="p-6">
                <h3 className="text-xl font-semibold">Plain‑English summary</h3>
                <ul className="mt-3 space-y-2 text-white/80 text-sm leading-relaxed">
                  <li><b>One engine.</b> We reduce the mess to a single motion law and a single accounting book that have to balance.</li>
                  <li><b>If it’s solvable,</b> we find the shortest path to an answer. If it’s blocked, we show exactly what’s missing and what it costs to proceed.</li>
                  <li><b>Everything is checkable.</b> For the skeptics, there’s a technical paper and verifiers to audit runs.</li>
                </ul>
                <div className="mt-4 inline-flex items-center gap-2 text-sm text-white/60">
                  <FileCheck className="h-4 w-4" /> 
                  <a href="https://www.recursivebecoming.info/RBT_v1.0_release.pdf" target="_blank" rel="noopener noreferrer" className="underline underline-offset-4 hover:text-white/80">
                    Read the theoretical RBT paper →
                  </a>
                </div>
              </CardContent>
            </Card>
            <Card className="rounded-2xl border-white/10 bg-[#0B0F1A]">
              <CardContent className="p-6">
                <h3 className="text-xl font-semibold">Ask your query</h3>
                <p className="mt-2 text-sm text-white/70">Bring your hardest technical problem. We'll analyze constraints and show you what's possible.</p>
                <AskBox />
              </CardContent>
            </Card>
          </div>
        )}
      </div>
    </section>
  );
}

function FAQ() {
  const qa = [
    { q: "What do I actually get?", a: "A precise plan, clear trade‑offs, and working artifacts when applicable. If something blocks progress, we show exactly what and how to remove it." },
    { q: "Is my data safe?", a: "Yes. We work under NDA on request and keep runs deterministic and auditable. You control what’s shared." },
    { q: "Who should apply?", a: "People already deep in the work: founders, PIs, staff+ engineers. If the stakes are high and time is short, you’re our people." },
    { q: "Can I visit in person?", a: "Yes. We host weekly Opoch Hours at 300 4th St, SF. Bring your dataset and leave with a plan." },
  ];
  return (
    <section id="faq" className="border-t border-white/10">
      <div className="mx-auto max-w-7xl px-4 py-16">
        <h2 className="text-3xl font-semibold">FAQ</h2>
        <div className="mt-8 grid gap-4 md:grid-cols-2">
          {qa.map((x, i) => (
            <div key={i} className="rounded-2xl border border-white/10 bg-white/5 p-5">
              <div className="flex items-center gap-3"><HelpCircle className="h-5 w-5 text-opoch-cyan-light" /><p className="font-medium">{x.q}</p></div>
              <p className="mt-2 text-sm text-white/70">{x.a}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function Footer({ openApply }: { openApply: () => void }) {
  return (
    <footer className="border-t border-white/10">
      <div className="mx-auto max-w-7xl px-4 py-12">
        <div className="mb-8 text-center">
          <h2 className="text-2xl font-semibold mb-3">Ready to ship faster?</h2>
          <p className="text-white/70 mb-6">Get a precise plan you can defend. Apply for membership.</p>
          <Button className="rounded-2xl bg-white text-black hover:bg-white/90" onClick={openApply}>Apply now →</Button>
        </div>
        <div className="flex flex-wrap items-center justify-between gap-4 text-sm text-white/60 pt-8 border-t border-white/10">
          <p>© {new Date().getFullYear()} Opoch. All rights reserved.</p>
          <div className="flex items-center gap-5">
            <a href="/updates" className="hover:text-white">Updates</a>
            <a href="/privacy" className="hover:text-white">Privacy</a>
            <a href="/terms" className="hover:text-white">Terms</a>
            <a href="/docs" className="hover:text-white">Docs</a>
          </div>
        </div>
        <p className="mt-3 text-xs text-white/50">We help people ship ambitious work faster—with plans they can defend. Not medical, legal, or investment advice.</p>
      </div>
    </footer>
  );
}