import React, { useState, useEffect, lazy, Suspense, useCallback } from "react";
import { useSearchParams } from "react-router-dom";
import { Building2, CheckCircle2, ChevronDown, FileCheck } from "lucide-react";
import { Button } from "./components/ui/button";
import { Card, CardContent } from "./components/ui/card";
import { trackApplyModalOpened } from "./lib/analytics";
import SEOHead from "./components/SEOHead";
import { getPageMetadata } from "./lib/metadata";
import { classNames } from "./lib/utils";
import OpochLogo from "./components/OpochLogo";
import { useAuth } from "./contexts/AuthContext";
import { ExploreWithOpoch } from "./components/ExploreWithOpoch";
import { toastSuccess, toastError } from "./hooks/use-toast";

// Lazy load components for better performance
const Nav = lazy(() => import("./components/v2/Nav"));
const Hero = lazy(() => import("./components/landing/Hero"));
const Outcomes = lazy(() => import("./components/landing/Outcomes"));
const Process = lazy(() => import("./components/landing/Process"));
const Who = lazy(() => import("./components/landing/Who"));
const Moonshots = lazy(() => import("./components/landing/Moonshots"));
const FAQ = lazy(() => import("./components/landing/FAQ"));

// Lazy load modals only when needed
const ApplyModal = lazy(() => import("./components/ApplyModal").then(module => ({ default: module.ApplyModal })));
const AuthModal = lazy(() => import("./components/AuthModal"));

// Loading component for Suspense
const SectionLoader = () => (
  <div className="flex justify-center items-center py-24">
    <div className="animate-pulse text-white/40">Loading...</div>
  </div>
);


export default function OpochLanding() {
  const [applyModalOpen, setApplyModalOpen] = useState(false);
  const [applySource, setApplySource] = useState("hero");
  const [authModalOpen, setAuthModalOpen] = useState(false);
  const [searchParams] = useSearchParams();
  const { user } = useAuth();
  
  useEffect(() => {
    if (searchParams.get('auth') === 'required') {
      setAuthModalOpen(true);
    }
  }, [searchParams]);
  
  const openApplyModal = useCallback((source: string) => {
    setApplySource(source);
    setApplyModalOpen(true);
    trackApplyModalOpened(source);
  }, []);

  const consultingMetadata = getPageMetadata('consulting');

  return (
    <>
      <SEOHead metadata={consultingMetadata} />
      <div className="min-h-screen bg-black text-white">
        <Suspense fallback={<SectionLoader />}>
          <Nav />
        </Suspense>
        
        <main>
          <Suspense fallback={<SectionLoader />}>
            <Hero openApply={() => openApplyModal("hero")} />
          </Suspense>
          
          <Suspense fallback={<SectionLoader />}>
            <Outcomes />
          </Suspense>
          
          <Suspense fallback={<SectionLoader />}>
            <Process />
          </Suspense>
          
          <Suspense fallback={<SectionLoader />}>
            <Who />
          </Suspense>
          
          <Suspense fallback={<SectionLoader />}>
            <Moonshots />
          </Suspense>
          
          <Pricing openApply={() => openApplyModal("pricing")} />
          <SF />
          <RBTSection />
          
          <Suspense fallback={<SectionLoader />}>
            <FAQ />
          </Suspense>
        </main>
        
        <Footer openApply={() => openApplyModal("footer")} />
      </div>
      
      {applyModalOpen && (
        <Suspense fallback={<div />}>
          <ApplyModal
            open={applyModalOpen}
            onOpenChange={setApplyModalOpen}
            source={applySource}
            user={user}
          />
        </Suspense>
      )}
      
      {authModalOpen && (
        <Suspense fallback={<div />}>
          <AuthModal
            open={authModalOpen}
            onOpenChange={setAuthModalOpen}
          />
        </Suspense>
      )}
    </>
  );
}

// Remaining components that are smaller and don't need extraction
const Pricing = React.memo(({ openApply }: { openApply: () => void }) => {
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
              <p className="mt-2 text-white/70">If the problem is solvable, we show the fastest path. If something blocks you, we point at it and show the cost of removing it. No wasted cycles.</p>
            </CardContent>
          </Card>
        </div>
      </div>
    </section>
  );
});

const SF = React.memo(() => {
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
});

const RBTSection = React.memo(() => {
  const [open, setOpen] = useState(false);
  const [authModalOpen, setAuthModalOpen] = useState(false);
  const { user } = useAuth();
  
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
                  <li><b>If it's solvable,</b> we find the shortest path to an answer. If it's blocked, we show exactly what's missing and what it costs to proceed.</li>
                  <li><b>Everything is checkable.</b> For the skeptics, there's a technical paper and verifiers to audit runs.</li>
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
                <ExploreWithOpoch 
                  user={user}
                  onQuerySubmit={() => {
                    toastSuccess('Query submitted!', 'We\'ll analyze your problem and get back to you soon.');
                  }}
                  onError={(error) => {
                    toastError('Failed to submit query', error.message || 'Please try again');
                  }}
                  onSignInClick={() => setAuthModalOpen(true)}
                />
                <div className="mt-4 text-center">
                  <a 
                    href="/moonshots" 
                    className="text-xs text-white/50 hover:text-white/70 underline underline-offset-4"
                  >
                    or browse moonshots →
                  </a>
                </div>
              </CardContent>
            </Card>
          </div>
        )}
      </div>
      
      {authModalOpen && (
        <Suspense fallback={<div />}>
          <AuthModal
            open={authModalOpen}
            onOpenChange={setAuthModalOpen}
          />
        </Suspense>
      )}
    </section>
  );
});

const Footer = React.memo(({ openApply }: { openApply: () => void }) => {
  return (
    <footer className="border-t border-white/10 bg-black">
      <div className="mx-auto max-w-7xl px-4 py-16">
        <div className="mb-4 text-3xl font-semibold">Ready to accelerate?</div>
        <p className="mb-8 max-w-2xl text-white/80">Join Opoch to get precise technical guidance and accelerate your most ambitious projects.</p>
        <Button className="rounded-2xl bg-white text-black hover:bg-white/90" onClick={openApply}>Apply now</Button>
        <div className="mt-16 flex flex-wrap justify-between gap-8 text-sm">
          <div className="flex items-center gap-2 text-white/60">
            <OpochLogo className="h-5 w-5" />
            <span>© 2025 Opoch</span>
          </div>
          <div className="flex gap-6">
            <a href="mailto:hello@opoch.com" className="text-white/60 hover:text-white">Contact</a>
            <a href="/terms" className="text-white/60 hover:text-white">Terms</a>
            <a href="/privacy" className="text-white/60 hover:text-white">Privacy</a>
          </div>
        </div>
      </div>
    </footer>
  );
});

// Add displayNames for debugging
Pricing.displayName = "Pricing";
SF.displayName = "SF";
RBTSection.displayName = "RBTSection";
Footer.displayName = "Footer";