"use client"

import { memo, useCallback, useEffect, useRef } from "react"
import { motion } from "framer-motion"
import { ArrowRight, Check, Minus, ExternalLink } from "lucide-react"
import { BenchmarkChart, type BenchmarkEntry } from "./BenchmarkChart"
import { config, docsLinks } from "../../lib/config"
import {
  trackHomePageViewed,
  trackHomeHeroCTAClicked,
  trackHomeFalsifyCTAClicked,
  trackHomeKernelDocsClicked,
  trackHomeCritPtLinkClicked,
  trackHomeProofBundleClicked,
  trackHomeReasoningPromptClicked,
  trackHomeApplePuzzlesClicked,
  trackHomeSolvesAtScaleClicked,
  trackHomeDerivationClicked,
  trackHomeSectionViewed,
} from "../../lib/analytics"

// Benchmark data - CritPt results
const BENCHMARK_DATA: BenchmarkEntry[] = [
  { name: "Opoch", score: 24.3, isHighlighted: true },
  { name: "GPT-5.1 (high)", score: 12.6 },
  { name: "GPT-5 (high)", score: 10.0 },
  { name: "Gemini-3 Pro", score: 9.1 },
  { name: "GPT-5 (base)", score: 5.7 },
]

interface HeroSectionProps {
  onPrimaryCTA?: () => void
}

function PureHeroSection({ onPrimaryCTA }: HeroSectionProps) {
  // Track page view on mount
  const hasTrackedPageView = useRef(false)
  useEffect(() => {
    if (!hasTrackedPageView.current) {
      trackHomePageViewed()
      hasTrackedPageView.current = true
    }
  }, [])

  // Section visibility tracking refs
  const section0thRef = useRef<HTMLElement>(null)
  const sectionComparisonRef = useRef<HTMLElement>(null)
  const sectionORCRef = useRef<HTMLElement>(null)
  const sectionVisionRef = useRef<HTMLElement>(null)

  // Track section visibility
  useEffect(() => {
    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            const sectionId = entry.target.getAttribute("data-section")
            if (sectionId) {
              trackHomeSectionViewed(sectionId as '0th_principle' | 'comparison' | 'orc_engine' | 'vision')
            }
          }
        })
      },
      { threshold: 0.3 }
    )

    const sections = [section0thRef, sectionComparisonRef, sectionORCRef, sectionVisionRef]
    sections.forEach((ref) => {
      if (ref.current) observer.observe(ref.current)
    })

    return () => observer.disconnect()
  }, [])

  const handlePrimaryCTA = useCallback(() => {
    trackHomeHeroCTAClicked("hero_desktop")
    if (onPrimaryCTA) {
      onPrimaryCTA()
    } else {
      window.open(config.chatUrl, "_blank")
    }
  }, [onPrimaryCTA])

  const handleMobilePrimaryCTA = useCallback(() => {
    trackHomeHeroCTAClicked("mobile_sticky")
    if (onPrimaryCTA) {
      onPrimaryCTA()
    } else {
      window.open(config.chatUrl, "_blank")
    }
  }, [onPrimaryCTA])

  const handleFalsifyCTA = useCallback(() => {
    trackHomeFalsifyCTAClicked("hero_desktop")
  }, [])

  const handleMobileFalsifyCTA = useCallback(() => {
    trackHomeFalsifyCTAClicked("mobile_sticky")
  }, [])

  return (
    <div className="pb-20 md:pb-0">
      {/* ===== SECTION 1: HERO + PROOF (merged) ===== */}
      <section className="relative min-h-screen flex flex-col justify-center overflow-hidden pt-16">
        {/* Background gradient */}
        <div className="pointer-events-none absolute inset-0 -z-10 bg-[radial-gradient(80%_50%_at_50%_-10%,rgba(27,205,255,0.28),transparent)]" />

        <div className="mx-auto max-w-5xl px-4 py-6 md:py-12">
          {/* Announcement Pill */}
          <motion.div
            initial={{ opacity: 0, y: 10 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5 }}
            className="flex justify-center mb-3 md:mb-4"
          >
            <a
              href={docsLinks.opochKernel()}
              target="_blank"
              rel="noopener noreferrer"
              onClick={trackHomeKernelDocsClicked}
              className="group inline-flex items-center gap-1.5 md:gap-2 rounded-full border border-opoch-cyan-light/30 bg-opoch-cyan-light/10 px-3 md:px-4 py-1.5 text-xs md:text-sm text-opoch-cyan-light transition-all hover:border-opoch-cyan-light/50 hover:bg-opoch-cyan-light/15"
            >
              <span className="hidden sm:inline">Derived from the 0<sup className="text-[0.7em]">th</sup> Principle:</span>
              <span className="sm:hidden">0<sup className="text-[0.7em]">th</sup> Principle:</span>
              <span className="font-medium text-white">Null-State Logic</span>
              <span className="transition-transform group-hover:translate-x-0.5">→</span>
            </a>
          </motion.div>

          {/* Main headline */}
          <motion.h1
            initial={{ opacity: 0, y: 10 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.1, duration: 0.6 }}
            className="text-balance text-center text-3xl font-semibold leading-tight tracking-tight text-white sm:text-4xl md:text-5xl lg:text-6xl mb-4 md:mb-6"
          >
            The End of{" "}
            <span className="bg-gradient-to-r from-opoch-blue to-opoch-cyan-light bg-clip-text text-transparent">
              Probabilistic Guessing.
            </span>
          </motion.h1>

          {/* Sub-headline */}
          <motion.p
            initial={{ opacity: 0, y: 10 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.2, duration: 0.6 }}
            className="mx-auto max-w-3xl text-center text-base leading-relaxed text-white/70 md:text-xl mb-4 md:mb-8"
          >
            Reasoning is not statistical. It is a strict binary: either derive the proof, or halt and identify the gap.
          </motion.p>

          {/* Proof headline */}
          <motion.div
            initial={{ opacity: 0, y: 15 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.2, duration: 0.5 }}
            className="text-center mb-3 md:mb-4"
          >
            <h2 className="text-xl sm:text-2xl md:text-3xl font-semibold text-white mb-1 md:mb-2">
              Proof:{" "}
              <span className="text-opoch-cyan-light">24%</span> on CritPt Benchmark
            </h2>
            <p className="text-sm text-white/60 mb-2">
              Unpublished research problems from 50+ physicists. Guess-resistant by design.{" "}
              <a
                href="https://critpt.com"
                target="_blank"
                rel="noopener noreferrer"
                onClick={trackHomeCritPtLinkClicked}
                className="text-opoch-cyan-light hover:underline"
              >
                View benchmark →
              </a>
            </p>
            <p className="text-xs">
              <span className="text-amber-400/70 italic">Official leaderboard update pending</span>
              <span className="text-white/30"> • </span>
              <a
                href={docsLinks.critPt()}
                target="_blank"
                rel="noopener noreferrer"
                onClick={trackHomeProofBundleClicked}
                className="text-opoch-cyan-light hover:underline"
              >
                View Proof Bundle →
              </a>
            </p>
          </motion.div>

          {/* Chart */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.3, duration: 0.5 }}
            className="rounded-2xl border border-white/10 bg-white/5 p-3 md:p-4 mb-3 md:mb-4"
          >
            <BenchmarkChart data={BENCHMARK_DATA} maxScore={30} hideFooter />
          </motion.div>

          {/* Same model, different reasoning */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            transition={{ delay: 0.4, duration: 0.4 }}
            className="text-center mb-3 md:mb-4"
          >
            <p className="text-sm md:text-base text-white/70">
              We applied our Reasoning Protocol to standard LLMs as a constraint. Same model. Same tools. Different contract:{" "}
              <span className="text-white font-medium">Proofs or Gaps, nothing else.</span>{" "}
              <span className="text-white/50">0% → 24%.</span>{" "}
              <a
                href={docsLinks.reasoningPrompt()}
                target="_blank"
                rel="noopener noreferrer"
                onClick={trackHomeReasoningPromptClicked}
                className="text-opoch-cyan-light hover:underline"
              >
                The reasoning prompt (open-sourced) →
              </a>
            </p>
          </motion.div>

          {/* Apple Puzzles - Complexity Scaling Proof */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            transition={{ delay: 0.45, duration: 0.4 }}
            className="text-center mb-4 md:mb-6"
          >
            <p className="text-sm md:text-base text-white/70">
              <span className="text-white font-semibold">Proof:{" "}
                <a
                  href="https://arxiv.org/abs/2506.06941"
                  target="_blank"
                  rel="noopener noreferrer"
                  onClick={trackHomeApplePuzzlesClicked}
                  className="inline-flex items-center gap-1 text-opoch-cyan-light hover:text-opoch-cyan-light/80 transition-colors"
                >
                  Apple Puzzles
                  <ExternalLink className="h-3.5 w-3.5" />
                </a>
              </span>{" "}
              <span className="text-white/50">(Illusion of Thinking)</span> — Same protocol. It turns LLMs from breaks at scale to{" "}
              <a
                href={docsLinks.applePuzzles()}
                target="_blank"
                rel="noopener noreferrer"
                onClick={trackHomeSolvesAtScaleClicked}
                className="text-opoch-cyan-light hover:underline"
              >
                solves at scale →
              </a>
            </p>
          </motion.div>

          {/* CTAs */}
          <motion.div
            initial={{ opacity: 0, y: 10 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.5, duration: 0.5 }}
            className="flex flex-col sm:flex-row items-center justify-center gap-3 sm:gap-4"
          >
            <button
              onClick={handlePrimaryCTA}
              className="group flex items-center gap-2 rounded-2xl bg-white px-6 py-3 md:px-8 md:py-4 font-medium text-black transition-all hover:bg-white/90"
            >
              Try for Yourself
              <ArrowRight className="h-5 w-5 transition-transform group-hover:translate-x-1" />
            </button>
            <a
              href="/falsify"
              onClick={handleFalsifyCTA}
              className="group relative flex items-center gap-2 rounded-2xl border border-white/20 bg-white/5 px-6 py-3 md:px-8 md:py-4 font-medium text-white transition-all hover:border-white/40 hover:bg-white/10"
            >
              <span className="absolute -top-2 -right-2 rounded-full bg-amber-500 px-2 py-0.5 text-[10px] font-bold text-black">Win $50k</span>
              Falsify Us
              <ArrowRight className="h-5 w-5 transition-transform group-hover:translate-x-1" />
            </a>
          </motion.div>
        </div>
      </section>

      {/* ===== SECTION 2: THE 0TH PRINCIPLE ===== */}
      <section ref={section0thRef} data-section="0th_principle" className="border-t border-white/10 py-24">
        <div className="mx-auto max-w-5xl px-4">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="text-center mb-12"
          >
            <h2 className="text-balance text-3xl font-semibold leading-tight tracking-tight text-white md:text-4xl mb-4">
              How? The Shift —{" "}
              <span className="bg-gradient-to-r from-opoch-blue to-opoch-cyan-light bg-clip-text text-transparent">
                0<span className="text-[0.65em] align-super">th</span> Principle
              </span>
            </h2>
            <p className="mx-auto max-w-3xl text-lg leading-relaxed text-white/60">
              We Refused to Assume. By adhering to the void, we derived <span className="text-white">Null-State Logic</span>.
            </p>
            <p className="mx-auto max-w-2xl text-base leading-relaxed text-white/50 mt-2">
              When you delete all unverified priors, Structure isn't invented. It is <span className="text-white">Forced</span>. Operationally. Inevitably.
            </p>
          </motion.div>

          {/* 3-Column Layout */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5, delay: 0.1 }}
            className="grid md:grid-cols-3 gap-6 mb-10"
          >
            {/* Column 1 */}
            <div className="rounded-2xl border border-white/10 bg-white/5 p-6">
              <div className="text-base text-opoch-cyan-light font-medium mb-2">The Constraint</div>
              <div className="inline-block rounded-md bg-white/10 px-2 py-0.5 text-xs text-white/50 uppercase tracking-wide mb-4">No Unverified Priors</div>
              <p className="text-white/70 leading-relaxed mb-3">
                Start from absolute zero. With no information, you cannot simply declare that things exist or differ. You have no basis for it yet.
              </p>
              <p className="text-white/70 leading-relaxed">
                You are forced to admit reality one piece at a time, under one rule: <span className="text-white">A difference is real only if there is a specific mechanism to tell 'A' from 'B'.</span>
              </p>
            </div>

            {/* Column 2 */}
            <div className="rounded-2xl border border-white/10 bg-white/5 p-6">
              <div className="text-base text-opoch-cyan-light font-medium mb-2">The Mechanism</div>
              <div className="inline-block rounded-md bg-white/10 px-2 py-0.5 text-xs text-white/50 uppercase tracking-wide mb-4">Tests Create Facts</div>
              <p className="text-white/70 leading-relaxed mb-3">
                That "mechanism" is an executable test that outputs a finite result. <span className="text-white">No test = No meaning.</span>
              </p>
              <p className="text-white/70 leading-relaxed">
                When a test runs, the result is recorded. This record is a fact. <span className="text-white">Facts are not opinions; they are permanent entries in the ledger.</span>
              </p>
            </div>

            {/* Column 3 */}
            <div className="rounded-2xl border border-white/10 bg-white/5 p-6">
              <div className="text-base text-opoch-cyan-light font-medium mb-2">The Consequence</div>
              <div className="inline-block rounded-md bg-white/10 px-2 py-0.5 text-xs text-white/50 uppercase tracking-wide mb-4">The Ω Stop</div>
              <p className="text-white/70 leading-relaxed mb-3">
                Truth is the closure of the ledger—what remains possible when the testing stops.
              </p>
              <p className="text-white/70 leading-relaxed">
                If the answer isn't unique, the system halts. It refuses to guess and outputs the <span className="text-white">Ω-Frontier</span>—the exact missing test needed to decide.
              </p>
            </div>
          </motion.div>

          {/* CTA */}
          <motion.div
            initial={{ opacity: 0 }}
            whileInView={{ opacity: 1 }}
            viewport={{ once: true }}
            transition={{ duration: 0.4, delay: 0.2 }}
            className="text-center"
          >
            <a
              href={docsLinks.theDerivation()}
              target="_blank"
              rel="noopener noreferrer"
              onClick={trackHomeDerivationClicked}
              className="inline-flex items-center gap-2 text-sm text-opoch-cyan-light hover:underline"
            >
              Read the "Nothingness" Derivation →
            </a>
          </motion.div>
        </div>
      </section>

      {/* ===== SECTION 3: BEYOND GENERATIVE AI (Comparison Table) ===== */}
      <section ref={sectionComparisonRef} data-section="comparison" className="border-t border-white/10 bg-white/5 py-24">
        <div className="mx-auto max-w-5xl px-4">
          {/* Header */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="text-center mb-12"
          >
            <h2 className="text-balance text-3xl font-semibold leading-tight tracking-tight text-white md:text-4xl mb-4">
              Beyond Generative AI:{" "}
              <span className="bg-gradient-to-r from-opoch-blue to-opoch-cyan-light bg-clip-text text-transparent">
                Opoch Reality Compiler
              </span>
            </h2>
            <p className="mx-auto max-w-3xl text-lg leading-relaxed text-white/60">
              Generative AI outputs text. <span className="text-white font-medium">ORC</span> outputs a decision object.
            </p>
          </motion.div>

          {/* Comparison Table */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5, delay: 0.1 }}
            className="rounded-2xl border border-white/10 bg-black overflow-hidden"
          >
            {/* Table Header */}
            <div className="grid grid-cols-3 border-b border-white/10">
              <div className="p-4 text-sm font-medium text-white/50">Feature</div>
              <div className="p-4 text-sm font-medium text-white/50 border-l border-white/10">Generative AI (Current)</div>
              <div className="p-4 text-sm font-medium text-opoch-cyan-light border-l border-white/10">Opoch Reality Compiler (Next)</div>
            </div>

            {/* Row 1: Output */}
            <div className="grid grid-cols-3 border-b border-white/10">
              <div className="p-4 text-sm font-medium text-white">Output</div>
              <div className="p-4 border-l border-white/10">
                <div className="flex items-center gap-2">
                  <Minus className="h-3.5 w-3.5 text-amber-400/70 shrink-0" />
                  <p className="text-sm text-white/50">Plausible completion</p>
                </div>
              </div>
              <div className="p-4 border-l border-emerald-500/20 bg-emerald-500/5">
                <div className="flex items-center gap-2">
                  <Check className="h-3.5 w-3.5 text-emerald-400 shrink-0" />
                  <p className="text-sm text-white">UNIQUE or Ω object</p>
                </div>
              </div>
            </div>

            {/* Row 2: When unsure */}
            <div className="grid grid-cols-3 border-b border-white/10">
              <div className="p-4 text-sm font-medium text-white">When unsure</div>
              <div className="p-4 border-l border-white/10">
                <div className="flex items-center gap-2">
                  <Minus className="h-3.5 w-3.5 text-amber-400/70 shrink-0" />
                  <p className="text-sm text-white/50">Hedges or guesses</p>
                </div>
              </div>
              <div className="p-4 border-l border-emerald-500/20 bg-emerald-500/5">
                <div className="flex items-center gap-2">
                  <Check className="h-3.5 w-3.5 text-emerald-400 shrink-0" />
                  <p className="text-sm text-white">Stops and returns frontier</p>
                </div>
              </div>
            </div>

            {/* Row 3: Verification */}
            <div className="grid grid-cols-3 border-b border-white/10">
              <div className="p-4 text-sm font-medium text-white">Verification</div>
              <div className="p-4 border-l border-white/10">
                <div className="flex items-center gap-2">
                  <Minus className="h-3.5 w-3.5 text-amber-400/70 shrink-0" />
                  <p className="text-sm text-white/50">Optional, manual</p>
                </div>
              </div>
              <div className="p-4 border-l border-emerald-500/20 bg-emerald-500/5">
                <div className="flex items-center gap-2">
                  <Check className="h-3.5 w-3.5 text-emerald-400 shrink-0" />
                  <p className="text-sm text-white">Witness + verifier + receipt</p>
                </div>
              </div>
            </div>

            {/* Row 4: Stability */}
            <div className="grid grid-cols-3 border-b border-white/10">
              <div className="p-4 text-sm font-medium text-white">Stability</div>
              <div className="p-4 border-l border-white/10">
                <div className="flex items-center gap-2">
                  <Minus className="h-3.5 w-3.5 text-amber-400/70 shrink-0" />
                  <p className="text-sm text-white/50">Sensitive to phrasing</p>
                </div>
              </div>
              <div className="p-4 border-l border-emerald-500/20 bg-emerald-500/5">
                <div className="flex items-center gap-2">
                  <Check className="h-3.5 w-3.5 text-emerald-400 shrink-0" />
                  <p className="text-sm text-white">Gauge-invariant closure</p>
                </div>
              </div>
            </div>

            {/* Row 5: Memory */}
            <div className="grid grid-cols-3 border-b border-white/10">
              <div className="p-4 text-sm font-medium text-white">Memory</div>
              <div className="p-4 border-l border-white/10">
                <div className="flex items-center gap-2">
                  <Minus className="h-3.5 w-3.5 text-amber-400/70 shrink-0" />
                  <p className="text-sm text-white/50">Window plus retrieval</p>
                </div>
              </div>
              <div className="p-4 border-l border-emerald-500/20 bg-emerald-500/5">
                <div className="flex items-center gap-2">
                  <Check className="h-3.5 w-3.5 text-emerald-400 shrink-0" />
                  <p className="text-sm text-white">Energy-based, causal activation</p>
                </div>
              </div>
            </div>

            {/* Row 6: Cost */}
            <div className="grid grid-cols-3">
              <div className="p-4 text-sm font-medium text-white">Cost</div>
              <div className="p-4 border-l border-white/10">
                <div className="flex items-center gap-2">
                  <Minus className="h-3.5 w-3.5 text-amber-400/70 shrink-0" />
                  <p className="text-sm text-white/50">Opaque</p>
                </div>
              </div>
              <div className="p-4 border-l border-emerald-500/20 bg-emerald-500/5">
                <div className="flex items-center gap-2">
                  <Check className="h-3.5 w-3.5 text-emerald-400 shrink-0" />
                  <p className="text-sm text-white">Explicit, costed tests</p>
                </div>
              </div>
            </div>
          </motion.div>

          {/* Footer line */}
          <motion.p
            initial={{ opacity: 0 }}
            whileInView={{ opacity: 1 }}
            viewport={{ once: true }}
            transition={{ duration: 0.4, delay: 0.2 }}
            className="text-center text-sm md:text-base text-white/70 mt-8"
          >
            Same interface, different guarantee.
          </motion.p>
        </div>
      </section>

      {/* ===== SECTION 4: THE ORC ENGINE ===== */}
      <section ref={sectionORCRef} data-section="orc_engine" className="border-t border-white/10 py-16">
        <div className="mx-auto max-w-5xl px-4">
          {/* Header */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="text-center mb-8"
          >
            <h2 className="text-balance text-3xl font-semibold leading-tight tracking-tight text-white md:text-4xl mb-3">
              How ORC Works:{" "}
              <span className="bg-gradient-to-r from-opoch-blue to-opoch-cyan-light bg-clip-text text-transparent">
                Derivational Intelligence.
              </span>
            </h2>
            <p className="mx-auto max-w-3xl text-base leading-relaxed text-white/60">
              <span className="text-white">Intelligence as forced structure</span> that derives what is true, and exposes what would make it true.
            </p>
          </motion.div>

          {/* Process Flow - 2x2 Grid */}
          <div className="max-w-4xl mx-auto grid md:grid-cols-2 gap-4">
            {/* Step 1: Compile */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5, delay: 0.1 }}
              className="rounded-2xl border border-white/10 bg-white/5 p-5"
            >
              <div className="flex items-center gap-2 mb-2">
                <div className="flex h-6 w-6 items-center justify-center rounded-full bg-opoch-cyan-light/20 text-opoch-cyan-light font-semibold text-xs">1</div>
                <h3 className="text-base text-white font-semibold">Compile</h3>
              </div>
              <p className="text-white/70 leading-relaxed text-sm mb-2">
                Your input is compiled into a finite decision object: <span className="text-white">hypotheses, witnesses, verifiers, cost, budget</span>.
              </p>
              <p className="text-white/50 leading-relaxed text-xs">
                If the question is underspecified, ORC returns an Ω frontier of what remains possible, instead of guessing.
              </p>
            </motion.div>

            {/* Step 2: Close */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5, delay: 0.15 }}
              className="rounded-2xl border border-white/10 bg-white/5 p-5"
            >
              <div className="flex items-center gap-2 mb-2">
                <div className="flex h-6 w-6 items-center justify-center rounded-full bg-opoch-cyan-light/20 text-opoch-cyan-light font-semibold text-xs">2</div>
                <h3 className="text-base text-white font-semibold">Close</h3>
              </div>
              <p className="text-white/70 leading-relaxed text-sm mb-2">
                ORC applies recorded tests and receipts to collapse what is <span className="text-white">invariant</span>.
              </p>
              <p className="text-white/50 leading-relaxed text-xs">
                Truth is what survives verification under the ledger, independent of phrasing.
              </p>
            </motion.div>

            {/* Step 3: Separate */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5, delay: 0.2 }}
              className="rounded-2xl border border-white/10 bg-white/5 p-5"
            >
              <div className="flex items-center gap-2 mb-2">
                <div className="flex h-6 w-6 items-center justify-center rounded-full bg-opoch-cyan-light/20 text-opoch-cyan-light font-semibold text-xs">3</div>
                <h3 className="text-base text-white font-semibold">Separate</h3>
              </div>
              <p className="text-white/70 leading-relaxed text-sm mb-2">
                If a unique result is forced, ORC returns it with a <span className="text-white">witness and a replayable verifier</span>.
              </p>
              <p className="text-white/50 leading-relaxed text-xs">
                If not, ORC returns the <span className="text-white">Ω frontier</span> and the minimal separator test τ*, plus the exact gap.
              </p>
            </motion.div>

            {/* Step 4: Receipt */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5, delay: 0.25 }}
              className="rounded-2xl border border-white/10 bg-white/5 p-5"
            >
              <div className="flex items-center gap-2 mb-2">
                <div className="flex h-6 w-6 items-center justify-center rounded-full bg-opoch-cyan-light/20 text-opoch-cyan-light font-semibold text-xs">4</div>
                <h3 className="text-base text-white font-semibold">Receipt</h3>
              </div>
              <p className="text-white/70 leading-relaxed text-sm mb-2">
                Every derivation emits a <span className="text-white">cryptographic receipt</span>.
              </p>
              <p className="text-white/50 leading-relaxed text-xs">
                The same result can be verified later without repeating the work. Attribution belongs to the derivation.
              </p>
            </motion.div>
          </div>

          {/* Closer */}
          <motion.p
            initial={{ opacity: 0 }}
            whileInView={{ opacity: 1 }}
            viewport={{ once: true }}
            transition={{ duration: 0.4, delay: 0.3 }}
            className="text-center text-sm md:text-base text-white/70 mt-6 max-w-2xl mx-auto"
          >
            Inside the engine, energy-based causal memory keeps relevant structure active, while irrelevant structure cools away.
          </motion.p>
        </div>
      </section>

      {/* ===== SECTION 5: THE VISION ===== */}
      <section ref={sectionVisionRef} data-section="vision" className="border-t border-white/10 bg-white/5 py-24">
        <div className="mx-auto max-w-5xl px-4">
          {/* Header */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="text-center mb-12"
          >
            <h2 className="text-balance text-3xl font-semibold leading-tight tracking-tight text-white md:text-4xl mb-4">
              Entering{" "}
              <span className="bg-gradient-to-r from-opoch-blue to-opoch-cyan-light bg-clip-text text-transparent">
                Opoch: The Age of Truth.
              </span>
            </h2>
            <p className="mx-auto max-w-3xl text-lg leading-relaxed text-white/60">
              We are building the infrastructure for a <span className="text-white font-medium">Verified Economy</span>, where truth is an asset, not a probability.
            </p>
          </motion.div>

          {/* Vertical Timeline Roadmap */}
          <div className="max-w-3xl mx-auto">
            {/* Stage 1: Now */}
            <motion.div
              initial={{ opacity: 0, x: -20 }}
              whileInView={{ opacity: 1, x: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5, delay: 0.1 }}
              className="relative pl-8 pb-12 border-l-2 border-emerald-500/30"
            >
              {/* Timeline dot */}
              <div className="absolute left-0 top-0 -translate-x-1/2 flex h-4 w-4 items-center justify-center rounded-full bg-emerald-500 ring-4 ring-emerald-500/20" />

              <div className="inline-block rounded-md bg-emerald-500/20 px-2 py-0.5 text-xs text-emerald-300 uppercase tracking-wide mb-3">Now</div>
              <h3 className="text-xl text-white font-semibold mb-2">Inference is Work</h3>
              <p className="text-white/70 leading-relaxed mb-3">
                Training runs are obsolete. The future is <span className="text-white">Heavy Inference</span>.
              </p>
              <p className="text-white/60 leading-relaxed">
                Real intelligence isn't about memorizing the internet; it's about expending energy to <span className="text-white">collapse ambiguity</span> into certainty. We value the <span className="italic">work</span> of thinking, not the <span className="italic">storage</span> of data.
              </p>
            </motion.div>

            {/* Stage 2: Next */}
            <motion.div
              initial={{ opacity: 0, x: -20 }}
              whileInView={{ opacity: 1, x: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5, delay: 0.2 }}
              className="relative pl-8 pb-12 border-l-2 border-amber-500/30"
            >
              {/* Timeline dot */}
              <div className="absolute left-0 top-0 -translate-x-1/2 flex h-4 w-4 items-center justify-center rounded-full bg-amber-500 ring-4 ring-amber-500/20" />

              <div className="inline-block rounded-md bg-amber-500/20 px-2 py-0.5 text-xs text-amber-300 uppercase tracking-wide mb-3">Next</div>
              <h3 className="text-xl text-white font-semibold mb-2">Compute Once, Verify Forever</h3>
              <p className="text-white/70 leading-relaxed mb-3">
                Why solve the same problem twice? When the GITM derives a truth, it mints a <span className="text-white">Cryptographic Receipt</span>. This "Invariant" is stored on a global Layer 1 Ledger.
              </p>
              <div className="rounded-xl border border-white/10 bg-white/5 p-4 mb-3">
                <div className="flex items-center gap-4 text-sm">
                  <div className="flex-1">
                    <span className="text-white/50">First User:</span>
                    <span className="text-white ml-2">Pays energy to derive</span>
                  </div>
                  <div className="text-white/30">→</div>
                  <div className="flex-1">
                    <span className="text-white/50">Next Users:</span>
                    <span className="text-emerald-300 ml-2">Pay fraction to verify</span>
                  </div>
                </div>
              </div>
              <p className="text-white/60 leading-relaxed">
                A global library of cached truth that gets <span className="text-white">faster and cheaper</span> over time.
              </p>
            </motion.div>

            {/* Stage 3: Future */}
            <motion.div
              initial={{ opacity: 0, x: -20 }}
              whileInView={{ opacity: 1, x: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5, delay: 0.3 }}
              className="relative pl-8 border-l-2 border-opoch-cyan-light/30"
            >
              {/* Timeline dot */}
              <div className="absolute left-0 top-0 -translate-x-1/2 flex h-4 w-4 items-center justify-center rounded-full bg-opoch-cyan-light ring-4 ring-opoch-cyan-light/20" />

              <div className="inline-block rounded-md bg-opoch-cyan-light/20 px-2 py-0.5 text-xs text-opoch-cyan-light uppercase tracking-wide mb-3">Future</div>
              <h3 className="text-xl text-white font-semibold mb-2">Fair Attribution</h3>
              <p className="text-white/70 leading-relaxed mb-3">
                In the Age of Truth, the creator is the one who did the <span className="text-white">Derivation</span>.
              </p>
              <p className="text-white/60 leading-relaxed">
                If your node collapses a complex description (e.g., a new protein fold or mathematical proof), the ledger permanently attributes that "Truth Asset" to you. Moving from an Attention Economy (Clicks) to a <span className="text-white">Verification Economy (Proof)</span>.
              </p>
            </motion.div>
          </div>
        </div>
      </section>

      {/* Sticky Mobile CTA */}
      <div className="fixed bottom-0 left-0 right-0 z-40 md:hidden">
        <div className="bg-gradient-to-t from-black via-black/95 to-transparent px-4 pb-4 pt-8">
          <div className="flex gap-2">
            <button
              onClick={handleMobilePrimaryCTA}
              className="group flex flex-1 items-center justify-center gap-2 rounded-2xl bg-white px-4 py-3.5 font-medium text-black transition-all hover:bg-white/90"
            >
              Try It
              <ArrowRight className="h-4 w-4 transition-transform group-hover:translate-x-1" />
            </button>
            <a
              href="/falsify"
              onClick={handleMobileFalsifyCTA}
              className="group relative flex flex-1 items-center justify-center gap-2 rounded-2xl border border-white/20 bg-white/10 px-4 py-3.5 font-medium text-white transition-all"
            >
              <span className="absolute -top-1.5 -right-1 rounded-full bg-amber-500 px-1.5 py-0.5 text-[9px] font-bold text-black">Win $50k</span>
              Falsify
            </a>
          </div>
        </div>
      </div>
    </div>
  )
}

export const HeroSection = memo(PureHeroSection)
