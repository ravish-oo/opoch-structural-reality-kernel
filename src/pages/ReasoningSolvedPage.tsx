import { useEffect } from "react"
import { Link } from "react-router-dom"
import { motion } from "framer-motion"
import { ArrowLeft, ArrowRight, ExternalLink } from "lucide-react"
import { Analytics } from "@vercel/analytics/react"
import { SpeedInsights } from "@vercel/speed-insights/react"
import SEOHead from "../components/SEOHead"
import Nav from "../components/v2/Nav"
import { docsLinks, config } from "../lib/config"

export default function ReasoningSolvedPage() {
  useEffect(() => {
    window.scrollTo(0, 0)
  }, [])

  return (
    <>
      <SEOHead metadata={{
        title: "Reasoning Solved - Opoch",
        description: "The formal theorem: reasoning reduces to test enumeration + projection + ledger. Π/Δ/T contract with proof.",
        image: "/og-image.jpg",
        url: "https://www.opoch.com/reasoning-solved",
        type: "website"
      }} />
      <Analytics />
      <SpeedInsights />

      <div className="min-h-screen bg-black text-white">
        <Nav />

        {/* Breadcrumb */}
        <div className="border-b border-white/10 bg-black/50 backdrop-blur">
          <div className="mx-auto max-w-7xl px-4 py-3">
            <Link
              to="/"
              className="flex items-center gap-2 text-sm text-white/60 hover:text-white transition-colors"
            >
              <ArrowLeft className="h-4 w-4" />
              Back to Home
            </Link>
          </div>
        </div>

        <main className="mx-auto max-w-4xl px-4 py-16 sm:px-6 lg:px-8">

          {/* Header */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6 }}
            className="mb-12 text-center"
          >
            <h1 className="mb-4 text-4xl font-bold text-white md:text-5xl">
              Reasoning Solved
            </h1>
            <p className="text-lg text-white/70">
              Not a claim. A theorem with proof.
            </p>
          </motion.div>

          {/* The Theorem */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="mb-12 rounded-2xl border border-opoch-cyan-light/20 bg-opoch-cyan-light/5 p-8"
          >
            <h2 className="mb-6 text-xl font-semibold text-opoch-cyan-light">Theorem</h2>

            <div className="mb-6 space-y-4 text-white/80 font-mono text-sm leading-relaxed">
              <p><span className="text-white font-semibold">Given:</span></p>
              <ul className="ml-4 space-y-1">
                <li>• a finite description universe D*</li>
                <li>• a lawful test algebra Δ<sub>lawful</sub> with computable cost</li>
                <li>• truth projection Π<sub>c</sub> induced by tests of cost ≤ c</li>
                <li>• truthpoint Π<sub>∞</sub> = lim<sub>c→∞</sub> Π<sub>c</sub></li>
              </ul>
            </div>

            <div className="mb-6 space-y-4 text-white/80 font-mono text-sm leading-relaxed">
              <p><span className="text-white font-semibold">There exists a universal solver</span> such that for every query Q:</p>
            </div>

            <div className="space-y-4">
              <div className="rounded-lg border border-emerald-500/30 bg-emerald-500/10 p-4">
                <p className="font-mono text-sm">
                  <span className="text-emerald-400 font-bold">(A)</span>{" "}
                  <span className="text-emerald-200">
                    If a finite-cost separating test exists → returns <span className="font-semibold">unique Π<sub>∞</sub>-fixed answer with finite witness</span>
                  </span>
                </p>
              </div>

              <div className="rounded-lg border border-amber-500/30 bg-amber-500/10 p-4">
                <p className="font-mono text-sm">
                  <span className="text-amber-400 font-bold">(B)</span>{" "}
                  <span className="text-amber-200">
                    Otherwise → returns <span className="font-semibold">Δ-incomplete certificate</span>: the Π<sub>∞</sub>-consistent answer family + minimal missing distinguisher
                  </span>
                </p>
              </div>
            </div>

            <p className="mt-6 text-white/60 text-sm">
              This exhausts all possibilities. The algorithm is correct by construction.
            </p>
          </motion.div>

          {/* One Line Explanation */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="mb-12"
          >
            <h2 className="mb-4 text-xl font-semibold text-white">In One Line</h2>
            <p className="text-lg text-white/80">
              <span className="text-white font-semibold">Reasoning = test enumeration + projection + ledger.</span>{" "}
              Either the tests resolve it, or they don't — and in both cases you get a precise answer.
            </p>
          </motion.div>

          {/* Why This Is Different */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="mb-12 rounded-2xl border border-white/10 bg-white/5 p-8"
          >
            <h2 className="mb-4 text-xl font-semibold text-white">Current AI vs This</h2>
            <div className="grid gap-6 md:grid-cols-2">
              <div>
                <h3 className="mb-3 text-sm font-semibold text-white/50 uppercase tracking-wide">Current AI</h3>
                <ul className="space-y-2 text-white/70">
                  <li>• Guesses when uncertain</li>
                  <li>• Hallucinates details</li>
                  <li>• Confidently wrong</li>
                  <li>• No witness for claims</li>
                </ul>
              </div>
              <div>
                <h3 className="mb-3 text-sm font-semibold text-opoch-cyan-light/70 uppercase tracking-wide">Solved Reasoning</h3>
                <ul className="space-y-2 text-white/70">
                  <li>• <span className="text-white">Never guesses</span> — knows or says what's missing</li>
                  <li>• <span className="text-white">Never hallucinates</span> — every claim has witness</li>
                  <li>• <span className="text-white">Never wrong without knowing</span> — uncertainty explicit</li>
                  <li>• <span className="text-white">Checkable</span> — proofs you can verify</li>
                </ul>
              </div>
            </div>
          </motion.div>

          {/* The Foundation */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="mb-12"
          >
            <h2 className="mb-4 text-xl font-semibold text-white">The Foundation: Π/Δ/T</h2>
            <p className="mb-4 text-white/70">
              This theorem rests on three primitives that are <span className="text-white">forced by the structure of meaning itself</span>:
            </p>
            <div className="space-y-3 font-mono text-sm">
              <p><span className="text-opoch-cyan-light font-semibold">Π (Truth)</span> — <span className="text-white/70">what survives all valid tests (the Δ-quotient)</span></p>
              <p><span className="text-opoch-cyan-light font-semibold">Δ (Tests)</span> — <span className="text-white/70">meaning requires distinguishers; no test = no difference</span></p>
              <p><span className="text-opoch-cyan-light font-semibold">T (Time)</span> — <span className="text-white/70">irreversible ledger; certainty costs</span></p>
            </div>
            <p className="mt-4 text-white/50 text-sm">
              The kernel proof shows these are the <span className="text-white/70">only consistent primitives</span> — any alternative either mints meaningless differences or discards meaning.
            </p>
          </motion.div>

          {/* Benchmark Evidence */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="mb-12 rounded-2xl border border-white/10 bg-white/5 p-8"
          >
            <h2 className="mb-4 text-xl font-semibold text-white">Evidence</h2>
            <p className="mb-4 text-white/70">
              CritPt Benchmark — 70 research-level physics problems by 50+ physicists:
            </p>
            <div className="flex items-baseline gap-3 mb-2">
              <span className="text-4xl font-bold text-opoch-cyan-light">24%</span>
              <span className="text-white/50">Opoch (GPT-5.2 medium, no checkpoints)</span>
            </div>
            <div className="flex items-baseline gap-3 mb-2">
              <span className="text-2xl font-bold text-white/40">10%</span>
              <span className="text-white/50">GPT-5 high (with checkpoints)</span>
            </div>
            <div className="flex items-baseline gap-3">
              <span className="text-2xl font-bold text-white/40">0%</span>
              <span className="text-white/50">GPT-5.2 medium without framework</span>
            </div>
            <p className="mt-4 text-white/60 text-sm">
              Same model. Same tools. Different reasoning framework. 0% → 24%.
            </p>
            <div className="mt-4 flex flex-wrap gap-3">
              <a
                href={docsLinks.critPt()}
                target="_blank"
                rel="noopener noreferrer"
                className="text-sm text-opoch-cyan-light hover:underline"
              >
                Full results →
              </a>
              <a
                href={docsLinks.reasoningPrompt()}
                target="_blank"
                rel="noopener noreferrer"
                className="text-sm text-opoch-cyan-light hover:underline"
              >
                The prompt (open-sourced) →
              </a>
            </div>
          </motion.div>

          {/* CTA */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="text-center"
          >
            <div className="flex flex-col items-center justify-center gap-4 sm:flex-row">
              <a
                href={docsLinks.reasoningSolved()}
                target="_blank"
                rel="noopener noreferrer"
                className="group flex items-center gap-2 rounded-2xl bg-white px-8 py-4 font-semibold text-black transition-all hover:bg-white/90"
              >
                Read the Full Proof
                <ExternalLink className="h-5 w-5" />
              </a>
              <a
                href={config.chatUrl}
                target="_blank"
                rel="noopener noreferrer"
                className="group flex items-center gap-2 rounded-2xl border border-white/20 bg-white/5 px-8 py-4 font-semibold text-white transition-all hover:border-white/30 hover:bg-white/10"
              >
                Try It
                <ArrowRight className="h-5 w-5 transition-transform group-hover:translate-x-1" />
              </a>
            </div>
            <p className="mt-6 text-white/50 text-sm">
              <a href={docsLinks.kernel()} className="text-opoch-cyan-light hover:underline">
                Read the kernel proof
              </a>
              {" "}— how Π/Δ/T are derived from nothingness
            </p>
          </motion.div>

        </main>
      </div>
    </>
  )
}
