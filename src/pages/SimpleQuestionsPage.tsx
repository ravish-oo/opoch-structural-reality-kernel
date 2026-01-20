import { useState, useCallback, memo, useEffect } from "react"
import { Link } from "react-router-dom"
import { motion } from "framer-motion"
import { ChevronDown, ArrowLeft } from "lucide-react"
import { Analytics } from "@vercel/analytics/react"
import { SpeedInsights } from "@vercel/speed-insights/react"
import { Helmet } from "react-helmet-async"
import Nav from "../components/v2/Nav"
import {
  trackSimpleQuestionsPageViewed,
  trackSimpleQuestionsSectionExpanded,
  trackSimpleQuestionsSectionCollapsed,
  trackSimpleQuestionsChatClicked,
  trackSimpleQuestionsBackClicked
} from "../lib/analytics"
import { config } from "../lib/config"

// Section data structure
interface QA {
  question: string
  answer: string | string[] // Can be string or array of bullet points
}

interface Section {
  id: string
  title: string
  qas: QA[]
}

const SECTIONS: Section[] = [
  {
    id: "first-principles",
    title: "Nothingness and Witnessability",
    qas: [
      { question: "What do you mean by \"nothingness (⊥)\"?", answer: "⊥ means \"no admissible distinctions exist,\" meaning there is no testable difference you are allowed to talk about. It is not empty space or a vacuum, it is the absence of any operationally meaningful separation." },
      { question: "Isn't witnessability (A0) just another assumption?", answer: "It is the minimal condition under which \"true vs false\" can mean anything operational. If you allow untestable distinctions, there is no correction mechanism, so \"truth\" collapses into assertion. The derivation treats witnessability as the smallest admissibility rule that prevents that collapse." },
      { question: "What exactly is forbidden by A0?", answer: "Any claim whose difference cannot be separated by a finite witness and a halting verifier. If you cannot specify what would decide it, you do not get to call it a distinction inside the system." }
    ]
  },
  {
    id: "meaning-computation",
    title: "Meaning, Computation, and Self-Delimiting Code",
    qas: [
      { question: "Why does everything reduce to finite descriptions (D*)?", answer: "Because witnesses must be finite and verifiers must halt. Finite procedures require finite handles. So admissible objects live in finite descriptions." },
      { question: "Why do you force self-delimiting (prefix-free) programs?", answer: "A closed world cannot rely on external file boundaries, lengths, or tokenizers. If \"where the program ends\" depends on an external convention, you minted an extra distinction. Prefix-free is the minimal way to make parsing internal." },
      { question: "Does this assume a specific model of computation?", answer: "No specific machine is privileged. The only requirement is: procedures are totalized to return explicit finite outcomes (including FAIL/TIMEOUT) so \"undefinedness\" does not become an untestable distinction." }
    ]
  },
  {
    id: "ledger-truth",
    title: "Ledger and Truth Closure",
    qas: [
      { question: "Why is the ledger the only history?", answer: "Because a difference is only real in this regime if it is witnessed and recorded. Without a record, there is no stable admissible fact, only uncommitted possibilities." },
      { question: "Why is the ledger order-free?", answer: "Because \"order mattered\" is itself a distinction. If order is not recorded as testable data, then any dependence on order is minted slack." },
      { question: "What is truth here, in one sentence?", answer: "Truth is the partition of possibilities induced by recorded tests. Not a hidden transcript, not a belief, not a narrative." }
    ]
  },
  {
    id: "time-energy",
    title: "Time, Energy, Feasibility Shrink, and ⊥op",
    qas: [
      { question: "Where does time come from in the derivation?", answer: "Time is not assumed. It is defined as the irreversible cost of commitment: when a record shrinks the surviving set of possibilities, you pay an additive time increment." },
      { question: "Why is the cost written as log(|W|/|W′|)?", answer: "Because commitments multiply constraints but costs must add across steps. Log is the forced way to convert multiplicative shrink of alternatives into additive accounting." },
      { question: "Are you claiming physical thermodynamics?", answer: "The derivation claims an irreversibility ledger is forced by witnessable commitments in a closed system. Mapping that ledger to physical thermodynamics is an interpretation layer: in physics, tests correspond to realizable experiments and cost corresponds to physical irreversibility." }
    ]
  },
  {
    id: "observer-control",
    title: "Observer and Control (Orthogonality)",
    qas: [
      { question: "Where is the \"observer\" in this framework?", answer: "The \"observer\" is not a mystical entity. It is the decision mechanism that selects which admissible tests to run and which records to commit, subject to feasibility and cost." },
      { question: "Why do you need the orthogonality law (truth vs control)?", answer: "Because without it, the system can inject arbitrary tie-breaks based on labels or convenience, reintroducing untestable distinctions. Orthogonality forces control to depend only on what is already Π-fixed." },
      { question: "What does \"consciousness as software enforcing nothingness\" mean?", answer: "It means the system refuses to manufacture distinctions. It either derives what is forced, or it returns the boundary of what remains undecided with the next decisive test." }
    ]
  },
  {
    id: "gauge-invariance",
    title: "Gauge and Invariance",
    qas: [
      { question: "What is \"gauge\" in plain language?", answer: "Anything that changes the presentation but not what can be distinguished by feasible tests: renamings, recodings, relabelings, and reorderings of independent events." },
      { question: "Why is gauge forced?", answer: "If a renaming changes \"truth,\" that difference is not testable. Therefore it is slack and must be quotiented out. Only invariants under admissible tests count as real." },
      { question: "Does this mean language is mostly gauge?", answer: "A lot of surface form is gauge. The derivation's goal is to strip away phrasing dependence so the remaining structure is stable under rewording." }
    ]
  },
  {
    id: "omega-frontier",
    title: "Ω Frontier and τ*",
    qas: [
      { question: "What is the Ω frontier?", answer: "It is the exact set of remaining possible answers consistent with the ledger, plus at least one minimal separator test (or an exact budget gap). It is \"unknown\" as a certified object, not prose." },
      { question: "Why is Ω mandatory, why not just guess?", answer: "Because guessing mints distinctions you cannot justify. Under A0 there are only two states: uniquely decided, or not uniquely decided. If not, you must return the boundary." },
      { question: "What is τ*?", answer: "τ* is the canonical next separator: the admissible test that most efficiently collapses the remaining uncertainty under the cost model, without heuristic choice." }
    ]
  },
  {
    id: "local-structure",
    title: "Local Growth via Boundary Flow",
    qas: [
      { question: "If feasibility shrinks, why doesn't everything quickly become indistinguishable?", answer: "Because the derivation distinguishes global closure from local pockets. Locally, subsystems can import usable structure (budget/gradients) from an environment and export irreversibility. Structure can grow locally for long epochs while global irreversibility remains monotone." },
      { question: "What does boundary flow add that the earlier kernel lacked?", answer: "It prevents a false conclusion that \"closed world implies immediate heat death everywhere.\" It formalizes local openness inside global closure and makes life, evolution, and intelligence compatible with the kernel." }
    ]
  },
  {
    id: "practical",
    title: "Practical Interpretation",
    qas: [
      { question: "What would falsify this derivation, on its own terms?", answer: "A counterexample must point to a specific step where an untestable distinction is being introduced while claiming it is forced, or show an alternative closed construction that preserves A0 yet avoids one of the forced structures (finite descriptions, endogenous tests, order-free ledger, quotient truth, gauge coequalization)." },
      { question: "What is the single biggest takeaway?", answer: "If you refuse untestable distinctions, then \"truth\" becomes a closure object induced by recorded tests, and \"unknown\" becomes an executable frontier with a next separator. Everything else is representation." }
    ]
  }
]


// SEO component
const SEOHead = memo(() => (
  <Helmet>
    <title>Frequently Asked Questions | Opoch</title>
    <meta name="description" content="The friction points: nothingness, witnessability, time, observer, gauge, Ω frontier. Questions that recur when reconciling the derivation with existing worldviews." />
    <meta property="og:title" content="Frequently Asked Questions | Opoch" />
    <meta property="og:description" content="The friction points: questions that recur when reconciling the derivation with existing worldviews." />
    <meta property="og:type" content="website" />
  </Helmet>
))

SEOHead.displayName = "SEOHead"

// Q&A Item component (memoized for performance)
const QAItem = memo(({ qa }: { qa: QA }) => {
  const isArrayAnswer = Array.isArray(qa.answer)

  return (
    <div className="space-y-3">
      <h3 className="text-lg font-semibold text-white">{qa.question}</h3>
      {isArrayAnswer ? (
        <ul className="space-y-2">
          {Array.isArray(qa.answer) && qa.answer.map((item: string, i: number) => (
            <li key={i} className="text-white/70 leading-relaxed">
              {item.startsWith("•") || item.startsWith("-") ? item : `• ${item}`}
            </li>
          ))}
        </ul>
      ) : (
        <p className="text-white/70 leading-relaxed">{qa.answer}</p>
      )}
    </div>
  )
})

QAItem.displayName = "QAItem"

// Section component (collapsible, memoized for performance)
const SectionBlock = memo(({ section, isExpanded, onToggle }: { section: Section; isExpanded: boolean; onToggle: () => void }) => {
  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      whileInView={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.5 }}
      viewport={{ once: true }}
      className="rounded-2xl border border-white/10 bg-white/5 overflow-hidden"
    >
      {/* Section Header - Clickable */}
      <button
        onClick={onToggle}
        className="w-full flex items-center justify-between p-6 text-left transition-colors hover:bg-white/10"
        aria-expanded={isExpanded}
      >
        <h2 className="text-2xl font-semibold text-white">{section.title}</h2>
        <ChevronDown
          className={`h-5 w-5 text-white/60 transition-transform duration-200 ${
            isExpanded ? "rotate-180" : ""
          }`}
        />
      </button>

      {/* Section Content - Collapsible */}
      {isExpanded && (
        <div className="border-t border-white/10 p-6 space-y-6">
          {section.qas.map((qa, index) => (
            <QAItem key={index} qa={qa} />
          ))}
        </div>
      )}
    </motion.div>
  )
})

SectionBlock.displayName = "SectionBlock"

export default function SimpleQuestionsPage() {
  const [expandedSections, setExpandedSections] = useState<Record<string, boolean>>({
    // Expand first section by default for better UX
    "first-principles": true
  })

  // Track page view on mount
  useEffect(() => {
    trackSimpleQuestionsPageViewed()
  }, [])

  const toggleSection = useCallback((sectionId: string) => {
    setExpandedSections(prev => {
      const isCurrentlyExpanded = prev[sectionId]
      const newState = !isCurrentlyExpanded

      // Track analytics
      if (newState) {
        trackSimpleQuestionsSectionExpanded(sectionId)
      } else {
        trackSimpleQuestionsSectionCollapsed(sectionId)
      }

      return { ...prev, [sectionId]: newState }
    })
  }, [])

  const handleChatClick = useCallback(() => {
    trackSimpleQuestionsChatClicked()
    window.location.href = config.chatUrl
  }, [])

  const handleBackClick = useCallback(() => {
    trackSimpleQuestionsBackClicked()
  }, [])

  return (
    <>
      <SEOHead />
      <Analytics />
      <SpeedInsights />

      <div className="min-h-screen bg-black text-white">
        {/* Navigation */}
        <Nav />

        {/* Hero Section */}
        <section className="relative overflow-hidden border-b border-white/10 bg-black px-4 py-20 sm:px-6 lg:px-8">
          {/* Gradient Background */}
          <div className="pointer-events-none absolute inset-0 -z-10 bg-[radial-gradient(80%_50%_at_50%_-10%,rgba(27,205,255,0.28),transparent)]"></div>

          <div className="relative mx-auto max-w-4xl">
            {/* Back Button */}
            <Link
              to="/"
              onClick={handleBackClick}
              className="mb-8 inline-flex items-center gap-2 text-sm text-white/60 transition-colors hover:text-white"
            >
              <ArrowLeft className="h-4 w-4" />
              Back to home
            </Link>

            {/* Header */}
            <motion.h1
              initial={{ opacity: 0, y: 10 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.6 }}
              className="mb-6 text-balance text-5xl font-semibold leading-tight tracking-tight text-white md:text-6xl"
            >
              Frequently Asked Questions
            </motion.h1>

            {/* Subheadline */}
            <motion.p
              initial={{ opacity: 0, y: 10 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.1, duration: 0.6 }}
              className="mb-8 max-w-3xl text-lg leading-relaxed text-white/80 md:text-xl"
            >
              The friction points: questions that recur when reconciling the derivation with existing worldviews.
            </motion.p>
          </div>
        </section>

        {/* Main Content - 10 Sections */}
        <section className="px-4 py-16 sm:px-6 lg:px-8">
          <div className="mx-auto max-w-4xl space-y-6">
            {SECTIONS.map((section) => (
              <SectionBlock
                key={section.id}
                section={section}
                isExpanded={expandedSections[section.id] || false}
                onToggle={() => toggleSection(section.id)}
              />
            ))}
          </div>
        </section>


        {/* CTA Section */}
        <section className="border-t border-white/10 px-4 py-24 sm:px-6 lg:px-8">
          <div className="mx-auto max-w-4xl text-center">
            <motion.h2
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5 }}
              viewport={{ once: true }}
              className="mb-8 text-3xl font-semibold text-white"
            >
              Ready to Go Deeper?
            </motion.h2>

            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5, delay: 0.1 }}
              viewport={{ once: true }}
            >
              <button
                onClick={handleChatClick}
                className="rounded-2xl bg-white px-8 py-4 text-base font-medium text-black shadow-lg transition-all hover:bg-white/90"
              >
                Test it Yourself →
              </button>
            </motion.div>
          </div>
        </section>

        {/* Footer - Standard */}
        <footer className="border-t border-white/10 px-4 py-8 sm:px-6 lg:px-8">
          <div className="mx-auto max-w-7xl">
            <div className="flex flex-col items-center justify-between gap-4 sm:flex-row">
              <p className="text-sm text-white/40">
                © 2025 Opoch. All rights reserved.
              </p>
              <div className="flex gap-6">
                <Link to="/" className="text-sm text-white/60 transition-colors hover:text-white">
                  Home
                </Link>
                <Link to="/verify-independently" className="text-sm text-white/60 transition-colors hover:text-white">
                  Verify
                </Link>
                <Link to="/falsify" className="text-sm text-white/60 transition-colors hover:text-white">
                  Falsify
                </Link>
              </div>
            </div>
          </div>
        </footer>
      </div>
    </>
  )
}
