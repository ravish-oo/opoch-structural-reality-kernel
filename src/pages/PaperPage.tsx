import { useEffect, memo, useCallback, useState } from "react"
import { Link } from "react-router-dom"
import { motion } from "framer-motion"
import { ArrowLeft, Download, ExternalLink, CheckCircle2, Copy, Check } from "lucide-react"
import { Analytics } from "@vercel/analytics/react"
import { SpeedInsights } from "@vercel/speed-insights/react"
import SEOHead from "../components/SEOHead"
import Nav from "../components/v2/Nav"
import {
  trackPaperPageViewed,
  trackPaperBackClicked,
  trackPaperPDFDownloadClicked,
  trackPaperPDFViewClicked,
  trackPaperCitationCopied,
  trackPaperRelatedLinkClicked
} from "../lib/analytics"

// Plain English content sections
const PLAIN_ENGLISH_SECTIONS = [
  {
    id: "three-rules",
    title: "Three Rules (Nothing Fancy)",
    items: [
      {
        label: "Truth",
        content: "Saying the same thing in different words doesn't make it \"more true.\" Clean the statement once; cleaning it again shouldn't change it."
      },
      {
        label: "Exact balance",
        content: "Every real change is powered by something. If you reduce uncertainty or lock in a fact, you pay (energy/time). No free lunch, no leftovers."
      },
      {
        label: "Gluing",
        content: "When you join parts, their shared edge must match. Like puzzle pieces: if the edges disagree, the whole picture can't be right."
      }
    ]
  },
  {
    id: "two-kinds",
    title: "Two Kinds of Change",
    items: [
      {
        label: "Free rearrangements",
        content: "Look different, mean the same. Costs nothing. (New wording, new viewpoint, camera angle, unit change.)"
      },
      {
        label: "Paid updates",
        content: "Real edits to what you know. They shrink uncertainty and cost energy/time. Each settled yes/no is a real, paid step."
      }
    ]
  },
  {
    id: "observer-observed",
    title: "\"Observer = Observed\" (What It Really Means)",
    bullets: [
      "Treat \"you + world\" as one system. Changing viewpoint is a free rearrangement; it shouldn't create or remove facts.",
      "Only paid steps (measurements, commitments, decisions) move the joint truth forward.",
      "Close any tiny loop \"look → act → look\": if it returns you exactly to where you started (no wobble), your steps were consistent. If there's a mismatch, you minted a difference or left a debt."
    ]
  },
  {
    id: "speed",
    title: "Speed of Understanding (Why Power Matters)",
    content: "Learning has a fuel meter: locking in facts costs at least a tiny fixed amount per bit. With limited power, there's a hard cap on how fast your uncertainty can drop. More power → faster, guaranteed progress; less power → slower."
  },
  {
    id: "fpg",
    title: "Fixed-Point Gauge (FPG), in One Idea",
    content: "Pick the viewpoint that removes unnecessary cross-talk between \"you\" and \"world,\" so you can solve the whole thing in one pass. Think: rotate the knobs so each knob controls just one thing. Sometimes you can't kill all cross-talk; then you align to make it as small as possible. Either way, life gets simpler and faster."
  },
  {
    id: "checklist",
    title: "Pocket Checklist (Receipts You Can Actually Run)",
    checks: [
      { label: "Rephrase test", text: "Did a rewording change the meaning? If yes, you minted a difference—fix it." },
      { label: "No-free-lunch test", text: "Did your certainty go up without spending anything? Then something's off—find the hidden cost." },
      { label: "Seam test", text: "Do the parts agree on their shared boundary? If not, don't trust the whole." },
      { label: "Loop test", text: "Do \"observe → act → observe\" loops come back clean? If not, you've got wobble to remove." },
      { label: "Power→progress", text: "Are you learning at a rate your power budget can actually support? If not, expectations are wrong." }
    ]
  }
]

const CONCLUSION = "That's the whole engine, simply: don't mint fake differences, pay exactly for real updates, make parts meet cleanly, and choose the viewpoint that cuts cross-talk. Then \"observer is observed\" isn't a slogan—it's how you move truth forward, at the exact speed your power allows."

const CITATION_TEXT = 'Chetan Chauhan and Dharamveer Chouhan. "A Theory of Everything." Opoch, 2025.'

// Section component (memoized for performance)
const PlainEnglishSection = memo(({ section }: { section: typeof PLAIN_ENGLISH_SECTIONS[0] }) => {
  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      whileInView={{ opacity: 1, y: 0 }}
      viewport={{ once: true }}
      transition={{ duration: 0.5 }}
      className="mb-12"
    >
      <h3 className="mb-4 text-2xl font-semibold text-white">{section.title}</h3>

      {section.items && (
        <div className="space-y-4">
          {section.items.map((item, index) => (
            <div key={index}>
              <span className="font-semibold text-opoch-cyan-light">{item.label}:</span>{" "}
              <span className="text-white/80">{item.content}</span>
            </div>
          ))}
        </div>
      )}

      {section.bullets && (
        <ul className="space-y-3">
          {section.bullets.map((bullet, index) => (
            <li key={index} className="flex gap-3">
              <span className="mt-1 text-opoch-cyan-light">•</span>
              <span className="text-white/80">{bullet}</span>
            </li>
          ))}
        </ul>
      )}

      {section.content && (
        <p className="leading-relaxed text-white/80">{section.content}</p>
      )}

      {section.checks && (
        <div className="space-y-3">
          {section.checks.map((check, index) => (
            <div key={index} className="flex gap-3">
              <CheckCircle2 className="h-5 w-5 flex-shrink-0 text-emerald-400" />
              <div>
                <span className="font-semibold text-white">{check.label}:</span>{" "}
                <span className="text-white/80">{check.text}</span>
              </div>
            </div>
          ))}
        </div>
      )}
    </motion.div>
  )
})

PlainEnglishSection.displayName = "PlainEnglishSection"

export default function PaperPage() {
  const [citationCopied, setCitationCopied] = useState(false)

  useEffect(() => {
    window.scrollTo(0, 0)
    trackPaperPageViewed()
  }, [])

  const handleBackClick = useCallback(() => {
    trackPaperBackClicked()
  }, [])

  const handlePDFDownloadClick = useCallback(() => {
    trackPaperPDFDownloadClicked()
  }, [])

  const handlePDFViewClick = useCallback(() => {
    trackPaperPDFViewClicked()
  }, [])

  const handleCitationCopy = useCallback(async () => {
    try {
      await navigator.clipboard.writeText(CITATION_TEXT)
      trackPaperCitationCopied()
      setCitationCopied(true)
      setTimeout(() => setCitationCopied(false), 2000) // Reset after 2 seconds
    } catch (err) {
      console.error('Failed to copy citation:', err)
    }
  }, [])

  const handleRelatedLinkClick = useCallback((destination: string) => {
    trackPaperRelatedLinkClicked(destination)
  }, [])

  return (
    <>
      <SEOHead metadata={{
        title: "The Paper - Opoch",
        description: "The complete Theory of Everything—plain English first, then the full technical version.",
        image: "/og-image.jpg",
        url: "https://www.opoch.com/paper",
        type: "website"
      }} />
      <Analytics />
      <SpeedInsights />

      <div className="min-h-screen bg-black text-white">
        {/* Navigation */}
        <Nav />

        {/* Breadcrumb */}
        <div className="border-b border-white/10 bg-black/50 backdrop-blur">
          <div className="mx-auto max-w-7xl px-4 py-3">
            <Link
              to="/"
              onClick={handleBackClick}
              className="flex items-center gap-2 text-sm text-white/60 hover:text-white transition-colors"
            >
              <ArrowLeft className="h-4 w-4" />
              Back to Home
            </Link>
          </div>
        </div>

        {/* Main Content */}
        <main className="mx-auto max-w-4xl px-4 py-16 sm:px-6 lg:px-8">

          {/* Header */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6 }}
            className="mb-16 text-center"
          >
            <h1 className="mb-4 text-4xl font-bold text-white md:text-5xl">
              The Paper
            </h1>
            <p className="text-lg text-white/70">
              The complete Theory of Everything—plain English first, then the full technical version.
            </p>
          </motion.div>

          {/* Intro Section */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="mb-16 rounded-2xl border border-white/10 bg-white/5 p-8"
          >
            <h2 className="mb-4 text-2xl font-semibold text-white">Two Versions</h2>
            <div className="space-y-4 text-white/80">
              <p>
                <span className="font-semibold text-white">Plain English</span> (below): The core ideas in everyday language.
                No math, no prerequisites. Start here.
              </p>
              <p>
                <span className="font-semibold text-white">Full Technical Paper</span> (PDF): Complete mathematical framework,
                proofs, and derivations. For researchers.
              </p>
              <p className="text-white/60">
                Both say the same thing. Pick your starting point.
              </p>
            </div>
          </motion.div>

          {/* Plain English Version */}
          <div className="mb-16">
            <motion.h2
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
              className="mb-8 text-3xl font-semibold text-white"
            >
              Plain English Version
            </motion.h2>

            {PLAIN_ENGLISH_SECTIONS.map((section) => (
              <PlainEnglishSection key={section.id} section={section} />
            ))}

            {/* Conclusion */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
              className="mt-12 rounded-2xl border border-opoch-cyan-light/20 bg-opoch-cyan-light/5 p-6"
            >
              <p className="leading-relaxed text-white/90">{CONCLUSION}</p>
            </motion.div>
          </div>

          {/* Divider */}
          <div className="my-16 border-t border-white/10"></div>

          {/* Full Technical Paper Section */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="mb-16"
          >
            <h2 className="mb-6 text-3xl font-semibold text-white">
              Full Technical Paper
            </h2>
            <p className="mb-8 text-lg text-white/70">
              Ready for the complete mathematical framework?
            </p>

            <div className="mb-8 space-y-4 text-white/80">
              <p>The full paper includes:</p>
              <ul className="ml-6 space-y-2">
                <li className="flex gap-2">
                  <span className="text-opoch-cyan-light">•</span>
                  <span>Formal proofs of the three axioms</span>
                </li>
                <li className="flex gap-2">
                  <span className="text-opoch-cyan-light">•</span>
                  <span>Derivations of quantum mechanics, relativity, and thermodynamics</span>
                </li>
                <li className="flex gap-2">
                  <span className="text-opoch-cyan-light">•</span>
                  <span>Mathematical treatment of consciousness and AGI</span>
                </li>
                <li className="flex gap-2">
                  <span className="text-opoch-cyan-light">•</span>
                  <span>Complete bibliography and references</span>
                </li>
              </ul>
            </div>

            {/* PDF Download Buttons */}
            <div className="flex flex-col gap-4 sm:flex-row">
              <a
                href="/theory_of_everything.pdf"
                download="Theory_of_Everything_Opoch.pdf"
                onClick={handlePDFDownloadClick}
                className="inline-flex items-center justify-center gap-2 rounded-2xl border border-opoch-cyan-light/30 bg-opoch-cyan-light/10 px-6 py-3 font-medium text-opoch-cyan-light transition-all hover:border-opoch-cyan-light/50 hover:bg-opoch-cyan-light/20"
              >
                <Download className="h-5 w-5" />
                Download PDF
              </a>
              <a
                href="/theory_of_everything.pdf"
                target="_blank"
                rel="noopener noreferrer"
                onClick={handlePDFViewClick}
                className="inline-flex items-center justify-center gap-2 rounded-2xl border border-white/20 bg-white/5 px-6 py-3 font-medium text-white transition-all hover:border-white/30 hover:bg-white/10"
              >
                <ExternalLink className="h-5 w-5" />
                View in Browser
              </a>
            </div>

            <p className="mt-4 text-sm text-white/40">
              434 KB • GitHub repository coming soon
            </p>
          </motion.div>

          {/* Citation Section */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="mb-16 rounded-2xl border border-white/10 bg-white/5 p-8"
          >
            <h2 className="mb-4 text-2xl font-semibold text-white">Citation</h2>
            <p className="mb-4 text-white/70">If you reference this work:</p>
            <div className="mb-4 rounded-lg border border-white/10 bg-black p-4 font-mono text-sm text-white/90">
              {CITATION_TEXT}
            </div>
            <button
              onClick={handleCitationCopy}
              className="inline-flex items-center gap-2 text-sm text-opoch-cyan-light transition-colors hover:text-opoch-cyan"
            >
              {citationCopied ? (
                <>
                  <Check className="h-4 w-4" />
                  Copied!
                </>
              ) : (
                <>
                  <Copy className="h-4 w-4" />
                  Copy citation
                </>
              )}
            </button>
          </motion.div>

          {/* Related Links */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="mb-16"
          >
            <h2 className="mb-6 text-2xl font-semibold text-white">Related</h2>
            <div className="grid gap-4 sm:grid-cols-3">
              <Link
                to="/simple-questions"
                onClick={() => handleRelatedLinkClick("simple_questions")}
                className="flex items-center gap-2 rounded-2xl border border-white/10 bg-white/5 p-4 transition-all hover:border-white/20 hover:bg-white/10"
              >
                <span className="text-white">Simple Questions</span>
                <ExternalLink className="h-4 w-4 text-white/60" />
              </Link>
              <a
                href="https://chat.opoch.com"
                onClick={() => handleRelatedLinkClick("chat")}
                className="flex items-center gap-2 rounded-2xl border border-white/10 bg-white/5 p-4 transition-all hover:border-white/20 hover:bg-white/10"
              >
                <span className="text-white">Chat with GPT-5</span>
                <ExternalLink className="h-4 w-4 text-white/60" />
              </a>
              <Link
                to="/falsify"
                onClick={() => handleRelatedLinkClick("falsify")}
                className="flex items-center gap-2 rounded-2xl border border-white/10 bg-white/5 p-4 transition-all hover:border-white/20 hover:bg-white/10"
              >
                <span className="text-white">Falsify Us</span>
                <ExternalLink className="h-4 w-4 text-white/60" />
              </Link>
            </div>
          </motion.div>

        </main>
      </div>
    </>
  )
}
