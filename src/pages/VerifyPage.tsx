import { useState, useCallback, memo, useEffect } from 'react'
import { Link } from 'react-router-dom'
import { motion } from 'framer-motion'
import { ArrowLeft, Copy, Check } from 'lucide-react'
import { Analytics } from '@vercel/analytics/react'
import { SpeedInsights } from '@vercel/speed-insights/react'
import Nav from '../components/v2/Nav'
import SEOHead from '../components/SEOHead'
import {
  trackVerifyPageViewed,
  trackVerifyPromptCopied,
  trackVerifyChatClicked,
  trackVerifyFAQExpanded,
  trackVerifyFAQCollapsed,
  trackVerifyFalsifyLinkClicked,
  trackVerifyBackClicked
} from '../lib/analytics'

// Starter prompt constant
const STARTER_PROMPT = `What would a complete Theory of Everything need to demonstrate? Give me the full criteria—from fundamental axioms to testable predictions—that would constitute actual proof.`

// Process steps data
const PROCESS_STEPS = [
  {
    id: 1,
    title: 'Ask Your AI',
    description: 'Start with a general question about what TOE should prove',
    action: 'Copy starter prompt',
  },
  {
    id: 2,
    title: 'Get Requirements',
    description: 'Your AI will list what TOE must prove (quantum gravity, consciousness, etc.)',
  },
  {
    id: 3,
    title: 'Ask Opoch\'s GPT-5',
    description: 'Paste the same question in our chat. See what our TOE-verified GPT-5 says',
    action: 'Start chatting',
  },
  {
    id: 4,
    title: 'Cross-Verify',
    description: 'Copy our GPT-5\'s answer. Paste it back to your AI. Ask: "Does this answer satisfy the requirements?"',
  },
  {
    id: 5,
    title: 'Repeat Until Satisfied',
    description: 'Keep going back and forth. Either: Your AI confirms or you find a contradiction',
    action: 'Submit falsification',
  },
]

// FAQ data
const FAQS = [
  {
    question: 'Why not just read the paper?',
    answer: 'The paper is technical. This process lets you test claims interactively, in plain language, with your own AI doing the fact-checking.',
  },
  {
    question: 'Can I use a different starting prompt?',
    answer: 'Yes! The starter is just a suggestion. Ask whatever you think a TOE should prove.',
  },
  {
    question: 'How long does this take?',
    answer: '5-15 minutes if you\'re thorough. Depends on how deep you dig.',
  },
  {
    question: 'What if my AI gives different answers than yours?',
    answer: 'That\'s the point! If they genuinely contradict (not just phrasing differences), you may have found a falsification. Visit our falsify page to submit it.',
    hasLink: true,
  },
  {
    question: 'Do I need to understand the math?',
    answer: 'No. Your AI translates technical claims into plain language. You\'re testing consistency, not doing proofs yourself.',
  },
]

// Copy button component
const CopyButton = memo(({ text, source, className = '' }: { text: string; source: string; className?: string }) => {
  const [copied, setCopied] = useState(false)

  const handleCopy = useCallback(async () => {
    try {
      await navigator.clipboard.writeText(text)
      setCopied(true)
      setTimeout(() => setCopied(false), 2000)
      trackVerifyPromptCopied(source)
    } catch (err) {
      console.error('Failed to copy:', err)
    }
  }, [text, source])

  return (
    <button
      onClick={handleCopy}
      className={`inline-flex items-center gap-2 rounded-2xl bg-white/10 px-4 py-2 text-sm font-medium text-white transition-all hover:bg-white/20 ${className}`}
    >
      {copied ? (
        <>
          <Check className="h-4 w-4" />
          Copied!
        </>
      ) : (
        <>
          <Copy className="h-4 w-4" />
          Copy to clipboard
        </>
      )}
    </button>
  )
})

CopyButton.displayName = 'CopyButton'

// FAQ Item component
const FAQItem = memo(({ faq, isExpanded, onToggle }: {
  faq: { question: string; answer: string; hasLink?: boolean }
  isExpanded: boolean
  onToggle: () => void
}) => {
  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      whileInView={{ opacity: 1, y: 0 }}
      viewport={{ once: true }}
      transition={{ duration: 0.4 }}
      className="rounded-2xl border border-white/10 bg-white/5 p-6 transition-all hover:border-white/20 hover:bg-white/10"
    >
      <button
        onClick={onToggle}
        className="flex w-full items-start justify-between text-left"
        aria-expanded={isExpanded}
      >
        <h3 className="pr-4 text-base font-medium text-white">{faq.question}</h3>
        <span className="flex-shrink-0 text-white/60 transition-transform" style={{ transform: isExpanded ? 'rotate(180deg)' : 'rotate(0deg)' }}>
          ▼
        </span>
      </button>

      {isExpanded && (
        <motion.div
          initial={{ opacity: 0, height: 0 }}
          animate={{ opacity: 1, height: 'auto' }}
          exit={{ opacity: 0, height: 0 }}
          transition={{ duration: 0.3 }}
          className="mt-4"
        >
          <p className="text-sm leading-relaxed text-white/70">
            {faq.hasLink ? (
              <>
                That's the point! If they genuinely contradict (not just phrasing differences), you may have found a falsification.{' '}
                <Link
                  to="/falsify"
                  onClick={() => trackVerifyFalsifyLinkClicked('faq')}
                  className="text-opoch-cyan-light underline hover:text-opoch-cyan"
                >
                  Visit our falsify page
                </Link>{' '}
                to submit it.
              </>
            ) : (
              faq.answer
            )}
          </p>
        </motion.div>
      )}
    </motion.div>
  )
})

FAQItem.displayName = 'FAQItem'

export default function VerifyPage() {
  const [expandedFAQs, setExpandedFAQs] = useState<Record<number, boolean>>({})

  // Track page view on mount
  useEffect(() => {
    trackVerifyPageViewed()
  }, [])

  const toggleFAQ = useCallback((index: number, question: string) => {
    setExpandedFAQs(prev => {
      const isCurrentlyExpanded = prev[index]
      const newState = !isCurrentlyExpanded

      // Track analytics
      if (newState) {
        trackVerifyFAQExpanded(question)
      } else {
        trackVerifyFAQCollapsed(question)
      }

      return {
        ...prev,
        [index]: newState
      }
    })
  }, [])

  const handleChatClick = useCallback(() => {
    trackVerifyChatClicked()
    const chatUrl = import.meta.env.DEV ? 'http://localhost:3000' : 'https://chat.opoch.com'
    window.location.href = chatUrl
  }, [])

  const handleBackClick = useCallback(() => {
    trackVerifyBackClicked()
  }, [])

  const handleFalsifyClick = useCallback((source: string) => {
    trackVerifyFalsifyLinkClicked(source)
  }, [])

  return (
    <>
      <SEOHead metadata={{
        title: 'Verify with Your Own AI - Opoch',
        description: 'Don\'t trust our verification? Test the theory yourself—in your own ChatGPT, Claude, or Grok account.',
        image: '/og-verify.png',
        url: 'https://www.opoch.com/verify',
        type: 'website'
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
              to="/home-v2"
              onClick={handleBackClick}
              className="flex items-center gap-1 text-sm text-white/60 hover:text-white transition-colors"
            >
              <ArrowLeft className="h-3 w-3" />
              Back to Home
            </Link>
          </div>
        </div>

        {/* Hero Section */}
        <section className="relative overflow-hidden border-b border-white/10">
          {/* Gradient Background */}
          <div className="pointer-events-none absolute inset-0 -z-10 bg-[radial-gradient(80%_50%_at_50%_-10%,rgba(27,205,255,0.28),transparent)]" />

          <div className="mx-auto max-w-4xl px-4 py-20 text-center">
            <motion.h1
              initial={{ opacity: 0, y: 10 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.6 }}
              className="mb-4 text-balance text-5xl font-semibold leading-tight text-white md:text-6xl"
            >
              Verify with Your Own AI
            </motion.h1>
            <motion.p
              initial={{ opacity: 0, y: 10 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.1, duration: 0.6 }}
              className="text-xl text-white/70"
            >
              Don't trust our verification? Test the theory yourself—<br />
              in your own ChatGPT, Claude, or Grok account.
            </motion.p>
          </div>
        </section>

        {/* Why Verify This Way */}
        <section className="border-b border-white/10 bg-white/5 px-4 py-16">
          <div className="mx-auto max-w-4xl">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
            >
              <h2 className="mb-6 text-3xl font-semibold text-white">Why Verify This Way?</h2>
              <div className="space-y-4 text-white/70 leading-relaxed">
                <p>
                  You might think our AI verification links are cherry-picked, prompt-engineered, or use a hidden vector store. Valid skepticism.
                </p>
                <p className="font-medium text-white/90">
                  So verify it yourself:
                </p>
                <ul className="ml-6 list-disc space-y-2">
                  <li>Use YOUR AI account (not ours)</li>
                  <li>No context from us (start fresh)</li>
                  <li>Compare answers independently</li>
                  <li>See if your AI reaches the same conclusions</li>
                </ul>
                <p className="font-medium text-white/90">
                  If the theory holds, your AI will confirm it. If it doesn't, you'll break it.
                </p>
              </div>
            </motion.div>
          </div>
        </section>

        {/* Process Steps */}
        <section className="border-b border-white/10 px-4 py-16">
          <div className="mx-auto max-w-4xl">
            <motion.h2
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
              className="mb-12 text-center text-3xl font-semibold text-white"
            >
              How It Works
            </motion.h2>

            <div className="space-y-6">
              {PROCESS_STEPS.map((step, index) => (
                <motion.div
                  key={step.id}
                  initial={{ opacity: 0, y: 20 }}
                  whileInView={{ opacity: 1, y: 0 }}
                  viewport={{ once: true }}
                  transition={{ duration: 0.4, delay: index * 0.1 }}
                  className="flex gap-6"
                >
                  {/* Step Number */}
                  <div className="flex h-10 w-10 flex-shrink-0 items-center justify-center rounded-full bg-white/10 text-lg font-semibold text-white">
                    {step.id}
                  </div>

                  {/* Step Content */}
                  <div className="flex-1 rounded-2xl border border-white/10 bg-white/5 p-6">
                    <h3 className="mb-2 text-lg font-semibold text-white">{step.title}</h3>
                    <p className="mb-4 text-sm leading-relaxed text-white/70">{step.description}</p>

                    {step.action && (
                      <div className="mt-4">
                        {step.action === 'Copy starter prompt' && (
                          <div className="space-y-3">
                            <div className="rounded-xl border border-white/10 bg-black/50 p-4">
                              <p className="font-mono text-xs leading-relaxed text-white/90">
                                {STARTER_PROMPT}
                              </p>
                            </div>
                            <CopyButton text={STARTER_PROMPT} source="step_1" />
                          </div>
                        )}
                        {step.action === 'Start chatting' && (
                          <button
                            onClick={handleChatClick}
                            className="inline-flex items-center gap-2 rounded-2xl bg-white px-6 py-3 text-base font-medium text-black shadow-lg transition-all hover:bg-white/90"
                          >
                            Start chatting with GPT-5 →
                          </button>
                        )}
                        {step.action === 'Submit falsification' && (
                          <p className="text-sm text-white/70">
                            Found a contradiction?{' '}
                            <Link
                              to="/falsify"
                              onClick={() => handleFalsifyClick('step_5')}
                              className="text-opoch-cyan-light underline hover:text-opoch-cyan"
                            >
                              Submit your falsification
                            </Link>
                            .
                          </p>
                        )}
                      </div>
                    )}
                  </div>
                </motion.div>
              ))}
            </div>
          </div>
        </section>

        {/* Starter Prompt */}
        <section className="border-b border-white/10 bg-white/5 px-4 py-16">
          <div className="mx-auto max-w-4xl">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
            >
              <h2 className="mb-6 text-3xl font-semibold text-white">1. Starter Prompt for Your AI</h2>
              <p className="mb-6 text-white/70">
                Copy this and paste into your ChatGPT, Claude, or Grok:
              </p>

              {/* Prompt Box */}
              <div className="rounded-2xl border border-white/10 bg-black p-6">
                <p className="mb-4 font-mono text-sm leading-relaxed text-white/90">
                  {STARTER_PROMPT}
                </p>
                <CopyButton text={STARTER_PROMPT} source="verify_page_starter_prompt" />
              </div>
            </motion.div>
          </div>
        </section>

        {/* What You're Testing */}
        <section className="border-b border-white/10 px-4 py-16">
          <div className="mx-auto max-w-4xl">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
            >
              <h2 className="mb-6 text-3xl font-semibold text-white">What You're Actually Verifying</h2>
              <p className="mb-6 text-white/70">
                When you do this process, you're testing:
              </p>

              <div className="space-y-4">
                {[
                  { label: 'Completeness', desc: 'Does TOE address all known frontiers?' },
                  { label: 'Consistency', desc: 'Do answers match across independent AIs?' },
                  { label: 'Rigor', desc: 'Can it withstand adversarial questioning?' },
                  { label: 'Independence', desc: 'Does our GPT-5 give the same answers without prompt engineering or hidden context?' },
                ].map((item, index) => (
                  <motion.div
                    key={item.label}
                    initial={{ opacity: 0, y: 20 }}
                    whileInView={{ opacity: 1, y: 0 }}
                    viewport={{ once: true }}
                    transition={{ duration: 0.4, delay: index * 0.05 }}
                    className="flex items-start gap-3 rounded-2xl border border-white/10 bg-white/5 p-4"
                  >
                    <Check className="h-5 w-5 flex-shrink-0 text-opoch-cyan-light" />
                    <div>
                      <span className="font-semibold text-white">{item.label}:</span>{' '}
                      <span className="text-white/70">{item.desc}</span>
                    </div>
                  </motion.div>
                ))}
              </div>

              <p className="mt-6 text-white/70">
                If all four hold after 5-10 rounds of cross-checking, the theory is robust. If any breaks,{' '}
                <Link
                  to="/falsify"
                  onClick={() => handleFalsifyClick('verification_section')}
                  className="text-opoch-cyan-light underline hover:text-opoch-cyan"
                >
                  you've falsified it
                </Link>
                .
              </p>
            </motion.div>
          </div>
        </section>

        {/* Expected Outcomes */}
        <section className="border-b border-white/10 bg-white/5 px-4 py-16">
          <div className="mx-auto max-w-4xl">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
            >
              <h2 className="mb-8 text-3xl font-semibold text-white">What to Expect</h2>

              <div className="grid gap-6 md:grid-cols-2">
                {/* If it holds */}
                <div className="rounded-2xl border border-emerald-500/30 bg-emerald-500/10 p-6">
                  <h3 className="mb-4 text-lg font-semibold text-emerald-300">If the Theory Holds:</h3>
                  <ul className="space-y-2 text-sm text-white/70">
                    <li className="flex items-start gap-2">
                      <span className="text-emerald-300">•</span>
                      <span>All major problems are addressed</span>
                    </li>
                    <li className="flex items-start gap-2">
                      <span className="text-emerald-300">•</span>
                      <span>Answers are consistent with physics/math</span>
                    </li>
                    <li className="flex items-start gap-2">
                      <span className="text-emerald-300">•</span>
                      <span>No circular reasoning or hand-waving</span>
                    </li>
                    <li className="flex items-start gap-2">
                      <span className="text-emerald-300">•</span>
                      <span>Solutions match what other AIs verified</span>
                    </li>
                  </ul>
                </div>

                {/* If you break it */}
                <div className="rounded-2xl border border-amber-500/30 bg-amber-500/10 p-6">
                  <h3 className="mb-4 text-lg font-semibold text-amber-300">If You Break It:</h3>
                  <ul className="space-y-2 text-sm text-white/70">
                    <li className="flex items-start gap-2">
                      <span className="text-amber-300">•</span>
                      <span>A requirement TOE doesn't meet</span>
                    </li>
                    <li className="flex items-start gap-2">
                      <span className="text-amber-300">•</span>
                      <span>Contradictory answers between AIs</span>
                    </li>
                    <li className="flex items-start gap-2">
                      <span className="text-amber-300">•</span>
                      <span>Logical gaps your AI can't reconcile</span>
                    </li>
                    <li className="flex items-start gap-2">
                      <span className="text-amber-300">•</span>
                      <span>
                        Submit to:{' '}
                        <Link
                          to="/falsify"
                          onClick={() => handleFalsifyClick('outcomes_section')}
                          className="text-amber-300 underline hover:text-amber-200"
                        >
                          opoch.com/falsify
                        </Link>
                      </span>
                    </li>
                  </ul>
                </div>
              </div>

              <p className="mt-6 text-center font-medium text-white">
                Either way, you've verified independently.
              </p>
            </motion.div>
          </div>
        </section>

        {/* FAQs */}
        <section className="px-4 py-16">
          <div className="mx-auto max-w-4xl">
            <motion.h2
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
              className="mb-8 text-3xl font-semibold text-white"
            >
              Common Questions
            </motion.h2>

            <div className="space-y-4">
              {FAQS.map((faq, index) => (
                <FAQItem
                  key={index}
                  faq={faq}
                  isExpanded={expandedFAQs[index] || false}
                  onToggle={() => toggleFAQ(index, faq.question)}
                />
              ))}
            </div>
          </div>
        </section>
      </div>
    </>
  )
}
