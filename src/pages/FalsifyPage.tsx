import { useState, useCallback, memo, useEffect } from 'react'
import { Link } from 'react-router-dom'
import { motion } from 'framer-motion'
import { ArrowLeft, ChevronDown, Mail, XCircle } from 'lucide-react'
import { Analytics } from '@vercel/analytics/react'
import { SpeedInsights } from '@vercel/speed-insights/react'
import Nav from '../components/v2/Nav'
import SEOHead from '../components/SEOHead'
import { falsifyStats, falsifySections, whatDoesntCount } from '../data/falsify'
import { trackFalsifyPageViewed, trackFalsifySectionExpanded, trackFalsifySectionCollapsed, trackFalsifyEmailClicked, trackFalsifyBackClicked } from '../lib/analytics'

// Expandable section component with React.memo
const ExpandableSection = memo(({ section, isExpanded, onToggle }: {
  section: { id: string; title: string; content: string };
  isExpanded: boolean;
  onToggle: () => void;
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
        className="flex w-full items-center justify-between text-left"
        aria-expanded={isExpanded}
      >
        <h3 className="text-lg font-semibold text-white">{section.title}</h3>
        <ChevronDown
          className={`h-5 w-5 text-white/60 transition-transform ${isExpanded ? 'rotate-180' : ''}`}
        />
      </button>

      {isExpanded && (
        <motion.div
          initial={{ opacity: 0, height: 0 }}
          animate={{ opacity: 1, height: 'auto' }}
          exit={{ opacity: 0, height: 0 }}
          transition={{ duration: 0.3 }}
          className="mt-6 space-y-4"
        >
          {section.content.split('\n\n').map((paragraph, idx) => {
            // Check if paragraph is a heading (starts with **)
            if (paragraph.startsWith('**') && paragraph.includes('**')) {
              const heading = paragraph.replace(/\*\*/g, '');
              return (
                <h4 key={idx} className="font-semibold text-white/90">
                  {heading}
                </h4>
              );
            }
            return (
              <p key={idx} className="text-sm leading-relaxed text-white/70">
                {paragraph}
              </p>
            );
          })}
        </motion.div>
      )}
    </motion.div>
  );
});

ExpandableSection.displayName = 'ExpandableSection';

export default function FalsifyPage() {
  const [expandedSections, setExpandedSections] = useState<Record<string, boolean>>({});

  // Track page view on mount
  useEffect(() => {
    trackFalsifyPageViewed();
  }, []);

  const toggleSection = useCallback((sectionId: string) => {
    setExpandedSections(prev => {
      const isCurrentlyExpanded = prev[sectionId];
      const newState = !isCurrentlyExpanded;

      // Track analytics
      if (newState) {
        trackFalsifySectionExpanded(sectionId);
      } else {
        trackFalsifySectionCollapsed(sectionId);
      }

      return {
        ...prev,
        [sectionId]: newState
      };
    });
  }, []);

  const handleEmailClick = useCallback(() => {
    trackFalsifyEmailClicked();
  }, []);

  const handleBackClick = useCallback(() => {
    trackFalsifyBackClicked();
  }, []);

  return (
    <>
      <SEOHead metadata={{
        title: 'Falsify Us - Opoch',
        description: 'The theory stands until you break it. Here\'s how to falsify the Theory of Everything.',
        image: '/og-falsify.png',
        url: 'https://www.opoch.com/falsify',
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
            <motion.div
              initial={{ opacity: 0, y: 10 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.6 }}
              className="mb-4 flex flex-col items-center gap-3 sm:flex-row sm:justify-center sm:gap-4"
            >
              <h1 className="text-balance text-5xl font-semibold leading-tight text-white md:text-6xl">
                Falsify Us
              </h1>
              <span className="rounded-full bg-amber-500 px-4 py-1.5 text-lg font-bold text-black sm:text-xl">
                $50k Prize
              </span>
            </motion.div>
            <motion.p
              initial={{ opacity: 0, y: 10 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.1, duration: 0.6 }}
              className="text-xl text-white/70"
            >
              The theory stands until you break it. Here's how.
            </motion.p>
          </div>
        </section>

        {/* Stats Dashboard */}
        <section className="border-b border-white/10 bg-white/5 px-4 py-16">
          <div className="mx-auto max-w-7xl">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
            >
              <h2 className="mb-8 text-center text-2xl font-semibold text-white">Current Status</h2>

              <div className="grid gap-6 sm:grid-cols-3">
                <div className="rounded-2xl border border-white/10 bg-black p-6 text-center">
                  <div className="mb-2 text-4xl font-bold text-white">{falsifyStats.attempts}+</div>
                  <div className="text-sm text-white/60">Falsification Attempts</div>
                </div>

                <div className="rounded-2xl border border-emerald-500/30 bg-emerald-500/10 p-6 text-center">
                  <div className="mb-2 text-4xl font-bold text-emerald-300">{falsifyStats.successful}</div>
                  <div className="text-sm text-emerald-300/80">Successful Falsifications</div>
                </div>

                <div className="rounded-2xl border border-white/10 bg-black p-6 text-center">
                  <div className="mb-2 text-lg font-medium text-white">{falsifyStats.mostRecent}</div>
                  <div className="text-sm text-white/60">Most Recent Attempt</div>
                </div>
              </div>
            </motion.div>
          </div>
        </section>

        {/* What It Means to Falsify */}
        <section className="border-b border-white/10 px-4 py-16">
          <div className="mx-auto max-w-4xl">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
            >
              <h2 className="mb-6 text-3xl font-semibold text-white">What It Means to Falsify</h2>
              <div className="space-y-4 text-white/70 leading-relaxed">
                <p>
                  We are not making an empirical claim like "gravity is 9.8." We are making a <span className="text-white font-medium">forced-structure claim</span>:
                </p>
                <p className="text-white/90 border-l-2 border-white/30 pl-4 my-4">
                  If you accept <span className="font-medium">⊥</span> (no admissible distinctions) and <span className="font-medium">A0</span> (witnessability = finite witness + halting verifier), then the rest is forced — except for a small list of clearly labeled primitives.
                </p>
                <p>
                  You cannot falsify ⊥ or A0 — they are the contract. You can only falsify our claim that <span className="text-white font-medium">everything else follows from them</span>.
                </p>
              </div>
            </motion.div>
          </div>
        </section>

        {/* How to Falsify - Expandable Sections */}
        <section className="border-b border-white/10 bg-white/5 px-4 py-16">
          <div className="mx-auto max-w-4xl">
            <motion.h2
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
              className="mb-8 text-3xl font-semibold text-white"
            >
              How to Falsify and Win
            </motion.h2>

            <div className="space-y-4">
              {falsifySections.map((section) => (
                <ExpandableSection
                  key={section.id}
                  section={section}
                  isExpanded={expandedSections[section.id] || false}
                  onToggle={() => toggleSection(section.id)}
                />
              ))}
            </div>
          </div>
        </section>

        {/* Submit Your Falsification */}
        <section className="border-b border-white/10 px-4 py-16">
          <div className="mx-auto max-w-4xl">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
              className="rounded-2xl border border-white/10 bg-white/5 p-8"
            >
              <h2 className="mb-6 text-3xl font-semibold text-white">Submit Your Falsification</h2>

              <div className="mb-6 space-y-3 text-white/70">
                <p>Found a counterexample? Send us:</p>
                <ol className="ml-6 list-decimal space-y-2">
                  <li>Which falsification type (alternative closure, smuggled assumption, or internal contradiction)</li>
                  <li>The specific step or structure you're targeting</li>
                  <li>Your construction or proof</li>
                </ol>
              </div>

              <a
                href="mailto:hello@opoch.com?subject=Falsification Attempt"
                onClick={handleEmailClick}
                className="inline-flex items-center gap-2 rounded-2xl bg-white px-6 py-3 text-black transition-all hover:bg-white/90"
              >
                <Mail className="h-5 w-5" />
                Email hello@opoch.com
              </a>
            </motion.div>
          </div>
        </section>

        {/* What Doesn't Count */}
        <section className="border-b border-white/10 px-4 py-16">
          <div className="mx-auto max-w-4xl">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
              className="rounded-2xl border border-amber-500/30 bg-amber-500/10 p-8"
            >
              <h2 className="mb-6 text-2xl font-semibold text-white">What Doesn't Count as Falsification</h2>

              <ul className="space-y-3">
                {whatDoesntCount.map((item, idx) => (
                  <li key={idx} className="flex items-start gap-3">
                    <XCircle className="h-5 w-5 flex-shrink-0 text-amber-300" />
                    <span className="text-white/70">{item}</span>
                  </li>
                ))}
              </ul>

              <p className="mt-6 font-medium text-white/90">
                To falsify us, show an alternative closure, a hidden assumption, or an internal inconsistency — nothing else.
              </p>
            </motion.div>
          </div>
        </section>

        {/* Terms and Conditions */}
        <section className="px-4 py-16">
          <div className="mx-auto max-w-4xl">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.5 }}
            >
              <h2 className="mb-8 text-3xl font-semibold text-white">Prize Terms & Conditions</h2>

              <div className="space-y-6 text-sm text-white/70">
                <div>
                  <h3 className="mb-2 text-base font-semibold text-white">1. Eligibility</h3>
                  <ul className="ml-4 list-disc space-y-1">
                    <li>Open to individuals worldwide, 18 years or older.</li>
                    <li>Employees, contractors, and immediate family members of Opoch are not eligible.</li>
                    <li>Submissions must be made by a single individual; team submissions must designate one prize recipient.</li>
                    <li>You must have legal capacity to receive prize money in your jurisdiction.</li>
                  </ul>
                </div>

                <div>
                  <h3 className="mb-2 text-base font-semibold text-white">2. Submission Requirements</h3>
                  <ul className="ml-4 list-disc space-y-1">
                    <li>All submissions must be in English and sent to hello@opoch.com with subject line "Falsification Attempt".</li>
                    <li>Submission must clearly identify which falsification type is being claimed (Alternative Closure, Smuggled Assumption, or Internal Contradiction).</li>
                    <li>Submission must include a complete, self-contained proof or construction — not a sketch or outline.</li>
                    <li>All mathematical notation must be precisely defined; informal arguments do not qualify.</li>
                    <li>Submission must explicitly reference the specific axiom, step, or structure being challenged.</li>
                    <li>Maximum submission length: 50 pages (excluding appendices with supporting calculations).</li>
                  </ul>
                </div>

                <div>
                  <h3 className="mb-2 text-base font-semibold text-white">3. Evaluation Process</h3>
                  <ul className="ml-4 list-disc space-y-1">
                    <li>Initial review will be conducted by Opoch within 30 business days of submission.</li>
                    <li>Submissions passing initial review will be sent to an independent panel of at least two qualified experts in mathematical logic, theoretical physics, or related fields.</li>
                    <li>Expert panelists will be disclosed to the submitter upon request after final determination.</li>
                    <li>The evaluation will assess: (a) logical validity, (b) adherence to the stated contract (⊥ + A0), and (c) whether the claimed consequence actually follows.</li>
                    <li>Opoch reserves the right to request clarifications; failure to respond within 14 days may result in disqualification.</li>
                  </ul>
                </div>

                <div>
                  <h3 className="mb-2 text-base font-semibold text-white">4. Determination of Winner</h3>
                  <ul className="ml-4 list-disc space-y-1">
                    <li>A submission is successful if and only if the independent panel unanimously agrees it constitutes a valid falsification under one of the three specified methods.</li>
                    <li>Partial falsifications, approximate falsifications, or "weakening" of claims do not qualify.</li>
                    <li>If multiple valid falsifications are submitted, the prize goes to the earliest submission by timestamp.</li>
                    <li>Opoch's determination, informed by the expert panel, is final and binding.</li>
                  </ul>
                </div>

                <div>
                  <h3 className="mb-2 text-base font-semibold text-white">5. Prize Payment</h3>
                  <ul className="ml-4 list-disc space-y-1">
                    <li>Prize amount: USD $50,000.</li>
                    <li>Payment will be made within 60 days of final determination via wire transfer or equivalent method.</li>
                    <li>Winner is solely responsible for all applicable taxes, fees, and reporting requirements in their jurisdiction.</li>
                    <li>Opoch will issue tax documentation (e.g., 1099) as required by U.S. law.</li>
                    <li>Prize may not be transferred, assigned, or substituted except at Opoch's sole discretion.</li>
                  </ul>
                </div>

                <div>
                  <h3 className="mb-2 text-base font-semibold text-white">6. Intellectual Property</h3>
                  <ul className="ml-4 list-disc space-y-1">
                    <li>Submitter retains all intellectual property rights to their submission.</li>
                    <li>By submitting, you grant Opoch a non-exclusive, royalty-free, perpetual license to publish, reference, and discuss the submission (with attribution) for educational and scientific purposes.</li>
                    <li>Successful falsifications will be publicly acknowledged with full credit to the submitter.</li>
                    <li>Submitter warrants that the submission is their original work and does not infringe on any third-party rights.</li>
                  </ul>
                </div>

                <div>
                  <h3 className="mb-2 text-base font-semibold text-white">7. Confidentiality</h3>
                  <ul className="ml-4 list-disc space-y-1">
                    <li>Unsuccessful submissions will not be published without the submitter's consent.</li>
                    <li>Opoch may publicly disclose aggregate statistics (e.g., number of attempts, categories of arguments).</li>
                    <li>Correspondence between submitter and Opoch is confidential unless both parties agree otherwise.</li>
                  </ul>
                </div>

                <div>
                  <h3 className="mb-2 text-base font-semibold text-white">8. Disqualification</h3>
                  <ul className="ml-4 list-disc space-y-1">
                    <li>Submissions containing plagiarized content, fraudulent claims, or bad-faith arguments will be disqualified.</li>
                    <li>Harassment of Opoch staff or expert panelists will result in immediate disqualification and ban from future submissions.</li>
                    <li>Submitting the same argument multiple times with trivial modifications is grounds for disqualification.</li>
                    <li>Opoch reserves the right to disqualify any submission that violates these terms or applicable law.</li>
                  </ul>
                </div>

                <div>
                  <h3 className="mb-2 text-base font-semibold text-white">9. Limitation of Liability</h3>
                  <ul className="ml-4 list-disc space-y-1">
                    <li>Opoch is not responsible for lost, late, misdirected, or undeliverable submissions.</li>
                    <li>Opoch's total liability is limited to the prize amount.</li>
                    <li>Opoch is not liable for any indirect, incidental, or consequential damages arising from participation.</li>
                  </ul>
                </div>

                <div>
                  <h3 className="mb-2 text-base font-semibold text-white">10. Modifications and Termination</h3>
                  <ul className="ml-4 list-disc space-y-1">
                    <li>Opoch reserves the right to modify these terms with 30 days notice posted on this page.</li>
                    <li>Opoch may terminate the challenge at any time; submissions received before termination will still be evaluated.</li>
                    <li>If the prize is claimed, Opoch may continue, modify, or discontinue the challenge at its discretion.</li>
                  </ul>
                </div>

                <div>
                  <h3 className="mb-2 text-base font-semibold text-white">11. Governing Law</h3>
                  <ul className="ml-4 list-disc space-y-1">
                    <li>These terms are governed by the laws of the State of Delaware, USA.</li>
                    <li>Any disputes shall be resolved through binding arbitration in Wilmington, Delaware.</li>
                    <li>Submitter waives any right to participate in class action litigation related to this challenge.</li>
                  </ul>
                </div>

                <div className="mt-8 rounded-xl border border-white/10 bg-white/5 p-4">
                  <p className="text-white/60 text-xs">
                    By submitting a falsification attempt, you acknowledge that you have read, understood, and agree to be bound by these terms and conditions. Last updated: January 2026.
                  </p>
                </div>
              </div>
            </motion.div>
          </div>
        </section>
      </div>
    </>
  );
}
