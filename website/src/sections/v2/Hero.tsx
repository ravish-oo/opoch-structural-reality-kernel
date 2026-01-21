import React, { useCallback, useState, lazy, Suspense } from "react"
import { motion } from "framer-motion"
import { ExternalLink, Sparkles } from "lucide-react"
import { HERO_COPY, AI_LOGOS } from "./constants"
import { trackToeCTAClicked } from "../../lib/analytics"
import { config, docsLinks } from "../../lib/config"

// Lazy load AuthModal only when needed
const AuthModal = lazy(() => import("../../components/AuthModal"))

const Hero = React.memo(() => {
  // Auth modal state kept for potential future use
  const [authModalOpen, setAuthModalOpen] = useState(false)

  const handleTestItYourself = useCallback(() => {
    // Track CTA click
    trackToeCTAClicked("hero_test_yourself")

    // Always redirect to chat - let chat.opoch.com handle authentication
    window.location.href = config.chatUrl
  }, [])

  const handleFalsifyClick = useCallback(() => {
    trackToeCTAClicked("hero_falsify")
  }, [])

  const handleAILogoClick = useCallback((aiName: string) => {
    trackToeCTAClicked(`hero_ai_verification_${aiName.toLowerCase()}`)
  }, [])

  const handlePaperClick = useCallback(() => {
    trackToeCTAClicked("hero_paper_link")
  }, [])

  return (
    <section className="relative flex min-h-screen items-center justify-center overflow-hidden bg-black px-4 sm:px-6 lg:px-8">
      {/* Gradient Background - Opoch Cyan */}
      <div className="pointer-events-none absolute inset-0 -z-10 bg-[radial-gradient(80%_50%_at_50%_-10%,rgba(27,205,255,0.28),transparent)]"></div>

      <div className="relative mx-auto max-w-4xl py-20 text-center">

        {/* H1 - Headline with Gradient Text */}
        <motion.h1
          initial={{ opacity: 0, y: 10 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6 }}
          className="mb-8 text-balance text-5xl font-semibold leading-tight tracking-tight text-white md:text-6xl"
        >
          The Theory of Everything{" "}
          <span className="bg-gradient-to-r from-opoch-blue to-opoch-cyan-light bg-clip-text text-transparent">
            is here.
            <a
              href={docsLinks.theDerivation()}
              target="_blank"
              rel="noopener noreferrer"
              onClick={handlePaperClick}
              className="inline-flex translate-y-[-0.15em] transition-transform hover:scale-110"
              aria-label="Read the derivation"
            >
              <ExternalLink className="h-5 w-5 text-opoch-cyan-light md:h-6 md:w-6" />
            </a>
          </span>
        </motion.h1>

        {/* Subheadline */}
        <motion.p
          initial={{ opacity: 0, y: 10 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.1, duration: 0.6 }}
          className="mx-auto mb-16 max-w-3xl text-lg leading-relaxed text-white/80 md:text-xl"
        >
          {HERO_COPY.subheadline}
        </motion.p>

        {/* Next Banner + Live Status */}
        <motion.div
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ delay: 0.2, duration: 0.6 }}
          className="mb-8 space-y-3"
        >
          {/* Next: Consciousness Tech Banner */}
          <div className="mb-3 flex items-center justify-center gap-2">
            <Sparkles className="h-3.5 w-3.5 animate-pulse text-opoch-cyan-light" />
            <p className="animate-glow text-xs font-medium text-amber-400/70">
              {HERO_COPY.nextBanner}
            </p>
            <Sparkles className="h-3.5 w-3.5 animate-pulse text-opoch-cyan-light" />
          </div>

          {/* Live Now + Chat Info */}
          <div className="flex flex-col items-center justify-center gap-2 sm:flex-row">
            <div className="flex items-center gap-2">
              <span className="relative flex h-2 w-2">
                <span className="absolute inline-flex h-full w-full animate-ping rounded-full bg-opoch-cyan-light opacity-75"></span>
                <span className="animate-pulse-dot relative inline-flex h-2 w-2 rounded-full bg-opoch-cyan-light"></span>
              </span>
              <span className="animate-shimmer text-sm font-semibold text-opoch-cyan-light">
                {HERO_COPY.liveStatus}
              </span>
            </div>
            <span className="hidden text-sm text-white/40 sm:inline">•</span>
            <p className="text-center text-sm font-medium text-white/50 sm:text-left">
              {HERO_COPY.chatInfo}
            </p>
          </div>
        </motion.div>

        {/* Primary CTA Button */}
        <motion.div
          initial={{ opacity: 0, y: 10 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.3, duration: 0.6 }}
          className="mb-6"
        >
          <button
            onClick={handleTestItYourself}
            className="rounded-2xl bg-white px-10 py-4 text-lg font-medium text-black shadow-lg transition-all duration-200 hover:bg-white/90"
          >
            {HERO_COPY.ctaButton}
          </button>
        </motion.div>

        {/* CTA Subtext */}
        <p className="mx-auto mb-8 max-w-xl text-base text-white/60">
          {HERO_COPY.ctaSubtext}
        </p>

        {/* Secondary CTA - Falsify Us */}
        <a
          href="/falsify"
          onClick={handleFalsifyClick}
          className="group relative mb-20 inline-flex items-center gap-2 rounded-2xl border border-white/20 bg-white/5 px-6 py-3 text-sm font-medium text-white transition-all hover:border-white/40 hover:bg-white/10"
        >
          <span className="absolute -top-2 -right-2 rounded-full bg-amber-500 px-2 py-0.5 text-[10px] font-bold text-black">Win $50k</span>
          {HERO_COPY.secondaryCta}
        </a>

        {/* Verification Section - Glass Card */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5 }}
          viewport={{ once: true }}
          className="mx-auto max-w-3xl space-y-5 rounded-2xl border border-white/10 bg-white/5 p-6"
        >
          {/* Label */}
          <div className="text-xs uppercase tracking-wide text-white/40">
            {HERO_COPY.verificationLabel}
          </div>

          {/* AI Logos */}
          <div className="flex flex-wrap items-center justify-center gap-8">
            {AI_LOGOS.map((ai) => (
              <a
                key={ai.name}
                href={ai.verificationUrl}
                target="_blank"
                rel="noopener noreferrer"
                onClick={() => handleAILogoClick(ai.name)}
                className="group flex items-center gap-2 text-sm text-white/60 transition-colors duration-200 hover:text-opoch-cyan-light"
              >
                <span className="font-medium">{ai.name}</span>
                <ExternalLink className="h-3.5 w-3.5 opacity-40 transition-opacity group-hover:opacity-100" />
              </a>
            ))}
          </div>

          {/* Verification Subtext with Verify Yourself Link */}
          <div className="flex flex-col items-center gap-1 text-xs text-white/40 sm:flex-row sm:justify-center sm:gap-0">
            <span className="text-center">{HERO_COPY.verificationSubtext}</span>
            <span className="mx-1.5 hidden sm:inline">|</span>
            <a
              href="/verify-independently"
              className="text-white/60 transition-colors hover:text-white"
            >
              Verify yourself →
            </a>
          </div>
        </motion.div>

      </div>

      {/* Auth Modal (lazy loaded) */}
      {authModalOpen && (
        <Suspense fallback={<div />}>
          <AuthModal
            open={authModalOpen}
            onOpenChange={setAuthModalOpen}
            title="Sign in to test the theory"
            description="Authenticate to access the chat interface"
          />
        </Suspense>
      )}
    </section>
  )
})

Hero.displayName = "Hero"

export default Hero
