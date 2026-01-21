import { lazy, Suspense } from "react"
import { Analytics } from "@vercel/analytics/react"
import { SpeedInsights } from "@vercel/speed-insights/react"
import SEOHead from "../components/SEOHead"
import { getPageMetadata } from "../lib/metadata"

// Lazy load components for better performance
const Nav = lazy(() => import("../components/v2/Nav"))
const Hero = lazy(() => import("../sections/v2/Hero"))
const ComparisonTable = lazy(() => import("../sections/v2/ComparisonTable"))
const FrontierProblems = lazy(() => import("../sections/v2/FrontierProblems"))
const Impact = lazy(() => import("../sections/v2/Impact"))
const Bounties = lazy(() => import("../sections/v2/Bounties"))

// Loading component for Suspense
const SectionLoader = () => (
  <div className="flex min-h-[200px] items-center justify-center">
    <div className="animate-pulse text-white/40">Loading...</div>
  </div>
)

export default function ToePage() {
  const toeMetadata = getPageMetadata("toe")

  return (
    <>
      <SEOHead metadata={toeMetadata} />
      <Analytics />
      <SpeedInsights />
      <div className="min-h-screen bg-black text-white">
        <Suspense fallback={<SectionLoader />}>
          <Nav />
        </Suspense>

        <main>
          <Suspense fallback={<SectionLoader />}>
            <Hero />
          </Suspense>

          <Suspense fallback={<SectionLoader />}>
            <FrontierProblems />
          </Suspense>

          <Suspense fallback={<SectionLoader />}>
            <Impact />
          </Suspense>

          <Suspense fallback={<SectionLoader />}>
            <Bounties />
          </Suspense>

          <Suspense fallback={<SectionLoader />}>
            <ComparisonTable />
          </Suspense>
        </main>
      </div>
    </>
  )
}
