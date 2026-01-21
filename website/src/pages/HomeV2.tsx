import { lazy, Suspense } from "react"
import { Analytics } from "@vercel/analytics/react"
import { SpeedInsights } from "@vercel/speed-insights/react"
import SEOHead from "../components/SEOHead"
import { getPageMetadata } from "../lib/metadata"

// Lazy load components for better performance
const Nav = lazy(() => import("../components/v2/Nav"))
const HeroSection = lazy(() =>
  import("../components/home").then((mod) => ({ default: mod.HeroSection }))
)

// Loading component for Suspense
const SectionLoader = () => (
  <div className="flex min-h-[200px] items-center justify-center">
    <div className="animate-pulse text-white/40">Loading...</div>
  </div>
)

export default function HomeV2() {
  const homeMetadata = getPageMetadata("home")

  return (
    <>
      <SEOHead metadata={homeMetadata} />
      <Analytics />
      <SpeedInsights />
      <div className="min-h-screen bg-black text-white">
        <Suspense fallback={<SectionLoader />}>
          <Nav />
        </Suspense>

        <main>
          {/* Above the fold: Hero with benchmark */}
          <Suspense fallback={<SectionLoader />}>
            <HeroSection />
          </Suspense>

          {/* TODO: Add more sections below */}
          {/* - How it works / Framework explanation */}
          {/* - Use cases / Applications */}
          {/* - Resources (TOE, Paper, etc.) */}
          {/* - Footer CTA */}
        </main>
      </div>
    </>
  )
}
