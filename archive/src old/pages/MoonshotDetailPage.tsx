import { useState, useEffect } from "react"
import { useParams, Link, Navigate } from "react-router-dom"
import { motion } from "framer-motion"
import { ArrowLeft, ChevronDown, Clock, Gauge, Sparkles } from "lucide-react"
import { getMoonshotBySlug } from "../data/moonshots"
import OpochLogo from "../components/OpochLogo"
import { Button } from "../components/ui/button"
import { cn } from "../lib/utils"
import ApplyModal from "../components/ApplyModal"
import SEOHead from "../components/SEOHead"
import { trackMoonshotViewed } from "../lib/analytics"
import { generateMoonshotMetadata } from "../lib/metadata"

const domainColors = {
  physics: "bg-purple-500/20 text-purple-300 border-purple-500/30",
  energy: "bg-yellow-500/20 text-yellow-300 border-yellow-500/30",
  bio: "bg-green-500/20 text-green-300 border-green-500/30",
  infra: "bg-blue-500/20 text-blue-300 border-blue-500/30",
  robotics: "bg-orange-500/20 text-orange-300 border-orange-500/30"
}

const statusColors = {
  open: "bg-emerald-500/20 text-emerald-300 border-emerald-500/30",
  "in-progress": "bg-amber-500/20 text-amber-300 border-amber-500/30",
  partners: "bg-sky-500/20 text-sky-300 border-sky-500/30"
}

export default function MoonshotDetailPage() {
  const { slug } = useParams<{ slug: string }>()
  const moonshot = slug ? getMoonshotBySlug(slug) : undefined
  const [applyModalOpen, setApplyModalOpen] = useState(false)
  const [expandedSections, setExpandedSections] = useState<Record<string, boolean>>({
    problem: true,
    approach: true
  })
  
  useEffect(() => {
    if (moonshot) {
      trackMoonshotViewed(moonshot.slug)
    }
  }, [moonshot])
  
  if (!moonshot) {
    return <Navigate to="/moonshots" replace />
  }
  
  const toggleSection = (section: string) => {
    setExpandedSections(prev => ({
      ...prev,
      [section]: !prev[section]
    }))
  }
  
  const sections = [
    { id: "problem", title: "Problem", content: moonshot.sections.problem },
    { id: "approach", title: "Our Approach", content: moonshot.sections.approach },
    { id: "requirements", title: "What We Need From You", content: moonshot.sections.requirements },
    { id: "deliverables", title: "What You Get", content: moonshot.sections.deliverables },
    { id: "verification", title: "How We'll Know It Worked", content: moonshot.sections.verification },
    { id: "impact", title: "Potential Impact", content: moonshot.sections.impact }
  ]
  
  const moonshotMetadata = generateMoonshotMetadata(moonshot)
  
  return (
    <>
      <SEOHead metadata={moonshotMetadata} />
      <div className="min-h-screen bg-black text-white">
      {/* Navigation */}
      <nav className="sticky top-0 z-40 border-b border-white/10 bg-black/80 backdrop-blur">
        <div className="mx-auto flex max-w-7xl items-center justify-between px-4 py-4">
          <div className="flex items-center gap-4">
            <Link to="/" className="flex items-center gap-3">
              <OpochLogo width={100} height={32} />
              <span className="hidden sm:inline-flex items-center gap-2 rounded-full border border-white/10 bg-white/5 px-3 py-1 text-xs text-white/80 backdrop-blur">
                <Sparkles className="h-3 w-3" /> Age of Truth
              </span>
            </Link>
            <Link 
              to="/moonshots" 
              className="flex items-center gap-1 text-sm text-white/60 hover:text-white"
            >
              <ArrowLeft className="h-3 w-3" />
              Moonshots
            </Link>
          </div>
          <Button 
            className="rounded-2xl bg-white text-black hover:bg-white/90"
            onClick={() => setApplyModalOpen(true)}
          >
            Apply
          </Button>
        </div>
      </nav>

      {/* Hero Section */}
      <section className="relative overflow-hidden border-b border-white/10">
        <div className={cn(
          "pointer-events-none absolute inset-0 -z-10",
          moonshot.domain === "physics" && "bg-[radial-gradient(80%_50%_at_50%_-20%,rgba(120,40,255,0.25),transparent)]",
          moonshot.domain === "energy" && "bg-[radial-gradient(80%_50%_at_50%_-20%,rgba(255,200,40,0.25),transparent)]",
          moonshot.domain === "bio" && "bg-[radial-gradient(80%_50%_at_50%_-20%,rgba(40,255,120,0.25),transparent)]",
          moonshot.domain === "infra" && "bg-[radial-gradient(80%_50%_at_50%_-20%,rgba(40,120,255,0.25),transparent)]",
          moonshot.domain === "robotics" && "bg-[radial-gradient(80%_50%_at_50%_-20%,rgba(255,120,40,0.25),transparent)]"
        )} />
        
        <div className="mx-auto max-w-4xl px-4 py-20">
          <motion.div
            initial={{ opacity: 0, scale: 0.8 }}
            animate={{ opacity: 1, scale: 1 }}
            className="mb-6 text-center text-6xl"
          >
            {moonshot.emoji}
          </motion.div>
          
          <motion.h1 
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            className="text-center text-4xl font-semibold tracking-tight md:text-5xl"
          >
            {moonshot.title}
          </motion.h1>
          
          <motion.p 
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.1 }}
            className="mt-4 text-center text-xl text-white/80"
          >
            {moonshot.tagline}
          </motion.p>
          
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.2 }}
            className="mt-8 flex flex-wrap items-center justify-center gap-3"
          >
            <span className={cn(
              "inline-flex items-center rounded-full border px-3 py-1 text-sm font-medium",
              statusColors[moonshot.status]
            )}>
              {moonshot.status === "open" ? "Open" : 
               moonshot.status === "in-progress" ? "In Progress" : 
               "Accepting Partners"}
            </span>
            
            <span className={cn(
              "inline-flex items-center rounded-full border px-3 py-1 text-sm font-medium capitalize",
              domainColors[moonshot.domain]
            )}>
              {moonshot.domain}
            </span>
            
            <span className="inline-flex items-center gap-1 rounded-full border border-white/20 bg-white/10 px-3 py-1 text-sm">
              <Clock className="h-3 w-3" />
              {moonshot.timeline}
            </span>
            
            <span className="inline-flex items-center gap-1 rounded-full border border-white/20 bg-white/10 px-3 py-1 text-sm">
              <Gauge className="h-3 w-3" />
              Complexity: {moonshot.complexity}/5
            </span>
          </motion.div>
        </div>
      </section>

      {/* Content Sections */}
      <section className="mx-auto max-w-4xl px-4 py-16">
        <motion.div
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ delay: 0.3 }}
          className="prose prose-invert max-w-none"
        >
          <p className="text-lg leading-relaxed text-white/80">
            {moonshot.excerpt}
          </p>
        </motion.div>
        
        <div className="mt-12 space-y-4">
          {sections.map((section, index) => (
            <motion.div
              key={section.id}
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.4 + index * 0.05 }}
              className="rounded-2xl border border-white/10 bg-white/5"
            >
              <button
                onClick={() => toggleSection(section.id)}
                className="flex w-full items-center justify-between p-6 text-left transition hover:bg-white/5"
              >
                <h2 className="text-xl font-semibold">{section.title}</h2>
                <ChevronDown 
                  className={cn(
                    "h-5 w-5 transition-transform",
                    expandedSections[section.id] && "rotate-180"
                  )} 
                />
              </button>
              
              {expandedSections[section.id] && (
                <div className="border-t border-white/10 p-6 pt-4">
                  <p className="text-white/80 whitespace-pre-line">
                    {section.content}
                  </p>
                </div>
              )}
            </motion.div>
          ))}
        </div>

        {/* CTA Section */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.8 }}
          className="mt-16 rounded-2xl border border-white/10 bg-gradient-to-br from-white/10 to-white/5 p-8 text-center"
        >
          <h2 className="text-2xl font-semibold">Ready to tackle this moonshot?</h2>
          <p className="mt-2 text-white/70">
            Join Opoch and let's prove what's possible together.
          </p>
          <Button 
            className="mt-6 rounded-2xl bg-white text-black hover:bg-white/90"
            onClick={() => setApplyModalOpen(true)}
          >
            Apply for membership →
          </Button>
        </motion.div>
      </section>

      <ApplyModal
        open={applyModalOpen}
        onOpenChange={setApplyModalOpen}
        source={`moonshot-${moonshot.slug}`}
      />
      </div>
    </>
  )
}