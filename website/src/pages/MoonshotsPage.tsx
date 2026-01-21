import { useState } from "react"
import { Link } from "react-router-dom"
import { motion } from "framer-motion"
import { ArrowRight, Filter } from "lucide-react"
import { moonshots, type MoonshotDomain, type MoonshotStatus } from "../data/moonshots"
import { Button } from "../components/ui/button"
import { cn } from "../lib/utils"
import { ApplyModal } from "../components/ApplyModal"
import SEOHead from "../components/SEOHead"
import { getPageMetadata } from "../lib/metadata"
import Nav from "../components/v2/Nav"

const domainColors: Record<MoonshotDomain, string> = {
  physics: "bg-purple-500/20 text-purple-300",
  energy: "bg-yellow-500/20 text-yellow-300",
  bio: "bg-green-500/20 text-green-300",
  infra: "bg-blue-500/20 text-blue-300",
  robotics: "bg-orange-500/20 text-orange-300"
}

const statusColors: Record<MoonshotStatus, string> = {
  open: "bg-emerald-500/20 text-emerald-300",
  "in-progress": "bg-amber-500/20 text-amber-300",
  partners: "bg-sky-500/20 text-sky-300"
}

function StatusChip({ status }: { status: MoonshotStatus }) {
  const labels: Record<MoonshotStatus, string> = {
    open: "Open",
    "in-progress": "In Progress",
    partners: "Accepting Partners"
  }
  
  return (
    <span className={cn("inline-flex items-center rounded-full px-2 py-1 text-xs font-medium", statusColors[status])}>
      {labels[status]}
    </span>
  )
}

function DomainChip({ domain }: { domain: MoonshotDomain }) {
  return (
    <span className={cn("inline-flex items-center rounded-full px-2 py-1 text-xs font-medium", domainColors[domain])}>
      {domain.charAt(0).toUpperCase() + domain.slice(1)}
    </span>
  )
}

export default function MoonshotsPage() {
  const [selectedDomain, setSelectedDomain] = useState<MoonshotDomain | null>(null)
  const [selectedStatus, setSelectedStatus] = useState<MoonshotStatus | null>(null)
  const [applyModalOpen, setApplyModalOpen] = useState(false)
  
  const filteredMoonshots = moonshots.filter(moonshot => {
    if (selectedDomain && moonshot.domain !== selectedDomain) return false
    if (selectedStatus && moonshot.status !== selectedStatus) return false
    return true
  })
  
  const domains: MoonshotDomain[] = ["physics", "energy", "bio", "infra", "robotics"]
  const statuses: MoonshotStatus[] = ["open", "in-progress", "partners"]
  
  const moonshotsMetadata = getPageMetadata('moonshots')
  
  return (
    <>
      <SEOHead metadata={moonshotsMetadata} />
      <div className="min-h-screen bg-black text-white">
      {/* Navigation */}
      <Nav />

      {/* Hero Section */}
      <section className="relative overflow-hidden border-b border-white/10">
        <div className="pointer-events-none absolute inset-0 -z-10 bg-[radial-gradient(80%_50%_at_50%_-20%,rgba(120,40,255,0.25),transparent)]" />
        <div className="mx-auto max-w-7xl px-4 py-20">
          <motion.h1 
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            className="text-5xl font-semibold tracking-tight md:text-6xl"
          >
            Moonshots
          </motion.h1>
          <motion.p 
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.1 }}
            className="mt-4 max-w-2xl text-xl text-white/80"
          >
            Hard problems with clear paths. Pick one, bring your data, and let's prove what's possible.
          </motion.p>
        </div>
      </section>

      {/* Filters */}
      <section className="border-b border-white/10">
        <div className="mx-auto max-w-7xl px-4 py-6">
          <div className="flex flex-wrap items-center gap-4">
            <div className="flex items-center gap-2 text-sm text-white/60">
              <Filter className="h-4 w-4" />
              <span>Filter by:</span>
            </div>
            
            <div className="flex flex-wrap gap-2">
              <button
                onClick={() => setSelectedDomain(null)}
                className={cn(
                  "rounded-full px-3 py-1 text-sm transition",
                  selectedDomain === null 
                    ? "bg-white text-black" 
                    : "bg-white/10 text-white/70 hover:bg-white/20"
                )}
              >
                All Domains
              </button>
              {domains.map(domain => (
                <button
                  key={domain}
                  onClick={() => setSelectedDomain(domain)}
                  className={cn(
                    "rounded-full px-3 py-1 text-sm capitalize transition",
                    selectedDomain === domain
                      ? "bg-white text-black" 
                      : "bg-white/10 text-white/70 hover:bg-white/20"
                  )}
                >
                  {domain}
                </button>
              ))}
            </div>
            
            <div className="h-4 w-px bg-white/20" />
            
            <div className="flex flex-wrap gap-2">
              <button
                onClick={() => setSelectedStatus(null)}
                className={cn(
                  "rounded-full px-3 py-1 text-sm transition",
                  selectedStatus === null 
                    ? "bg-white text-black" 
                    : "bg-white/10 text-white/70 hover:bg-white/20"
                )}
              >
                All Status
              </button>
              {statuses.map(status => (
                <button
                  key={status}
                  onClick={() => setSelectedStatus(status)}
                  className={cn(
                    "rounded-full px-3 py-1 text-sm transition",
                    selectedStatus === status
                      ? "bg-white text-black" 
                      : "bg-white/10 text-white/70 hover:bg-white/20"
                  )}
                >
                  {status === "in-progress" ? "In Progress" : status === "partners" ? "Partners" : "Open"}
                </button>
              ))}
            </div>
          </div>
        </div>
      </section>

      {/* Moonshots Grid */}
      <section className="mx-auto max-w-7xl px-4 py-16">
        <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
          {filteredMoonshots.map((moonshot, index) => (
            <motion.div
              key={moonshot.slug}
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: index * 0.05 }}
            >
              <Link
                to={`/moonshots/${moonshot.slug}`}
                className="group block h-full rounded-2xl border border-white/10 bg-white/5 p-6 transition hover:border-white/20 hover:bg-white/10"
              >
                <div className="mb-4 flex items-start justify-between">
                  <span className="text-4xl">{moonshot.emoji}</span>
                  <StatusChip status={moonshot.status} />
                </div>
                
                <h3 className="text-xl font-semibold">{moonshot.title}</h3>
                <p className="mt-1 text-sm text-white/60">{moonshot.tagline}</p>
                
                <p className="mt-4 text-sm text-white/80 line-clamp-3">
                  {moonshot.excerpt}
                </p>
                
                <div className="mt-6 flex items-center justify-between">
                  <div className="flex items-center gap-2">
                    <DomainChip domain={moonshot.domain} />
                    <span className="text-xs text-white/50">{moonshot.timeline}</span>
                  </div>
                  <ArrowRight className="h-4 w-4 text-white/40 transition group-hover:translate-x-1 group-hover:text-white" />
                </div>
              </Link>
            </motion.div>
          ))}
        </div>

        {filteredMoonshots.length === 0 && (
          <div className="py-20 text-center">
            <p className="text-white/60">No moonshots match your filters.</p>
          </div>
        )}
      </section>

      {/* CTA Section */}
      <section className="border-t border-white/10">
        <div className="mx-auto max-w-7xl px-4 py-16 text-center">
          <h2 className="text-3xl font-semibold">Have a moonshot in mind?</h2>
          <p className="mt-2 text-white/70">Bring your hardest problem. We'll show you what's possible.</p>
          <Button 
            className="mt-6 rounded-2xl bg-white text-black hover:bg-white/90"
            onClick={() => setApplyModalOpen(true)}
          >
            Apply for membership →
          </Button>
        </div>
      </section>

      <ApplyModal
        open={applyModalOpen}
        onOpenChange={setApplyModalOpen}
        source="moonshots"
      />
      </div>
    </>
  )
}