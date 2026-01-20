"use client"

import { memo, useState } from "react"
import { motion } from "framer-motion"
import { Play } from "lucide-react"

interface VideoSectionProps {
  videoId?: string // YouTube video ID
  title?: string
  subtitle?: string
  className?: string
}

// Placeholder video ID - replace with actual video
const PLACEHOLDER_VIDEO_ID = "dQw4w9WgXcQ" // Replace with actual video

function PureVideoSection({
  videoId = PLACEHOLDER_VIDEO_ID,
  title = "See Reasoning in Action",
  subtitle = "From school-level problems to frontier physics — watch how the framework derives answers step by step.",
  className,
}: VideoSectionProps) {
  const [isPlaying, setIsPlaying] = useState(false)

  const thumbnailUrl = `https://img.youtube.com/vi/${videoId}/maxresdefault.jpg`

  return (
    <section className={className}>
      <div className="mx-auto max-w-5xl px-4 py-20 sm:px-6 lg:px-8">
        {/* Section header */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5 }}
          viewport={{ once: true }}
          className="text-center mb-12"
        >
          <h2 className="text-3xl sm:text-4xl md:text-5xl font-bold mb-4 text-balance">
            <span className="bg-gradient-to-r from-white to-white/60 bg-clip-text text-transparent">
              {title}
            </span>
          </h2>
          <p className="text-lg text-white/60 max-w-2xl mx-auto text-balance">
            {subtitle}
          </p>
        </motion.div>

        {/* Video container */}
        <motion.div
          initial={{ opacity: 0, y: 30 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6 }}
          viewport={{ once: true }}
          className="relative aspect-video rounded-2xl overflow-hidden border border-white/10 bg-white/5"
        >
          {!isPlaying ? (
            // Thumbnail with play button
            <div className="relative w-full h-full">
              {/* Thumbnail image */}
              <img
                src={thumbnailUrl}
                alt="Video thumbnail"
                className="w-full h-full object-cover"
                onError={(e) => {
                  // Fallback if thumbnail doesn't load
                  (e.target as HTMLImageElement).src = `https://img.youtube.com/vi/${videoId}/hqdefault.jpg`
                }}
              />

              {/* Overlay */}
              <div className="absolute inset-0 bg-black/40" />

              {/* Play button */}
              <button
                onClick={() => setIsPlaying(true)}
                className="absolute inset-0 flex items-center justify-center group"
                aria-label="Play video"
              >
                <div className="flex items-center justify-center w-20 h-20 rounded-full bg-white/10 backdrop-blur-sm border border-white/20 group-hover:bg-white/20 group-hover:scale-110 transition-all duration-300">
                  <Play className="w-8 h-8 text-white ml-1" fill="white" />
                </div>
              </button>

              {/* Placeholder notice */}
              <div className="absolute bottom-4 left-4 px-3 py-1.5 rounded-lg bg-black/60 backdrop-blur-sm text-xs text-white/60">
                Video placeholder — replace with actual demo
              </div>
            </div>
          ) : (
            // YouTube iframe
            <iframe
              src={`https://www.youtube.com/embed/${videoId}?autoplay=1&rel=0`}
              title="Reasoning demonstration"
              allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
              allowFullScreen
              className="w-full h-full"
            />
          )}
        </motion.div>

        {/* Video highlights */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5, delay: 0.2 }}
          viewport={{ once: true }}
          className="mt-8 grid grid-cols-1 sm:grid-cols-3 gap-4"
        >
          {[
            { time: "0:00", label: "School-level example", desc: "Basic derivation" },
            { time: "2:30", label: "University-level", desc: "Complex reasoning" },
            { time: "5:00", label: "Research frontier", desc: "CritPt problem" },
          ].map((chapter, index) => (
            <button
              key={index}
              onClick={() => setIsPlaying(true)}
              className="flex items-start gap-3 p-4 rounded-xl border border-white/10 bg-white/5 hover:bg-white/10 hover:border-white/20 transition-all text-left"
            >
              <span className="text-xs font-mono text-opoch-cyan-light bg-opoch-cyan-light/10 px-2 py-1 rounded">
                {chapter.time}
              </span>
              <div>
                <p className="font-medium text-white/80">{chapter.label}</p>
                <p className="text-sm text-white/40">{chapter.desc}</p>
              </div>
            </button>
          ))}
        </motion.div>
      </div>
    </section>
  )
}

export const VideoSection = memo(PureVideoSection)
