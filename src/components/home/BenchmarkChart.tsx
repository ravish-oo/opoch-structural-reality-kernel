"use client"

import { memo } from "react"
import { motion } from "framer-motion"
import { cn } from "../../lib/utils"

export interface BenchmarkEntry {
  name: string
  score: number
  isHighlighted?: boolean
  color?: string
}

interface BenchmarkChartProps {
  data: BenchmarkEntry[]
  maxScore?: number
  sourceLabel?: string
  sourceUrl?: string
  className?: string
  hideFooter?: boolean
}

function PureBenchmarkChart({
  data,
  maxScore = 30,
  sourceLabel = "CritPt Benchmark",
  sourceUrl = "https://critpt.com",
  className,
  hideFooter = false,
}: BenchmarkChartProps) {
  // Sort by score descending
  const sortedData = [...data].sort((a, b) => b.score - a.score)

  return (
    <div className={cn("w-full", className)}>
      <div className="space-y-3">
        {sortedData.map((entry, index) => {
          const percentage = (entry.score / maxScore) * 100
          const isOpoch = entry.isHighlighted

          return (
            <motion.div
              key={entry.name}
              initial={{ opacity: 0, x: -20 }}
              whileInView={{ opacity: 1, x: 0 }}
              transition={{ duration: 0.4, delay: index * 0.1 }}
              viewport={{ once: true }}
              className="flex items-center gap-4"
            >
              {/* Label */}
              <div className={cn(
                "w-32 sm:w-40 text-sm font-medium truncate",
                isOpoch ? "text-white" : "text-white/60"
              )}>
                {entry.name}
              </div>

              {/* Bar */}
              <div className="flex-1 h-8 bg-white/5 rounded-lg overflow-hidden relative">
                <motion.div
                  initial={{ width: 0 }}
                  whileInView={{ width: `${percentage}%` }}
                  transition={{ duration: 0.8, delay: index * 0.1, ease: "easeOut" }}
                  viewport={{ once: true }}
                  className={cn(
                    "h-full rounded-lg",
                    isOpoch
                      ? "bg-gradient-to-r from-opoch-cyan to-opoch-cyan-light"
                      : entry.color || "bg-white/20"
                  )}
                />
                {/* Glow effect for highlighted */}
                {isOpoch && (
                  <div className="absolute inset-0 bg-gradient-to-r from-opoch-cyan/20 to-opoch-cyan-light/20 blur-xl" />
                )}
              </div>

              {/* Score */}
              <div className={cn(
                "w-16 text-right text-sm font-semibold tabular-nums",
                isOpoch ? "text-opoch-cyan-light" : "text-white/60"
              )}>
                {entry.score.toFixed(1)}%
              </div>
            </motion.div>
          )
        })}
      </div>

      {/* Source attribution */}
      {!hideFooter && (
        <div className="mt-4 pt-4 border-t border-white/10 flex items-center justify-between text-xs text-white/40">
          <span>71 research-level physics problems by 50+ researchers</span>
          <a
            href={sourceUrl}
            target="_blank"
            rel="noopener noreferrer"
            className="hover:text-white/60 transition-colors underline underline-offset-2"
          >
            {sourceLabel} →
          </a>
        </div>
      )}
    </div>
  )
}

export const BenchmarkChart = memo(PureBenchmarkChart)
