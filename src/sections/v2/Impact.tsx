import { useState, useCallback, memo } from 'react';
import { motion } from 'framer-motion';
import {
  Pill, Brain, BatteryCharging, Zap, Cpu, Key,
  Clock, Leaf, Sparkles, Bot, Atom, Users,
  Rocket, Cloud, Upload, Globe, Circle,
  Sun, Move, Star, Infinity,
  Layers, Undo, Code, CircleDot
} from 'lucide-react';
import { trackToeImpactExpanded, trackToeImpactCollapsed, trackToeEasterEggRevealed } from '../../lib/analytics';

interface ImpactOutcome {
  outcome: string;
  mechanism: string;
  description: string;
  market: string;
  marketValue: string;
  icon: React.ComponentType<{ className?: string }>;
  isKardashev?: boolean;
  isEasterEgg?: boolean;
}

interface ImpactStage {
  label: string;
  timeframe: string;
  emoji: string;
  color: string;
  glowColor: string;
  outcomes: ImpactOutcome[];
}

// Stage 1: IMMEDIATE (0-5 years)
const stage1: ImpactStage = {
  label: 'Stage 1',
  timeframe: 'Now - 5 Years',
  emoji: '⚛️',
  color: 'from-blue-500 to-cyan-400',
  glowColor: 'rgba(59, 130, 246, 0.5)',
  outcomes: [
    {
      outcome: 'Cure any disease with a single treatment',
      mechanism: 'via precision medicine',
      description: 'Molecular-level biological optimization',
      market: 'Global healthcare transforms to predictive/preventive model',
      marketValue: '$15T',
      icon: Pill,
    },
    {
      outcome: 'AI that never hallucinates, proves its answers',
      mechanism: 'via consciousness formalization',
      description: 'Truth mechanically verifiable with FY receipts',
      market: 'Knowledge work augmentation, end of misinformation',
      marketValue: '$50T',
      icon: Brain,
    },
    {
      outcome: 'Batteries that charge instantly, last decades',
      mechanism: 'via thermodynamic bounds',
      description: 'Minimal-loss energy storage chemistry',
      market: 'Energy storage + EV acceleration',
      marketValue: '$15T',
      icon: BatteryCharging,
    },
    {
      outcome: 'Energy too cheap to meter',
      mechanism: 'via superconductors + fusion',
      description: 'Zero-loss transmission, optimized fusion',
      market: 'Global energy, post-scarcity begins',
      marketValue: '$8T',
      icon: Zap,
    },
    {
      outcome: 'Computers that run cold and never break',
      mechanism: 'via minimal-dissipation computing',
      description: 'Physical limits approached, Landauer bound',
      market: 'Computing efficiency, AI costs collapse',
      marketValue: '$3T',
      icon: Cpu,
    },
    {
      outcome: 'Crack any code, secure any system',
      mechanism: 'via quantum computing',
      description: 'Decoherence solved, perfect error correction',
      market: 'Cybersecurity + cryptography revolution',
      marketValue: '$7T',
      icon: Key,
    },
  ],
};

// Stage 2: NEAR-TERM (5-20 years)
const stage2: ImpactStage = {
  label: 'Stage 2',
  timeframe: '5 - 20 Years',
  emoji: '🌍',
  color: 'from-blue-400 to-cyan-300',
  glowColor: 'rgba(56, 189, 248, 0.5)',
  outcomes: [
    {
      outcome: 'Live 200+ healthy years',
      mechanism: 'via aging reversal',
      description: 'Intervention targets from entropy optimization',
      market: 'Longevity economy + restructured civilization',
      marketValue: '$20T',
      icon: Clock,
    },
    {
      outcome: 'Reverse climate change completely',
      mechanism: 'via optimized CO₂ capture',
      description: 'Abundant energy enables planetary restoration',
      market: 'Climate tech + incalculable avoided damage',
      marketValue: '$10T',
      icon: Leaf,
    },
    {
      outcome: 'Personal AI smarter than any human',
      mechanism: 'via safe recursive self-improvement',
      description: 'Alignment solved, consciousness integrated',
      market: 'Cognitive augmentation',
      marketValue: '$100T+',
      icon: Sparkles,
    },
    {
      outcome: 'Robot workers in every home',
      mechanism: 'via consciousness understanding',
      description: 'Human-level physical intelligence',
      market: 'Automation + labor transformation',
      marketValue: '$30T',
      icon: Bot,
    },
    {
      outcome: 'Build anything atom by atom',
      mechanism: 'via molecular manufacturing',
      description: 'Star Trek replicators, material science + quantum control',
      market: 'Manufacturing reimagined',
      marketValue: '$50T',
      icon: Atom,
    },
    {
      outcome: 'Food, water, shelter for everyone',
      mechanism: 'via post-scarcity production',
      description: 'Energy abundance + molecular manufacturing',
      market: 'End of poverty (transformation, not a market)',
      marketValue: '∞',
      icon: Users,
    },
  ],
};

// Stage 3: MEDIUM-TERM (20-50 years)
const stage3: ImpactStage = {
  label: 'Stage 3',
  timeframe: '20 - 50 Years',
  emoji: '🚀',
  color: 'from-purple-500 to-pink-400',
  glowColor: 'rgba(168, 85, 247, 0.5)',
  outcomes: [
    {
      outcome: 'Cities on Mars, Moon, asteroids',
      mechanism: 'via closed-loop life support',
      description: 'Optimal propulsion, indefinite space habitation',
      market: 'Space economy, multi-planetary insurance',
      marketValue: '$100T+',
      icon: Rocket,
    },
    {
      outcome: 'Control weather perfectly',
      mechanism: 'via planetary-scale engineering',
      description: 'Atmosphere as controllable system',
      market: 'Weather control + agriculture optimization',
      marketValue: '$5T',
      icon: Cloud,
    },
    {
      outcome: 'Upload your consciousness',
      mechanism: 'via observer formalization',
      description: 'Backup, transfer, or run in different substrates',
      market: 'Immortality option (beyond economics)',
      marketValue: '∞',
      icon: Upload,
    },
    {
      outcome: 'Terraform dead planets into Earths',
      mechanism: 'via planetary engineering',
      description: 'Mars habitable in decades, not millennia',
      market: 'Unlimited expansion room',
      marketValue: '∞',
      icon: Globe,
    },
    {
      outcome: 'Humanity reaches Kardashev Type 1',
      mechanism: 'via full planetary energy control',
      description: 'Harness Earth\'s total energy (~10¹⁶ W)',
      market: 'Civilization-scale leap complete',
      marketValue: '∞',
      icon: Zap,
      isKardashev: true,
    },
  ],
};

// Stage 4: LONG-TERM (50-200 years)
const stage4: ImpactStage = {
  label: 'Stage 4',
  timeframe: '50 - 200 Years',
  emoji: '☀️',
  color: 'from-yellow-500 to-orange-400',
  glowColor: 'rgba(234, 179, 8, 0.5)',
  outcomes: [
    {
      outcome: 'Harvest the entire Sun\'s energy',
      mechanism: 'via Dyson swarm',
      description: 'Stellar-scale engineering',
      market: 'Energy 1 billion times current global use',
      marketValue: '∞',
      icon: Sun,
    },
    {
      outcome: 'Travel at near light speed',
      mechanism: 'via thermodynamic limit propulsion',
      description: 'Interstellar distances become reachable',
      market: 'Galaxy opens',
      marketValue: '∞',
      icon: Rocket,
    },
    {
      outcome: 'Black holes as power plants',
      mechanism: 'via horizon mechanics',
      description: 'Penrose process, infinite clean energy',
      market: 'Energy constraints disappear forever',
      marketValue: '∞',
      icon: Circle,
    },
    {
      outcome: 'Engineer spacetime itself',
      mechanism: 'via quantum gravity understanding',
      description: 'Controlled experiments on fabric of reality',
      market: 'Physics becomes engineering',
      marketValue: '∞',
      icon: Move,
    },
    {
      outcome: 'Humanity reaches Kardashev Type 2',
      mechanism: 'via stellar-scale energy control',
      description: 'Harness entire star\'s output (~10²⁶ W)',
      market: 'Star-faring civilization',
      marketValue: '∞',
      icon: Sun,
      isKardashev: true,
    },
  ],
};

// Stage 5: FAR FUTURE (200-1000 years)
const stage5: ImpactStage = {
  label: 'Stage 5',
  timeframe: '200 - 1000 Years',
  emoji: '✨',
  color: 'from-gray-200 to-gray-400',
  glowColor: 'rgba(229, 231, 235, 0.5)',
  outcomes: [
    {
      outcome: 'Colonize thousands of star systems',
      mechanism: 'via interstellar fleets',
      description: 'Generation ships, consciousness transfer, breakthrough propulsion',
      market: 'Galactic real estate',
      marketValue: '∞',
      icon: Star,
    },
    {
      outcome: 'Civilizations around every star',
      mechanism: 'via multi-star coordination',
      description: 'Galactic-scale humanity',
      market: 'Species survival assured',
      marketValue: '∞',
      icon: Sparkles,
    },
    {
      outcome: 'Control dark energy',
      mechanism: 'via cosmological constant manipulation',
      description: 'Universe-scale engineering',
      market: 'Reshape cosmos',
      marketValue: '∞',
      icon: Infinity,
    },
    {
      outcome: 'Humanity reaches Kardashev Type 3',
      mechanism: 'via galactic energy control',
      description: 'Harness billions of stars (~10³⁶ W)',
      market: 'Galactic civilization',
      marketValue: '∞',
      icon: Sparkles,
      isKardashev: true,
    },
  ],
};

// Stage 6: EASTER EGG (Beyond Time) - Portal Green Theme
const stage6: ImpactStage = {
  label: 'Stage 6',
  timeframe: 'Beyond Time',
  emoji: '🌀',
  color: 'from-emerald-500 via-green-500 to-cyan-500',
  glowColor: 'rgba(52, 211, 153, 0.5)',
  outcomes: [
    {
      outcome: 'Travel between universes',
      mechanism: 'if FY/gluing extends beyond our reality',
      description: 'Multiverse becomes accessible',
      market: 'Infinite possibilities',
      marketValue: '∞',
      icon: Layers,
      isEasterEgg: true,
    },
    {
      outcome: 'Rewind or fast-forward time',
      mechanism: 'if paid/free split allows temporal engineering',
      description: 'Causality becomes navigable',
      market: 'Control timeline itself',
      marketValue: '∞',
      icon: Undo,
      isEasterEgg: true,
    },
    {
      outcome: 'Program new realities from scratch',
      mechanism: 'if Π/FY framework is universal code',
      description: 'Create pocket universes',
      market: 'Reality as code',
      marketValue: '∞',
      icon: Code,
      isEasterEgg: true,
    },
    {
      outcome: 'Portal guns (obviously)',
      mechanism: 'because if you have TOE, you get portal guns',
      description: 'Rick & Morty was a documentary',
      market: 'Literally infinite',
      marketValue: '∞',
      icon: CircleDot,
      isEasterEgg: true,
    },
  ],
};

const ImpactCard = memo(({ outcome, index }: { outcome: ImpactOutcome; index: number }) => {
  const Icon = outcome.icon;

  // Special styling for Kardashev milestones
  const isKardashev = outcome.isKardashev;
  const isEasterEgg = outcome.isEasterEgg;

  const cardClasses = isKardashev
    ? "group relative flex flex-col rounded-2xl border-2 border-yellow-500/30 bg-gradient-to-br from-yellow-500/10 to-orange-500/10 p-6 hover:border-yellow-500/50 hover:shadow-2xl hover:shadow-yellow-500/20 transition-all"
    : isEasterEgg
    ? "group relative flex flex-col rounded-2xl border-2 border-emerald-400/40 bg-gradient-to-br from-emerald-500/15 via-green-500/10 to-cyan-500/15 p-6 hover:border-emerald-400/60 hover:shadow-2xl hover:shadow-emerald-400/30 transition-all backdrop-blur-sm"
    : "group relative flex flex-col rounded-2xl border border-white/10 bg-white/5 p-6 hover:border-white/20 hover:bg-white/10 transition-all";

  const iconClasses = isKardashev
    ? "h-10 w-10 text-yellow-400"
    : isEasterEgg
    ? "h-10 w-10 text-emerald-400"
    : "h-8 w-8 text-opoch-cyan-light";

  return (
    <motion.div
      initial={{ opacity: 0, x: -20, y: 20 }}
      whileInView={{ opacity: 1, x: 0, y: 0 }}
      viewport={{ once: true }}
      transition={{ duration: 0.5, delay: index * 0.05 }}
      whileHover={{ y: -6, transition: { duration: 0.2 } }}
      className={cardClasses}
    >
      {/* Icon - Visual Anchor */}
      <motion.div
        className="mb-4"
        initial={{ scale: 1 }}
        whileInView={{ scale: [1, 1.15, 1] }}
        transition={{ duration: 0.6, delay: index * 0.05 + 0.3 }}
        viewport={{ once: true }}
      >
        <Icon className={iconClasses} />
      </motion.div>

      {/* Outcome - Headline */}
      <h3 className="mb-3 text-xl font-semibold leading-tight tracking-tight text-white">
        {outcome.outcome}
      </h3>

      {/* Mechanism */}
      <p className="mb-2 text-sm italic text-white/50">
        {outcome.mechanism}
      </p>

      {/* Description - One line, tight */}
      <p className="mb-16 text-sm text-white/60">
        {outcome.description}
      </p>

      {/* Bottom Section - Market Info */}
      <div className="absolute bottom-6 left-6 right-6 flex items-center justify-between gap-4">
        {/* Market Description - Bottom Left */}
        <div className="max-w-[60%]">
          <p className="text-xs text-opoch-cyan-light/70">
            {outcome.market}
          </p>
        </div>

        {/* Market Badge - Bottom Right, Prominent */}
        <div className="group/badge flex-shrink-0">
          <div className="rounded-xl bg-emerald-500/10 px-4 py-2">
            <div className="text-2xl font-bold text-emerald-400">
              {outcome.marketValue}
            </div>
          </div>
          {/* Tooltip on hover */}
          <div className="absolute bottom-full right-0 mb-2 hidden group-hover/badge:block">
            <div className="whitespace-nowrap rounded-lg bg-black border border-white/20 px-3 py-1.5 text-xs text-white/80">
              Est. market size in excess of {outcome.marketValue}
            </div>
          </div>
        </div>
      </div>
    </motion.div>
  );
});

ImpactCard.displayName = 'ImpactCard';

export default function Impact() {
  const [isExpanded, setIsExpanded] = useState(false);
  const [easterEggRevealed, setEasterEggRevealed] = useState(false);
  const allStages = [stage1, stage2, stage3, stage4, stage5];
  const visibleStages = isExpanded ? allStages : allStages.slice(0, 2); // Show Stage 1 + 2 initially

  const handleExpand = useCallback(() => {
    setIsExpanded(true);
    trackToeImpactExpanded();
  }, []);

  const handleCollapse = useCallback(() => {
    setIsExpanded(false);
    trackToeImpactCollapsed();
  }, []);

  return (
    <section className="border-t border-white/10 bg-black px-4 py-24 sm:px-6 lg:px-8">
      <div className="mx-auto max-w-7xl">
        {/* Section Heading */}
        <motion.div
          initial={{ opacity: 0, y: 10, scale: 0.95 }}
          whileInView={{ opacity: 1, y: 0, scale: 1 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
          className="mb-6 text-center"
        >
          <h2 className="mb-3 text-balance text-center">
            <div
              className="text-xl font-medium text-white/80 md:text-2xl"
              style={{
                textShadow: '0 0 60px rgba(27, 205, 255, 0.6), 0 0 30px rgba(27, 205, 255, 0.4)',
              }}
            >
              Commencing
            </div>
            <div className="mt-2 text-4xl font-semibold text-white md:text-6xl">
              The Age of Truth
            </div>
          </h2>
          <p className="text-lg text-white/60 md:text-xl">
            From Here to the Stars
          </p>
        </motion.div>

        {/* All Stages */}
        {visibleStages.map((stage, _stageIndex) => (
          <motion.div
            key={stage.label}
            initial={{ opacity: 0, y: 30 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true, margin: "-100px" }}
            transition={{ duration: 0.6, delay: 0.1 }}
            className="mt-16"
          >
            {/* Stage Label */}
            <motion.div
              initial={{ opacity: 0, scale: 0.9 }}
              whileInView={{ opacity: 1, scale: 1 }}
              transition={{ duration: 0.5, delay: 0.2 }}
              viewport={{ once: true }}
              className="mb-8 text-center"
            >
              <div
                className={`inline-flex items-center gap-3 rounded-2xl border border-white/20 bg-gradient-to-r ${stage.color} bg-clip-padding px-5 py-2.5`}
                style={{
                  boxShadow: `0 0 20px ${stage.glowColor}`,
                  background: `linear-gradient(to right, ${stage.glowColor.replace('0.5', '0.15')}, ${stage.glowColor.replace('0.5', '0.1')})`,
                }}
              >
                <span className="text-xl">{stage.emoji}</span>
                <span className="text-sm font-medium text-white/80">{stage.label}</span>
                <span className="text-white/40">•</span>
                <span className="text-sm text-white/60">{stage.timeframe}</span>
              </div>
            </motion.div>

            {/* Cards Grid */}
            <div className="grid gap-6 sm:grid-cols-2 lg:grid-cols-3">
              {stage.outcomes.map((outcome, index) => (
                <ImpactCard key={outcome.outcome} outcome={outcome} index={index} />
              ))}
            </div>
          </motion.div>
        ))}

        {/* Expand/Collapse Button */}
        {!isExpanded ? (
          <motion.div
            initial={{ opacity: 0 }}
            whileInView={{ opacity: 1 }}
            transition={{ duration: 0.4, delay: 0.2 }}
            viewport={{ once: true }}
            className="mt-16 flex justify-center"
          >
            <button
              onClick={handleExpand}
              className="group inline-flex items-center gap-2 rounded-2xl border border-white/20 bg-white/5 px-6 py-3 text-sm font-medium text-white transition-all hover:border-white/40 hover:bg-white/10"
            >
              View the full timeline →
            </button>
          </motion.div>
        ) : (
          <>
            {/* Easter Egg - Stage 6 */}
            <div className="mt-20">
              {!easterEggRevealed ? (
            <motion.div
              initial={{ opacity: 0 }}
              whileInView={{ opacity: 1 }}
              transition={{ duration: 0.5 }}
              viewport={{ once: true }}
              className="text-center"
            >
              <button
                onClick={() => {
                  setEasterEggRevealed(true);
                  trackToeEasterEggRevealed();
                }}
                onMouseEnter={() => {
                  setEasterEggRevealed(true);
                  trackToeEasterEggRevealed();
                }}
                className="group inline-flex flex-col items-center gap-3 rounded-2xl border-2 border-dashed border-emerald-400/40 bg-gradient-to-br from-emerald-500/10 via-green-500/5 to-cyan-500/10 px-8 py-6 transition-all hover:border-emerald-400/60 hover:shadow-2xl hover:shadow-emerald-400/30 backdrop-blur-sm"
              >
                <div
                  className="inline-flex items-center gap-3"
                  style={{
                    boxShadow: '0 0 20px rgba(52, 211, 153, 0.4)',
                  }}
                >
                  <span className="text-2xl text-emerald-400">∞</span>
                  <span className="text-sm font-medium text-emerald-300">Stage 6</span>
                  <span className="text-white/40">•</span>
                  <span className="text-sm text-emerald-200/60">Beyond Time</span>
                </div>
                <span className="text-xs text-emerald-400/60 group-hover:text-emerald-400 transition-colors">
                  [ Hover or click to reveal the portal ]
                </span>
              </button>
            </motion.div>
          ) : (
            <div>
              {/* Stage 6 Label */}
              <motion.div
                initial={{ opacity: 0, scale: 0.9 }}
                animate={{ opacity: 1, scale: 1 }}
                transition={{ duration: 0.5 }}
                className="mb-8 text-center"
              >
                <div
                  className="inline-flex items-center gap-3 rounded-2xl border-2 border-emerald-400/50 bg-gradient-to-r from-emerald-500/20 via-green-500/20 to-cyan-500/20 px-5 py-2.5 backdrop-blur-sm"
                  style={{
                    boxShadow: '0 0 40px rgba(52, 211, 153, 0.5), 0 0 80px rgba(52, 211, 153, 0.2)',
                  }}
                >
                  <span className="text-xl animate-pulse text-emerald-400">∞</span>
                  <span className="text-sm font-medium text-emerald-300">{stage6.label}</span>
                  <span className="text-emerald-400/40">•</span>
                  <span className="text-sm text-emerald-200/80">{stage6.timeframe}</span>
                  <span className="text-xs text-emerald-400/60">🌀</span>
                </div>
              </motion.div>

              {/* Stage 6 Cards */}
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.6, delay: 0.2 }}
                className="grid gap-6 sm:grid-cols-2 lg:grid-cols-3"
              >
                {stage6.outcomes.map((outcome, index) => (
                  <ImpactCard key={outcome.outcome} outcome={outcome} index={index} />
                ))}
              </motion.div>
            </div>
          )}
        </div>

            {/* Collapse Button */}
            <div className="mt-16 flex justify-center">
              <button
                onClick={handleCollapse}
                className="group inline-flex items-center gap-2 text-sm text-white/60 transition-colors hover:text-white"
              >
                Collapse timeline ↑
              </button>
            </div>
          </>
        )}
      </div>
    </section>
  );
}
