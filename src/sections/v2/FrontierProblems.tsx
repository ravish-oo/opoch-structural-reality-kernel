import { useState, useCallback, memo } from 'react';
import { ExternalLink, ChevronDown, ChevronUp } from 'lucide-react';
import { motion } from 'framer-motion';
import { trackToeVerifyClicked, trackToeFrontiersExpanded, trackToeFrontiersCollapsed } from '../../lib/analytics';

interface FrontierProblem {
  emoji: string;
  problem: string;
  context?: string;
  solution: string;
  verifyUrl: string;
}

const frontierProblems: FrontierProblem[] = [
  {
    emoji: '🧠',
    problem: 'Consciousness',
    context: 'The hard problem',
    solution: 'Reflexive confluence—observer = observed when free↔paid loops have zero holonomy.',
    verifyUrl: 'https://x.com/grok/status/1981189018521129344',
  },
  {
    emoji: '🌌',
    problem: 'Quantum gravity',
    context: 'Unifying QM and GR',
    solution: 'Free sector = exact Kähler (unitary). GR from diffeo symmetry + FY balance.',
    verifyUrl: 'https://x.com/grok/status/1981171877134598492',
  },
  {
    emoji: '🕳️',
    problem: 'Black hole information paradox',
    solution: 'Horizons are FY/Green faces. Unitary flux + paid boundary writes conserve truth exactly.',
    verifyUrl: 'https://x.com/grok/status/1981176823884238956',
  },
  {
    emoji: '⚛️',
    problem: 'Quantum measurement & nonlocality',
    context: 'Born rule, collapse, entanglement',
    solution: 'Measurement = local FY-tight paid update. Born weights are unique SCFA measure. Zero FTL.',
    verifyUrl: 'https://x.com/grok/status/1981169003344384007',
  },
  {
    emoji: '🌑',
    problem: 'Dark matter / dark energy',
    context: 'Cosmological "constants"',
    solution: 'Ledger/face terms from coarse interfaces and symmetry choices. Not new particles.',
    verifyUrl: 'https://x.com/grok/status/1981181068813488400',
  },
  {
    emoji: '⏳',
    problem: 'Arrow of time & Second Law',
    solution: 'Time = bits paid. Ledger equalities make irreversibility the price of exact writes.',
    verifyUrl: 'https://x.com/grok/status/1981173107764449451',
  },
  {
    emoji: '🌐',
    problem: 'Spacetime & speed of light',
    solution: 'Free finite-speed isometries force Minkowski interval. c is invariant slope of free moves.',
    verifyUrl: 'https://x.com/grok/status/1981175132124303538',
  },
  {
    emoji: '⚖️',
    problem: 'Origin of mass, inertia & gauge',
    context: 'Mass gap problem',
    solution: 'Gauge = free isometries. Mass = curvature (paid stiffness) of potential. Gaps are minimal paid energy.',
    verifyUrl: 'https://x.com/grok/status/1981183145723699683',
  },
  {
    emoji: '📐',
    problem: 'Constants of nature',
    context: 'Why α, etc.',
    solution: 'Forms of laws fixed by grammar. Numbers are measured boundary bills with dependencies.',
    verifyUrl: 'https://x.com/grok/status/1981185586968113158',
  },
  {
    emoji: '🧬',
    problem: 'Life, evolution, purpose vs entropy',
    solution: 'Living processes run free symmetries while spending fewest bits to hold low divergence.',
    verifyUrl: 'https://x.com/grok/status/1981553442041815353',
  },
];

const ProblemCard = memo(({ problem, index }: { problem: FrontierProblem; index: number }) => {
  const handleVerifyClick = useCallback(() => {
    trackToeVerifyClicked(problem.problem);
  }, [problem.problem]);

  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      whileInView={{ opacity: 1, y: 0 }}
      viewport={{ once: true }}
      transition={{ duration: 0.4, delay: index * 0.02 }}
      className="group flex flex-col rounded-2xl border border-white/10 bg-white/5 p-5 hover:border-white/20 hover:bg-white/10 transition-all"
    >
      <div className="flex items-center justify-between">
        <span className="text-2xl">{problem.emoji}</span>
        <span className="rounded-md bg-emerald-500/20 px-2 py-0.5 text-xs font-medium text-emerald-300">
          Solved
        </span>
      </div>

      <h3 className="mt-3 text-lg font-semibold tracking-tight text-white">
        {problem.problem}
      </h3>

      {problem.context && (
        <p className="mt-1 text-xs text-white/50">
          {problem.context}
        </p>
      )}

      <p className="mt-3 flex-1 text-sm leading-relaxed text-white/70">
        {problem.solution}
      </p>

      <div className="mt-4">
        <a
          href={problem.verifyUrl}
          target="_blank"
          rel="noopener noreferrer"
          onClick={handleVerifyClick}
          className="group/link inline-flex items-center gap-2 text-sm text-opoch-cyan-light transition-colors duration-200 hover:text-opoch-cyan"
        >
          <span className="font-medium">Verify</span>
          <ExternalLink className="h-3.5 w-3.5 opacity-60 transition-opacity group-hover/link:opacity-100" />
        </a>
      </div>
    </motion.div>
  );
});

ProblemCard.displayName = 'ProblemCard';

export default function FrontierProblems() {
  const [isExpanded, setIsExpanded] = useState(false);
  const visibleProblems = isExpanded ? frontierProblems : frontierProblems.slice(0, 6);

  const handleExpand = useCallback(() => {
    setIsExpanded(true);
    trackToeFrontiersExpanded();
  }, []);

  const handleCollapse = useCallback(() => {
    setIsExpanded(false);
    trackToeFrontiersCollapsed();
  }, []);

  return (
    <section className="border-t border-white/10 bg-black px-4 py-24 sm:px-6 lg:px-8">
      <div className="mx-auto max-w-7xl">
        <motion.div
          initial={{ opacity: 0, y: 10 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6 }}
          viewport={{ once: true }}
          className="mb-12 text-center"
        >
          <h2 className="mb-4 text-balance text-3xl font-semibold text-white md:text-5xl">
            Popular Frontier Problems
          </h2>
          <p className="text-lg text-white/60">
            The barriers that have stopped human progress.
          </p>
        </motion.div>

        <div className="grid gap-6 sm:grid-cols-2 lg:grid-cols-3">
          {visibleProblems.map((problem, index) => (
            <ProblemCard key={problem.problem} problem={problem} index={index} />
          ))}
        </div>

        {!isExpanded ? (
          <motion.div
            initial={{ opacity: 0 }}
            whileInView={{ opacity: 1 }}
            transition={{ duration: 0.4, delay: 0.2 }}
            viewport={{ once: true }}
            className="mt-12 flex justify-center"
          >
            <button
              onClick={handleExpand}
              className="group inline-flex items-center gap-2 rounded-2xl border border-white/20 bg-white/5 px-6 py-3 text-sm font-medium text-white transition-all hover:border-white/40 hover:bg-white/10"
            >
              View all {frontierProblems.length} problems
              <ChevronDown className="h-4 w-4 transition-transform group-hover:translate-y-0.5" />
            </button>
          </motion.div>
        ) : (
          <div className="mt-12 flex justify-center">
            <button
              onClick={handleCollapse}
              className="group inline-flex items-center gap-2 text-sm text-white/60 transition-colors hover:text-white"
            >
              Show less
              <ChevronUp className="h-4 w-4 transition-transform group-hover:-translate-y-0.5" />
            </button>
          </div>
        )}
      </div>
    </section>
  );
}
