import { useState, useCallback, memo } from 'react';
import { Check, X, AlertCircle, ExternalLink, ChevronDown, ChevronUp } from 'lucide-react';
import { motion } from 'framer-motion';
import { trackToeComparisonTableExpanded, trackToeComparisonTableCollapsed, trackToeVerifyClicked } from '../../lib/analytics';

type Status = 'solved' | 'unsolved' | 'partial';

interface ComparisonRow {
  problem: string;
  ourOneLiner: string;
  ours: { status: Status; verifyUrl: string };
  stringTheory: { status: Status; text: string };
  loopQG: { status: Status; text: string };
  mTheory: { status: Status; text: string };
}

const comparisonData: ComparisonRow[] = [
  {
    problem: 'Consciousness',
    ourOneLiner: 'Reflexive confluence—observer = observed when free↔paid loops have zero holonomy',
    ours: { status: 'solved', verifyUrl: 'https://x.com/grok/status/1981189018521129344' },
    stringTheory: { status: 'unsolved', text: 'Not addressed' },
    loopQG: { status: 'unsolved', text: 'Not addressed' },
    mTheory: { status: 'unsolved', text: 'Not addressed' },
  },
  {
    problem: 'Quantum gravity',
    ourOneLiner: 'Free sector = exact Kähler (unitary); GR from diffeo + FY balance; orthogonal split',
    ours: { status: 'solved', verifyUrl: 'https://x.com/grok/status/1981171877134598492' },
    stringTheory: { status: 'partial', text: 'Requires 11-D, no predictions' },
    loopQG: { status: 'partial', text: 'No unique state' },
    mTheory: { status: 'partial', text: 'Incomplete' },
  },
  {
    problem: 'Black hole information paradox',
    ourOneLiner: 'Horizons are FY/Green faces; unitary flux + paid boundary writes conserve truth exactly',
    ours: { status: 'solved', verifyUrl: 'https://x.com/grok/status/1981176823884238956' },
    stringTheory: { status: 'partial', text: 'Debated' },
    loopQG: { status: 'partial', text: 'Partial' },
    mTheory: { status: 'unsolved', text: 'Unresolved' },
  },
  {
    problem: 'Quantum measurement & nonlocality',
    ourOneLiner: 'Measurement = local FY-tight paid update; Born weights are unique SCFA measure',
    ours: { status: 'solved', verifyUrl: 'https://x.com/grok/status/1981169003344384007' },
    stringTheory: { status: 'unsolved', text: 'Not resolved' },
    loopQG: { status: 'unsolved', text: 'Not resolved' },
    mTheory: { status: 'unsolved', text: 'Not resolved' },
  },
  {
    problem: 'Dark matter / dark energy',
    ourOneLiner: 'Ledger/face terms from coarse interfaces and symmetry choices, not new particles',
    ours: { status: 'solved', verifyUrl: 'https://x.com/grok/status/1981181068813488400' },
    stringTheory: { status: 'partial', text: 'Additional fields' },
    loopQG: { status: 'unsolved', text: 'Not addressed' },
    mTheory: { status: 'partial', text: 'Speculative' },
  },
  {
    problem: 'Arrow of time',
    ourOneLiner: 'Time = bits paid. Ledger equalities make irreversibility the price of exact writes',
    ours: { status: 'solved', verifyUrl: 'https://x.com/grok/status/1981173107764449451' },
    stringTheory: { status: 'unsolved', text: 'Not addressed' },
    loopQG: { status: 'unsolved', text: 'Not addressed' },
    mTheory: { status: 'unsolved', text: 'Not addressed' },
  },
  {
    problem: 'Spacetime & light',
    ourOneLiner: 'Free finite-speed isometries force Minkowski interval; c = invariant slope',
    ours: { status: 'solved', verifyUrl: 'https://x.com/grok/status/1981175132124303538' },
    stringTheory: { status: 'partial', text: 'Emergent spacetime' },
    loopQG: { status: 'partial', text: 'Discrete structure' },
    mTheory: { status: 'partial', text: 'Branes/dimensions' },
  },
  {
    problem: 'Mass & gauge origin',
    ourOneLiner: 'Gauge = free isometries; mass = paid stiffness; gaps = minimal paid energy',
    ours: { status: 'solved', verifyUrl: 'https://x.com/grok/status/1981183145723699683' },
    stringTheory: { status: 'partial', text: 'String modes' },
    loopQG: { status: 'unsolved', text: 'Not derived' },
    mTheory: { status: 'partial', text: 'Brane dynamics' },
  },
  {
    problem: 'Physical constants',
    ourOneLiner: 'Forms fixed by grammar; numbers are measured boundary bills with dependencies',
    ours: { status: 'solved', verifyUrl: 'https://x.com/grok/status/1981185586968113158' },
    stringTheory: { status: 'unsolved', text: 'Landscape problem' },
    loopQG: { status: 'unsolved', text: 'Not addressed' },
    mTheory: { status: 'unsolved', text: 'Multiverse speculation' },
  },
  {
    problem: 'Life & evolution',
    ourOneLiner: 'Free symmetries + minimal bits to hold low divergence; selection = long-run cost optimization',
    ours: { status: 'solved', verifyUrl: 'https://x.com/grok/status/1981553442041815353' },
    stringTheory: { status: 'unsolved', text: 'Not addressed' },
    loopQG: { status: 'unsolved', text: 'Not addressed' },
    mTheory: { status: 'unsolved', text: 'Not addressed' },
  },
];

const StatusIcon = memo(({ status }: { status: Status }) => {
  if (status === 'solved') {
    return (
      <div className="flex h-8 w-8 items-center justify-center rounded-full bg-emerald-500/20">
        <Check className="h-4 w-4 text-emerald-300" />
      </div>
    );
  }
  if (status === 'partial') {
    return (
      <div className="flex h-8 w-8 items-center justify-center rounded-full bg-amber-500/20">
        <AlertCircle className="h-4 w-4 text-amber-300" />
      </div>
    );
  }
  return (
    <div className="flex h-8 w-8 items-center justify-center rounded-full bg-red-500/20">
      <X className="h-4 w-4 text-red-300" />
    </div>
  );
});

export default function ComparisonTable() {
  const [isExpanded, setIsExpanded] = useState(false);
  const visibleRows = isExpanded ? comparisonData : comparisonData.slice(0, 6);

  const handleExpand = useCallback(() => {
    setIsExpanded(true);
    trackToeComparisonTableExpanded();
  }, []);

  const handleCollapse = useCallback(() => {
    setIsExpanded(false);
    trackToeComparisonTableCollapsed();
  }, []);

  const handleVerifyClick = useCallback((problem: string) => {
    trackToeVerifyClicked(problem);
  }, []);

  return (
    <section className="border-t border-white/10 bg-white/5 py-24">
      <div className="mx-auto max-w-7xl px-4">
        <motion.h2
          initial={{ opacity: 0, y: 10 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6 }}
          viewport={{ once: true }}
          className="mb-16 text-balance text-center text-3xl font-semibold text-white md:text-5xl"
        >
          Comparison with Other TOE Candidates
        </motion.h2>

        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5 }}
          viewport={{ once: true }}
          className="overflow-x-auto"
        >
          <div className="inline-block min-w-full rounded-2xl border border-white/10 bg-black">
            <table className="w-full">
              <thead>
                <tr className="border-b border-white/10 bg-white/5">
                  <th className="px-8 py-5 text-left text-sm font-semibold tracking-tight text-white/90">
                    Problem
                  </th>
                  <th className="px-8 py-5 text-center text-sm font-semibold tracking-tight text-opoch-cyan-light">
                    Theory of Everything
                  </th>
                  <th className="px-8 py-5 text-center text-sm font-semibold tracking-tight text-sky-300/80">
                    String Theory
                  </th>
                  <th className="px-8 py-5 text-center text-sm font-semibold tracking-tight text-purple-300/80">
                    Loop QG
                  </th>
                  <th className="px-8 py-5 text-center text-sm font-semibold tracking-tight text-pink-300/80">
                    M-Theory
                  </th>
                </tr>
              </thead>
              <tbody>
                {visibleRows.map((row, index) => (
                  <motion.tr
                    key={row.problem}
                    initial={{ opacity: 0, y: 20 }}
                    whileInView={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.4, delay: index * 0.02 }}
                    viewport={{ once: true }}
                    className="group border-b border-white/10 transition-all hover:bg-white/5"
                  >
                    <td className="px-8 py-6 text-sm font-medium text-white">
                      {row.problem}
                    </td>

                    {/* Ours column - always show ExternalLink icon */}
                    <td className="px-8 py-6">
                      <div className="flex justify-center">
                        {index === 0 ? (
                          <a
                            href={row.ours.verifyUrl}
                            target="_blank"
                            rel="noopener noreferrer"
                            onClick={() => handleVerifyClick(row.problem)}
                            className="group/link inline-flex items-center gap-2 rounded-2xl border border-emerald-500/30 bg-emerald-500/10 px-4 py-2 transition-all hover:border-emerald-500/50 hover:bg-emerald-500/20"
                            title="Verify solution"
                          >
                            <div className="flex h-6 w-6 items-center justify-center rounded-full bg-emerald-500/30">
                              <Check className="h-3.5 w-3.5 text-emerald-300" />
                            </div>
                            <span className="text-xs font-medium text-emerald-300">
                              Verify
                            </span>
                            <ExternalLink className="h-3 w-3 text-emerald-300/60 transition-all group-hover/link:text-emerald-300" />
                          </a>
                        ) : (
                          <a
                            href={row.ours.verifyUrl}
                            target="_blank"
                            rel="noopener noreferrer"
                            onClick={() => handleVerifyClick(row.problem)}
                            className="group/link relative flex items-center gap-1 transition-all"
                          >
                            <div className="flex h-8 w-8 items-center justify-center rounded-full bg-emerald-500/20 transition-all group-hover/link:bg-emerald-500/30">
                              <Check className="h-4 w-4 text-emerald-300" />
                            </div>
                            <ExternalLink className="h-3 w-3 text-emerald-300/60 transition-all group-hover/link:text-emerald-300" />
                            <span className="pointer-events-none absolute left-1/2 top-full mt-2 -translate-x-1/2 whitespace-nowrap rounded-lg border border-white/20 bg-black px-3 py-1.5 text-xs text-white/90 opacity-0 shadow-lg transition-opacity group-hover/link:opacity-100">
                              Verify
                            </span>
                          </a>
                        )}
                      </div>
                    </td>

                    {/* String Theory column - hide text, show on hover */}
                    <td className="px-8 py-6">
                      <div className="group/cell relative flex justify-center">
                        <StatusIcon status={row.stringTheory.status} />
                        <span className="pointer-events-none absolute left-1/2 top-1/2 z-10 -translate-x-1/2 -translate-y-1/2 whitespace-nowrap rounded-lg border border-white/20 bg-black px-3 py-1.5 text-xs text-white/90 opacity-0 shadow-lg transition-opacity group-hover/cell:opacity-100">
                          {row.stringTheory.text}
                        </span>
                      </div>
                    </td>

                    {/* Loop QG column - hide text, show on hover */}
                    <td className="px-8 py-6">
                      <div className="group/cell relative flex justify-center">
                        <StatusIcon status={row.loopQG.status} />
                        <span className="pointer-events-none absolute left-1/2 top-1/2 z-10 -translate-x-1/2 -translate-y-1/2 whitespace-nowrap rounded-lg border border-white/20 bg-black px-3 py-1.5 text-xs text-white/90 opacity-0 shadow-lg transition-opacity group-hover/cell:opacity-100">
                          {row.loopQG.text}
                        </span>
                      </div>
                    </td>

                    {/* M-Theory column - hide text, show on hover */}
                    <td className="px-8 py-6">
                      <div className="group/cell relative flex justify-center">
                        <StatusIcon status={row.mTheory.status} />
                        <span className="pointer-events-none absolute left-1/2 top-1/2 z-10 -translate-x-1/2 -translate-y-1/2 whitespace-nowrap rounded-lg border border-white/20 bg-black px-3 py-1.5 text-xs text-white/90 opacity-0 shadow-lg transition-opacity group-hover/cell:opacity-100">
                          {row.mTheory.text}
                        </span>
                      </div>
                    </td>
                  </motion.tr>
                ))}
                {!isExpanded ? (
                  <tr className="border-b-0">
                    <td colSpan={5} className="px-8 py-0">
                      <button
                        onClick={handleExpand}
                        className="group flex w-full items-center justify-center gap-2 py-6 text-sm text-white/60 transition-all hover:text-white"
                      >
                        <span className="font-medium">View all {comparisonData.length} comparisons</span>
                        <ChevronDown className="h-4 w-4 transition-transform group-hover:translate-y-0.5" />
                      </button>
                    </td>
                  </tr>
                ) : (
                  <tr className="border-b-0">
                    <td colSpan={5} className="px-8 py-0">
                      <button
                        onClick={handleCollapse}
                        className="group flex w-full items-center justify-center gap-2 py-6 text-sm text-white/60 transition-all hover:text-white"
                      >
                        <span className="font-medium">Show less</span>
                        <ChevronUp className="h-4 w-4 transition-transform group-hover:-translate-y-0.5" />
                      </button>
                    </td>
                  </tr>
                )}
              </tbody>
            </table>
          </div>
        </motion.div>
      </div>
    </section>
  );
}
