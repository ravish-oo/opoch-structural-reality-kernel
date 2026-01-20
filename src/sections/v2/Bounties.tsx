import { useState, useCallback, memo } from 'react';
import { motion } from 'framer-motion';
import { ExternalLink, MessageSquare, ChevronDown, ChevronUp } from 'lucide-react';
import { trackToeBountiesExpanded, trackToeBountiesCollapsed, trackToeBountyDetailsClicked, trackToeBountyChatClicked } from '../../lib/analytics';

interface Bounty {
  id: string;
  prize: string;
  prizeAmount: number; // for sorting
  icon: string;
  name: string;
  description: string;
  fields: string[];
  toeApproach: string;
  externalUrl: string;
  status: 'Active' | 'Deadline' | 'Open';
  deadline?: string;
}

const bounties: Bounty[] = [
  // Top 6 - Visible Initially
  {
    id: 'arc-agi',
    prize: '$1,000,000',
    prizeAmount: 1000000,
    icon: '🤖',
    name: 'ARC-AGI Challenge',
    description: 'Build AI that learns like humans',
    fields: ['AI', 'Machine Learning', 'AGI'],
    toeApproach: 'Universal intelligence compiler',
    externalUrl: 'https://arcprize.org',
    status: 'Deadline',
    deadline: 'Nov 3, 2025',
  },
  {
    id: 'indus-script',
    prize: '$1,000,000',
    prizeAmount: 1000000,
    icon: '📜',
    name: 'Indus Valley Script',
    description: 'Decipher 5,300-year-old writing',
    fields: ['Linguistics', 'Archaeology', 'AI'],
    toeApproach: 'Pattern recognition via SCFA',
    externalUrl: 'https://archaeologymag.com/2025/01/prize-offered-to-decipher-indus-valley-script/',
    status: 'Open',
  },
  {
    id: 'riemann',
    prize: '$1,000,000',
    prizeAmount: 1000000,
    icon: '🔢',
    name: 'Riemann Hypothesis',
    description: 'Where do prime numbers hide?',
    fields: ['Mathematics', 'Number Theory'],
    toeApproach: 'Π/FY grammar forces constraints',
    externalUrl: 'https://www.claymath.org/millennium/riemann-hypothesis/',
    status: 'Open',
  },
  {
    id: 'yang-mills',
    prize: '$1,000,000',
    prizeAmount: 1000000,
    icon: '⚛️',
    name: 'Yang-Mills Mass Gap',
    description: 'Why do particles have mass?',
    fields: ['Physics', 'Quantum Field Theory'],
    toeApproach: 'Mass = paid stiffness in gauge orbits',
    externalUrl: 'https://www.claymath.org/millennium/yang-mills/',
    status: 'Open',
  },
  {
    id: 'navier-stokes',
    prize: '$1,000,000',
    prizeAmount: 1000000,
    icon: '🌊',
    name: 'Navier-Stokes Equations',
    description: 'Do fluid equations always work?',
    fields: ['Mathematics', 'Physics'],
    toeApproach: 'FY balance + smoothness conditions',
    externalUrl: 'https://www.claymath.org/millennium/navier-stokes/',
    status: 'Open',
  },
  // Hidden Initially - Cards 7-12
  {
    id: 'birch-swinnerton-dyer',
    prize: '$1,000,000',
    prizeAmount: 1000000,
    icon: '📐',
    name: 'Birch and Swinnerton-Dyer',
    description: 'Counting solutions to equations',
    fields: ['Mathematics', 'Algebraic Geometry'],
    toeApproach: 'Elliptic curves in FY framework',
    externalUrl: 'https://www.claymath.org/millennium/birch-swinnerton-dyer/',
    status: 'Open',
  },
  {
    id: 'hodge',
    prize: '$1,000,000',
    prizeAmount: 1000000,
    icon: '🔷',
    name: 'Hodge Conjecture',
    description: 'Are shapes made of algebra?',
    fields: ['Mathematics', 'Topology'],
    toeApproach: 'Sheaf structure cohomology',
    externalUrl: 'https://www.claymath.org/millennium/hodge-conjecture/',
    status: 'Open',
  },
  {
    id: 'erdos-1',
    prize: '$10,000',
    prizeAmount: 10000,
    icon: '🎯',
    name: 'Selected Erdős Problems',
    description: 'Graph theory chromatic number',
    fields: ['Mathematics', 'Combinatorics'],
    toeApproach: 'Combinatorial optimization',
    externalUrl: 'https://www.erdosproblems.com/',
    status: 'Open',
  },
  {
    id: 'erdos-2',
    prize: '$5,000',
    prizeAmount: 5000,
    icon: '🎯',
    name: 'Selected Erdős Problems',
    description: 'Prime gaps and distribution',
    fields: ['Mathematics', 'Number Theory'],
    toeApproach: 'Number theoretic bounds',
    externalUrl: 'https://www.erdosproblems.com/',
    status: 'Open',
  },
  {
    id: 'erdos-3',
    prize: '$3,000',
    prizeAmount: 3000,
    icon: '🎯',
    name: 'Selected Erdős Problems',
    description: 'Ramsey theory bounds',
    fields: ['Mathematics', 'Combinatorics'],
    toeApproach: 'Graph structure analysis',
    externalUrl: 'https://www.erdosproblems.com/',
    status: 'Open',
  },
  {
    id: 'erdos-4',
    prize: '$1,000',
    prizeAmount: 1000,
    icon: '🎯',
    name: 'Selected Erdős Problems',
    description: 'Partition and sum-free sets',
    fields: ['Mathematics', 'Number Theory'],
    toeApproach: 'Additive combinatorics',
    externalUrl: 'https://www.erdosproblems.com/',
    status: 'Open',
  },
];

const BountyCard = memo(({ bounty, index, onChatClick }: {
  bounty: Bounty;
  index: number;
  onChatClick: (bountyId: string, bountyName: string) => void;
}) => {
  const statusColors = {
    Active: 'bg-emerald-500/20 text-emerald-300',
    Deadline: 'bg-amber-500/20 text-amber-300',
    Open: 'bg-sky-500/20 text-sky-300',
  };

  const handleDetailsClick = useCallback(() => {
    trackToeBountyDetailsClicked(bounty.id);
  }, [bounty.id]);

  const handleChatClick = useCallback(() => {
    onChatClick(bounty.id, bounty.name);
  }, [bounty.id, bounty.name, onChatClick]);

  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      whileInView={{ opacity: 1, y: 0 }}
      viewport={{ once: true }}
      transition={{ duration: 0.4, delay: index * 0.05 }}
      className="group relative flex flex-col rounded-2xl border border-white/10 bg-white/5 p-6 hover:border-white/20 hover:bg-white/10 transition-all"
    >
      {/* Top Row - Prize + Status */}
      <div className="mb-4 flex items-start justify-between">
        {/* Prize Money */}
        <div className="text-3xl font-bold text-emerald-400">
          💰 {bounty.prize}
        </div>

        {/* Status Badge */}
        <span className={`rounded-md px-2 py-0.5 text-xs font-medium ${statusColors[bounty.status]}`}>
          {bounty.deadline || bounty.status}
        </span>
      </div>

      {/* Icon + Problem Name */}
      <div className="mb-2 flex items-center gap-2">
        <span className="text-2xl">{bounty.icon}</span>
        <h3 className="text-lg font-semibold text-white">{bounty.name}</h3>
      </div>

      {/* Description */}
      <p className="mb-3 text-sm text-white/60">{bounty.description}</p>

      {/* Field Tags */}
      <div className="mb-3 flex flex-wrap items-center gap-2 text-xs text-white/50">
        <span>🏷️</span>
        {bounty.fields.map((field, idx) => (
          <span key={field}>
            {field}
            {idx < bounty.fields.length - 1 && <span className="mx-1">•</span>}
          </span>
        ))}
      </div>

      {/* TOE Approach Hint */}
      <div className="mb-4 flex items-start gap-2 text-sm italic text-white/60">
        <span className="flex-shrink-0">💡</span>
        <span>TOE approach: {bounty.toeApproach}</span>
      </div>

      {/* CTAs */}
      <div className="mt-auto flex flex-wrap items-center gap-3">
        {/* Primary CTA - Chat */}
        <button
          onClick={handleChatClick}
          className="inline-flex items-center gap-2 rounded-2xl bg-white/10 px-4 py-2 text-sm font-medium text-white transition-all hover:bg-white/20"
        >
          <MessageSquare className="h-4 w-4" />
          Chat about this →
        </button>

        {/* Secondary CTA - External Link */}
        <a
          href={bounty.externalUrl}
          target="_blank"
          rel="noopener noreferrer"
          onClick={handleDetailsClick}
          className="inline-flex items-center gap-2 text-sm text-opoch-cyan-light transition-colors hover:text-opoch-cyan"
        >
          Problem details
          <ExternalLink className="h-3.5 w-3.5" />
        </a>
      </div>
    </motion.div>
  );
});

BountyCard.displayName = 'BountyCard';

export default function Bounties() {
  const [isExpanded, setIsExpanded] = useState(false);

  const visibleBounties = isExpanded ? bounties : bounties.slice(0, 6);

  const totalPrize = bounties.reduce((sum, b) => sum + b.prizeAmount, 0);
  const formattedTotal = `$${(totalPrize / 1000000).toFixed(1)}M+`;

  const handleExpand = useCallback(() => {
    setIsExpanded(true);
    trackToeBountiesExpanded();
  }, []);

  const handleCollapse = useCallback(() => {
    setIsExpanded(false);
    trackToeBountiesCollapsed();
  }, []);

  const handleChatClick = useCallback((bountyId: string, bountyName: string) => {
    // Track the chat button click
    trackToeBountyChatClicked(bountyId, bountyName);

    // Always redirect to chat - let chat.opoch.com handle authentication
    const chatUrl = import.meta.env.DEV ? 'http://localhost:3000' : 'https://chat.opoch.com';
    window.location.href = chatUrl;
  }, []);

  return (
    <section id="bounties" className="border-t border-white/10 bg-black px-4 py-24 sm:px-6 lg:px-8">
      <div className="mx-auto max-w-7xl">
        {/* Stats Bar */}
        <motion.div
          initial={{ opacity: 0, y: 10 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5 }}
          viewport={{ once: true }}
          className="mb-4 text-center text-sm text-white/60"
        >
          {formattedTotal} in Prize Money • {bounties.length} Active Bounties in the World
        </motion.div>

        {/* Section Heading */}
        <motion.div
          initial={{ opacity: 0, y: 10 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6 }}
          viewport={{ once: true }}
          className="mb-6 text-center"
        >
          <h2 className="mb-3 text-balance text-4xl font-semibold text-white md:text-5xl">
            Apply the Theory, Win Real Money
          </h2>
          <p className="text-lg text-white/60">
            Major unsolved problems with verified prize money. Use TOE to crack them.
          </p>
        </motion.div>

        {/* Bounty Cards Grid */}
        <div className="mt-12 grid gap-6 sm:grid-cols-2 lg:grid-cols-3">
          {visibleBounties.map((bounty, index) => (
            <BountyCard key={bounty.id} bounty={bounty} index={index} onChatClick={handleChatClick} />
          ))}
        </div>

        {/* Expand/Collapse Button */}
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
              View all bounties
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

        {/* After Section Text */}
        <motion.div
          initial={{ opacity: 0 }}
          whileInView={{ opacity: 1 }}
          transition={{ duration: 0.5, delay: 0.3 }}
          viewport={{ once: true }}
          className="mt-16 text-center"
        >
          <p className="mb-2 text-white/60">
            Work with our chat now to crack these!
          </p>
          <p className="text-xs text-amber-400/60">
            Coming soon: Claude Code prompts, code anchors & exact math for bounties
          </p>
        </motion.div>
      </div>

      {/* Auth Modal (lazy loaded) */}
      {/* Auth modal removed - now redirecting to chat.opoch.com */}
    </section>
  );
}
