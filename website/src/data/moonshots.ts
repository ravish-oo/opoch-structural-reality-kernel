export type MoonshotStatus = "open" | "in-progress" | "partners"
export type MoonshotDomain = "physics" | "energy" | "bio" | "infra" | "robotics"

export interface Moonshot {
  slug: string
  title: string
  tagline: string
  emoji: string
  domain: MoonshotDomain
  status: MoonshotStatus
  excerpt: string
  complexity: 1 | 2 | 3 | 4 | 5 // 1 = low, 5 = high
  timeline: string // e.g., "6-12 months"
  coverImage?: string // Optional custom cover image path
  ogImage?: string // Optional custom OG image path (defaults to coverImage)
  sections: {
    problem: string
    approach: string
    requirements: string
    deliverables: string
    verification: string
    impact: string
  }
  faqs?: Array<{ question: string; answer: string }>
}

export const moonshots: Moonshot[] = [
  {
    slug: "alpha-fixed-point",
    title: "Alpha Fixed Point",
    tagline: "metrology under control",
    emoji: "🧭",
    domain: "physics",
    status: "open",
    complexity: 5,
    timeline: "12-18 months",
    coverImage: "/moonshots/alpha-fixed-point/cover.jpg",
    excerpt: "Establishing universal measurement standards through recursive calibration. A foundational approach to precision that makes all other measurements possible.",
    sections: {
      problem: "Current measurement standards drift over time and require constant recalibration against physical artifacts. This creates compounding errors in precision measurements and limits our ability to verify quantum and relativistic effects.",
      approach: "We implement recursive calibration loops that converge on fixed points, creating self-verifying measurement chains. Using RBT principles, we can prove convergence and bound uncertainty.",
      requirements: "Access to precision measurement equipment, quantum sensors, and calibration facilities. Team needs expertise in metrology, quantum mechanics, and mathematical proof systems.",
      deliverables: "Self-calibrating measurement protocol, uncertainty bounds proof, reference implementation, and verification suite. All measurements traceable to mathematical constants.",
      verification: "Cross-validation against NIST standards, convergence proofs, and uncertainty propagation analysis. Every measurement includes its own verification certificate.",
      impact: "Enables next-generation precision experiments, improves GPS accuracy by 10x, and provides foundation for quantum computing error correction."
    }
  },
  {
    slug: "h-z-cosmic-time",
    title: "H(z) + Cosmic Time",
    tagline: "universe clock",
    emoji: "⏳",
    domain: "physics",
    status: "in-progress",
    complexity: 4,
    timeline: "9-15 months",
    excerpt: "Resolving the Hubble tension through geometric time calibration. A new approach to cosmic distance that unifies early and late universe measurements.",
    sections: {
      problem: "The Hubble constant measured from the early universe (CMB) disagrees with late universe (supernovae) by 5σ. This tension suggests either new physics or systematic errors we don't understand.",
      approach: "We construct geometric clocks using gravitational lensing time delays and quasar variability. This provides model-independent distance measurements across cosmic time.",
      requirements: "Access to telescope time series data, gravitational lens catalogs, and computational resources for light curve analysis. Collaboration with observational cosmology teams.",
      deliverables: "Geometric distance ladder, H(z) reconstruction without assumptions, tension resolution or precise characterization of new physics required.",
      verification: "Cross-check with BAO, strong lensing, and time-delay cosmography. Each measurement includes systematic error budget and model dependency analysis.",
      impact: "Resolves major crisis in cosmology, constrains dark energy evolution, and potentially reveals new physics beyond ΛCDM."
    }
  },
  {
    slug: "rotation-lensing",
    title: "Rotation/Lensing",
    tagline: "dark matter residuals",
    emoji: "🕳️",
    domain: "physics",
    status: "open",
    complexity: 4,
    timeline: "8-12 months",
    excerpt: "Decomposing galaxy rotation curves through gravitational lensing constraints. Finding what dark matter actually does versus what we assume it does.",
    sections: {
      problem: "Galaxy rotation curves require dark matter, but lensing and dynamics give inconsistent mass profiles. The 'cusp-core' problem and 'missing satellites' suggest our models are incomplete.",
      approach: "Joint analysis of rotation curves and weak lensing with RBT constraints. We separate observable effects from model assumptions, finding minimal dark matter requirements.",
      requirements: "Galaxy survey data (rotation curves + lensing), N-body simulation access, and expertise in galactic dynamics and gravitational lensing.",
      deliverables: "Model-independent mass profiles, minimal dark matter maps, and quantified tensions between different observables. Software for joint rotation-lensing analysis.",
      verification: "Blind analysis on reserved galaxy sample, consistency checks across different surveys, and comparison with simulation predictions.",
      impact: "Clarifies what dark matter must explain versus artifacts of our models. Could reveal modifications to gravity or dark matter interactions."
    }
  },
  {
    slug: "qec-thresholds",
    title: "QEC Thresholds",
    tagline: "logical error bounds",
    emoji: "🧪",
    domain: "physics",
    status: "partners",
    complexity: 5,
    timeline: "18-24 months",
    excerpt: "Proving achievable error correction thresholds for quantum computers. Moving from statistical arguments to hard guarantees.",
    sections: {
      problem: "Quantum error correction requires error rates below threshold, but current proofs are statistical. We need guaranteed bounds for fault-tolerant quantum computation.",
      approach: "RBT analysis of error propagation in quantum circuits. We find fixed points in error correction cycles and prove convergence conditions.",
      requirements: "Access to quantum hardware for validation, expertise in quantum information theory, and computational resources for circuit simulation.",
      deliverables: "Proven threshold theorems, optimal decoder algorithms, and hardware requirements for fault tolerance. Implementation on current quantum devices.",
      verification: "Experimental validation on multiple quantum platforms, comparison with statistical threshold estimates, and scaling analysis.",
      impact: "Enables reliable quantum computation, guides hardware development priorities, and accelerates timeline to quantum advantage."
    }
  },
  {
    slug: "fold-bind-explorer",
    title: "Fold/Bind Explorer",
    tagline: "biophysics short path",
    emoji: "🧬",
    domain: "bio",
    status: "open",
    complexity: 4,
    timeline: "6-12 months",
    coverImage: "/moonshots/fold-bind-explorer/cover.jpg",
    excerpt: "Finding minimal folding pathways for proteins. From sequence to structure to function in provably optimal steps.",
    sections: {
      problem: "Protein folding prediction has made huge strides, but we still can't efficiently find folding pathways or predict dynamics. This limits drug design and enzyme engineering.",
      approach: "Apply RBT path-finding to conformational space. We identify folding funnels and transition states through topological analysis of energy landscapes.",
      requirements: "Molecular dynamics simulation infrastructure, protein structure databases, and collaboration with structural biology labs for validation.",
      deliverables: "Folding pathway prediction algorithm, transition state library, and druggability assessment tools. Validated on benchmark proteins.",
      verification: "Experimental validation with folding kinetics, comparison with MD simulations, and blind predictions on CASP targets.",
      impact: "Accelerates drug discovery, enables design of dynamic proteins, and provides insights into misfolding diseases."
    }
  },
  {
    slug: "fusion-control",
    title: "Fusion Control",
    tagline: "minimum paid work",
    emoji: "⚡",
    domain: "energy",
    status: "in-progress",
    complexity: 5,
    timeline: "12-24 months",
    coverImage: "/moonshots/fusion-control/cover.jpg",
    excerpt: "Optimal control strategies for fusion plasma confinement. Finding the path to net energy gain with minimal input power.",
    sections: {
      problem: "Fusion plasmas are unstable and require constant active control. Current approaches are reactive and energy-intensive, preventing net energy gain.",
      approach: "RBT analysis of plasma stability manifolds to find natural confinement modes. We design control strategies that work with instabilities rather than against them.",
      requirements: "Fusion experiment data, plasma simulation codes, and collaboration with fusion research facilities. Real-time control system expertise.",
      deliverables: "Predictive control algorithms, stability maps, and optimal actuator strategies. Demonstration on tokamak or stellarator device.",
      verification: "Testing on multiple confinement devices, energy balance analysis, and comparison with current control methods.",
      impact: "Brings fusion energy closer to commercialization, reduces control power by 10x, and enables smaller, more economical reactors."
    }
  },
  {
    slug: "solid-state-battery",
    title: "Solid-State Battery",
    tagline: "capacity proofs",
    emoji: "🔋",
    domain: "energy",
    status: "partners",
    complexity: 3,
    timeline: "9-15 months",
    excerpt: "Proving achievable energy density limits for solid-state batteries. From materials to manufacturing with guaranteed performance.",
    sections: {
      problem: "Solid-state batteries promise higher energy density and safety, but interface resistance and manufacturing challenges prevent commercialization.",
      approach: "Multi-scale modeling from atomic interfaces to cell design. RBT optimization finds minimal resistance pathways and stable cycling conditions.",
      requirements: "Materials characterization facilities, battery testing equipment, and partnership with battery manufacturers. Electrochemistry and materials science expertise.",
      deliverables: "Optimized interface designs, manufacturing process constraints, and performance guarantees. Prototype cells with validated models.",
      verification: "Long-term cycling tests, post-mortem analysis, and scale-up validation. Every cell includes predicted vs actual performance metrics.",
      impact: "Enables EVs with 1000km range, grid storage at $50/kWh, and safer consumer electronics."
    }
  },
  {
    slug: "grid-phasor-flow",
    title: "Grid Phasor Flow",
    tagline: "routing receipts",
    emoji: "🌐",
    domain: "infra",
    status: "open",
    complexity: 3,
    timeline: "6-9 months",
    excerpt: "Real-time optimization of power grid flows with renewable integration. Provable stability with maximum renewable penetration.",
    sections: {
      problem: "Power grids become unstable with high renewable penetration. Current models can't guarantee stability with more than 30% variable generation.",
      approach: "Phasor-based flow analysis with RBT stability proofs. We find operating regions that maintain stability with 80%+ renewables.",
      requirements: "Grid operational data, power flow simulation tools, and collaboration with grid operators. Power systems engineering expertise.",
      deliverables: "Real-time stability assessment tool, optimal dispatch algorithms, and renewable integration limits. Validated on actual grid data.",
      verification: "Back-testing on historical grid events, simulation of extreme scenarios, and pilot deployment with grid operator.",
      impact: "Enables deep decarbonization of electricity, prevents blackouts, and reduces grid operation costs by 20%."
    }
  },
  {
    slug: "robotics-control",
    title: "Robotics Control",
    tagline: "Z₄ rhythm",
    emoji: "🤖",
    domain: "robotics",
    status: "in-progress",
    complexity: 4,
    timeline: "8-14 months",
    coverImage: "/moonshots/robotics-control/cover.jpg",
    excerpt: "Discovering natural movement patterns for legged robots. From balance to ballet through topological control.",
    sections: {
      problem: "Legged robots require complex control systems that fail in unexpected situations. Current methods are brittle and energy-intensive.",
      approach: "Topological analysis of movement space using Z₄ symmetries. We find natural gaits and transitions that require minimal control input.",
      requirements: "Robotic hardware platforms, motion capture systems, and simulation environments. Expertise in robotics, topology, and control theory.",
      deliverables: "Gait library with transition graphs, energy-optimal controllers, and robustness guarantees. Implementation on quadruped and biped platforms.",
      verification: "Testing on varied terrain, energy efficiency analysis, and comparison with state-of-the-art controllers.",
      impact: "Enables robots to navigate human environments, reduces battery requirements by 3x, and provides foundation for robotic agility."
    }
  },
  {
    slug: "climate-mrv",
    title: "Climate MRV",
    tagline: "measure what matters",
    emoji: "🌿",
    domain: "infra",
    status: "open",
    complexity: 3,
    timeline: "6-12 months",
    excerpt: "Verification system for carbon removal and climate interventions. From measurement to market with mathematical proof.",
    sections: {
      problem: "Carbon markets lack credible measurement, reporting, and verification (MRV). This undermines climate action and enables greenwashing.",
      approach: "Sensor fusion with RBT verification chains. We create tamper-proof measurement systems with uncertainty quantification.",
      requirements: "Environmental sensor networks, satellite data access, and blockchain infrastructure. Climate science and data assurance expertise.",
      deliverables: "MRV protocol with mathematical guarantees, sensor specifications, and smart contract templates. Pilot deployment with carbon removal project.",
      verification: "Cross-validation with independent measurements, uncertainty propagation analysis, and audit trail verification.",
      impact: "Unlocks $100B+ carbon removal market, ensures climate investments deliver results, and builds trust in climate action."
    }
  },
  {
    slug: "port-logistics",
    title: "Port Logistics",
    tagline: "throughput lift",
    emoji: "🚢",
    domain: "infra",
    status: "partners",
    complexity: 2,
    timeline: "3-6 months",
    excerpt: "Optimizing container port operations for 2x throughput. From ship to truck with provable efficiency gains.",
    sections: {
      problem: "Ports are bottlenecks in global supply chains. Congestion costs billions annually and current optimization misses systemic constraints.",
      approach: "Full-stack optimization from berth allocation to truck routing. RBT finds globally optimal solutions considering all constraints.",
      requirements: "Port operational data, simulation environment, and partnership with port operator. Operations research and logistics expertise.",
      deliverables: "Scheduling algorithms, yard layout optimization, and performance guarantees. Real-time dashboard with decision support.",
      verification: "Simulation validation, pilot testing in port section, and KPI tracking over 6 months.",
      impact: "Doubles port capacity without infrastructure investment, reduces shipping costs, and cuts emissions from idling vessels."
    }
  },
  {
    slug: "deep-space-traj",
    title: "Deep Space Trajectory",
    tagline: "Δv budgets",
    emoji: "🛰️",
    domain: "physics",
    status: "open",
    complexity: 3,
    timeline: "6-9 months",
    excerpt: "Optimal trajectories for deep space missions with guaranteed fuel margins. From Earth to anywhere with minimal propellant.",
    sections: {
      problem: "Deep space missions require precise trajectory planning with fuel margins. Current methods are conservative and miss optimal transfer opportunities.",
      approach: "Multi-body dynamics with RBT optimization. We find ballistic captures and weak stability boundaries that minimize Δv requirements.",
      requirements: "Trajectory simulation tools, ephemeris data, and collaboration with space agencies. Astrodynamics and numerical optimization expertise.",
      deliverables: "Trajectory optimization software, fuel-optimal transfer catalogs, and contingency planning tools. Validated against historical missions.",
      verification: "Comparison with flown missions, Monte Carlo analysis with navigation errors, and independent verification by mission planners.",
      impact: "Enables ambitious deep space missions, reduces launch mass by 30%, and opens new destinations for exploration."
    }
  },
  {
    slug: "safety-refuters",
    title: "Safety Refuters",
    tagline: "mint/commit changes",
    emoji: "🛡️",
    domain: "infra",
    status: "in-progress",
    complexity: 4,
    timeline: "9-15 months",
    excerpt: "Formal verification for critical system changes. Prove safety or find the breaking case before deployment.",
    sections: {
      problem: "Critical systems (power, transport, finance) can't be safely updated. Testing misses edge cases that cause cascading failures.",
      approach: "RBT refutation engine that either proves safety or constructs counter-examples. We verify changes against formal specifications.",
      requirements: "System specifications, test environments, and partnership with critical infrastructure operators. Formal methods and safety engineering expertise.",
      deliverables: "Verification framework, safety proof certificates, and counter-example generator. Integration with CI/CD pipelines.",
      verification: "Fault injection testing, comparison with traditional testing, and validation on historical incidents.",
      impact: "Prevents critical infrastructure failures, enables safe modernization of legacy systems, and reduces testing costs by 90%."
    }
  },
  {
    slug: "phase-maps-ckl",
    title: "Phase Maps (CKL)",
    tagline: "holonomy collapse",
    emoji: "🧊",
    domain: "physics",
    status: "open",
    complexity: 5,
    timeline: "12-18 months",
    excerpt: "Mapping phase transitions through topological methods. From quantum to classical with geometric understanding.",
    sections: {
      problem: "Phase transitions in complex systems are poorly understood. We can't predict critical points or design materials with desired phases.",
      approach: "Cohomological analysis of phase space with Chern-Klein-Langlands correspondence. We map phase boundaries through topological invariants.",
      requirements: "Computational physics resources, materials characterization, and mathematical physics expertise. Access to quantum simulators.",
      deliverables: "Phase prediction algorithms, topological phase catalogs, and design principles for novel materials. Software for phase analysis.",
      verification: "Experimental validation of predicted phases, comparison with known transitions, and blind predictions on new materials.",
      impact: "Enables design of topological quantum computers, room-temperature superconductors, and materials with exotic properties."
    }
  },
  {
    slug: "causal-route-finding",
    title: "Causal Route-Finding",
    tagline: "policy proofs",
    emoji: "🧠",
    domain: "infra",
    status: "partners",
    complexity: 4,
    timeline: "9-12 months",
    excerpt: "Finding causal paths in complex systems for policy design. From intervention to outcome with guaranteed effects.",
    sections: {
      problem: "Policy interventions often fail or have unintended consequences. We can't trace causal paths through complex social and economic systems.",
      approach: "Causal graph analysis with RBT path-finding. We identify minimal intervention sets and prove policy outcomes.",
      requirements: "Large-scale data sets, causal inference expertise, and partnership with policy makers. Social science and economics knowledge.",
      deliverables: "Causal discovery algorithms, policy simulation tools, and outcome guarantees. Case studies on real policy decisions.",
      verification: "Natural experiments, randomized trials where possible, and historical policy analysis.",
      impact: "Evidence-based policy making, reduced unintended consequences, and faster iteration on social programs."
    }
  }
]

// Helper functions
export function getMoonshotBySlug(slug: string): Moonshot | undefined {
  return moonshots.find(m => m.slug === slug)
}

export function getMoonshotsByDomain(domain: MoonshotDomain): Moonshot[] {
  return moonshots.filter(m => m.domain === domain)
}

export function getMoonshotsByStatus(status: MoonshotStatus): Moonshot[] {
  return moonshots.filter(m => m.status === status)
}