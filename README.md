# Opoch - Structural Reality Kernel

A mathematical framework for solving NP-hard problems through **quotient collapse** - the principle that reality is defined by the equivalence classes of indistinguishability.

## MAPF Module

**The MAPF (Multi-Agent Path Finding) module has been moved to its own repository:**

**https://github.com/chetannothingness/MAPF**

Features:
- 10,000-robot Transport Fabric at 31.5 Hz
- 400-robot Production Scale demos
- CBS solver with V1-V5 verification
- Mathematical collision impossibility guarantee

---

## Repository Structure

```
opoch-structural-reality-kernel/
├── kernel/                      # Structural Reality Kernel (Python)
│   ├── core/                   # Foundational kernel primitives
│   ├── empirical_evidences/    # Domain implementations
│   │   ├── physics/           # Quantum, space, time, energy
│   │   ├── logic/             # Computation, complexity, Godel
│   │   ├── mind/              # Consciousness, intelligence
│   │   ├── applications/      # Biotech, climate, finance
│   │   └── verification/      # All verification tests
│   └── demos/                  # Core kernel demos
│
├── website/                     # Opoch Website (React/TypeScript)
│   ├── src/                    # React components
│   ├── public/                 # Static assets
│   └── package.json            # Dependencies
│
├── api/                         # Backend API (Vercel serverless)
│
├── demos/                       # Standalone Python demos
│   ├── apple_puzzles_kernel_demo.py
│   ├── consciousness_demo.py
│   └── ...
│
├── docs/                        # All documentation
│   ├── theory_of_everything.pdf
│   ├── brand/                  # Branding assets
│   └── website/                # Website documentation
│
├── tools/                       # Development tools
│   ├── mcp/                    # MCP integrations
│   ├── scripts/                # Utility scripts
│   └── supabase/               # Database schemas
│
└── archive/                     # Archived code
```

## Core Insight

> **Axiom A0:** If two states are indistinguishable under ALL available tests, they MUST be identified.

This single axiom, rigorously applied, collapses exponential search spaces into tractable computations.

## Mathematical Foundation

### The Three Axioms

**A0: Indistinguishability → Identification**
```
x ~_Δ y ⟺ ∀τ ∈ Δ, τ(x) = τ(y)  →  x ≡ y in Q_Δ
```

**A1: Budget-Feasibility**
```
Feasible tests: Δ(L) = {τ : cost(τ) ≤ Budget(L)}
```

**A2: Totality Discipline**
```
Every test τ: D₀ → A is total (PASS, FAIL, or TIMEOUT)
```

### Output Contract

```
UNIQUE    → Solution found + cryptographic receipt
UNSAT     → Proven impossible + certificate
OMEGA_GAP → Honest uncertainty + frontier witness
```

**TrustGain: ∞** (zero silent failures via mathematical verification)

## Quick Start

### Run Website

```bash
cd website
npm install
npm run dev
```

## Requirements

### Kernel (Python)
```bash
python >= 3.8
```

### Website (Node.js)
```bash
node >= 20.9
npm >= 10
```

## Related Repositories

- **[MAPF](https://github.com/chetannothingness/MAPF)** - Multi-Agent Path Finding (10K robots)

## Documentation

- [Mathematical Theory](kernel/docs/THEORY.md) - Complete axiom system

## License

MIT License - See [kernel/LICENSE](kernel/LICENSE)

## Security

See [SECURITY.md](SECURITY.md) for security policies.
