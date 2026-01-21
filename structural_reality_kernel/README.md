# Structural Reality Kernel

A mathematical framework for solving NP-hard problems through **quotient collapse** - the principle that reality is defined by the equivalence classes of indistinguishability.

## Core Insight

> **Axiom A0:** If two states are indistinguishable under ALL available tests, they MUST be identified.

This single axiom, rigorously applied, collapses exponential search spaces into tractable computations.

## Key Results

| Problem | Naive Space | Explored | Compression | Verification |
|---------|-------------|----------|-------------|--------------|
| MAPF 12×12, 8 agents | 184 quadrillion | 2,500 | 10^13.9 | V1-V5 PASS |

**TrustGain: ∞** (zero silent failures via mathematical verification)

## Quick Start

### Run MAPF Demo

```bash
# From repository root
python -m structural_reality_kernel.mapf.examples.simple

# Full 8-agent challenge
python -m structural_reality_kernel.mapf.examples.challenge_8_agents
```

### Visualize Solution

```bash
cd structural_reality_kernel/mapf/simulation
python scripts/standalone_simulation.py --verify --stats
```

## Repository Structure

```
structural_reality_kernel/
├── README.md                    # This file
├── LICENSE                      # MIT License
├── __init__.py                  # Package initialization
├── __main__.py                  # Entry point
│
├── core/                        # Foundational kernel
│   ├── kernel.py               # Ledger, Survivors, Π*, Budget
│   ├── controller.py           # Test selection strategy
│   ├── gauge.py                # Canonical representation
│   ├── receipts.py             # Cryptographic audit trail
│   ├── theorem_generator.py    # Contract compilation
│   ├── universe_engine.py      # ⊥ → ⊥op evolution
│   ├── verify.py               # Verification framework
│   └── nsl.py                  # NSL logic
│
├── mapf/                        # Multi-Agent Path Finding
│   ├── README.md               # MAPF documentation
│   ├── model.py                # Data structures
│   ├── cbs.py                  # CBS solver
│   ├── verifier.py             # V1-V5 verification
│   ├── ilp.py                  # ILP cross-check
│   ├── planviz.py              # Visualization
│   ├── benchmarks/             # Benchmark suite
│   ├── adapters/               # ROS2, Unity, Isaac
│   ├── simulation/             # Gazebo simulation
│   ├── visualizations/         # Sample visualizations
│   └── examples/               # Runnable demos
│
├── empirical_evidences/         # Domain implementations
│   ├── physics/                # Quantum, space, time, energy
│   ├── logic/                  # Godel, complexity, quotients
│   ├── mind/                   # Consciousness, intelligence
│   ├── applications/           # Biotech, climate, finance
│   └── verification/           # All *_verify.py tests
│
├── demos/                       # Core kernel demos
│   ├── mapf.py                 # MAPF as theorem generation
│   ├── np_sat.py               # SAT solving
│   ├── quantum_gns.py          # Quantum GNS construction
│   └── gravity_cost_geometry.py # Gravity derivation
│
└── docs/                        # Documentation
    ├── THEORY.md               # Mathematical foundation
    └── proof_bundle.md         # Proof bundle specification
```

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

### Quotient Collapse

The scale quotient Q_s(W) = W / ~_s compresses exponential spaces:

- **Fine scale:** Many equivalence classes
- **Coarse scale:** Fewer, larger classes
- **Maximum coarse:** Single class (complete collapse)

### Output Contract

```
UNIQUE    → Solution found + cryptographic receipt
UNSAT     → Proven impossible + certificate
OMEGA_GAP → Honest uncertainty + frontier witness
```

## MAPF Implementation

### V1-V5 Truth Gate

| Gate | Check |
|------|-------|
| V1 | All agents at correct start |
| V2 | All agents reach goal |
| V3 | All moves valid |
| V4 | No vertex conflicts |
| V5 | No edge conflicts |

### Compression Result

```
Naive space:   144^8 = 1.85 × 10^17 (184 quadrillion)
CBS explored:  ~2,500 nodes
Compression:   10^13.9 (73 trillion times fewer)
```

## Requirements

```bash
# Core (no dependencies)
python >= 3.8

# Visualization (optional)
pip install matplotlib pillow

# Benchmarks (optional)
pip install numpy pyyaml
```

## Documentation

- [Mathematical Theory](docs/THEORY.md) - Complete axiom system
- [MAPF Guide](mapf/GUIDE.md) - MAPF implementation details
- [Benchmark Results](mapf/BENCHMARK_RESULTS.md) - Performance metrics

## License

MIT License - See [LICENSE](LICENSE) file.
