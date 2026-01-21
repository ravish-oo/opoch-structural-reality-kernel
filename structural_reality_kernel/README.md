# Structural Reality Kernel

A mathematical framework for solving NP-hard problems through **quotient collapse** - the principle that reality is defined by the equivalence classes of indistinguishability.

## Core Insight

> **Axiom A0:** If two states are indistinguishable under ALL available tests, they MUST be identified.

This single axiom, rigorously applied, collapses exponential search spaces into tractable computations.

## Demonstrated Results

| Problem | Naive Space | Explored | Compression | Verification |
|---------|-------------|----------|-------------|--------------|
| MAPF 12×12, 8 agents | 184 quadrillion | 2,516 | 10^13.9 | V1-V5 PASS |

**TrustGain: ∞** (zero silent failures via mathematical verification)

## Quick Start

### Run MAPF Demo

```bash
# From repository root
python -m structural_reality_kernel

# Or run specific demo
python -m structural_reality_kernel.demos.mapf
```

### Visualize Solution

```bash
# Standalone simulation (no ROS2 required)
cd empirical_evidences/gazebo_simulation
python scripts/standalone_simulation.py --verify --stats

# Generate animation
python scripts/standalone_simulation.py --save simulation.gif
```

### Run Benchmarks

```bash
cd empirical_evidences
./run_benchmarks.sh --quick
```

## Repository Structure

```
structural_reality_kernel/
├── README.md                    # This file
├── __init__.py                  # Package initialization
├── __main__.py                  # Entry point for python -m
│
├── core/                        # Foundational kernel objects
│   ├── kernel.py               # Ledger, Survivors, Π*, Budget, Time
│   ├── controller.py           # Π-consistent test selection
│   ├── gauge.py                # Canonical representation
│   ├── receipts.py             # Cryptographic audit trail
│   ├── theorem_generator.py    # Contract compilation
│   ├── universe_engine.py      # ⊥ → ⊥op evolution
│   └── verify.py               # Verification framework
│
├── empirical_evidences/         # Domain implementations
│   ├── mapf_*.py               # Multi-Agent Path Finding
│   ├── scale_quotient.py       # Scale-based quotient construction
│   ├── np_hardness.py          # NP as separator theorems
│   ├── equivalence.py          # Test indistinguishability
│   ├── quantum.py              # Quantum mechanics derivation
│   ├── consciousness.py        # Consciousness as quotient
│   └── gazebo_simulation/      # ROS2/Gazebo MAPF simulation
│       ├── config/             # Robot and solution configs
│       ├── scripts/            # Execution scripts
│       ├── launch/             # ROS2 launch files
│       └── worlds/             # Gazebo world files
│
├── demos/                       # Runnable demonstrations
│   ├── mapf.py                 # MAPF demo
│   ├── np_sat.py               # SAT solving demo
│   └── quantum_gns.py          # Quantum GNS demo
│
└── docs/                        # Documentation
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
Budget(L) ≈ log₂|W(L)|
```

**A2: Totality Discipline**
```
Every test τ: D₀ → A is total (PASS, FAIL, or TIMEOUT - never undefined)
```

### Quotient Collapse

The scale quotient Q_s(W) = W / ~_s represents the observable world at resolution s:

- **Fine scale:** Many equivalence classes (high resolution)
- **Coarse scale:** Fewer classes (information compressed)
- **Maximum coarse:** Single class (complete collapse)

### Semigroup Property

```
R_s2 ∘ R_s1 = R_s2  (for s2 coarser than s1)
```

Coarse-graining is **irreversible** - the computational analog of the Second Law of Thermodynamics.

## MAPF Implementation

### How CBS Implements Quotient Collapse

```python
# CBS = Kernel Refinement
while open_set:
    node = pop_best(open_set)

    # Verify (source of truth)
    result = verify_paths(node.paths)

    if result.passed:
        return UNIQUE(node.paths)  # ⊥op reached

    # Branch on τ* (minimal separator)
    conflict = result.conflict
    for agent in conflict.agents:
        constraint = forbid(agent, conflict)  # Quotient map
        child = replan(node, constraint)
        push(open_set, child)
```

### V1-V5 Truth Gate

| Gate | Check | Formula |
|------|-------|---------|
| V1 | Start | ∀i. path[i][0] = start[i] |
| V2 | Goal | ∀i. path[i][-1] = goal[i] |
| V3 | Dynamics | ∀i,t. valid_move(path[i][t], path[i][t+1]) |
| V4 | Vertex | ∀i≠j,t. path[i][t] ≠ path[j][t] |
| V5 | Edge | ∀i≠j,t. ¬swap(i, j, t) |

### Output Contract

```
UNIQUE    → Solution found + cryptographic receipt
UNSAT     → Proven impossible + certificate
OMEGA_GAP → Honest uncertainty + frontier witness
```

## Key Files

### Core Theory
- `core/kernel.py` - Fundamental objects (Ledger, Survivors, Π*, Budget)
- `empirical_evidences/scale_quotient.py` - Quotient construction
- `empirical_evidences/np_hardness.py` - NP as separator cost

### MAPF Solver
- `empirical_evidences/mapf_model.py` - MAPF data structures
- `empirical_evidences/mapf_cbs.py` - CBS solver (quotient collapse)
- `empirical_evidences/mapf_verifier.py` - V1-V5 verification
- `empirical_evidences/mapf_planviz.py` - Visualization

### Simulation
- `empirical_evidences/gazebo_simulation/` - Complete ROS2/Gazebo package

## Benchmark Results

From `empirical_evidences/MAPF_BENCHMARK_RESULTS.md`:

```
┌─────────────────────────────────────────────────────────────────────┐
│ TOTAL NAIVE SEARCH SPACE                                            │
│ 184,885,258,911,814,272 states (184 quadrillion)                   │
│                                                                     │
│ TOTAL CBS NODES EXPLORED                                            │
│ 2,580 states                                                        │
│                                                                     │
│ OVERALL COMPRESSION                                                 │
│ 10^13.9 (73 trillion times fewer)                                  │
└─────────────────────────────────────────────────────────────────────┘
```

## Requirements

```bash
# Core (no dependencies)
python >= 3.8

# Visualization (optional)
pip install matplotlib pillow

# Benchmarks (optional)
pip install numpy pyyaml

# ROS2 Simulation (optional)
# See empirical_evidences/gazebo_simulation/README.md
```

## Citation

If you use this work, please cite:

```bibtex
@software{structural_reality_kernel,
  title = {Structural Reality Kernel: Quotient Collapse for NP-Hard Problems},
  author = {Structural Reality Team},
  year = {2024},
  url = {https://github.com/ravish-oo/opoch-structural-reality-kernel}
}
```

## License

MIT License - See LICENSE file for details.
