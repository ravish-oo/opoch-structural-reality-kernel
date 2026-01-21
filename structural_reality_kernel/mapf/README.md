# MAPF (Multi-Agent Path Finding)

Complete implementation of MAPF solving via **quotient collapse**.

## Overview

This module implements the CBS (Conflict-Based Search) algorithm as a quotient collapse mechanism, demonstrating how NP-hard problems can be solved efficiently through the structural reality kernel framework.

### Key Results

| Metric | Value |
|--------|-------|
| Grid Size | 12 × 12 = 144 cells |
| Agents | 8 |
| Naive Search Space | 144^8 = 1.85 × 10^17 states |
| CBS Nodes Explored | ~2,500 |
| Compression | 10^13.9 (73 trillion times fewer) |
| Verification | V1-V5 ALL PASS |

## Directory Structure

```
mapf/
├── README.md              # This file
├── GUIDE.md               # Complete usage guide
├── BENCHMARK_RESULTS.md   # Benchmark metrics
│
├── model.py               # Core data structures
├── cbs.py                 # CBS solver (quotient collapse)
├── verifier.py            # V1-V5 verification
├── ilp.py                 # ILP cross-validation
├── planviz.py             # Visualization
├── proof_bundle.py        # Cryptographic proofs
├── verify.py              # Integration verification
│
├── benchmarks/            # Benchmark framework
│   ├── benchmarks.py      # Test suite
│   ├── runner.py          # Benchmark runner
│   ├── external.py        # External benchmarks
│   ├── movingai.py        # MovingAI format
│   ├── lorr.py            # LoRR format
│   ├── lorr_submission.py # Competition submission
│   └── run_benchmarks.sh  # Shell runner
│
├── adapters/              # Platform integrations
│   ├── ros2.py            # ROS2/Gazebo
│   ├── unity.py           # Unity engine
│   ├── isaac.py           # NVIDIA Isaac Sim
│   └── industry_pitch.py  # Industry demos
│
├── simulation/            # Gazebo simulation
│   ├── README.md
│   ├── config/            # Robot configs
│   ├── scripts/           # Execution scripts
│   ├── launch/            # ROS2 launch files
│   └── worlds/            # Gazebo worlds
│
├── visualizations/        # Sample visualizations
│   ├── viewer.html        # Interactive viewer
│   └── *.json             # Sample solutions
│
└── examples/              # Runnable demos
    ├── simple.py          # 4-agent demo
    └── challenge_8_agents.py  # 8-agent challenge
```

## Quick Start

### Run Examples

```bash
# From repository root
cd structural_reality_kernel

# Simple 4-agent demo
python -m mapf.examples.simple

# Full 8-agent challenge
python -m mapf.examples.challenge_8_agents
```

### Use as Library

```python
from structural_reality_kernel.mapf import (
    MAPFInstance,
    CBSSolver,
    verify_paths,
    create_grid_graph
)

# Create instance
graph = create_grid_graph(12, 12)
instance = MAPFInstance(graph, starts=[0, 143], goals=[143, 0])

# Solve
solver = CBSSolver(instance, max_time=100, max_nodes=10000)
result = solver.solve()

# Verify
if result.is_unique():
    verification = verify_paths(instance, result.paths, result.makespan())
    assert verification.passed
```

### Run Simulation

```bash
cd mapf/simulation

# Standalone (no ROS2)
python scripts/standalone_simulation.py --verify --stats

# Generate animation
python scripts/standalone_simulation.py --save solution.gif
```

## V1-V5 Verification

Every solution passes 5 mandatory checks:

| Check | Description |
|-------|-------------|
| V1 | All agents at correct start positions |
| V2 | All agents reach their goals |
| V3 | All moves are valid (edge or wait) |
| V4 | No vertex conflicts (same cell, same time) |
| V5 | No edge conflicts (head-on collisions) |

## Output Contract

The solver returns exactly one of:

- **UNIQUE**: Solution found with cryptographic receipt
- **UNSAT**: Proven impossible with certificate
- **OMEGA_GAP**: Budget exhausted, honest uncertainty

## Files

### Core

- `model.py` - Graph, Instance, Path, Conflict data structures
- `cbs.py` - CBS solver implementing quotient collapse
- `verifier.py` - V1-V5 verification (source of truth)

### Visualization

- `planviz.py` - PlanViz format conversion, video generation
- `visualizations/viewer.html` - Interactive web viewer

### Benchmarks

- `benchmarks/benchmarks.py` - Standard test suite
- `benchmarks/movingai.py` - MovingAI benchmark support
- `BENCHMARK_RESULTS.md` - Detailed results

### Simulation

- `simulation/scripts/standalone_simulation.py` - No ROS2 needed
- `simulation/scripts/execute_mapf_solution.py` - ROS2 executor
- `simulation/scripts/safety_monitor.py` - Collision monitoring

## References

- [CBS Paper](https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/view/5062) - Sharon et al., 2012
- [MAPF Survey](https://arxiv.org/abs/1906.08291) - Stern et al., 2019
