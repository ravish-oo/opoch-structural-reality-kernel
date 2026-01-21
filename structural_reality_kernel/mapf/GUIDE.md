# MAPF (Multi-Agent Path Finding) Guide

Complete guide to the MAPF implementation in Structural Reality Kernel.

## Overview

MAPF is the problem of finding collision-free paths for multiple agents from their starts to their goals. Our implementation uses **CBS (Conflict-Based Search)** as the quotient collapse algorithm.

## Quick Start

### 1. Run the Demo

```bash
# From repository root
python -m structural_reality_kernel.mapf.examples.simple

# Full 8-agent challenge
python -m structural_reality_kernel.mapf.examples.challenge_8_agents
```

### 2. Visualize a Solution

```bash
cd structural_reality_kernel/mapf/simulation

# Verify solution and show stats
python scripts/standalone_simulation.py --verify --stats

# Run animation
python scripts/standalone_simulation.py

# Save as GIF
python scripts/standalone_simulation.py --save solution.gif
```

### 3. Run Benchmarks

```bash
cd structural_reality_kernel/mapf/benchmarks
./run_benchmarks.sh --quick
```

## Architecture

### Core Files

```
mapf/
├── model.py               # Data structures (Graph, Instance, Path, Conflict)
├── cbs.py                 # CBS solver (quotient collapse algorithm)
├── verifier.py            # V1-V5 verification
├── planviz.py             # Visualization and video generation
├── ilp.py                 # ILP cross-validation
├── verify.py              # Complete verification suite
├── proof_bundle.py        # Cryptographic proof bundles
├── benchmarks/            # Benchmark framework
├── adapters/              # ROS2, Unity, Isaac adapters
├── simulation/            # Gazebo simulation
├── visualizations/        # Sample visualizations
└── examples/              # Runnable demos
```

### Data Flow

```
Instance → CBS Solver → Paths → Verifier → Result
              ↓
         [Conflicts]
              ↓
         [Branching]
              ↓
         [Quotient Collapse]
```

## API Reference

### Creating an Instance

```python
from structural_reality_kernel.mapf.model import MAPFInstance, create_grid_graph

# Create a 12x12 grid
graph = create_grid_graph(12, 12)

# Define agents (vertex indices)
starts = [0, 143, 132, 11]      # Corners
goals = [143, 0, 11, 132]       # Opposite corners

instance = MAPFInstance(
    graph=graph,
    starts=starts,
    goals=goals
)
```

### Solving

```python
from structural_reality_kernel.mapf.cbs import CBSSolver

solver = CBSSolver(
    instance=instance,
    max_time=100,      # Max timesteps per path
    max_nodes=10000    # Max CBS nodes to expand
)

result = solver.solve()

if result.is_unique():
    print(f"Solution found! Cost: {result.cost}")
    print(f"Paths: {result.paths}")
elif result.is_unsat():
    print(f"No solution exists: {result.certificate}")
else:
    print(f"Budget exhausted: {result.gap}")
```

### Verification

```python
from structural_reality_kernel.mapf.verifier import verify_paths

# Verify independently
T = max(len(p) - 1 for p in result.paths)
verification = verify_paths(instance, result.paths, T)

if verification.passed:
    print("V1-V5 ALL PASS")
else:
    print(f"Failed: {verification.check}")
    print(f"Conflict: {verification.conflict}")
```

### Visualization

```python
from structural_reality_kernel.mapf.planviz import (
    MAPFToPlanViz,
    PlanVizVideoRenderer,
    quick_visualize
)

# Quick visualization
quick_visualize(result.paths, instance.graph, "output.json")

# Convert to PlanViz format
converter = MAPFToPlanViz(instance, result.paths)
planviz = converter.to_planviz()

# Generate video
renderer = PlanVizVideoRenderer()
renderer.render_to_gif(planviz, "solution.gif")
```

## V1-V5 Verification

Every solution passes through 5 mandatory checks:

| Check | Name | Description |
|-------|------|-------------|
| V1 | Start | All agents at correct initial positions |
| V2 | Goal | All agents reach their destinations |
| V3 | Dynamics | All moves are valid (edge or wait) |
| V4 | Vertex | No two agents at same cell at same time |
| V5 | Edge | No head-on collisions (swap conflicts) |

```python
# Formal definitions
V1 = lambda paths, starts: all(p[0] == s for p, s in zip(paths, starts))
V2 = lambda paths, goals: all(p[-1] == g for p, g in zip(paths, goals))
V3 = lambda paths, graph: all(valid_move(p[t], p[t+1], graph) for p in paths for t in range(len(p)-1))
V4 = lambda paths: all(paths[i][t] != paths[j][t] for i,j in pairs for t in timesteps)
V5 = lambda paths: no_swaps(paths)
```

## Output Contract

The solver returns exactly one of three results:

### UNIQUE
Solution found and verified.
```python
result.status == ResultStatus.UNIQUE
result.paths  # List of paths
result.cost   # Sum of costs
result.receipt  # SHA-256 hash
```

### UNSAT
Proven impossible.
```python
result.status == ResultStatus.UNSAT
result.certificate  # Why impossible (e.g., goal collision)
```

### OMEGA_GAP
Budget exhausted, undecided.
```python
result.status == ResultStatus.OMEGA_GAP
result.gap      # What limit hit
result.frontier # Last known state
```

## Simulation

### Prerequisites

```bash
# ROS2 Humble (optional, for full simulation)
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3*

# Python
pip install numpy pyyaml matplotlib pillow
```

### Running

```bash
cd structural_reality_kernel/mapf/simulation

# Option 1: Standalone (no ROS2)
python scripts/standalone_simulation.py

# Option 2: Full ROS2/Gazebo
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

# Terminal 1: Launch simulation
ros2 launch mapf_simulation multi_robot_gazebo.launch.py

# Terminal 2: Execute paths
ros2 run mapf_simulation execute_mapf_solution.py

# Terminal 3: Monitor safety
ros2 run mapf_simulation safety_monitor.py
```

### Configuration Files

```
simulation/
├── config/
│   ├── robots.yaml         # Robot positions and goals
│   └── mapf_solution.json  # Pre-computed CBS solution
├── scripts/
│   ├── standalone_simulation.py  # No ROS2 needed
│   ├── execute_mapf_solution.py  # ROS2 waypoint executor
│   └── safety_monitor.py         # Collision detection
├── launch/
│   └── multi_robot_gazebo.launch.py
└── worlds/
    └── mapf_grid_12x12.world
```

## Benchmark Results

### Test Instances

| Instance | Grid | Agents | Naive Space | CBS Nodes | Compression |
|----------|------|--------|-------------|-----------|-------------|
| Small | 5x5 | 2 | 625 | 2 | 10^2.5 |
| Medium | 8x8 | 4 | 16M | 23 | 10^5.9 |
| Large | 10x10 | 6 | 1T | 39 | 10^10.4 |
| Challenge | 12x12 | 8 | 185Q | 2,516 | 10^13.9 |

### Key Metrics

```
Total Naive Space:     184,885,258,911,814,272 states
Total CBS Explored:    2,580 states
Overall Compression:   71,660,953,066,595x
Log10 Compression:     10^13.9
```

## Troubleshooting

### Import Errors

```bash
# Make sure you're in the repository root
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
```

### Visualization Not Working

```bash
# Install matplotlib
pip install matplotlib pillow
```

### ROS2 Issues

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Set TurtleBot model
export TURTLEBOT3_MODEL=burger
```

## References

- [CBS Paper](https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/view/5062) - Sharon et al., 2012
- [MAPF Survey](https://arxiv.org/abs/1906.08291) - Stern et al., 2019
- [PlanViz Format](https://github.com/MAPF-Competition/PlanViz)
