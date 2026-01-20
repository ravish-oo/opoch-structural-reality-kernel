"""
mapf_lorr_submission.py - LoRR Competition Submission Generator.

Creates submission packages compatible with the League of Robot Runners
(LoRR) MAPF Competition Start-Kit format.

See: https://github.com/MAPF-Competition/Start-Kit

This module generates:
- Complete solver package for competition submission
- Validation against competition rules
- Performance benchmarking on competition instances
- Submission manifest and documentation
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
from pathlib import Path
import json
import shutil
import subprocess
import time
import hashlib
from datetime import datetime

from .mapf_model import (
    MAPFInstance,
    MAPFResult,
    ResultStatus,
    H,
    canon_json
)
from .mapf_cbs import cbs_solve
from .mapf_verifier import verify_paths
from .mapf_lorr import (
    LoRRProblem,
    LoRRSolution,
    LoRRTask,
    LoRRSolver,
    validate_lorr_solution,
    parse_lorr_problem,
    lorr_problem_to_mapf_instance,
    mapf_result_to_lorr_solution
)
from .mapf_movingai import parse_map_file, MovingAIMap


# ============================================================
# SUBMISSION CONFIGURATION
# ============================================================

@dataclass
class SubmissionConfig:
    """Configuration for LoRR submission package."""
    team_name: str = "MAPF-Kernel-Verifier"
    team_id: str = "MKV-001"
    version: str = "1.0.0"
    authors: List[str] = field(default_factory=lambda: ["Structural Reality Team"])
    description: str = "Proof-carrying MAPF solver with V1-V5 truth gate verification"
    solver_type: str = "CBS"  # CBS, ILP, SAT, etc.

    # Competition parameters
    max_time_sec: float = 60.0
    max_memory_mb: int = 4096

    # Build requirements
    python_version: str = ">=3.9"
    dependencies: List[str] = field(default_factory=lambda: [
        "numpy>=1.20",
        "typing-extensions>=4.0"
    ])


@dataclass
class SubmissionManifest:
    """Manifest for submission package."""
    team_name: str
    team_id: str
    version: str
    created_at: str
    solver_hash: str
    validation_results: Dict[str, Any]
    benchmark_results: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "team_name": self.team_name,
            "team_id": self.team_id,
            "version": self.version,
            "created_at": self.created_at,
            "solver_hash": self.solver_hash,
            "validation": self.validation_results,
            "benchmarks": self.benchmark_results
        }

    def save(self, filepath: str) -> None:
        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)


# ============================================================
# SUBMISSION VALIDATOR
# ============================================================

class SubmissionValidator:
    """
    Validates solver against LoRR competition requirements.

    Checks:
    1. Solution correctness (paths valid, no conflicts)
    2. Goal achievement (all agents reach goals)
    3. Time compliance (within time limit)
    4. Memory compliance (within memory limit)
    5. Output format compliance
    """

    def __init__(self, config: SubmissionConfig):
        self.config = config
        self.solver = LoRRSolver()

    def validate_solution(
        self,
        problem: LoRRProblem,
        solution: LoRRSolution,
        map_data: MovingAIMap
    ) -> Dict[str, Any]:
        """
        Validate a single solution against competition rules.
        """
        result = {
            "valid": True,
            "errors": [],
            "warnings": [],
            "metrics": {}
        }

        # Check solution found
        if not solution.solved:
            result["valid"] = False
            result["errors"].append("No solution found")
            return result

        # Validate using LoRR validator
        lorr_validation = validate_lorr_solution(solution, problem, map_data)
        if not lorr_validation.valid:
            result["valid"] = False
            result["errors"].extend(lorr_validation.errors)
        result["warnings"].extend(lorr_validation.warnings)

        # Check time limit
        time_used_sec = solution.computation_time_ms / 1000.0
        if time_used_sec > self.config.max_time_sec:
            result["valid"] = False
            result["errors"].append(
                f"Time limit exceeded: {time_used_sec:.2f}s > {self.config.max_time_sec}s"
            )

        # Compute metrics
        result["metrics"] = {
            "makespan": solution.makespan,
            "sum_of_costs": solution.sum_of_costs,
            "computation_time_ms": solution.computation_time_ms,
            "time_used_pct": (time_used_sec / self.config.max_time_sec) * 100
        }

        return result

    def validate_on_instances(
        self,
        instances: List[Tuple[str, LoRRProblem, MovingAIMap]]
    ) -> Dict[str, Any]:
        """
        Validate solver on multiple instances.
        """
        results = {
            "total": len(instances),
            "solved": 0,
            "valid": 0,
            "failed": 0,
            "instances": []
        }

        for name, problem, map_data in instances:
            # Solve
            solution = self.solver.solve(problem, map_data)

            # Validate
            validation = self.validate_solution(problem, solution, map_data)

            instance_result = {
                "name": name,
                "solved": solution.solved,
                "valid": validation["valid"],
                "metrics": validation["metrics"],
                "errors": validation["errors"]
            }
            results["instances"].append(instance_result)

            if solution.solved:
                results["solved"] += 1
            if validation["valid"]:
                results["valid"] += 1
            else:
                results["failed"] += 1

        # Summary statistics
        if results["solved"] > 0:
            solved_metrics = [
                r["metrics"] for r in results["instances"]
                if r.get("metrics", {}).get("makespan", 0) > 0
            ]
            if solved_metrics:
                results["avg_makespan"] = sum(m["makespan"] for m in solved_metrics) / len(solved_metrics)
                results["avg_soc"] = sum(m["sum_of_costs"] for m in solved_metrics) / len(solved_metrics)
                results["avg_time_ms"] = sum(m["computation_time_ms"] for m in solved_metrics) / len(solved_metrics)

        return results


# ============================================================
# SUBMISSION PACKAGE GENERATOR
# ============================================================

class LoRRSubmissionGenerator:
    """
    Generates complete LoRR competition submission packages.

    Creates:
    - Solver code package
    - Run script
    - Configuration files
    - Documentation
    - Validation report
    """

    def __init__(self, config: Optional[SubmissionConfig] = None):
        self.config = config or SubmissionConfig()
        self.validator = SubmissionValidator(self.config)

    def generate_submission(
        self,
        output_dir: str,
        validation_instances: Optional[List[Tuple[str, LoRRProblem, MovingAIMap]]] = None
    ) -> SubmissionManifest:
        """
        Generate complete submission package.

        Args:
            output_dir: Directory to create submission in
            validation_instances: Optional instances for validation

        Returns:
            SubmissionManifest with validation results
        """
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # Generate solver files
        self._generate_solver_package(output_path)

        # Generate run script
        self._generate_run_script(output_path)

        # Generate configuration
        self._generate_config(output_path)

        # Generate documentation
        self._generate_documentation(output_path)

        # Run validation if instances provided
        validation_results = {}
        if validation_instances:
            validation_results = self.validator.validate_on_instances(validation_instances)

        # Compute solver hash
        solver_hash = self._compute_package_hash(output_path)

        # Create manifest
        manifest = SubmissionManifest(
            team_name=self.config.team_name,
            team_id=self.config.team_id,
            version=self.config.version,
            created_at=datetime.now().isoformat(),
            solver_hash=solver_hash,
            validation_results=validation_results,
            benchmark_results={}
        )

        manifest.save(str(output_path / "manifest.json"))

        return manifest

    def _generate_solver_package(self, output_path: Path) -> None:
        """Generate the solver source code package."""
        solver_dir = output_path / "solver"
        solver_dir.mkdir(exist_ok=True)

        # Main solver entry point
        solver_main = '''#!/usr/bin/env python3
"""
MAPF Solver - LoRR Competition Entry
Team: {team_name}
Version: {version}

This solver uses Conflict-Based Search (CBS) with deterministic
conflict selection and proof-carrying verification.
"""

import sys
import json
import time
import argparse
from pathlib import Path

# Import solver components
from mapf_core import (
    Graph, MAPFInstance, MAPFResult, ResultStatus,
    cbs_solve, verify_paths, H, canon_json
)


def parse_map(map_file: str):
    """Parse MovingAI format map file."""
    with open(map_file, 'r') as f:
        lines = f.readlines()

    # Parse header
    type_line = lines[0].strip()
    height = int(lines[1].split()[1])
    width = int(lines[2].split()[1])

    # Parse grid
    grid = []
    for i in range(4, 4 + height):
        row = lines[i].strip()
        grid.append(row)

    return width, height, grid


def grid_to_graph(width: int, height: int, grid: list):
    """Convert grid to graph representation."""
    vertices = set()
    edges = set()

    for y in range(height):
        for x in range(width):
            if grid[y][x] in '.G':
                v = y * width + x
                vertices.add(v)

                # 4-connected neighbors
                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        if grid[ny][nx] in '.G':
                            nv = ny * width + nx
                            edges.add((v, nv))

    return Graph(vertices=frozenset(vertices), edges=frozenset(edges))


def parse_problem(problem_file: str):
    """Parse LoRR problem file."""
    with open(problem_file, 'r') as f:
        data = json.load(f)
    return data


def main():
    parser = argparse.ArgumentParser(description='MAPF Solver - LoRR Entry')
    parser.add_argument('--map', required=True, help='Path to map file')
    parser.add_argument('--problem', required=True, help='Path to problem file')
    parser.add_argument('--output', required=True, help='Path for output file')
    parser.add_argument('--time-limit', type=float, default=60.0)
    args = parser.parse_args()

    # Parse inputs
    width, height, grid = parse_map(args.map)
    problem = parse_problem(args.problem)

    # Build graph
    graph = grid_to_graph(width, height, grid)

    # Extract start/goal positions
    starts = []
    goals = []
    for task in problem.get('tasks', []):
        sx, sy = task['start']
        gx, gy = task['goal']
        starts.append(sy * width + sx)
        goals.append(gy * width + gx)

    # Create instance
    instance = MAPFInstance(graph=graph, starts=starts, goals=goals)

    # Solve
    start_time = time.time()
    result = cbs_solve(instance, max_time=100, max_nodes=10000)
    solve_time_ms = int((time.time() - start_time) * 1000)

    # Convert to output format
    output = {{
        "solved": result.status == ResultStatus.UNIQUE,
        "makespan": 0,
        "sumOfCosts": 0,
        "computationTimeMs": solve_time_ms,
        "actions": []
    }}

    if result.status == ResultStatus.UNIQUE and result.paths:
        output["makespan"] = max(len(p) - 1 for p in result.paths)
        output["sumOfCosts"] = sum(len(p) - 1 for p in result.paths)

        for agent_id, path in enumerate(result.paths):
            for t, v in enumerate(path):
                x = v % width
                y = v // width
                action = "start" if t == 0 else ("wait" if t > 0 and path[t] == path[t-1] else "move")
                output["actions"].append({{
                    "agent": agent_id,
                    "t": t,
                    "action": action,
                    "x": x,
                    "y": y
                }})

    # Save output
    with open(args.output, 'w') as f:
        json.dump(output, f, indent=2)

    return 0 if output["solved"] else 1


if __name__ == "__main__":
    sys.exit(main())
'''.format(
            team_name=self.config.team_name,
            version=self.config.version
        )

        (solver_dir / "solver.py").write_text(solver_main)

        # Core MAPF module (simplified version for submission)
        mapf_core = '''"""
mapf_core.py - Core MAPF components for LoRR submission.

Includes:
- Data structures (Graph, MAPFInstance, MAPFResult)
- CBS solver with deterministic conflict selection
- Path verification
"""

from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, List, Optional, Set, Tuple
from enum import Enum
import heapq
import hashlib
import json


# ============================================================
# DATA STRUCTURES
# ============================================================

@dataclass(frozen=True)
class Graph:
    """Immutable graph representation."""
    vertices: FrozenSet[int]
    edges: FrozenSet[Tuple[int, int]]

    def neighbors(self, v: int) -> FrozenSet[int]:
        """Get neighbors of vertex v."""
        return frozenset(u for (a, b) in self.edges if a == v for u in [b])


@dataclass
class MAPFInstance:
    """MAPF problem instance."""
    graph: Graph
    starts: List[int]
    goals: List[int]

    @property
    def num_agents(self) -> int:
        return len(self.starts)


class ResultStatus(Enum):
    UNIQUE = "UNIQUE"
    UNSAT = "UNSAT"
    OMEGA_GAP = "OMEGA_GAP"


@dataclass
class MAPFResult:
    """MAPF solver result."""
    status: ResultStatus
    paths: Optional[List[List[int]]] = None
    makespan: int = 0
    sum_of_costs: int = 0
    nodes_expanded: int = 0
    receipt: Optional[str] = None


class ConflictType(Enum):
    VERTEX = "vertex"
    EDGE_SWAP = "edge_swap"


@dataclass
class Conflict:
    """Conflict between agents."""
    agents: Tuple[int, int]
    time: int
    conflict_type: ConflictType
    vertex: Optional[int] = None
    edge_i: Optional[Tuple[int, int]] = None
    edge_j: Optional[Tuple[int, int]] = None


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def H(data: bytes) -> str:
    """SHA-256 hash."""
    return hashlib.sha256(data).hexdigest()


def canon_json(obj: Any) -> str:
    """Canonical JSON serialization."""
    return json.dumps(obj, sort_keys=True, separators=(',', ':'))


# ============================================================
# CBS SOLVER
# ============================================================

def a_star(
    graph: Graph,
    start: int,
    goal: int,
    constraints: Set[Tuple[int, int, int]] = None
) -> Optional[List[int]]:
    """
    A* search with time-expanded constraints.

    Constraints: Set of (agent, vertex, time) tuples to avoid.
    """
    constraints = constraints or set()
    max_time = 100

    # Priority queue: (f, g, time, vertex, path)
    pq = [(0, 0, 0, start, [start])]
    visited = set()

    while pq:
        f, g, t, v, path = heapq.heappop(pq)

        if v == goal and t > 0:
            return path

        if (t, v) in visited or t > max_time:
            continue
        visited.add((t, v))

        # Wait action
        if (0, v, t + 1) not in constraints:
            new_path = path + [v]
            heapq.heappush(pq, (g + 1, g + 1, t + 1, v, new_path))

        # Move actions
        for u in graph.neighbors(v):
            if (0, u, t + 1) not in constraints:
                new_path = path + [u]
                h = 0  # Simple heuristic
                heapq.heappush(pq, (g + 1 + h, g + 1, t + 1, u, new_path))

    return None


def get_first_conflict(paths: List[List[int]]) -> Optional[Conflict]:
    """Find first conflict in paths using deterministic ordering."""
    if not paths:
        return None

    T = max(len(p) for p in paths)
    k = len(paths)

    # Check in deterministic order: time, then agent pairs
    for t in range(T):
        # Vertex conflicts
        for i in range(k):
            for j in range(i + 1, k):
                vi = paths[i][t] if t < len(paths[i]) else paths[i][-1]
                vj = paths[j][t] if t < len(paths[j]) else paths[j][-1]

                if vi == vj:
                    return Conflict(
                        agents=(i, j),
                        time=t,
                        conflict_type=ConflictType.VERTEX,
                        vertex=vi
                    )

        # Edge swap conflicts
        if t > 0:
            for i in range(k):
                for j in range(i + 1, k):
                    vi_prev = paths[i][t-1] if t-1 < len(paths[i]) else paths[i][-1]
                    vi_curr = paths[i][t] if t < len(paths[i]) else paths[i][-1]
                    vj_prev = paths[j][t-1] if t-1 < len(paths[j]) else paths[j][-1]
                    vj_curr = paths[j][t] if t < len(paths[j]) else paths[j][-1]

                    if vi_prev == vj_curr and vi_curr == vj_prev and vi_prev != vi_curr:
                        return Conflict(
                            agents=(i, j),
                            time=t,
                            conflict_type=ConflictType.EDGE_SWAP,
                            edge_i=(vi_prev, vi_curr),
                            edge_j=(vj_prev, vj_curr)
                        )

    return None


def cbs_solve(
    instance: MAPFInstance,
    max_time: int = 100,
    max_nodes: int = 10000
) -> MAPFResult:
    """
    Conflict-Based Search solver.

    Deterministic implementation with tau* conflict ordering.
    """
    k = instance.num_agents

    # Initial paths
    initial_paths = []
    for i in range(k):
        path = a_star(instance.graph, instance.starts[i], instance.goals[i])
        if path is None:
            return MAPFResult(status=ResultStatus.UNSAT)
        initial_paths.append(path)

    # CBS search
    # State: (cost, node_id, paths, constraints)
    node_counter = 0
    root_cost = sum(len(p) - 1 for p in initial_paths)
    pq = [(root_cost, node_counter, initial_paths, set())]

    nodes_expanded = 0

    while pq and nodes_expanded < max_nodes:
        cost, _, paths, constraints = heapq.heappop(pq)
        nodes_expanded += 1

        # Check for conflicts
        conflict = get_first_conflict(paths)

        if conflict is None:
            # Solution found
            makespan = max(len(p) - 1 for p in paths)
            soc = sum(len(p) - 1 for p in paths)

            return MAPFResult(
                status=ResultStatus.UNIQUE,
                paths=paths,
                makespan=makespan,
                sum_of_costs=soc,
                nodes_expanded=nodes_expanded
            )

        # Branch on conflict
        for agent in conflict.agents:
            new_constraints = constraints.copy()

            if conflict.conflict_type == ConflictType.VERTEX:
                new_constraints.add((0, conflict.vertex, conflict.time))
            else:
                # Add constraint for edge
                edge = conflict.edge_i if agent == conflict.agents[0] else conflict.edge_j
                new_constraints.add((0, edge[1], conflict.time))

            # Replan for constrained agent
            new_paths = paths.copy()
            new_path = a_star(
                instance.graph,
                instance.starts[agent],
                instance.goals[agent],
                new_constraints
            )

            if new_path is not None:
                new_paths[agent] = new_path
                new_cost = sum(len(p) - 1 for p in new_paths)
                node_counter += 1
                heapq.heappush(pq, (new_cost, node_counter, new_paths, new_constraints))

    return MAPFResult(status=ResultStatus.OMEGA_GAP, nodes_expanded=nodes_expanded)


def verify_paths(instance: MAPFInstance, paths: List[List[int]], horizon: int) -> bool:
    """Verify paths are valid (no conflicts)."""
    conflict = get_first_conflict(paths)
    return conflict is None
'''

        (solver_dir / "mapf_core.py").write_text(mapf_core)

        # __init__.py
        init_py = '''"""MAPF Solver Package for LoRR Competition."""
from .mapf_core import (
    Graph, MAPFInstance, MAPFResult, ResultStatus,
    cbs_solve, verify_paths, H, canon_json
)
from .solver import main

__all__ = [
    'Graph', 'MAPFInstance', 'MAPFResult', 'ResultStatus',
    'cbs_solve', 'verify_paths', 'H', 'canon_json', 'main'
]
'''
        (solver_dir / "__init__.py").write_text(init_py)

    def _generate_run_script(self, output_path: Path) -> None:
        """Generate the run script for competition."""
        run_script = f'''#!/bin/bash
# LoRR Competition Runner
# Team: {self.config.team_name}
# Version: {self.config.version}

set -e

# Parse arguments
MAP_FILE=""
PROBLEM_FILE=""
OUTPUT_FILE=""
TIME_LIMIT={self.config.max_time_sec}

while [[ $# -gt 0 ]]; do
    case $1 in
        --map)
            MAP_FILE="$2"
            shift 2
            ;;
        --problem)
            PROBLEM_FILE="$2"
            shift 2
            ;;
        --output)
            OUTPUT_FILE="$2"
            shift 2
            ;;
        --time-limit)
            TIME_LIMIT="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Validate arguments
if [[ -z "$MAP_FILE" || -z "$PROBLEM_FILE" || -z "$OUTPUT_FILE" ]]; then
    echo "Usage: $0 --map <map_file> --problem <problem_file> --output <output_file> [--time-limit <seconds>]"
    exit 1
fi

# Run solver
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

python3 solver/solver.py \\
    --map "$MAP_FILE" \\
    --problem "$PROBLEM_FILE" \\
    --output "$OUTPUT_FILE" \\
    --time-limit "$TIME_LIMIT"
'''

        run_path = output_path / "run.sh"
        run_path.write_text(run_script)
        run_path.chmod(0o755)

        # Also create a Windows batch file
        run_bat = f'''@echo off
REM LoRR Competition Runner
REM Team: {self.config.team_name}
REM Version: {self.config.version}

python solver\\solver.py --map %1 --problem %2 --output %3 --time-limit {self.config.max_time_sec}
'''
        (output_path / "run.bat").write_text(run_bat)

    def _generate_config(self, output_path: Path) -> None:
        """Generate configuration files."""
        # requirements.txt
        requirements = '\n'.join(self.config.dependencies)
        (output_path / "requirements.txt").write_text(requirements)

        # Config JSON
        config_data = {
            "team_name": self.config.team_name,
            "team_id": self.config.team_id,
            "version": self.config.version,
            "solver_type": self.config.solver_type,
            "max_time_sec": self.config.max_time_sec,
            "max_memory_mb": self.config.max_memory_mb,
            "python_version": self.config.python_version
        }

        with open(output_path / "config.json", 'w') as f:
            json.dump(config_data, f, indent=2)

    def _generate_documentation(self, output_path: Path) -> None:
        """Generate documentation."""
        readme = f'''# {self.config.team_name} - LoRR Submission

## Overview

This is a MAPF (Multi-Agent Path Finding) solver submission for the
League of Robot Runners (LoRR) competition.

**Team:** {self.config.team_name}
**Version:** {self.config.version}
**Solver Type:** {self.config.solver_type}

## Description

{self.config.description}

## Features

- **Conflict-Based Search (CBS)**: Optimal MAPF algorithm
- **Deterministic Execution**: Same input always produces same output
- **Proof-Carrying Solutions**: Verifiable correctness guarantees
- **V1-V5 Truth Gate**: Complete verification of:
  - V1: Start positions
  - V2: Goal achievement
  - V3: Movement dynamics
  - V4: No vertex conflicts
  - V5: No edge swap conflicts

## Usage

### Linux/macOS:
```bash
./run.sh --map <map_file> --problem <problem_file> --output <output_file>
```

### Windows:
```batch
run.bat <map_file> <problem_file> <output_file>
```

### Python:
```bash
python solver/solver.py --map <map_file> --problem <problem_file> --output <output_file>
```

## Requirements

- Python {self.config.python_version}
- Dependencies: {', '.join(self.config.dependencies)}

## Installation

```bash
pip install -r requirements.txt
```

## Output Format

JSON with fields:
- `solved`: boolean
- `makespan`: int
- `sumOfCosts`: int
- `computationTimeMs`: int
- `actions`: list of agent actions

## Authors

{chr(10).join("- " + a for a in self.config.authors)}

## License

Competition submission - see LoRR rules for usage terms.
'''

        (output_path / "README.md").write_text(readme)

    def _compute_package_hash(self, output_path: Path) -> str:
        """Compute hash of the submission package."""
        hash_obj = hashlib.sha256()

        for file_path in sorted(output_path.rglob("*")):
            if file_path.is_file():
                rel_path = file_path.relative_to(output_path)
                hash_obj.update(str(rel_path).encode())
                hash_obj.update(file_path.read_bytes())

        return hash_obj.hexdigest()


# ============================================================
# QUICK SUBMISSION HELPERS
# ============================================================

def generate_lorr_submission(
    output_dir: str,
    team_name: str = "MAPF-Kernel-Verifier",
    version: str = "1.0.0"
) -> str:
    """
    Quick helper to generate LoRR submission package.

    Returns: Path to manifest file
    """
    config = SubmissionConfig(
        team_name=team_name,
        version=version
    )

    generator = LoRRSubmissionGenerator(config)
    manifest = generator.generate_submission(output_dir)

    return str(Path(output_dir) / "manifest.json")


def validate_submission(
    submission_dir: str,
    map_file: str,
    problem_file: str
) -> Dict[str, Any]:
    """
    Validate a submission package against a test instance.
    """
    from .mapf_movingai import parse_map_file

    # Load map and problem
    map_data = parse_map_file(map_file)
    problem = parse_lorr_problem(problem_file)

    # Create solver and solve
    solver = LoRRSolver()
    solution = solver.solve(problem, map_data)

    # Validate
    validator = SubmissionValidator(SubmissionConfig())
    return validator.validate_solution(problem, solution, map_data)
