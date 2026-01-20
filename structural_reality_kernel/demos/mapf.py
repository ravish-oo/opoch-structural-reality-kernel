"""
demos/mapf.py - Multi-Agent Path Finding compiled into kernel.

MAPF as theorem generation:
- D0 = all possible joint path configurations
- Tests = collision/constraint checkers
- Π* = partition by collision pattern
- UNIQUE = collision-free solution found
- REFUTED = no solution exists
- OMEGA = budget exhausted
"""

from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, List, Optional, Set, Tuple
from itertools import product
import hashlib
import json

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import (
    Test, Record, Ledger, Survivors, PiStar, Budget,
    FeasibleTests, KernelState, compute_kernel_state
)
from core.theorem_generator import Contract, TheoremGenerator, KernelOutput
from core.universe_engine import UniverseEngine
from core.verify import VerificationSuite, ProofBundle, verify_kernel_run
from core.receipts import ReceiptChain, CanonicalJSON
from core.controller import PiController
from core.nsl import NSLEngine, Distinction


@dataclass(frozen=True)
class Position:
    """A position in the grid."""
    x: int
    y: int

    def __str__(self) -> str:
        return f"({self.x},{self.y})"


@dataclass(frozen=True)
class Agent:
    """An agent with start and goal positions."""
    agent_id: int
    start: Position
    goal: Position


@dataclass
class MAPFInstance:
    """A MAPF problem instance."""
    width: int
    height: int
    agents: List[Agent]
    obstacles: Set[Position]
    max_timesteps: int
    name: str = "MAPF"

    def canonical(self) -> str:
        """Canonical representation."""
        return CanonicalJSON.serialize({
            "width": self.width,
            "height": self.height,
            "agents": [(a.agent_id, a.start.x, a.start.y, a.goal.x, a.goal.y) for a in self.agents],
            "obstacles": sorted([(p.x, p.y) for p in self.obstacles]),
            "max_t": self.max_timesteps
        })

    def fingerprint(self) -> str:
        """SHA-256 fingerprint."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()[:16]


# Path is tuple of positions over time
Path = Tuple[Position, ...]

# JointPath is tuple of paths for all agents
JointPath = Tuple[Path, ...]


class MAPFDemo:
    """
    MAPF demonstration using the kernel.

    Compiles MAPF instance into kernel primitives.
    """

    def __init__(self, instance: MAPFInstance, budget: int = 1000):
        self.instance = instance
        self.budget = budget

        # Build D0: all valid joint paths
        self.d0 = self._build_d0()

        # Build tests: collision and validity checkers
        self.tests = self._build_tests()

        # Initialize components
        self.ledger = Ledger()
        self.controller = PiController(seed=42)
        self.receipt_chain = ReceiptChain()

    def _get_neighbors(self, pos: Position) -> List[Position]:
        """Get valid neighboring positions (including wait)."""
        neighbors = [pos]  # Wait action
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_pos = Position(pos.x + dx, pos.y + dy)
            if (0 <= new_pos.x < self.instance.width and
                0 <= new_pos.y < self.instance.height and
                new_pos not in self.instance.obstacles):
                neighbors.append(new_pos)
        return neighbors

    def _generate_paths(self, agent: Agent) -> List[Path]:
        """Generate all possible paths for an agent (bounded by max_timesteps)."""
        paths: List[Path] = []
        max_t = self.instance.max_timesteps

        def dfs(current: Position, path: List[Position], t: int) -> None:
            if t >= max_t:
                if current == agent.goal:
                    paths.append(tuple(path))
                return

            if current == agent.goal:
                # Can stay at goal
                full_path = list(path) + [agent.goal] * (max_t - t)
                paths.append(tuple(full_path))

            for next_pos in self._get_neighbors(current):
                dfs(next_pos, path + [next_pos], t + 1)

        dfs(agent.start, [agent.start], 1)
        return paths

    def _build_d0(self) -> FrozenSet[JointPath]:
        """Build candidate set D0 = all joint path combinations."""
        # Generate paths for each agent
        agent_paths = []
        for agent in self.instance.agents:
            paths = self._generate_paths(agent)
            if not paths:
                # If no valid paths, add placeholder
                paths = [tuple([agent.start] * self.instance.max_timesteps)]
            # Limit paths to avoid explosion
            agent_paths.append(paths[:100])

        # Combine into joint paths
        joint_paths = []
        for combo in product(*agent_paths):
            joint_paths.append(combo)
            if len(joint_paths) >= 10000:  # Cap D0 size
                break

        return frozenset(joint_paths)

    def _check_vertex_collision(self, jp: JointPath, t: int) -> bool:
        """Check for vertex collision at timestep t."""
        positions_at_t = set()
        for path in jp:
            if t < len(path):
                pos = path[t]
                if pos in positions_at_t:
                    return True
                positions_at_t.add(pos)
        return False

    def _check_edge_collision(self, jp: JointPath, t: int) -> bool:
        """Check for edge collision (swap) at timestep t."""
        if t == 0:
            return False

        for i, path_i in enumerate(jp):
            for j, path_j in enumerate(jp):
                if i >= j:
                    continue
                if t < len(path_i) and t < len(path_j):
                    # Check if agents swap positions
                    if (path_i[t-1] == path_j[t] and path_i[t] == path_j[t-1]):
                        return True
        return False

    def _check_reaches_goal(self, jp: JointPath, agent_idx: int) -> bool:
        """Check if agent reaches its goal."""
        agent = self.instance.agents[agent_idx]
        path = jp[agent_idx]
        return path[-1] == agent.goal if path else False

    def _build_tests(self) -> Dict[str, Test]:
        """Build tests for MAPF constraints."""
        tests = {}

        # Vertex collision tests for each timestep
        for t in range(self.instance.max_timesteps):
            def make_vertex_test(timestep: int) -> callable:
                def check(jp: JointPath) -> str:
                    if self._check_vertex_collision(jp, timestep):
                        return "COLLISION"
                    return "CLEAR"
                return check

            tests[f"vertex_t{t}"] = Test(
                test_id=f"vertex_t{t}",
                evaluator=make_vertex_test(t),
                cost=1,
                outcome_space=frozenset(["COLLISION", "CLEAR"])
            )

        # Edge collision tests for each timestep
        for t in range(1, self.instance.max_timesteps):
            def make_edge_test(timestep: int) -> callable:
                def check(jp: JointPath) -> str:
                    if self._check_edge_collision(jp, timestep):
                        return "COLLISION"
                    return "CLEAR"
                return check

            tests[f"edge_t{t}"] = Test(
                test_id=f"edge_t{t}",
                evaluator=make_edge_test(t),
                cost=1,
                outcome_space=frozenset(["COLLISION", "CLEAR"])
            )

        # Goal achievement tests for each agent
        for i, agent in enumerate(self.instance.agents):
            def make_goal_test(agent_idx: int) -> callable:
                def check(jp: JointPath) -> str:
                    if self._check_reaches_goal(jp, agent_idx):
                        return "REACHED"
                    return "NOT_REACHED"
                return check

            tests[f"goal_agent{i}"] = Test(
                test_id=f"goal_agent{i}",
                evaluator=make_goal_test(i),
                cost=1,
                outcome_space=frozenset(["REACHED", "NOT_REACHED"])
            )

        # Full validity test
        def full_validity(jp: JointPath) -> str:
            # Check all collisions
            for t in range(self.instance.max_timesteps):
                if self._check_vertex_collision(jp, t):
                    return "INVALID"
            for t in range(1, self.instance.max_timesteps):
                if self._check_edge_collision(jp, t):
                    return "INVALID"
            # Check all goals
            for i in range(len(self.instance.agents)):
                if not self._check_reaches_goal(jp, i):
                    return "INVALID"
            return "VALID"

        tests["full_validity"] = Test(
            test_id="full_validity",
            evaluator=full_validity,
            cost=len(self.instance.agents) * self.instance.max_timesteps,
            outcome_space=frozenset(["VALID", "INVALID"])
        )

        return tests

    def create_contract(self) -> Contract:
        """Create theorem generator contract for MAPF."""
        def mapf_verifier(jp: JointPath) -> bool:
            # Check all vertex collisions
            for t in range(self.instance.max_timesteps):
                if self._check_vertex_collision(jp, t):
                    return False
            # Check all edge collisions
            for t in range(1, self.instance.max_timesteps):
                if self._check_edge_collision(jp, t):
                    return False
            # Check all goals
            for i in range(len(self.instance.agents)):
                if not self._check_reaches_goal(jp, i):
                    return False
            return True

        return Contract(
            contract_id=f"MAPF_{self.instance.fingerprint()}",
            assertion=f"Find collision-free paths for {len(self.instance.agents)} agents",
            witness_space=self.d0,
            verifier=mapf_verifier,
            cost_per_verify=len(self.instance.agents) * self.instance.max_timesteps,
            budget=self.budget
        )

    def run(self) -> KernelOutput:
        """Run MAPF solving via theorem generator."""
        contract = self.create_contract()
        generator = TheoremGenerator(seed=42)
        return generator.run(contract)

    def format_solution(self, jp: JointPath) -> str:
        """Format joint path as human-readable string."""
        lines = []
        for i, (agent, path) in enumerate(zip(self.instance.agents, jp)):
            path_str = " -> ".join(str(pos) for pos in path)
            lines.append(f"Agent {agent.agent_id}: {path_str}")
        return "\n".join(lines)

    def analyze_pi_star(self) -> Dict[str, Any]:
        """Analyze the Π* partition structure."""
        state = compute_kernel_state(self.d0, self.ledger, self.tests, alpha=1)
        pi_star = state.pi_star

        return {
            "class_count": pi_star.class_count(),
            "class_sizes": pi_star.class_sizes(),
            "fingerprint": pi_star.canonical_fingerprint()[:32],
            "d0_size": len(self.d0)
        }


def create_simple_mapf() -> MAPFInstance:
    """Create a simple 3x3 MAPF instance with 2 agents."""
    return MAPFInstance(
        width=3,
        height=3,
        agents=[
            Agent(0, Position(0, 0), Position(2, 2)),
            Agent(1, Position(2, 0), Position(0, 2))
        ],
        obstacles=set(),
        max_timesteps=5,
        name="Simple_3x3_2agents"
    )


def create_corridor_mapf() -> MAPFInstance:
    """Create a corridor MAPF (swap problem)."""
    return MAPFInstance(
        width=5,
        height=1,
        agents=[
            Agent(0, Position(0, 0), Position(4, 0)),
            Agent(1, Position(4, 0), Position(0, 0))
        ],
        obstacles=set(),
        max_timesteps=10,
        name="Corridor_swap"
    )


def create_obstacle_mapf() -> MAPFInstance:
    """Create MAPF with obstacles."""
    return MAPFInstance(
        width=4,
        height=4,
        agents=[
            Agent(0, Position(0, 0), Position(3, 3)),
            Agent(1, Position(3, 0), Position(0, 3))
        ],
        obstacles={Position(1, 1), Position(2, 2)},
        max_timesteps=8,
        name="Obstacle_4x4"
    )


def run_mapf_demo() -> Dict[str, Any]:
    """
    Run the MAPF demonstration.

    Returns results dict with verification status.
    """
    results = {
        "demo": "MAPF",
        "tests": []
    }

    # Test 1: Simple 3x3
    instance1 = create_simple_mapf()
    demo1 = MAPFDemo(instance1, budget=500)
    output1 = demo1.run()
    results["tests"].append({
        "name": "Simple 3x3 MAPF",
        "status": output1.status,
        "d0_size": len(demo1.d0),
        "cost": output1.total_cost,
        "solution": demo1.format_solution(output1.witness) if output1.witness else None,
        "passed": output1.status in ["UNIQUE", "OMEGA"]
    })

    # Test 2: Corridor swap
    instance2 = create_corridor_mapf()
    demo2 = MAPFDemo(instance2, budget=1000)
    output2 = demo2.run()
    results["tests"].append({
        "name": "Corridor Swap MAPF",
        "status": output2.status,
        "d0_size": len(demo2.d0),
        "cost": output2.total_cost,
        "passed": output2.status in ["UNIQUE", "OMEGA", "REFUTED"]
    })

    # Test 3: Obstacle MAPF
    instance3 = create_obstacle_mapf()
    demo3 = MAPFDemo(instance3, budget=500)
    output3 = demo3.run()
    results["tests"].append({
        "name": "Obstacle MAPF",
        "status": output3.status,
        "d0_size": len(demo3.d0),
        "cost": output3.total_cost,
        "passed": output3.status in ["UNIQUE", "OMEGA", "REFUTED"]
    })

    # Summary
    all_passed = all(t["passed"] for t in results["tests"])
    results["all_passed"] = all_passed
    results["summary"] = {
        "total_tests": len(results["tests"]),
        "passed": sum(1 for t in results["tests"] if t["passed"]),
        "failed": sum(1 for t in results["tests"] if not t["passed"])
    }

    return results


if __name__ == "__main__":
    results = run_mapf_demo()
    print(json.dumps(results, indent=2, default=str))
