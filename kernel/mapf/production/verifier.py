"""
Production MAPF Verifier - V1 through V5 Verification Gates

The verifier is the SOURCE OF TRUTH.
Solvers (CBS, ILP, Flow) are merely proposal mechanisms.

Verification Gates:
- V1: Start positions correct
- V2: Goal positions reached (with goal-hold)
- V3: Valid dynamics (edge traversal or wait)
- V4: Vertex capacity (no two agents at same vertex)
- V5: Edge swap prevention (no head-on collisions)

Complexity: O(k²T) where k = agents, T = horizon
This is polynomial and checkable by anyone.

On failure, returns minimal conflict witness (not generic error).
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, FrozenSet
from enum import Enum

from .time_expanded_graph import TimeExpandedGraph, WarehouseGraph
from .path_decompose import PathDecompositionResult, ExtractedPath


class ProductionConflictType(Enum):
    """Type of conflict detected."""
    V1_START = "V1_START"           # Wrong start position
    V2_GOAL = "V2_GOAL"             # Goal not reached
    V3_DYNAMICS = "V3_DYNAMICS"     # Invalid move
    V4_VERTEX = "V4_VERTEX"         # Vertex collision
    V5_EDGE_SWAP = "V5_EDGE_SWAP"   # Edge swap collision


@dataclass
class ProductionConflictWitness:
    """
    Minimal witness for a conflict.

    Contains exact information to reconstruct the violation:
    - Type of conflict
    - Agents involved
    - Time step
    - Location(s)
    """
    conflict_type: ProductionConflictType
    agents: List[int]  # Agent/path IDs involved
    time: int
    vertices: List[int]  # Vertices involved
    details: str  # Human-readable description

    def to_dict(self) -> Dict:
        return {
            "type": self.conflict_type.value,
            "agents": self.agents,
            "time": self.time,
            "vertices": self.vertices,
            "details": self.details
        }


@dataclass
class ProductionVerificationResult:
    """
    Complete verification result.

    Contains:
    - passed: True if all V1-V5 checks pass
    - checks: Individual check results
    - conflicts: List of conflict witnesses (if any)
    - statistics: Verification metrics
    """
    passed: bool
    checks: Dict[str, bool]
    conflicts: List[ProductionConflictWitness] = field(default_factory=list)
    statistics: Dict = field(default_factory=dict)

    def to_dict(self) -> Dict:
        return {
            "passed": self.passed,
            "checks": self.checks,
            "conflicts": [c.to_dict() for c in self.conflicts],
            "statistics": self.statistics
        }


class ProductionMAPFVerifier:
    """
    Complete MAPF solution verifier for production-scale problems.

    Implements the five verification gates (V1-V5) that
    constitute the truth definition for MAPF solutions.

    Usage:
        verifier = ProductionMAPFVerifier(graph, starts, goals, horizon)
        result = verifier.verify(paths)
        if result.passed:
            # Solution is valid
    """

    def __init__(self, graph: WarehouseGraph,
                 starts: FrozenSet[int],
                 goals: FrozenSet[int],
                 horizon: int):
        """
        Initialize verifier.

        Args:
            graph: Base warehouse graph
            starts: Set of start vertices
            goals: Set of goal vertices
            horizon: Time horizon
        """
        self.graph = graph
        self.starts = starts
        self.goals = goals
        self.horizon = horizon

        # Precompute edge set for fast lookup
        self.edge_set = set(graph.edges)

    def verify(self, paths: Dict[int, List[int]]) -> ProductionVerificationResult:
        """
        Verify a MAPF solution.

        Args:
            paths: {agent_id: [v0, v1, ..., vT]}

        Returns:
            ProductionVerificationResult with pass/fail and any conflicts
        """
        conflicts = []
        checks = {
            "V1_START": True,
            "V2_GOAL": True,
            "V3_DYNAMICS": True,
            "V4_VERTEX": True,
            "V5_EDGE_SWAP": True
        }

        # Pad paths to horizon (goal-hold convention)
        padded_paths = self._pad_paths(paths)

        # V1: Start positions
        v1_conflicts = self._check_v1_starts(padded_paths)
        if v1_conflicts:
            checks["V1_START"] = False
            conflicts.extend(v1_conflicts)

        # V2: Goal positions
        v2_conflicts = self._check_v2_goals(padded_paths)
        if v2_conflicts:
            checks["V2_GOAL"] = False
            conflicts.extend(v2_conflicts)

        # V3: Valid dynamics
        v3_conflicts = self._check_v3_dynamics(padded_paths)
        if v3_conflicts:
            checks["V3_DYNAMICS"] = False
            conflicts.extend(v3_conflicts)

        # V4: Vertex collisions
        v4_conflicts = self._check_v4_vertex_collisions(padded_paths)
        if v4_conflicts:
            checks["V4_VERTEX"] = False
            conflicts.extend(v4_conflicts)

        # V5: Edge swap collisions
        v5_conflicts = self._check_v5_edge_swaps(padded_paths)
        if v5_conflicts:
            checks["V5_EDGE_SWAP"] = False
            conflicts.extend(v5_conflicts)

        passed = all(checks.values())

        return ProductionVerificationResult(
            passed=passed,
            checks=checks,
            conflicts=conflicts,
            statistics={
                "num_agents": len(paths),
                "horizon": self.horizon,
                "num_conflicts": len(conflicts),
                "checks_passed": sum(checks.values()),
                "checks_total": len(checks)
            }
        )

    def _pad_paths(self, paths: Dict[int, List[int]]) -> Dict[int, List[int]]:
        """
        Pad paths to horizon using goal-hold convention.

        If path ends before horizon, agent waits at final position.
        """
        padded = {}
        for agent_id, path in paths.items():
            if len(path) <= self.horizon + 1:
                # Pad with final vertex
                padded_path = path + [path[-1]] * (self.horizon + 1 - len(path))
            else:
                # Truncate if too long
                padded_path = path[:self.horizon + 1]
            padded[agent_id] = padded_path
        return padded

    def _check_v1_starts(self, paths: Dict[int, List[int]]) -> List[ProductionConflictWitness]:
        """V1: Verify all agents start at valid start positions."""
        conflicts = []
        path_starts = set()

        for agent_id, path in paths.items():
            start_vertex = path[0]
            path_starts.add(start_vertex)

            if start_vertex not in self.starts:
                conflicts.append(ProductionConflictWitness(
                    conflict_type=ProductionConflictType.V1_START,
                    agents=[agent_id],
                    time=0,
                    vertices=[start_vertex],
                    details=f"Agent {agent_id} starts at {start_vertex}, not in starts"
                ))

        # Check all starts are covered
        if path_starts != self.starts:
            missing = self.starts - path_starts
            if missing:
                conflicts.append(ProductionConflictWitness(
                    conflict_type=ProductionConflictType.V1_START,
                    agents=[],
                    time=0,
                    vertices=sorted(missing),
                    details=f"Start positions not covered: {sorted(missing)}"
                ))

        return conflicts

    def _check_v2_goals(self, paths: Dict[int, List[int]]) -> List[ProductionConflictWitness]:
        """V2: Verify all agents reach goal positions at horizon."""
        conflicts = []
        path_goals = set()

        for agent_id, path in paths.items():
            goal_vertex = path[self.horizon]
            path_goals.add(goal_vertex)

            if goal_vertex not in self.goals:
                conflicts.append(ProductionConflictWitness(
                    conflict_type=ProductionConflictType.V2_GOAL,
                    agents=[agent_id],
                    time=self.horizon,
                    vertices=[goal_vertex],
                    details=f"Agent {agent_id} at {goal_vertex} at t={self.horizon}, not in goals"
                ))

        # Check all goals are covered
        if path_goals != self.goals:
            missing = self.goals - path_goals
            if missing:
                conflicts.append(ProductionConflictWitness(
                    conflict_type=ProductionConflictType.V2_GOAL,
                    agents=[],
                    time=self.horizon,
                    vertices=sorted(missing),
                    details=f"Goal positions not covered: {sorted(missing)}"
                ))

        return conflicts

    def _check_v3_dynamics(self, paths: Dict[int, List[int]]) -> List[ProductionConflictWitness]:
        """V3: Verify all moves are valid (edge or wait)."""
        conflicts = []

        for agent_id, path in paths.items():
            for t in range(len(path) - 1):
                u = path[t]
                v = path[t + 1]

                # Wait is always valid
                if u == v:
                    continue

                # Move must be along edge
                if (u, v) not in self.edge_set:
                    conflicts.append(ProductionConflictWitness(
                        conflict_type=ProductionConflictType.V3_DYNAMICS,
                        agents=[agent_id],
                        time=t,
                        vertices=[u, v],
                        details=f"Agent {agent_id} invalid move {u}->{v} at t={t}"
                    ))

        return conflicts

    def _check_v4_vertex_collisions(self, paths: Dict[int, List[int]]) -> List[ProductionConflictWitness]:
        """V4: Verify no two agents at same vertex at same time."""
        conflicts = []

        for t in range(self.horizon + 1):
            # Get positions at time t
            positions = {}  # vertex -> list of agent_ids
            for agent_id, path in paths.items():
                v = path[t]
                if v not in positions:
                    positions[v] = []
                positions[v].append(agent_id)

            # Check for collisions
            for vertex, agents in positions.items():
                if len(agents) > 1:
                    conflicts.append(ProductionConflictWitness(
                        conflict_type=ProductionConflictType.V4_VERTEX,
                        agents=sorted(agents),
                        time=t,
                        vertices=[vertex],
                        details=f"Vertex collision at v={vertex}, t={t}: agents {agents}"
                    ))

        return conflicts

    def _check_v5_edge_swaps(self, paths: Dict[int, List[int]]) -> List[ProductionConflictWitness]:
        """V5: Verify no edge swap collisions."""
        conflicts = []
        agent_ids = sorted(paths.keys())

        for t in range(self.horizon):
            # Get moves at time t
            moves = {}  # agent_id -> (from, to)
            for agent_id in agent_ids:
                path = paths[agent_id]
                moves[agent_id] = (path[t], path[t + 1])

            # Check all pairs for swaps
            for i, agent_i in enumerate(agent_ids):
                u_i, v_i = moves[agent_i]
                for agent_j in agent_ids[i + 1:]:
                    u_j, v_j = moves[agent_j]

                    # Check for swap: i goes u->v, j goes v->u
                    if u_i == v_j and v_i == u_j and u_i != v_i:
                        conflicts.append(ProductionConflictWitness(
                            conflict_type=ProductionConflictType.V5_EDGE_SWAP,
                            agents=[agent_i, agent_j],
                            time=t,
                            vertices=[u_i, v_i],
                            details=f"Edge swap at t={t}: agent {agent_i} {u_i}->{v_i}, agent {agent_j} {u_j}->{v_j}"
                        ))

        return conflicts

    def get_first_conflict(self, paths: Dict[int, List[int]]) -> Optional[ProductionConflictWitness]:
        """
        Get the first conflict found (deterministic ordering).

        Used by CBS and other refinement algorithms.
        """
        result = self.verify(paths)
        if result.conflicts:
            # Sort by (time, type, agents) for determinism
            sorted_conflicts = sorted(
                result.conflicts,
                key=lambda c: (c.time, c.conflict_type.value, tuple(c.agents))
            )
            return sorted_conflicts[0]
        return None


def verify_production_solution(graph: TimeExpandedGraph,
                               decomposition: PathDecompositionResult) -> ProductionVerificationResult:
    """
    Verify a production-scale path decomposition result.

    Convenience wrapper that extracts paths and runs verification.

    Args:
        graph: Time-expanded graph
        decomposition: Path decomposition result

    Returns:
        ProductionVerificationResult
    """
    paths = {
        assignment.robot_id: assignment.path.vertices
        for assignment in decomposition.robot_assignments
    }

    verifier = ProductionMAPFVerifier(
        graph.base_graph,
        graph.starts,
        graph.goals,
        graph.horizon
    )
    return verifier.verify(paths)


def verify_production_paths(graph: WarehouseGraph,
                           starts: FrozenSet[int],
                           goals: FrozenSet[int],
                           horizon: int,
                           paths: Dict[int, List[int]]) -> ProductionVerificationResult:
    """
    Convenience function to verify MAPF paths.

    Args:
        graph: Warehouse graph
        starts: Start vertices
        goals: Goal vertices
        horizon: Time horizon
        paths: {agent_id: path}

    Returns:
        ProductionVerificationResult
    """
    verifier = ProductionMAPFVerifier(graph, starts, goals, horizon)
    return verifier.verify(paths)
