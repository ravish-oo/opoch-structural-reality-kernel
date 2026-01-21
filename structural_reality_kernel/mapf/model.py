"""
mapf_model.py - Core MAPF model definitions.

Implements the exact MAPF model from MAPF_KERNEL_SPEC_v3:
- Graph G=(V,E), agents 1..k, starts s_i, goals g_i, horizon T
- Dynamics: move along edge or wait each step
- Collisions: forbid vertex conflicts and edge swaps
- Goal-hold convention (mandatory)
- Witness object with receipts
- Output contract: exactly one of UNIQUE / UNSAT / OMEGA_GAP
"""

from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, List, Optional, Set, Tuple, Union
from enum import Enum
import hashlib
import json

try:
    from ..core.receipts import CanonicalJSON
except ImportError:
    # Fallback for direct execution
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from core.receipts import CanonicalJSON


# ============================================================
# CANONICALIZATION (EXACTLY AS SPECIFIED)
# ============================================================

def canon_json(obj: Any) -> str:
    """Canonical JSON: sorted keys, minimal whitespace, UTF-8."""
    return json.dumps(obj, sort_keys=True, separators=(",", ":"))


def sha256_hex(s: str) -> str:
    """SHA-256 hash of string."""
    return hashlib.sha256(s.encode("utf-8")).hexdigest()


def H(obj: Any) -> str:
    """Hash of canonical JSON representation."""
    return sha256_hex(canon_json(obj))


# ============================================================
# ENUMS (EXACTLY AS SPECIFIED)
# ============================================================

class ConflictType(Enum):
    """Types of conflicts in MAPF."""
    VERTEX = "VERTEX"
    EDGE_SWAP = "EDGE_SWAP"


class ResultStatus(Enum):
    """
    Output contract: exactly one of UNIQUE / UNSAT / OMEGA_GAP.

    - UNIQUE: paths + verifier PASS + receipt (solution found)
    - UNSAT: infeasibility certificate (proven impossible)
    - OMEGA_GAP: undecided under budget, with frontier witness
    """
    UNIQUE = "UNIQUE"
    UNSAT = "UNSAT"
    OMEGA_GAP = "OMEGA_GAP"


class VerifierCheck(Enum):
    """Verifier check identifiers V1-V5."""
    V1_START = "V1"       # p_i(0) = s_i for all i
    V2_GOAL = "V2"        # p_i(T) = g_i for all i
    V3_DYNAMICS = "V3"    # valid edge or wait
    V4_VERTEX = "V4"      # no two agents at same v,t
    V5_EDGE_SWAP = "V5"   # no head-on collisions


# ============================================================
# DATA STRUCTURES (EXACTLY AS SPECIFIED)
# ============================================================

@dataclass(frozen=True)
class Conflict:
    """
    Conflict object storing minimal separator witness (τ*).

    For EDGE_SWAP conflicts, stores directed edges per agent.
    This is the first conflict under deterministic ordering.
    """
    conflict_type: ConflictType
    time: int
    agents: Tuple[int, int]  # (i, j) where i < j
    vertex: Optional[int] = None  # For VERTEX conflicts
    edge_i: Optional[Tuple[int, int]] = None  # Directed edge for agent i
    edge_j: Optional[Tuple[int, int]] = None  # Directed edge for agent j

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        d = {
            "type": self.conflict_type.value,
            "time": self.time,
            "agents": list(self.agents)
        }
        if self.vertex is not None:
            d["vertex"] = self.vertex
        if self.edge_i is not None:
            d["edge_i"] = list(self.edge_i)
        if self.edge_j is not None:
            d["edge_j"] = list(self.edge_j)
        return d

    def canonical(self) -> str:
        return canon_json(self.to_dict())

    def fingerprint(self) -> str:
        return sha256_hex(self.canonical())


@dataclass(frozen=True)
class Constraint:
    """
    Constraint for CBS branching.

    For VERTEX: forbids (vertex, time)
    For EDGE_SWAP: forbids directed edge (from, to) at time
    """
    agent: int
    time: int
    vertex: Optional[int] = None  # For vertex constraints
    edge: Optional[Tuple[int, int]] = None  # Directed edge (from, to)

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "agent": self.agent,
            "time": self.time
        }
        if self.vertex is not None:
            d["vertex"] = self.vertex
        if self.edge is not None:
            d["edge"] = list(self.edge)
        return d

    def canonical(self) -> str:
        return canon_json(self.to_dict())


@dataclass
class Graph:
    """
    Graph G = (V, E) for MAPF.

    Vertices are integers. Edges are directed pairs (u, v).
    For undirected graphs, include both (u, v) and (v, u).
    """
    vertices: List[int]
    edges: Set[Tuple[int, int]]

    def neighbors(self, v: int) -> List[int]:
        """Return all vertices reachable from v in one step."""
        return sorted([u for (x, u) in self.edges if x == v])

    def is_edge(self, u: int, v: int) -> bool:
        """Check if (u, v) is a valid edge."""
        return (u, v) in self.edges

    def num_vertices(self) -> int:
        return len(self.vertices)

    def num_edges(self) -> int:
        return len(self.edges)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "vertices": sorted(self.vertices),
            "edges": sorted([list(e) for e in self.edges])
        }

    def canonical(self) -> str:
        return canon_json(self.to_dict())

    def fingerprint(self) -> str:
        return sha256_hex(self.canonical())


@dataclass
class MAPFInstance:
    """
    MAPF instance: graph + starts + goals.

    Defines the problem completely.
    """
    graph: Graph
    starts: List[int]  # s_i for agent i
    goals: List[int]   # g_i for agent i

    @property
    def num_agents(self) -> int:
        return len(self.starts)

    def agent_ids(self) -> List[int]:
        return list(range(self.num_agents))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "graph": self.graph.to_dict(),
            "starts": self.starts,
            "goals": self.goals,
            "num_agents": self.num_agents
        }

    def canonical(self) -> str:
        return canon_json(self.to_dict())

    def fingerprint(self) -> str:
        return sha256_hex(self.canonical())


Path = List[int]  # Path is a list of vertices


@dataclass
class VerifierResult:
    """
    Result of verifier execution.

    Either PASS or FAIL with minimal conflict witness.
    """
    passed: bool
    check: Optional[VerifierCheck] = None  # Which check failed
    conflict: Optional[Conflict] = None     # Minimal separator (τ*)
    details: Optional[Dict[str, Any]] = None  # Additional failure details

    def to_dict(self) -> Dict[str, Any]:
        d = {"passed": self.passed}
        if self.check is not None:
            d["check"] = self.check.value
        if self.conflict is not None:
            d["conflict"] = self.conflict.to_dict()
        if self.details is not None:
            d["details"] = self.details
        return d

    def canonical(self) -> str:
        return canon_json(self.to_dict())


@dataclass
class SolutionWitness:
    """
    Complete solution witness object.

    Contains everything needed to verify the solution.
    """
    instance: MAPFInstance
    horizon: int
    paths: List[Path]
    cost: int  # Sum of path lengths - 1
    verifier_result: VerifierResult
    receipt: str  # SHA256(canonical(witness))

    def makespan(self) -> int:
        """Maximum path length - 1."""
        return max(len(p) - 1 for p in self.paths) if self.paths else 0

    def sum_of_costs(self) -> int:
        """Sum of (path length - 1) for all agents."""
        return sum(len(p) - 1 for p in self.paths)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "instance": self.instance.fingerprint(),
            "horizon": self.horizon,
            "paths": self.paths,
            "cost": self.cost,
            "verifier": "PASS" if self.verifier_result.passed else "FAIL",
            "receipt": self.receipt
        }

    def canonical(self) -> str:
        return canon_json(self.to_dict())


@dataclass
class UNSATCertificate:
    """
    Certificate for proven infeasibility.

    Examples: goal collision, no path exists, etc.
    """
    cert_type: str
    agents: Optional[List[int]] = None
    vertex: Optional[int] = None
    reason: str = ""

    def to_dict(self) -> Dict[str, Any]:
        d = {"type": self.cert_type, "reason": self.reason}
        if self.agents is not None:
            d["agents"] = self.agents
        if self.vertex is not None:
            d["vertex"] = self.vertex
        return d

    def canonical(self) -> str:
        return canon_json(self.to_dict())


@dataclass
class FrontierWitness:
    """
    Frontier witness for OMEGA_GAP state.

    Contains last conflict and best lower bound.
    """
    last_conflict: Optional[Conflict]
    best_lower_bound: Optional[int]
    best_solution_found: Optional[List[Path]]

    def to_dict(self) -> Dict[str, Any]:
        d = {}
        if self.last_conflict is not None:
            d["last_conflict"] = self.last_conflict.to_dict()
        if self.best_lower_bound is not None:
            d["best_lower_bound"] = self.best_lower_bound
        if self.best_solution_found is not None:
            d["best_solution_found"] = True
        return d


@dataclass
class GapInfo:
    """
    Information about resource limit that caused OMEGA_GAP.
    """
    gap_type: str  # "NODE_LIMIT", "TIME_LIMIT", etc.
    nodes_expanded: Optional[int] = None
    time_elapsed_ms: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        d = {"type": self.gap_type}
        if self.nodes_expanded is not None:
            d["nodes_expanded"] = self.nodes_expanded
        if self.time_elapsed_ms is not None:
            d["time_elapsed_ms"] = self.time_elapsed_ms
        return d


@dataclass
class MAPFResult:
    """
    Complete MAPF result.

    Output contract: exactly one of UNIQUE / UNSAT / OMEGA_GAP.
    """
    status: ResultStatus
    paths: Optional[List[Path]] = None      # For UNIQUE
    cost: Optional[int] = None              # For UNIQUE
    receipt: Optional[str] = None           # For UNIQUE
    certificate: Optional[UNSATCertificate] = None  # For UNSAT
    gap: Optional[GapInfo] = None           # For OMEGA_GAP
    frontier: Optional[FrontierWitness] = None  # For OMEGA_GAP
    nodes_expanded: int = 0

    def is_unique(self) -> bool:
        return self.status == ResultStatus.UNIQUE

    def is_unsat(self) -> bool:
        return self.status == ResultStatus.UNSAT

    def is_omega_gap(self) -> bool:
        return self.status == ResultStatus.OMEGA_GAP

    def to_dict(self) -> Dict[str, Any]:
        d = {"status": self.status.value}
        if self.paths is not None:
            d["paths"] = self.paths
        if self.cost is not None:
            d["cost"] = self.cost
        if self.receipt is not None:
            d["receipt"] = self.receipt
        if self.certificate is not None:
            d["certificate"] = self.certificate.to_dict()
        if self.gap is not None:
            d["gap"] = self.gap.to_dict()
        if self.frontier is not None:
            d["frontier"] = self.frontier.to_dict()
        d["nodes_expanded"] = self.nodes_expanded
        return d

    def canonical(self) -> str:
        return canon_json(self.to_dict())

    def fingerprint(self) -> str:
        return sha256_hex(self.canonical())


# ============================================================
# HELPER FUNCTIONS FOR GRAPH CONSTRUCTION
# ============================================================

def create_grid_graph(width: int, height: int) -> Graph:
    """
    Create a grid graph with 4-connectivity.

    Vertices are numbered 0 to width*height-1, row by row.
    """
    vertices = list(range(width * height))
    edges = set()

    def v(x: int, y: int) -> int:
        return y * width + x

    for y in range(height):
        for x in range(width):
            curr = v(x, y)
            # Right neighbor
            if x + 1 < width:
                edges.add((curr, v(x + 1, y)))
                edges.add((v(x + 1, y), curr))
            # Down neighbor
            if y + 1 < height:
                edges.add((curr, v(x, y + 1)))
                edges.add((v(x, y + 1), curr))

    return Graph(vertices=vertices, edges=edges)


def create_line_graph(length: int) -> Graph:
    """Create a simple line graph: 0 -- 1 -- 2 -- ... -- (length-1)."""
    vertices = list(range(length))
    edges = set()
    for i in range(length - 1):
        edges.add((i, i + 1))
        edges.add((i + 1, i))
    return Graph(vertices=vertices, edges=edges)


def create_corridor_with_bypass(length: int, bypass_pos: int) -> Graph:
    """
    Create a corridor with a bypass vertex.

    Main corridor: 0 -- 1 -- 2 -- ... -- (length-1)
    Bypass at position bypass_pos connected to vertex (bypass_pos)
    Bypass vertex is numbered 'length'.
    """
    vertices = list(range(length + 1))
    edges = set()

    # Main corridor
    for i in range(length - 1):
        edges.add((i, i + 1))
        edges.add((i + 1, i))

    # Bypass connection
    bypass_vertex = length
    edges.add((bypass_pos, bypass_vertex))
    edges.add((bypass_vertex, bypass_pos))

    return Graph(vertices=vertices, edges=edges)


def pad_path_to_horizon(path: Path, horizon: int, goal: int) -> Path:
    """
    Pad path to horizon T by repeating the goal (goal-hold convention).

    After agent reaches goal, it may only wait at the goal.
    """
    padded = list(path)
    while len(padded) <= horizon:
        padded.append(goal)
    return padded


def generate_receipt(paths: List[Path], cost: int) -> str:
    """Generate deterministic receipt for solution."""
    witness = {
        "paths": paths,
        "cost": cost,
        "verifier": "PASS"
    }
    return H(witness)
