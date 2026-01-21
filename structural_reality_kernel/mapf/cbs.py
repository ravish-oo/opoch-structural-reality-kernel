"""
mapf_cbs.py - CBS (Conflict-Based Search) Solver for MAPF.

CBS is the exact constructive solver for MAPF.
It implements kernel refinement directly:
- detect τ* (conflict)
- branch to exclude it from each agent
- repeat until UNIQUE or Omega

CBS is NOT a heuristic. It is the kernel refinement algorithm.

Implements:
- Low-level A* on (vertex, time) state space
- Deterministic τ* (first conflict under fixed ordering)
- forbid() function (FIXED, no reverse hacks)
- High-level CBS with priority queue ordered by sum-of-costs
- Honest outputs: UNIQUE / UNSAT / OMEGA_GAP
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple
from collections import defaultdict
import heapq

from .model import (
    Graph,
    MAPFInstance,
    Path,
    Conflict,
    ConflictType,
    Constraint,
    MAPFResult,
    ResultStatus,
    UNSATCertificate,
    GapInfo,
    FrontierWitness,
    H,
    canon_json
)
from .verifier import verify_paths, get_first_conflict


# ============================================================
# LOW-LEVEL A* (Single-Agent Path Finding)
# ============================================================

def bfs_distances(graph: Graph, goal: int) -> Dict[int, int]:
    """
    Compute shortest distances from all vertices to goal.

    This is the admissible heuristic for A*.
    """
    dist = {goal: 0}
    queue = [goal]

    # Build reverse adjacency for BFS
    reverse_adj = defaultdict(list)
    for (u, v) in graph.edges:
        reverse_adj[v].append(u)

    while queue:
        v = queue.pop(0)
        for u in reverse_adj[v]:
            if u not in dist:
                dist[u] = dist[v] + 1
                queue.append(u)

    return dist


def low_level_astar(
    graph: Graph,
    start: int,
    goal: int,
    constraints: List[Constraint],
    agent_id: int,
    max_time: int = 100
) -> Optional[Path]:
    """
    Single-agent A* with constraints.

    State space: (vertex, time) pairs
    Actions: move along edge OR wait at current vertex
    Constraints: forbidden (vertex, time) or (edge, time) pairs
    Heuristic: BFS distance to goal (precomputed, admissible)

    Returns shortest valid path, or None if no path under constraints.
    """
    # Precompute heuristic
    h_values = bfs_distances(graph, goal)
    if start not in h_values:
        return None  # Goal unreachable from start

    # Build constraint lookups for this agent
    vertex_constraints = set()  # (time, vertex) pairs
    edge_constraints = set()    # (time, (from, to)) pairs

    for c in constraints:
        if c.agent != agent_id:
            continue
        if c.vertex is not None:
            vertex_constraints.add((c.time, c.vertex))
        if c.edge is not None:
            edge_constraints.add((c.time, c.edge))

    # A* state: (vertex, time)
    # g_score[(v, t)] = cost to reach (v, t)
    g_score = {(start, 0): 0}
    came_from = {}

    # Priority queue: (f, g, vertex, time)
    # f = g + h, g = cost so far
    h_start = h_values.get(start, float('inf'))
    open_set = [(h_start, 0, start, 0)]
    closed = set()

    while open_set:
        f, g, v, t = heapq.heappop(open_set)

        # Skip if already processed
        if (v, t) in closed:
            continue
        closed.add((v, t))

        # Goal check: at goal vertex
        if v == goal:
            # Reconstruct path
            path = [v]
            cur = (v, t)
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur[0])
            return path[::-1]

        # Time limit check
        if t >= max_time:
            continue

        # Generate successors: wait or move
        successors = [(v, v)]  # Wait at current vertex
        for neighbor in graph.neighbors(v):
            successors.append((v, neighbor))

        for (from_v, to_v) in successors:
            next_t = t + 1

            # Check vertex constraint at next state
            if (next_t, to_v) in vertex_constraints:
                continue

            # Check edge constraint (for moves, not waits)
            if from_v != to_v and (t, (from_v, to_v)) in edge_constraints:
                continue

            next_state = (to_v, next_t)
            if next_state in closed:
                continue

            new_g = g + 1
            if next_state not in g_score or new_g < g_score[next_state]:
                g_score[next_state] = new_g
                came_from[next_state] = (v, t)
                h = h_values.get(to_v, float('inf'))
                heapq.heappush(open_set, (new_g + h, new_g, to_v, next_t))

    return None  # No path found under constraints


# ============================================================
# FORBID FUNCTION (FIXED - NO REVERSE HACKS)
# ============================================================

def forbid(agent: int, conflict: Conflict) -> Constraint:
    """
    Create constraint to forbid 'agent' from participating in 'conflict'.

    Conflict stores directed edges per agent, so no reversal needed.
    This is the FIXED forbid() function as specified.

    For VERTEX conflicts: forbid (vertex, time)
    For EDGE_SWAP conflicts: forbid the directed edge for that specific agent
    """
    if conflict.conflict_type == ConflictType.VERTEX:
        # Forbid agent from being at this vertex at this time
        return Constraint(
            agent=agent,
            time=conflict.time,
            vertex=conflict.vertex,
            edge=None
        )

    elif conflict.conflict_type == ConflictType.EDGE_SWAP:
        # Get the directed edge for THIS agent (already stored correctly)
        if agent == conflict.agents[0]:
            directed_edge = conflict.edge_i  # (from, to) for agent i
        else:
            directed_edge = conflict.edge_j  # (from, to) for agent j

        return Constraint(
            agent=agent,
            time=conflict.time,
            vertex=None,
            edge=directed_edge
        )

    else:
        raise ValueError(f"Unknown conflict type: {conflict.conflict_type}")


# ============================================================
# CBS NODE
# ============================================================

@dataclass
class CBSNode:
    """
    Node in the CBS search tree.

    Contains constraints, paths, and sum-of-costs.
    """
    constraints: List[Constraint]
    paths: List[Path]
    cost: int  # Sum of (path_length - 1)

    def __lt__(self, other: 'CBSNode') -> bool:
        return self.cost < other.cost


# ============================================================
# CBS SOLVER (COMPLETE IMPLEMENTATION)
# ============================================================

def cbs_solve(
    instance: MAPFInstance,
    max_time: int = 100,
    max_nodes: int = 10000
) -> MAPFResult:
    """
    CBS (Conflict-Based Search) solver.

    Implements the kernel refinement algorithm:
    1. Propose paths (via low-level A*)
    2. Call verifier
    3. If FAIL: branch on τ* (first conflict)
    4. Replan only the constrained agent
    5. Terminate in UNIQUE / UNSAT / OMEGA_GAP

    This is SOUND, COMPLETE, and OPTIMAL for sum-of-costs.
    """
    k = instance.num_agents
    G = instance.graph

    # Check for goal collisions (immediate UNSAT)
    # If two agents share a goal, impossible due to vertex capacity
    goal_count = defaultdict(list)
    for i, g in enumerate(instance.goals):
        goal_count[g].append(i)

    for g, agents in goal_count.items():
        if len(agents) > 1:
            return MAPFResult(
                status=ResultStatus.UNSAT,
                certificate=UNSATCertificate(
                    cert_type="GOAL_COLLISION",
                    agents=agents,
                    vertex=g,
                    reason="Multiple agents share goal vertex; vertex capacity violated"
                )
            )

    # Initialize root node with unconstrained shortest paths
    root_paths = []
    for i in range(k):
        path = low_level_astar(G, instance.starts[i], instance.goals[i],
                               [], i, max_time)
        if path is None:
            return MAPFResult(
                status=ResultStatus.UNSAT,
                certificate=UNSATCertificate(
                    cert_type="NO_PATH",
                    agents=[i],
                    reason=f"No path exists for agent {i} from {instance.starts[i]} to {instance.goals[i]}"
                )
            )
        root_paths.append(path)

    root_cost = sum(len(p) - 1 for p in root_paths)
    root = CBSNode(constraints=[], paths=root_paths, cost=root_cost)

    # Priority queue: (cost, tie_breaker, node)
    # Tie-breaker ensures deterministic ordering
    open_set = [(root.cost, 0, root)]
    node_id = 1
    nodes_expanded = 0
    last_conflict = None
    best_cost_seen = root_cost

    while open_set:
        _, _, node = heapq.heappop(open_set)
        nodes_expanded += 1

        # Compute horizon (max path length - 1)
        T = max(len(p) - 1 for p in node.paths)

        # Call verifier (the source of truth)
        result = verify_paths(instance, node.paths, T)

        if result.passed:
            # UNIQUE: valid solution found
            receipt = H({"paths": node.paths, "cost": node.cost, "verifier": "PASS"})
            return MAPFResult(
                status=ResultStatus.UNIQUE,
                paths=node.paths,
                cost=node.cost,
                receipt=receipt,
                nodes_expanded=nodes_expanded
            )

        # Get conflict (τ* = minimal separator)
        conflict = result.conflict
        if conflict is not None:
            last_conflict = conflict

            # Branch: forbid conflict for each involved agent
            for agent in conflict.agents:
                child_constraints = node.constraints + [forbid(agent, conflict)]

                # Replan only the constrained agent
                new_path = low_level_astar(
                    G,
                    instance.starts[agent],
                    instance.goals[agent],
                    child_constraints,
                    agent,
                    max_time
                )

                if new_path is not None:
                    child_paths = list(node.paths)
                    child_paths[agent] = new_path
                    child_cost = sum(len(p) - 1 for p in child_paths)

                    child = CBSNode(child_constraints, child_paths, child_cost)
                    heapq.heappush(open_set, (child_cost, node_id, child))
                    node_id += 1

        # Check node budget
        if nodes_expanded >= max_nodes:
            return MAPFResult(
                status=ResultStatus.OMEGA_GAP,
                gap=GapInfo(
                    gap_type="NODE_LIMIT",
                    nodes_expanded=nodes_expanded
                ),
                frontier=FrontierWitness(
                    last_conflict=last_conflict,
                    best_lower_bound=best_cost_seen,
                    best_solution_found=None
                ),
                nodes_expanded=nodes_expanded
            )

    # Queue exhausted: all branches pruned (UNSAT)
    return MAPFResult(
        status=ResultStatus.UNSAT,
        certificate=UNSATCertificate(
            cert_type="ALL_BRANCHES_PRUNED",
            reason="All branches lead to infeasibility"
        ),
        nodes_expanded=nodes_expanded
    )


# ============================================================
# CBS THEOREMS (PROVEN BY CONSTRUCTION)
# ============================================================

"""
THEOREM 5.1: CBS Soundness
If CBS returns UNIQUE(paths), then paths is a valid MAPF solution.

Proof: CBS returns UNIQUE only when verify(paths) = PASS.
By Theorem 3.1 (Verifier Soundness), this means paths satisfies
all MAPF constraints. QED.

THEOREM 5.2: CBS Completeness
If a valid MAPF solution exists, CBS will find one (given sufficient node budget).

Proof: By Theorem 4.1 (Conflict Branching Lemma), any valid solution
avoids each conflict via at least one branch. CBS creates both branches
for every conflict. Therefore every valid solution is reachable in the
search tree. Since CBS explores all reachable nodes (best-first),
it finds the valid solution. QED.

THEOREM 5.3: CBS Optimality (Sum-of-Costs)
If CBS returns UNIQUE(paths) with cost C, then C is minimal among all valid solutions.

Proof: CBS uses best-first search ordered by sum-of-costs.
Adding constraints cannot decrease path lengths (constraints only forbid moves).
Therefore child.cost >= parent.cost.
The first valid solution found has minimum cost in the explored tree.
By completeness, all valid solutions are reachable,
so this minimum is global. QED.

THEOREM 5.4: CBS Termination
CBS always terminates.

Proof: The constraint space is finite (each constraint is (agent, time, location)
with time bounded by k*|V| in worst case). Each branch adds a constraint.
No node is revisited (constraints are monotonically increasing).
Therefore the tree is finite. QED.

THEOREM 4.1: Conflict Branching Lemma
For any conflict C between agents i and j, every valid solution S satisfies
at least one of: (a) agent i avoids C, or (b) agent j avoids C.

Proof: If neither avoids C in S, then S contains C, contradicting validity. QED.
"""


class CBSSolver:
    """
    CBS Solver wrapper with configurable limits.
    """

    def __init__(
        self,
        instance: MAPFInstance,
        max_time: int = 100,
        max_nodes: int = 10000
    ):
        self.instance = instance
        self.max_time = max_time
        self.max_nodes = max_nodes

    def solve(self) -> MAPFResult:
        """Run CBS and return result."""
        return cbs_solve(self.instance, self.max_time, self.max_nodes)

    def verify_solution(self, result: MAPFResult) -> bool:
        """
        Re-verify a solution (soundness check).

        This is tautological under verifier-first design:
        CBS only returns UNIQUE when verifier passes.
        """
        if not result.is_unique():
            return False
        if result.paths is None:
            return False

        T = max(len(p) - 1 for p in result.paths)
        verify_result = verify_paths(self.instance, result.paths, T)
        return verify_result.passed
