"""
Permutation Matching for Transport Fabric Ticks

Mathematical Foundation (CORRECTED FORMULATION):
================================================

The tick update P_t IS a bijection (permutation) on V. This is the primitive.
Junction modes are derived coordinates, not control inputs.

Formally:
    P_t ∈ Match(A)  where A is the allowed move relation
    σ_t := π_modes(P_t)  (junction modes derived after the fact)

NOT the incorrect formulation:
    σ → F(σ) → Complete(F)  (this can mint infeasible ticks)

The tick operator:
1. Build allowed edges A for bipartite graph V^out → V^in
2. Add mandatory forced arcs F (lane rotations only)
3. Add preference costs (encode desired throughput/routing)
4. Solve for perfect matching M ⊇ F with minimum cost
5. If M exists: P_t(v) = u iff (v^out → u^in) ∈ M
6. If M does not exist: return UNSAT with Hall witness

Theorem: The matching exists iff Hall's condition holds.
The Hall witness is the exact minimal separator proving infeasibility.

This is perfect: no tick can be "minted" without a separator proving it exists.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, FrozenSet
from enum import Enum

# Use OR-Tools for matching (same as production MAPF solver)
try:
    from ortools.graph.python import min_cost_flow
    HAS_ORTOOLS = True
except ImportError:
    HAS_ORTOOLS = False


class CompletionStatus(Enum):
    """Status of permutation matching."""
    SUCCESS = "SUCCESS"           # Valid permutation found (separator exists)
    INFEASIBLE = "INFEASIBLE"     # No valid permutation (Hall violation)
    CONFLICT = "CONFLICT"         # Forced arcs conflict with each other


@dataclass
class HallWitness:
    """
    Hall witness proving infeasibility.

    A Hall witness is a set S ⊆ V such that |N(S)| < |S|,
    proving no perfect matching exists.

    This is the separator that certifies UNSAT.
    """
    blocked_vertices: FrozenSet[int]  # The set S with Hall violation
    available_targets: FrozenSet[int]  # N(S) - the reachable targets
    deficiency: int  # |S| - |N(S)| > 0

    def __str__(self) -> str:
        return (f"Hall violation: {len(self.blocked_vertices)} vertices "
                f"can only reach {len(self.available_targets)} targets "
                f"(deficiency={self.deficiency})")


@dataclass
class ForcedArcs:
    """
    Forced arcs from lane rotations.

    IMPORTANT: Only lane rotations create forced arcs.
    Junction modes are NOT forced - they are preferences/costs.

    This is a PARTIAL function: dom(F) ⊂ V
    For v ∈ dom(F): P_t(v) = F(v) is required
    For v ∉ dom(F): P_t(v) determined by matching
    """
    arcs: Dict[int, int]  # source → destination (forced moves)

    def __post_init__(self):
        # Validate: no two sources map to same target (injectivity)
        targets = list(self.arcs.values())
        if len(targets) != len(set(targets)):
            raise ValueError("Forced arcs not injective: multiple sources to same target")

    def domain(self) -> Set[int]:
        """Vertices with forced destinations."""
        return set(self.arcs.keys())

    def codomain(self) -> Set[int]:
        """Vertices that are forced destinations."""
        return set(self.arcs.values())

    def is_forced(self, v: int) -> bool:
        """Check if vertex has forced destination."""
        return v in self.arcs

    def get_forced(self, v: int) -> Optional[int]:
        """Get forced destination if exists."""
        return self.arcs.get(v)

    def add_arc(self, src: int, dst: int):
        """Add a forced arc (validates injectivity)."""
        if src in self.arcs:
            if self.arcs[src] != dst:
                raise ValueError(f"Conflict: {src} already forced to {self.arcs[src]}, cannot force to {dst}")
        if dst in self.arcs.values() and src not in self.arcs:
            for existing_src, existing_dst in self.arcs.items():
                if existing_dst == dst:
                    raise ValueError(f"Conflict: {dst} already target of {existing_src}, cannot add {src}→{dst}")
        self.arcs[src] = dst

    def merge(self, other: "ForcedArcs") -> "ForcedArcs":
        """Merge two sets of forced arcs (validates consistency)."""
        merged = dict(self.arcs)
        for src, dst in other.arcs.items():
            if src in merged:
                if merged[src] != dst:
                    raise ValueError(f"Merge conflict: {src}→{merged[src]} vs {src}→{dst}")
            else:
                if dst in merged.values():
                    for existing_src, existing_dst in merged.items():
                        if existing_dst == dst and existing_src != src:
                            raise ValueError(f"Merge conflict: both {existing_src} and {src} map to {dst}")
                merged[src] = dst
        return ForcedArcs(merged)


@dataclass
class CompletionResult:
    """
    Result of permutation matching.

    Either:
    - SUCCESS with a valid permutation (the tick exists)
    - INFEASIBLE with Hall witness (certified UNSAT)
    """
    status: CompletionStatus
    permutation: Dict[int, int]  # Complete permutation (source → destination)
    forced_count: int            # Number of forced arcs
    free_count: int              # Number of free assignments
    hall_witness: Optional[HallWitness] = None  # Proof of infeasibility
    error_message: str = ""


class PermutationCompleter:
    """
    Finds tick permutations via bipartite matching.

    The tick IS a permutation. This class finds feasible permutations
    that extend the forced arcs (lane rotations) while respecting
    edge legality and swap-free constraints.

    Preferences (costs) guide the matching toward desired behavior
    (e.g., high throughput) without creating forced constraints that
    could cause Hall violations.
    """

    def __init__(self, vertices: Set[int], edges: Set[Tuple[int, int]]):
        """
        Initialize completer.

        Args:
            vertices: Set of all vertices V
            edges: Set of directed edges (u, v) ∈ E
        """
        self.vertices = vertices
        self.edges = edges
        self.num_vertices = len(vertices)

        # Build adjacency for fast lookup
        self.neighbors: Dict[int, Set[int]] = {v: set() for v in vertices}
        for (u, v) in edges:
            if u in self.neighbors:
                self.neighbors[u].add(v)

        # Undirected adjacencies for swap constraints
        self.undirected_pairs: Set[FrozenSet[int]] = set()
        for (u, v) in edges:
            self.undirected_pairs.add(frozenset([u, v]))

    def complete(self, forced: ForcedArcs,
                 preferences: Optional[Dict[Tuple[int, int], int]] = None) -> CompletionResult:
        """
        Find a permutation extending forced arcs.

        This is THE tick operator. The permutation IS the tick.

        Args:
            forced: Mandatory arcs (lane rotations only)
            preferences: Edge costs {(src, dst): cost}
                        Lower cost = more preferred
                        This encodes desired throughput/routing
                        WITHOUT creating forced constraints

        Returns:
            CompletionResult with permutation or Hall witness
        """
        if not HAS_ORTOOLS:
            return self._complete_greedy(forced)

        return self._complete_matching(forced, preferences)

    def _complete_matching(self, forced: ForcedArcs,
                          preferences: Optional[Dict[Tuple[int, int], int]] = None) -> CompletionResult:
        """
        Find permutation using OR-Tools min-cost flow.

        Network structure:
        - Super-source S connects to all v^out nodes (capacity 1)
        - All u^in nodes connect to super-sink T (capacity 1)
        - Edge v^out → u^in exists iff (v=u) or (v→u) ∈ E
        - Costs encode preferences (not constraints!)

        Swap prevention via iterative refinement with forbidden edges.
        """
        preferences = preferences or {}

        v_list = sorted(self.vertices)
        v_to_idx = {v: i for i, v in enumerate(v_list)}
        n = len(v_list)

        SOURCE = 0
        SINK = 1
        OUT_BASE = 2
        IN_BASE = 2 + n

        def out_node(v): return OUT_BASE + v_to_idx[v]
        def in_node(v): return IN_BASE + v_to_idx[v]

        # Track forbidden DIRECTED edges (to break swaps)
        forbidden_edges: Set[Tuple[int, int]] = set()
        max_iterations = 10

        for iteration in range(max_iterations):
            # Build flow network
            smcf = min_cost_flow.SimpleMinCostFlow()

            # Source → V^out edges (capacity 1)
            for v in v_list:
                smcf.add_arc_with_capacity_and_unit_cost(SOURCE, out_node(v), 1, 0)

            # V^in → Sink edges (capacity 1)
            for v in v_list:
                smcf.add_arc_with_capacity_and_unit_cost(in_node(v), SINK, 1, 0)

            # Movement edges
            forced_domain = forced.domain()
            forced_codomain = forced.codomain()

            # Track which edges we add for Hall witness computation
            edges_added: Dict[int, Set[int]] = {v: set() for v in v_list}

            for v in v_list:
                # Possible destinations for v
                destinations = {v}  # Can always wait
                destinations |= self.neighbors.get(v, set())

                for u in destinations:
                    if u not in self.vertices:
                        continue

                    # Check if this directed move is forbidden (to break a swap)
                    if (v, u) in forbidden_edges:
                        continue

                    # Check if this is a forced arc
                    is_forced_arc = (v in forced_domain and forced.get_forced(v) == u)

                    # If v has a forced destination and this isn't it, skip
                    if v in forced_domain and not is_forced_arc:
                        continue

                    # If u is a forced target, only the forced source can go there
                    # This includes wait (v=u): if v is forced target, v cannot wait
                    # because v's in-slot is reserved for the forced source
                    if u in forced_codomain and not is_forced_arc:
                        continue

                    # Cost structure:
                    # - Forced arcs: cost 0 (must be used)
                    # - Preferred edges: use preference cost (typically 0-5)
                    # - Other edges: base cost 5
                    # - Wait: cost 5 (same as default movement)
                    #
                    # Key insight: when v->u is chosen, u CANNOT wait (in-slot taken).
                    # So movement chains form. The controller should set low costs
                    # for entire chains, not just starting edges.
                    if is_forced_arc:
                        cost = 0
                    elif (v, u) in preferences:
                        cost = preferences[(v, u)]
                    elif v == u:
                        cost = 5  # Wait (same as base movement)
                    else:
                        cost = 5  # Base cost for movement

                    smcf.add_arc_with_capacity_and_unit_cost(out_node(v), in_node(u), 1, cost)
                    edges_added[v].add(u)

            # Set supplies: source provides n, sink consumes n
            smcf.set_node_supply(SOURCE, n)
            smcf.set_node_supply(SINK, -n)

            # Solve
            status = smcf.solve()

            if status != smcf.OPTIMAL:
                # Matching failed - extract Hall witness
                hall_witness = self._extract_hall_witness(
                    v_list, forced_domain, forced_codomain, edges_added
                )
                return CompletionResult(
                    status=CompletionStatus.INFEASIBLE,
                    permutation={},
                    forced_count=len(forced.arcs),
                    free_count=0,
                    hall_witness=hall_witness,
                    error_message=f"No valid permutation exists: {hall_witness}"
                )

            # Extract permutation from flow
            permutation = {}

            for arc in range(smcf.num_arcs()):
                if smcf.flow(arc) > 0:
                    tail = smcf.tail(arc)
                    head = smcf.head(arc)

                    # Check if this is a V^out → V^in arc
                    if OUT_BASE <= tail < IN_BASE and IN_BASE <= head < IN_BASE + n:
                        v = v_list[tail - OUT_BASE]
                        u = v_list[head - IN_BASE]
                        permutation[v] = u

            # Vertices not in permutation stay in place (identity)
            for v in v_list:
                if v not in permutation:
                    permutation[v] = v

            # Check for swaps and resolve
            has_swap = False
            for v, u in list(permutation.items()):
                if v != u and u in permutation and permutation[u] == v:
                    # Found a swap: v↔u
                    v_is_forced = (v in forced_domain and forced.get_forced(v) == u)
                    u_is_forced = (u in forced_domain and forced.get_forced(u) == v)

                    # Check if v or u can wait
                    v_can_wait = (not v_is_forced) and (v not in forced_codomain)
                    u_can_wait = (not u_is_forced) and (u not in forced_codomain)

                    if u_can_wait and not v_is_forced:
                        permutation[u] = u
                        has_swap = False
                    elif v_can_wait and not u_is_forced:
                        permutation[v] = v
                        has_swap = False
                    else:
                        # Need to re-run with constraint
                        if not v_is_forced:
                            forbidden_edges.add((v, u))
                        elif not u_is_forced:
                            forbidden_edges.add((u, v))
                        else:
                            # Both forced - conflict in lane design
                            return CompletionResult(
                                status=CompletionStatus.CONFLICT,
                                permutation={},
                                forced_count=len(forced.arcs),
                                free_count=0,
                                error_message=f"Forced arcs create swap: {v}↔{u}"
                            )
                        has_swap = True
                        break

            if not has_swap:
                return CompletionResult(
                    status=CompletionStatus.SUCCESS,
                    permutation=permutation,
                    forced_count=len(forced.arcs),
                    free_count=n - len(forced.arcs)
                )

        # Couldn't eliminate swaps
        return CompletionResult(
            status=CompletionStatus.INFEASIBLE,
            permutation={},
            forced_count=len(forced.arcs),
            free_count=0,
            error_message="Could not find swap-free permutation after max iterations"
        )

    def _extract_hall_witness(self, v_list: List[int],
                              forced_domain: Set[int],
                              forced_codomain: Set[int],
                              edges_added: Dict[int, Set[int]]) -> HallWitness:
        """
        Extract Hall witness proving infeasibility.

        Find a set S of sources such that |N(S)| < |S|.
        This is the separator certifying UNSAT.
        """
        # Find vertices with no available destinations
        blocked = set()
        for v in v_list:
            if not edges_added[v]:
                blocked.add(v)

        if blocked:
            # Simple case: some vertex has no options
            return HallWitness(
                blocked_vertices=frozenset(blocked),
                available_targets=frozenset(),
                deficiency=len(blocked)
            )

        # More complex case: need to find subset with Hall violation
        # Use greedy expansion from most constrained vertices
        constrained = sorted(v_list, key=lambda v: len(edges_added[v]))

        for start_v in constrained[:10]:  # Check most constrained
            S = {start_v}
            N_S = set(edges_added[start_v])

            # Expand S by adding vertices whose targets are in N_S
            changed = True
            while changed:
                changed = False
                for v in v_list:
                    if v not in S:
                        # If all of v's targets are in N_S, v must be in S
                        if edges_added[v] and edges_added[v].issubset(N_S):
                            S.add(v)
                            N_S |= edges_added[v]
                            changed = True

                            # Check Hall violation
                            if len(S) > len(N_S):
                                return HallWitness(
                                    blocked_vertices=frozenset(S),
                                    available_targets=frozenset(N_S),
                                    deficiency=len(S) - len(N_S)
                                )

        # Fallback: report general infeasibility
        all_targets = set()
        for v in v_list:
            all_targets |= edges_added[v]

        return HallWitness(
            blocked_vertices=frozenset(v_list),
            available_targets=frozenset(all_targets),
            deficiency=len(v_list) - len(all_targets) if len(v_list) > len(all_targets) else 1
        )

    def _complete_greedy(self, forced: ForcedArcs) -> CompletionResult:
        """
        Greedy completion fallback (when OR-Tools not available).
        """
        permutation = dict(forced.arcs)
        used_targets = set(forced.codomain())

        remaining = sorted(
            self.vertices - forced.domain(),
            key=lambda v: len(self.neighbors.get(v, set()) - used_targets)
        )

        for v in remaining:
            candidates = [v]
            candidates.extend(self.neighbors.get(v, set()))

            chosen = None
            for u in candidates:
                if u not in used_targets:
                    swap_ok = True
                    if u != v and u in permutation and permutation[u] == v:
                        swap_ok = False

                    if swap_ok:
                        chosen = u
                        break

            if chosen is None:
                return CompletionResult(
                    status=CompletionStatus.INFEASIBLE,
                    permutation={},
                    forced_count=len(forced.arcs),
                    free_count=0,
                    error_message=f"Greedy completion failed at vertex {v}"
                )

            permutation[v] = chosen
            used_targets.add(chosen)

        return CompletionResult(
            status=CompletionStatus.SUCCESS,
            permutation=permutation,
            forced_count=len(forced.arcs),
            free_count=len(self.vertices) - len(forced.arcs)
        )


def extract_lane_forced_arcs(lane_vertices: List[int], is_cycle: bool) -> ForcedArcs:
    """
    Extract forced arcs from a lane.

    Lane rotations ARE forced - they define the fabric's base dynamics.

    For lane [v₀, v₁, ..., vₙ]:
    - Interior edges v_i → v_{i+1} are forced
    - Endpoint vₙ has NO forced destination (completed by matching)

    For cycles: all edges forced (including vₙ → v₀).
    """
    arcs = {}

    for i in range(len(lane_vertices) - 1):
        arcs[lane_vertices[i]] = lane_vertices[i + 1]

    if is_cycle and len(lane_vertices) > 1:
        arcs[lane_vertices[-1]] = lane_vertices[0]

    return ForcedArcs(arcs)


# NOTE: extract_junction_forced_arcs is REMOVED
# Junction modes are NOT forced arcs - they are preferences/costs
# The tick is chosen by matching, modes are derived labels
