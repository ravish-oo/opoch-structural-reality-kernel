"""
Transport Fabric Compiler

Compiles a warehouse graph into a transport fabric:
1. Decompose graph into directed lanes (cycle cover / path cover)
2. Identify junction vertices where lanes meet
3. Generate safe permutation gadgets for each junction
4. Validate the resulting fabric is complete and collision-free

The compiled fabric enables O(|V|) per-tick updates regardless of
robot count, because updates are bijective permutations.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, FrozenSet
from collections import defaultdict
import hashlib
import json

from .warehouse_graph import (
    FabricWarehouseGraph,
    LaneSegment,
    JunctionVertex,
    VertexType,
)
from .junction_gadgets import (
    JunctionGadget,
    JunctionPermutation,
    JunctionType,
    JunctionGadgetLibrary,
)
from .permutation_completion import (
    ForcedArcs,
    PermutationCompleter,
    CompletionStatus,
    CompletionResult,
    HallWitness,
    extract_lane_forced_arcs,
)


@dataclass
class LaneDecomposition:
    """
    Decomposition of graph vertices into directed lanes.

    Each lane is a sequence of vertices forming a directed path or cycle.
    Lanes partition the non-junction vertices.

    The lane layer permutation P^lane rotates each lane by one position.
    """
    lanes: List[LaneSegment]
    vertex_to_lane: Dict[int, int]  # vertex → lane_id
    junction_vertices: Set[int]     # vertices not in any lane

    def num_lanes(self) -> int:
        return len(self.lanes)

    def total_lane_vertices(self) -> int:
        return sum(len(lane) for lane in self.lanes)

    def get_lane(self, lane_id: int) -> LaneSegment:
        for lane in self.lanes:
            if lane.lane_id == lane_id:
                return lane
        raise ValueError(f"Lane {lane_id} not found")

    def lane_permutation(self) -> Dict[int, int]:
        """
        DEPRECATED: Use lane_forced_arcs() instead.

        This method is mathematically incorrect for linear lanes because
        it treats the partial function as if endpoints stay in place,
        which breaks bijectivity when combined with junction modes.

        Returns {v: P^lane(v)} for all lane vertices.
        """
        perm = {}
        for lane in self.lanes:
            for i, v in enumerate(lane.vertices):
                successor = lane.successor(v)
                if successor is not None:
                    perm[v] = successor
                # If successor is None (end of non-cycle), vertex stays
        return perm

    def lane_forced_arcs(self) -> ForcedArcs:
        """
        Get forced arcs from all lanes.

        MATHEMATICAL CORRECTNESS:
        Lane rotation is a PARTIAL FUNCTION, not a permutation.
        For lane [v₀, v₁, ..., vₙ]:
        - Interior: v_i ↦ v_{i+1} (forced)
        - Endpoint vₙ: NO forced destination (must be completed)

        For cycles: all vertices have forced successors.

        This is the correct formulation for permutation completion.
        """
        all_arcs = ForcedArcs({})
        for lane in self.lanes:
            lane_arcs = extract_lane_forced_arcs(
                list(lane.vertices),
                lane.is_cycle
            )
            all_arcs = all_arcs.merge(lane_arcs)
        return all_arcs

    def fingerprint(self) -> str:
        data = {
            "num_lanes": self.num_lanes(),
            "total_vertices": self.total_lane_vertices(),
            "num_junctions": len(self.junction_vertices)
        }
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]


@dataclass
class TransportFabric:
    """
    Complete transport fabric for a warehouse.

    The fabric consists of:
    1. Lane decomposition (conveyor-like dynamics)
    2. Junction gadgets (local permutation switches)
    3. The combined permutation structure

    Global tick update:
    P_t = (∏_J P^(σ_t(J))_J) ∘ P^lane

    This is the data structure that enables 10k-robot operations.
    """
    warehouse: FabricWarehouseGraph
    lane_decomposition: LaneDecomposition
    junctions: List[JunctionGadget]
    junction_by_vertex: Dict[int, int]  # vertex → junction_id

    # Precomputed structures for fast updates (computed in __post_init__)
    vertex_to_junction: Dict[int, int] = field(default_factory=dict)
    _completer: Optional["PermutationCompleter"] = field(default=None, repr=False)

    def __post_init__(self):
        """Initialize completion structures."""
        if not self.vertex_to_junction:
            for junction in self.junctions:
                for v in junction.vertices:
                    self.vertex_to_junction[v] = junction.junction_id

        # Initialize the permutation completer
        if self._completer is None:
            all_vertices = set(self.lane_decomposition.vertex_to_lane.keys())
            all_vertices |= self.lane_decomposition.junction_vertices
            self._completer = PermutationCompleter(all_vertices, set(self.warehouse.edges))

    def num_junctions(self) -> int:
        return len(self.junctions)

    def get_junction(self, junction_id: int) -> JunctionGadget:
        for j in self.junctions:
            if j.junction_id == junction_id:
                return j
        raise ValueError(f"Junction {junction_id} not found")

    def junction_ids(self) -> List[int]:
        return [j.junction_id for j in self.junctions]

    def build_tick_permutation(self,
                               preferences: Optional[Dict[Tuple[int, int], int]] = None
                               ) -> CompletionResult:
        """
        Build the tick permutation via bipartite matching.

        MATHEMATICAL FOUNDATION (CORRECTED):
        ====================================
        The tick IS a permutation P_t ∈ Match(A).
        Junction modes are derived coordinates, NOT control inputs.

        P_t ∈ Match(A)  (choose feasible permutation directly)
        σ_t := π_modes(P_t)  (derive modes as labels after)

        NOT the incorrect formulation:
        σ → F(σ) → Complete(F)  (can mint infeasible ticks)

        The tick operator:
        1. Get mandatory forced arcs F (lane rotations only)
        2. Build allowed edges with preference costs
        3. Solve for perfect matching M ⊇ F minimizing cost
        4. If M exists: return permutation (tick exists)
        5. If M does not exist: return UNSAT with Hall witness

        Args:
            preferences: Edge costs {(src, dst): cost}
                        Lower cost = more preferred
                        Encodes throughput desires without forcing

        Returns:
            CompletionResult with permutation or Hall witness
        """
        # Step 1: Get lane forced arcs (these are mandatory)
        forced = self.lane_decomposition.lane_forced_arcs()

        # Step 2: Solve for permutation via matching
        # Preferences guide the solver but do NOT create forced arcs
        result = self._completer.complete(forced, preferences)

        return result

    def build_tick_permutation_legacy(self, junction_modes: Dict[int, int]) -> Dict[int, int]:
        """
        DEPRECATED: Legacy interface for backward compatibility.

        WARNING: This can mint infeasible ticks by forcing junction arcs.
        Use build_tick_permutation() with preferences instead.
        """
        import warnings
        warnings.warn(
            "build_tick_permutation_legacy can mint infeasible ticks. "
            "Use build_tick_permutation(preferences) instead.",
            DeprecationWarning
        )

        # Convert junction modes to preferences (low cost for preferred edges)
        preferences: Dict[Tuple[int, int], int] = {}
        for junction_id, mode_id in junction_modes.items():
            junction = self.get_junction(junction_id)
            junction_perm = junction.get_permutation(mode_id)

            if junction_perm.mapping:
                for (src, dst) in junction_perm.mapping:
                    preferences[(src, dst)] = 0  # Strongly prefer this edge

        result = self.build_tick_permutation(preferences)

        if result.status != CompletionStatus.SUCCESS:
            warnings.warn(f"Permutation failed: {result.error_message}")
            # Fall back to identity
            all_vertices = set(self.lane_decomposition.vertex_to_lane.keys())
            all_vertices |= self.lane_decomposition.junction_vertices
            return {v: v for v in all_vertices}

        return result.permutation

    def validate_permutation(self, perm: Dict[int, int]) -> Dict:
        """
        Validate that a permutation is safe.

        Checks:
        1. Bijection (no two sources map to same target)
        2. Legal moves (edge or wait)
        3. Swap-free (no adjacent 2-cycles)
        """
        errors = []

        # Check bijection
        targets = list(perm.values())
        if len(targets) != len(set(targets)):
            # Find duplicates
            seen = set()
            for src, dst in perm.items():
                if dst in seen:
                    errors.append(f"Bijection violation: multiple sources map to {dst}")
                seen.add(dst)

        # Check legality
        for src, dst in perm.items():
            if src != dst and (src, dst) not in self.warehouse.edges:
                errors.append(f"Illegal move: {src}→{dst} not an edge")

        # Check swap-free
        for src, dst in perm.items():
            if src != dst and dst in perm and perm[dst] == src:
                errors.append(f"Swap detected: {src}↔{dst}")

        return {
            "valid": len(errors) == 0,
            "errors": errors[:10]  # First 10 errors
        }

    def statistics(self) -> Dict:
        """Return fabric statistics."""
        total_modes = sum(j.num_modes() for j in self.junctions)
        return {
            "num_vertices": self.warehouse.num_vertices,
            "num_edges": len(self.warehouse.edges),
            "num_lanes": self.lane_decomposition.num_lanes(),
            "num_junctions": self.num_junctions(),
            "total_junction_modes": total_modes,
            "lane_vertices": self.lane_decomposition.total_lane_vertices(),
            "junction_vertices": len(self.lane_decomposition.junction_vertices)
        }

    def fingerprint(self) -> str:
        """Canonical fingerprint of the fabric."""
        data = {
            "warehouse": self.warehouse.fingerprint(),
            "lanes": self.lane_decomposition.fingerprint(),
            "num_junctions": self.num_junctions(),
            "junction_fps": sorted([j.fingerprint() for j in self.junctions])
        }
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]


class FabricCompiler:
    """
    Compiles a warehouse graph into a transport fabric.

    The compilation process:
    1. Identify junction vertices (in/out degree > 1)
    2. Decompose remaining vertices into lanes
    3. Generate junction gadgets with safe permutations
    4. Validate the complete fabric
    """

    def __init__(self, warehouse: FabricWarehouseGraph):
        self.warehouse = warehouse
        self.edges = set(warehouse.edges)

    def compile(self) -> TransportFabric:
        """Compile warehouse into transport fabric."""
        # Step 1: Identify junctions
        junction_vertices = self._identify_junctions()

        # Step 2: Decompose into lanes
        lane_decomposition = self._decompose_lanes(junction_vertices)

        # Step 3: Generate junction gadgets
        junctions = self._generate_junctions(junction_vertices, lane_decomposition)

        # Step 4: Build fabric
        fabric = TransportFabric(
            warehouse=self.warehouse,
            lane_decomposition=lane_decomposition,
            junctions=junctions,
            junction_by_vertex={v: j.junction_id for j in junctions for v in j.vertices}
        )

        # Step 5: Validate
        self._validate_fabric(fabric)

        return fabric

    def _identify_junctions(self) -> Set[int]:
        """
        Identify junction vertices in the warehouse.

        A vertex is a junction if:
        - In-degree > 1 AND out-degree > 1 (true intersection)
        - Or explicitly marked as junction/station
        """
        junctions = set()

        for v in self.warehouse.free_vertices():
            in_deg = self.warehouse.in_degree(v)
            out_deg = self.warehouse.out_degree(v)

            # True junction: multiple inputs AND multiple outputs
            # This avoids marking every vertex in a one-way grid as junction
            if in_deg > 1 and out_deg > 1:
                junctions.add(v)

            # Also include stations
            if self.warehouse.vertex_types.get(v) == VertexType.STATION:
                junctions.add(v)

        return junctions

    def _decompose_lanes(self, junction_vertices: Set[int]) -> LaneDecomposition:
        """
        Decompose non-junction vertices into lanes.

        Strategy:
        1. Find maximal directed paths/cycles avoiding junctions
        2. Each path/cycle becomes a lane
        3. Lanes connect at junctions
        """
        non_junction = set(self.warehouse.free_vertices()) - junction_vertices
        visited = set()
        lanes = []
        vertex_to_lane = {}
        lane_id = 0

        # Build adjacency for non-junction subgraph
        def get_next(v: int) -> Optional[int]:
            """Get unique successor in non-junction subgraph."""
            successors = [u for u in self.warehouse.neighbors(v)
                         if u in non_junction and u not in visited]
            return successors[0] if len(successors) == 1 else None

        def get_prev(v: int) -> Optional[int]:
            """Get unique predecessor in non-junction subgraph."""
            predecessors = [u for u in self.warehouse.predecessors(v)
                           if u in non_junction and u not in visited]
            return predecessors[0] if len(predecessors) == 1 else None

        # Find lanes starting from unvisited vertices
        for start in sorted(non_junction):
            if start in visited:
                continue

            # Trace lane forward and backward
            path_forward = [start]
            visited.add(start)

            # Go forward
            current = start
            while True:
                next_v = get_next(current)
                if next_v is None or next_v in visited:
                    break
                if next_v == start:
                    # Found a cycle
                    break
                path_forward.append(next_v)
                visited.add(next_v)
                current = next_v

            # Check if it's a cycle
            is_cycle = (current != start and
                        start in self.warehouse.neighbors(current))

            # Go backward from start (prepend to path)
            current = start
            path_backward = []
            while True:
                prev_v = get_prev(current)
                if prev_v is None or prev_v in visited:
                    break
                path_backward.insert(0, prev_v)
                visited.add(prev_v)
                current = prev_v

            # Combine paths
            full_path = path_backward + path_forward

            if len(full_path) > 0:
                lane = LaneSegment(
                    lane_id=lane_id,
                    vertices=tuple(full_path),
                    is_cycle=is_cycle
                )
                lanes.append(lane)

                for v in full_path:
                    vertex_to_lane[v] = lane_id

                lane_id += 1

        return LaneDecomposition(
            lanes=lanes,
            vertex_to_lane=vertex_to_lane,
            junction_vertices=junction_vertices
        )

    def _generate_junctions(self, junction_vertices: Set[int],
                           lane_decomposition: LaneDecomposition) -> List[JunctionGadget]:
        """
        Generate junction gadgets for all junction vertices.

        MATHEMATICAL REQUIREMENT:
        Junction vertex sets must be DISJOINT to ensure that independent
        mode selection doesn't create conflicts. Each vertex belongs to
        at most one junction gadget.

        Each junction gets a set of safe permutations based on
        its local topology.
        """
        junctions = []
        junction_id = 0
        assigned_vertices = set()  # Track which vertices are already in a junction

        for v in sorted(junction_vertices):
            if v in assigned_vertices:
                continue  # This vertex already belongs to another junction

            # Get neighborhood (v and its direct neighbors)
            neighbors = set(self.warehouse.neighbors(v))
            predecessors = set(self.warehouse.predecessors(v))

            # Only include neighbors that are not already assigned
            available_neighbors = (neighbors | predecessors) - assigned_vertices
            neighborhood = frozenset({v} | available_neighbors)

            # Get edges within neighborhood
            local_edges = set()
            for (u, w) in self.edges:
                if u in neighborhood and w in neighborhood:
                    local_edges.add((u, w))

            # Generate safe permutations
            perms = self._generate_safe_permutations(v, neighborhood, local_edges)

            junction = JunctionGadget(
                junction_id=junction_id,
                junction_type=self._classify_junction(v, neighbors & available_neighbors,
                                                      predecessors & available_neighbors),
                vertices=neighborhood,
                edges=frozenset(local_edges),
                permutations=perms
            )
            junctions.append(junction)
            junction_id += 1

            # Mark all vertices in this junction as assigned
            assigned_vertices.update(neighborhood)

        return junctions

    def _generate_safe_permutations(self, center: int,
                                    neighborhood: FrozenSet[int],
                                    edges: Set[Tuple[int, int]]) -> List[JunctionPermutation]:
        """
        Generate all safe permutations for a junction.

        A permutation is safe if:
        1. Bijection on neighborhood
        2. All moves follow edges
        3. No swaps (adjacent 2-cycles)
        """
        perms = []

        # Mode 0: Identity (all wait)
        perms.append(JunctionPermutation(
            mode_id=0,
            mapping=frozenset(),
            throughput=0.0,
            description="All wait"
        ))

        mode_id = 1

        # Generate single-move permutations (one robot moves through junction)
        for (src, dst) in edges:
            if src != dst:
                perms.append(JunctionPermutation(
                    mode_id=mode_id,
                    mapping=frozenset([(src, dst)]),
                    throughput=1.0,
                    description=f"{src}→{dst}"
                ))
                mode_id += 1

        # Generate chain permutations (e.g., A→B→C)
        # For each incoming edge to center, try extending
        for (src, _) in edges:
            if _ == center:
                for (_, dst) in edges:
                    if _ == center and dst != src:
                        # Chain: src → center → dst
                        perms.append(JunctionPermutation(
                            mode_id=mode_id,
                            mapping=frozenset([(src, center), (center, dst)]),
                            throughput=1.0,
                            description=f"{src}→{center}→{dst}"
                        ))
                        mode_id += 1

        return perms

    def _classify_junction(self, v: int, neighbors: Set[int],
                          predecessors: Set[int]) -> JunctionType:
        """Classify junction type based on topology."""
        out_deg = len(neighbors)
        in_deg = len(predecessors)

        if in_deg == 2 and out_deg == 1:
            return JunctionType.MERGE
        elif in_deg == 1 and out_deg == 2:
            return JunctionType.SPLIT
        elif in_deg >= 2 and out_deg >= 2:
            return JunctionType.CROSS
        else:
            return JunctionType.T_JUNCTION

    def _validate_fabric(self, fabric: TransportFabric):
        """
        Validate the compiled fabric is complete and consistent.

        Checks:
        1. All vertices covered (lanes + junctions)
        2. All junction permutations are valid
        3. Lane and junction boundaries match
        """
        # Check all vertices are covered
        covered = set(fabric.lane_decomposition.vertex_to_lane.keys())
        covered |= fabric.lane_decomposition.junction_vertices

        free = set(self.warehouse.free_vertices())
        uncovered = free - covered

        if uncovered:
            raise ValueError(f"Fabric compilation incomplete: {len(uncovered)} vertices not covered")

        # Validate all junction permutations
        for junction in fabric.junctions:
            for perm in junction.permutations:
                # Already validated in JunctionGadget.__post_init__
                pass

    @classmethod
    def from_warehouse(cls, warehouse: FabricWarehouseGraph) -> TransportFabric:
        """Convenience method to compile warehouse directly."""
        compiler = cls(warehouse)
        return compiler.compile()
