"""
Incremental Matching for Scalable Transport Fabric

MATHEMATICAL FOUNDATION:
========================

The scaling key: matchings differ by local augmentations.

Theorem: M △ M' decomposes into disjoint alternating cycles.

So "choosing a new tick permutation" is:
    1. Start from existing perfect matching M
    2. Apply disjoint local cycle flips

This gives O(J·m) per tick instead of O(V²).

Algorithm:
----------
1. Maintain base safe permutation M₀ (cycle cover, always feasible)
2. Each tick: apply bounded local augmentations driven by preferences
3. Each flip preserves perfect matching feasibility
4. Stop when no positive-gain local cycles or hit budget K

Complexity:
-----------
- Evaluate local gains: O(J · m^c) for bounded neighborhood size m
- Apply K flips: O(K · m)
- Linear in junctions, not quadratic in vertices

This is how 10k+ real-time works.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, FrozenSet
from collections import defaultdict


@dataclass
class CycleCover:
    """
    A cycle cover of the vertex set - a base safe permutation.

    Every vertex belongs to exactly one cycle.
    This is always a valid permutation (bijection).
    """
    cycles: List[List[int]]  # List of cycles, each cycle is [v₀, v₁, ..., vₖ] where vₖ→v₀
    vertex_to_cycle: Dict[int, int]  # vertex → cycle index
    permutation: Dict[int, int]  # The actual permutation mapping

    @classmethod
    def from_graph(cls, vertices: Set[int], edges: Set[Tuple[int, int]]) -> "CycleCover":
        """
        Build a SWAP-FREE cycle cover from a graph.

        CRITICAL: 2-cycles (swaps) are forbidden.
        In bipartite graphs (grids), minimum cycle length is 4.

        Strategy:
        1. Try to build 4-cycles (or longer) greedily
        2. Vertices not in cycles become self-loops (wait)
        """
        # Build adjacency
        neighbors: Dict[int, Set[int]] = {v: set() for v in vertices}
        for (u, v) in edges:
            if u in neighbors:
                neighbors[u].add(v)

        cycles = []
        vertex_to_cycle = {}
        visited = set()

        # Try to build 4-cycles (minimum non-swap cycle in bipartite graph)
        for start in sorted(vertices):
            if start in visited:
                continue

            # Try to find a 4-cycle from this vertex
            # Path: start -> a -> b -> c -> start
            found_cycle = False

            for a in neighbors.get(start, set()):
                if a in visited:
                    continue

                for b in neighbors.get(a, set()):
                    if b in visited or b == start:
                        continue

                    for c in neighbors.get(b, set()):
                        if c in visited or c == a:
                            continue

                        # Check if c connects back to start
                        if start in neighbors.get(c, set()):
                            # Found a 4-cycle!
                            cycle = [start, a, b, c]
                            cycle_idx = len(cycles)
                            cycles.append(cycle)
                            for v in cycle:
                                vertex_to_cycle[v] = cycle_idx
                                visited.add(v)
                            found_cycle = True
                            break
                    if found_cycle:
                        break
                if found_cycle:
                    break

            if not found_cycle:
                # Could not find a 4-cycle, make this vertex a self-loop
                visited.add(start)
                cycle_idx = len(cycles)
                cycles.append([start])
                vertex_to_cycle[start] = cycle_idx

        # Build permutation from cycles
        permutation = {}
        for cycle in cycles:
            if len(cycle) == 1:
                permutation[cycle[0]] = cycle[0]  # Self-loop (wait)
            else:
                for i, v in enumerate(cycle):
                    next_v = cycle[(i + 1) % len(cycle)]
                    permutation[v] = next_v

        return cls(cycles=cycles, vertex_to_cycle=vertex_to_cycle, permutation=permutation)

    def is_valid_permutation(self) -> bool:
        """Verify this is a valid bijection."""
        targets = set(self.permutation.values())
        return len(targets) == len(self.permutation)


@dataclass
class AugmentingCycle:
    """
    An alternating cycle that can be flipped to improve the matching.

    The cycle alternates between:
    - Edges IN the current matching (to be removed)
    - Edges NOT in matching but in allowed moves (to be added)

    Flipping swaps which edges are in the matching.
    """
    vertices: List[int]  # Cycle vertices in order
    gain: float  # Weight gain from flipping (positive = improvement)

    def edges_to_add(self) -> List[Tuple[int, int]]:
        """Edges that will be added to matching after flip."""
        result = []
        for i in range(0, len(self.vertices), 2):
            u = self.vertices[i]
            v = self.vertices[(i + 1) % len(self.vertices)]
            result.append((u, v))
        return result

    def edges_to_remove(self) -> List[Tuple[int, int]]:
        """Edges that will be removed from matching after flip."""
        result = []
        for i in range(1, len(self.vertices), 2):
            u = self.vertices[i]
            v = self.vertices[(i + 1) % len(self.vertices)]
            result.append((u, v))
        return result


class IncrementalMatcher:
    """
    Maintains a perfect matching and applies local augmentations.

    This is the scalable tick operator:
    - Start from base cycle cover M₀
    - Apply bounded local augmentations each tick
    - Each augmentation preserves matching feasibility
    - O(J·m) per tick instead of O(V²)
    """

    def __init__(self, vertices: Set[int], edges: Set[Tuple[int, int]],
                 junction_neighborhoods: List[FrozenSet[int]],
                 max_augmentations_per_tick: int = 100):
        """
        Initialize incremental matcher.

        Args:
            vertices: All vertices V
            edges: Allowed move edges E
            junction_neighborhoods: List of junction neighborhoods for local search
            max_augmentations_per_tick: Budget K for augmentations per tick
        """
        self.vertices = vertices
        self.edges = edges
        self.num_vertices = len(vertices)

        # Build adjacency
        self.neighbors: Dict[int, Set[int]] = {v: set() for v in vertices}
        for (u, v) in edges:
            if u in self.neighbors:
                self.neighbors[u].add(v)

        # Junction neighborhoods for local search
        self.neighborhoods = junction_neighborhoods

        # OPTIMIZATION: Precompute vertex → neighborhood indices mapping
        # This allows O(occupied) filtering instead of O(neighborhoods * neighborhood_size)
        self.vertex_to_neighborhoods: Dict[int, List[int]] = {v: [] for v in vertices}
        for idx, neighborhood in enumerate(junction_neighborhoods):
            for v in neighborhood:
                if v in self.vertex_to_neighborhoods:
                    self.vertex_to_neighborhoods[v].append(idx)

        # Budget
        self.max_augmentations = max_augmentations_per_tick

        # Build base cycle cover
        self.base_cover = CycleCover.from_graph(vertices, edges)

        # Current matching (starts as base cover)
        self.current_matching: Dict[int, int] = dict(self.base_cover.permutation)

        # Reverse mapping for fast lookup
        self._rebuild_reverse()

    def _rebuild_reverse(self):
        """Rebuild reverse mapping (target → source)."""
        self.reverse_matching: Dict[int, int] = {}
        for src, dst in self.current_matching.items():
            self.reverse_matching[dst] = src

    def get_permutation(self) -> Dict[int, int]:
        """Get current matching as permutation."""
        return dict(self.current_matching)

    def update_tick(self, weights: Dict[Tuple[int, int], float],
                    occupied_vertices: Optional[Set[int]] = None) -> Dict[int, int]:
        """
        Update matching for this tick using local augmentations.

        OPTIMIZATION: Only search neighborhoods containing occupied vertices.
        Empty neighborhoods have no robots and no improvement to make.

        Args:
            weights: Edge weights {(src, dst): weight}
                    Higher weight = more desirable
            occupied_vertices: Optional set of occupied vertices for filtering
                              If provided, only searches relevant neighborhoods

        Returns:
            Updated permutation (always feasible)
        """
        augmentations_applied = 0

        # OPTIMIZATION: Only search neighborhoods containing occupied vertices
        if occupied_vertices is not None:
            # Use precomputed vertex → neighborhood mapping for O(occupied) lookup
            relevant_neighborhood_indices = set()
            for v in occupied_vertices:
                relevant_neighborhood_indices.update(self.vertex_to_neighborhoods.get(v, []))
            relevant_neighborhoods = [self.neighborhoods[i] for i in relevant_neighborhood_indices]
        else:
            relevant_neighborhoods = self.neighborhoods

        # Search relevant neighborhoods for positive-gain operations
        for neighborhood in relevant_neighborhoods:
            if augmentations_applied >= self.max_augmentations:
                break

            # Try 1: Find edge swap (requires existing non-self-loop edges)
            swap = self._find_augmenting_cycle(neighborhood, weights)
            if swap is not None and swap.gain > 0:
                self._apply_augmentation(swap)
                augmentations_applied += 1
                continue

            # Try 2: Activate a 4-cycle from self-loops
            activation = self._find_cycle_activation(neighborhood, weights)
            if activation is not None and activation.gain > 0:
                self._apply_cycle_activation(activation)
                augmentations_applied += 1

        return self.get_permutation()

    def _find_cycle_activation(self, neighborhood: FrozenSet[int],
                                weights: Dict[Tuple[int, int], float]
                                ) -> Optional[AugmentingCycle]:
        """
        Find 4 self-loops that can be converted to a 4-cycle.

        This is needed when the current matching has many self-loops
        that could be "activated" into a cycle for movement.

        A valid 4-cycle activation requires:
        - 4 vertices a, b, c, d that are all self-loops (a→a, b→b, c→c, d→d)
        - Edges a→b, b→c, c→d, d→a all exist in the graph
        - After: a→b→c→d→a (4-cycle)
        """
        # Find all self-loops in neighborhood
        self_loops = [v for v in neighborhood
                      if self.current_matching.get(v) == v]

        if len(self_loops) < 4:
            return None

        best_activation = None
        best_gain = 0

        # Try to find 4 vertices that can form a 4-cycle
        for a in self_loops:
            for b in self.neighbors.get(a, set()):
                if b not in self_loops or b == a:
                    continue
                for c in self.neighbors.get(b, set()):
                    if c not in self_loops or c == a or c == b:
                        continue
                    for d in self.neighbors.get(c, set()):
                        if d not in self_loops or d in (a, b, c):
                            continue
                        # Check if d→a is allowed (closes the cycle)
                        if a not in self.neighbors.get(d, set()):
                            continue

                        # Found a valid 4-cycle: a→b→c→d→a
                        # Gain = new_weight - old_weight
                        # Old: all self-loops (weight 0 for self-loops typically)
                        old_weight = (weights.get((a, a), 0) + weights.get((b, b), 0) +
                                     weights.get((c, c), 0) + weights.get((d, d), 0))
                        new_weight = (weights.get((a, b), 0) + weights.get((b, c), 0) +
                                     weights.get((c, d), 0) + weights.get((d, a), 0))
                        gain = new_weight - old_weight

                        if gain > best_gain:
                            best_gain = gain
                            best_activation = AugmentingCycle(
                                vertices=[a, b, c, d],
                                gain=gain
                            )

        return best_activation

    def _apply_cycle_activation(self, activation: AugmentingCycle):
        """
        Convert 4 self-loops into a 4-cycle.

        Given [a, b, c, d], change:
        - a→a to a→b
        - b→b to b→c
        - c→c to c→d
        - d→d to d→a
        """
        a, b, c, d = activation.vertices
        self.current_matching[a] = b
        self.current_matching[b] = c
        self.current_matching[c] = d
        self.current_matching[d] = a
        self._rebuild_reverse()

    def _find_augmenting_cycle(self, neighborhood: FrozenSet[int],
                                weights: Dict[Tuple[int, int], float]
                                ) -> Optional[AugmentingCycle]:
        """
        Find a positive-gain edge swap in the neighborhood.

        For permutation matching, a valid local modification is an EDGE SWAP:
        Given two edges (a→b) and (c→d) in the current matching,
        swap them to (a→d) and (c→b).

        This preserves bijectivity because:
        - b's source changes from a to c
        - d's source changes from c to a
        - No other incoming/outgoing edges change

        Requirements:
        - All 4 vertices a, b, c, d are distinct
        - (a, d) and (c, b) are allowed edges in the graph
        - No swaps created: d doesn't map to a, b doesn't map to c
        """
        neighborhood_list = list(neighborhood)
        best_swap = None
        best_gain = 0

        # Find pairs of edges (a→b, c→d) that can be swapped to (a→d, c→b)
        for a in neighborhood_list:
            b = self.current_matching.get(a)
            if b is None or b not in neighborhood:
                continue
            # Skip self-loops
            if b == a:
                continue

            for c in neighborhood_list:
                if c == a or c == b:
                    continue

                d = self.current_matching.get(c)
                if d is None or d not in neighborhood:
                    continue
                # Skip self-loops
                if d == c:
                    continue
                # All 4 must be distinct
                if d == a or d == b:
                    continue

                # Check if swap edges (a→d) and (c→b) are allowed
                if d not in self.neighbors.get(a, set()):
                    continue
                if b not in self.neighbors.get(c, set()):
                    continue

                # Check swap-free constraint:
                # After swap: a→d, c→b
                # No swap if: d doesn't map to a, b doesn't map to c
                if self.current_matching.get(d) == a:
                    continue  # Would create swap a↔d
                if self.current_matching.get(b) == c:
                    continue  # Would create swap c↔b

                # Calculate gain
                old_weight = weights.get((a, b), 0) + weights.get((c, d), 0)
                new_weight = weights.get((a, d), 0) + weights.get((c, b), 0)
                gain = new_weight - old_weight

                if gain > best_gain:
                    best_gain = gain
                    # Store as [a, d, c, b] for _apply_augmentation
                    # which will set: a→d, c→b
                    best_swap = AugmentingCycle(
                        vertices=[a, b, c, d],  # Store original edges for clarity
                        gain=gain
                    )

        return best_swap

    def _would_create_swap(self, src: int, dst: int) -> bool:
        """Check if adding src→dst would create a swap (2-cycle)."""
        # Swap occurs if dst currently maps to src
        return self.current_matching.get(dst) == src

    def _apply_augmentation(self, swap: AugmentingCycle):
        """
        Apply an edge swap.

        Given vertices [a, b, c, d] representing original edges:
        - (a → b)
        - (c → d)

        Swap to:
        - (a → d)
        - (c → b)

        This preserves bijectivity:
        - b's source changes from a to c
        - d's source changes from c to a
        - All other edges unchanged
        """
        vertices = swap.vertices
        if len(vertices) != 4:
            raise ValueError(f"Edge swap requires exactly 4 vertices, got {len(vertices)}")

        a, b, c, d = vertices

        # Apply the swap: (a→b, c→d) becomes (a→d, c→b)
        self.current_matching[a] = d
        self.current_matching[c] = b

        # Rebuild reverse mapping
        self._rebuild_reverse()

    def reset_to_base(self):
        """Reset matching to base cycle cover."""
        self.current_matching = dict(self.base_cover.permutation)
        self._rebuild_reverse()

    def verify_matching(self) -> bool:
        """Verify current matching is a valid permutation."""
        # Check bijection
        targets = set(self.current_matching.values())
        if len(targets) != len(self.current_matching):
            return False

        # Check all vertices covered
        if set(self.current_matching.keys()) != self.vertices:
            return False

        # Check no swaps
        for src, dst in self.current_matching.items():
            if src != dst and self.current_matching.get(dst) == src:
                return False

        return True


def build_incremental_matcher(fabric: "TransportFabric") -> IncrementalMatcher:
    """
    Build an incremental matcher from a compiled fabric.

    Uses junction neighborhoods for local search regions.
    """
    # Get vertices and edges
    vertices = set(fabric.lane_decomposition.vertex_to_lane.keys())
    vertices |= fabric.lane_decomposition.junction_vertices
    edges = set(fabric.warehouse.edges)

    # Get junction neighborhoods
    neighborhoods = [j.vertices for j in fabric.junctions]

    return IncrementalMatcher(
        vertices=vertices,
        edges=edges,
        junction_neighborhoods=neighborhoods,
        max_augmentations_per_tick=len(neighborhoods)  # At most one per junction
    )
