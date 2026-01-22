"""
Permutation Executor for Transport Fabric

Applies tick permutations to occupancy state with O(|V|) complexity.

CORRECTED FORMULATION:
The tick IS a permutation P_t ∈ Match(A).
Junction modes are derived coordinates, not inputs.

Mathematical Foundation:
-----------------------
Occupancy: ρ_t: V → {0,1}
Permutation: P_t: V → V (bijection found by matching)
Update: ρ_{t+1} = ρ_t ∘ P_t^{-1}

In code: new_occupancy[P(v)] = old_occupancy[v]

This is O(|V|) regardless of robot count k.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, FrozenSet, Iterator, TYPE_CHECKING
import hashlib
import json
import array

if TYPE_CHECKING:
    from .permutation_completion import CompletionResult


@dataclass
class Permutation:
    """
    A permutation P: V → V represented sparsely.

    Only stores non-identity mappings: {v: P(v)} where P(v) ≠ v.
    Identity positions are implicit.
    """
    mapping: Dict[int, int]  # v → P(v) for non-identity
    num_vertices: int

    def apply(self, v: int) -> int:
        """Apply permutation: return P(v)."""
        return self.mapping.get(v, v)

    def inverse_apply(self, v: int) -> int:
        """Apply inverse: return P^{-1}(v)."""
        # Find w such that P(w) = v
        for w, pv in self.mapping.items():
            if pv == v:
                return w
        return v  # Identity

    def compose(self, other: "Permutation") -> "Permutation":
        """Compose permutations: return P ∘ Q (apply Q first, then P)."""
        result = {}
        all_vertices = set(self.mapping.keys()) | set(other.mapping.keys())

        for v in all_vertices:
            qv = other.apply(v)
            pqv = self.apply(qv)
            if pqv != v:
                result[v] = pqv

        return Permutation(result, self.num_vertices)

    def is_identity(self) -> bool:
        """Check if permutation is identity."""
        return len(self.mapping) == 0

    def non_identity_count(self) -> int:
        """Number of vertices where P(v) ≠ v."""
        return len(self.mapping)

    def to_dict(self) -> Dict:
        return {
            "mapping": self.mapping,
            "num_vertices": self.num_vertices
        }

    def fingerprint(self) -> str:
        # Sort mapping for determinism
        sorted_mapping = sorted(self.mapping.items())
        data = {"map": sorted_mapping, "n": self.num_vertices}
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]

    @classmethod
    def identity(cls, num_vertices: int) -> "Permutation":
        """Create identity permutation."""
        return cls({}, num_vertices)

    @classmethod
    def from_dict(cls, mapping: Dict[int, int], num_vertices: int) -> "Permutation":
        """Create permutation from mapping dictionary."""
        # Filter out identity entries
        non_identity = {v: pv for v, pv in mapping.items() if v != pv}
        return cls(non_identity, num_vertices)


class OccupancyBitset:
    """
    Efficient bitset representation of robot occupancy.

    ρ_t: V → {0,1}

    Uses Python's array module for efficient bit operations.
    For very large graphs, could use numpy or custom SIMD.
    """

    def __init__(self, num_vertices: int, occupied: Optional[Set[int]] = None):
        """
        Initialize occupancy bitset.

        Args:
            num_vertices: Total number of vertices
            occupied: Set of initially occupied vertices
        """
        self.num_vertices = num_vertices
        # Use array of unsigned bytes, 8 vertices per byte
        self.num_words = (num_vertices + 63) // 64
        self._data = array.array('Q', [0] * self.num_words)  # 64-bit words

        if occupied:
            for v in occupied:
                self.set(v)

    def set(self, v: int):
        """Mark vertex v as occupied."""
        if 0 <= v < self.num_vertices:
            word_idx = v // 64
            bit_idx = v % 64
            self._data[word_idx] |= (1 << bit_idx)

    def clear(self, v: int):
        """Mark vertex v as unoccupied."""
        if 0 <= v < self.num_vertices:
            word_idx = v // 64
            bit_idx = v % 64
            self._data[word_idx] &= ~(1 << bit_idx)

    def get(self, v: int) -> bool:
        """Check if vertex v is occupied."""
        if 0 <= v < self.num_vertices:
            word_idx = v // 64
            bit_idx = v % 64
            return bool(self._data[word_idx] & (1 << bit_idx))
        return False

    def __getitem__(self, v: int) -> bool:
        return self.get(v)

    def __setitem__(self, v: int, value: bool):
        if value:
            self.set(v)
        else:
            self.clear(v)

    def count(self) -> int:
        """Count number of occupied vertices (popcount)."""
        total = 0
        for word in self._data:
            # Brian Kernighan's algorithm
            while word:
                word &= word - 1
                total += 1
        return total

    def occupied_vertices(self) -> Set[int]:
        """Get set of all occupied vertices."""
        result = set()
        for v in range(self.num_vertices):
            if self.get(v):
                result.add(v)
        return result

    def copy(self) -> "OccupancyBitset":
        """Create a copy of this bitset."""
        new = OccupancyBitset(self.num_vertices)
        new._data = array.array('Q', self._data)
        return new

    def fingerprint(self) -> str:
        """Hash of occupancy state."""
        data_bytes = self._data.tobytes()
        return hashlib.sha256(data_bytes).hexdigest()[:16]

    def to_list(self) -> List[int]:
        """Get sorted list of occupied vertices."""
        return sorted(self.occupied_vertices())


class PermutationExecutor:
    """
    Executes permutations on occupancy state.

    This is the core of the transport fabric update:
    ρ_{t+1} = ρ_t ∘ P_t^{-1}

    In practical terms: new_occupancy[P(v)] = old_occupancy[v]
    """

    def __init__(self, num_vertices: int):
        self.num_vertices = num_vertices

    def apply(self, occupancy: OccupancyBitset,
              permutation: Permutation) -> OccupancyBitset:
        """
        Apply permutation to occupancy.

        ρ_{t+1} = ρ_t ∘ P_t^{-1}

        Equivalent to: for each v, new[P(v)] = old[v]

        Args:
            occupancy: Current occupancy state
            permutation: Permutation to apply

        Returns:
            New occupancy state after permutation
        """
        new_occupancy = OccupancyBitset(self.num_vertices)

        # For non-identity mappings, move robots
        for v, pv in permutation.mapping.items():
            if occupancy.get(v):
                new_occupancy.set(pv)

        # For identity positions, keep robots in place
        for v in range(self.num_vertices):
            if v not in permutation.mapping and occupancy.get(v):
                new_occupancy.set(v)

        return new_occupancy

    def apply_inplace(self, occupancy: OccupancyBitset,
                      permutation: Permutation) -> None:
        """
        Apply permutation in-place (more memory efficient but requires care).

        For safety, this actually creates a new state then copies back.
        True in-place would require careful ordering.
        """
        new_state = self.apply(occupancy, permutation)
        occupancy._data = new_state._data

    def verify_bijection(self, permutation: Permutation) -> bool:
        """
        Verify permutation is a valid bijection.

        Checks that no two sources map to the same target.
        """
        targets = set(permutation.mapping.values())
        return len(targets) == len(permutation.mapping)

    def verify_conservation(self, old: OccupancyBitset,
                           new: OccupancyBitset) -> bool:
        """
        Verify robot count is conserved after permutation.

        If P is a bijection, this should always hold.
        """
        return old.count() == new.count()


@dataclass
class TickState:
    """
    Complete state at a single tick.

    Contains:
    - Occupancy bitset
    - Junction mode choices
    - Applied permutation
    """
    tick: int
    occupancy: OccupancyBitset
    junction_modes: Dict[int, int]  # junction_id → mode_id
    permutation: Permutation
    occupancy_hash: str = ""

    def __post_init__(self):
        if not self.occupancy_hash:
            self.occupancy_hash = self.occupancy.fingerprint()

    def to_dict(self) -> Dict:
        return {
            "tick": self.tick,
            "robot_count": self.occupancy.count(),
            "junction_modes": self.junction_modes,
            "permutation_size": self.permutation.non_identity_count(),
            "occupancy_hash": self.occupancy_hash
        }


class FabricExecutor:
    """
    High-level executor for transport fabric operations.

    CORRECTED FORMULATION:
    The tick IS a permutation. Junction modes are derived labels.

    The executor:
    1. Takes edge preferences (from controller)
    2. Solves for feasible permutation via matching
    3. Applies permutation to occupancy
    4. Returns result (SUCCESS with permutation, or UNSAT with Hall witness)
    """

    def __init__(self, fabric: "TransportFabric"):
        """
        Initialize fabric executor.

        Args:
            fabric: Compiled transport fabric
        """
        from .fabric_compiler import TransportFabric
        self.fabric = fabric
        self.num_vertices = fabric.warehouse.num_vertices
        self.executor = PermutationExecutor(self.num_vertices)

    def execute_tick(self, occupancy: OccupancyBitset,
                     preferences: Optional[Dict[Tuple[int, int], int]] = None
                     ) -> Tuple[OccupancyBitset, Permutation, "CompletionResult"]:
        """
        Execute one tick of the transport fabric.

        CORRECTED: The tick IS a permutation found by matching.

        1. Solve for permutation via bipartite matching
        2. If feasible: apply permutation to occupancy
        3. If UNSAT: return Hall witness

        Args:
            occupancy: Current occupancy state
            preferences: Edge costs {(src, dst): cost} for matching solver

        Returns:
            (new_occupancy, permutation, completion_result)
            If UNSAT, occupancy is unchanged and permutation is identity.
        """
        from .permutation_completion import CompletionStatus

        # Build tick permutation via matching (THE tick operator)
        result = self.fabric.build_tick_permutation(preferences)

        if result.status != CompletionStatus.SUCCESS:
            # UNSAT - return unchanged with Hall witness
            return (
                occupancy.copy(),
                Permutation.identity(self.num_vertices),
                result
            )

        # Build Permutation object
        permutation = Permutation.from_dict(result.permutation, self.num_vertices)

        # Verify bijection (should always pass if solver is correct)
        if not self.executor.verify_bijection(permutation):
            raise ValueError("Invalid permutation: not a bijection")

        # Apply permutation
        new_occupancy = self.executor.apply(occupancy, permutation)

        # Verify conservation
        if not self.executor.verify_conservation(occupancy, new_occupancy):
            raise ValueError("Robot count changed after permutation")

        return new_occupancy, permutation, result

    def execute_tick_legacy(self, occupancy: OccupancyBitset,
                            junction_modes: Dict[int, int]) -> Tuple[OccupancyBitset, Permutation]:
        """
        DEPRECATED: Legacy interface for backward compatibility.

        Use execute_tick(occupancy, preferences) instead.
        """
        import warnings
        warnings.warn(
            "execute_tick_legacy is deprecated. Use execute_tick(occupancy, preferences) instead.",
            DeprecationWarning
        )

        # Convert modes to preferences
        preferences: Dict[Tuple[int, int], int] = {}
        for junction_id, mode_id in junction_modes.items():
            junction = self.fabric.get_junction(junction_id)
            junction_perm = junction.get_permutation(mode_id)
            if junction_perm.mapping:
                for (src, dst) in junction_perm.mapping:
                    preferences[(src, dst)] = 0

        new_occ, perm, result = self.execute_tick(occupancy, preferences)
        return new_occ, perm

    def create_initial_state(self, robot_positions: Set[int]) -> OccupancyBitset:
        """Create initial occupancy state from robot positions."""
        return OccupancyBitset(self.num_vertices, robot_positions)

    def run_ticks(self, initial_occupancy: OccupancyBitset,
                  preference_sequence: List[Optional[Dict[Tuple[int, int], int]]],
                  track_history: bool = False) -> List[TickState]:
        """
        Run multiple ticks with specified preference sequence.

        CORRECTED: Takes preferences (edge costs), not junction modes.
        The matching solver finds optimal feasible permutations.

        Args:
            initial_occupancy: Starting occupancy
            preference_sequence: List of preference dicts, one per tick
            track_history: If True, return all intermediate states

        Returns:
            List of TickState (just final if not tracking, all if tracking)
        """
        states = []
        current = initial_occupancy.copy()

        for t, preferences in enumerate(preference_sequence):
            new_occupancy, permutation, result = self.execute_tick(current, preferences)

            # Derive junction modes from permutation (for logging/UI)
            derived_modes = self._derive_junction_modes(permutation)

            state = TickState(
                tick=t,
                occupancy=new_occupancy if track_history else current,
                junction_modes=derived_modes,
                permutation=permutation
            )

            if track_history:
                states.append(state)

            current = new_occupancy

        # Final state
        if not track_history:
            states.append(TickState(
                tick=len(preference_sequence),
                occupancy=current,
                junction_modes={},
                permutation=Permutation.identity(self.num_vertices)
            ))

        return states

    def _derive_junction_modes(self, permutation: Permutation) -> Dict[int, int]:
        """
        Derive junction modes from a permutation (for logging/UI).

        This is π_modes(P_t) - projecting the permutation to mode coordinates.
        """
        derived = {}
        for junction in self.fabric.junctions:
            # Find which mode best matches the permutation
            best_mode = 0
            best_match = 0

            for perm_mode in junction.permutations:
                match_count = 0
                for (src, dst) in perm_mode.mapping:
                    if permutation.apply(src) == dst:
                        match_count += 1
                if match_count > best_match:
                    best_match = match_count
                    best_mode = perm_mode.mode_id

            derived[junction.junction_id] = best_mode

        return derived


class FastFabricExecutor:
    """
    Fast executor using incremental matching.

    SCALING SOLUTION:
    Instead of solving global matching each tick (O(V²)),
    maintain a base cycle cover and apply local augmentations (O(J·m)).

    OPTIMIZATION: Tracks occupied vertices incrementally to avoid
    O(V) scan each tick. This is critical for 30+ Hz at 10k scale.

    This is the production 10k executor.
    """

    def __init__(self, fabric: "TransportFabric"):
        """
        Initialize fast executor with incremental matcher.

        Args:
            fabric: Compiled transport fabric
        """
        from .fabric_compiler import TransportFabric
        from .incremental_matching import IncrementalMatcher, build_incremental_matcher

        self.fabric = fabric
        self.num_vertices = fabric.warehouse.num_vertices
        self.executor = PermutationExecutor(self.num_vertices)

        # Build incremental matcher
        self.matcher = build_incremental_matcher(fabric)

        # Tracked occupied set for incremental updates (O(k) instead of O(V))
        self._tracked_occupied: Optional[Set[int]] = None

    def execute_tick(self, occupancy: OccupancyBitset,
                     weights,
                     occupied_set: Optional[Set[int]] = None) -> Tuple[OccupancyBitset, Permutation]:
        """
        Execute one tick using incremental matching.

        O(J·m) per tick - scalable to 10k+.

        Args:
            occupancy: Current occupancy state
            weights: Edge weights - either Dict[Tuple[int, int], float] or
                    a WeightFunction with .get() method for lazy evaluation
            occupied_set: Optional pre-computed occupied set (avoids O(V) scan)

        Returns:
            (new_occupancy, permutation)
        """
        # OPTIMIZATION: Use provided occupied_set or tracked set to avoid O(V) scan
        if occupied_set is not None:
            current_occupied = occupied_set
        elif self._tracked_occupied is not None:
            current_occupied = self._tracked_occupied
        else:
            # First call - initialize tracking
            current_occupied = occupancy.occupied_vertices()
            self._tracked_occupied = current_occupied

        # Update matching with local augmentations (only in occupied neighborhoods)
        perm_dict = self.matcher.update_tick(weights, occupied_vertices=current_occupied)

        # Build Permutation object
        permutation = Permutation.from_dict(perm_dict, self.num_vertices)

        # Verify (in debug mode)
        if not self.matcher.verify_matching():
            raise ValueError("Incremental matching produced invalid permutation")

        # Apply permutation
        new_occupancy = self.executor.apply(occupancy, permutation)

        # Update tracked occupied set incrementally: new = {P(v) for v in old}
        new_occupied = {permutation.apply(v) for v in current_occupied}
        self._tracked_occupied = new_occupied

        return new_occupancy, permutation

    def create_initial_state(self, robot_positions: Set[int]) -> OccupancyBitset:
        """Create initial occupancy state from robot positions."""
        return OccupancyBitset(self.num_vertices, robot_positions)

    def reset_matching(self):
        """Reset to base cycle cover (safe fallback)."""
        self.matcher.reset_to_base()
