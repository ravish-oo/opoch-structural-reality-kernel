"""
Junction Gadgets for Transport Fabric

A junction gadget is a small local structure that provides a finite set
of safe permutations for routing robots through an intersection.

Mathematical Foundation:
------------------------
Each junction J has a vertex set V_J and a permutation set P_J:
  P_J = {P_J^(1), ..., P_J^(m)}

Each P_J^(i) is:
1. A bijection on V_J (extended by identity outside)
2. Legal: P(v) = v or (v → P(v)) ∈ E
3. Swap-free: no adjacent 2-cycles (P(u) = v ∧ P(v) = u with u ≠ v)

The controller picks one permutation per junction per tick.

Common Junction Types:
---------------------
- Cross junction (4-way): 2-5 safe permutations
- T-junction (3-way): 2-3 safe permutations
- Merge junction: 2 inputs → 1 output
- Split junction: 1 input → 2 outputs
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, FrozenSet
from enum import Enum
import hashlib
import json


class JunctionType(Enum):
    """Type of junction gadget."""
    CROSS = "CROSS"       # 4-way intersection
    T_JUNCTION = "T"      # 3-way intersection
    MERGE = "MERGE"       # 2 → 1
    SPLIT = "SPLIT"       # 1 → 2
    BUFFER = "BUFFER"     # Waiting area
    CUSTOM = "CUSTOM"     # User-defined


@dataclass(frozen=True)
class JunctionPermutation:
    """
    A single safe permutation for a junction.

    Represented as a mapping: vertex → successor vertex.
    Vertices not in the mapping stay in place (identity).

    Properties:
    - mode_id: index in junction's permutation set
    - mapping: {v: P(v)} for non-identity moves
    - throughput: expected robots moved per tick with this mode
    - description: human-readable description
    """
    mode_id: int
    mapping: FrozenSet[Tuple[int, int]]  # (from, to) pairs
    throughput: float = 1.0
    description: str = ""

    def apply(self, v: int) -> int:
        """Apply permutation to vertex v."""
        for (src, dst) in self.mapping:
            if src == v:
                return dst
        return v  # Identity

    def inverse_apply(self, v: int) -> int:
        """Apply inverse permutation to vertex v."""
        for (src, dst) in self.mapping:
            if dst == v:
                return src
        return v  # Identity

    def is_identity_at(self, v: int) -> bool:
        """Check if permutation is identity at v."""
        return self.apply(v) == v

    def non_identity_vertices(self) -> Set[int]:
        """Get vertices where permutation is not identity."""
        return {src for (src, _) in self.mapping}

    def moved_vertices(self) -> Set[int]:
        """Get all vertices involved in non-identity moves."""
        result = set()
        for (src, dst) in self.mapping:
            result.add(src)
            result.add(dst)
        return result

    def to_dict(self) -> Dict:
        return {
            "mode_id": self.mode_id,
            "mapping": list(self.mapping),
            "throughput": self.throughput,
            "description": self.description
        }

    def fingerprint(self) -> str:
        data = {"mode": self.mode_id, "map": sorted(list(self.mapping))}
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:8]


@dataclass
class JunctionGadget:
    """
    Complete junction gadget with all safe permutations.

    A junction gadget encapsulates:
    - The vertex neighborhood
    - The set of legal edges within/through the junction
    - All safe permutation modes
    - Validation that each mode is bijective and swap-free
    """
    junction_id: int
    junction_type: JunctionType
    vertices: FrozenSet[int]
    edges: FrozenSet[Tuple[int, int]]  # Legal edges within junction
    permutations: List[JunctionPermutation]

    # Optional: incoming/outgoing lane connections
    incoming: Dict[int, int] = field(default_factory=dict)  # lane_id → entry vertex
    outgoing: Dict[int, int] = field(default_factory=dict)  # lane_id → exit vertex

    def __post_init__(self):
        """Validate all permutations."""
        for perm in self.permutations:
            self._validate_permutation(perm)

    def _validate_permutation(self, perm: JunctionPermutation):
        """
        Validate a permutation is safe.

        Checks:
        1. Bijection: each source maps to unique target
        2. Legal: all moves follow edges or are waits
        3. Swap-free: no adjacent 2-cycles
        """
        mapping_dict = {src: dst for (src, dst) in perm.mapping}

        # Check bijection (no two sources map to same target)
        targets = list(mapping_dict.values())
        if len(targets) != len(set(targets)):
            raise ValueError(f"Permutation {perm.mode_id} is not a bijection: duplicate targets")

        # Check legality (edge or wait)
        for src, dst in mapping_dict.items():
            if src != dst and (src, dst) not in self.edges:
                raise ValueError(f"Permutation {perm.mode_id} has illegal move {src}→{dst}")

        # Check swap-free
        for src, dst in mapping_dict.items():
            if src != dst:
                # Check if reverse exists
                if dst in mapping_dict and mapping_dict[dst] == src:
                    raise ValueError(f"Permutation {perm.mode_id} has swap: {src}↔{dst}")

    def num_modes(self) -> int:
        return len(self.permutations)

    def get_permutation(self, mode_id: int) -> JunctionPermutation:
        """Get permutation by mode ID."""
        for perm in self.permutations:
            if perm.mode_id == mode_id:
                return perm
        raise ValueError(f"Mode {mode_id} not found in junction {self.junction_id}")

    def identity_mode(self) -> Optional[JunctionPermutation]:
        """Get the identity (wait-all) mode if it exists."""
        for perm in self.permutations:
            if len(perm.mapping) == 0:
                return perm
        return None

    def fingerprint(self) -> str:
        data = {
            "id": self.junction_id,
            "type": self.junction_type.value,
            "vertices": sorted(list(self.vertices)),
            "num_modes": self.num_modes()
        }
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]


class JunctionGadgetLibrary:
    """
    Library of pre-built junction gadgets for common patterns.

    This provides validated, safe junction gadgets that can be
    instantiated at specific vertices in the warehouse.
    """

    @staticmethod
    def create_identity_gadget(junction_id: int, vertices: FrozenSet[int]) -> JunctionGadget:
        """
        Create a trivial gadget where all robots wait.

        Useful for buffers or temporarily blocked junctions.
        """
        identity_perm = JunctionPermutation(
            mode_id=0,
            mapping=frozenset(),
            throughput=0.0,
            description="All wait"
        )
        return JunctionGadget(
            junction_id=junction_id,
            junction_type=JunctionType.BUFFER,
            vertices=vertices,
            edges=frozenset(),
            permutations=[identity_perm]
        )

    @staticmethod
    def create_straight_gadget(junction_id: int, v1: int, v2: int) -> JunctionGadget:
        """
        Create a simple straight-through gadget (2 vertices).

        Modes:
        0. Both wait
        1. v1 → v2 (if edge exists)
        """
        vertices = frozenset([v1, v2])
        edges = frozenset([(v1, v2)])

        perms = [
            JunctionPermutation(
                mode_id=0,
                mapping=frozenset(),
                throughput=0.0,
                description="Wait"
            ),
            JunctionPermutation(
                mode_id=1,
                mapping=frozenset([(v1, v2)]),
                throughput=1.0,
                description=f"{v1}→{v2}"
            )
        ]

        return JunctionGadget(
            junction_id=junction_id,
            junction_type=JunctionType.CUSTOM,
            vertices=vertices,
            edges=edges,
            permutations=perms
        )


def create_cross_junction(junction_id: int,
                          center: int,
                          north: int, south: int,
                          east: int, west: int,
                          edges: Set[Tuple[int, int]]) -> JunctionGadget:
    """
    Create a 4-way cross junction gadget.

    Layout:
           N
           |
        W--C--E
           |
           S

    Safe permutation modes depend on which edges exist.
    Typical modes:
    0. All wait
    1. N→S, S→N (straight vertical) - only if swap-free
    2. E→W, W→E (straight horizontal) - only if swap-free
    3. N→E, E→S, S→W, W→N (rotate clockwise)
    4. N→W, W→S, S→E, E→N (rotate counter-clockwise)

    Note: Straight-through modes with swaps are NOT included.
    """
    vertices = frozenset([center, north, south, east, west])
    edge_set = frozenset(edges) & frozenset([
        (north, center), (center, north),
        (south, center), (center, south),
        (east, center), (center, east),
        (west, center), (center, west),
        (north, south), (south, north),
        (east, west), (west, east)
    ])

    perms = []

    # Mode 0: All wait
    perms.append(JunctionPermutation(
        mode_id=0,
        mapping=frozenset(),
        throughput=0.0,
        description="All wait"
    ))

    mode_id = 1

    # Mode: N→C→S (north to south through center)
    if (north, center) in edge_set and (center, south) in edge_set:
        # This is safe (no swap since S doesn't go to N simultaneously in this mode)
        perms.append(JunctionPermutation(
            mode_id=mode_id,
            mapping=frozenset([(north, center), (center, south)]),
            throughput=1.0,
            description="N→C→S"
        ))
        mode_id += 1

    # Mode: S→C→N
    if (south, center) in edge_set and (center, north) in edge_set:
        perms.append(JunctionPermutation(
            mode_id=mode_id,
            mapping=frozenset([(south, center), (center, north)]),
            throughput=1.0,
            description="S→C→N"
        ))
        mode_id += 1

    # Mode: E→C→W
    if (east, center) in edge_set and (center, west) in edge_set:
        perms.append(JunctionPermutation(
            mode_id=mode_id,
            mapping=frozenset([(east, center), (center, west)]),
            throughput=1.0,
            description="E→C→W"
        ))
        mode_id += 1

    # Mode: W→C→E
    if (west, center) in edge_set and (center, east) in edge_set:
        perms.append(JunctionPermutation(
            mode_id=mode_id,
            mapping=frozenset([(west, center), (center, east)]),
            throughput=1.0,
            description="W→C→E"
        ))
        mode_id += 1

    # Mode: Rotate clockwise (N→E, E→S, S→W, W→N via center)
    # This is a safe 4-cycle, no swaps
    if all((v1, center) in edge_set and (center, v2) in edge_set
           for v1, v2 in [(north, east), (east, south), (south, west), (west, north)]):
        perms.append(JunctionPermutation(
            mode_id=mode_id,
            mapping=frozenset([
                (north, center),  # N enters
                (center, east),   # Someone exits to E
            ]),
            throughput=1.0,
            description="N→E (turn right)"
        ))
        mode_id += 1

    return JunctionGadget(
        junction_id=junction_id,
        junction_type=JunctionType.CROSS,
        vertices=vertices,
        edges=edge_set,
        permutations=perms
    )


def create_merge_junction(junction_id: int,
                          output: int,
                          input1: int, input2: int,
                          edges: Set[Tuple[int, int]]) -> JunctionGadget:
    """
    Create a merge junction (2 inputs → 1 output).

    Only one input can move to output per tick (capacity constraint).

    Modes:
    0. All wait
    1. input1 → output
    2. input2 → output
    """
    vertices = frozenset([output, input1, input2])
    edge_set = frozenset(edges) & frozenset([
        (input1, output), (input2, output)
    ])

    perms = [
        JunctionPermutation(
            mode_id=0,
            mapping=frozenset(),
            throughput=0.0,
            description="All wait"
        )
    ]

    mode_id = 1
    if (input1, output) in edge_set:
        perms.append(JunctionPermutation(
            mode_id=mode_id,
            mapping=frozenset([(input1, output)]),
            throughput=1.0,
            description=f"{input1}→{output}"
        ))
        mode_id += 1

    if (input2, output) in edge_set:
        perms.append(JunctionPermutation(
            mode_id=mode_id,
            mapping=frozenset([(input2, output)]),
            throughput=1.0,
            description=f"{input2}→{output}"
        ))

    return JunctionGadget(
        junction_id=junction_id,
        junction_type=JunctionType.MERGE,
        vertices=vertices,
        edges=edge_set,
        permutations=perms
    )


def create_split_junction(junction_id: int,
                          input_v: int,
                          output1: int, output2: int,
                          edges: Set[Tuple[int, int]]) -> JunctionGadget:
    """
    Create a split junction (1 input → 2 outputs).

    The input can go to either output (routing decision).

    Modes:
    0. Wait
    1. input → output1
    2. input → output2
    """
    vertices = frozenset([input_v, output1, output2])
    edge_set = frozenset(edges) & frozenset([
        (input_v, output1), (input_v, output2)
    ])

    perms = [
        JunctionPermutation(
            mode_id=0,
            mapping=frozenset(),
            throughput=0.0,
            description="Wait"
        )
    ]

    mode_id = 1
    if (input_v, output1) in edge_set:
        perms.append(JunctionPermutation(
            mode_id=mode_id,
            mapping=frozenset([(input_v, output1)]),
            throughput=1.0,
            description=f"{input_v}→{output1}"
        ))
        mode_id += 1

    if (input_v, output2) in edge_set:
        perms.append(JunctionPermutation(
            mode_id=mode_id,
            mapping=frozenset([(input_v, output2)]),
            throughput=1.0,
            description=f"{input_v}→{output2}"
        ))

    return JunctionGadget(
        junction_id=junction_id,
        junction_type=JunctionType.SPLIT,
        vertices=vertices,
        edges=edge_set,
        permutations=perms
    )
