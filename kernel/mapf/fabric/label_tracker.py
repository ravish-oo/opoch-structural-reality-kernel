"""
Label Tracker for Transport Fabric

Recovers labeled robot trajectories from unlabeled permutation history.

Mathematical Foundation:
-----------------------
Even though planning is unlabeled, we recover labels via lineage:
- Each occupied vertex at time t has unique predecessor under P_t
- Each token has unique lineage across time

If robot i is at vertex v at time 0, its position at time t is:
  p_i(t) = P_{t-1} ∘ ... ∘ P_0(v)

This gives explicit per-robot paths without ever solving labeled MAPF.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, FrozenSet
import hashlib
import json

from .permutation_executor import Permutation, OccupancyBitset, TickState


@dataclass
class RobotLineage:
    """
    Complete trajectory (lineage) of a single robot.

    The lineage is recovered by tracing forward through permutations:
    position[t+1] = P_t(position[t])
    """
    robot_id: int
    start_vertex: int
    positions: List[int]  # positions[t] = vertex at time t
    moves: List[Tuple[int, int]]  # moves[t] = (from, to) at time t

    def __post_init__(self):
        if not self.moves and len(self.positions) > 1:
            self.moves = [(self.positions[t], self.positions[t+1])
                         for t in range(len(self.positions) - 1)]

    def position_at(self, t: int) -> int:
        """Get position at time t (with goal-hold)."""
        if t < len(self.positions):
            return self.positions[t]
        return self.positions[-1]  # Hold at final position

    def move_at(self, t: int) -> Tuple[int, int]:
        """Get move at time t."""
        if t < len(self.moves):
            return self.moves[t]
        final = self.positions[-1]
        return (final, final)  # Wait at final

    def is_waiting_at(self, t: int) -> bool:
        """Check if robot is waiting at time t."""
        move = self.move_at(t)
        return move[0] == move[1]

    def total_moves(self) -> int:
        """Count non-wait moves."""
        return sum(1 for m in self.moves if m[0] != m[1])

    def path_length(self) -> int:
        """Total path length (number of timesteps)."""
        return len(self.positions) - 1

    def to_dict(self) -> Dict:
        return {
            "robot_id": self.robot_id,
            "start": self.start_vertex,
            "end": self.positions[-1] if self.positions else None,
            "positions": self.positions,
            "total_moves": self.total_moves(),
            "path_length": self.path_length()
        }

    def fingerprint(self) -> str:
        data = {"id": self.robot_id, "pos": self.positions}
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]


@dataclass
class LabeledSolution:
    """
    Complete labeled solution recovered from unlabeled permutations.

    Contains:
    - All robot lineages
    - Verification that paths are collision-free
    - Statistics
    """
    lineages: List[RobotLineage]
    horizon: int
    num_robots: int
    total_moves: int
    makespan: int  # max path length

    def get_robot(self, robot_id: int) -> Optional[RobotLineage]:
        """Get lineage for specific robot."""
        for lineage in self.lineages:
            if lineage.robot_id == robot_id:
                return lineage
        return None

    def positions_at_time(self, t: int) -> Dict[int, int]:
        """Get {robot_id: position} at time t."""
        return {l.robot_id: l.position_at(t) for l in self.lineages}

    def to_paths_dict(self) -> Dict[int, List[int]]:
        """Convert to {robot_id: [positions]} format."""
        return {l.robot_id: l.positions for l in self.lineages}

    def to_dict(self) -> Dict:
        return {
            "num_robots": self.num_robots,
            "horizon": self.horizon,
            "total_moves": self.total_moves,
            "makespan": self.makespan,
            "lineages": [l.to_dict() for l in self.lineages]
        }

    def fingerprint(self) -> str:
        # Sort lineages by robot_id for determinism
        sorted_fps = sorted([l.fingerprint() for l in self.lineages])
        data = {
            "n": self.num_robots,
            "h": self.horizon,
            "lineage_fps": sorted_fps
        }
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]


class LabelTracker:
    """
    Tracks robot labels through permutation history.

    Given:
    - Initial positions {robot_id: vertex}
    - Sequence of permutations [P_0, P_1, ..., P_{T-1}]

    Produces:
    - Complete lineage for each robot
    """

    def __init__(self, initial_positions: Dict[int, int]):
        """
        Initialize label tracker.

        Args:
            initial_positions: {robot_id: start_vertex}
        """
        self.initial_positions = dict(initial_positions)
        self.current_positions = dict(initial_positions)
        self.history: List[Dict[int, int]] = [dict(initial_positions)]
        self.permutation_history: List[Permutation] = []

    def apply_permutation(self, permutation: Permutation):
        """
        Apply permutation and track new positions.

        For each robot i at position v:
          new_position[i] = P(v)
        """
        new_positions = {}

        for robot_id, current_pos in self.current_positions.items():
            new_pos = permutation.apply(current_pos)
            new_positions[robot_id] = new_pos

        self.current_positions = new_positions
        self.history.append(dict(new_positions))
        self.permutation_history.append(permutation)

    def apply_permutations(self, permutations: List[Permutation]):
        """Apply sequence of permutations."""
        for perm in permutations:
            self.apply_permutation(perm)

    def get_lineage(self, robot_id: int) -> RobotLineage:
        """Get complete lineage for a robot."""
        positions = [h[robot_id] for h in self.history]

        return RobotLineage(
            robot_id=robot_id,
            start_vertex=positions[0],
            positions=positions,
            moves=[]
        )

    def get_all_lineages(self) -> List[RobotLineage]:
        """Get lineages for all robots."""
        return [self.get_lineage(rid) for rid in sorted(self.initial_positions.keys())]

    def get_solution(self) -> LabeledSolution:
        """Get complete labeled solution."""
        lineages = self.get_all_lineages()

        total_moves = sum(l.total_moves() for l in lineages)
        makespan = max(l.path_length() for l in lineages) if lineages else 0

        return LabeledSolution(
            lineages=lineages,
            horizon=len(self.history) - 1,
            num_robots=len(self.initial_positions),
            total_moves=total_moves,
            makespan=makespan
        )

    def verify_no_collisions(self) -> Dict:
        """
        Verify no two robots occupy same vertex at any time.

        This should always pass if permutations are bijections.
        """
        errors = []

        for t, positions in enumerate(self.history):
            # Check for duplicate positions
            pos_to_robots = {}
            for robot_id, pos in positions.items():
                if pos in pos_to_robots:
                    errors.append({
                        "type": "VERTEX_COLLISION",
                        "time": t,
                        "vertex": pos,
                        "robots": [pos_to_robots[pos], robot_id]
                    })
                else:
                    pos_to_robots[pos] = robot_id

        return {
            "valid": len(errors) == 0,
            "num_collisions": len(errors),
            "errors": errors[:10]
        }

    def verify_no_swaps(self) -> Dict:
        """
        Verify no edge swaps between consecutive timesteps.

        A swap occurs if robot i at u goes to v while robot j at v goes to u.
        """
        errors = []

        for t in range(len(self.history) - 1):
            old_pos = self.history[t]
            new_pos = self.history[t + 1]

            # Get moves: robot_id → (from, to)
            moves = {rid: (old_pos[rid], new_pos[rid]) for rid in old_pos}

            # Check for swaps
            robot_ids = list(moves.keys())
            for i, r1 in enumerate(robot_ids):
                u1, v1 = moves[r1]
                for r2 in robot_ids[i+1:]:
                    u2, v2 = moves[r2]
                    # Check if swap: r1 goes u→v, r2 goes v→u
                    if u1 == v2 and v1 == u2 and u1 != v1:
                        errors.append({
                            "type": "EDGE_SWAP",
                            "time": t,
                            "robots": [r1, r2],
                            "moves": [(u1, v1), (u2, v2)]
                        })

        return {
            "valid": len(errors) == 0,
            "num_swaps": len(errors),
            "errors": errors[:10]
        }


def extract_robot_paths(initial_positions: Dict[int, int],
                        permutations: List[Permutation]) -> LabeledSolution:
    """
    Convenience function to extract robot paths from permutation sequence.

    Args:
        initial_positions: {robot_id: start_vertex}
        permutations: List of permutations [P_0, ..., P_{T-1}]

    Returns:
        LabeledSolution with all robot lineages
    """
    tracker = LabelTracker(initial_positions)
    tracker.apply_permutations(permutations)
    return tracker.get_solution()


def verify_labeled_solution(solution: LabeledSolution,
                           edges: Set[Tuple[int, int]]) -> Dict:
    """
    Verify a labeled solution satisfies all constraints.

    Checks:
    - V3: All moves are valid (edge or wait)
    - V4: No vertex collisions
    - V5: No edge swaps
    """
    errors = []

    # V3: Valid moves
    for lineage in solution.lineages:
        for t, (u, v) in enumerate(lineage.moves):
            if u != v and (u, v) not in edges:
                errors.append({
                    "type": "V3_INVALID_MOVE",
                    "robot": lineage.robot_id,
                    "time": t,
                    "move": (u, v)
                })

    # V4: Vertex collisions
    for t in range(solution.horizon + 1):
        positions = solution.positions_at_time(t)
        pos_to_robots = {}
        for rid, pos in positions.items():
            if pos in pos_to_robots:
                errors.append({
                    "type": "V4_VERTEX_COLLISION",
                    "time": t,
                    "vertex": pos,
                    "robots": [pos_to_robots[pos], rid]
                })
            else:
                pos_to_robots[pos] = rid

    # V5: Edge swaps
    for t in range(solution.horizon):
        pos_t = solution.positions_at_time(t)
        pos_t1 = solution.positions_at_time(t + 1)

        moves = {rid: (pos_t[rid], pos_t1[rid]) for rid in pos_t}

        rids = list(moves.keys())
        for i, r1 in enumerate(rids):
            u1, v1 = moves[r1]
            for r2 in rids[i+1:]:
                u2, v2 = moves[r2]
                if u1 == v2 and v1 == u2 and u1 != v1:
                    errors.append({
                        "type": "V5_EDGE_SWAP",
                        "time": t,
                        "robots": [r1, r2],
                        "edge": (u1, v1)
                    })

    return {
        "valid": len(errors) == 0,
        "num_errors": len(errors),
        "errors": errors[:20]
    }
