"""
Deterministic Path Decomposition for Unlabeled MAPF

Mathematical Foundation:
Given integral flow of value k on time-expanded network,
decompose into exactly k unit-flow paths from starts to goals.

Deterministic Extraction Rule:
1. Always pick lexicographically smallest available start node first
2. Follow smallest outgoing edge with flow=1 at each time
3. Remove that unit flow, repeat

This produces a CANONICAL multiset of paths (order-free by hash).

Robot Assignment:
- Physical robots labeled 1..k with positions s_i
- Map extracted path starting at s_i to robot i
- Tie-breaker: lexicographic on robot ID if starts coincide
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, FrozenSet
from collections import defaultdict
import hashlib
import json

from .time_expanded_graph import (
    TimeExpandedGraph,
    TimeExpandedNode,
    TimeExpandedEdge,
    EdgeType,
)
from .flow_solver import ProductionFlowResult, ProductionFlowStatus


@dataclass
class ExtractedPath:
    """
    Single path extracted from flow decomposition.

    A path is a sequence of vertices v_0, v_1, ..., v_T
    where v_0 is a start and v_T is a goal.
    """
    path_id: int
    vertices: List[int]  # v_0, v_1, ..., v_T
    start_vertex: int
    goal_vertex: int
    makespan: int  # Length of path

    def __post_init__(self):
        """Validate path structure."""
        assert len(self.vertices) > 0
        assert self.vertices[0] == self.start_vertex
        assert self.vertices[-1] == self.goal_vertex
        self.makespan = len(self.vertices) - 1

    def to_dict(self) -> Dict:
        return {
            "path_id": self.path_id,
            "vertices": self.vertices,
            "start": self.start_vertex,
            "goal": self.goal_vertex,
            "makespan": self.makespan
        }

    def fingerprint(self) -> str:
        """Canonical fingerprint of path."""
        data = self.to_dict()
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]

    def get_position_at_time(self, t: int) -> int:
        """Get vertex at time t (with goal-hold)."""
        if t < len(self.vertices):
            return self.vertices[t]
        else:
            # Goal-hold: stay at goal after reaching it
            return self.goal_vertex

    def get_move_at_time(self, t: int) -> Tuple[int, int]:
        """Get (from_vertex, to_vertex) at time t."""
        if t < len(self.vertices) - 1:
            return (self.vertices[t], self.vertices[t + 1])
        else:
            # Wait at goal
            return (self.goal_vertex, self.goal_vertex)


@dataclass
class RobotAssignment:
    """
    Assignment of extracted paths to physical robots.

    Robot i gets the path whose start matches robot i's position.
    """
    robot_id: int
    path: ExtractedPath
    physical_start: int  # Robot's actual starting position

    def to_dict(self) -> Dict:
        return {
            "robot_id": self.robot_id,
            "physical_start": self.physical_start,
            "path": self.path.to_dict()
        }


@dataclass
class PathDecompositionResult:
    """
    Complete result of path decomposition.

    Contains:
    - extracted_paths: List of paths (canonical order)
    - robot_assignments: Mapping to physical robots
    - statistics: Decomposition metrics
    """
    extracted_paths: List[ExtractedPath]
    robot_assignments: List[RobotAssignment]
    total_makespan: int  # max path length
    sum_of_costs: int    # sum of path lengths
    statistics: Dict = field(default_factory=dict)

    @property
    def num_paths(self) -> int:
        """Number of extracted paths."""
        return len(self.extracted_paths)

    @property
    def makespan(self) -> int:
        """Alias for total_makespan."""
        return self.total_makespan

    def to_dict(self) -> Dict:
        return {
            "num_paths": len(self.extracted_paths),
            "total_makespan": self.total_makespan,
            "sum_of_costs": self.sum_of_costs,
            "paths": [p.to_dict() for p in self.extracted_paths],
            "assignments": [a.to_dict() for a in self.robot_assignments]
        }

    def fingerprint(self) -> str:
        """Canonical fingerprint of decomposition."""
        # Sort paths by fingerprint for canonical ordering
        path_fps = sorted([p.fingerprint() for p in self.extracted_paths])
        data = {
            "num_paths": len(self.extracted_paths),
            "total_makespan": self.total_makespan,
            "sum_of_costs": self.sum_of_costs,
            "path_fingerprints": path_fps
        }
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]


class PathDecomposer:
    """
    Deterministic path decomposition from integral flow.

    Algorithm:
    1. Build residual flow graph from flow assignment
    2. For each start (in lexicographic order):
       a. Trace path following positive flow edges
       b. At each step, choose lexicographically smallest edge with flow
       c. Decrement flow along path
       d. Record extracted path
    3. Assign paths to robots based on start positions
    """

    def __init__(self, graph: TimeExpandedGraph):
        self.graph = graph
        self.horizon = graph.horizon

    def decompose(self, flow_result,
                  robot_positions: Optional[Dict[int, int]] = None
                  ) -> PathDecompositionResult:
        """
        Decompose flow into paths and assign to robots.

        Args:
            flow_result: Result from flow solver (FlowResult or ORToolsFlowResult)
            robot_positions: {robot_id: vertex} for physical robots
                           If None, uses starts in sorted order

        Returns:
            PathDecompositionResult with paths and assignments
        """
        # Verify flow result is valid
        if hasattr(flow_result, 'status'):
            if flow_result.status != ProductionFlowStatus.UNIQUE:
                raise ValueError(f"Cannot decompose infeasible flow: {flow_result.status}")

        # Build mutable flow graph
        flow_graph = self._build_flow_graph(flow_result.flow_assignment)

        # Extract paths in deterministic order
        extracted_paths = []
        starts_sorted = sorted(self.graph.starts)

        path_id = 0
        for start in starts_sorted:
            # Check if there's flow from this start
            start_node_id = f"v{start}_t0_in"

            while self._has_outgoing_flow(flow_graph, start_node_id):
                path = self._extract_single_path(flow_graph, start)
                if path:
                    path_obj = ExtractedPath(
                        path_id=path_id,
                        vertices=path,
                        start_vertex=path[0],
                        goal_vertex=path[-1],
                        makespan=len(path) - 1
                    )
                    extracted_paths.append(path_obj)
                    path_id += 1

        # Compute metrics
        total_makespan = max(p.makespan for p in extracted_paths) if extracted_paths else 0
        sum_of_costs = sum(p.makespan for p in extracted_paths)

        # Assign to robots
        robot_assignments = self._assign_to_robots(
            extracted_paths, robot_positions
        )

        return PathDecompositionResult(
            extracted_paths=extracted_paths,
            robot_assignments=robot_assignments,
            total_makespan=total_makespan,
            sum_of_costs=sum_of_costs,
            statistics={
                "num_paths": len(extracted_paths),
                "avg_path_length": sum_of_costs / len(extracted_paths) if extracted_paths else 0
            }
        )

    def _build_flow_graph(self, flow_assignment: Dict[str, int]
                         ) -> Dict[str, Dict[str, int]]:
        """
        Build mutable flow graph from assignment.

        Returns: {source_node_id: {target_node_id: flow}}

        Edge ID formats:
        - v{X}_t{T}_in->v{X}_t{T}_out_node_split
        - v{X}_t{T}_out->v{Y}_t{T+1}_in_move
        - v{X}_t{T}_out->v{X}_t{T+1}_in_wait
        - v{-1}_t{-1}_out->v{X}_t{0}_in_super_source
        - v{X}_t{T}_out->v{-2}_t{T+1}_in_super_sink
        - swap_X_Y_tT->v{X}_t{T+1}_in (gadget edges)
        """
        flow_graph = defaultdict(lambda: defaultdict(int))

        for edge_id, flow in flow_assignment.items():
            if flow > 0 and "->" in edge_id:
                # Split on first "->" only
                arrow_idx = edge_id.find("->")
                source_id = edge_id[:arrow_idx]
                target_with_suffix = edge_id[arrow_idx + 2:]

                # Remove edge type suffix (_node_split, _move, _wait, _super_source, _super_sink)
                # Find the last underscore that's not part of the node ID
                suffixes = ["_node_split", "_move", "_wait", "_super_source", "_super_sink"]
                target_id = target_with_suffix
                for suffix in suffixes:
                    if target_id.endswith(suffix):
                        target_id = target_id[:-len(suffix)]
                        break

                flow_graph[source_id][target_id] += flow

        return flow_graph

    def _has_outgoing_flow(self, flow_graph: Dict[str, Dict[str, int]],
                          node_id: str) -> bool:
        """Check if node has outgoing flow."""
        if node_id not in flow_graph:
            return False
        return any(f > 0 for f in flow_graph[node_id].values())

    def _extract_single_path(self, flow_graph: Dict[str, Dict[str, int]],
                            start_vertex: int) -> Optional[List[int]]:
        """
        Extract a single path starting from given vertex.

        Follows positive flow edges, choosing lexicographically smallest
        at each step. Decrements flow as we go.

        Handles swap gadget nodes (swap_X_Y_tT) by following through them.

        Returns list of vertices forming the path.
        """
        path_vertices = [start_vertex]
        current_vertex = start_vertex
        current_time = 0

        while current_time <= self.horizon:
            # Get current node ID (out node)
            current_out_id = f"v{current_vertex}_t{current_time}_out"

            # First traverse split edge (in -> out)
            current_in_id = f"v{current_vertex}_t{current_time}_in"
            if current_in_id in flow_graph:
                if current_out_id in flow_graph[current_in_id]:
                    if flow_graph[current_in_id][current_out_id] > 0:
                        flow_graph[current_in_id][current_out_id] -= 1

            # Check if at goal at final time
            if current_time == self.horizon:
                if current_vertex in self.graph.goals:
                    # Decrement flow to sink
                    sink_id = self.graph.super_sink.canonical_id()
                    if current_out_id in flow_graph:
                        if sink_id in flow_graph[current_out_id]:
                            flow_graph[current_out_id][sink_id] -= 1
                    return path_vertices
                else:
                    # Didn't reach goal - path incomplete
                    return None

            # Find next move - may go through swap gadget or directly
            if current_out_id not in flow_graph:
                return None

            next_options = []
            swap_gadget_routes = []

            for target_id, flow in flow_graph[current_out_id].items():
                if flow > 0:
                    # Check if direct edge to next vertex
                    if target_id.startswith("v") and "_t" in target_id:
                        parts = target_id.split("_")
                        try:
                            next_vertex = int(parts[0][1:])
                            next_time = int(parts[1][1:])
                            if next_time == current_time + 1:
                                next_options.append((next_vertex, target_id, None))
                        except (ValueError, IndexError):
                            continue
                    # Check if edge to swap gadget
                    elif target_id.startswith("swap_"):
                        swap_gadget_routes.append(target_id)

            # If we have swap gadget routes, follow them to find actual destinations
            # Swap gadgets now have structure: source -> swap_X_Y_tT_in -> swap_X_Y_tT_out -> dest
            for swap_id in swap_gadget_routes:
                # Follow the swap gadget path: swap_in -> swap_out -> destination
                swap_in_id = swap_id
                swap_out_id = swap_id.replace("_in", "_out") if swap_id.endswith("_in") else swap_id + "_out"

                # If we entered swap_in, follow to swap_out
                if swap_in_id in flow_graph:
                    for mid_id, mid_flow in flow_graph[swap_in_id].items():
                        if mid_flow > 0:
                            # This should be swap_out or direct dest
                            if mid_id.endswith("_out") and "swap_" in mid_id:
                                # Follow swap_out to destinations
                                if mid_id in flow_graph:
                                    for dest_id, dest_flow in flow_graph[mid_id].items():
                                        if dest_flow > 0 and dest_id.startswith("v") and "_t" in dest_id:
                                            parts = dest_id.split("_")
                                            try:
                                                next_vertex = int(parts[0][1:])
                                                next_time = int(parts[1][1:])
                                                if next_time == current_time + 1:
                                                    next_options.append((next_vertex, dest_id, (swap_in_id, mid_id)))
                                            except (ValueError, IndexError):
                                                continue
                            elif dest_id.startswith("v") and "_t" in mid_id:
                                # Direct route (old format compatibility)
                                parts = mid_id.split("_")
                                try:
                                    next_vertex = int(parts[0][1:])
                                    next_time = int(parts[1][1:])
                                    if next_time == current_time + 1:
                                        next_options.append((next_vertex, mid_id, swap_in_id))
                                except (ValueError, IndexError):
                                    continue

            if not next_options:
                return None

            # Sort lexicographically and pick smallest
            next_options.sort(key=lambda x: x[0])
            next_vertex, next_target_id, swap_id = next_options[0]

            # Decrement flow along the path taken
            if swap_id:
                if isinstance(swap_id, tuple):
                    # New format: (swap_in, swap_out)
                    swap_in_id, swap_out_id = swap_id
                    flow_graph[current_out_id][swap_in_id] -= 1
                    flow_graph[swap_in_id][swap_out_id] -= 1
                    flow_graph[swap_out_id][next_target_id] -= 1
                else:
                    # Old format: single swap_id
                    flow_graph[current_out_id][swap_id] -= 1
                    flow_graph[swap_id][next_target_id] -= 1
            else:
                # Direct edge
                flow_graph[current_out_id][next_target_id] -= 1

            # Move to next
            path_vertices.append(next_vertex)
            current_vertex = next_vertex
            current_time += 1

        return path_vertices if len(path_vertices) > 1 else None

    def _assign_to_robots(self, paths: List[ExtractedPath],
                         robot_positions: Optional[Dict[int, int]]
                         ) -> List[RobotAssignment]:
        """
        Assign extracted paths to physical robots.

        If robot_positions is given, match by start vertex.
        Otherwise, assign in sorted order.
        """
        assignments = []

        if robot_positions is None:
            # Default: assign in order of sorted starts
            sorted_starts = sorted(self.graph.starts)
            robot_positions = {i: s for i, s in enumerate(sorted_starts)}

        # Group paths by start vertex
        paths_by_start = defaultdict(list)
        for path in paths:
            paths_by_start[path.start_vertex].append(path)

        # Assign robots to paths
        for robot_id in sorted(robot_positions.keys()):
            robot_start = robot_positions[robot_id]

            if robot_start in paths_by_start and paths_by_start[robot_start]:
                # Take first available path from this start
                path = paths_by_start[robot_start].pop(0)
                assignments.append(RobotAssignment(
                    robot_id=robot_id,
                    path=path,
                    physical_start=robot_start
                ))
            else:
                # No path available - this shouldn't happen with valid flow
                raise ValueError(
                    f"No path available for robot {robot_id} at start {robot_start}"
                )

        return assignments


def extract_paths_from_flow(graph: TimeExpandedGraph,
                           flow_result: ProductionFlowResult,
                           robot_positions: Optional[Dict[int, int]] = None
                           ) -> PathDecompositionResult:
    """
    Convenience function to extract paths from flow result.

    Args:
        graph: Time-expanded MAPF graph
        flow_result: Result from flow solver (must be UNIQUE)
        robot_positions: Optional mapping {robot_id: start_vertex}

    Returns:
        PathDecompositionResult with extracted paths
    """
    decomposer = PathDecomposer(graph)
    return decomposer.decompose(flow_result, robot_positions)


def extract_paths(graph: TimeExpandedGraph,
                  flow_result: ProductionFlowResult,
                  robot_positions: Optional[Dict[int, int]] = None
                  ) -> PathDecompositionResult:
    """
    Main entry point to extract paths from production flow result.

    This is an alias for extract_paths_from_flow for cleaner API.

    Args:
        graph: Time-expanded MAPF graph
        flow_result: Result from production flow solver (must be UNIQUE)
        robot_positions: Optional mapping {robot_id: start_vertex}

    Returns:
        PathDecompositionResult with extracted paths and robot assignments
    """
    return extract_paths_from_flow(graph, flow_result, robot_positions)


def paths_to_labeled_solution(decomposition: PathDecompositionResult
                             ) -> Dict[int, List[int]]:
    """
    Convert decomposition to standard labeled MAPF solution format.

    Returns: {robot_id: [v0, v1, ..., vT]}
    """
    return {
        assignment.robot_id: assignment.path.vertices
        for assignment in decomposition.robot_assignments
    }


class PathDecompositionVerifier:
    """
    Verifies path decomposition is correct.

    Checks:
    1. Number of paths equals number of robots
    2. All starts are covered
    3. All goals are covered
    4. Paths are vertex-disjoint at each time (no collisions)
    5. No edge swaps between paths
    """

    def __init__(self, graph: TimeExpandedGraph):
        self.graph = graph

    def verify(self, decomposition: PathDecompositionResult) -> Dict:
        """
        Verify decomposition is valid.

        Returns verification result with any violations.
        """
        errors = []
        paths = decomposition.extracted_paths

        # Check 1: Number of paths
        expected_paths = self.graph.num_robots()
        if len(paths) != expected_paths:
            errors.append({
                "type": "PATH_COUNT",
                "expected": expected_paths,
                "actual": len(paths)
            })

        # Check 2: All starts covered
        path_starts = set(p.start_vertex for p in paths)
        if path_starts != self.graph.starts:
            errors.append({
                "type": "START_COVERAGE",
                "expected": sorted(self.graph.starts),
                "actual": sorted(path_starts)
            })

        # Check 3: All goals covered
        path_goals = set(p.goal_vertex for p in paths)
        if path_goals != self.graph.goals:
            errors.append({
                "type": "GOAL_COVERAGE",
                "expected": sorted(self.graph.goals),
                "actual": sorted(path_goals)
            })

        # Check 4: Vertex collisions
        for t in range(self.graph.horizon + 1):
            positions_at_t = []
            for p in paths:
                pos = p.get_position_at_time(t)
                positions_at_t.append((pos, p.path_id))

            # Check for duplicates
            vertex_counts = defaultdict(list)
            for pos, path_id in positions_at_t:
                vertex_counts[pos].append(path_id)

            for vertex, path_ids in vertex_counts.items():
                if len(path_ids) > 1:
                    errors.append({
                        "type": "VERTEX_COLLISION",
                        "time": t,
                        "vertex": vertex,
                        "paths": path_ids
                    })

        # Check 5: Edge swaps
        for t in range(self.graph.horizon):
            moves_at_t = []
            for p in paths:
                move = p.get_move_at_time(t)
                moves_at_t.append((move, p.path_id))

            # Check for swaps
            for i, (move_i, path_i) in enumerate(moves_at_t):
                for j, (move_j, path_j) in enumerate(moves_at_t[i+1:], i+1):
                    if move_i[0] != move_i[1] and move_j[0] != move_j[1]:
                        # Both are actual moves (not waits)
                        if move_i[0] == move_j[1] and move_i[1] == move_j[0]:
                            errors.append({
                                "type": "EDGE_SWAP",
                                "time": t,
                                "paths": [path_i, path_j],
                                "moves": [move_i, move_j]
                            })

        return {
            "valid": len(errors) == 0,
            "num_errors": len(errors),
            "errors": errors[:10]  # First 10 errors
        }


# Example usage
if __name__ == "__main__":
    from .time_expanded_graph import WarehouseGraph, TimeExpandedGraph
    from .flow_solver import solve_production_mapf

    # Create test instance
    warehouse = WarehouseGraph.from_grid(10, 10)
    starts = frozenset([0, 1, 2])
    goals = frozenset([97, 98, 99])

    graph = TimeExpandedGraph(
        base_graph=warehouse,
        horizon=20,
        starts=starts,
        goals=goals
    )

    # Solve flow
    flow_result = solve_production_mapf(graph, optimize_cost=True)
    print(f"Flow result: {flow_result.status.value}")

    if flow_result.status == ProductionFlowStatus.UNIQUE:
        # Decompose
        decomposition = extract_paths(graph, flow_result)
        print(f"\nDecomposition:")
        print(f"  Paths: {len(decomposition.extracted_paths)}")
        print(f"  Makespan: {decomposition.total_makespan}")
        print(f"  Sum of costs: {decomposition.sum_of_costs}")
        print(f"  Fingerprint: {decomposition.fingerprint()}")

        # Verify
        verifier = PathDecompositionVerifier(graph)
        result = verifier.verify(decomposition)
        print(f"\nVerification: {'PASS' if result['valid'] else 'FAIL'}")
        if not result['valid']:
            print(f"  Errors: {result['errors']}")
