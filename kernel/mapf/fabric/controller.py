"""
Backpressure Controller for Transport Fabric

MATHEMATICAL FOUNDATION (CORRECTED):
====================================

The tick IS a permutation P_t ∈ Match(A). Junction modes are derived labels.

Controller outputs PREFERENCES (edge costs), not mode selections.
The matching solver finds the optimal feasible permutation.

σ_t(J) = π_modes(P_t)  (modes derived AFTER choosing P_t)

Pressure-based edge costs:
  cost(u→v) = C_base - w(u,v) * Δ_{u→v}(t)

where:
- Δ_{u→v}(t) = Q_u(t) - Q_v(t)  (queue differential)
- w(u,v) = weight/priority for edge
- C_base = constant to keep costs positive

Min-cost matching minimizes Σ cost(e), which maximizes Σ pressure(e).

Properties:
- Throughput optimal: if demand within capacity, queues stable
- Feasibility guaranteed: matching solver finds valid permutation or UNSAT
- No minted infeasible ticks: Hall witness certifies impossibility

This is the proper scale math for 10k robots.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, FrozenSet
from collections import defaultdict
import heapq

from .fabric_compiler import TransportFabric
from .junction_gadgets import JunctionGadget, JunctionPermutation
from .permutation_executor import OccupancyBitset


@dataclass
class QueueState:
    """
    Queue state for backpressure routing.

    Maintains queue lengths at each vertex, optionally by destination class.
    """
    # Simple queues: vertex → queue length
    vertex_queues: Dict[int, int] = field(default_factory=lambda: defaultdict(int))

    # Destination-class queues: vertex → {dest_class → queue length}
    class_queues: Dict[int, Dict[int, int]] = field(default_factory=lambda: defaultdict(dict))

    # Total robots at each vertex (for occupancy tracking)
    occupancy_count: Dict[int, int] = field(default_factory=lambda: defaultdict(int))

    def get_queue(self, v: int) -> int:
        """Get total queue length at vertex."""
        return self.vertex_queues[v]

    def get_class_queue(self, v: int, dest_class: int) -> int:
        """Get queue length for specific destination class."""
        return self.class_queues[v].get(dest_class, 0)

    def increment(self, v: int, amount: int = 1, dest_class: Optional[int] = None):
        """Add to queue at vertex."""
        self.vertex_queues[v] += amount
        if dest_class is not None:
            if dest_class not in self.class_queues[v]:
                self.class_queues[v][dest_class] = 0
            self.class_queues[v][dest_class] += amount

    def decrement(self, v: int, amount: int = 1, dest_class: Optional[int] = None):
        """Remove from queue at vertex."""
        self.vertex_queues[v] = max(0, self.vertex_queues[v] - amount)
        if dest_class is not None and dest_class in self.class_queues[v]:
            self.class_queues[v][dest_class] = max(0, self.class_queues[v][dest_class] - amount)

    def total_queue_length(self) -> int:
        """Total queue length across all vertices."""
        return sum(self.vertex_queues.values())

    def max_queue_length(self) -> int:
        """Maximum queue length at any vertex."""
        return max(self.vertex_queues.values()) if self.vertex_queues else 0


@dataclass
class PressureMetrics:
    """Metrics from pressure computation."""
    total_pressure: float
    max_pressure: float
    congested_vertices: int  # vertices with queue > threshold
    mode_pressures: Dict[int, float] = field(default_factory=dict)  # mode_id → pressure


class BackpressureController:
    """
    Backpressure controller for transport fabric.

    CORRECTED FORMULATION:
    The controller outputs edge PREFERENCES (costs), not mode selections.
    The matching solver finds the optimal feasible permutation.
    Junction modes are derived labels, not control inputs.
    """

    # Base cost for edges (ensures costs are positive after pressure adjustment)
    BASE_COST = 100

    def __init__(self, fabric: TransportFabric,
                 congestion_threshold: int = 5,
                 edge_weights: Optional[Dict[Tuple[int, int], float]] = None):
        """
        Initialize backpressure controller.

        Args:
            fabric: Compiled transport fabric
            congestion_threshold: Queue length considered "congested"
            edge_weights: Optional {(u,v): weight} for edge priorities
        """
        self.fabric = fabric
        self.congestion_threshold = congestion_threshold
        self.edge_weights = edge_weights or {}

    def compute_edge_preferences(self, occupancy: OccupancyBitset,
                                  queues: QueueState) -> Dict[Tuple[int, int], int]:
        """
        Compute edge preferences (costs) for the matching solver.

        This is the PRIMARY method. The matching solver uses these costs
        to find the optimal feasible permutation.

        MATHEMATICAL REQUIREMENT:
        In a bipartite graph (like a grid), minimum cycle length is 4.
        A 4-cycle needs LOW COSTS in the 2-hop neighborhood of occupied vertices.

        Cost structure:
        - Edges FROM occupied: cost 0 (start of chain)
        - Edges TO occupied: cost 2 (close the cycle)
        - Edges BETWEEN neighbors of occupied: cost 2 (chain continuation)
        - All other edges: cost 5 (base)
        - Wait: cost 5 (in completer)

        Lower cost = more preferred. Min-cost matching finds optimal cycles.

        Args:
            occupancy: Current occupancy bitset
            queues: Current queue state

        Returns:
            {(src, dst): cost} edge preferences for matching solver
        """
        preferences: Dict[Tuple[int, int], int] = {}
        occupied_set = occupancy.occupied_vertices()

        # Compute 1-hop and 2-hop neighborhoods of occupied vertices
        neighbors_of_occupied: Set[int] = set()
        for v in occupied_set:
            neighbors_of_occupied.update(self.fabric.warehouse.neighbors(v))

        two_hop_of_occupied: Set[int] = set()
        for v in neighbors_of_occupied:
            two_hop_of_occupied.update(self.fabric.warehouse.neighbors(v))

        # Compute pressure-based costs for all graph edges
        for (src, dst) in self.fabric.warehouse.edges:
            if src == dst:
                continue

            # Priority 1: Edges FROM occupied vertices (start of chain)
            if src in occupied_set:
                delta = queues.get_queue(src) - queues.get_queue(dst)
                weight = self.edge_weights.get((src, dst), 1.0)
                pressure = weight * (1.0 + max(0, delta))
                cost = max(0, min(2, int(3 - pressure)))
                preferences[(src, dst)] = cost

            # Priority 2: Edges TO occupied vertices (close cycle)
            elif dst in occupied_set:
                preferences[(src, dst)] = 2

            # Priority 3: Edges BETWEEN neighbors of occupied (chain continuation)
            # This is crucial for 4-cycles in bipartite graphs
            elif src in neighbors_of_occupied and dst in neighbors_of_occupied:
                preferences[(src, dst)] = 2

            # Priority 4: Edges in 2-hop neighborhood (longer chains)
            elif src in two_hop_of_occupied or dst in two_hop_of_occupied:
                preferences[(src, dst)] = 3

            # All other edges: base cost
            else:
                preferences[(src, dst)] = 5

        return preferences

    def compute_edge_weights(self, occupancy: OccupancyBitset,
                             queues: QueueState) -> Dict[Tuple[int, int], float]:
        """
        Compute edge weights for incremental matching.

        HIGHER weight = more desirable (opposite of costs).

        Args:
            occupancy: Current occupancy bitset
            queues: Current queue state

        Returns:
            {(src, dst): weight} edge weights for incremental matcher
        """
        weights: Dict[Tuple[int, int], float] = {}
        occupied_set = occupancy.occupied_vertices()

        # Precompute neighbors of occupied set
        neighbors_of_occupied = self._get_neighbors_of_set(occupied_set)

        for (src, dst) in self.fabric.warehouse.edges:
            if src == dst:
                continue

            # Base weight
            weight = 1.0

            # Bonus for edges from occupied (want robots to move)
            if src in occupied_set:
                delta = queues.get_queue(src) - queues.get_queue(dst)
                pressure = 1.0 + max(0, delta)
                weight = 10.0 * pressure

            # Bonus for edges to occupied (helps close cycles)
            elif dst in occupied_set:
                weight = 5.0

            # Small bonus for edges near occupied (chain continuation)
            elif src in neighbors_of_occupied:
                weight = 3.0

            weights[(src, dst)] = weight

        return weights

    def _get_neighbors_of_set(self, vertex_set: Set[int]) -> Set[int]:
        """Get all neighbors of vertices in the set."""
        neighbors = set()
        for v in vertex_set:
            neighbors.update(self.fabric.warehouse.neighbors(v))
        return neighbors

    def create_weight_function(self, occupancy: OccupancyBitset,
                               queues: QueueState) -> "WeightFunction":
        """
        Create a lazy weight function for on-demand weight computation.

        This is much faster than precomputing all weights when only
        a small fraction of edges are actually queried.

        Returns:
            WeightFunction that computes weights on-demand
        """
        occupied_set = occupancy.occupied_vertices()
        neighbors_of_occupied = self._get_neighbors_of_set(occupied_set)
        return WeightFunction(occupied_set, neighbors_of_occupied, queues)

    def compute_junction_modes(self, occupancy: OccupancyBitset,
                               queues: QueueState) -> Dict[int, int]:
        """Compute optimal junction modes using backpressure."""
        modes = {}
        for junction in self.fabric.junctions:
            mode_id = self._select_junction_mode(junction, occupancy, queues)
            modes[junction.junction_id] = mode_id
        return modes

    def _select_junction_mode(self, junction: JunctionGadget,
                              occupancy: OccupancyBitset,
                              queues: QueueState) -> int:
        """Select optimal mode for a single junction."""
        occupied_in_junction = [v for v in junction.vertices if occupancy.get(v)]
        if not occupied_in_junction:
            return 0
        best_mode = 0
        best_pressure = -float('inf')
        for perm in junction.permutations:
            moves_occupied = any(src in occupied_in_junction and src != dst
                                for (src, dst) in perm.mapping)
            pressure = self._compute_mode_pressure(perm, occupancy, queues)
            if moves_occupied:
                pressure += 10.0
            if pressure > best_pressure:
                best_pressure = pressure
                best_mode = perm.mode_id
        return best_mode

    def _compute_mode_pressure(self, perm: JunctionPermutation,
                               occupancy: OccupancyBitset,
                               queues: QueueState) -> float:
        """Compute pressure relief for a single mode."""
        total_pressure = 0.0
        for (src, dst) in perm.mapping:
            if src != dst and occupancy.get(src):
                delta = 1.0 + queues.get_queue(src) - queues.get_queue(dst)
                weight = self.edge_weights.get((src, dst), 1.0)
                total_pressure += weight * delta
        return total_pressure

    def compute_all_pressures(self, occupancy: OccupancyBitset,
                              queues: QueueState) -> Dict[int, PressureMetrics]:
        """Compute detailed pressure metrics for all junctions."""
        metrics = {}
        for junction in self.fabric.junctions:
            mode_pressures = {}
            max_pressure = 0.0
            for perm in junction.permutations:
                pressure = self._compute_mode_pressure(perm, occupancy, queues)
                mode_pressures[perm.mode_id] = pressure
                max_pressure = max(max_pressure, pressure)
            congested = sum(1 for v in junction.vertices
                           if queues.get_queue(v) > self.congestion_threshold)
            metrics[junction.junction_id] = PressureMetrics(
                total_pressure=sum(mode_pressures.values()),
                max_pressure=max_pressure,
                congested_vertices=congested,
                mode_pressures=mode_pressures
            )
        return metrics


class WeightFunction:
    """
    Lazy weight function that computes edge weights on-demand.

    This avoids iterating all edges when only a subset are queried.
    """
    __slots__ = ['occupied_set', 'neighbors_of_occupied', 'queues', '_cache']

    def __init__(self, occupied_set: Set[int], neighbors_of_occupied: Set[int],
                 queues: "QueueState"):
        self.occupied_set = occupied_set
        self.neighbors_of_occupied = neighbors_of_occupied
        self.queues = queues
        self._cache: Dict[Tuple[int, int], float] = {}

    def get(self, edge: Tuple[int, int], default: float = 0.0) -> float:
        """Get weight for edge, computing lazily if not cached."""
        if edge in self._cache:
            return self._cache[edge]

        src, dst = edge
        if src == dst:
            return default

        weight = 1.0
        if src in self.occupied_set:
            delta = self.queues.get_queue(src) - self.queues.get_queue(dst)
            pressure = 1.0 + max(0, delta)
            weight = 10.0 * pressure
        elif dst in self.occupied_set:
            weight = 5.0
        elif src in self.neighbors_of_occupied:
            weight = 3.0

        self._cache[edge] = weight
        return weight


class DestinationRoutingController(BackpressureController):
    """
    Extended controller with destination-aware routing.

    Uses per-destination-class queues for more intelligent routing
    when robots have specific destinations.
    """

    def __init__(self, fabric: TransportFabric,
                 destination_classes: Dict[int, Set[int]],
                 **kwargs):
        """
        Initialize destination-aware controller.

        Args:
            fabric: Transport fabric
            destination_classes: {class_id: set of destination vertices}
        """
        super().__init__(fabric, **kwargs)
        self.destination_classes = destination_classes

        # Precompute shortest paths to destinations (optional optimization)
        self.distance_to_dest: Dict[int, Dict[int, int]] = {}

    def compute_junction_modes_by_class(self, occupancy: OccupancyBitset,
                                        queues: QueueState,
                                        robot_destinations: Dict[int, int]
                                        ) -> Dict[int, int]:
        """
        Compute modes considering robot destinations.

        Prioritizes moves that bring robots closer to their destinations.
        """
        # For now, fall back to standard backpressure
        # Full implementation would use destination-class pressures
        return self.compute_junction_modes(occupancy, queues)


class AdaptiveController(BackpressureController):
    """
    Adaptive controller that adjusts weights based on congestion.

    Increases weights for congested areas to relieve bottlenecks.
    """

    def __init__(self, fabric: TransportFabric,
                 adaptation_rate: float = 0.1,
                 **kwargs):
        """
        Initialize adaptive controller.

        Args:
            fabric: Transport fabric
            adaptation_rate: Rate of weight adaptation (0-1)
        """
        super().__init__(fabric, **kwargs)
        self.adaptation_rate = adaptation_rate
        self.learned_weights: Dict[Tuple[int, int], float] = {}

    def update_weights(self, queues: QueueState):
        """
        Update edge weights based on observed congestion.

        Increases weights for edges leaving congested vertices.
        """
        for v, q in queues.vertex_queues.items():
            if q > self.congestion_threshold:
                # Increase weight for outgoing edges
                for neighbor in self.fabric.warehouse.neighbors(v):
                    edge = (v, neighbor)
                    current = self.learned_weights.get(edge, 1.0)
                    # Adaptive increase
                    self.learned_weights[edge] = current + self.adaptation_rate * (q / self.congestion_threshold)

        # Update edge weights
        self.edge_weights.update(self.learned_weights)


class PriorityController(BackpressureController):
    """
    Controller with explicit priority handling.

    Some robots/tasks may have higher priority and should be
    routed preferentially.
    """

    def __init__(self, fabric: TransportFabric,
                 priority_weights: Dict[int, float] = None,
                 **kwargs):
        """
        Initialize priority controller.

        Args:
            fabric: Transport fabric
            priority_weights: {priority_level: weight_multiplier}
        """
        super().__init__(fabric, **kwargs)
        self.priority_weights = priority_weights or {0: 1.0, 1: 2.0, 2: 5.0}

    def compute_junction_modes_with_priority(self, occupancy: OccupancyBitset,
                                              queues: QueueState,
                                              vertex_priorities: Dict[int, int]
                                              ) -> Dict[int, int]:
        """
        Compute modes considering robot priorities.

        Higher priority robots get preference in routing decisions.
        """
        # Adjust queue weights by priority
        adjusted_queues = QueueState()

        for v, q in queues.vertex_queues.items():
            priority = vertex_priorities.get(v, 0)
            weight = self.priority_weights.get(priority, 1.0)
            adjusted_queues.vertex_queues[v] = int(q * weight)

        return self.compute_junction_modes(occupancy, adjusted_queues)
