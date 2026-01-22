"""
Warehouse Graph for Transport Fabric

The warehouse graph is the base directed graph G = (V, E) that gets
compiled into a transport fabric of lanes and junctions.

Key structures:
- LaneSegment: directed path/cycle of vertices (conveyor-like)
- JunctionVertex: intersection where lanes meet and switch
- VertexType: lane vertex, junction vertex, station, buffer
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, FrozenSet
from enum import Enum
import hashlib
import json


class VertexType(Enum):
    """Type of vertex in warehouse graph."""
    LANE = "LANE"           # Part of a lane segment
    JUNCTION = "JUNCTION"   # Intersection/switch point
    STATION = "STATION"     # Pick/pack/dock location
    BUFFER = "BUFFER"       # Waiting/staging area
    CHARGER = "CHARGER"     # Robot charging station


@dataclass(frozen=True)
class LaneSegment:
    """
    A directed lane segment (conveyor-like dynamics).

    Vertices form a directed path or cycle. Each tick, all robots
    in the lane advance one position (permutation by rotation).

    Properties:
    - lane_id: unique identifier
    - vertices: ordered list of vertices in the lane
    - is_cycle: if True, last vertex connects to first
    - direction: flow direction (for visualization)
    """
    lane_id: int
    vertices: Tuple[int, ...]
    is_cycle: bool = False

    def __len__(self) -> int:
        return len(self.vertices)

    def successor(self, v: int) -> Optional[int]:
        """Get successor vertex in lane (None if end of non-cycle)."""
        try:
            idx = self.vertices.index(v)
            if idx == len(self.vertices) - 1:
                return self.vertices[0] if self.is_cycle else None
            return self.vertices[idx + 1]
        except ValueError:
            return None

    def predecessor(self, v: int) -> Optional[int]:
        """Get predecessor vertex in lane."""
        try:
            idx = self.vertices.index(v)
            if idx == 0:
                return self.vertices[-1] if self.is_cycle else None
            return self.vertices[idx - 1]
        except ValueError:
            return None

    def contains(self, v: int) -> bool:
        return v in self.vertices

    def fingerprint(self) -> str:
        data = {
            "lane_id": self.lane_id,
            "vertices": list(self.vertices),
            "is_cycle": self.is_cycle
        }
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]


@dataclass
class JunctionVertex:
    """
    A junction vertex where lanes meet and robots can switch.

    Junctions have a set of safe local permutations that:
    - Are bijections on the junction neighborhood
    - Respect directed edges
    - Forbid swaps (no adjacent 2-cycles)
    - Connect incoming lanes to outgoing lanes

    Properties:
    - junction_id: unique identifier
    - center: central vertex of junction
    - neighborhood: all vertices involved in junction permutations
    - incoming_lanes: lane segments feeding into junction
    - outgoing_lanes: lane segments exiting junction
    """
    junction_id: int
    center: int
    neighborhood: FrozenSet[int]
    incoming_lanes: List[int] = field(default_factory=list)  # lane_ids
    outgoing_lanes: List[int] = field(default_factory=list)  # lane_ids

    def size(self) -> int:
        return len(self.neighborhood)

    def contains(self, v: int) -> bool:
        return v in self.neighborhood


@dataclass
class FabricWarehouseGraph:
    """
    Warehouse graph structured for transport fabric compilation.

    This extends a basic directed graph with:
    - Vertex typing (lane, junction, station, buffer)
    - Lane decomposition hints
    - Junction locations
    - Station/charger locations

    The graph is designed to be compiled into a TransportFabric
    where lanes become cyclic permutations and junctions become
    local switch gadgets.
    """
    num_vertices: int
    edges: FrozenSet[Tuple[int, int]]
    vertex_types: Dict[int, VertexType] = field(default_factory=dict)
    positions: Optional[Dict[int, Tuple[float, float]]] = None

    # Pre-identified structures (can be auto-detected or specified)
    lane_hints: List[LaneSegment] = field(default_factory=list)
    junction_hints: List[JunctionVertex] = field(default_factory=list)
    stations: FrozenSet[int] = field(default_factory=frozenset)
    chargers: FrozenSet[int] = field(default_factory=frozenset)

    def __post_init__(self):
        """Initialize vertex types if not provided."""
        if not self.vertex_types:
            # Default all to LANE, will be updated by compiler
            self.vertex_types = {v: VertexType.LANE for v in range(self.num_vertices)}

        # Mark stations and chargers
        for s in self.stations:
            self.vertex_types[s] = VertexType.STATION
        for c in self.chargers:
            self.vertex_types[c] = VertexType.CHARGER

        # Build adjacency list cache for O(1) neighbor lookup
        # This is critical for performance with large graphs
        object.__setattr__(self, '_adj_out', {})
        object.__setattr__(self, '_adj_in', {})
        adj_out = {}
        adj_in = {}
        for (u, v) in self.edges:
            if u not in adj_out:
                adj_out[u] = []
            adj_out[u].append(v)
            if v not in adj_in:
                adj_in[v] = []
            adj_in[v].append(u)
        object.__setattr__(self, '_adj_out', adj_out)
        object.__setattr__(self, '_adj_in', adj_in)

    @classmethod
    def from_grid(cls, width: int, height: int,
                  obstacles: Optional[Set[Tuple[int, int]]] = None,
                  one_way_horizontal: bool = False,
                  one_way_vertical: bool = False) -> "FabricWarehouseGraph":
        """
        Create warehouse graph from grid with optional one-way lanes.

        For large warehouses, one-way lanes significantly improve
        throughput by avoiding head-on conflicts.

        MATHEMATICAL REQUIREMENT:
        Every vertex must have at least one outgoing edge to avoid dead ends.
        Dead ends make the permutation completion infeasible.
        We ensure this by wrapping one-way lanes at boundaries (torus-like).

        Args:
            width: Grid width
            height: Grid height
            obstacles: Set of (x, y) obstacle positions
            one_way_horizontal: If True, horizontal edges are one-way (alternating)
            one_way_vertical: If True, vertical edges are one-way (alternating)
        """
        obstacles = obstacles or set()

        def pos_to_vertex(x: int, y: int) -> int:
            return y * width + x

        edges = set()
        positions = {}

        # First pass: identify free vertices
        free_positions = set()
        for y in range(height):
            for x in range(width):
                if (x, y) not in obstacles:
                    free_positions.add((x, y))
                    v = pos_to_vertex(x, y)
                    positions[v] = (float(x), float(y))

        # Second pass: add edges with wrap-around for one-way lanes
        for y in range(height):
            for x in range(width):
                if (x, y) in obstacles:
                    continue

                v = pos_to_vertex(x, y)

                # Horizontal neighbors
                if one_way_horizontal:
                    # Alternating direction based on row
                    if y % 2 == 0:
                        # Rightward lane
                        if x + 1 < width and (x + 1, y) not in obstacles:
                            edges.add((v, pos_to_vertex(x + 1, y)))
                        else:
                            # At right boundary: wrap to vertical connector or add return
                            # Find nearest free vertex in the next row (down)
                            if y + 1 < height and (x, y + 1) not in obstacles:
                                edges.add((v, pos_to_vertex(x, y + 1)))
                            elif y > 0 and (x, y - 1) not in obstacles:
                                edges.add((v, pos_to_vertex(x, y - 1)))
                    else:
                        # Leftward lane
                        if x > 0 and (x - 1, y) not in obstacles:
                            edges.add((v, pos_to_vertex(x - 1, y)))
                        else:
                            # At left boundary: wrap to vertical connector
                            if y + 1 < height and (x, y + 1) not in obstacles:
                                edges.add((v, pos_to_vertex(x, y + 1)))
                            elif y > 0 and (x, y - 1) not in obstacles:
                                edges.add((v, pos_to_vertex(x, y - 1)))
                else:
                    # Bidirectional horizontal
                    if x + 1 < width and (x + 1, y) not in obstacles:
                        u = pos_to_vertex(x + 1, y)
                        edges.add((v, u))
                        edges.add((u, v))

                # Vertical neighbors
                if one_way_vertical:
                    # Alternating direction based on column
                    if x % 2 == 0:
                        # Downward lane
                        if y + 1 < height and (x, y + 1) not in obstacles:
                            edges.add((v, pos_to_vertex(x, y + 1)))
                        else:
                            # At bottom boundary: wrap to horizontal connector
                            if x + 1 < width and (x + 1, y) not in obstacles:
                                edges.add((v, pos_to_vertex(x + 1, y)))
                            elif x > 0 and (x - 1, y) not in obstacles:
                                edges.add((v, pos_to_vertex(x - 1, y)))
                    else:
                        # Upward lane
                        if y > 0 and (x, y - 1) not in obstacles:
                            edges.add((v, pos_to_vertex(x, y - 1)))
                        else:
                            # At top boundary: wrap to horizontal connector
                            if x + 1 < width and (x + 1, y) not in obstacles:
                                edges.add((v, pos_to_vertex(x + 1, y)))
                            elif x > 0 and (x - 1, y) not in obstacles:
                                edges.add((v, pos_to_vertex(x - 1, y)))
                else:
                    # Bidirectional vertical
                    if y + 1 < height and (x, y + 1) not in obstacles:
                        u = pos_to_vertex(x, y + 1)
                        edges.add((v, u))
                        edges.add((u, v))

        # Final pass: ensure graph supports valid permutation completion
        # MATHEMATICAL REQUIREMENT:
        # For every vertex v that can receive inflow (forced target), v must have
        # sufficient outgoing edges to avoid Hall's theorem violations.
        #
        # The safest approach: make the graph bidirectional at the physical layer.
        # One-way flow is enforced at the lane decomposition layer, not the edge layer.
        # This ensures any vertex receiving flow can always send flow somewhere.

        bidirectional_edges = set()
        for (u, v) in edges:
            bidirectional_edges.add((u, v))
            bidirectional_edges.add((v, u))
        edges = bidirectional_edges

        # Ensure no isolated vertices
        for (x, y) in free_positions:
            v = pos_to_vertex(x, y)
            has_outgoing = any(e[0] == v for e in edges)
            if not has_outgoing:
                # Add edges to all adjacent free vertices
                for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                    nx, ny = x + dx, y + dy
                    if (nx, ny) in free_positions:
                        u = pos_to_vertex(nx, ny)
                        edges.add((v, u))
                        edges.add((u, v))

        num_vertices = width * height
        return cls(
            num_vertices=num_vertices,
            edges=frozenset(edges),
            positions=positions
        )

    @classmethod
    def create_aisle_warehouse(cls, width: int, height: int,
                               aisle_width: int = 2,
                               shelf_depth: int = 3,
                               num_stations: int = 10) -> "FabricWarehouseGraph":
        """
        Create realistic warehouse layout with one-way aisles.

        Layout:
        - Horizontal aisles (alternating direction)
        - Vertical cross-aisles (alternating direction)
        - Shelf blocks between aisles
        - Stations along edges
        """
        obstacles = set()
        period = aisle_width + shelf_depth

        # Create shelf obstacles (leaving aisles)
        for y in range(height):
            for x in range(width):
                x_in_period = x % period
                y_in_period = y % period

                # Shelves in interior of period blocks
                if (aisle_width <= x_in_period < period and
                    aisle_width <= y_in_period < period):
                    # Leave cross-aisle gaps
                    if not (y % (period * 2) < aisle_width):
                        obstacles.add((x, y))

        # Create base graph with one-way aisles
        graph = cls.from_grid(
            width, height,
            obstacles=obstacles,
            one_way_horizontal=True,
            one_way_vertical=True
        )

        # Add stations along bottom edge
        stations = set()
        free_vertices = [v for v in range(graph.num_vertices)
                        if graph.positions and v in graph.positions
                        and (graph.positions[v][0], graph.positions[v][1]) not in obstacles]

        # Pick station locations
        station_vertices = free_vertices[:num_stations] if len(free_vertices) >= num_stations else free_vertices
        stations = frozenset(station_vertices)

        return cls(
            num_vertices=graph.num_vertices,
            edges=graph.edges,
            positions=graph.positions,
            stations=stations
        )

    def neighbors(self, v: int) -> List[int]:
        """Get outgoing neighbors of vertex v (O(1) using cached adjacency list)."""
        return self._adj_out.get(v, [])

    def predecessors(self, v: int) -> List[int]:
        """Get incoming neighbors of vertex v (O(1) using cached adjacency list)."""
        return self._adj_in.get(v, [])

    def out_degree(self, v: int) -> int:
        return len(self.neighbors(v))

    def in_degree(self, v: int) -> int:
        return len(self.predecessors(v))

    def is_junction_candidate(self, v: int) -> bool:
        """
        Check if vertex is a junction candidate.

        A junction is where multiple lanes meet:
        - In-degree > 1 or out-degree > 1
        - Not a station or buffer
        """
        if self.vertex_types.get(v) in (VertexType.STATION, VertexType.BUFFER, VertexType.CHARGER):
            return False
        return self.in_degree(v) > 1 or self.out_degree(v) > 1

    def free_vertices(self) -> List[int]:
        """Get all non-obstacle vertices."""
        if self.positions:
            return list(self.positions.keys())
        return list(range(self.num_vertices))

    def fingerprint(self) -> str:
        """Canonical fingerprint of graph structure."""
        data = {
            "num_vertices": self.num_vertices,
            "num_edges": len(self.edges),
            "edges_hash": hashlib.sha256(
                str(sorted(self.edges)).encode()
            ).hexdigest()[:16]
        }
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]

    def statistics(self) -> Dict:
        """Return graph statistics."""
        free = self.free_vertices()
        return {
            "num_vertices": self.num_vertices,
            "num_edges": len(self.edges),
            "num_free": len(free),
            "num_stations": len(self.stations),
            "num_chargers": len(self.chargers),
            "avg_out_degree": sum(self.out_degree(v) for v in free) / len(free) if free else 0,
            "junction_candidates": sum(1 for v in free if self.is_junction_candidate(v))
        }
