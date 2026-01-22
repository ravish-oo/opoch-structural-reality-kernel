"""
Time-Expanded Graph for Unlabeled MAPF

Mathematical Foundation:
- Warehouse floor as directed graph G = (V, E)
- Discrete time t = 0, 1, ..., T
- Time-expanded node: (v, t) for each vertex v and time t
- Time-expanded edges: wait (v,t) -> (v,t+1) and move (u,t) -> (v,t+1)

The unlabeled occupancy state is Π-fixed:
- ρ_t(v) ∈ {0,1} : occupancy at vertex v, time t
- f_t(u→v) ∈ {0,1} : flow from u to v at time t

Conservation Law (forced motion):
  ρ_{t+1}(v) = ρ_t(v) + Σ_{u:(u,v)∈E} f_t(u→v) - Σ_{w:(v,w)∈E} f_t(v→w)
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, FrozenSet
from enum import Enum
import hashlib
import json


class EdgeType(Enum):
    """Type of edge in time-expanded graph."""
    WAIT = "wait"
    MOVE = "move"
    SUPER_SOURCE = "super_source"
    SUPER_SINK = "super_sink"
    NODE_SPLIT = "node_split"  # For vertex capacity via node splitting


@dataclass(frozen=True)
class TimeExpandedNode:
    """
    Node in time-expanded graph: (vertex, time) pair.

    For node-splitting (vertex capacity enforcement):
    - Each (v,t) splits into (v,t,in) and (v,t,out)
    - Capacity 1 edge between them enforces vertex capacity
    """
    vertex: int
    time: int
    is_split_out: bool = False  # True for v_out in node-splitting

    def __repr__(self) -> str:
        suffix = "_out" if self.is_split_out else "_in" if self.is_split_out is not None else ""
        return f"({self.vertex}, t={self.time}{suffix})"

    def canonical_id(self) -> str:
        """Deterministic string ID for hashing."""
        return f"v{self.vertex}_t{self.time}_{'out' if self.is_split_out else 'in'}"


@dataclass(frozen=True)
class TimeExpandedEdge:
    """
    Edge in time-expanded graph.

    Types:
    - WAIT: (v,t,out) -> (v,t+1,in)
    - MOVE: (u,t,out) -> (v,t+1,in) for (u,v) ∈ E
    - NODE_SPLIT: (v,t,in) -> (v,t,out) with capacity 1
    - SUPER_SOURCE: source -> (s,0,in) for start s
    - SUPER_SINK: (g,T,out) -> sink for goal g
    """
    source: TimeExpandedNode
    target: TimeExpandedNode
    edge_type: EdgeType
    capacity: int = 1
    cost: int = 0  # For min-cost flow
    original_edge: Optional[Tuple[int, int]] = None  # (u, v) in base graph

    def canonical_id(self) -> str:
        """Deterministic string ID."""
        return f"{self.source.canonical_id()}->{self.target.canonical_id()}_{self.edge_type.value}"


@dataclass
class WarehouseGraph:
    """
    Base warehouse graph G = (V, E).

    Vertices are integers 0..n-1.
    Edges are directed pairs (u, v).
    Optionally includes positions for visualization.
    """
    num_vertices: int
    edges: FrozenSet[Tuple[int, int]]
    positions: Optional[Dict[int, Tuple[float, float]]] = None
    obstacles: FrozenSet[int] = field(default_factory=frozenset)

    @classmethod
    def from_grid(cls, width: int, height: int,
                  obstacles: Optional[Set[Tuple[int, int]]] = None,
                  four_connected: bool = True) -> "WarehouseGraph":
        """
        Create warehouse graph from grid.

        Args:
            width: Grid width
            height: Grid height
            obstacles: Set of (x, y) obstacle positions
            four_connected: If True, 4-connected; else 8-connected
        """
        obstacles = obstacles or set()

        def pos_to_vertex(x: int, y: int) -> int:
            return y * width + x

        def vertex_to_pos(v: int) -> Tuple[int, int]:
            return (v % width, v // width)

        # Build edges
        edges = set()
        positions = {}
        obstacle_vertices = set()

        for y in range(height):
            for x in range(width):
                v = pos_to_vertex(x, y)
                positions[v] = (float(x), float(y))

                if (x, y) in obstacles:
                    obstacle_vertices.add(v)
                    continue

                # 4-connected neighbors
                neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]

                # 8-connected adds diagonals
                if not four_connected:
                    neighbors.extend([(x+1, y+1), (x-1, y-1), (x+1, y-1), (x-1, y+1)])

                for nx, ny in neighbors:
                    if 0 <= nx < width and 0 <= ny < height:
                        if (nx, ny) not in obstacles:
                            u = pos_to_vertex(nx, ny)
                            edges.add((v, u))

        return cls(
            num_vertices=width * height,
            edges=frozenset(edges),
            positions=positions,
            obstacles=frozenset(obstacle_vertices)
        )

    def neighbors(self, v: int) -> List[int]:
        """Get outgoing neighbors of vertex v."""
        return [e[1] for e in self.edges if e[0] == v]

    def fingerprint(self) -> str:
        """Canonical fingerprint of graph structure."""
        data = {
            "num_vertices": self.num_vertices,
            "edges": sorted(list(self.edges)),
            "obstacles": sorted(list(self.obstacles))
        }
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]


@dataclass
class TimeExpandedGraph:
    """
    Time-expanded graph for MAPF flow formulation.

    Construction:
    1. For each (v, t): create split nodes (v,t,in) and (v,t,out)
    2. Add capacity-1 edge (v,t,in) -> (v,t,out) [vertex capacity]
    3. For each t < T:
       - Wait edges: (v,t,out) -> (v,t+1,in)
       - Move edges: (u,t,out) -> (v,t+1,in) for (u,v) ∈ E
    4. Add super-source connected to start nodes
    5. Add super-sink connected to goal nodes

    The resulting network has integral optimal flow = collision-free plan.
    """
    base_graph: WarehouseGraph
    horizon: int
    starts: FrozenSet[int]
    goals: FrozenSet[int]

    # Computed structures
    nodes: Dict[str, TimeExpandedNode] = field(default_factory=dict)
    edges: List[TimeExpandedEdge] = field(default_factory=list)
    adjacency: Dict[str, List[TimeExpandedEdge]] = field(default_factory=dict)
    reverse_adjacency: Dict[str, List[TimeExpandedEdge]] = field(default_factory=dict)

    # Special nodes
    super_source: Optional[TimeExpandedNode] = None
    super_sink: Optional[TimeExpandedNode] = None

    def __post_init__(self):
        """Build the time-expanded graph."""
        self._build_graph()

    def _build_graph(self):
        """Construct the complete time-expanded network."""
        T = self.horizon
        G = self.base_graph

        # Create super source and sink
        self.super_source = TimeExpandedNode(vertex=-1, time=-1, is_split_out=True)
        self.super_sink = TimeExpandedNode(vertex=-2, time=T+1, is_split_out=False)
        self.nodes[self.super_source.canonical_id()] = self.super_source
        self.nodes[self.super_sink.canonical_id()] = self.super_sink

        # Create time-expanded nodes with splitting
        for v in range(G.num_vertices):
            if v in G.obstacles:
                continue
            for t in range(T + 1):
                # In-node
                node_in = TimeExpandedNode(vertex=v, time=t, is_split_out=False)
                self.nodes[node_in.canonical_id()] = node_in

                # Out-node
                node_out = TimeExpandedNode(vertex=v, time=t, is_split_out=True)
                self.nodes[node_out.canonical_id()] = node_out

                # Node-split edge (capacity 1 for vertex capacity)
                split_edge = TimeExpandedEdge(
                    source=node_in,
                    target=node_out,
                    edge_type=EdgeType.NODE_SPLIT,
                    capacity=1,
                    cost=0
                )
                self._add_edge(split_edge)

        # Create temporal edges
        for t in range(T):
            for v in range(G.num_vertices):
                if v in G.obstacles:
                    continue

                v_out = self._get_node(v, t, is_out=True)

                # Wait edge: (v,t,out) -> (v,t+1,in)
                v_next_in = self._get_node(v, t+1, is_out=False)
                if v_out and v_next_in:
                    wait_edge = TimeExpandedEdge(
                        source=v_out,
                        target=v_next_in,
                        edge_type=EdgeType.WAIT,
                        capacity=1,
                        cost=1,  # Small cost to prefer shorter paths
                        original_edge=(v, v)
                    )
                    self._add_edge(wait_edge)

                # Move edges: (v,t,out) -> (u,t+1,in) for (v,u) ∈ E
                for u in G.neighbors(v):
                    if u in G.obstacles:
                        continue
                    u_next_in = self._get_node(u, t+1, is_out=False)
                    if v_out and u_next_in:
                        move_edge = TimeExpandedEdge(
                            source=v_out,
                            target=u_next_in,
                            edge_type=EdgeType.MOVE,
                            capacity=1,
                            cost=1,
                            original_edge=(v, u)
                        )
                        self._add_edge(move_edge)

        # Connect super-source to starts at t=0
        for s in self.starts:
            if s in G.obstacles:
                raise ValueError(f"Start {s} is an obstacle")
            s_in = self._get_node(s, 0, is_out=False)
            if s_in:
                source_edge = TimeExpandedEdge(
                    source=self.super_source,
                    target=s_in,
                    edge_type=EdgeType.SUPER_SOURCE,
                    capacity=1,
                    cost=0
                )
                self._add_edge(source_edge)

        # Connect goals at t=T to super-sink
        for g in self.goals:
            if g in G.obstacles:
                raise ValueError(f"Goal {g} is an obstacle")
            g_out = self._get_node(g, T, is_out=True)
            if g_out:
                sink_edge = TimeExpandedEdge(
                    source=g_out,
                    target=self.super_sink,
                    edge_type=EdgeType.SUPER_SINK,
                    capacity=1,
                    cost=0
                )
                self._add_edge(sink_edge)

    def _get_node(self, vertex: int, time: int, is_out: bool) -> Optional[TimeExpandedNode]:
        """Get node by coordinates."""
        node = TimeExpandedNode(vertex=vertex, time=time, is_split_out=is_out)
        return self.nodes.get(node.canonical_id())

    def _add_edge(self, edge: TimeExpandedEdge):
        """Add edge and update adjacency lists."""
        self.edges.append(edge)

        src_id = edge.source.canonical_id()
        tgt_id = edge.target.canonical_id()

        if src_id not in self.adjacency:
            self.adjacency[src_id] = []
        self.adjacency[src_id].append(edge)

        if tgt_id not in self.reverse_adjacency:
            self.reverse_adjacency[tgt_id] = []
        self.reverse_adjacency[tgt_id].append(edge)

    def get_outgoing(self, node: TimeExpandedNode) -> List[TimeExpandedEdge]:
        """Get outgoing edges from a node."""
        return self.adjacency.get(node.canonical_id(), [])

    def get_incoming(self, node: TimeExpandedNode) -> List[TimeExpandedEdge]:
        """Get incoming edges to a node."""
        return self.reverse_adjacency.get(node.canonical_id(), [])

    def num_robots(self) -> int:
        """Number of robots = |starts| = |goals|."""
        return len(self.starts)

    def statistics(self) -> Dict:
        """Graph statistics for benchmarking."""
        return {
            "num_base_vertices": self.base_graph.num_vertices,
            "num_base_edges": len(self.base_graph.edges),
            "horizon": self.horizon,
            "num_robots": self.num_robots(),
            "num_te_nodes": len(self.nodes),
            "num_te_edges": len(self.edges),
            "naive_space_log10": self.num_robots() * (
                len([v for v in range(self.base_graph.num_vertices)
                     if v not in self.base_graph.obstacles])
            ),
        }

    def fingerprint(self) -> str:
        """Canonical fingerprint of the time-expanded graph."""
        data = {
            "base_graph": self.base_graph.fingerprint(),
            "horizon": self.horizon,
            "starts": sorted(list(self.starts)),
            "goals": sorted(list(self.goals)),
            "num_nodes": len(self.nodes),
            "num_edges": len(self.edges)
        }
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]


def create_warehouse_graph(width: int, height: int,
                          aisle_width: int = 2,
                          shelf_width: int = 3) -> WarehouseGraph:
    """
    Create a realistic warehouse layout with aisles and shelves.

    Args:
        width: Total grid width
        height: Total grid height
        aisle_width: Width of aisles (passable)
        shelf_width: Width of shelf blocks (obstacles)
    """
    obstacles = set()
    period = aisle_width + shelf_width

    for y in range(height):
        for x in range(width):
            # Create shelf blocks with periodic aisles
            x_in_period = x % period
            y_in_period = y % period

            # Shelves occupy the interior of each period block
            if (aisle_width <= x_in_period < period and
                aisle_width <= y_in_period < period):
                # Leave some gaps for cross-aisles
                if not (y % (period * 2) < aisle_width):
                    obstacles.add((x, y))

    return WarehouseGraph.from_grid(width, height, obstacles)


# Example usage
if __name__ == "__main__":
    # Create a small warehouse
    warehouse = create_warehouse_graph(20, 20, aisle_width=2, shelf_width=2)
    print(f"Warehouse: {warehouse.num_vertices} vertices, {len(warehouse.edges)} edges")
    print(f"Obstacles: {len(warehouse.obstacles)}")

    # Create time-expanded graph
    starts = frozenset([0, 1, 2, 3, 4])
    goals = frozenset([395, 396, 397, 398, 399])

    te_graph = TimeExpandedGraph(
        base_graph=warehouse,
        horizon=30,
        starts=starts,
        goals=goals
    )

    stats = te_graph.statistics()
    print(f"Time-expanded graph: {stats['num_te_nodes']} nodes, {stats['num_te_edges']} edges")
    print(f"Fingerprint: {te_graph.fingerprint()}")
