"""
Swap Constraint Gadget for MAPF

Mathematical Foundation:
Edge-swap constraint prevents head-on collisions:
  f_t(u→v) + f_t(v→u) ≤ 1

This is enforced via gadget construction:
1. For each undirected edge {u,v} at time t
2. Create auxiliary node aux_{u,v,t}
3. Route both directions through the auxiliary
4. Auxiliary has capacity 1

Alternative: Direct constraint in ILP formulation.

Node Splitting for Vertex Capacity:
- Each (v,t) splits into (v,t,in) and (v,t,out)
- Edge (v,t,in) → (v,t,out) has capacity 1
- Ensures ρ_t(v) ≤ 1
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, FrozenSet
import hashlib
import json

from .time_expanded_graph import (
    TimeExpandedGraph,
    TimeExpandedNode,
    TimeExpandedEdge,
    EdgeType,
    WarehouseGraph,
)


@dataclass(frozen=True)
class SwapGadgetNode:
    """
    Auxiliary node for swap constraint gadget.

    For undirected edge {u,v} at time t:
    - Creates node aux_{min(u,v), max(u,v), t}
    - Both (u,t,out)→(v,t+1,in) and (v,t,out)→(u,t+1,in)
      route through this node with capacity 1
    """
    vertex_low: int   # min(u, v)
    vertex_high: int  # max(u, v)
    time: int

    def canonical_id(self) -> str:
        return f"swap_{self.vertex_low}_{self.vertex_high}_t{self.time}"


@dataclass
class SwapGadgetEdge:
    """Edge in swap gadget construction."""
    source_id: str
    target_id: str
    capacity: int = 1
    cost: int = 0


@dataclass
class SwapConstraintGadget:
    """
    Constructs swap constraint gadgets for time-expanded graph.

    For each undirected edge {u,v} and time t:

    Original edges:
      (u,t,out) → (v,t+1,in)
      (v,t,out) → (u,t+1,in)

    Transformed to:
      (u,t,out) → aux_{u,v,t}     [capacity 1]
      (v,t,out) → aux_{u,v,t}     [capacity 1]
      aux_{u,v,t} → (v,t+1,in)    [capacity 1]
      aux_{u,v,t} → (u,t+1,in)    [capacity 1]

    The auxiliary node has effective capacity 1 (single incoming + single outgoing).
    This prevents both directions from being used simultaneously.

    Note: This gadget construction is sound but may be pessimistic.
    For exact modeling, use ILP with explicit constraints.
    """

    base_graph: WarehouseGraph
    horizon: int
    gadget_nodes: Dict[str, SwapGadgetNode] = field(default_factory=dict)
    gadget_edges: List[SwapGadgetEdge] = field(default_factory=list)

    def build(self):
        """Build all swap gadgets."""
        # Find all undirected edges
        undirected_edges = set()
        for (u, v) in self.base_graph.edges:
            # Check if reverse exists (bidirectional)
            if (v, u) in self.base_graph.edges:
                undirected_edges.add((min(u, v), max(u, v)))

        # Create gadgets for each undirected edge at each time
        for t in range(self.horizon):
            for (u_low, u_high) in undirected_edges:
                self._create_gadget(u_low, u_high, t)

    def _create_gadget(self, u_low: int, u_high: int, t: int):
        """
        Create swap gadget for edge {u_low, u_high} at time t.

        The swap constraint is: f_t(u→v) + f_t(v→u) ≤ 1

        Gadget structure with node splitting (capacity 1 on aux):
        - aux_in: receives flow from both directions
        - aux_out: sends flow to both directions
        - Edge aux_in -> aux_out with capacity 1 (the bottleneck)

        This ensures only ONE of the two crossing flows can pass.
        """
        # Create auxiliary node (we use _in and _out suffixes for the split)
        aux_node = SwapGadgetNode(
            vertex_low=u_low,
            vertex_high=u_high,
            time=t
        )
        self.gadget_nodes[aux_node.canonical_id()] = aux_node

        aux_in_id = aux_node.canonical_id() + "_in"
        aux_out_id = aux_node.canonical_id() + "_out"

        # Create edges
        # From (u_low, t, out) to aux_in
        self.gadget_edges.append(SwapGadgetEdge(
            source_id=f"v{u_low}_t{t}_out",
            target_id=aux_in_id,
            capacity=1,
            cost=0
        ))

        # From (u_high, t, out) to aux_in
        self.gadget_edges.append(SwapGadgetEdge(
            source_id=f"v{u_high}_t{t}_out",
            target_id=aux_in_id,
            capacity=1,
            cost=0
        ))

        # CRITICAL: aux_in -> aux_out with capacity 1 (the bottleneck)
        # This enforces the swap constraint!
        self.gadget_edges.append(SwapGadgetEdge(
            source_id=aux_in_id,
            target_id=aux_out_id,
            capacity=1,  # Only 1 robot can pass through
            cost=0
        ))

        # From aux_out to (u_low, t+1, in)
        self.gadget_edges.append(SwapGadgetEdge(
            source_id=aux_out_id,
            target_id=f"v{u_low}_t{t+1}_in",
            capacity=1,
            cost=1
        ))

        # From aux_out to (u_high, t+1, in)
        self.gadget_edges.append(SwapGadgetEdge(
            source_id=aux_out_id,
            target_id=f"v{u_high}_t{t+1}_in",
            capacity=1,
            cost=1
        ))

    def get_modified_edges(self, original_edges: List[TimeExpandedEdge]
                          ) -> Tuple[List[TimeExpandedEdge], List[SwapGadgetEdge]]:
        """
        Get modified edge list with swap constraints.

        Returns:
            - original_edges with swap edges removed
            - gadget_edges to add
        """
        # Identify edges to remove (those passing through gadgets)
        edges_to_remove = set()
        for gadget_node in self.gadget_nodes.values():
            u_low, u_high, t = gadget_node.vertex_low, gadget_node.vertex_high, gadget_node.time

            # Mark original cross-edges for removal
            edges_to_remove.add(f"v{u_low}_t{t}_out->v{u_high}_t{t+1}_in_move")
            edges_to_remove.add(f"v{u_high}_t{t}_out->v{u_low}_t{t+1}_in_move")

        # Filter original edges
        filtered_edges = [
            e for e in original_edges
            if e.canonical_id() not in edges_to_remove
        ]

        return filtered_edges, self.gadget_edges

    def statistics(self) -> Dict:
        """Return gadget statistics."""
        return {
            "num_gadget_nodes": len(self.gadget_nodes),
            "num_gadget_edges": len(self.gadget_edges),
            "undirected_edges_covered": len(self.gadget_nodes) // self.horizon
        }


@dataclass
class NodeSplitGadget:
    """
    Node splitting for vertex capacity enforcement.

    For each (v, t):
      Split into (v, t, in) and (v, t, out)
      Add edge (v, t, in) → (v, t, out) with capacity 1

    This ensures:
      ρ_t(v) ≤ 1  (vertex capacity constraint)

    Note: This is already implemented in TimeExpandedGraph.
    This class provides utilities for verification.
    """

    def verify_split_structure(self, graph: TimeExpandedGraph) -> Dict:
        """Verify that node splitting is correctly implemented."""
        errors = []
        split_edges_found = 0

        # Check each vertex-time pair
        for v in range(graph.base_graph.num_vertices):
            if v in graph.base_graph.obstacles:
                continue

            for t in range(graph.horizon + 1):
                # Check in-node exists
                in_node = graph._get_node(v, t, is_out=False)
                if in_node is None:
                    errors.append(f"Missing in-node for v={v}, t={t}")
                    continue

                # Check out-node exists
                out_node = graph._get_node(v, t, is_out=True)
                if out_node is None:
                    errors.append(f"Missing out-node for v={v}, t={t}")
                    continue

                # Check split edge exists
                found_split = False
                for edge in graph.get_outgoing(in_node):
                    if (edge.target == out_node and
                        edge.edge_type == EdgeType.NODE_SPLIT and
                        edge.capacity == 1):
                        found_split = True
                        split_edges_found += 1
                        break

                if not found_split:
                    errors.append(f"Missing split edge for v={v}, t={t}")

        return {
            "valid": len(errors) == 0,
            "split_edges_found": split_edges_found,
            "errors": errors[:10]  # First 10 errors
        }


class SwapConstraintVerifier:
    """
    Verifies that swap constraints are satisfied in a flow solution.

    For each undirected edge {u,v} at time t:
      f_t(u→v) + f_t(v→u) ≤ 1

    A violation indicates head-on collision.
    """

    def __init__(self, graph: TimeExpandedGraph):
        self.graph = graph

    def verify(self, flow_assignment: Dict[str, int]) -> Dict:
        """
        Verify swap constraints are satisfied.

        Args:
            flow_assignment: edge_id -> flow value

        Returns:
            Verification result with any violations
        """
        violations = []

        # Find all undirected edges
        undirected_edges = set()
        for (u, v) in self.graph.base_graph.edges:
            if (v, u) in self.graph.base_graph.edges:
                undirected_edges.add((min(u, v), max(u, v)))

        # Check each undirected edge at each time
        for t in range(self.graph.horizon):
            for (u_low, u_high) in undirected_edges:
                # Get flows in both directions
                edge_id_fwd = f"v{u_low}_t{t}_out->v{u_high}_t{t+1}_in"
                edge_id_bwd = f"v{u_high}_t{t}_out->v{u_low}_t{t+1}_in"

                flow_fwd = flow_assignment.get(edge_id_fwd, 0)
                flow_bwd = flow_assignment.get(edge_id_bwd, 0)

                if flow_fwd + flow_bwd > 1:
                    violations.append({
                        "type": "EDGE_SWAP",
                        "vertices": (u_low, u_high),
                        "time": t,
                        "flow_fwd": flow_fwd,
                        "flow_bwd": flow_bwd
                    })

        return {
            "valid": len(violations) == 0,
            "num_violations": len(violations),
            "violations": violations[:10]  # First 10
        }


class VertexCapacityVerifier:
    """
    Verifies vertex capacity constraints in a flow solution.

    For each vertex v at time t:
      ρ_t(v) ≤ 1

    Equivalently, flow through (v,t,in)→(v,t,out) ≤ 1.
    """

    def __init__(self, graph: TimeExpandedGraph):
        self.graph = graph

    def verify(self, flow_assignment: Dict[str, int]) -> Dict:
        """
        Verify vertex capacity constraints.

        Args:
            flow_assignment: edge_id -> flow value

        Returns:
            Verification result with any violations
        """
        violations = []

        for v in range(self.graph.base_graph.num_vertices):
            if v in self.graph.base_graph.obstacles:
                continue

            for t in range(self.graph.horizon + 1):
                # Check flow through split edge
                split_edge_id = f"v{v}_t{t}_in->v{v}_t{t}_out"

                # Find actual flow (may be keyed differently)
                flow = 0
                for edge_id, f in flow_assignment.items():
                    if f"v{v}_t{t}_in" in edge_id and f"v{v}_t{t}_out" in edge_id:
                        flow = f
                        break

                if flow > 1:
                    violations.append({
                        "type": "VERTEX_CAPACITY",
                        "vertex": v,
                        "time": t,
                        "flow": flow
                    })

        return {
            "valid": len(violations) == 0,
            "num_violations": len(violations),
            "violations": violations[:10]
        }


def verify_flow_constraints(graph: TimeExpandedGraph,
                           flow_assignment: Dict[str, int]) -> Dict:
    """
    Verify all flow constraints are satisfied.

    Checks:
    1. Vertex capacity (ρ_t(v) ≤ 1)
    2. Edge swap (f_t(u→v) + f_t(v→u) ≤ 1)

    Returns combined verification result.
    """
    vertex_verifier = VertexCapacityVerifier(graph)
    swap_verifier = SwapConstraintVerifier(graph)

    vertex_result = vertex_verifier.verify(flow_assignment)
    swap_result = swap_verifier.verify(flow_assignment)

    return {
        "valid": vertex_result["valid"] and swap_result["valid"],
        "vertex_capacity": vertex_result,
        "edge_swap": swap_result
    }


# Example usage
if __name__ == "__main__":
    from .time_expanded_graph import WarehouseGraph, TimeExpandedGraph

    # Create test graph
    warehouse = WarehouseGraph.from_grid(5, 5)
    print(f"Warehouse edges: {len(warehouse.edges)}")

    # Count undirected edges
    undirected = set()
    for (u, v) in warehouse.edges:
        if (v, u) in warehouse.edges:
            undirected.add((min(u, v), max(u, v)))
    print(f"Undirected edges: {len(undirected)}")

    # Build swap gadget
    gadget = SwapConstraintGadget(
        base_graph=warehouse,
        horizon=10
    )
    gadget.build()

    print(f"\nSwap gadget statistics: {gadget.statistics()}")
