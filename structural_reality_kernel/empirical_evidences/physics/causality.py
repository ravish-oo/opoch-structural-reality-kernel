"""
causality.py - Causality as event dependency poset.

Causality is not assumed; it is the dependency structure of records.

H = (E, <) where e1 < e2 iff e2 depends on e1.

Events cannot be globally ordered unless that order is recorded.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON


@dataclass
class RecordEvent:
    """
    A record event (observation/commit).

    An event is a test applied with observed outcome.
    """
    event_id: str
    test_id: str
    outcome: Any
    timestamp: int  # Logical timestamp (order of recording)

    def fingerprint(self) -> str:
        canonical = CanonicalJSON.serialize({
            "event_id": self.event_id,
            "test_id": self.test_id,
            "outcome": str(self.outcome)
        })
        return hashlib.sha256(canonical.encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "RECORD_EVENT",
            "event_id": self.event_id,
            "test_id": self.test_id,
            "outcome": str(self.outcome),
            "timestamp": self.timestamp,
            "fingerprint": self.fingerprint()[:16],
            "result": "PASS"
        }


@dataclass
class DependencyEdge:
    """
    A dependency edge: e1 -> e2 means e2 depends on e1.

    e2 cannot occur without e1 having occurred.
    """
    from_event: str
    to_event: str
    dependency_type: str  # "data" | "causal" | "recorded"

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "DEPENDENCY_EDGE",
            "from": self.from_event,
            "to": self.to_event,
            "dependency_type": self.dependency_type
        })


@dataclass
class CausalityPoset:
    """
    The dependency poset H = (E, <).

    Events form a partial order under dependency.
    """
    events: Dict[str, RecordEvent]
    edges: List[DependencyEdge]

    def __post_init__(self):
        # Build adjacency lists
        self._successors: Dict[str, Set[str]] = {e: set() for e in self.events}
        self._predecessors: Dict[str, Set[str]] = {e: set() for e in self.events}

        for edge in self.edges:
            if edge.from_event in self._successors:
                self._successors[edge.from_event].add(edge.to_event)
            if edge.to_event in self._predecessors:
                self._predecessors[edge.to_event].add(edge.from_event)

    def get_successors(self, event_id: str) -> Set[str]:
        """Get direct successors (dependent events)."""
        return self._successors.get(event_id, set())

    def get_predecessors(self, event_id: str) -> Set[str]:
        """Get direct predecessors (dependencies)."""
        return self._predecessors.get(event_id, set())

    def depends_on(self, e1: str, e2: str) -> bool:
        """Does e1 depend on e2 (is e2 an ancestor of e1)?"""
        if e1 == e2:
            return False

        # BFS to find if e2 is reachable going backwards from e1
        visited = set()
        queue = [e1]

        while queue:
            current = queue.pop(0)
            if current in visited:
                continue
            visited.add(current)

            preds = self.get_predecessors(current)
            if e2 in preds:
                return True
            queue.extend(preds)

        return False

    def are_independent(self, e1: str, e2: str) -> bool:
        """Are e1 and e2 causally independent (no path between them)?"""
        return not self.depends_on(e1, e2) and not self.depends_on(e2, e1)

    def get_independent_pairs(self) -> List[Tuple[str, str]]:
        """Get all pairs of independent events."""
        pairs = []
        event_ids = list(self.events.keys())

        for i, e1 in enumerate(event_ids):
            for e2 in event_ids[i+1:]:
                if self.are_independent(e1, e2):
                    pairs.append((e1, e2))

        return pairs

    def is_acyclic(self) -> bool:
        """Check if the poset is acyclic (valid partial order)."""
        # Use DFS to detect cycles
        visited = set()
        rec_stack = set()

        def dfs(node: str) -> bool:
            visited.add(node)
            rec_stack.add(node)

            for succ in self.get_successors(node):
                if succ not in visited:
                    if dfs(succ):
                        return True
                elif succ in rec_stack:
                    return True

            rec_stack.remove(node)
            return False

        for event_id in self.events:
            if event_id not in visited:
                if dfs(event_id):
                    return False

        return True

    def topological_order(self) -> List[str]:
        """Get a valid topological ordering of events."""
        if not self.is_acyclic():
            return []

        in_degree = {e: 0 for e in self.events}
        for edge in self.edges:
            if edge.to_event in in_degree:
                in_degree[edge.to_event] += 1

        queue = [e for e, d in in_degree.items() if d == 0]
        order = []

        while queue:
            node = queue.pop(0)
            order.append(node)

            for succ in self.get_successors(node):
                in_degree[succ] -= 1
                if in_degree[succ] == 0:
                    queue.append(succ)

        return order

    def get_causal_cone(self, event_id: str) -> Set[str]:
        """
        Get the causal future cone of an event.

        All events that can be influenced by this event.
        """
        cone = set()
        queue = [event_id]

        while queue:
            current = queue.pop(0)
            if current in cone:
                continue
            cone.add(current)
            queue.extend(self.get_successors(current))

        return cone

    def get_causal_past(self, event_id: str) -> Set[str]:
        """
        Get the causal past of an event.

        All events that this event depends on.
        """
        past = set()
        queue = [event_id]

        while queue:
            current = queue.pop(0)
            if current in past:
                continue
            past.add(current)
            queue.extend(self.get_predecessors(current))

        return past

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CAUSALITY_POSET",
            "event_count": len(self.events),
            "edge_count": len(self.edges),
            "is_acyclic": self.is_acyclic()
        })

    def to_receipt(self) -> Dict[str, Any]:
        independent_pairs = self.get_independent_pairs()

        return {
            "type": "CAUSALITY_POSET",
            "event_count": len(self.events),
            "dependency_edges": len(self.edges),
            "is_acyclic": self.is_acyclic(),
            "independent_pairs_count": len(independent_pairs),
            "result": "PASS" if self.is_acyclic() else "FAIL"
        }


class CausalityChecker:
    """
    Checks causal properties and swap invariance.
    """

    def __init__(self, poset: CausalityPoset):
        self.poset = poset

    def verify_swap_invariance(
        self,
        e1: str,
        e2: str,
        compute_fingerprint: callable
    ) -> Tuple[bool, Dict[str, Any]]:
        """
        Verify that swapping independent events doesn't change Pi* fingerprint.

        If e1 and e2 are independent, their order is gauge.
        """
        if not self.poset.are_independent(e1, e2):
            return True, {
                "e1": e1,
                "e2": e2,
                "are_independent": False,
                "swap_invariant": True,  # N/A for dependent events
                "note": "Events are dependent, swap not applicable"
            }

        # Compute fingerprint with e1 before e2
        fp_e1_first = compute_fingerprint(e1, e2)

        # Compute fingerprint with e2 before e1
        fp_e2_first = compute_fingerprint(e2, e1)

        # Should be equal for independent events
        is_invariant = (fp_e1_first == fp_e2_first)

        return is_invariant, {
            "e1": e1,
            "e2": e2,
            "are_independent": True,
            "fingerprint_e1_first": fp_e1_first[:16] if fp_e1_first else "N/A",
            "fingerprint_e2_first": fp_e2_first[:16] if fp_e2_first else "N/A",
            "swap_invariant": is_invariant
        }

    def verify_all_swap_invariance(
        self,
        compute_fingerprint: callable
    ) -> Tuple[bool, List[Dict[str, Any]]]:
        """Verify swap invariance for all independent pairs."""
        receipts = []
        all_invariant = True

        independent_pairs = self.poset.get_independent_pairs()

        for e1, e2 in independent_pairs:
            ok, receipt = self.verify_swap_invariance(e1, e2, compute_fingerprint)
            if not ok:
                all_invariant = False
            receipts.append(receipt)

        return all_invariant, receipts


def create_linear_poset(events: List[RecordEvent]) -> CausalityPoset:
    """Create a linear (totally ordered) poset from events."""
    event_dict = {e.event_id: e for e in events}

    edges = []
    for i in range(len(events) - 1):
        edges.append(DependencyEdge(
            from_event=events[i].event_id,
            to_event=events[i + 1].event_id,
            dependency_type="recorded"
        ))

    return CausalityPoset(events=event_dict, edges=edges)


def create_partial_poset(
    events: List[RecordEvent],
    dependencies: List[Tuple[str, str]]
) -> CausalityPoset:
    """
    Create a partial order poset from explicit dependencies.

    dependencies: list of (from_event_id, to_event_id) pairs
    """
    event_dict = {e.event_id: e for e in events}

    edges = [
        DependencyEdge(
            from_event=from_e,
            to_event=to_e,
            dependency_type="causal"
        )
        for from_e, to_e in dependencies
    ]

    return CausalityPoset(events=event_dict, edges=edges)
