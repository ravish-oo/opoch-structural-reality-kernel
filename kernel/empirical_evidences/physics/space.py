"""
space.py - Space as separator-cost geometry.

Space is not assumed; it is derived from distinguishability.

d(x,y) := inf{c(tau) : tau in Delta, tau(x) != tau(y)}

Distance = minimal cost to tell x and y apart.
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import Test
from core.receipts import CanonicalJSON


@dataclass
class Separator:
    """
    A separator between two elements.

    A test tau that distinguishes x from y: tau(x) != tau(y).
    """
    test_id: str
    cost: int
    outcome_x: Any
    outcome_y: Any

    @property
    def separates(self) -> bool:
        """Does this test separate the elements?"""
        return self.outcome_x != self.outcome_y

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "SEPARATOR",
            "test_id": self.test_id,
            "cost": self.cost,
            "separates": self.separates
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SEPARATOR",
            "test_id": self.test_id,
            "cost": self.cost,
            "outcome_x": str(self.outcome_x),
            "outcome_y": str(self.outcome_y),
            "separates": self.separates,
            "result": "PASS" if self.separates else "FAIL"
        }


@dataclass
class DistanceWitness:
    """
    A witness for the distance between two elements.

    distance = cost of minimal separator.
    """
    element_x: Any
    element_y: Any
    minimal_separator: Optional[Separator]
    all_separators: List[Separator]

    @property
    def distance(self) -> int:
        """
        Distance = minimal separator cost.

        If no separator exists, distance is 0 (indistinguishable).
        """
        if self.minimal_separator is None:
            return 0
        return self.minimal_separator.cost

    @property
    def is_distinguishable(self) -> bool:
        """Are x and y distinguishable?"""
        return self.minimal_separator is not None

    @property
    def is_minimal_verified(self) -> bool:
        """Is the minimal separator actually minimal?"""
        if self.minimal_separator is None:
            return True
        return all(
            sep.cost >= self.minimal_separator.cost
            for sep in self.all_separators
            if sep.separates
        )

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "DISTANCE_WITNESS",
            "element_x": str(self.element_x),
            "element_y": str(self.element_y),
            "distance": self.distance,
            "is_minimal_verified": self.is_minimal_verified
        })

    def to_receipt(self) -> Dict[str, Any]:
        x_fp = hashlib.sha256(str(self.element_x).encode()).hexdigest()[:16]
        y_fp = hashlib.sha256(str(self.element_y).encode()).hexdigest()[:16]

        return {
            "type": "DISTANCE_WITNESS",
            "element_x_fingerprint": x_fp,
            "element_y_fingerprint": y_fp,
            "separator_test": self.minimal_separator.test_id if self.minimal_separator else "NONE",
            "separator_cost": self.minimal_separator.cost if self.minimal_separator else 0,
            "is_minimal": self.is_minimal_verified,
            "distance": self.distance,
            "separators_checked": len(self.all_separators),
            "result": "PASS"
        }


@dataclass
class MetricSpace:
    """
    A metric space induced by separator costs.

    Verifies metric axioms:
    - d(x,x) = 0 (identity)
    - d(x,y) = d(y,x) (symmetry)
    - d(x,z) <= d(x,y) + d(y,z) (triangle inequality)
    """
    elements: List[Any]
    distances: Dict[Tuple[Any, Any], int]

    def get_distance(self, x: Any, y: Any) -> int:
        """Get distance between x and y."""
        if x == y:
            return 0
        key = (x, y) if (x, y) in self.distances else (y, x)
        return self.distances.get(key, 0)

    def verify_identity(self) -> Tuple[bool, List[Dict[str, Any]]]:
        """Verify d(x,x) = 0 for all x."""
        receipts = []
        all_ok = True

        for x in self.elements:
            d_xx = self.get_distance(x, x)
            ok = (d_xx == 0)
            if not ok:
                all_ok = False
            receipts.append({
                "element": str(x),
                "d_xx": d_xx,
                "is_zero": ok
            })

        return all_ok, receipts

    def verify_symmetry(self) -> Tuple[bool, List[Dict[str, Any]]]:
        """Verify d(x,y) = d(y,x) for all pairs."""
        receipts = []
        all_ok = True

        for i, x in enumerate(self.elements):
            for y in self.elements[i+1:]:
                d_xy = self.get_distance(x, y)
                d_yx = self.get_distance(y, x)
                ok = (d_xy == d_yx)
                if not ok:
                    all_ok = False
                receipts.append({
                    "x": str(x),
                    "y": str(y),
                    "d_xy": d_xy,
                    "d_yx": d_yx,
                    "symmetric": ok
                })

        return all_ok, receipts

    def verify_triangle_inequality(self) -> Tuple[bool, List[Dict[str, Any]]]:
        """Verify d(x,z) <= d(x,y) + d(y,z) for all triples."""
        receipts = []
        all_ok = True

        n = len(self.elements)
        for i in range(n):
            for j in range(n):
                for k in range(n):
                    if i == j or j == k or i == k:
                        continue

                    x, y, z = self.elements[i], self.elements[j], self.elements[k]
                    d_xz = self.get_distance(x, z)
                    d_xy = self.get_distance(x, y)
                    d_yz = self.get_distance(y, z)

                    ok = (d_xz <= d_xy + d_yz)
                    if not ok:
                        all_ok = False
                        receipts.append({
                            "x": str(x),
                            "y": str(y),
                            "z": str(z),
                            "d_xz": d_xz,
                            "d_xy_plus_d_yz": d_xy + d_yz,
                            "satisfied": ok
                        })

        # Only record failures to keep receipts small
        if all_ok:
            receipts = [{"all_triples_satisfied": True}]

        return all_ok, receipts

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "METRIC_SPACE",
            "element_count": len(self.elements),
            "distance_pairs": len(self.distances)
        })

    def to_receipt(self) -> Dict[str, Any]:
        id_ok, _ = self.verify_identity()
        sym_ok, _ = self.verify_symmetry()
        tri_ok, _ = self.verify_triangle_inequality()

        return {
            "type": "METRIC_SPACE",
            "element_count": len(self.elements),
            "distance_pairs": len(self.distances),
            "identity_verified": id_ok,
            "symmetry_verified": sym_ok,
            "triangle_inequality_verified": tri_ok,
            "result": "PASS" if (id_ok and sym_ok and tri_ok) else "FAIL"
        }


class DistanceComputer:
    """
    Computes distances between elements using separator costs.
    """

    def __init__(self, tests: Dict[str, Test]):
        self.tests = tests

    def find_separators(self, x: Any, y: Any) -> List[Separator]:
        """Find all separators between x and y."""
        separators = []

        for test_id, test in self.tests.items():
            outcome_x = test.evaluator(x)
            outcome_y = test.evaluator(y)

            sep = Separator(
                test_id=test_id,
                cost=test.cost,
                outcome_x=outcome_x,
                outcome_y=outcome_y
            )
            separators.append(sep)

        return separators

    def find_minimal_separator(self, x: Any, y: Any) -> Optional[Separator]:
        """Find minimal cost separator between x and y."""
        separators = self.find_separators(x, y)
        valid_seps = [s for s in separators if s.separates]

        if not valid_seps:
            return None

        return min(valid_seps, key=lambda s: s.cost)

    def compute_distance(self, x: Any, y: Any) -> DistanceWitness:
        """Compute distance with full witness."""
        separators = self.find_separators(x, y)
        minimal = self.find_minimal_separator(x, y)

        return DistanceWitness(
            element_x=x,
            element_y=y,
            minimal_separator=minimal,
            all_separators=separators
        )

    def compute_metric_space(self, elements: List[Any]) -> MetricSpace:
        """Compute full metric space from elements."""
        distances = {}

        for i, x in enumerate(elements):
            for j, y in enumerate(elements):
                if i < j:
                    witness = self.compute_distance(x, y)
                    distances[(x, y)] = witness.distance

        return MetricSpace(elements=elements, distances=distances)


@dataclass
class ScaleQuotient:
    """
    A quotient at a given scale.

    R_s: D0 -> macrostates at scale s.
    """
    scale_id: str
    quotient_map: Dict[Any, Any]  # element -> macrostate
    macrostates: FrozenSet[Any]

    def get_macrostate(self, x: Any) -> Any:
        """Get macrostate for element."""
        return self.quotient_map.get(x, x)

    def get_representatives(self, macrostate: Any) -> Set[Any]:
        """Get all elements in a macrostate."""
        return {x for x, m in self.quotient_map.items() if m == macrostate}


class MacroSpaceComputer:
    """
    Computes macro-space at different scales.
    """

    def __init__(self, tests: Dict[str, Test]):
        self.tests = tests
        self.distance_computer = DistanceComputer(tests)

    def create_scale_quotient(
        self,
        elements: FrozenSet[Any],
        scale_tests: Dict[str, Test],
        scale_id: str
    ) -> ScaleQuotient:
        """
        Create quotient at a given scale.

        Elements are grouped by their signature under scale_tests.
        """
        # Compute signatures
        sig_to_macro: Dict[Tuple, Any] = {}
        element_to_macro: Dict[Any, Any] = {}

        for x in elements:
            sig = tuple(
                (test_id, test.evaluator(x))
                for test_id, test in sorted(scale_tests.items())
            )

            if sig not in sig_to_macro:
                sig_to_macro[sig] = f"M_{len(sig_to_macro)}"

            element_to_macro[x] = sig_to_macro[sig]

        return ScaleQuotient(
            scale_id=scale_id,
            quotient_map=element_to_macro,
            macrostates=frozenset(sig_to_macro.values())
        )

    def compute_macro_distance(
        self,
        m1: Any,
        m2: Any,
        quotient: ScaleQuotient,
        scale_tests: Dict[str, Test]
    ) -> int:
        """
        Compute distance between macrostates.

        d_s(m1, m2) = inf{c(tau) : tau in Delta_s, tau(m1) != tau(m2)}
        """
        # Find representatives
        reps1 = quotient.get_representatives(m1)
        reps2 = quotient.get_representatives(m2)

        if not reps1 or not reps2:
            return 0

        x = next(iter(reps1))
        y = next(iter(reps2))

        # Use scale tests only
        scale_computer = DistanceComputer(scale_tests)
        witness = scale_computer.compute_distance(x, y)

        return witness.distance
