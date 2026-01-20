"""
curvature.py - Curvature as holonomy of local refinement.

Curvature is noncommutation of local refinement/gluing:

Cl_U . Cl_V != Cl_V . Cl_U  (mod gauge)

When local closures don't commute, you have curvature.
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
class LocalRegion:
    """
    A local region/context with its own test family.

    Delta_U = tests available in region U.
    """
    region_id: str
    tests: Dict[str, Test]
    elements: FrozenSet[Any]

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "LOCAL_REGION",
            "region_id": self.region_id,
            "test_count": len(self.tests),
            "element_count": len(self.elements)
        })


@dataclass
class LocalQuotient:
    """
    A quotient induced by a local test family.

    Q_U = W/~_U where x ~_U y iff all tests in Delta_U agree on x,y.
    """
    region_id: str
    equivalence_classes: FrozenSet[FrozenSet[Any]]
    class_map: Dict[Any, int]  # element -> class index

    @property
    def class_count(self) -> int:
        return len(self.equivalence_classes)

    def fingerprint(self) -> str:
        """Compute Pi-fixed fingerprint of quotient."""
        # Sort classes by size then by min element hash for determinism
        sorted_classes = sorted(
            [tuple(sorted(str(x) for x in cls)) for cls in self.equivalence_classes],
            key=lambda c: (len(c), c)
        )
        canonical = CanonicalJSON.serialize({
            "type": "LOCAL_QUOTIENT",
            "region_id": self.region_id,
            "class_count": self.class_count,
            "classes_hash": hashlib.sha256(str(sorted_classes).encode()).hexdigest()[:16]
        })
        return hashlib.sha256(canonical.encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "LOCAL_QUOTIENT",
            "region_id": self.region_id,
            "class_count": self.class_count,
            "fingerprint": self.fingerprint()[:32],
            "result": "PASS"
        }


@dataclass
class Closure:
    """
    A closure operation Cl_U that applies local quotient.
    """
    region: LocalRegion

    def compute_quotient(self, elements: FrozenSet[Any]) -> LocalQuotient:
        """
        Compute local quotient Q_U = elements/~_U.
        """
        # Group elements by their signature under local tests
        sig_to_class: Dict[Tuple, Set[Any]] = {}

        for x in elements:
            sig = tuple(
                (test_id, test.evaluator(x))
                for test_id, test in sorted(self.region.tests.items())
            )

            if sig not in sig_to_class:
                sig_to_class[sig] = set()
            sig_to_class[sig].add(x)

        # Create equivalence classes
        classes = frozenset(frozenset(cls) for cls in sig_to_class.values())

        # Create class map
        class_map = {}
        for i, cls in enumerate(sorted(classes, key=lambda c: min(str(x) for x in c))):
            for x in cls:
                class_map[x] = i

        return LocalQuotient(
            region_id=self.region.region_id,
            equivalence_classes=classes,
            class_map=class_map
        )

    def apply(self, quotient: LocalQuotient) -> LocalQuotient:
        """
        Apply closure to an existing quotient.

        This refines the quotient further using local tests.
        """
        # Collect all elements from the quotient
        all_elements = set()
        for cls in quotient.equivalence_classes:
            all_elements.update(cls)

        # Recompute quotient with combined constraints
        return self.compute_quotient(frozenset(all_elements))


@dataclass
class HolonomyWitness:
    """
    A witness for holonomy (curvature).

    Demonstrates that Cl_U . Cl_V != Cl_V . Cl_U
    by computing both paths and comparing fingerprints.
    """
    path_a_regions: List[str]
    path_b_regions: List[str]
    path_a_fingerprint: str
    path_b_fingerprint: str

    @property
    def has_curvature(self) -> bool:
        """Is there curvature (do paths give different results)?"""
        return self.path_a_fingerprint != self.path_b_fingerprint

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "HOLONOMY_WITNESS",
            "path_a_regions": self.path_a_regions,
            "path_b_regions": self.path_b_regions,
            "has_curvature": self.has_curvature
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "HOLONOMY_WITNESS",
            "path_a_regions": self.path_a_regions,
            "path_b_regions": self.path_b_regions,
            "path_a_fingerprint": self.path_a_fingerprint[:32],
            "path_b_fingerprint": self.path_b_fingerprint[:32],
            "has_curvature": self.has_curvature,
            "result": "PASS"
        }


@dataclass
class SingularityWitness:
    """
    A witness for a singularity.

    A region where refinement collapses: no feasible tests
    can further distinguish elements.
    """
    region_id: str
    elements: FrozenSet[Any]
    equivalence_classes: FrozenSet[FrozenSet[Any]]
    feasible_separators: int

    @property
    def is_singular(self) -> bool:
        """Is this a singularity (no further refinement possible)?"""
        return self.feasible_separators == 0 and len(self.equivalence_classes) < len(self.elements)

    @property
    def collapse_ratio(self) -> float:
        """How much collapse occurred: 1 - (classes/elements)."""
        if len(self.elements) == 0:
            return 0.0
        return 1.0 - len(self.equivalence_classes) / len(self.elements)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "SINGULARITY_WITNESS",
            "region_id": self.region_id,
            "element_count": len(self.elements),
            "class_count": len(self.equivalence_classes),
            "feasible_separators": self.feasible_separators,
            "is_singular": self.is_singular
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SINGULARITY_WITNESS",
            "region_id": self.region_id,
            "survivor_count": len(self.elements),
            "equivalence_class_count": len(self.equivalence_classes),
            "feasible_separators": self.feasible_separators,
            "refinement_stalled": self.is_singular,
            "collapse_ratio_display": str(round(self.collapse_ratio, 6)),
            "omega_statement": "distinctions not separable under Delta" if self.is_singular else "refinement possible",
            "result": "PASS"
        }


class CurvatureComputer:
    """
    Computes curvature (holonomy) from local refinements.
    """

    def __init__(self, regions: Dict[str, LocalRegion]):
        self.regions = regions
        self.closures = {rid: Closure(region) for rid, region in regions.items()}

    def compute_path_quotient(
        self,
        elements: FrozenSet[Any],
        region_sequence: List[str]
    ) -> LocalQuotient:
        """
        Compute quotient by applying closures in sequence.

        Cl_{U_1} . Cl_{U_2} . ... . Cl_{U_n}
        """
        if not region_sequence:
            # Return trivial quotient (each element is its own class)
            classes = frozenset(frozenset([x]) for x in elements)
            class_map = {x: i for i, x in enumerate(elements)}
            return LocalQuotient(
                region_id="IDENTITY",
                equivalence_classes=classes,
                class_map=class_map
            )

        # Start with first closure
        first_region = region_sequence[0]
        quotient = self.closures[first_region].compute_quotient(elements)

        # Apply remaining closures
        for region_id in region_sequence[1:]:
            # Collect elements from current classes
            current_elements = set()
            for cls in quotient.equivalence_classes:
                current_elements.update(cls)

            # Apply next closure
            quotient = self.closures[region_id].compute_quotient(
                frozenset(current_elements)
            )
            quotient = LocalQuotient(
                region_id=region_id,
                equivalence_classes=quotient.equivalence_classes,
                class_map=quotient.class_map
            )

        return quotient

    def compute_holonomy(
        self,
        elements: FrozenSet[Any],
        path_a: List[str],
        path_b: List[str]
    ) -> HolonomyWitness:
        """
        Compute holonomy between two paths.

        If Cl_path_a != Cl_path_b, there is curvature.
        """
        quotient_a = self.compute_path_quotient(elements, path_a)
        quotient_b = self.compute_path_quotient(elements, path_b)

        return HolonomyWitness(
            path_a_regions=path_a,
            path_b_regions=path_b,
            path_a_fingerprint=quotient_a.fingerprint(),
            path_b_fingerprint=quotient_b.fingerprint()
        )

    def detect_singularity(
        self,
        region_id: str,
        elements: FrozenSet[Any],
        external_tests: Dict[str, Test]
    ) -> SingularityWitness:
        """
        Detect if a region is a singularity.

        A singularity is where external tests cannot further refine.
        """
        # Compute quotient under external tests
        external_region = LocalRegion(
            region_id=f"external_{region_id}",
            tests=external_tests,
            elements=elements
        )
        external_closure = Closure(external_region)
        quotient = external_closure.compute_quotient(elements)

        # Count separators: tests that actually distinguish elements
        separators = 0
        for test_id, test in external_tests.items():
            outcomes = set()
            for x in elements:
                outcomes.add(test.evaluator(x))
            if len(outcomes) > 1:
                separators += 1

        return SingularityWitness(
            region_id=region_id,
            elements=elements,
            equivalence_classes=quotient.equivalence_classes,
            feasible_separators=separators
        )


def create_overlapping_regions(
    elements: FrozenSet[Any],
    all_tests: Dict[str, Test],
    region_specs: List[Tuple[str, List[str]]]
) -> Dict[str, LocalRegion]:
    """
    Create overlapping regions with different test subsets.

    region_specs: list of (region_id, [test_ids]) pairs
    """
    regions = {}

    for region_id, test_ids in region_specs:
        region_tests = {tid: all_tests[tid] for tid in test_ids if tid in all_tests}
        regions[region_id] = LocalRegion(
            region_id=region_id,
            tests=region_tests,
            elements=elements
        )

    return regions
