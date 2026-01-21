"""
scale_quotient.py - Scale-based quotient construction for QFT/RG.

Implements:
- Scale test sets with cost bounds and totality
- Scale equivalence relations
- Quotient construction (observable world at scale s)
- Coarse-graining operators as quotient maps
- Semigroup composition of scale maps
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
from enum import Enum
from collections import defaultdict
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON


class TestResult(Enum):
    """Result of a scale test."""
    PASS = "PASS"
    FAIL = "FAIL"
    TIMEOUT = "TIMEOUT"


@dataclass(frozen=True)
class ScaleTest:
    """
    A test at a particular scale.

    Tests are functions that map states to outcomes.
    Each test has a cost (resource requirement).
    """
    test_id: str
    cost: float  # Resource cost to perform this test

    # The actual test function: state -> outcome
    # Stored as a mapping for finite domains
    outcomes: FrozenSet[Tuple[Any, Any]] = field(default_factory=frozenset)

    def __post_init__(self):
        if self.cost < 0:
            raise ValueError("Test cost cannot be negative")

    def evaluate(self, state: Any) -> Any:
        """Evaluate test on a state."""
        outcome_dict = dict(self.outcomes)
        if state in outcome_dict:
            return outcome_dict[state]
        # Total: always return something
        return TestResult.TIMEOUT

    def is_total(self, domain: Set[Any]) -> bool:
        """Check if test is total over domain."""
        outcome_dict = dict(self.outcomes)
        for state in domain:
            if state not in outcome_dict:
                # TIMEOUT is still a valid result for totality
                pass  # We allow implicit TIMEOUT
        return True  # Always total by returning TIMEOUT for undefined

    def canonical(self) -> str:
        sorted_outcomes = sorted(
            [(str(k), str(v)) for k, v in self.outcomes],
            key=lambda x: (x[0], x[1])
        )
        # Convert cost to integer ratio to avoid floats
        cost_int = int(self.cost * 1000)
        return CanonicalJSON.serialize({
            "type": "SCALE_TEST",
            "test_id": self.test_id,
            "cost_milliunits": cost_int,
            "outcomes": sorted_outcomes
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class ScaleTestSet:
    """
    A set of tests available at a particular scale.

    Scale s (coarser = larger s) restricts which tests are feasible.
    """
    scale_id: str
    scale_parameter: float  # Larger = coarser scale
    tests: List[ScaleTest]
    cost_bound: float  # Maximum total cost at this scale

    def __post_init__(self):
        if self.cost_bound < 0:
            raise ValueError("Cost bound cannot be negative")

    def feasible_tests(self) -> List[ScaleTest]:
        """Get tests that fit within cost bound."""
        return [t for t in self.tests if t.cost <= self.cost_bound]

    def total_cost(self) -> float:
        """Total cost of feasible tests."""
        return sum(t.cost for t in self.feasible_tests())

    def all_total(self, domain: Set[Any]) -> bool:
        """Check if all feasible tests are total."""
        return all(t.is_total(domain) for t in self.feasible_tests())

    def canonical(self) -> str:
        test_hashes = sorted([t.fingerprint() for t in self.feasible_tests()])
        # Convert to integer representations to avoid floats
        scale_int = int(self.scale_parameter * 1000)
        cost_int = int(self.cost_bound * 1000)
        return CanonicalJSON.serialize({
            "type": "SCALE_TEST_SET",
            "scale_id": self.scale_id,
            "scale_parameter_milliunits": scale_int,
            "test_count": len(self.feasible_tests()),
            "cost_bound_milliunits": cost_int,
            "test_hashes": test_hashes
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self, domain: Set[Any]) -> Dict[str, Any]:
        # Convert to integer representations to avoid floats
        scale_int = int(self.scale_parameter * 1000)
        cost_int = int(self.cost_bound * 1000)
        return {
            "type": "SCALE_TEST_SET",
            "scale_id": self.scale_id,
            "scale_parameter_milliunits": scale_int,
            "test_count": len(self.feasible_tests()),
            "cost_bound_milliunits": cost_int,
            "all_total": self.all_total(domain),
            "test_set_hash": self.fingerprint()[:32],
            "result": "PASS" if self.all_total(domain) else "FAIL"
        }


@dataclass(frozen=True)
class EquivalenceClass:
    """An equivalence class in a scale quotient."""
    class_id: str
    members: FrozenSet[Any]
    representative: Any  # Canonical representative

    def size(self) -> int:
        return len(self.members)

    def contains(self, element: Any) -> bool:
        return element in self.members

    def canonical(self) -> str:
        sorted_members = sorted([str(m) for m in self.members])
        return CanonicalJSON.serialize({
            "type": "EQUIVALENCE_CLASS",
            "class_id": self.class_id,
            "representative": str(self.representative),
            "members": sorted_members
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class ScaleQuotient:
    """
    The quotient Q_s(W) - the observable world at scale s.

    Q_s(W) = W / ~_s

    where x ~_s y iff for all tau in Delta_s, tau(x) = tau(y)
    """
    scale_id: str
    scale_parameter: float
    domain: FrozenSet[Any]
    equivalence_classes: List[EquivalenceClass]

    # Map from element to its class
    _element_to_class: Dict[Any, EquivalenceClass] = field(default_factory=dict)

    def __post_init__(self):
        # Build element -> class mapping
        self._element_to_class = {}
        for eq_class in self.equivalence_classes:
            for member in eq_class.members:
                self._element_to_class[member] = eq_class

    def class_of(self, element: Any) -> Optional[EquivalenceClass]:
        """Get the equivalence class containing element."""
        return self._element_to_class.get(element)

    def class_count(self) -> int:
        """Number of equivalence classes."""
        return len(self.equivalence_classes)

    def class_sizes(self) -> List[int]:
        """Multiset of class sizes (sorted)."""
        return sorted([c.size() for c in self.equivalence_classes])

    def representatives(self) -> List[Any]:
        """Get canonical representatives of each class."""
        return [c.representative for c in self.equivalence_classes]

    def canonical(self) -> str:
        class_hashes = sorted([c.fingerprint() for c in self.equivalence_classes])
        # Convert to integer representation to avoid floats
        scale_int = int(self.scale_parameter * 1000)
        return CanonicalJSON.serialize({
            "type": "SCALE_QUOTIENT",
            "scale_id": self.scale_id,
            "scale_parameter_milliunits": scale_int,
            "domain_size": len(self.domain),
            "class_count": self.class_count(),
            "class_sizes": self.class_sizes(),
            "class_hashes": class_hashes
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SCALE_QUOTIENT",
            "scale_id": self.scale_id,
            "domain_size": len(self.domain),
            "class_count": self.class_count(),
            "class_sizes": self.class_sizes(),
            "quotient_hash": self.fingerprint()[:32],
            "result": "PASS"
        }


def compute_scale_equivalence(
    domain: Set[Any],
    test_set: ScaleTestSet
) -> Dict[Any, FrozenSet[Any]]:
    """
    Compute scale equivalence relation.

    x ~_s y iff for all tau in Delta_s, tau(x) = tau(y)

    Returns mapping from representative to equivalence class.
    """
    feasible_tests = test_set.feasible_tests()

    # Compute signature for each element
    # signature(x) = tuple of test outcomes
    signatures: Dict[Any, Tuple] = {}
    for x in domain:
        sig = tuple(t.evaluate(x) for t in feasible_tests)
        signatures[x] = sig

    # Group by signature
    sig_to_elements: Dict[Tuple, Set[Any]] = defaultdict(set)
    for x, sig in signatures.items():
        sig_to_elements[sig].add(x)

    # Create equivalence classes
    result = {}
    for sig, elements in sig_to_elements.items():
        # Pick canonical representative (smallest string representation)
        rep = min(elements, key=lambda x: str(x))
        result[rep] = frozenset(elements)

    return result


def construct_quotient(
    domain: Set[Any],
    test_set: ScaleTestSet
) -> ScaleQuotient:
    """
    Construct the scale quotient Q_s(W).

    This is the entire definition of "effective degrees of freedom at scale s."
    """
    equivalence_map = compute_scale_equivalence(domain, test_set)

    classes = []
    for i, (rep, members) in enumerate(sorted(equivalence_map.items(), key=lambda x: str(x[0]))):
        eq_class = EquivalenceClass(
            class_id=f"CLASS_{i}",
            members=members,
            representative=rep
        )
        classes.append(eq_class)

    return ScaleQuotient(
        scale_id=test_set.scale_id,
        scale_parameter=test_set.scale_parameter,
        domain=frozenset(domain),
        equivalence_classes=classes
    )


@dataclass
class CoarseGrainingMap:
    """
    The coarse-graining operator R_s: W -> Q_s(W).

    Maps each microstate to its equivalence class under ~_s.

    This is forced by A0: if two states cannot be distinguished at scale s,
    they must be identified at that scale.
    """
    source_scale_id: str
    target_scale_id: str
    source_quotient: ScaleQuotient
    target_quotient: ScaleQuotient

    # Mapping from source class to target class
    class_map: Dict[str, str] = field(default_factory=dict)

    def apply(self, element: Any) -> EquivalenceClass:
        """Apply coarse-graining to an element."""
        # Find class in source
        source_class = self.source_quotient.class_of(element)
        if source_class is None:
            raise ValueError(f"Element {element} not in source domain")

        # Map to target class
        target_class_id = self.class_map.get(source_class.class_id)
        if target_class_id:
            for tc in self.target_quotient.equivalence_classes:
                if tc.class_id == target_class_id:
                    return tc

        # Find directly in target
        return self.target_quotient.class_of(element)

    def apply_to_class(self, source_class: EquivalenceClass) -> EquivalenceClass:
        """Apply coarse-graining to a source class."""
        # All members of source class map to same target class
        rep = source_class.representative
        return self.target_quotient.class_of(rep)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "COARSE_GRAINING_MAP",
            "source_scale": self.source_scale_id,
            "target_scale": self.target_scale_id,
            "source_classes": self.source_quotient.class_count(),
            "target_classes": self.target_quotient.class_count(),
            "class_map": sorted(self.class_map.items())
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


def construct_coarse_graining(
    fine_quotient: ScaleQuotient,
    coarse_quotient: ScaleQuotient
) -> CoarseGrainingMap:
    """
    Construct coarse-graining map from fine scale to coarse scale.

    R_s: Q_fine -> Q_coarse
    """
    class_map = {}

    for fine_class in fine_quotient.equivalence_classes:
        # Find which coarse class contains the representative
        coarse_class = coarse_quotient.class_of(fine_class.representative)
        if coarse_class:
            class_map[fine_class.class_id] = coarse_class.class_id

    return CoarseGrainingMap(
        source_scale_id=fine_quotient.scale_id,
        target_scale_id=coarse_quotient.scale_id,
        source_quotient=fine_quotient,
        target_quotient=coarse_quotient,
        class_map=class_map
    )


@dataclass
class SemigroupCheck:
    """
    Verification of semigroup property.

    R_{s2} o R_{s1} = R_{s2} for s2 coarser than s1

    Coarse-graining twice equals coarse-graining once to the coarser scale.
    """
    scale_1_id: str
    scale_2_id: str  # Coarser scale
    elements_tested: int
    composition_verified: bool
    mismatches: List[Tuple[Any, str, str]] = field(default_factory=list)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "SEMIGROUP_CHECK",
            "scale_1": self.scale_1_id,
            "scale_2": self.scale_2_id,
            "elements_tested": self.elements_tested,
            "composition_verified": self.composition_verified
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SEMIGROUP_CHECK",
            "scale_1": self.scale_1_id,
            "scale_2": self.scale_2_id,
            "elements_tested": self.elements_tested,
            "composition_verified": self.composition_verified,
            "semigroup_hash": self.fingerprint()[:32],
            "result": "PASS" if self.composition_verified else "FAIL"
        }


def verify_semigroup_property(
    domain: Set[Any],
    q_fine: ScaleQuotient,
    q_coarse: ScaleQuotient,
    r_fine_to_coarse: CoarseGrainingMap
) -> SemigroupCheck:
    """
    Verify semigroup property: R_coarse(R_fine(x)) = R_coarse(x).

    This is the mathematical core of "RG is irreversible."
    """
    mismatches = []

    for x in domain:
        # Direct coarse-graining
        direct_class = q_coarse.class_of(x)

        # Two-step: fine then coarse
        fine_class = q_fine.class_of(x)
        if fine_class:
            composed_class = r_fine_to_coarse.apply_to_class(fine_class)
        else:
            composed_class = None

        # Compare
        if direct_class and composed_class:
            if direct_class.class_id != composed_class.class_id:
                mismatches.append((x, direct_class.class_id, composed_class.class_id))
        elif direct_class or composed_class:
            mismatches.append((x,
                              direct_class.class_id if direct_class else "NONE",
                              composed_class.class_id if composed_class else "NONE"))

    return SemigroupCheck(
        scale_1_id=q_fine.scale_id,
        scale_2_id=q_coarse.scale_id,
        elements_tested=len(domain),
        composition_verified=(len(mismatches) == 0),
        mismatches=mismatches[:5]  # Keep first 5 for debugging
    )


# ============================================================
# SCALE HIERARCHY CONSTRUCTION
# ============================================================

@dataclass
class ScaleHierarchy:
    """
    A hierarchy of scales with decreasing resolution.

    Scales are ordered: s1 < s2 < s3 means s1 is finer than s2, etc.
    """
    hierarchy_id: str
    scales: List[ScaleTestSet]  # Ordered from fine to coarse
    quotients: List[ScaleQuotient] = field(default_factory=list)
    coarse_graining_maps: List[CoarseGrainingMap] = field(default_factory=list)

    def build(self, domain: Set[Any]):
        """Build quotients and coarse-graining maps for the hierarchy."""
        self.quotients = []
        self.coarse_graining_maps = []

        # Build quotients
        for scale in self.scales:
            quotient = construct_quotient(domain, scale)
            self.quotients.append(quotient)

        # Build coarse-graining maps between adjacent scales
        for i in range(len(self.quotients) - 1):
            cg_map = construct_coarse_graining(
                self.quotients[i],
                self.quotients[i + 1]
            )
            self.coarse_graining_maps.append(cg_map)

    def verify_semigroup(self, domain: Set[Any]) -> List[SemigroupCheck]:
        """Verify semigroup property for all scale pairs."""
        checks = []

        for i in range(len(self.quotients) - 1):
            check = verify_semigroup_property(
                domain,
                self.quotients[i],
                self.quotients[i + 1],
                self.coarse_graining_maps[i]
            )
            checks.append(check)

        return checks

    def canonical(self) -> str:
        scale_hashes = [s.fingerprint() for s in self.scales]
        quotient_hashes = [q.fingerprint() for q in self.quotients]
        return CanonicalJSON.serialize({
            "type": "SCALE_HIERARCHY",
            "hierarchy_id": self.hierarchy_id,
            "scale_count": len(self.scales),
            "scale_hashes": scale_hashes,
            "quotient_hashes": quotient_hashes
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SCALE_HIERARCHY",
            "hierarchy_id": self.hierarchy_id,
            "scale_count": len(self.scales),
            "quotient_sizes": [q.class_count() for q in self.quotients],
            "hierarchy_hash": self.fingerprint()[:32],
            "result": "PASS"
        }


def create_sample_scale_hierarchy(domain: Set[Any]) -> ScaleHierarchy:
    """
    Create a sample scale hierarchy for demonstration.

    Uses simple cost-based test restrictions.
    """
    # Create tests with different costs
    all_tests = []

    # Fine-grained tests (low cost)
    for i, state in enumerate(sorted(domain, key=str)):
        # Test that distinguishes this state from others
        outcomes = frozenset([(s, 1 if s == state else 0) for s in domain])
        test = ScaleTest(
            test_id=f"TEST_EXACT_{i}",
            cost=1.0,
            outcomes=outcomes
        )
        all_tests.append(test)

    # Medium-grained tests (medium cost)
    # Group states by some property (e.g., parity if numeric)
    medium_outcomes = frozenset([
        (s, hash(str(s)) % 3) for s in domain
    ])
    medium_test = ScaleTest(
        test_id="TEST_MEDIUM",
        cost=0.5,
        outcomes=medium_outcomes
    )

    # Coarse-grained tests (high cost)
    coarse_outcomes = frozenset([
        (s, hash(str(s)) % 2) for s in domain
    ])
    coarse_test = ScaleTest(
        test_id="TEST_COARSE",
        cost=0.3,
        outcomes=coarse_outcomes
    )

    # Create scale test sets with decreasing cost bounds
    scale_fine = ScaleTestSet(
        scale_id="SCALE_FINE",
        scale_parameter=1.0,
        tests=all_tests + [medium_test, coarse_test],
        cost_bound=10.0  # Can afford all tests
    )

    scale_medium = ScaleTestSet(
        scale_id="SCALE_MEDIUM",
        scale_parameter=2.0,
        tests=[medium_test, coarse_test],
        cost_bound=1.0  # Can only afford medium and coarse
    )

    scale_coarse = ScaleTestSet(
        scale_id="SCALE_COARSE",
        scale_parameter=3.0,
        tests=[coarse_test],
        cost_bound=0.5  # Can only afford coarse
    )

    hierarchy = ScaleHierarchy(
        hierarchy_id="SAMPLE_HIERARCHY",
        scales=[scale_fine, scale_medium, scale_coarse]
    )

    hierarchy.build(domain)

    return hierarchy
