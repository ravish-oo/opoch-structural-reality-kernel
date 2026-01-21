"""
object_equality/equivalence.py - Equivalence relations from test indistinguishability.

Implements:
- x ~_Δ y iff ∀τ∈Δ, τ(x)=τ(y)
- Equivalence classes [x] = {y : y ~_Δ x}
- Quotient D₀/~_Δ
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import json

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import Test
from core.receipts import CanonicalJSON


@dataclass
class TestSignature:
    """
    The signature of an element under a test set.

    sig(x) = (τ₁(x), τ₂(x), ..., τₙ(x))
    """
    element: Any
    outcomes: Tuple[Tuple[str, Any], ...]  # Sorted (test_id, outcome) pairs

    def __hash__(self) -> int:
        return hash(self.outcomes)

    def __eq__(self, other: 'TestSignature') -> bool:
        return self.outcomes == other.outcomes

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "element": str(self.element),
            "outcomes": list(self.outcomes)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()[:16]


class TestIndistinguishability:
    """
    Implements the equivalence relation x ~_Δ y.

    Two elements are indistinguishable iff all tests in Δ give the same outcome.
    """

    def __init__(self, d0: FrozenSet[Any], tests: Dict[str, Test]):
        """
        Initialize with domain D₀ and test set Δ.

        Args:
            d0: Finite working domain
            tests: Available tests (the set Δ)
        """
        self.d0 = d0
        self.tests = tests
        self._signatures: Dict[Any, TestSignature] = {}
        self._compute_signatures()

    def _compute_signatures(self) -> None:
        """Compute signature for each element."""
        sorted_test_ids = sorted(self.tests.keys())

        for x in self.d0:
            outcomes = []
            for test_id in sorted_test_ids:
                test = self.tests[test_id]
                outcome = test(x)
                outcomes.append((test_id, outcome))

            self._signatures[x] = TestSignature(
                element=x,
                outcomes=tuple(outcomes)
            )

    def signature(self, x: Any) -> TestSignature:
        """Get the test signature of an element."""
        return self._signatures[x]

    def are_indistinguishable(self, x: Any, y: Any) -> bool:
        """
        Check if x ~_Δ y.

        x and y are indistinguishable iff they have the same signature.
        """
        return self._signatures[x].outcomes == self._signatures[y].outcomes

    def check_reflexive(self) -> Tuple[bool, Dict[str, Any]]:
        """
        Verify reflexivity: ∀x, x ~_Δ x.

        Always true by definition (same element has same outcomes).
        """
        violations = []
        for x in self.d0:
            if not self.are_indistinguishable(x, x):
                violations.append(str(x))

        passed = len(violations) == 0
        return passed, {
            "property": "reflexive",
            "elements_checked": len(self.d0),
            "violations": violations,
            "passed": passed
        }

    def check_symmetric(self) -> Tuple[bool, Dict[str, Any]]:
        """
        Verify symmetry: x ~_Δ y ⟹ y ~_Δ x.

        Always true by definition (equality of outcomes is symmetric).
        """
        violations = []
        pairs_checked = 0

        elements = list(self.d0)
        for i, x in enumerate(elements):
            for y in elements[i+1:]:
                pairs_checked += 1
                xy = self.are_indistinguishable(x, y)
                yx = self.are_indistinguishable(y, x)
                if xy != yx:
                    violations.append((str(x), str(y)))

        passed = len(violations) == 0
        return passed, {
            "property": "symmetric",
            "pairs_checked": pairs_checked,
            "violations": violations,
            "passed": passed
        }

    def check_transitive(self) -> Tuple[bool, Dict[str, Any]]:
        """
        Verify transitivity: x ~_Δ y ∧ y ~_Δ z ⟹ x ~_Δ z.

        Always true by definition (equality of outcomes is transitive).
        """
        violations = []
        triples_checked = 0

        elements = list(self.d0)
        n = len(elements)

        # For efficiency, only check representative triples
        # Full check would be O(n³)
        max_triples = min(n * n * n, 100000)

        for i, x in enumerate(elements):
            for j, y in enumerate(elements):
                if i == j:
                    continue
                if not self.are_indistinguishable(x, y):
                    continue

                for k, z in enumerate(elements):
                    if j == k:
                        continue
                    if triples_checked >= max_triples:
                        break

                    triples_checked += 1

                    if self.are_indistinguishable(y, z):
                        if not self.are_indistinguishable(x, z):
                            violations.append((str(x), str(y), str(z)))

                if triples_checked >= max_triples:
                    break
            if triples_checked >= max_triples:
                break

        passed = len(violations) == 0
        return passed, {
            "property": "transitive",
            "triples_checked": triples_checked,
            "violations": violations,
            "passed": passed
        }

    def verify_equivalence_relation(self) -> Dict[str, Any]:
        """Verify all equivalence relation properties."""
        ref_passed, ref_details = self.check_reflexive()
        sym_passed, sym_details = self.check_symmetric()
        trans_passed, trans_details = self.check_transitive()

        all_passed = ref_passed and sym_passed and trans_passed

        return {
            "type": "EQUIVALENCE_RELATION",
            "domain_size": len(self.d0),
            "test_count": len(self.tests),
            "checks": {
                "reflexive": ref_details,
                "symmetric": sym_details,
                "transitive": trans_details
            },
            "all_passed": all_passed,
            "result": "PASS" if all_passed else "FAIL"
        }


@dataclass
class EquivalenceClass:
    """
    An equivalence class [x] = {y : y ~_Δ x}.

    This is what an "object" actually is.
    """
    representative: Any
    members: FrozenSet[Any]
    signature: TestSignature

    @property
    def size(self) -> int:
        return len(self.members)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "representative": str(self.representative),
            "size": self.size,
            "signature_fp": self.signature.fingerprint()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()[:16]


class Quotient:
    """
    The quotient D₀/~_Δ.

    This is "the world of objects" - the only admissible reality.
    """

    def __init__(self, indistinguishability: TestIndistinguishability):
        """
        Construct quotient from indistinguishability relation.
        """
        self.indist = indistinguishability
        self._classes: List[EquivalenceClass] = []
        self._element_to_class: Dict[Any, EquivalenceClass] = {}
        self._compute_quotient()

    def _compute_quotient(self) -> None:
        """Compute the quotient by grouping elements by signature."""
        signature_groups: Dict[Tuple, List[Any]] = {}

        for x in self.indist.d0:
            sig = self.indist.signature(x)
            key = sig.outcomes
            if key not in signature_groups:
                signature_groups[key] = []
            signature_groups[key].append(x)

        # Build equivalence classes
        for sig_key, members in signature_groups.items():
            # Sort for deterministic representative selection
            sorted_members = sorted(members, key=str)
            representative = sorted_members[0]
            sig = self.indist.signature(representative)

            eq_class = EquivalenceClass(
                representative=representative,
                members=frozenset(members),
                signature=sig
            )
            self._classes.append(eq_class)

            for m in members:
                self._element_to_class[m] = eq_class

        # Sort classes for deterministic ordering
        self._classes.sort(key=lambda c: c.fingerprint())

    @property
    def classes(self) -> List[EquivalenceClass]:
        """Get all equivalence classes."""
        return self._classes

    def class_of(self, x: Any) -> EquivalenceClass:
        """Get the equivalence class containing x."""
        return self._element_to_class[x]

    def project(self, x: Any) -> Any:
        """
        The projection map π(x) = [x].

        Returns the representative of x's class.
        """
        return self._element_to_class[x].representative

    def class_count(self) -> int:
        """Number of equivalence classes."""
        return len(self._classes)

    def class_sizes(self) -> List[int]:
        """Sorted list of class sizes (multiset)."""
        return sorted([c.size for c in self._classes], reverse=True)

    def canonical(self) -> str:
        """Canonical representation (gauge-invariant)."""
        return CanonicalJSON.serialize({
            "class_count": self.class_count(),
            "class_sizes": self.class_sizes(),
            "domain_size": len(self.indist.d0)
        })

    def fingerprint(self) -> str:
        """SHA-256 fingerprint of quotient (gauge-invariant)."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        """Generate canonical receipt for quotient construction."""
        return {
            "type": "QUOTIENT_CONSTRUCTION",
            "domain_size": len(self.indist.d0),
            "class_count": self.class_count(),
            "class_sizes": self.class_sizes(),
            "quotient_fingerprint": self.fingerprint()[:32],
            "result": "PASS"
        }


def compute_equivalence_relation(
    d0: FrozenSet[Any],
    tests: Dict[str, Test]
) -> Tuple[TestIndistinguishability, Quotient]:
    """
    Compute the equivalence relation and quotient.

    Returns:
        (indistinguishability relation, quotient)
    """
    indist = TestIndistinguishability(d0, tests)
    quotient = Quotient(indist)
    return indist, quotient
