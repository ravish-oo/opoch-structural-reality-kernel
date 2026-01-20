"""
object_equality/factorization.py - The Factorization Theorem.

Any meaningful computation must factor through the quotient:
    f = f̄ ∘ π

where π(x) = [x] is the projection map.
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import json

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON
from .equivalence import TestIndistinguishability, Quotient, EquivalenceClass


@dataclass
class FactorizationViolation:
    """A violation of the factorization property."""
    x: Any
    y: Any
    fx: Any
    fy: Any
    reason: str

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "x": str(self.x),
            "y": str(self.y),
            "f(x)": str(self.fx),
            "f(y)": str(self.fy),
            "reason": self.reason
        })


class FactorizationChecker:
    """
    Verifies that a function factors through the quotient.

    A function f: D₀ → Y factors through D₀/~_Δ iff:
        x ~_Δ y ⟹ f(x) = f(y)
    """

    def __init__(
        self,
        indist: TestIndistinguishability,
        quotient: Quotient
    ):
        self.indist = indist
        self.quotient = quotient

    def check_factors(
        self,
        f: Callable[[Any], Any],
        function_id: str = "f"
    ) -> Tuple[bool, Dict[str, Any]]:
        """
        Check if function f factors through the quotient.

        Args:
            f: Function D₀ → Y to check
            function_id: Name for the function (for receipts)

        Returns:
            (passed, details_dict)
        """
        violations: List[FactorizationViolation] = []
        pairs_checked = 0

        # For each equivalence class, check f is constant
        for eq_class in self.quotient.classes:
            members = list(eq_class.members)
            if len(members) < 2:
                continue

            # Get f value on representative
            rep = eq_class.representative
            f_rep = f(rep)

            # Check all other members give same value
            for member in members:
                if member == rep:
                    continue
                pairs_checked += 1
                f_member = f(member)

                if f_member != f_rep:
                    violations.append(FactorizationViolation(
                        x=rep,
                        y=member,
                        fx=f_rep,
                        fy=f_member,
                        reason=f"f({rep})={f_rep} but f({member})={f_member}"
                    ))

        passed = len(violations) == 0

        return passed, {
            "type": "FACTORIZATION",
            "function_id": function_id,
            "pairs_checked": pairs_checked,
            "violations": len(violations),
            "violation_details": [v.canonical() for v in violations[:10]],
            "result": "PASS" if passed else "FAIL"
        }

    def compute_factored_function(
        self,
        f: Callable[[Any], Any]
    ) -> 'FactoredFunction':
        """
        Compute the factored function f̄: D₀/~_Δ → Y.

        Only valid if f actually factors through the quotient.
        """
        # Build f̄ by evaluating f on each class representative
        class_values: Dict[str, Any] = {}

        for eq_class in self.quotient.classes:
            rep = eq_class.representative
            value = f(rep)
            class_values[eq_class.fingerprint()] = value

        return FactoredFunction(
            quotient=self.quotient,
            class_values=class_values
        )


@dataclass
class FactoredFunction:
    """
    A function f̄: D₀/~_Δ → Y that operates on equivalence classes.

    This is the "lifted" version of f that only depends on objects,
    not on raw labels.
    """
    quotient: Quotient
    class_values: Dict[str, Any]  # class_fingerprint -> value

    def __call__(self, x: Any) -> Any:
        """Evaluate f̄(π(x)) = f̄([x])."""
        eq_class = self.quotient.class_of(x)
        return self.class_values[eq_class.fingerprint()]

    def evaluate_on_class(self, eq_class: EquivalenceClass) -> Any:
        """Evaluate f̄ directly on an equivalence class."""
        return self.class_values[eq_class.fingerprint()]

    def canonical(self) -> str:
        """Canonical representation."""
        # Sort by fingerprint for determinism
        sorted_values = sorted(self.class_values.items())
        return CanonicalJSON.serialize({
            "class_count": len(self.class_values),
            "values": [(k[:8], str(v)) for k, v in sorted_values]
        })

    def fingerprint(self) -> str:
        """SHA-256 fingerprint."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()


def verify_factorization(
    d0: FrozenSet[Any],
    indist: TestIndistinguishability,
    quotient: Quotient,
    functions: Dict[str, Callable[[Any], Any]]
) -> Dict[str, Any]:
    """
    Verify that all given functions factor through the quotient.

    Args:
        d0: Domain
        indist: Indistinguishability relation
        quotient: The quotient
        functions: Dict of function_id -> function to check

    Returns:
        Verification results dict
    """
    checker = FactorizationChecker(indist, quotient)

    results = {
        "type": "FACTORIZATION_BUNDLE",
        "domain_size": len(d0),
        "class_count": quotient.class_count(),
        "functions_checked": len(functions),
        "function_results": {},
        "all_passed": True
    }

    for func_id, func in functions.items():
        passed, details = checker.check_factors(func, func_id)
        results["function_results"][func_id] = details

        if not passed:
            results["all_passed"] = False

    results["result"] = "PASS" if results["all_passed"] else "FAIL"
    return results


class GaugeInvarianceChecker:
    """
    Verifies gauge invariance: quotient fingerprint unchanged under recoding.
    """

    def __init__(self, d0: FrozenSet[Any], tests: Dict[str, Any]):
        self.d0 = d0
        self.tests = tests

    def check_gauge_invariance(self, seed: int = 42) -> Dict[str, Any]:
        """
        Check that quotient fingerprint is gauge-invariant.

        Applies a random recoding of D₀ labels and verifies the
        quotient fingerprint is unchanged.
        """
        from core.kernel import Test

        # Original quotient
        indist1 = TestIndistinguishability(self.d0, self.tests)
        quotient1 = Quotient(indist1)
        fp1 = quotient1.fingerprint()

        # Create random recoding
        import random
        rng = random.Random(seed)
        elements = list(self.d0)
        shuffled = elements.copy()
        rng.shuffle(shuffled)
        recoding = dict(zip(elements, shuffled))
        inverse_recoding = dict(zip(shuffled, elements))

        # Recoded D₀
        recoded_d0 = frozenset(recoding[x] for x in self.d0)

        # Recode tests
        recoded_tests = {}
        for tid, test in self.tests.items():
            def make_recoded_eval(orig_test, inv_rec):
                def recoded_eval(x):
                    orig_x = inv_rec.get(x, x)
                    return orig_test(orig_x)
                return recoded_eval

            recoded_tests[tid] = Test(
                test_id=tid,
                evaluator=make_recoded_eval(test, inverse_recoding),
                cost=test.cost,
                outcome_space=test.outcome_space
            )

        # Recoded quotient
        indist2 = TestIndistinguishability(recoded_d0, recoded_tests)
        quotient2 = Quotient(indist2)
        fp2 = quotient2.fingerprint()

        match = (fp1 == fp2)

        return {
            "type": "GAUGE_INVARIANCE",
            "original_fingerprint": fp1[:32],
            "recoded_fingerprint": fp2[:32],
            "match": match,
            "original_class_sizes": quotient1.class_sizes(),
            "recoded_class_sizes": quotient2.class_sizes(),
            "result": "PASS" if match else "FAIL"
        }
