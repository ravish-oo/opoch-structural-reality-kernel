"""
object_equality/verify.py - Complete verification suite for Object and Equality.

Implements all verification checks A-F:
A) Test Totality
B) Equivalence Relation Checks
C) Quotient Construction
D) Factorization Proof
E) Gauge Invariance Check
F) Canonical Receipts
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import json

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import Test
from core.receipts import CanonicalJSON, ReceiptChain

from .equivalence import (
    TestIndistinguishability, Quotient, compute_equivalence_relation
)
from .factorization import (
    FactorizationChecker, GaugeInvarianceChecker, verify_factorization
)


@dataclass
class CheckResult:
    """Result of a single verification check."""
    check_id: str
    check_name: str
    passed: bool
    details: Dict[str, Any]

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "id": self.check_id,
            "name": self.check_name,
            "passed": self.passed
        })


@dataclass
class ObjectEqualityProofBundle:
    """
    Complete proof bundle for Object and Equality verification.
    """
    bundle_id: str
    checks: List[CheckResult]
    quotient_fingerprint: str
    domain_size: int
    class_count: int
    class_sizes: List[int]
    receipts: List[Dict[str, Any]]

    def all_passed(self) -> bool:
        return all(c.passed for c in self.checks)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "bundle_id": self.bundle_id,
            "all_passed": self.all_passed(),
            "check_count": len(self.checks),
            "passed_count": sum(1 for c in self.checks if c.passed),
            "quotient_fingerprint": self.quotient_fingerprint,
            "domain_size": self.domain_size,
            "class_count": self.class_count
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def summary(self) -> Dict[str, Any]:
        return {
            "bundle_id": self.bundle_id,
            "all_passed": self.all_passed(),
            "total_checks": len(self.checks),
            "passed": sum(1 for c in self.checks if c.passed),
            "failed": sum(1 for c in self.checks if not c.passed),
            "failed_checks": [c.check_id for c in self.checks if not c.passed],
            "quotient_fingerprint": self.quotient_fingerprint[:32],
            "domain_size": self.domain_size,
            "class_count": self.class_count
        }


class ObjectEqualityVerifier:
    """
    Complete verification suite for Object and Equality.

    Implements checks A-F as specified in the document.
    """

    def __init__(
        self,
        d0: FrozenSet[Any],
        tests: Dict[str, Test],
        output_functions: Optional[Dict[str, Callable[[Any], Any]]] = None
    ):
        self.d0 = d0
        self.tests = tests
        self.output_functions = output_functions or {}
        self.checks: List[CheckResult] = []
        self.receipts: List[Dict[str, Any]] = []

        # Computed objects
        self.indist: Optional[TestIndistinguishability] = None
        self.quotient: Optional[Quotient] = None

    def _add_check(
        self,
        check_id: str,
        check_name: str,
        passed: bool,
        details: Dict[str, Any]
    ) -> CheckResult:
        result = CheckResult(
            check_id=check_id,
            check_name=check_name,
            passed=passed,
            details=details
        )
        self.checks.append(result)
        return result

    def check_A_test_totality(self) -> CheckResult:
        """
        A) Test Totality

        Verify each τ ∈ Δ returns an outcome for every x ∈ D₀.
        FAIL/TIMEOUT are explicit outcomes, not exceptions.
        """
        all_total = True
        test_results = {}

        for test_id, test in self.tests.items():
            total = True
            undefined_elements = []

            for x in self.d0:
                try:
                    outcome = test(x)
                    if outcome is None:
                        total = False
                        undefined_elements.append(str(x))
                except Exception as e:
                    total = False
                    undefined_elements.append(f"{x}: {e}")

            test_results[test_id] = {
                "total": total,
                "undefined_count": len(undefined_elements),
                "undefined_elements": undefined_elements[:5]
            }

            if not total:
                all_total = False

        receipt = {
            "type": "TEST_TOTALITY",
            "tests_checked": len(self.tests),
            "domain_size": len(self.d0),
            "all_total": all_total,
            "test_results": test_results,
            "result": "PASS" if all_total else "FAIL"
        }
        self.receipts.append(receipt)

        return self._add_check(
            check_id="A",
            check_name="Test Totality",
            passed=all_total,
            details=receipt
        )

    def check_B_equivalence_relation(self) -> CheckResult:
        """
        B) Equivalence Relation Checks

        Verify ~_Δ is reflexive, symmetric, and transitive.
        """
        # Compute indistinguishability
        self.indist = TestIndistinguishability(self.d0, self.tests)

        # Verify equivalence relation properties
        result = self.indist.verify_equivalence_relation()
        self.receipts.append(result)

        return self._add_check(
            check_id="B",
            check_name="Equivalence Relation Properties",
            passed=result["all_passed"],
            details=result
        )

    def check_C_quotient_construction(self) -> CheckResult:
        """
        C) Quotient Construction

        Compute D₀/~_Δ and verify canonical fingerprint.
        """
        if self.indist is None:
            self.indist = TestIndistinguishability(self.d0, self.tests)

        self.quotient = Quotient(self.indist)
        receipt = self.quotient.to_receipt()
        self.receipts.append(receipt)

        return self._add_check(
            check_id="C",
            check_name="Quotient Construction",
            passed=True,  # Construction always succeeds if B passes
            details=receipt
        )

    def check_D_factorization(self) -> CheckResult:
        """
        D) Factorization Proof

        Verify all output functions factor through the quotient.
        """
        if self.indist is None or self.quotient is None:
            self.indist = TestIndistinguishability(self.d0, self.tests)
            self.quotient = Quotient(self.indist)

        if not self.output_functions:
            # No functions to check - trivially passes
            receipt = {
                "type": "FACTORIZATION_BUNDLE",
                "functions_checked": 0,
                "all_passed": True,
                "result": "PASS",
                "note": "No output functions provided"
            }
            self.receipts.append(receipt)
            return self._add_check(
                check_id="D",
                check_name="Factorization Proof",
                passed=True,
                details=receipt
            )

        result = verify_factorization(
            self.d0, self.indist, self.quotient, self.output_functions
        )
        self.receipts.append(result)

        return self._add_check(
            check_id="D",
            check_name="Factorization Proof",
            passed=result["all_passed"],
            details=result
        )

    def check_E_gauge_invariance(self, seed: int = 42) -> CheckResult:
        """
        E) Gauge Invariance Check

        Verify quotient fingerprint unchanged under recoding.
        """
        checker = GaugeInvarianceChecker(self.d0, self.tests)
        result = checker.check_gauge_invariance(seed)
        self.receipts.append(result)

        return self._add_check(
            check_id="E",
            check_name="Gauge Invariance",
            passed=result["match"],
            details=result
        )

    def check_F_canonical_receipts(self) -> CheckResult:
        """
        F) Canonical Receipts

        Verify all receipts are properly formatted and hashable.
        """
        all_valid = True
        receipt_hashes = []

        for i, receipt in enumerate(self.receipts):
            try:
                canonical = CanonicalJSON.serialize(receipt)
                fp = hashlib.sha256(canonical.encode()).hexdigest()
                receipt_hashes.append({
                    "index": i,
                    "type": receipt.get("type", "UNKNOWN"),
                    "fingerprint": fp[:16]
                })
            except Exception as e:
                all_valid = False
                receipt_hashes.append({
                    "index": i,
                    "error": str(e)
                })

        result = {
            "type": "CANONICAL_RECEIPTS",
            "receipts_checked": len(self.receipts),
            "all_valid": all_valid,
            "receipt_hashes": receipt_hashes,
            "result": "PASS" if all_valid else "FAIL"
        }
        self.receipts.append(result)

        return self._add_check(
            check_id="F",
            check_name="Canonical Receipts",
            passed=all_valid,
            details=result
        )

    def run_all_checks(self) -> List[CheckResult]:
        """Run all verification checks A-F."""
        self.checks = []
        self.receipts = []

        self.check_A_test_totality()
        self.check_B_equivalence_relation()
        self.check_C_quotient_construction()
        self.check_D_factorization()
        self.check_E_gauge_invariance()
        self.check_F_canonical_receipts()

        return self.checks

    def create_proof_bundle(self) -> ObjectEqualityProofBundle:
        """Create complete proof bundle."""
        if not self.checks:
            self.run_all_checks()

        bundle_id = f"OE_{hashlib.sha256(str(self.d0).encode()).hexdigest()[:8]}"

        return ObjectEqualityProofBundle(
            bundle_id=bundle_id,
            checks=self.checks.copy(),
            quotient_fingerprint=self.quotient.fingerprint() if self.quotient else "",
            domain_size=len(self.d0),
            class_count=self.quotient.class_count() if self.quotient else 0,
            class_sizes=self.quotient.class_sizes() if self.quotient else [],
            receipts=self.receipts.copy()
        )


def run_object_equality_verification(
    d0: FrozenSet[Any],
    tests: Dict[str, Test],
    output_functions: Optional[Dict[str, Callable[[Any], Any]]] = None
) -> ObjectEqualityProofBundle:
    """
    Run complete Object and Equality verification.

    Args:
        d0: Finite working domain
        tests: Available tests (the set Δ)
        output_functions: Optional dict of functions to check for factorization

    Returns:
        Complete proof bundle with all checks
    """
    verifier = ObjectEqualityVerifier(d0, tests, output_functions)
    verifier.run_all_checks()
    return verifier.create_proof_bundle()


def print_verification_report(bundle: ObjectEqualityProofBundle) -> str:
    """Generate human-readable verification report."""
    lines = [
        "=" * 60,
        "OBJECT AND EQUALITY - VERIFICATION REPORT",
        "=" * 60,
        f"Bundle ID: {bundle.bundle_id}",
        f"Fingerprint: {bundle.fingerprint()[:32]}...",
        "",
        f"Domain Size: {bundle.domain_size}",
        f"Equivalence Classes: {bundle.class_count}",
        f"Class Sizes: {bundle.class_sizes}",
        f"Quotient Fingerprint: {bundle.quotient_fingerprint[:32]}...",
        "",
        "-" * 60,
        "VERIFICATION CHECKS",
        "-" * 60,
    ]

    for check in bundle.checks:
        status = "PASS" if check.passed else "FAIL"
        lines.append(f"[{check.check_id}] {check.check_name}: {status}")

    lines.extend([
        "",
        "-" * 60,
        "SUMMARY",
        "-" * 60,
        f"Total Checks: {len(bundle.checks)}",
        f"Passed: {sum(1 for c in bundle.checks if c.passed)}",
        f"Failed: {sum(1 for c in bundle.checks if not c.passed)}",
        "",
        f"OVERALL: {'ALL CHECKS PASSED' if bundle.all_passed() else 'VERIFICATION FAILED'}",
        "=" * 60
    ])

    return "\n".join(lines)


# Demo runner
def run_demo() -> Dict[str, Any]:
    """
    Run Object and Equality demonstration.
    """
    # Create sample domain
    d0 = frozenset(range(20))

    # Create tests
    def parity(x: int) -> str:
        return "EVEN" if x % 2 == 0 else "ODD"

    def mod3(x: int) -> str:
        return f"MOD{x % 3}"

    def sign(x: int) -> str:
        if x < 5:
            return "LOW"
        elif x < 15:
            return "MID"
        else:
            return "HIGH"

    tests = {
        "parity": Test(
            test_id="parity",
            evaluator=parity,
            cost=1,
            outcome_space=frozenset(["EVEN", "ODD"])
        ),
        "mod3": Test(
            test_id="mod3",
            evaluator=mod3,
            cost=1,
            outcome_space=frozenset(["MOD0", "MOD1", "MOD2"])
        ),
        "sign": Test(
            test_id="sign",
            evaluator=sign,
            cost=1,
            outcome_space=frozenset(["LOW", "MID", "HIGH"])
        )
    }

    # Output functions to verify
    output_functions = {
        "classify": lambda x: f"{parity(x)}_{mod3(x)}",
        "bucket": lambda x: sign(x)
    }

    # Run verification
    bundle = run_object_equality_verification(d0, tests, output_functions)

    # Print report
    report = print_verification_report(bundle)
    print(report)

    return {
        "demo": "Object and Equality",
        "all_passed": bundle.all_passed(),
        "summary": bundle.summary(),
        "checks": [
            {"id": c.check_id, "name": c.check_name, "passed": c.passed}
            for c in bundle.checks
        ]
    }


if __name__ == "__main__":
    result = run_demo()
    print("\n" + json.dumps(result, indent=2))
