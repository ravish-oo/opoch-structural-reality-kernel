"""
uncertainty_verify.py - Complete verification suite for Uncertainty, Probability, and Quantum.

Implements all verification checks A-D:
A) Frontier Correctness - UNIQUE vs OMEGA output
B) Probability Correctness - nonnegativity, normalization, additivity
C) Refinement Update Correctness - normalized restriction (Bayes)
D) Quantum Bookkeeping Correctness - positivity, normalization, GNS identity
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import json
import math

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import Test
from core.receipts import CanonicalJSON

from .probability import (
    Query, AnswerSet, Fiber, ProbabilityDistribution,
    RefinementUpdate, FrontierComputer, compute_frontier_status
)
from .quantum import (
    AlgebraElement, StarAlgebra, QuantumState, GNSConstruction,
    create_qubit_algebra, create_pure_state, create_gns_representation
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
class UncertaintyProofBundle:
    """
    Complete proof bundle for Uncertainty, Probability, and Quantum verification.
    """
    bundle_id: str
    checks: List[CheckResult]
    frontier_count: int
    omega_count: int
    unique_count: int
    quantum_verified: bool
    receipts: List[Dict[str, Any]]

    def all_passed(self) -> bool:
        return all(c.passed for c in self.checks)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "bundle_id": self.bundle_id,
            "all_passed": self.all_passed(),
            "check_count": len(self.checks),
            "passed_count": sum(1 for c in self.checks if c.passed),
            "frontier_count": self.frontier_count,
            "omega_count": self.omega_count,
            "unique_count": self.unique_count
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
            "frontier_count": self.frontier_count,
            "omega_count": self.omega_count,
            "unique_count": self.unique_count,
            "quantum_verified": self.quantum_verified
        }


class UncertaintyVerifier:
    """
    Complete verification suite for Uncertainty, Probability, and Quantum.

    Implements checks A-D.
    """

    def __init__(
        self,
        d0: FrozenSet[Any],
        tests: Dict[str, Test],
        queries: List[Query]
    ):
        self.d0 = d0
        self.tests = tests
        self.queries = queries
        self.computer = FrontierComputer(d0, tests)

        self.checks: List[CheckResult] = []
        self.receipts: List[Dict[str, Any]] = []

        # Tracking
        self.frontier_results: List[Tuple[AnswerSet, Optional[ProbabilityDistribution]]] = []
        self.ledger: List[Tuple[str, Any]] = []

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

    def record_event(self, test_id: str, outcome: Any):
        """Record a test event to the ledger."""
        self.ledger.append((test_id, outcome))

    def check_A_frontier_correctness(self) -> CheckResult:
        """
        A) Frontier Correctness

        For each query, compute Ans_L(q) and verify:
        - If size = 1: output UNIQUE
        - If size > 1: output OMEGA with surviving family
        """
        all_correct = True
        frontier_receipts = []

        survivors = self.computer.compute_survivors(self.ledger)

        for query in self.queries:
            answer_set = self.computer.compute_answer_set(survivors, query)

            # Verify correct status
            if answer_set.size == 0:
                status_correct = (answer_set.status == "CONTRADICTION")
            elif answer_set.size == 1:
                status_correct = (answer_set.status == "UNIQUE")
            else:
                status_correct = (answer_set.status == "OMEGA")

            if not status_correct:
                all_correct = False

            # Compute probability if OMEGA
            prob_dist = None
            if answer_set.is_omega:
                prob_dist = self.computer.compute_probability_distribution(survivors, query)

            self.frontier_results.append((answer_set, prob_dist))

            receipt = answer_set.to_receipt()
            receipt["status_correct"] = status_correct
            frontier_receipts.append(receipt)
            self.receipts.append(receipt)

        bundle_receipt = {
            "type": "FRONTIER_CORRECTNESS_BUNDLE",
            "queries_checked": len(self.queries),
            "all_correct": all_correct,
            "checks": frontier_receipts,
            "result": "PASS" if all_correct else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="A",
            check_name="Frontier Correctness",
            passed=all_correct,
            details=bundle_receipt
        )

    def check_B_probability_correctness(self) -> CheckResult:
        """
        B) Probability Correctness (Finite Case)

        For each OMEGA frontier, verify:
        - nonnegativity: |W_b| >= 0
        - normalization: Sum |W_b| = |W|
        - additivity: P(a or b) = P(a) + P(b) for disjoint
        """
        all_correct = True
        probability_receipts = []

        for answer_set, prob_dist in self.frontier_results:
            if prob_dist is None:
                continue  # UNIQUE, no probability

            # Verify properties
            nonneg_ok = prob_dist.verify_nonnegativity()
            norm_ok = prob_dist.verify_normalization()

            # Test additivity with first two answers if available
            additivity_ok = True
            additivity_receipt = None
            if len(prob_dist.fibers) >= 2:
                answers_to_merge = [prob_dist.fibers[0].answer, prob_dist.fibers[1].answer]
                additivity_ok, additivity_receipt = prob_dist.verify_additivity(answers_to_merge)

            all_ok = nonneg_ok and norm_ok and additivity_ok
            if not all_ok:
                all_correct = False

            receipt = prob_dist.to_receipt()
            receipt["additivity_verified"] = additivity_ok
            if additivity_receipt:
                receipt["additivity_check"] = additivity_receipt
            probability_receipts.append(receipt)
            self.receipts.append(receipt)

        bundle_receipt = {
            "type": "PROBABILITY_CORRECTNESS_BUNDLE",
            "distributions_checked": len(probability_receipts),
            "all_correct": all_correct,
            "checks": probability_receipts,
            "result": "PASS" if all_correct else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="B",
            check_name="Probability Correctness",
            passed=all_correct,
            details=bundle_receipt
        )

    def check_C_refinement_correctness(
        self,
        test_id: str,
        outcome: Any
    ) -> CheckResult:
        """
        C) Refinement Update Correctness

        After new evidence, verify update is normalized restriction.
        """
        all_correct = True
        refinement_receipts = []

        # Get pre-refinement survivors
        survivors_pre = self.computer.compute_survivors(self.ledger)

        # Record new event
        self.record_event(test_id, outcome)

        # Get post-refinement survivors
        survivors_post = self.computer.compute_survivors(self.ledger)

        # Verify refinement for each query
        for query in self.queries:
            refinement = self.computer.compute_refinement(
                survivors_pre, survivors_post, query
            )

            restriction_ok = refinement.verify_restriction()
            if not restriction_ok:
                all_correct = False

            receipt = refinement.to_receipt()
            refinement_receipts.append(receipt)
            self.receipts.append(receipt)

        bundle_receipt = {
            "type": "REFINEMENT_CORRECTNESS_BUNDLE",
            "test_id": test_id,
            "outcome": str(outcome),
            "refinements_checked": len(refinement_receipts),
            "all_correct": all_correct,
            "checks": refinement_receipts,
            "result": "PASS" if all_correct else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="C",
            check_name="Refinement Update Correctness",
            passed=all_correct,
            details=bundle_receipt
        )

    def check_D_quantum_correctness(self) -> CheckResult:
        """
        D) Quantum Bookkeeping Correctness

        For a noncommutative algebra and state, verify:
        - positivity: omega(a*a) >= 0
        - normalization: omega(1) = 1
        - GNS identity: omega(a) = <Omega, pi(a) Omega>
        """
        # Create qubit algebra (standard noncommutative example)
        algebra = create_qubit_algebra()

        # Create a pure state (|0> state)
        state_vector = [complex(1, 0), complex(0, 0)]
        state = create_pure_state(algebra, state_vector)

        # Create GNS representation
        gns = create_gns_representation(algebra, state)

        # Verify properties
        test_elements = algebra.basis

        # Positivity
        pos_ok, pos_receipts = state.verify_positivity(test_elements)

        # Normalization
        norm_ok, norm_receipt = state.verify_normalization()

        # GNS identity
        gns_ok, gns_receipts = gns.verify_gns_identity(test_elements)
        cyclic_ok, cyclic_receipt = gns.verify_cyclic_norm()

        all_ok = pos_ok and norm_ok and gns_ok and cyclic_ok

        # Algebra receipt
        algebra_receipt = algebra.to_receipt()
        self.receipts.append(algebra_receipt)

        # State receipt
        state_receipt = state.to_receipt(test_elements)
        self.receipts.append(state_receipt)

        # GNS receipt
        gns_receipt = gns.to_receipt(test_elements)
        self.receipts.append(gns_receipt)

        bundle_receipt = {
            "type": "QUANTUM_CORRECTNESS_BUNDLE",
            "algebra_dimension": algebra.dimension,
            "is_noncommutative": algebra.is_noncommutative,
            "positivity_verified": pos_ok,
            "normalization_verified": norm_ok,
            "gns_identity_verified": gns_ok,
            "cyclic_norm_verified": cyclic_ok,
            "all_correct": all_ok,
            "algebra_receipt": algebra_receipt,
            "state_receipt": state_receipt,
            "gns_receipt": gns_receipt,
            "result": "PASS" if all_ok else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="D",
            check_name="Quantum Bookkeeping Correctness",
            passed=all_ok,
            details=bundle_receipt
        )

    def run_all_checks(
        self,
        refinement_test: Optional[str] = None,
        refinement_outcome: Optional[Any] = None
    ) -> List[CheckResult]:
        """Run all verification checks A-D."""
        self.checks = []

        # A) Frontier correctness
        self.check_A_frontier_correctness()

        # B) Probability correctness
        self.check_B_probability_correctness()

        # C) Refinement correctness (if test provided)
        if refinement_test and refinement_outcome is not None:
            self.check_C_refinement_correctness(refinement_test, refinement_outcome)
        else:
            # Use first available test
            if self.tests:
                test_id = list(self.tests.keys())[0]
                test = self.tests[test_id]
                # Pick an outcome
                if self.d0:
                    sample = next(iter(self.d0))
                    outcome = test.evaluator(sample)
                    self.check_C_refinement_correctness(test_id, outcome)

        # D) Quantum correctness
        self.check_D_quantum_correctness()

        return self.checks

    def create_proof_bundle(self) -> UncertaintyProofBundle:
        """Create complete proof bundle."""
        if not self.checks:
            raise ValueError("Must run checks before creating bundle")

        bundle_id = f"UPQ_{hashlib.sha256(str(self.d0).encode()).hexdigest()[:8]}"

        # Count frontiers
        omega_count = sum(1 for ans, _ in self.frontier_results if ans.is_omega)
        unique_count = sum(1 for ans, _ in self.frontier_results if ans.is_decided)

        # Check if quantum passed
        quantum_check = next((c for c in self.checks if c.check_id == "D"), None)
        quantum_verified = quantum_check.passed if quantum_check else False

        return UncertaintyProofBundle(
            bundle_id=bundle_id,
            checks=self.checks.copy(),
            frontier_count=len(self.frontier_results),
            omega_count=omega_count,
            unique_count=unique_count,
            quantum_verified=quantum_verified,
            receipts=self.receipts.copy()
        )


def run_uncertainty_verification(
    d0: FrozenSet[Any],
    tests: Dict[str, Test],
    queries: List[Query],
    initial_ledger: List[Tuple[str, Any]] = None
) -> UncertaintyProofBundle:
    """
    Run complete Uncertainty, Probability, and Quantum verification.

    Args:
        d0: Finite working domain
        tests: Available tests
        queries: Queries to evaluate
        initial_ledger: Initial ledger entries

    Returns:
        Complete proof bundle with all checks
    """
    verifier = UncertaintyVerifier(d0, tests, queries)

    # Set initial ledger if provided
    if initial_ledger:
        for test_id, outcome in initial_ledger:
            verifier.record_event(test_id, outcome)

    verifier.run_all_checks()
    return verifier.create_proof_bundle()


def print_uncertainty_report(bundle: UncertaintyProofBundle) -> str:
    """Generate human-readable verification report."""
    lines = [
        "=" * 60,
        "UNCERTAINTY, PROBABILITY, QUANTUM - VERIFICATION REPORT",
        "=" * 60,
        f"Bundle ID: {bundle.bundle_id}",
        f"Fingerprint: {bundle.fingerprint()[:32]}...",
        "",
        f"Frontiers Evaluated: {bundle.frontier_count}",
        f"  OMEGA (undecided): {bundle.omega_count}",
        f"  UNIQUE (decided): {bundle.unique_count}",
        f"Quantum Verified: {bundle.quantum_verified}",
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


def run_demo() -> Dict[str, Any]:
    """
    Run Uncertainty, Probability, and Quantum demonstration.
    """
    # Create sample domain
    d0 = frozenset(range(30))

    # Create tests
    def parity(x: int) -> str:
        return "EVEN" if x % 2 == 0 else "ODD"

    def mod3(x: int) -> str:
        return f"MOD{x % 3}"

    def threshold(x: int) -> str:
        if x < 10:
            return "LOW"
        elif x < 20:
            return "MEDIUM"
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
            cost=2,
            outcome_space=frozenset([f"MOD{i}" for i in range(3)])
        ),
        "threshold": Test(
            test_id="threshold",
            evaluator=threshold,
            cost=1,
            outcome_space=frozenset(["LOW", "MEDIUM", "HIGH"])
        )
    }

    # Create queries
    queries = [
        Query(
            query_id="q_parity",
            evaluator=parity,
            answer_space=frozenset(["EVEN", "ODD"])
        ),
        Query(
            query_id="q_threshold",
            evaluator=threshold,
            answer_space=frozenset(["LOW", "MEDIUM", "HIGH"])
        )
    ]

    # Run with some initial evidence
    initial_ledger = [
        ("parity", "EVEN"),  # Know it's even
    ]

    # Run verification
    bundle = run_uncertainty_verification(d0, tests, queries, initial_ledger)

    # Print report
    report = print_uncertainty_report(bundle)
    print(report)

    return {
        "demo": "Uncertainty, Probability, and Quantum",
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
