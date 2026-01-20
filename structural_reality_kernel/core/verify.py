"""
core/verify.py - Verification suite and proof bundle generation.

Implements all verification checks A-J:
A. Ledger hash-chain integrity
B. Survivor consistency: W(L) recomputed matches recorded
C. Π* class-sizes gauge-invariant
D. Time ratios: (|W_pre|, |W_post|) all integers
E. Budget monotonicity: Budget(L) ≥ Budget(L ∪ {r})
F. Feasibility: c(τ) ≤ BudgetUnits before every test
G. NSL closure idempotent
H. No float in any receipt
I. Commutation witness for every control decision
J. Final Π* fingerprint matches recomputation
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import json
import re

from .kernel import (
    Test, Record, Ledger, Survivors, PiStar, Budget,
    FeasibleTests, KernelState, compute_kernel_state
)
from .nsl import NSLState, NSLEngine, NSLClosure, Distinction, Trit
from .controller import PiController, CommutationChecker, CommutationWitness
from .gauge import Canonicalizer, GaugeChecker, create_random_recoding
from .receipts import CanonicalJSON, ReceiptChain, Receipt


@dataclass
class CheckResult:
    """Result of a single verification check."""
    check_id: str
    check_name: str
    passed: bool
    details: Dict[str, Any]
    error_message: Optional[str] = None

    def canonical(self) -> str:
        """Canonical representation."""
        return CanonicalJSON.serialize({
            "id": self.check_id,
            "name": self.check_name,
            "passed": self.passed,
            "details": self.details,
            "error": self.error_message
        })


@dataclass
class ProofBundle:
    """
    Complete proof bundle for a kernel run.

    Contains:
    - All verification results
    - Receipt chain
    - State fingerprints
    - Gauge witnesses
    """
    bundle_id: str
    checks: List[CheckResult]
    receipt_chain_fp: str
    initial_state_fp: str
    final_state_fp: str
    gauge_witnesses: Dict[str, Any]
    metadata: Dict[str, Any]

    def all_passed(self) -> bool:
        """Check if all verification checks passed."""
        return all(c.passed for c in self.checks)

    def canonical(self) -> str:
        """Canonical representation."""
        return CanonicalJSON.serialize({
            "bundle_id": self.bundle_id,
            "all_passed": self.all_passed(),
            "check_count": len(self.checks),
            "passed_count": sum(1 for c in self.checks if c.passed),
            "receipt_chain_fp": self.receipt_chain_fp,
            "initial_fp": self.initial_state_fp,
            "final_fp": self.final_state_fp
        })

    def fingerprint(self) -> str:
        """SHA-256 fingerprint."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def summary(self) -> Dict[str, Any]:
        """Get summary of verification results."""
        return {
            "bundle_id": self.bundle_id,
            "all_passed": self.all_passed(),
            "total_checks": len(self.checks),
            "passed": sum(1 for c in self.checks if c.passed),
            "failed": sum(1 for c in self.checks if not c.passed),
            "failed_checks": [c.check_id for c in self.checks if not c.passed],
            "fingerprint": self.fingerprint()
        }


class VerificationSuite:
    """
    Complete verification suite for kernel runs.

    Implements checks A-J.
    """

    def __init__(
        self,
        d0: FrozenSet[Any],
        tests: Dict[str, Test],
        alpha: int = 1
    ):
        self.d0 = d0
        self.tests = tests
        self.alpha = alpha
        self.checks: List[CheckResult] = []

    def _add_check(
        self,
        check_id: str,
        check_name: str,
        passed: bool,
        details: Dict[str, Any],
        error: Optional[str] = None
    ) -> CheckResult:
        """Add a check result."""
        result = CheckResult(
            check_id=check_id,
            check_name=check_name,
            passed=passed,
            details=details,
            error_message=error
        )
        self.checks.append(result)
        return result

    def check_A_ledger_integrity(
        self,
        receipt_chain: ReceiptChain
    ) -> CheckResult:
        """
        A. Ledger hash-chain integrity.

        Verify each receipt links to previous by fingerprint.
        """
        valid, broken_at = receipt_chain.verify_chain_integrity()

        return self._add_check(
            check_id="A",
            check_name="Ledger hash-chain integrity",
            passed=valid,
            details={
                "receipt_count": len(receipt_chain),
                "broken_at": broken_at
            },
            error=f"Chain broken at receipt {broken_at}" if not valid else None
        )

    def check_B_survivor_consistency(
        self,
        ledger: Ledger,
        recorded_survivors: int
    ) -> CheckResult:
        """
        B. Survivor consistency.

        Recompute W(L) and verify it matches recorded value.
        """
        survivors = Survivors(self.d0, ledger, self.tests)
        recomputed = len(survivors)
        passed = recomputed == recorded_survivors

        return self._add_check(
            check_id="B",
            check_name="Survivor consistency",
            passed=passed,
            details={
                "recorded": recorded_survivors,
                "recomputed": recomputed
            },
            error=f"Mismatch: recorded {recorded_survivors}, recomputed {recomputed}" if not passed else None
        )

    def check_C_gauge_invariance(
        self,
        ledger: Ledger,
        seed: int = 42
    ) -> CheckResult:
        """
        C. Π* class-sizes gauge-invariant.

        Verify class-size multiset is unchanged under recoding.
        """
        checker = GaugeChecker()
        recoding = create_random_recoding(self.d0, seed=seed)

        result = checker.check_recoding_invariance(
            self.d0, ledger, self.tests, recoding, self.alpha
        )

        return self._add_check(
            check_id="C",
            check_name="Gauge invariance (Π* class-sizes)",
            passed=result["passed"],
            details=result,
            error="Class-size multiset changed under recoding" if not result["passed"] else None
        )

    def check_D_time_ratios(
        self,
        receipt_chain: ReceiptChain
    ) -> CheckResult:
        """
        D. Time ratios are integers.

        Verify all (|W_pre|, |W_post|) values are integers.
        """
        all_integer = True
        invalid_receipts = []

        for receipt in receipt_chain:
            payload = receipt.payload
            if "w_pre" in payload:
                if not isinstance(payload["w_pre"], int):
                    all_integer = False
                    invalid_receipts.append(receipt.receipt_id)
            if "w_post" in payload:
                if not isinstance(payload["w_post"], int):
                    all_integer = False
                    invalid_receipts.append(receipt.receipt_id)

        return self._add_check(
            check_id="D",
            check_name="Time ratios are integers",
            passed=all_integer,
            details={
                "receipts_checked": len(receipt_chain),
                "invalid_receipts": invalid_receipts
            },
            error=f"Non-integer ratios in receipts: {invalid_receipts}" if not all_integer else None
        )

    def check_E_budget_monotonicity(
        self,
        receipt_chain: ReceiptChain
    ) -> CheckResult:
        """
        E. Budget monotonicity.

        Verify Budget(L) ≥ Budget(L ∪ {r}) for each record.
        """
        monotone = True
        violations = []
        prev_budget = None

        for receipt in receipt_chain:
            if receipt.operation == "APPLY_TEST":
                w_post = receipt.payload.get("w_post", 0)
                current_budget = Budget(w_post, self.alpha).budget_units()

                if prev_budget is not None and current_budget > prev_budget:
                    monotone = False
                    violations.append({
                        "receipt": receipt.receipt_id,
                        "prev_budget": prev_budget,
                        "curr_budget": current_budget
                    })

                prev_budget = current_budget

        return self._add_check(
            check_id="E",
            check_name="Budget monotonicity",
            passed=monotone,
            details={
                "violations": violations
            },
            error=f"Budget increased: {violations}" if not monotone else None
        )

    def check_F_feasibility(
        self,
        receipt_chain: ReceiptChain
    ) -> CheckResult:
        """
        F. Feasibility constraint.

        Verify c(τ) ≤ BudgetUnits before every test.
        """
        feasible = True
        violations = []

        ledger = Ledger()
        for receipt in receipt_chain:
            if receipt.operation == "APPLY_TEST":
                test_id = receipt.payload.get("test_id")
                cost = receipt.payload.get("cost", 0)

                # Get budget before this test
                survivors = Survivors(self.d0, ledger, self.tests)
                budget = Budget(len(survivors), self.alpha)
                budget_units = budget.budget_units()

                if cost > budget_units:
                    feasible = False
                    violations.append({
                        "receipt": receipt.receipt_id,
                        "test_id": test_id,
                        "cost": cost,
                        "budget_units": budget_units
                    })

                # Update ledger for next iteration
                outcome = receipt.payload.get("outcome")
                if test_id and outcome:
                    # Parse outcome string back to value
                    record = Record(test_id=test_id, outcome=outcome)
                    ledger = ledger.append(record)

        return self._add_check(
            check_id="F",
            check_name="Feasibility constraint",
            passed=feasible,
            details={
                "violations": violations
            },
            error=f"Infeasible tests: {violations}" if not feasible else None
        )

    def check_G_nsl_closure_idempotent(
        self,
        nsl_state: NSLState,
        closure: NSLClosure
    ) -> CheckResult:
        """
        G. NSL closure is idempotent.

        Verify Cl(Cl(s)) = Cl(s).
        """
        closed_once = closure.apply(nsl_state)
        closed_twice = closure.apply(closed_once)

        idempotent = closed_once.fingerprint() == closed_twice.fingerprint()

        return self._add_check(
            check_id="G",
            check_name="NSL closure idempotent",
            passed=idempotent,
            details={
                "fp_once": closed_once.fingerprint()[:16],
                "fp_twice": closed_twice.fingerprint()[:16]
            },
            error="Closure not idempotent" if not idempotent else None
        )

    def check_H_no_floats(
        self,
        receipt_chain: ReceiptChain
    ) -> CheckResult:
        """
        H. No floats in any receipt.

        Verify all receipt payloads contain only integers, strings, etc.
        """
        has_float = False
        float_locations = []

        def find_floats(obj: Any, path: str) -> None:
            nonlocal has_float, float_locations
            if isinstance(obj, float):
                has_float = True
                float_locations.append(path)
            elif isinstance(obj, dict):
                for k, v in obj.items():
                    find_floats(v, f"{path}.{k}")
            elif isinstance(obj, (list, tuple)):
                for i, v in enumerate(obj):
                    find_floats(v, f"{path}[{i}]")

        for receipt in receipt_chain:
            find_floats(receipt.payload, f"{receipt.receipt_id}.payload")

        return self._add_check(
            check_id="H",
            check_name="No floats in receipts",
            passed=not has_float,
            details={
                "float_locations": float_locations
            },
            error=f"Floats found at: {float_locations}" if has_float else None
        )

    def check_I_commutation(
        self,
        controller: PiController,
        kernel_state: KernelState
    ) -> CheckResult:
        """
        I. Commutation witness for control decisions.

        Verify N ∘ Q = Q ∘ N for the controller.
        """
        checker = CommutationChecker(controller)
        witness = checker.check_commutation(
            kernel_state, self.d0, self.tests, self.alpha
        )

        return self._add_check(
            check_id="I",
            check_name="Commutation law (N∘Q = Q∘N)",
            passed=witness.passed,
            details=witness.details,
            error="Commutation law violated" if not witness.passed else None
        )

    def check_J_final_pi_star(
        self,
        ledger: Ledger,
        recorded_fp: str
    ) -> CheckResult:
        """
        J. Final Π* fingerprint matches recomputation.

        Recompute Π* and verify fingerprint matches.
        """
        pi_star = PiStar(self.d0, ledger, self.tests)
        recomputed_fp = pi_star.canonical_fingerprint()

        passed = recomputed_fp == recorded_fp

        return self._add_check(
            check_id="J",
            check_name="Final Π* fingerprint",
            passed=passed,
            details={
                "recorded": recorded_fp[:32],
                "recomputed": recomputed_fp[:32]
            },
            error=f"Fingerprint mismatch" if not passed else None
        )

    def run_all_checks(
        self,
        ledger: Ledger,
        receipt_chain: ReceiptChain,
        nsl_state: NSLState,
        nsl_closure: NSLClosure,
        controller: PiController,
        final_state: KernelState,
        recorded_survivors: int,
        recorded_pi_fp: str
    ) -> List[CheckResult]:
        """
        Run all verification checks A-J.
        """
        self.checks = []  # Reset

        self.check_A_ledger_integrity(receipt_chain)
        self.check_B_survivor_consistency(ledger, recorded_survivors)
        self.check_C_gauge_invariance(ledger)
        self.check_D_time_ratios(receipt_chain)
        self.check_E_budget_monotonicity(receipt_chain)
        self.check_F_feasibility(receipt_chain)
        self.check_G_nsl_closure_idempotent(nsl_state, nsl_closure)
        self.check_H_no_floats(receipt_chain)
        self.check_I_commutation(controller, final_state)
        self.check_J_final_pi_star(ledger, recorded_pi_fp)

        return self.checks

    def create_proof_bundle(
        self,
        bundle_id: str,
        receipt_chain: ReceiptChain,
        initial_state_fp: str,
        final_state_fp: str,
        gauge_witnesses: Dict[str, Any],
        metadata: Optional[Dict[str, Any]] = None
    ) -> ProofBundle:
        """
        Create a proof bundle from verification results.
        """
        return ProofBundle(
            bundle_id=bundle_id,
            checks=self.checks.copy(),
            receipt_chain_fp=receipt_chain.chain_fingerprint(),
            initial_state_fp=initial_state_fp,
            final_state_fp=final_state_fp,
            gauge_witnesses=gauge_witnesses,
            metadata=metadata or {}
        )


def verify_kernel_run(
    d0: FrozenSet[Any],
    tests: Dict[str, Test],
    ledger: Ledger,
    receipt_chain: ReceiptChain,
    controller: PiController,
    nsl_engine: NSLEngine,
    final_state: KernelState,
    alpha: int = 1
) -> ProofBundle:
    """
    Complete verification of a kernel run.

    Returns a proof bundle with all checks.
    """
    suite = VerificationSuite(d0, tests, alpha)

    # Get recorded values
    recorded_survivors = len(final_state.survivors)
    recorded_pi_fp = final_state.pi_star.canonical_fingerprint()

    # Run all checks
    suite.run_all_checks(
        ledger=ledger,
        receipt_chain=receipt_chain,
        nsl_state=nsl_engine.state,
        nsl_closure=nsl_engine.closure,
        controller=controller,
        final_state=final_state,
        recorded_survivors=recorded_survivors,
        recorded_pi_fp=recorded_pi_fp
    )

    # Get gauge witnesses
    gauge_checker = GaugeChecker()
    recoding = create_random_recoding(d0)
    recoding_result = gauge_checker.check_recoding_invariance(
        d0, ledger, tests, recoding, alpha
    )

    # Build initial state for fingerprint
    initial_state = compute_kernel_state(d0, Ledger(), tests, alpha)

    # Create bundle
    bundle = suite.create_proof_bundle(
        bundle_id=f"PB_{hashlib.sha256(str(d0).encode()).hexdigest()[:8]}",
        receipt_chain=receipt_chain,
        initial_state_fp=initial_state.canonical_fingerprint(),
        final_state_fp=final_state.canonical_fingerprint(),
        gauge_witnesses={
            "recoding": recoding_result,
            "order": gauge_checker.summary()
        },
        metadata={
            "d0_size": len(d0),
            "test_count": len(tests),
            "alpha": alpha,
            "ledger_size": len(ledger)
        }
    )

    return bundle


def print_verification_report(bundle: ProofBundle) -> str:
    """
    Generate human-readable verification report.
    """
    lines = [
        "=" * 60,
        "STRUCTURAL REALITY KERNEL - VERIFICATION REPORT",
        "=" * 60,
        f"Bundle ID: {bundle.bundle_id}",
        f"Fingerprint: {bundle.fingerprint()[:32]}...",
        "",
        f"Initial State: {bundle.initial_state_fp[:32]}...",
        f"Final State: {bundle.final_state_fp[:32]}...",
        "",
        "-" * 60,
        "VERIFICATION CHECKS",
        "-" * 60,
    ]

    for check in bundle.checks:
        status = "✓ PASS" if check.passed else "✗ FAIL"
        lines.append(f"[{check.check_id}] {check.check_name}: {status}")
        if not check.passed and check.error_message:
            lines.append(f"    Error: {check.error_message}")

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
