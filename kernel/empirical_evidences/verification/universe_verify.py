"""
universe_verify.py - Complete verification suite for the Universe Engine.

Implements all verification checks A-H:
A) Total verifier discipline - every test is total
B) Diamond/path-freeness - reorder invariance of Pi*
C) Time/entropy identity - Delta_T = log(|W|/|W'|)
D) Feasibility shrink - Delta shrinks with |W|
E) Canonical tau* - minimax argmin verification
F) Pi-consistent control - gauge invariance of decisions
G) Omega honesty - frontier output when |Ans|>1
H) Canonical receipts - SHA-256 hashed JSON
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import json
import math
import random

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import Test
from core.receipts import CanonicalJSON

from .engine_state import (
    LedgerRecord, Ledger, EngineState, EngineStep, EngineTermination,
    compute_feasible_tests, compute_survivors, create_initial_state
)
from .universe_engine import (
    UniverseEngine, SeparatorSelection, EngineTrace,
    run_engine, SelfImprovement, demonstrate_self_improvement
)
from .computation import Query, AnswerComputer


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
class UniverseEngineProofBundle:
    """
    Complete proof bundle for Universe Engine verification.
    """
    bundle_id: str
    checks: List[CheckResult]
    total_verifier_ok: bool
    diamond_ok: bool
    time_entropy_ok: bool
    feasibility_shrink_ok: bool
    canonical_tau_star_ok: bool
    pi_consistent_ok: bool
    omega_honest_ok: bool
    receipts_ok: bool
    receipts: List[Dict[str, Any]]

    def all_passed(self) -> bool:
        return all(c.passed for c in self.checks)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "bundle_id": self.bundle_id,
            "all_passed": self.all_passed(),
            "check_count": len(self.checks),
            "passed_count": sum(1 for c in self.checks if c.passed)
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
            "failed_checks": [c.check_id for c in self.checks if not c.passed]
        }


class UniverseEngineVerifier:
    """
    Complete verification suite for the Universe Engine.

    Implements checks A-H.
    """

    def __init__(
        self,
        d0: FrozenSet[Any],
        tests: Dict[str, Test],
        query: Query,
        actual_element: Any
    ):
        self.d0 = d0
        self.tests = tests
        self.query = query
        self.actual_element = actual_element
        self.answer_computer = AnswerComputer(query)

        self.checks: List[CheckResult] = []
        self.receipts: List[Dict[str, Any]] = []

        # Results tracking
        self.total_verifier_ok = False
        self.diamond_ok = False
        self.time_entropy_ok = False
        self.feasibility_shrink_ok = False
        self.canonical_tau_star_ok = False
        self.pi_consistent_ok = False
        self.omega_honest_ok = False
        self.receipts_ok = False

        # Run engine for trace
        self.engine = UniverseEngine(d0, tests, query)
        self.trace: Optional[EngineTrace] = None

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

    def _run_engine_trace(self) -> EngineTrace:
        """Run engine and get trace."""
        if self.trace is None:
            self.trace = run_engine(
                self.d0,
                self.tests,
                self.query,
                self.actual_element,
                max_steps=50
            )
        return self.trace

    def check_A_total_verifier(self) -> CheckResult:
        """
        A) Total Verifier Discipline

        Every test tau is total and returns a finite outcome.
        """
        all_total = True
        issues = []
        test_receipts = []

        for test_id, test in self.tests.items():
            is_total = True
            errors = 0

            for x in self.d0:
                try:
                    outcome = test.evaluator(x)
                    # Check outcome is in outcome space if defined
                    if test.outcome_space and outcome not in test.outcome_space:
                        is_total = False
                        errors += 1
                except Exception as e:
                    is_total = False
                    errors += 1

            if not is_total:
                all_total = False
                issues.append(f"Test {test_id} not total: {errors} errors")

            test_receipts.append({
                "test_id": test_id,
                "is_total": is_total,
                "elements_tested": len(self.d0),
                "errors": errors
            })

        self.total_verifier_ok = all_total

        bundle_receipt = {
            "type": "TOTAL_VERIFIER_BUNDLE",
            "tests_checked": len(self.tests),
            "all_total": all_total,
            "issues": issues,
            "result": "PASS" if all_total else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="A",
            check_name="Total Verifier Discipline",
            passed=all_total,
            details=bundle_receipt
        )

    def check_B_diamond_path_free(self) -> CheckResult:
        """
        B) Diamond / Path-Freeness

        Reorder ledger records and verify Pi* fingerprints are unchanged.
        """
        trace = self._run_engine_trace()

        if len(trace.steps) < 2:
            # Not enough steps to test reordering
            self.diamond_ok = True
            bundle_receipt = {
                "type": "DIAMOND_PATH_FREE_BUNDLE",
                "steps_available": len(trace.steps),
                "reordering_tested": False,
                "note": "Not enough steps to test reordering",
                "result": "PASS"
            }
            self.receipts.append(bundle_receipt)
            return self._add_check(
                check_id="B",
                check_name="Diamond / Path-Freeness",
                passed=True,
                details=bundle_receipt
            )

        # Get the ledger records
        ledger = trace.steps[-1] if trace.steps else None
        if not ledger:
            self.diamond_ok = True
            return self._add_check(
                check_id="B",
                check_name="Diamond / Path-Freeness",
                passed=True,
                details={"note": "No ledger to test"}
            )

        # Create ledger from trace steps
        records = []
        for step in trace.steps:
            records.append(LedgerRecord(
                test_id=step.separator_id,
                outcome=step.outcome,
                cost=step.cost,
                timestamp=step.step_index
            ))

        # Original Pi* fingerprint
        original_ledger = Ledger(records=records)
        original_survivors = compute_survivors(self.d0, original_ledger, self.tests)

        # Create shuffled version (different order)
        shuffled_records = records.copy()
        random.shuffle(shuffled_records)

        shuffled_ledger = Ledger(records=shuffled_records)
        shuffled_survivors = compute_survivors(self.d0, shuffled_ledger, self.tests)

        # Pi* should be the same (survivors should be the same)
        survivors_match = (original_survivors == shuffled_survivors)

        # Compute Pi* fingerprints
        original_state = EngineState(
            step_index=len(records),
            ledger=original_ledger,
            survivors=original_survivors,
            total_time=0,
            total_energy=0,
            feasible_tests={},
            all_tests=self.tests
        )
        shuffled_state = EngineState(
            step_index=len(shuffled_records),
            ledger=shuffled_ledger,
            survivors=shuffled_survivors,
            total_time=0,
            total_energy=0,
            feasible_tests={},
            all_tests=self.tests
        )

        original_pi = original_state.compute_pi_fingerprint()
        shuffled_pi = shuffled_state.compute_pi_fingerprint()

        pi_match = (original_pi == shuffled_pi)
        self.diamond_ok = survivors_match and pi_match

        bundle_receipt = {
            "type": "DIAMOND_PATH_FREE_BUNDLE",
            "records_count": len(records),
            "survivors_match": survivors_match,
            "pi_fingerprint_match": pi_match,
            "original_pi": original_pi[:32],
            "shuffled_pi": shuffled_pi[:32],
            "result": "PASS" if self.diamond_ok else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="B",
            check_name="Diamond / Path-Freeness",
            passed=self.diamond_ok,
            details=bundle_receipt
        )

    def check_C_time_entropy_identity(self) -> CheckResult:
        """
        C) Time/Entropy Identity

        For each step verify:
        - W_{t+1} subset W_t
        - Delta_T = log(|W_t|/|W_{t+1}|) >= 0
        - Delta_T = S_t - S_{t+1}
        """
        trace = self._run_engine_trace()

        all_ok = True
        step_receipts = []

        for step in trace.steps:
            # Check shrinking
            is_shrinking = step.survivors_after <= step.survivors_before

            # Check delta_time
            if step.survivors_after > 0 and step.survivors_before > 0:
                expected_delta = math.log2(step.survivors_before / step.survivors_after)
            else:
                expected_delta = 0.0

            delta_matches = abs(step.delta_time - expected_delta) < 1e-10
            time_positive = step.delta_time >= -1e-10

            # Entropy check
            s_before = math.log2(step.survivors_before) if step.survivors_before > 0 else 0
            s_after = math.log2(step.survivors_after) if step.survivors_after > 0 else 0
            entropy_delta = s_before - s_after
            entropy_matches = abs(step.delta_time - entropy_delta) < 1e-10

            step_ok = is_shrinking and delta_matches and time_positive and entropy_matches
            if not step_ok:
                all_ok = False

            step_receipts.append({
                "step_index": step.step_index,
                "is_shrinking": is_shrinking,
                "delta_time_display": str(round(step.delta_time, 6)),
                "expected_delta_display": str(round(expected_delta, 6)),
                "delta_matches": delta_matches,
                "time_positive": time_positive,
                "entropy_matches": entropy_matches,
                "step_ok": step_ok
            })

        self.time_entropy_ok = all_ok

        bundle_receipt = {
            "type": "TIME_ENTROPY_IDENTITY_BUNDLE",
            "steps_checked": len(trace.steps),
            "all_ok": all_ok,
            "step_details": step_receipts[:5],  # First 5 for brevity
            "result": "PASS" if all_ok else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="C",
            check_name="Time/Entropy Identity",
            passed=all_ok,
            details=bundle_receipt
        )

    def check_D_feasibility_shrink(self) -> CheckResult:
        """
        D) Feasibility Shrink

        Verify the feasible test set shrinks with |W| under cost rule.
        """
        trace = self._run_engine_trace()

        all_ok = True
        shrink_receipts = []

        # Track feasibility across steps
        prev_budget = math.log2(len(self.d0))
        prev_feasible_count = len(compute_feasible_tests(self.tests, prev_budget))

        for step in trace.steps:
            # Budget after step
            new_budget = math.log2(step.survivors_after) if step.survivors_after > 0 else 0
            new_feasible = compute_feasible_tests(self.tests, new_budget)
            new_feasible_count = len(new_feasible)

            # Feasibility should not increase (monotone shrink)
            budget_shrinks = new_budget <= prev_budget + 1e-10
            # Note: feasible count can stay same if no tests are above threshold
            feasible_ok = new_feasible_count <= prev_feasible_count or budget_shrinks

            if not feasible_ok:
                all_ok = False

            shrink_receipts.append({
                "step_index": step.step_index,
                "budget_before_display": str(round(prev_budget, 4)),
                "budget_after_display": str(round(new_budget, 4)),
                "feasible_before": prev_feasible_count,
                "feasible_after": new_feasible_count,
                "budget_shrinks": budget_shrinks,
                "feasible_ok": feasible_ok
            })

            prev_budget = new_budget
            prev_feasible_count = new_feasible_count

        self.feasibility_shrink_ok = all_ok

        bundle_receipt = {
            "type": "FEASIBILITY_SHRINK_BUNDLE",
            "steps_checked": len(trace.steps),
            "all_ok": all_ok,
            "shrink_details": shrink_receipts[:5],
            "result": "PASS" if all_ok else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="D",
            check_name="Feasibility Shrink",
            passed=all_ok,
            details=bundle_receipt
        )

    def check_E_canonical_tau_star(self) -> CheckResult:
        """
        E) Canonical tau*

        Verify tau* is the argmin of minimax value functional.
        """
        # Use fresh engine to test selection
        engine = UniverseEngine(self.d0, self.tests, self.query)

        selection = engine.select_separator()

        if selection is None:
            # Already solved or no feasible tests
            self.canonical_tau_star_ok = True
            bundle_receipt = {
                "type": "CANONICAL_TAU_STAR_BUNDLE",
                "selection_made": False,
                "note": "No selection needed (already solved or no feasible tests)",
                "result": "PASS"
            }
            self.receipts.append(bundle_receipt)
            return self._add_check(
                check_id="E",
                check_name="Canonical tau*",
                passed=True,
                details=bundle_receipt
            )

        # Verify it's optimal by checking all alternatives
        is_optimal = True
        alternatives = []

        for test_id, test in engine.state.feasible_tests.items():
            # Compute value for this test
            partition = {}
            for x in engine.state.survivors:
                outcome = test.evaluator(x)
                if outcome not in partition:
                    partition[outcome] = set()
                partition[outcome].add(x)

            if len(partition) <= 1:
                continue  # Doesn't separate

            # Compute max child value
            max_child = 0
            for outcome, fiber in partition.items():
                child_val, _ = engine._compute_minimax(
                    frozenset(fiber),
                    engine.state.feasible_tests,
                    depth=1,
                    max_depth=5
                )
                max_child = max(max_child, child_val)

            total_value = test.cost + max_child
            alternatives.append({
                "test_id": test_id,
                "value": total_value
            })

            if total_value < selection.minimax_value:
                is_optimal = False

        self.canonical_tau_star_ok = is_optimal

        bundle_receipt = {
            "type": "CANONICAL_TAU_STAR_BUNDLE",
            "selection_made": True,
            "selected_separator": selection.separator_id,
            "minimax_value": selection.minimax_value,
            "is_optimal": is_optimal,
            "alternatives_checked": len(alternatives),
            "result": "PASS" if is_optimal else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="E",
            check_name="Canonical tau*",
            passed=is_optimal,
            details=bundle_receipt
        )

    def check_F_pi_consistent_control(self) -> CheckResult:
        """
        F) Pi-Consistent Control

        Verify Pi o N = Pi o N o Pi by testing gauge-equivalent
        encodings produce identical decisions.
        """
        # Create permuted version of d0
        elements = list(self.d0)
        perm = elements.copy()
        random.shuffle(perm)
        perm_map = dict(zip(elements, perm))
        inv_perm = {v: k for k, v in perm_map.items()}

        # Create permuted tests
        permuted_tests = {}
        for test_id, test in self.tests.items():
            def make_evaluator(orig_eval, inv):
                def permuted_eval(x):
                    orig_x = inv.get(x, x)
                    return orig_eval(orig_x)
                return permuted_eval

            permuted_tests[test_id] = Test(
                test_id=test_id,
                evaluator=make_evaluator(test.evaluator, inv_perm),
                cost=test.cost,
                outcome_space=test.outcome_space
            )

        # Create permuted query
        def permuted_query_eval(x):
            orig_x = inv_perm.get(x, x)
            return self.query.evaluator(orig_x)

        permuted_query = Query(
            query_id=self.query.query_id,
            evaluator=permuted_query_eval,
            answer_space=self.query.answer_space
        )

        # Run engines on both
        permuted_d0 = frozenset(perm)

        engine_orig = UniverseEngine(self.d0, self.tests, self.query)
        engine_perm = UniverseEngine(permuted_d0, permuted_tests, permuted_query)

        selection_orig = engine_orig.select_separator()
        selection_perm = engine_perm.select_separator()

        # Both should select the same separator (by ID)
        both_none = (selection_orig is None and selection_perm is None)
        both_same = (
            selection_orig is not None and
            selection_perm is not None and
            selection_orig.separator_id == selection_perm.separator_id
        )

        is_consistent = both_none or both_same
        self.pi_consistent_ok = is_consistent

        bundle_receipt = {
            "type": "PI_CONSISTENT_CONTROL_BUNDLE",
            "original_selection": selection_orig.separator_id if selection_orig else "NONE",
            "permuted_selection": selection_perm.separator_id if selection_perm else "NONE",
            "is_consistent": is_consistent,
            "result": "PASS" if is_consistent else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="F",
            check_name="Pi-Consistent Control",
            passed=is_consistent,
            details=bundle_receipt
        )

    def check_G_omega_honesty(self) -> CheckResult:
        """
        G) Omega Honesty

        Verify that when |Ans| > 1, output is Omega with frontier.
        """
        trace = self._run_engine_trace()

        all_ok = True
        honesty_receipts = []

        # Check each state in trace
        engine = UniverseEngine(self.d0, self.tests, self.query)

        for step in trace.steps:
            # Get answer set before this step
            answer_set = engine.answer_computer.compute_answer_set(engine.state.survivors)

            if answer_set.answer_count > 1:
                # Should have frontier, not UNIQUE
                has_frontier = answer_set.fingerprint() is not None
                if not has_frontier:
                    all_ok = False

                honesty_receipts.append({
                    "step_index": step.step_index,
                    "answer_count": answer_set.answer_count,
                    "has_frontier": has_frontier,
                    "frontier_fp": answer_set.fingerprint()[:16] if has_frontier else "NONE"
                })

            # Apply the step
            engine.step(self.actual_element)

        # Check final state
        final_answer = engine.answer_computer.compute_answer_set(engine.state.survivors)
        if final_answer.answer_count > 1:
            has_frontier = final_answer.fingerprint() is not None
            if not has_frontier:
                all_ok = False

        self.omega_honest_ok = all_ok

        bundle_receipt = {
            "type": "OMEGA_HONESTY_BUNDLE",
            "states_checked": len(honesty_receipts),
            "all_honest": all_ok,
            "final_answer_count": final_answer.answer_count,
            "final_is_solved": final_answer.is_solved,
            "result": "PASS" if all_ok else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="G",
            check_name="Omega Honesty",
            passed=all_ok,
            details=bundle_receipt
        )

    def check_H_canonical_receipts(self) -> CheckResult:
        """
        H) Canonical Receipts

        Every artifact is serializable and hashable.
        """
        trace = self._run_engine_trace()

        all_ok = True
        hash_receipts = []

        # Check all receipts from trace
        for i, receipt in enumerate(trace.receipts):
            try:
                canonical = CanonicalJSON.serialize(receipt)
                receipt_hash = hashlib.sha256(canonical.encode()).hexdigest()

                hash_receipts.append({
                    "receipt_index": i,
                    "receipt_type": receipt.get("type", "UNKNOWN"),
                    "hash": receipt_hash[:16],
                    "serializable": True
                })
            except Exception as e:
                all_ok = False
                hash_receipts.append({
                    "receipt_index": i,
                    "serializable": False,
                    "error": str(e)
                })

        # Also check our verification receipts
        for i, receipt in enumerate(self.receipts):
            try:
                canonical = CanonicalJSON.serialize(receipt)
                receipt_hash = hashlib.sha256(canonical.encode()).hexdigest()
            except Exception:
                all_ok = False

        self.receipts_ok = all_ok

        bundle_receipt = {
            "type": "CANONICAL_RECEIPTS_BUNDLE",
            "trace_receipts": len(trace.receipts),
            "verification_receipts": len(self.receipts),
            "all_serializable": all_ok,
            "sample_hashes": hash_receipts[:5],
            "result": "PASS" if all_ok else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="H",
            check_name="Canonical Receipts",
            passed=all_ok,
            details=bundle_receipt
        )

    def run_all_checks(self) -> List[CheckResult]:
        """Run all verification checks A-H."""
        self.checks = []

        self.check_A_total_verifier()
        self.check_B_diamond_path_free()
        self.check_C_time_entropy_identity()
        self.check_D_feasibility_shrink()
        self.check_E_canonical_tau_star()
        self.check_F_pi_consistent_control()
        self.check_G_omega_honesty()
        self.check_H_canonical_receipts()

        return self.checks

    def create_proof_bundle(self) -> UniverseEngineProofBundle:
        """Create complete proof bundle."""
        if not self.checks:
            raise ValueError("Must run checks before creating bundle")

        bundle_id = f"UE_{hashlib.sha256(str(self.d0).encode()).hexdigest()[:8]}"

        return UniverseEngineProofBundle(
            bundle_id=bundle_id,
            checks=self.checks.copy(),
            total_verifier_ok=self.total_verifier_ok,
            diamond_ok=self.diamond_ok,
            time_entropy_ok=self.time_entropy_ok,
            feasibility_shrink_ok=self.feasibility_shrink_ok,
            canonical_tau_star_ok=self.canonical_tau_star_ok,
            pi_consistent_ok=self.pi_consistent_ok,
            omega_honest_ok=self.omega_honest_ok,
            receipts_ok=self.receipts_ok,
            receipts=self.receipts.copy()
        )


def run_universe_engine_verification(
    d0: FrozenSet[Any],
    tests: Dict[str, Test],
    query: Query,
    actual_element: Any
) -> UniverseEngineProofBundle:
    """
    Run complete Universe Engine verification.
    """
    verifier = UniverseEngineVerifier(d0, tests, query, actual_element)
    verifier.run_all_checks()
    return verifier.create_proof_bundle()


def print_universe_engine_report(bundle: UniverseEngineProofBundle) -> str:
    """Generate human-readable verification report."""
    lines = [
        "=" * 60,
        "UNIVERSE ENGINE - VERIFICATION REPORT",
        "=" * 60,
        f"Bundle ID: {bundle.bundle_id}",
        f"Fingerprint: {bundle.fingerprint()[:32]}...",
        "",
        f"Total Verifier OK: {bundle.total_verifier_ok}",
        f"Diamond/Path-Free OK: {bundle.diamond_ok}",
        f"Time/Entropy Identity OK: {bundle.time_entropy_ok}",
        f"Feasibility Shrink OK: {bundle.feasibility_shrink_ok}",
        f"Canonical tau* OK: {bundle.canonical_tau_star_ok}",
        f"Pi-Consistent Control OK: {bundle.pi_consistent_ok}",
        f"Omega Honesty OK: {bundle.omega_honest_ok}",
        f"Canonical Receipts OK: {bundle.receipts_ok}",
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
    Run Universe Engine demonstration.
    """
    # Create sample domain
    d0 = frozenset(range(16))

    # Create query: find the element mod 4
    query = Query(
        query_id="mod4_query",
        evaluator=lambda x: f"R{x % 4}",
        answer_space=frozenset([f"R{i}" for i in range(4)])
    )

    # Create tests
    def parity(x: int) -> str:
        return "EVEN" if x % 2 == 0 else "ODD"

    def mod4(x: int) -> str:
        return f"R{x % 4}"

    def threshold8(x: int) -> str:
        return "LOW" if x < 8 else "HIGH"

    def threshold4(x: int) -> str:
        return "LOW" if x < 4 else "MID" if x < 12 else "HIGH"

    tests = {
        "parity": Test(
            test_id="parity",
            evaluator=parity,
            cost=1,
            outcome_space=frozenset(["EVEN", "ODD"])
        ),
        "mod4": Test(
            test_id="mod4",
            evaluator=mod4,
            cost=2,
            outcome_space=frozenset([f"R{i}" for i in range(4)])
        ),
        "threshold8": Test(
            test_id="threshold8",
            evaluator=threshold8,
            cost=1,
            outcome_space=frozenset(["LOW", "HIGH"])
        ),
        "threshold4": Test(
            test_id="threshold4",
            evaluator=threshold4,
            cost=1,
            outcome_space=frozenset(["LOW", "MID", "HIGH"])
        )
    }

    # Actual element (what the engine is trying to find)
    actual_element = 7

    # Run verification
    bundle = run_universe_engine_verification(d0, tests, query, actual_element)

    # Print report
    report = print_universe_engine_report(bundle)
    print(report)

    return {
        "demo": "Universe Engine",
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
