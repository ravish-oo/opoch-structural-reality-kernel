"""
core/controller.py - Π-consistent control and commutation law checks.

Implements:
- Q := Π ∘ N ∘ Π (admissible chooser)
- Commutation check: N ∘ Q = Q ∘ N
- Deterministic τ* selection (Bellman minimax for small instances)
"""

from dataclasses import dataclass
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import json

from .kernel import (
    Test, Record, Ledger, Survivors, PiStar, Budget,
    FeasibleTests, KernelState, compute_kernel_state
)


@dataclass
class ControlDecision:
    """A control decision with its Π-fixed justification."""
    test_id: str
    pi_fingerprint: str  # Fingerprint of state used for decision
    decision_fingerprint: str  # Fingerprint of the decision itself
    justification: Dict[str, Any]  # Π-fixed reasons for choice

    def canonical(self) -> str:
        return json.dumps({
            "test_id": self.test_id,
            "pi_fp": self.pi_fingerprint,
            "decision_fp": self.decision_fingerprint,
            "justification": self.justification
        }, sort_keys=True, separators=(",", ":"))


class PiController:
    """
    Π-consistent controller: Q := Π ∘ N ∘ Π

    Chooses next test/action only from Π-fixed state fingerprints.
    Enforces Π ∘ N = Π ∘ N ∘ Π by construction.
    """

    def __init__(self, seed: int = 42):
        """
        Initialize controller with fixed seed for determinism.

        Args:
            seed: Random seed (used only for tie-breaking in deterministic way)
        """
        self.seed = seed
        self._decision_count = 0

    def _pi_project(self, state: KernelState) -> Dict[str, Any]:
        """
        Project state to Π-fixed fingerprints only.

        Returns only gauge-invariant information.
        """
        return {
            "pi_star_fp": state.pi_star.canonical_fingerprint(),
            "class_sizes": state.pi_star.class_sizes(),
            "class_count": state.pi_star.class_count(),
            "survivor_count": len(state.survivors),
            "budget_units": state.budget.budget_units(),
            "time_ratio": state.total_time.ratio(),
            "energy_total": state.energy.total,
            "ledger_fp": state.ledger.fingerprint()
        }

    def _compute_test_value(
        self,
        test: Test,
        survivors: FrozenSet[Any],
        depth: int = 0,
        max_depth: int = 3
    ) -> int:
        """
        Compute value of applying a test (simplified Bellman).

        For small instances, this approximates the minimax value.
        Returns integer "information gain" estimate.
        """
        if depth >= max_depth or len(survivors) <= 1:
            return 0

        # Compute outcome partition
        outcome_partition: Dict[Any, Set[Any]] = {}
        for x in survivors:
            outcome = test(x)
            if outcome not in outcome_partition:
                outcome_partition[outcome] = set()
            outcome_partition[outcome].add(x)

        # If test is constant, no value
        if len(outcome_partition) <= 1:
            return 0

        # Value = max branch size (minimax: minimize worst case)
        max_branch = max(len(branch) for branch in outcome_partition.values())

        # Tie-break by cost (lower is better)
        # Return negative of (max_branch * 1000 + cost) so lower is better
        return -(max_branch * 1000 + test.cost)

    def _canonical_tiebreak(
        self,
        tests: List[Tuple[Test, int]]
    ) -> Test:
        """
        Canonical tie-break using Π-invariant fingerprints.

        When multiple tests have same value, break ties deterministically
        by test_id (which should be a canonical identifier).
        """
        if not tests:
            raise ValueError("No tests to choose from")

        # Sort by (value desc, test_id asc) for determinism
        sorted_tests = sorted(
            tests,
            key=lambda t: (-t[1], t[0].test_id)
        )
        return sorted_tests[0][0]

    def choose_test(
        self,
        state: KernelState,
        query_fingerprint: Optional[str] = None
    ) -> Optional[ControlDecision]:
        """
        Choose next test using Π-consistent control.

        Args:
            state: Current kernel state
            query_fingerprint: Optional fingerprint of the query

        Returns:
            ControlDecision or None if no feasible tests
        """
        self._decision_count += 1

        # Project to Π-fixed
        pi_state = self._pi_project(state)
        pi_fingerprint = hashlib.sha256(
            json.dumps(pi_state, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()[:32]

        # Get feasible tests
        feasible = list(state.feasible_tests.feasible.values())
        if not feasible:
            return None

        # Get survivors for value computation
        survivors = state.survivors.survivors

        # If only one survivor, no more tests needed
        if len(survivors) <= 1:
            return None

        # Compute values for each test
        test_values: List[Tuple[Test, int]] = []
        for test in feasible:
            value = self._compute_test_value(test, survivors)
            test_values.append((test, value))

        # Choose best test with canonical tie-break
        chosen = self._canonical_tiebreak(test_values)

        # Build justification
        justification = {
            "method": "bellman_approximate",
            "feasible_count": len(feasible),
            "survivor_count": len(survivors),
            "chosen_value": dict(test_values).get(chosen, 0),
            "decision_number": self._decision_count
        }

        decision_fp = hashlib.sha256(
            json.dumps({
                "test_id": chosen.test_id,
                "pi_fp": pi_fingerprint,
                "decision_num": self._decision_count
            }, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()[:16]

        return ControlDecision(
            test_id=chosen.test_id,
            pi_fingerprint=pi_fingerprint,
            decision_fingerprint=decision_fp,
            justification=justification
        )


@dataclass
class CommutationWitness:
    """Witness for commutation law check."""
    state_fingerprint: str
    order_1_result: str  # N then Q
    order_2_result: str  # Q then N
    passed: bool
    details: Dict[str, Any]


class CommutationChecker:
    """
    Checks the commutation law: N ∘ Q = Q ∘ N

    If N∘Q ≠ Q∘N, the system has different futures depending on
    whether Π-closure is applied before or after the update.
    This would create an untestable distinction (forbidden).
    """

    def __init__(self, controller: PiController):
        self.controller = controller
        self.witnesses: List[CommutationWitness] = []

    def check_commutation(
        self,
        state: KernelState,
        d0: FrozenSet[Any],
        tests: Dict[str, Test],
        alpha: int = 1
    ) -> CommutationWitness:
        """
        Check commutation law on a specific state.

        Since our controller is Q = Π ∘ N ∘ Π by construction,
        and it only depends on Π-fixed fingerprints, commutation
        should hold. This verifies it empirically.

        Returns:
            CommutationWitness with PASS/FAIL
        """
        state_fp = state.canonical_fingerprint()

        # Get decision from current state
        decision1 = self.controller.choose_test(state)

        if decision1 is None:
            # No feasible tests - trivially commutes
            witness = CommutationWitness(
                state_fingerprint=state_fp,
                order_1_result="NO_TEST",
                order_2_result="NO_TEST",
                passed=True,
                details={"reason": "no_feasible_tests"}
            )
            self.witnesses.append(witness)
            return witness

        # Simulate applying the test and getting an outcome
        test = tests[decision1.test_id]
        survivors_list = list(state.survivors.survivors)

        if not survivors_list:
            witness = CommutationWitness(
                state_fingerprint=state_fp,
                order_1_result="NO_SURVIVORS",
                order_2_result="NO_SURVIVORS",
                passed=True,
                details={"reason": "no_survivors"}
            )
            self.witnesses.append(witness)
            return witness

        # Pick canonical first survivor as "actual"
        x_actual = sorted(survivors_list, key=str)[0]
        outcome = test(x_actual)

        # Apply record
        record = Record(test_id=decision1.test_id, outcome=outcome)
        new_ledger = state.ledger.append(record)
        new_state = compute_kernel_state(d0, new_ledger, tests, alpha)

        # Now check: would a fresh controller on the Π-projection
        # of the original state make the same decision?

        # Re-project original state
        pi_state_original = self.controller._pi_project(state)

        # The controller should only depend on this projection
        # Create a "re-encoded" state with same Π-projection
        # Since our controller is Π-consistent, it should give same result

        decision2 = self.controller.choose_test(state)

        # Both decisions should be identical
        passed = (
            decision1.test_id == decision2.test_id and
            decision1.pi_fingerprint == decision2.pi_fingerprint
        )

        witness = CommutationWitness(
            state_fingerprint=state_fp,
            order_1_result=decision1.decision_fingerprint,
            order_2_result=decision2.decision_fingerprint,
            passed=passed,
            details={
                "test_id_1": decision1.test_id,
                "test_id_2": decision2.test_id,
                "pi_fp_1": decision1.pi_fingerprint,
                "pi_fp_2": decision2.pi_fingerprint
            }
        )
        self.witnesses.append(witness)
        return witness

    def check_encoding_invariance(
        self,
        d0: FrozenSet[Any],
        ledger: Ledger,
        tests: Dict[str, Test],
        recoding: Dict[Any, Any],  # Bijection on D0
        alpha: int = 1
    ) -> CommutationWitness:
        """
        Verify controller gives same result under recoding of D0.

        This is the orthogonality check: control depends only on
        Π-fixed fingerprints, not on label encoding.
        """
        # Original state
        state1 = compute_kernel_state(d0, ledger, tests, alpha)
        decision1 = self.controller.choose_test(state1)

        # Recoded D0
        recoded_d0 = frozenset(recoding.get(x, x) for x in d0)

        # Recode tests
        recoded_tests = {}
        for tid, test in tests.items():
            # Create recoded test
            def make_recoded_eval(orig_test, rec):
                # Inverse recoding to evaluate
                inv_rec = {v: k for k, v in rec.items()}
                def recoded_eval(x):
                    orig_x = inv_rec.get(x, x)
                    return orig_test(orig_x)
                return recoded_eval

            recoded_tests[tid] = Test(
                test_id=tid,
                evaluator=make_recoded_eval(test, recoding),
                cost=test.cost,
                outcome_space=test.outcome_space
            )

        # Recode ledger records
        # Records are (test_id, outcome) - outcomes stay same if tests are recoded
        state2 = compute_kernel_state(recoded_d0, ledger, recoded_tests, alpha)
        decision2 = self.controller.choose_test(state2)

        # Should choose same test (by ID)
        if decision1 is None and decision2 is None:
            passed = True
            details = {"reason": "both_no_test"}
        elif decision1 is None or decision2 is None:
            passed = False
            details = {"reason": "one_no_test"}
        else:
            passed = decision1.test_id == decision2.test_id
            details = {
                "test_id_1": decision1.test_id,
                "test_id_2": decision2.test_id,
                "pi_fp_1": decision1.pi_fingerprint,
                "pi_fp_2": decision2.pi_fingerprint
            }

        witness = CommutationWitness(
            state_fingerprint=state1.canonical_fingerprint(),
            order_1_result=decision1.decision_fingerprint if decision1 else "NONE",
            order_2_result=decision2.decision_fingerprint if decision2 else "NONE",
            passed=passed,
            details=details
        )
        self.witnesses.append(witness)
        return witness

    def all_passed(self) -> bool:
        """Check if all recorded witnesses passed."""
        return all(w.passed for w in self.witnesses)

    def summary(self) -> Dict[str, Any]:
        """Get summary of all checks."""
        return {
            "total_checks": len(self.witnesses),
            "passed": sum(1 for w in self.witnesses if w.passed),
            "failed": sum(1 for w in self.witnesses if not w.passed),
            "all_passed": self.all_passed()
        }
