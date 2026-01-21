"""
core/universe_engine.py - The Universe Engine: ⊥ → ⊥op evolution.

Implements:
- Internal objective K (minimize underdetermination)
- Deterministic update loop
- ⊥op termination (operational nothingness)
- Self-improvement within constraints
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import json

from .kernel import (
    Test, Record, Ledger, Survivors, PiStar, Budget,
    FeasibleTests, KernelState, compute_kernel_state
)
from .nsl import NSLState, NSLEngine, Distinction, Trit, NSLClosure
from .controller import PiController, ControlDecision, CommutationChecker
from .gauge import Canonicalizer, GaugeChecker
from .receipts import CanonicalJSON, ReceiptChain, Receipt
from .theorem_generator import Contract, KernelOutput, TheoremGenerator


@dataclass
class BotOp:
    """
    ⊥op: Operational Nothingness.

    The terminal state where:
    - |W(L)| = 1 (unique survivor)
    - All distinctions decided
    - No remaining budget for further tests
    - Or: equivalently all feasible tests are constant on survivors
    """
    survivor: Any
    final_state_fp: str
    total_time_ratio: Tuple[int, int]
    total_energy: int
    ledger_fp: str

    def canonical(self) -> str:
        """Canonical representation."""
        return CanonicalJSON.serialize({
            "survivor": str(self.survivor),
            "final_fp": self.final_state_fp,
            "time_ratio": list(self.total_time_ratio),
            "energy": self.total_energy,
            "ledger_fp": self.ledger_fp
        })

    def fingerprint(self) -> str:
        """SHA-256 fingerprint."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    @staticmethod
    def is_bot_op(state: KernelState) -> bool:
        """Check if state is ⊥op."""
        # Condition 1: Single survivor
        if len(state.survivors) != 1:
            return False

        # Condition 2: All feasible tests are constant on survivor
        survivor = next(iter(state.survivors.survivors))
        for test in state.feasible_tests:
            # Test is already constant on single element
            pass

        return True


@dataclass
class UniverseState:
    """
    Complete universe state at time t.

    Combines kernel state with NSL state and controller.
    """
    kernel_state: KernelState
    nsl_state: NSLState
    step: int
    objective_value: int  # K = underdetermination measure

    def canonical(self) -> str:
        """Canonical representation."""
        return CanonicalJSON.serialize({
            "kernel_fp": self.kernel_state.canonical_fingerprint(),
            "nsl_fp": self.nsl_state.fingerprint(),
            "step": self.step,
            "K": self.objective_value
        })

    def fingerprint(self) -> str:
        """SHA-256 fingerprint."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()


class UniverseEngine:
    """
    The Universe Engine: deterministic evolution from ⊥ to ⊥op.

    Core loop:
    1. Compute current state S_t
    2. Select next test τ* via Π-consistent control
    3. Apply test, record outcome
    4. Update to S_{t+1}
    5. Repeat until ⊥op or budget exhausted

    Internal objective K: minimize |unknown distinctions| + log|survivors|
    """

    def __init__(
        self,
        d0: FrozenSet[Any],
        tests: Dict[str, Test],
        seed: int = 42,
        alpha: int = 1
    ):
        """
        Initialize universe engine.

        Args:
            d0: Initial candidate set D0
            tests: Available tests
            seed: Random seed for tie-breaking
            alpha: Budget scaling factor
        """
        self.d0 = d0
        self.tests = tests
        self.seed = seed
        self.alpha = alpha

        # Initialize components
        self.ledger = Ledger()
        self.controller = PiController(seed=seed)
        self.receipt_chain = ReceiptChain()
        self.gauge_checker = GaugeChecker()

        # Build distinctions from tests
        self.distinctions = self._build_distinctions()

        # Initialize states
        self.kernel_state = compute_kernel_state(d0, self.ledger, tests, alpha)
        self.nsl_engine = NSLEngine(self.distinctions)

        # History
        self.history: List[UniverseState] = []
        self.step = 0

    def _build_distinctions(self) -> Dict[str, Distinction]:
        """Build distinction set from tests."""
        distinctions = {}
        for test_id, test in self.tests.items():
            for outcome in test.outcome_space:
                d_id = f"{test_id}:{outcome}"
                distinctions[d_id] = Distinction(
                    distinction_id=d_id,
                    description=f"Test {test_id} gives outcome {outcome}",
                    test_id=test_id,
                    outcome=outcome
                )
        return distinctions

    def _compute_objective(self) -> int:
        """
        Compute internal objective K.

        K = |unknown distinctions| * 1000 + log2(|survivors|) * 100

        Lower is better.
        """
        import math

        unknown_count = self.nsl_engine.state.count_unknown()
        survivor_count = len(self.kernel_state.survivors)

        if survivor_count <= 1:
            log_survivors = 0
        else:
            log_survivors = int(math.log2(survivor_count) * 100)

        return unknown_count * 1000 + log_survivors

    def _record_state(self) -> UniverseState:
        """Record current state."""
        state = UniverseState(
            kernel_state=self.kernel_state,
            nsl_state=self.nsl_engine.state.copy(),
            step=self.step,
            objective_value=self._compute_objective()
        )
        self.history.append(state)
        return state

    def _apply_test(self, test: Test, actual_x: Any) -> Record:
        """
        Apply test to actual element and record.

        Updates:
        - Ledger with new record
        - NSL state with verified/refuted distinctions
        - Kernel state
        """
        pre_state_fp = self.kernel_state.canonical_fingerprint()
        pre_survivors = len(self.kernel_state.survivors)

        # Apply test
        outcome = test(actual_x)
        record = Record(test_id=test.test_id, outcome=outcome)

        # Update ledger
        self.ledger = self.ledger.append(record)

        # Update NSL state
        d_id = f"{test.test_id}:{outcome}"
        if d_id in self.distinctions:
            self.nsl_engine.state.verify(d_id)

        # Mark other outcomes for this test as refuted
        for other_outcome in test.outcome_space:
            if other_outcome != outcome:
                other_d_id = f"{test.test_id}:{other_outcome}"
                if other_d_id in self.distinctions:
                    self.nsl_engine.state.refute(other_d_id)

        # Recompute kernel state
        self.kernel_state = compute_kernel_state(
            self.d0, self.ledger, self.tests, self.alpha
        )
        self.kernel_state.energy.record_cost(test)

        post_state_fp = self.kernel_state.canonical_fingerprint()
        post_survivors = len(self.kernel_state.survivors)

        # Create receipt
        self.receipt_chain.append(
            operation="APPLY_TEST",
            pre_state_fp=pre_state_fp,
            post_state_fp=post_state_fp,
            payload={
                "test_id": test.test_id,
                "outcome": str(outcome),
                "w_pre": pre_survivors,
                "w_post": post_survivors,
                "cost": test.cost
            }
        )

        return record

    def _select_actual(self) -> Optional[Any]:
        """
        Select 'actual' element from survivors.

        For simulation, picks canonical first survivor.
        In reality, this would be the unknown physical reality.
        """
        survivors = self.kernel_state.survivors.survivors
        if not survivors:
            return None
        return sorted(survivors, key=str)[0]

    def step_forward(self) -> Optional[ControlDecision]:
        """
        Execute one step of universe evolution.

        Returns:
            ControlDecision if a test was applied, None if ⊥op reached
        """
        self.step += 1

        # Record pre-step state
        self._record_state()

        # Check for ⊥op
        if BotOp.is_bot_op(self.kernel_state):
            return None

        # Select next test
        decision = self.controller.choose_test(self.kernel_state)
        if decision is None:
            return None

        # Get actual element
        actual_x = self._select_actual()
        if actual_x is None:
            return None

        # Apply test
        test = self.tests[decision.test_id]
        self._apply_test(test, actual_x)

        return decision

    def run_to_completion(self, max_steps: int = 1000) -> BotOp:
        """
        Run universe evolution until ⊥op or max steps.

        Returns:
            BotOp terminal state
        """
        # Initial receipt
        self.receipt_chain.append(
            operation="INIT",
            pre_state_fp="GENESIS",
            post_state_fp=self.kernel_state.canonical_fingerprint(),
            payload={
                "d0_size": len(self.d0),
                "test_count": len(self.tests),
                "alpha": self.alpha
            }
        )

        for _ in range(max_steps):
            decision = self.step_forward()
            if decision is None:
                break

        # Record final state
        self._record_state()

        # Create ⊥op
        survivors = self.kernel_state.survivors.survivors
        survivor = next(iter(survivors)) if survivors else None

        bot_op = BotOp(
            survivor=survivor,
            final_state_fp=self.kernel_state.canonical_fingerprint(),
            total_time_ratio=self.kernel_state.total_time.ratio(),
            total_energy=self.kernel_state.energy.total,
            ledger_fp=self.ledger.fingerprint()
        )

        # Final receipt
        self.receipt_chain.append(
            operation="COMPLETE",
            pre_state_fp=self.kernel_state.canonical_fingerprint(),
            post_state_fp=bot_op.fingerprint(),
            payload={
                "survivor": str(survivor),
                "steps": self.step,
                "total_energy": self.kernel_state.energy.total,
                "final_survivors": len(survivors)
            }
        )

        return bot_op

    def run_with_contract(self, contract: Contract) -> KernelOutput:
        """
        Run universe as theorem generator for a contract.
        """
        generator = TheoremGenerator(seed=self.seed)
        return generator.run(contract)

    def get_audit_bundle(self) -> Dict[str, Any]:
        """
        Get complete audit bundle for this run.
        """
        chain_valid, broken_at = self.receipt_chain.verify_chain_integrity()

        return {
            "chain_fingerprint": self.receipt_chain.chain_fingerprint(),
            "receipt_count": len(self.receipt_chain),
            "chain_valid": chain_valid,
            "broken_at": broken_at,
            "steps": self.step,
            "final_state_fp": self.kernel_state.canonical_fingerprint(),
            "history_length": len(self.history)
        }

    def verify_gauge_invariance(self) -> Dict[str, Any]:
        """
        Verify gauge invariance of the run.
        """
        from .gauge import create_random_recoding

        # Check recoding invariance
        recoding = create_random_recoding(self.d0, seed=self.seed + 1)
        result = self.gauge_checker.check_recoding_invariance(
            self.d0, self.ledger, self.tests, recoding, self.alpha
        )

        # Check order invariance
        records = self.ledger.records_list()
        if len(records) >= 2:
            order_result = self.gauge_checker.check_order_invariance(
                self.d0, records, self.tests, self.alpha
            )
        else:
            order_result = {"passed": True, "reason": "insufficient records"}

        return {
            "recoding_invariance": result,
            "order_invariance": order_result,
            "all_passed": self.gauge_checker.all_passed()
        }


class SelfImprovingEngine(UniverseEngine):
    """
    Self-improving universe engine.

    Can generate new tests and refine its own objective function,
    within the constraints of the kernel axioms.
    """

    def __init__(
        self,
        d0: FrozenSet[Any],
        tests: Dict[str, Test],
        test_generator: Optional[Callable[[KernelState], Optional[Test]]] = None,
        seed: int = 42,
        alpha: int = 1
    ):
        super().__init__(d0, tests, seed, alpha)
        self.test_generator = test_generator
        self.generated_tests: List[Test] = []

    def _maybe_generate_test(self) -> Optional[Test]:
        """
        Maybe generate a new test to improve resolution.

        Returns new test if generator produces one.
        """
        if self.test_generator is None:
            return None

        new_test = self.test_generator(self.kernel_state)
        if new_test is None:
            return None

        # Add to tests
        self.tests[new_test.test_id] = new_test
        self.generated_tests.append(new_test)

        # Add distinctions
        for outcome in new_test.outcome_space:
            d_id = f"{new_test.test_id}:{outcome}"
            self.distinctions[d_id] = Distinction(
                distinction_id=d_id,
                description=f"Generated test {new_test.test_id} gives {outcome}",
                test_id=new_test.test_id,
                outcome=outcome
            )

        # Rebuild NSL state with new distinctions
        new_nsl_state = NSLState(self.distinctions)
        # Copy old state values
        for d_id in self.nsl_engine.state._state:
            if d_id in new_nsl_state.distinctions:
                new_nsl_state._state[d_id] = self.nsl_engine.state._state[d_id]
        self.nsl_engine.state = new_nsl_state

        # Recompute kernel state
        self.kernel_state = compute_kernel_state(
            self.d0, self.ledger, self.tests, self.alpha
        )

        return new_test

    def step_forward(self) -> Optional[ControlDecision]:
        """
        Execute one step with possible test generation.
        """
        # Maybe generate new test if stuck
        if len(self.kernel_state.feasible_tests) == 0:
            new_test = self._maybe_generate_test()
            if new_test:
                # Recompute feasible tests
                self.kernel_state = compute_kernel_state(
                    self.d0, self.ledger, self.tests, self.alpha
                )

        return super().step_forward()


def create_binary_search_universe(
    target: int,
    low: int = 0,
    high: int = 100
) -> UniverseEngine:
    """
    Create a universe for binary search.

    D0 = {low, low+1, ..., high}
    Tests = comparison tests
    Target = the unknown "actual" element
    """
    d0 = frozenset(range(low, high + 1))

    # Create comparison tests for each threshold
    tests: Dict[str, Test] = {}
    for threshold in range(low, high + 1):
        def make_comparator(t: int) -> Callable[[int], str]:
            def compare(x: int) -> str:
                if x < t:
                    return "LESS"
                elif x > t:
                    return "GREATER"
                else:
                    return "EQUAL"
            return compare

        tests[f"cmp_{threshold}"] = Test(
            test_id=f"cmp_{threshold}",
            evaluator=make_comparator(threshold),
            cost=1,
            outcome_space=frozenset(["LESS", "EQUAL", "GREATER"])
        )

    return UniverseEngine(d0, tests)
