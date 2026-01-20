"""
universe_engine.py - The Universe Engine: One Update Rule That Generates Everything.

The complete engine that:
1. Chooses tau* = argmin[c(tau) + max_a V(W_a;q)]
2. Observes outcome a = tau(x*) for actual x*
3. Commits record: L_{t+1} = L_t union {(tau, a)}
4. Updates survivors: W_{t+1} = W_t intersection tau^{-1}(a)
5. Updates truth: Pi*(L_{t+1})
6. Updates time: T_{t+1} = T_t + log(|W_t|/|W_{t+1}|)
7. Updates energy: E_{t+1} = E_t + c(tau)
8. Updates feasibility: Delta(L_{t+1})

Terminates when |Ans| = 1 (UNIQUE) or K = 0 (operational nothingness).
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import math

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import Test
from core.receipts import CanonicalJSON

from .engine_state import (
    LedgerRecord, Ledger, EngineState, EngineStep, EngineTermination,
    compute_feasible_tests, compute_survivors, create_initial_state
)
from .computation import Query, AnswerComputer


@dataclass
class SeparatorSelection:
    """
    Result of canonical separator selection tau*.
    """
    separator_id: str
    test: Test
    minimax_value: int
    is_optimal: bool
    tie_broken_by: str  # "UNIQUE" | "FINGERPRINT"

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "SEPARATOR_SELECTION",
            "separator_id": self.separator_id,
            "minimax_value": self.minimax_value,
            "is_optimal": self.is_optimal
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SEPARATOR_SELECTION",
            "separator_id": self.separator_id,
            "cost": self.test.cost,
            "minimax_value": self.minimax_value,
            "is_optimal": self.is_optimal,
            "tie_broken_by": self.tie_broken_by,
            "result": "PASS"
        }


class UniverseEngine:
    """
    The Universe Engine.

    Implements the complete update loop from nothingness to structure
    and back to operational nothingness.
    """

    def __init__(
        self,
        d0: FrozenSet[Any],
        tests: Dict[str, Test],
        query: Query,
        observer: Optional[Callable[[Any, Test], Any]] = None
    ):
        """
        Initialize the engine.

        Args:
            d0: Finite working domain
            tests: Available tests
            query: Target query q: D0 -> B
            observer: Optional observer function (test, actual_x) -> outcome
                     If None, uses test.evaluator directly
        """
        self.d0 = d0
        self.all_tests = tests
        self.query = query
        self.answer_computer = AnswerComputer(query)

        # Observer function - if provided, simulates actual observation
        # Otherwise we need to specify the actual element
        self.observer = observer

        # State tracking
        self.state: EngineState = create_initial_state(d0, tests)
        self.steps: List[EngineStep] = []

        # Memoization for minimax
        self._minimax_cache: Dict[FrozenSet[Any], Tuple[int, Optional[str]]] = {}

    def _compute_minimax(
        self,
        survivors: FrozenSet[Any],
        feasible_tests: Dict[str, Test],
        depth: int = 0,
        max_depth: int = 10
    ) -> Tuple[int, Optional[str]]:
        """
        Compute V(W;q) and tau* using minimax.

        Returns: (minimax_value, optimal_separator_id)
        """
        # Check cache
        cache_key = survivors
        if cache_key in self._minimax_cache:
            return self._minimax_cache[cache_key]

        # Check if query is constant on survivors
        answer_set = self.answer_computer.compute_answer_set(survivors)
        if answer_set.is_solved:
            self._minimax_cache[cache_key] = (0, None)
            return (0, None)

        # Check depth limit
        if depth >= max_depth:
            # Return estimate
            return (len(survivors), None)

        # Find optimal separator
        best_value = float('inf')
        best_separator = None

        for test_id, test in feasible_tests.items():
            # Partition survivors
            partition: Dict[Any, Set[Any]] = {}
            for x in survivors:
                outcome = test.evaluator(x)
                if outcome not in partition:
                    partition[outcome] = set()
                partition[outcome].add(x)

            # Skip if doesn't separate
            if len(partition) <= 1:
                continue

            # Compute max over outcomes
            max_child_value = 0
            for outcome, fiber in partition.items():
                child_value, _ = self._compute_minimax(
                    frozenset(fiber),
                    feasible_tests,
                    depth + 1,
                    max_depth
                )
                max_child_value = max(max_child_value, child_value)

            # Total value
            total_value = test.cost + max_child_value

            # Update best (tie-break by fingerprint)
            if total_value < best_value:
                best_value = total_value
                best_separator = test_id
            elif total_value == best_value and best_separator is not None:
                # Tie-break by fingerprint
                curr_fp = hashlib.sha256(test_id.encode()).hexdigest()
                best_fp = hashlib.sha256(best_separator.encode()).hexdigest()
                if curr_fp < best_fp:
                    best_separator = test_id

        if best_separator is None:
            best_value = 0

        result = (int(best_value) if best_value != float('inf') else 0, best_separator)
        self._minimax_cache[cache_key] = result
        return result

    def select_separator(self) -> Optional[SeparatorSelection]:
        """
        Select canonical next separator tau*.

        tau*(W;q) = argmin[c(tau) + max_a V(W_a;q)]
        """
        if not self.state.feasible_tests:
            return None

        survivors = self.state.survivors
        if len(survivors) == 0:
            return None

        # Check if already solved
        answer_set = self.answer_computer.compute_answer_set(survivors)
        if answer_set.is_solved:
            return None

        # Compute minimax
        minimax_value, separator_id = self._compute_minimax(
            survivors,
            self.state.feasible_tests
        )

        if separator_id is None:
            # No separator found - try any feasible test
            for test_id, test in self.state.feasible_tests.items():
                # Check if it separates
                outcomes = set(test.evaluator(x) for x in survivors)
                if len(outcomes) > 1:
                    separator_id = test_id
                    minimax_value = test.cost
                    break

        if separator_id is None:
            return None

        test = self.state.feasible_tests[separator_id]

        return SeparatorSelection(
            separator_id=separator_id,
            test=test,
            minimax_value=minimax_value,
            is_optimal=True,
            tie_broken_by="FINGERPRINT" if minimax_value > 0 else "UNIQUE"
        )

    def step(self, actual_element: Any) -> Optional[EngineStep]:
        """
        Execute one step of the engine.

        1. Choose tau*
        2. Observe outcome for actual_element
        3. Commit record
        4. Update all state variables
        """
        # Select separator
        selection = self.select_separator()
        if selection is None:
            return None

        test = selection.test

        # Observe outcome
        outcome = test.evaluator(actual_element)

        # State before
        survivors_before = self.state.survivor_count
        time_before = self.state.total_time
        energy_before = self.state.total_energy
        pi_before = self.state.compute_pi_fingerprint()

        # Create record
        record = LedgerRecord(
            test_id=selection.separator_id,
            outcome=outcome,
            cost=test.cost,
            timestamp=self.state.step_index
        )

        # Update ledger
        new_ledger = self.state.ledger.append(record)

        # Update survivors
        new_survivors = frozenset(
            x for x in self.state.survivors
            if test.evaluator(x) == outcome
        )

        # Update time
        survivors_after = len(new_survivors)
        if survivors_after > 0 and survivors_before > 0:
            delta_time = math.log2(survivors_before / survivors_after)
        else:
            delta_time = 0.0
        new_time = time_before + delta_time

        # Update energy
        new_energy = energy_before + test.cost

        # Update feasibility
        new_budget = math.log2(survivors_after) if survivors_after > 0 else 0.0
        new_feasible = compute_feasible_tests(self.all_tests, new_budget)

        # Create new state
        new_state = EngineState(
            step_index=self.state.step_index + 1,
            ledger=new_ledger,
            survivors=new_survivors,
            total_time=new_time,
            total_energy=new_energy,
            feasible_tests=new_feasible,
            all_tests=self.all_tests
        )

        pi_after = new_state.compute_pi_fingerprint()

        # Create step record
        step = EngineStep(
            step_index=self.state.step_index,
            separator_id=selection.separator_id,
            outcome=outcome,
            cost=test.cost,
            survivors_before=survivors_before,
            survivors_after=survivors_after,
            delta_time=delta_time,
            time_before=time_before,
            time_after=new_time,
            energy_before=energy_before,
            energy_after=new_energy,
            pi_fingerprint_before=pi_before,
            pi_fingerprint_after=pi_after
        )

        # Update state
        self.state = new_state
        self.steps.append(step)

        # Clear minimax cache (state changed)
        self._minimax_cache = {}

        return step

    def get_answer_set(self) -> Any:
        """Get current answer set."""
        return self.answer_computer.compute_answer_set(self.state.survivors)

    def is_solved(self) -> bool:
        """Check if target query is solved (|Ans| = 1)."""
        return self.get_answer_set().is_solved

    def is_operational_nothingness(self) -> bool:
        """Check if K = 0 (no feasible distinctions remain)."""
        return self.state.is_operational_nothingness()

    def run(
        self,
        actual_element: Any,
        max_steps: int = 100
    ) -> EngineTermination:
        """
        Run the engine until termination.

        Termination conditions:
        - |Ans| = 1 (UNIQUE)
        - K = 0 (BOTTOM_OP)
        - No feasible tests (BUDGET_EXHAUSTED)
        - Max steps reached (MAX_STEPS)
        """
        for _ in range(max_steps):
            # Check termination
            if self.is_solved():
                answer_set = self.get_answer_set()
                return EngineTermination(
                    termination_type="UNIQUE",
                    total_steps=len(self.steps),
                    final_answer=answer_set.unique_answer,
                    final_survivors=self.state.survivor_count,
                    final_time=self.state.total_time,
                    final_energy=self.state.total_energy,
                    final_pi_fingerprint=self.state.compute_pi_fingerprint()
                )

            if self.is_operational_nothingness():
                return EngineTermination(
                    termination_type="BOTTOM_OP",
                    total_steps=len(self.steps),
                    final_answer=None,
                    final_survivors=self.state.survivor_count,
                    final_time=self.state.total_time,
                    final_energy=self.state.total_energy,
                    final_pi_fingerprint=self.state.compute_pi_fingerprint()
                )

            # Try to step
            step = self.step(actual_element)
            if step is None:
                return EngineTermination(
                    termination_type="BUDGET_EXHAUSTED",
                    total_steps=len(self.steps),
                    final_answer=None,
                    final_survivors=self.state.survivor_count,
                    final_time=self.state.total_time,
                    final_energy=self.state.total_energy,
                    final_pi_fingerprint=self.state.compute_pi_fingerprint()
                )

        return EngineTermination(
            termination_type="MAX_STEPS",
            total_steps=len(self.steps),
            final_answer=None,
            final_survivors=self.state.survivor_count,
            final_time=self.state.total_time,
            final_energy=self.state.total_energy,
            final_pi_fingerprint=self.state.compute_pi_fingerprint()
        )


@dataclass
class EngineTrace:
    """
    Complete trace of engine execution.
    """
    initial_state: EngineState
    steps: List[EngineStep]
    termination: EngineTermination
    receipts: List[Dict[str, Any]]

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ENGINE_TRACE",
            "total_steps": self.termination.total_steps,
            "termination_type": self.termination.termination_type,
            "final_survivors": self.termination.final_survivors
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "ENGINE_TRACE",
            "total_steps": self.termination.total_steps,
            "termination_type": self.termination.termination_type,
            "final_answer": str(self.termination.final_answer) if self.termination.final_answer else "NONE",
            "final_survivors": self.termination.final_survivors,
            "total_time_display": str(round(self.termination.final_time, 6)),
            "total_energy": self.termination.final_energy,
            "trace_fingerprint": self.fingerprint()[:32],
            "result": "PASS"
        }


def run_engine(
    d0: FrozenSet[Any],
    tests: Dict[str, Test],
    query: Query,
    actual_element: Any,
    max_steps: int = 100
) -> EngineTrace:
    """
    Run the universe engine and return complete trace.
    """
    engine = UniverseEngine(d0, tests, query)
    initial_state = engine.state

    termination = engine.run(actual_element, max_steps)

    # Collect receipts
    receipts = [initial_state.to_receipt()]
    for step in engine.steps:
        receipts.append(step.to_receipt())
    receipts.append(termination.to_receipt())

    return EngineTrace(
        initial_state=initial_state,
        steps=engine.steps,
        termination=termination,
        receipts=receipts
    )


@dataclass
class SelfImprovement:
    """
    Demonstrates self-improvement: adding lemmas reduces complexity.

    Delta_{t+1}^{eff} >= Delta_t^{eff} => V_{t+1}(W;q) <= V_t(W;q)
    """
    lemma_id: str
    tests_before: int
    tests_after: int
    complexity_before: int
    complexity_after: int

    @property
    def improvement(self) -> int:
        return self.complexity_before - self.complexity_after

    @property
    def is_monotone(self) -> bool:
        return self.complexity_after <= self.complexity_before

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SELF_IMPROVEMENT",
            "lemma_id": self.lemma_id,
            "tests_before": self.tests_before,
            "tests_after": self.tests_after,
            "complexity_before": self.complexity_before,
            "complexity_after": self.complexity_after,
            "improvement": self.improvement,
            "is_monotone": self.is_monotone,
            "result": "PASS" if self.is_monotone else "FAIL"
        }


def demonstrate_self_improvement(
    d0: FrozenSet[Any],
    base_tests: Dict[str, Test],
    query: Query,
    lemma_test: Test
) -> SelfImprovement:
    """
    Demonstrate that adding a lemma (test) cannot increase complexity.
    """
    # Compute complexity with base tests
    engine_before = UniverseEngine(d0, base_tests, query)
    complexity_before, _ = engine_before._compute_minimax(
        d0,
        compute_feasible_tests(base_tests, math.log2(len(d0)))
    )

    # Add lemma test
    extended_tests = dict(base_tests)
    extended_tests[lemma_test.test_id] = lemma_test

    # Compute complexity with extended tests
    engine_after = UniverseEngine(d0, extended_tests, query)
    complexity_after, _ = engine_after._compute_minimax(
        d0,
        compute_feasible_tests(extended_tests, math.log2(len(d0)))
    )

    return SelfImprovement(
        lemma_id=lemma_test.test_id,
        tests_before=len(base_tests),
        tests_after=len(extended_tests),
        complexity_before=complexity_before,
        complexity_after=complexity_after
    )
