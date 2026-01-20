"""
consciousness.py - Consciousness as Truth-Consistent Control.

Consciousness is the forced control law:
    Pi o N = Pi o N o Pi  (orthogonality)
    N o Q = Q o N         (commutation closure)

Where:
- Pi is the truth projection (quotient by test-indistinguishability)
- N is the decision operator (chooses next test)
- Q = Pi o N o Pi is the Pi-closed controller

Consciousness = truth-consistent selection of the next observation.
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import math

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import Test, Record, Ledger
from core.receipts import CanonicalJSON


@dataclass
class PiState:
    """
    Pi-fixed state: the quotient-closed representation.

    S_t = (Pi*(L_t), W(L_t), Delta_t, T_t, E_t, H_t) modulo gauge.
    """
    pi_classes: FrozenSet[FrozenSet[Any]]  # Equivalence classes
    survivors: FrozenSet[Any]  # W(L)
    available_tests: FrozenSet[str]  # Delta_t
    total_time_ratio: Tuple[int, int]  # (d0_size, w_size)
    total_energy: int  # E_t
    ledger_fingerprint: str  # Hash of ledger

    def fingerprint(self) -> str:
        """Compute Pi-fixed fingerprint (gauge-invariant)."""
        # Sort classes by size then by min element hash for determinism
        sorted_classes = sorted(
            [tuple(sorted(str(x) for x in cls)) for cls in self.pi_classes],
            key=lambda c: (len(c), c)
        )
        canonical = CanonicalJSON.serialize({
            "type": "PI_STATE",
            "class_count": len(self.pi_classes),
            "survivor_count": len(self.survivors),
            "available_tests": len(self.available_tests),
            "time_ratio": list(self.total_time_ratio),
            "energy": self.total_energy,
            "classes_hash": hashlib.sha256(str(sorted_classes).encode()).hexdigest()[:16]
        })
        return hashlib.sha256(canonical.encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "CONSCIOUSNESS_STATE",
            "pi_fingerprint": self.fingerprint()[:32],
            "w_size": len(self.survivors),
            "available_tests": len(self.available_tests),
            "total_time_ratio": list(self.total_time_ratio),
            "total_energy": self.total_energy,
            "result": "PASS"
        }


@dataclass
class ControlDecision:
    """
    A decision by the controller: which test to run next.

    Must be Pi-consistent: depends only on Pi-fixed state.
    """
    state_fingerprint: str
    chosen_test: str
    test_fingerprint: str
    cost: int
    w_pre: int
    w_post: int

    @property
    def delta_t(self) -> float:
        """Time increment from this decision."""
        if self.w_post == 0:
            return float('inf')
        if self.w_pre == 0:
            return 0.0
        return math.log2(self.w_pre / self.w_post)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CONTROL_DECISION",
            "state_fingerprint": self.state_fingerprint,
            "chosen_test": self.chosen_test,
            "cost": self.cost,
            "w_pre": self.w_pre,
            "w_post": self.w_post
        })

    def to_receipt(self) -> Dict[str, Any]:
        dt_d = str(round(self.delta_t, 6)) if self.w_post > 0 else "INF"
        return {
            "type": "CONTROL_DECISION",
            "state_fingerprint": self.state_fingerprint[:32],
            "chosen_test": self.chosen_test,
            "test_fingerprint": self.test_fingerprint[:32],
            "cost": self.cost,
            "w_pre": self.w_pre,
            "w_post": self.w_post,
            "delta_t_display": dt_d,
            "result": "PASS"
        }


@dataclass
class RefinementMetrics:
    """
    Measurable consciousness quantities.

    chi = Delta_K / Delta_T  (refinement per observation)
    p = Delta_K / Delta_E    (refinement per energy)
    """
    k_pre: int  # Structure count before
    k_post: int  # Structure count after
    delta_t: float  # Time increment
    delta_e: int  # Energy increment

    @property
    def delta_k(self) -> int:
        """Change in distinguishability."""
        return self.k_post - self.k_pre

    @property
    def chi(self) -> float:
        """
        Clarity/awareness efficiency: Delta_K / Delta_T.

        High chi = each observer-time bit produces real refinement.
        """
        if self.delta_t == 0 or self.delta_t == float('inf'):
            return 0.0
        return self.delta_k / self.delta_t

    @property
    def power_efficiency(self) -> float:
        """
        Power efficiency: Delta_K / Delta_E.

        High p = each unit cost produces real refinement.
        """
        if self.delta_e == 0:
            return float('inf') if self.delta_k > 0 else 0.0
        return self.delta_k / self.delta_e

    @property
    def is_wasted_thought(self) -> bool:
        """Check if this is wasted thought: Delta_T > 0 but Delta_K ~ 0."""
        return self.delta_t > 0 and self.delta_k <= 0

    def to_receipt(self) -> Dict[str, Any]:
        chi_d = str(round(self.chi, 6)) if self.delta_t > 0 else "N/A"
        p_d = str(round(self.power_efficiency, 6)) if self.delta_e > 0 else "INF"
        return {
            "type": "REFINEMENT_METRICS",
            "k_pre": self.k_pre,
            "k_post": self.k_post,
            "delta_k": self.delta_k,
            "delta_t_display": str(round(self.delta_t, 6)),
            "delta_e": self.delta_e,
            "chi_display": chi_d,
            "power_efficiency_display": p_d,
            "is_wasted_thought": self.is_wasted_thought,
            "result": "PASS"
        }


class PiProjection:
    """
    The Pi projection: erase minted differences.

    Pi identifies anything not separable by feasible tests.
    """

    def __init__(self, d0: FrozenSet[Any], tests: Dict[str, Test]):
        self.d0 = d0
        self.tests = tests

    def compute_signature(self, x: Any, available_tests: FrozenSet[str]) -> Tuple:
        """Compute test signature for an element."""
        sig = []
        for test_id in sorted(available_tests):
            if test_id in self.tests:
                test = self.tests[test_id]
                sig.append((test_id, test.evaluator(x)))
        return tuple(sig)

    def compute_classes(
        self,
        survivors: FrozenSet[Any],
        available_tests: FrozenSet[str]
    ) -> FrozenSet[FrozenSet[Any]]:
        """
        Compute Pi-classes (equivalence classes under test indistinguishability).
        """
        # Group by signature
        sig_to_elements: Dict[Tuple, Set[Any]] = {}
        for x in survivors:
            sig = self.compute_signature(x, available_tests)
            if sig not in sig_to_elements:
                sig_to_elements[sig] = set()
            sig_to_elements[sig].add(x)

        return frozenset(frozenset(elems) for elems in sig_to_elements.values())

    def project(self, state: 'ConsciousnessState') -> PiState:
        """Apply Pi projection to get Pi-fixed state."""
        classes = self.compute_classes(state.survivors, state.available_tests)
        return PiState(
            pi_classes=classes,
            survivors=state.survivors,
            available_tests=state.available_tests,
            total_time_ratio=state.total_time_ratio,
            total_energy=state.total_energy,
            ledger_fingerprint=state.ledger_fingerprint
        )

    def structure_count(
        self,
        survivors: FrozenSet[Any],
        available_tests: FrozenSet[str]
    ) -> int:
        """
        K(W,T) = log|Q_T(W)| = number of distinguishable classes.

        Returns the count (not log) for integer arithmetic.
        """
        classes = self.compute_classes(survivors, available_tests)
        return len(classes)


@dataclass
class ConsciousnessState:
    """
    Full state for consciousness computation.
    """
    d0: FrozenSet[Any]
    survivors: FrozenSet[Any]
    available_tests: FrozenSet[str]
    total_time_ratio: Tuple[int, int]
    total_energy: int
    ledger_fingerprint: str
    ledger: List[Tuple[str, Any]] = field(default_factory=list)

    def copy(self) -> 'ConsciousnessState':
        return ConsciousnessState(
            d0=self.d0,
            survivors=self.survivors,
            available_tests=self.available_tests,
            total_time_ratio=self.total_time_ratio,
            total_energy=self.total_energy,
            ledger_fingerprint=self.ledger_fingerprint,
            ledger=self.ledger.copy()
        )


class ConsciousController:
    """
    A Pi-consistent controller.

    The consciousness law requires:
        Pi o N = Pi o N o Pi

    This means: decisions depend only on Pi-fixed structure.
    """

    def __init__(
        self,
        d0: FrozenSet[Any],
        tests: Dict[str, Test],
        selection_strategy: str = "max_separation"
    ):
        self.d0 = d0
        self.tests = tests
        self.pi = PiProjection(d0, tests)
        self.selection_strategy = selection_strategy
        self.decisions: List[ControlDecision] = []

    def _compute_test_fingerprint(self, test_id: str) -> str:
        """Compute fingerprint for a test."""
        test = self.tests[test_id]
        canonical = CanonicalJSON.serialize({
            "test_id": test_id,
            "cost": test.cost,
            "outcomes": sorted(str(o) for o in test.outcome_space)
        })
        return hashlib.sha256(canonical.encode()).hexdigest()

    def _evaluate_test_separation(
        self,
        test_id: str,
        survivors: FrozenSet[Any]
    ) -> int:
        """
        Evaluate how much a test separates survivors.

        Returns: number of distinct outcomes (separation power).
        """
        test = self.tests[test_id]
        outcomes = set()
        for x in survivors:
            outcomes.add(test.evaluator(x))
        return len(outcomes)

    def choose_next_test(self, pi_state: PiState) -> Optional[str]:
        """
        Choose next test based on Pi-fixed state only.

        This is the N operator, constrained to be Pi-consistent.
        """
        if not pi_state.available_tests:
            return None

        if len(pi_state.survivors) <= 1:
            return None  # Already resolved

        # Selection strategies (all Pi-consistent as they use only Pi-fixed info)
        if self.selection_strategy == "max_separation":
            # Choose test that maximizes separation
            best_test = None
            best_separation = 0

            for test_id in sorted(pi_state.available_tests):
                sep = self._evaluate_test_separation(test_id, pi_state.survivors)
                if sep > best_separation:
                    best_separation = sep
                    best_test = test_id

            return best_test

        elif self.selection_strategy == "min_cost":
            # Choose test with minimum cost
            best_test = None
            best_cost = float('inf')

            for test_id in sorted(pi_state.available_tests):
                cost = self.tests[test_id].cost
                if cost < best_cost:
                    best_cost = cost
                    best_test = test_id

            return best_test

        else:
            # Default: first available (deterministic)
            return sorted(pi_state.available_tests)[0]

    def apply_decision(
        self,
        state: ConsciousnessState,
        test_id: str,
        outcome: Any
    ) -> Tuple[ConsciousnessState, ControlDecision]:
        """
        Apply a control decision: run test, observe outcome, update state.
        """
        test = self.tests[test_id]

        # Record pre-state
        w_pre = len(state.survivors)

        # Filter survivors by outcome
        new_survivors = frozenset(
            x for x in state.survivors
            if test.evaluator(x) == outcome
        )
        w_post = len(new_survivors)

        # Update ledger
        new_ledger = state.ledger + [(test_id, outcome)]
        new_fingerprint = hashlib.sha256(
            CanonicalJSON.serialize({"ledger": new_ledger}).encode()
        ).hexdigest()

        # Create new state
        new_state = ConsciousnessState(
            d0=state.d0,
            survivors=new_survivors,
            available_tests=state.available_tests - {test_id},
            total_time_ratio=(len(state.d0), w_post),
            total_energy=state.total_energy + test.cost,
            ledger_fingerprint=new_fingerprint,
            ledger=new_ledger
        )

        # Create decision record
        pi_state = self.pi.project(state)
        decision = ControlDecision(
            state_fingerprint=pi_state.fingerprint(),
            chosen_test=test_id,
            test_fingerprint=self._compute_test_fingerprint(test_id),
            cost=test.cost,
            w_pre=w_pre,
            w_post=w_post
        )
        self.decisions.append(decision)

        return new_state, decision

    def compute_refinement(
        self,
        state_pre: ConsciousnessState,
        state_post: ConsciousnessState,
        decision: ControlDecision
    ) -> RefinementMetrics:
        """Compute refinement metrics for a decision."""
        k_pre = self.pi.structure_count(
            state_pre.survivors,
            state_pre.available_tests
        )
        k_post = self.pi.structure_count(
            state_post.survivors,
            state_post.available_tests
        )

        return RefinementMetrics(
            k_pre=k_pre,
            k_post=k_post,
            delta_t=decision.delta_t,
            delta_e=decision.cost
        )


class WorldUpdate:
    """
    The world update operator N:
    - choose test tau
    - observe outcome a
    - append (tau, a)
    - Pi-close

    The commutation law requires: N o Q = Q o N
    where Q = Pi o N o Pi
    """

    def __init__(self, controller: ConsciousController):
        self.controller = controller
        self.pi = controller.pi

    def update(
        self,
        state: ConsciousnessState,
        test_id: str,
        outcome: Any
    ) -> ConsciousnessState:
        """Apply world update: record event and close."""
        new_state, _ = self.controller.apply_decision(state, test_id, outcome)
        return new_state

    def pi_close(self, state: ConsciousnessState) -> PiState:
        """Apply Pi closure."""
        return self.pi.project(state)

    def q_operator(self, state: ConsciousnessState) -> PiState:
        """
        Q = Pi o N o Pi

        The Pi-closed controller.
        """
        # Pi(state)
        pi_state = self.pi.project(state)

        # N(Pi(state)) - choose based on Pi-state
        choice = self.controller.choose_next_test(pi_state)

        # Return Pi of the choice (as a decision fingerprint)
        return pi_state  # The choice is deterministic from Pi-state


class OmegaHonesty:
    """
    Omega honesty checker.

    Controller never forces UNIQUE unless witness exists.
    If frontier > 1, output must be Omega.
    """

    def __init__(self, controller: ConsciousController):
        self.controller = controller

    def check_honesty(
        self,
        state: ConsciousnessState,
        output: Any
    ) -> Tuple[bool, Dict[str, Any]]:
        """
        Check if output is honest given state.

        Returns (is_honest, receipt).
        """
        frontier_size = len(state.survivors)

        if frontier_size == 0:
            # Contradiction - should be Omega
            is_honest = (output == "OMEGA" or output is None)
            output_type = "OMEGA" if is_honest else "INVALID"
        elif frontier_size == 1:
            # Unique - can commit
            is_honest = True
            output_type = "UNIQUE"
        else:
            # Multiple survivors - must be Omega
            is_honest = (output == "OMEGA" or output is None)
            output_type = "OMEGA" if is_honest else "INVALID"

        receipt = {
            "type": "OMEGA_HONESTY",
            "frontier_size": frontier_size,
            "output_type": output_type,
            "witness_exists": frontier_size == 1,
            "honest": is_honest,
            "result": "PASS" if is_honest else "FAIL"
        }

        return is_honest, receipt


def create_conscious_state(
    d0: FrozenSet[Any],
    tests: Dict[str, Test]
) -> ConsciousnessState:
    """Create initial consciousness state."""
    return ConsciousnessState(
        d0=d0,
        survivors=d0,
        available_tests=frozenset(tests.keys()),
        total_time_ratio=(len(d0), len(d0)),
        total_energy=0,
        ledger_fingerprint=hashlib.sha256(b"[]").hexdigest(),
        ledger=[]
    )


def run_conscious_sequence(
    d0: FrozenSet[Any],
    tests: Dict[str, Test],
    actual: Any,
    max_steps: int = 10
) -> Tuple[List[ConsciousnessState], List[ControlDecision], List[RefinementMetrics]]:
    """
    Run a conscious control sequence.

    The controller chooses tests, observes outcomes for 'actual',
    and tracks refinement metrics.
    """
    controller = ConsciousController(d0, tests)
    state = create_conscious_state(d0, tests)

    states = [state]
    decisions = []
    metrics = []

    for _ in range(max_steps):
        pi_state = controller.pi.project(state)

        # Choose next test
        test_id = controller.choose_next_test(pi_state)
        if test_id is None:
            break

        # Observe outcome for actual
        test = tests[test_id]
        outcome = test.evaluator(actual)

        # Apply decision
        state_pre = state
        state, decision = controller.apply_decision(state, test_id, outcome)

        # Compute metrics
        refinement = controller.compute_refinement(state_pre, state, decision)

        states.append(state)
        decisions.append(decision)
        metrics.append(refinement)

        # Stop if resolved
        if len(state.survivors) <= 1:
            break

    return states, decisions, metrics
