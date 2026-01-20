"""
engine_state.py - State tracking for the Universe Engine.

Tracks all state variables:
- Ledger L (records)
- Survivors W(L)
- Truth quotient Pi*(L)
- Time T and entropy S
- Energy E
- Feasibility Delta(L)
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


@dataclass
class LedgerRecord:
    """
    A single record (tau, a) in the ledger.
    """
    test_id: str
    outcome: Any
    cost: int
    timestamp: int  # Step index when recorded

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "test_id": self.test_id,
            "outcome": str(self.outcome),
            "cost": self.cost,
            "timestamp": self.timestamp
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class Ledger:
    """
    The ledger L = {(tau_i, a_i)}.

    A multiset of records tracking all observations.
    """
    records: List[LedgerRecord] = field(default_factory=list)

    def append(self, record: LedgerRecord) -> 'Ledger':
        """Append a record and return new ledger."""
        new_records = self.records.copy()
        new_records.append(record)
        return Ledger(records=new_records)

    @property
    def size(self) -> int:
        return len(self.records)

    @property
    def total_cost(self) -> int:
        return sum(r.cost for r in self.records)

    def canonical(self) -> str:
        """Canonical representation (order-independent for Pi*)."""
        sorted_records = sorted(
            [r.canonical() for r in self.records]
        )
        return CanonicalJSON.serialize({
            "type": "LEDGER",
            "record_count": self.size,
            "records_hash": hashlib.sha256(
                "".join(sorted_records).encode()
            ).hexdigest()[:32]
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "LEDGER",
            "record_count": self.size,
            "total_cost": self.total_cost,
            "fingerprint": self.fingerprint()[:32],
            "result": "PASS"
        }


@dataclass
class EngineState:
    """
    Complete state of the universe engine at a given step.

    Contains:
    - ledger L
    - survivors W(L)
    - total time T
    - total energy E
    - feasible tests Delta(L)
    """
    step_index: int
    ledger: Ledger
    survivors: FrozenSet[Any]
    total_time: float
    total_energy: int
    feasible_tests: Dict[str, Test]
    all_tests: Dict[str, Test]

    @property
    def survivor_count(self) -> int:
        return len(self.survivors)

    @property
    def entropy(self) -> float:
        """S(L) = log|W(L)|."""
        if self.survivor_count == 0:
            return 0.0
        return math.log2(self.survivor_count)

    @property
    def budget(self) -> float:
        """Budget(L) = log|W(L)|."""
        return self.entropy

    @property
    def feasible_test_count(self) -> int:
        return len(self.feasible_tests)

    def compute_pi_fingerprint(self) -> str:
        """
        Compute Pi*(L) fingerprint.

        Pi* is the quotient D0/equiv_L where x equiv y iff
        all recorded tests agree on x and y.
        """
        # Group survivors by their signature under recorded tests
        test_ids = sorted(set(r.test_id for r in self.ledger.records))

        signatures: Dict[Tuple, Set[Any]] = {}
        for x in self.survivors:
            sig = []
            for test_id in test_ids:
                if test_id in self.all_tests:
                    test = self.all_tests[test_id]
                    sig.append((test_id, test.evaluator(x)))
            sig_tuple = tuple(sig)

            if sig_tuple not in signatures:
                signatures[sig_tuple] = set()
            signatures[sig_tuple].add(x)

        # Create canonical fingerprint from equivalence classes
        class_sizes = sorted(len(cls) for cls in signatures.values())
        canonical = CanonicalJSON.serialize({
            "type": "PI_STAR",
            "class_count": len(signatures),
            "class_sizes": class_sizes,
            "survivor_count": self.survivor_count
        })
        return hashlib.sha256(canonical.encode()).hexdigest()

    def is_operational_nothingness(self) -> bool:
        """
        K = 0 <=> bottom_op.

        Check if no feasible tests can further distinguish survivors.
        """
        if self.survivor_count <= 1:
            return True

        # Check if any feasible test can separate survivors
        for test_id, test in self.feasible_tests.items():
            outcomes = set()
            for x in self.survivors:
                outcomes.add(test.evaluator(x))
            if len(outcomes) > 1:
                return False  # This test can still separate

        return True  # No feasible test can separate

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ENGINE_STATE",
            "step_index": self.step_index,
            "survivor_count": self.survivor_count,
            "total_time_display": str(round(self.total_time, 6)),
            "total_energy": self.total_energy,
            "feasible_tests": self.feasible_test_count,
            "pi_fingerprint": self.compute_pi_fingerprint()[:16]
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "ENGINE_STATE",
            "step_index": self.step_index,
            "survivor_count": self.survivor_count,
            "entropy_display": str(round(self.entropy, 6)),
            "total_time_display": str(round(self.total_time, 6)),
            "total_energy": self.total_energy,
            "feasible_tests": self.feasible_test_count,
            "pi_fingerprint": self.compute_pi_fingerprint()[:32],
            "is_operational_nothingness": self.is_operational_nothingness(),
            "result": "PASS"
        }


@dataclass
class EngineStep:
    """
    A single step of the universe engine.

    Records all changes from state_t to state_{t+1}.
    """
    step_index: int
    separator_id: str
    outcome: Any
    cost: int
    survivors_before: int
    survivors_after: int
    delta_time: float
    time_before: float
    time_after: float
    energy_before: int
    energy_after: int
    pi_fingerprint_before: str
    pi_fingerprint_after: str

    @property
    def is_shrinking(self) -> bool:
        """Did survivors shrink?"""
        return self.survivors_after <= self.survivors_before

    @property
    def is_time_positive(self) -> bool:
        """Is delta_time >= 0?"""
        return self.delta_time >= -1e-10  # Allow small floating point error

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ENGINE_STEP",
            "step_index": self.step_index,
            "separator_id": self.separator_id,
            "outcome": str(self.outcome),
            "survivors_before": self.survivors_before,
            "survivors_after": self.survivors_after
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "ENGINE_STEP",
            "step_index": self.step_index,
            "separator_id": self.separator_id,
            "outcome": str(self.outcome),
            "cost": self.cost,
            "survivors_before": self.survivors_before,
            "survivors_after": self.survivors_after,
            "delta_time_display": str(round(self.delta_time, 6)),
            "total_time_display": str(round(self.time_after, 6)),
            "total_energy": self.energy_after,
            "pi_fingerprint": self.pi_fingerprint_after[:32],
            "is_shrinking": self.is_shrinking,
            "is_time_positive": self.is_time_positive,
            "result": "PASS" if (self.is_shrinking and self.is_time_positive) else "FAIL"
        }


@dataclass
class EngineTermination:
    """
    Termination state of the engine.
    """
    termination_type: str  # "UNIQUE" | "BOTTOM_OP" | "BUDGET_EXHAUSTED" | "MAX_STEPS"
    total_steps: int
    final_answer: Optional[Any]
    final_survivors: int
    final_time: float
    final_energy: int
    final_pi_fingerprint: str

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ENGINE_TERMINATION",
            "termination_type": self.termination_type,
            "total_steps": self.total_steps,
            "final_survivors": self.final_survivors
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "ENGINE_TERMINATION",
            "termination_type": self.termination_type,
            "total_steps": self.total_steps,
            "final_answer": str(self.final_answer) if self.final_answer else "NONE",
            "final_survivors": self.final_survivors,
            "total_time_display": str(round(self.final_time, 6)),
            "total_energy": self.final_energy,
            "pi_fingerprint": self.final_pi_fingerprint[:32],
            "result": "PASS"
        }


def compute_feasible_tests(
    all_tests: Dict[str, Test],
    budget: float
) -> Dict[str, Test]:
    """
    Compute Delta(L) = {tau : c(tau) <= Budget(L)}.
    """
    return {
        test_id: test
        for test_id, test in all_tests.items()
        if test.cost <= budget
    }


def compute_survivors(
    d0: FrozenSet[Any],
    ledger: Ledger,
    all_tests: Dict[str, Test]
) -> FrozenSet[Any]:
    """
    Compute W(L) = {x in D0 : for all (tau,a) in L, tau(x) = a}.
    """
    survivors = set(d0)

    for record in ledger.records:
        if record.test_id not in all_tests:
            continue

        test = all_tests[record.test_id]
        survivors = {
            x for x in survivors
            if test.evaluator(x) == record.outcome
        }

    return frozenset(survivors)


def create_initial_state(
    d0: FrozenSet[Any],
    all_tests: Dict[str, Test]
) -> EngineState:
    """
    Create initial engine state.
    """
    ledger = Ledger()
    survivors = d0
    entropy = math.log2(len(survivors)) if survivors else 0.0
    feasible = compute_feasible_tests(all_tests, entropy)

    return EngineState(
        step_index=0,
        ledger=ledger,
        survivors=survivors,
        total_time=0.0,
        total_energy=0,
        feasible_tests=feasible,
        all_tests=all_tests
    )
