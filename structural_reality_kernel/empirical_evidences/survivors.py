"""
time_entropy_energy/survivors.py - Survivor set W(L) computation.

W(L) = {x ∈ D₀ : ∀(τ,a) ∈ L, τ(x) = a}

The consistency fiber - elements surviving all recorded tests.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import json

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.kernel import Test, Record, Ledger
from core.receipts import CanonicalJSON


@dataclass
class SurvivorSnapshot:
    """Snapshot of survivors at a point in time."""
    survivors: FrozenSet[Any]
    ledger_size: int
    cardinality: int

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "cardinality": self.cardinality,
            "ledger_size": self.ledger_size
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()[:16]


class Survivors:
    """
    W(L) = {x ∈ D₀ : ∀(τ,a) ∈ L, τ(x) = a}

    The set of possibilities consistent with all recorded tests.
    """

    def __init__(
        self,
        d0: FrozenSet[Any],
        ledger: Ledger,
        tests: Dict[str, Test]
    ):
        self.d0 = d0
        self.ledger = ledger
        self.tests = tests
        self._survivors: Optional[FrozenSet[Any]] = None

    @property
    def survivors(self) -> FrozenSet[Any]:
        """Compute or return cached survivors."""
        if self._survivors is None:
            self._compute_survivors()
        return self._survivors

    def _compute_survivors(self) -> None:
        """Compute W(L) by filtering D₀."""
        current = set(self.d0)

        for record, count in self.ledger:
            if not current:
                break
            test = self.tests.get(record.test_id)
            if test is None:
                continue
            # Filter to elements matching the recorded outcome
            current = {
                x for x in current
                if test(x) == record.outcome
            }

        self._survivors = frozenset(current)

    def cardinality(self) -> int:
        """Return |W(L)| as integer."""
        return len(self.survivors)

    def snapshot(self) -> SurvivorSnapshot:
        """Create a snapshot of current state."""
        return SurvivorSnapshot(
            survivors=self.survivors,
            ledger_size=len(self.ledger),
            cardinality=self.cardinality()
        )

    def is_subset_of(self, other: 'Survivors') -> bool:
        """Check if this survivor set is subset of another."""
        return self.survivors.issubset(other.survivors)


@dataclass
class SurvivorTransition:
    """A transition from W to W' after a record event."""
    w_pre: int  # |W| before
    w_post: int  # |W'| after
    record: Record
    survivors_pre: FrozenSet[Any]
    survivors_post: FrozenSet[Any]

    @property
    def is_monotone(self) -> bool:
        """Check W' ⊆ W (arrow of time)."""
        return self.survivors_post.issubset(self.survivors_pre)

    @property
    def shrink_factor(self) -> Tuple[int, int]:
        """Return (|W|, |W'|) as integer ratio."""
        return (self.w_pre, self.w_post)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "w_pre": self.w_pre,
            "w_post": self.w_post,
            "test_id": self.record.test_id,
            "outcome": str(self.record.outcome),
            "is_monotone": self.is_monotone
        })


class SurvivorHistory:
    """
    Tracks the history of survivor sets through record events.

    This is the foundation for time, entropy, and the arrow.
    """

    def __init__(self, d0: FrozenSet[Any], tests: Dict[str, Test]):
        self.d0 = d0
        self.tests = tests
        self.ledger = Ledger()
        self.transitions: List[SurvivorTransition] = []
        self._current_survivors = frozenset(d0)

    def record_event(self, test_id: str, outcome: Any) -> SurvivorTransition:
        """
        Record a test outcome and track the survivor transition.

        Returns the transition W → W'.
        """
        # Pre-state
        w_pre = len(self._current_survivors)
        survivors_pre = self._current_survivors

        # Apply record
        record = Record(test_id=test_id, outcome=outcome)
        self.ledger = self.ledger.append(record)

        # Post-state
        test = self.tests[test_id]
        new_survivors = frozenset(
            x for x in self._current_survivors
            if test(x) == outcome
        )
        w_post = len(new_survivors)

        # Create transition
        transition = SurvivorTransition(
            w_pre=w_pre,
            w_post=w_post,
            record=record,
            survivors_pre=survivors_pre,
            survivors_post=new_survivors
        )

        self.transitions.append(transition)
        self._current_survivors = new_survivors

        return transition

    @property
    def current_survivors(self) -> FrozenSet[Any]:
        """Current survivor set."""
        return self._current_survivors

    @property
    def current_cardinality(self) -> int:
        """Current |W(L)|."""
        return len(self._current_survivors)

    def verify_all_monotone(self) -> Tuple[bool, List[int]]:
        """
        Verify all transitions satisfy W' ⊆ W.

        Returns (all_monotone, list_of_violation_indices)
        """
        violations = []
        for i, trans in enumerate(self.transitions):
            if not trans.is_monotone:
                violations.append(i)
        return (len(violations) == 0, violations)


def compute_survivors(
    d0: FrozenSet[Any],
    ledger: Ledger,
    tests: Dict[str, Test]
) -> Survivors:
    """Compute survivor set from domain, ledger, and tests."""
    return Survivors(d0, ledger, tests)
