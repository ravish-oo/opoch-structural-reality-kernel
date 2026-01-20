"""
core/kernel.py - The fundamental kernel objects.

Implements: Ledger, Survivors W(L), Π*, Δ (feasible tests), Budget, Time, Entropy, Energy.
All without floats in receipts - uses integer ratios only.
"""

from collections import Counter
from dataclasses import dataclass, field
from typing import (
    Any, Callable, Dict, FrozenSet, List, Optional,
    Set, Tuple, TypeVar, Generic, Iterator
)
from functools import cached_property
import hashlib
import json

T = TypeVar('T')


@dataclass(frozen=True)
class Record:
    """A single ledger record: (test_id, outcome)."""
    test_id: str
    outcome: Any

    def canonical(self) -> str:
        """Canonical string for hashing."""
        return json.dumps(
            {"test_id": self.test_id, "outcome": self.outcome},
            sort_keys=True, separators=(",", ":")
        )

    def fingerprint(self) -> str:
        """SHA-256 fingerprint."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()[:16]


@dataclass
class Test:
    """
    A total test τ: D0 → A with finite A and integer cost.

    Totality discipline: must return an outcome for every x in D0.
    FAIL/TIMEOUT are explicit outcomes, never exceptions.
    """
    test_id: str
    evaluator: Callable[[Any], Any]  # Must be total
    cost: int  # Integer cost units, c(τ) >= 0
    outcome_space: FrozenSet[Any]  # Finite A

    def __call__(self, x: Any) -> Any:
        """Evaluate test on x. Must be total."""
        try:
            result = self.evaluator(x)
            if result not in self.outcome_space:
                return ("INVALID_OUTCOME", result)
            return result
        except Exception as e:
            return ("FAIL", str(type(e).__name__))

    def canonical_id(self) -> str:
        """Canonical test identifier for receipts."""
        return self.test_id


class Ledger:
    """
    Multiset of records. Order is gauge - stored as Counter.

    Implements the ledger L = {(τ_i, a_i)} as a multiset.
    """

    def __init__(self, records: Optional[List[Record]] = None):
        self._records: Counter = Counter()
        if records:
            for r in records:
                self._records[r] += 1

    def append(self, record: Record) -> 'Ledger':
        """Return new ledger with record appended."""
        new_ledger = Ledger()
        new_ledger._records = self._records.copy()
        new_ledger._records[record] += 1
        return new_ledger

    def __iter__(self) -> Iterator[Tuple[Record, int]]:
        """Iterate over (record, count) pairs."""
        return iter(self._records.items())

    def records_list(self) -> List[Record]:
        """Flat list of all records (with multiplicity)."""
        result = []
        for r, count in self._records.items():
            result.extend([r] * count)
        return result

    def __len__(self) -> int:
        """Total number of records (with multiplicity)."""
        return sum(self._records.values())

    def canonical(self) -> str:
        """
        Canonical representation - sorted by record fingerprint.
        Order-independent (gauge-invariant).
        """
        sorted_records = sorted(
            [(r.canonical(), count) for r, count in self._records.items()],
            key=lambda x: x[0]
        )
        return json.dumps(sorted_records, sort_keys=True, separators=(",", ":"))

    def fingerprint(self) -> str:
        """SHA-256 fingerprint of canonical form."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()


class Survivors:
    """
    W(L) = { x ∈ D0 : ∀(τ,a) ∈ L, τ(x) = a }

    The consistency fiber - elements surviving all recorded tests.
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
        """Compute W(L) by filtering D0."""
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

    def __len__(self) -> int:
        return len(self.survivors)

    def __iter__(self) -> Iterator[Any]:
        return iter(self.survivors)

    def __contains__(self, x: Any) -> bool:
        return x in self.survivors

    def cardinality(self) -> int:
        """Return |W(L)| as integer (for receipts)."""
        return len(self.survivors)


@dataclass
class PartitionClass:
    """A single equivalence class in Π*."""
    fingerprint: str  # Canonical fingerprint
    size: int  # Number of elements
    representative: Any  # One canonical representative
    outcome_signature: Tuple[Tuple[str, Any], ...]  # Sorted test outcomes


class PiStar:
    """
    Π*(L) = D0 / ≡_L

    Truth quotient: partition of D0 by indistinguishability under recorded tests.
    Two elements are equivalent iff all recorded tests give same outcomes on them.

    Represented by class-size multiset + canonical fingerprints (not raw elements).
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
        self._partition: Optional[List[PartitionClass]] = None

    @property
    def partition(self) -> List[PartitionClass]:
        """Compute or return cached partition."""
        if self._partition is None:
            self._compute_partition()
        return self._partition

    def _compute_partition(self) -> None:
        """Compute Π*(L) by grouping elements with same test outcomes."""
        # Build outcome signature for each element
        signatures: Dict[Tuple[Tuple[str, Any], ...], List[Any]] = {}

        # Get all recorded tests
        recorded_tests = set()
        for record, _ in self.ledger:
            recorded_tests.add(record.test_id)

        # Sort test IDs for determinism
        sorted_test_ids = sorted(recorded_tests)

        for x in self.d0:
            sig = []
            for test_id in sorted_test_ids:
                test = self.tests.get(test_id)
                if test:
                    outcome = test(x)
                    sig.append((test_id, outcome))

            sig_tuple = tuple(sig)
            if sig_tuple not in signatures:
                signatures[sig_tuple] = []
            signatures[sig_tuple].append(x)

        # Build partition classes
        self._partition = []
        for sig, elements in signatures.items():
            # Sort elements for deterministic representative selection
            sorted_elements = sorted(elements, key=lambda e: str(e))
            representative = sorted_elements[0]

            # Fingerprint from signature
            fp = hashlib.sha256(
                json.dumps(sig, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest()[:16]

            self._partition.append(PartitionClass(
                fingerprint=fp,
                size=len(elements),
                representative=representative,
                outcome_signature=sig
            ))

        # Sort partition by fingerprint for canonical order
        self._partition.sort(key=lambda c: c.fingerprint)

    def class_sizes(self) -> List[int]:
        """Return sorted list of class sizes (gauge-invariant)."""
        return sorted([c.size for c in self.partition], reverse=True)

    def class_count(self) -> int:
        """Number of equivalence classes."""
        return len(self.partition)

    def canonical_fingerprint(self) -> str:
        """
        Canonical fingerprint of the entire partition.
        Uses class-size multiset (gauge-invariant).
        """
        # Size multiset is gauge-invariant
        size_multiset = sorted([c.size for c in self.partition], reverse=True)
        canonical = json.dumps(
            {"sizes": size_multiset, "count": len(self.partition)},
            sort_keys=True, separators=(",", ":")
        )
        return hashlib.sha256(canonical.encode()).hexdigest()

    def is_singleton(self) -> bool:
        """Check if partition has exactly one class (⊥op condition)."""
        return len(self.partition) == 1


@dataclass
class TimeWitness:
    """
    Time increment witness: stores (|W_pre|, |W_post|) as integers.

    ΔT = log(|W_pre|/|W_post|) but we store only the ratio pair.
    Logs computed only for display, never for receipts.
    """
    w_pre: int  # |W| before record
    w_post: int  # |W| after record

    def ratio(self) -> Tuple[int, int]:
        """Return the ratio pair for receipts."""
        return (self.w_pre, self.w_post)

    def delta_t_display(self) -> float:
        """Human-readable ΔT (not for receipts)."""
        import math
        if self.w_post == 0:
            return float('inf')
        if self.w_pre == 0:
            return 0.0
        return math.log2(self.w_pre / self.w_post)

    def canonical(self) -> str:
        """Canonical form for receipts - integers only."""
        return json.dumps(
            {"w_pre": self.w_pre, "w_post": self.w_post},
            sort_keys=True, separators=(",", ":")
        )


@dataclass
class TotalTime:
    """
    Total time T represented by (|D0|, |W(L)|) since T = log(|D0|/|W|).
    Stores pair only - no floats.
    """
    d0_size: int
    w_size: int

    def ratio(self) -> Tuple[int, int]:
        return (self.d0_size, self.w_size)

    def t_display(self) -> float:
        """Human-readable T (not for receipts)."""
        import math
        if self.w_size == 0:
            return float('inf')
        if self.d0_size == 0:
            return 0.0
        return math.log2(self.d0_size / self.w_size)

    def canonical(self) -> str:
        return json.dumps(
            {"d0_size": self.d0_size, "w_size": self.w_size},
            sort_keys=True, separators=(",", ":")
        )


class Budget:
    """
    Budget(L) := log|W(L)|, but stored as |W(L)| integer.

    Feasibility: test τ is feasible iff c(τ) ≤ BudgetUnits(L).
    BudgetUnits is a deterministic monotone map from |W| to integer units.
    """

    def __init__(self, w_size: int, alpha: int = 1):
        """
        Args:
            w_size: |W(L)| - survivor count
            alpha: scaling factor for budget units
        """
        self.w_size = w_size
        self.alpha = alpha

    def budget_units(self) -> int:
        """
        BudgetUnits = floor(alpha * log2|W|).
        Returns integer budget units.
        """
        import math
        if self.w_size <= 1:
            return 0
        return int(self.alpha * math.log2(self.w_size))

    def is_feasible(self, test: Test) -> bool:
        """Check if test is feasible under current budget."""
        return test.cost <= self.budget_units()

    def entropy_display(self) -> float:
        """Human-readable S = log|W| (not for receipts)."""
        import math
        if self.w_size <= 0:
            return float('-inf')
        return math.log2(self.w_size)

    def canonical(self) -> str:
        """Canonical form - integers only."""
        return json.dumps(
            {"w_size": self.w_size, "alpha": self.alpha,
             "budget_units": self.budget_units()},
            sort_keys=True, separators=(",", ":")
        )


class EnergyLedger:
    """
    Energy accounting: E = Σ c(τ) for all applied tests.
    All integers - no floats.
    """

    def __init__(self):
        self.total: int = 0
        self.events: List[Tuple[str, int]] = []  # (test_id, cost)

    def record_cost(self, test: Test) -> None:
        """Record the energy cost of applying a test."""
        self.total += test.cost
        self.events.append((test.test_id, test.cost))

    def canonical(self) -> str:
        """Canonical form for receipts."""
        return json.dumps(
            {"total": self.total, "events": self.events},
            sort_keys=True, separators=(",", ":")
        )


class FeasibleTests:
    """
    Δ(L) = {τ : c(τ) ≤ Budget(L)}

    The set of feasible tests under current budget.
    """

    def __init__(self, all_tests: Dict[str, Test], budget: Budget):
        self.all_tests = all_tests
        self.budget = budget
        self._feasible: Optional[Dict[str, Test]] = None

    @property
    def feasible(self) -> Dict[str, Test]:
        """Return feasible tests."""
        if self._feasible is None:
            self._feasible = {
                tid: t for tid, t in self.all_tests.items()
                if self.budget.is_feasible(t)
            }
        return self._feasible

    def __iter__(self) -> Iterator[Test]:
        return iter(self.feasible.values())

    def __len__(self) -> int:
        return len(self.feasible)

    def get(self, test_id: str) -> Optional[Test]:
        return self.feasible.get(test_id)


@dataclass
class KernelState:
    """
    Complete kernel state S_t = (Π*(L), Δ, T, H, Λ).

    Only Π-fixed objects allowed. No raw encodings or meta-order.
    """
    ledger: Ledger
    survivors: Survivors
    pi_star: PiStar
    feasible_tests: FeasibleTests
    total_time: TotalTime
    energy: EnergyLedger
    budget: Budget

    # Event poset (dependency time) - simplified as list of events
    event_history: List[Record] = field(default_factory=list)

    # Lemma library (Π-fixed memory)
    lemmas: Dict[str, Any] = field(default_factory=dict)

    def canonical_fingerprint(self) -> str:
        """Π-fixed fingerprint of entire state."""
        state_obj = {
            "ledger_fp": self.ledger.fingerprint(),
            "pi_star_fp": self.pi_star.canonical_fingerprint(),
            "survivors_count": len(self.survivors),
            "time": self.total_time.ratio(),
            "energy_total": self.energy.total,
            "budget_units": self.budget.budget_units()
        }
        canonical = json.dumps(state_obj, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(canonical.encode()).hexdigest()


def compute_kernel_state(
    d0: FrozenSet[Any],
    ledger: Ledger,
    tests: Dict[str, Test],
    alpha: int = 1
) -> KernelState:
    """
    Compute the complete kernel state from D0, ledger, and tests.
    """
    survivors = Survivors(d0, ledger, tests)
    pi_star = PiStar(d0, ledger, tests)
    budget = Budget(len(survivors), alpha)
    feasible = FeasibleTests(tests, budget)
    total_time = TotalTime(len(d0), len(survivors))
    energy = EnergyLedger()

    return KernelState(
        ledger=ledger,
        survivors=survivors,
        pi_star=pi_star,
        feasible_tests=feasible,
        total_time=total_time,
        energy=energy,
        budget=budget
    )
