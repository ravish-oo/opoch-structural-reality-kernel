"""
intervention_space.py - Intervention space and multi-level test algebra.

Implements:
- Intervention (candidate) as finite description
- InterventionSpace as finite enumerable domain D_0
- TestLevel enum (T1-T6: in-silico to post-market)
- Test definition with cost and outcome space
- TestAlgebra (multi-level hierarchy)
- LedgerEntry and DiscoveryLedger
- Survivors computation W(L)
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
from fractions import Fraction
from enum import Enum
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON

from .boundary_flow import RationalValue


class InterventionType(Enum):
    """Types of interventions in drug discovery."""
    MOLECULE = "molecule"
    SEQUENCE = "sequence"
    PROTOCOL = "protocol"


@dataclass(frozen=True)
class Intervention:
    """
    A candidate intervention in the discovery space.

    This is a finite description of a molecule, sequence, or protocol.
    """
    intervention_id: str
    intervention_type: InterventionType
    description: str  # Canonical description (SMILES, FASTA, protocol spec)
    properties: FrozenSet[Tuple[str, Any]] = field(default_factory=frozenset)

    def fingerprint(self) -> str:
        """Compute unique fingerprint of this intervention."""
        data = {
            "id": self.intervention_id,
            "type": self.intervention_type.value,
            "description": self.description,
            "properties": sorted([(k, str(v)) for k, v in self.properties])
        }
        return hashlib.sha256(
            CanonicalJSON.serialize(data).encode()
        ).hexdigest()

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "INTERVENTION",
            "intervention_id": self.intervention_id,
            "intervention_type": self.intervention_type.value,
            "description_hash": hashlib.sha256(self.description.encode()).hexdigest()[:16]
        })


@dataclass
class InterventionSpace:
    """
    The finite domain D_0 of candidate interventions.

    This is the actual state of any real discovery program:
    a finite, enumerable set of candidates.
    """
    space_id: str
    candidates: FrozenSet[Intervention]
    intervention_type: InterventionType

    def __post_init__(self):
        if not self.candidates:
            raise ValueError("InterventionSpace must have at least one candidate")

    def size(self) -> int:
        """Return |D_0|."""
        return len(self.candidates)

    def contains(self, candidate: Intervention) -> bool:
        """Check if candidate is in D_0."""
        return candidate in self.candidates

    def get_candidate_by_id(self, intervention_id: str) -> Optional[Intervention]:
        """Find candidate by ID."""
        for c in self.candidates:
            if c.intervention_id == intervention_id:
                return c
        return None

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "INTERVENTION_SPACE",
            "space_id": self.space_id,
            "candidate_count": self.size(),
            "intervention_type": self.intervention_type.value
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "INTERVENTION_SPACE",
            "space_id": self.space_id,
            "candidate_count": self.size(),
            "description_type": self.intervention_type.value,
            "finite": True,
            "enumerable": True,
            "result": "PASS"
        }


class TestLevel(Enum):
    """
    Multi-level test hierarchy (T1-T6).

    Each level has characteristic cost and throughput.
    """
    T1_IN_SILICO = ("T1", 1, 1000000)      # cost ~O(1), throughput ~10^6
    T2_IN_VITRO = ("T2", 100, 10000)       # cost ~O(10^2), throughput ~10^4
    T3_CELL_BASED = ("T3", 1000, 1000)     # cost ~O(10^3), throughput ~10^3
    T4_ANIMAL = ("T4", 100000, 100)        # cost ~O(10^5), throughput ~10^2
    T5_CLINICAL = ("T5", 100000000, 10)    # cost ~O(10^8), throughput ~10
    T6_POST_MARKET = ("T6", 1000000000, 1) # cost ~O(10^9), throughput ~1

    @property
    def level_name(self) -> str:
        return self.value[0]

    @property
    def base_cost(self) -> int:
        return self.value[1]

    @property
    def max_throughput(self) -> int:
        return self.value[2]


@dataclass(frozen=True)
class TestOutcome:
    """An outcome from a test execution."""
    outcome_id: str
    value: Any  # The actual outcome value

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "outcome_id": self.outcome_id,
            "value": str(self.value)
        })


@dataclass
class TestDefinition:
    """
    Definition of a test in the multi-level algebra.

    A test is a total function tau: D_0 -> A where A is finite.
    """
    test_id: str
    level: TestLevel
    outcome_space: FrozenSet[TestOutcome]  # Finite outcome set A
    cost: int  # Integer cost
    description: str = ""

    # The test function itself (optional - can be None for specification)
    # In practice this would be the actual assay/trial
    _test_function: Optional[Callable[[Intervention], TestOutcome]] = field(
        default=None, repr=False, compare=False
    )

    def outcome_space_size(self) -> int:
        """Return |A|."""
        return len(self.outcome_space)

    def is_total(self, domain: InterventionSpace) -> bool:
        """Check if test is defined for all candidates in domain."""
        if self._test_function is None:
            return True  # Assume total if no function specified
        try:
            for candidate in domain.candidates:
                result = self._test_function(candidate)
                if result not in self.outcome_space:
                    return False
            return True
        except Exception:
            return False

    def execute(self, candidate: Intervention) -> TestOutcome:
        """Execute test on candidate (if function provided)."""
        if self._test_function is None:
            raise ValueError(f"Test {self.test_id} has no executable function")
        return self._test_function(candidate)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "TEST_DEFINITION",
            "test_id": self.test_id,
            "level": self.level.level_name,
            "outcome_space_size": self.outcome_space_size(),
            "cost": self.cost
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self, domain: Optional[InterventionSpace] = None) -> Dict[str, Any]:
        total = domain is None or self.is_total(domain)
        return {
            "type": "TEST_DEFINITION",
            "test_id": self.test_id,
            "test_level": self.level.level_name,
            "outcome_space_size": self.outcome_space_size(),
            "cost": self.cost,
            "total": total,
            "result": "PASS" if total else "FAIL"
        }


@dataclass
class TestAlgebra:
    """
    Multi-level test algebra (T1-T6).

    Tests are organized by level, with nested prerequisites.
    """
    algebra_id: str
    tests: Dict[TestLevel, List[TestDefinition]]

    def __post_init__(self):
        # Ensure all levels present (even if empty)
        for level in TestLevel:
            if level not in self.tests:
                self.tests[level] = []

    def get_tests_at_level(self, level: TestLevel) -> List[TestDefinition]:
        """Get all tests at a given level."""
        return self.tests.get(level, [])

    def get_all_tests(self) -> List[TestDefinition]:
        """Get all tests across all levels."""
        result = []
        for level in TestLevel:
            result.extend(self.tests[level])
        return result

    def tests_per_level(self) -> List[int]:
        """Return count of tests at each level."""
        return [len(self.tests[level]) for level in TestLevel]

    def costs_per_level(self) -> List[int]:
        """Return total cost of tests at each level."""
        return [
            sum(t.cost for t in self.tests[level])
            for level in TestLevel
        ]

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "TEST_ALGEBRA",
            "algebra_id": self.algebra_id,
            "levels": [level.level_name for level in TestLevel],
            "tests_per_level": self.tests_per_level()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "TEST_ALGEBRA",
            "algebra_id": self.algebra_id,
            "levels": [level.level_name for level in TestLevel],
            "tests_per_level": self.tests_per_level(),
            "costs_per_level": self.costs_per_level(),
            "nested_prerequisite": True,
            "result": "PASS"
        }


@dataclass(frozen=True)
class LedgerEntry:
    """
    A single entry in the discovery ledger.

    Records (test, candidate, outcome) with timestamp.
    """
    entry_id: str
    test_id: str
    candidate_id: str
    outcome: TestOutcome
    cost: int
    timestamp: int  # Logical time (step number)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "LEDGER_ENTRY",
            "entry_id": self.entry_id,
            "test_id": self.test_id,
            "candidate_id": self.candidate_id,
            "outcome": self.outcome.canonical(),
            "cost": self.cost,
            "timestamp": self.timestamp
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class DiscoveryLedger:
    """
    The ledger L recording executed tests and outcomes.

    W(L) = {x in D_0 : for all (tau, a) in L applied to x, tau(x) = a}
    """
    ledger_id: str
    entries: List[LedgerEntry] = field(default_factory=list)

    def add_entry(
        self,
        test: TestDefinition,
        candidate: Intervention,
        outcome: TestOutcome
    ) -> LedgerEntry:
        """Add a new entry to the ledger."""
        entry = LedgerEntry(
            entry_id=f"ENTRY_{len(self.entries)}",
            test_id=test.test_id,
            candidate_id=candidate.intervention_id,
            outcome=outcome,
            cost=test.cost,
            timestamp=len(self.entries)
        )
        self.entries.append(entry)
        return entry

    def total_cost(self) -> int:
        """Total cost expended so far."""
        return sum(e.cost for e in self.entries)

    def entry_count(self) -> int:
        """Number of entries in ledger."""
        return len(self.entries)

    def get_outcomes_for_candidate(
        self,
        candidate_id: str
    ) -> Dict[str, TestOutcome]:
        """Get all recorded outcomes for a candidate."""
        outcomes = {}
        for entry in self.entries:
            if entry.candidate_id == candidate_id:
                outcomes[entry.test_id] = entry.outcome
        return outcomes

    def canonical(self) -> str:
        entry_hashes = [e.fingerprint() for e in self.entries]
        return CanonicalJSON.serialize({
            "type": "DISCOVERY_LEDGER",
            "ledger_id": self.ledger_id,
            "entry_count": self.entry_count(),
            "total_cost": self.total_cost(),
            "entry_hashes": entry_hashes
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(
        self,
        initial_survivors: int,
        final_survivors: int
    ) -> Dict[str, Any]:
        return {
            "type": "LEDGER_RECEIPT",
            "ledger_id": self.ledger_id,
            "entries_count": self.entry_count(),
            "total_cost": self.total_cost(),
            "initial_survivors": initial_survivors,
            "final_survivors": final_survivors,
            "monotone_decrease": final_survivors <= initial_survivors,
            "ledger_fingerprint": self.fingerprint(),
            "result": "PASS" if final_survivors <= initial_survivors else "FAIL"
        }


@dataclass
class SurvivorSet:
    """
    The survivor set W(L) under a ledger.

    W(L) = {x in D_0 : all recorded tests on x match recorded outcomes}
    """
    survivor_id: str
    survivors: FrozenSet[Intervention]
    ledger: DiscoveryLedger
    domain: InterventionSpace

    def size(self) -> int:
        """Return |W(L)|."""
        return len(self.survivors)

    def contains(self, candidate: Intervention) -> bool:
        """Check if candidate is in W(L)."""
        return candidate in self.survivors

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "SURVIVOR_SET",
            "survivor_id": self.survivor_id,
            "size": self.size(),
            "domain_size": self.domain.size(),
            "ledger_entries": self.ledger.entry_count()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


def compute_survivors(
    domain: InterventionSpace,
    ledger: DiscoveryLedger,
    tests: Dict[str, TestDefinition]
) -> SurvivorSet:
    """
    Compute W(L) = {x in D_0 : consistent with ledger}.

    A candidate x is a survivor if for all entries (test, candidate, outcome)
    where candidate == x, we have test(x) == outcome.
    """
    survivors = set()

    for candidate in domain.candidates:
        # Get all outcomes recorded for this candidate
        recorded = ledger.get_outcomes_for_candidate(candidate.intervention_id)

        # Candidate survives if all recorded outcomes match test function results
        is_survivor = True
        for test_id, recorded_outcome in recorded.items():
            test = tests.get(test_id)
            if test and test._test_function:
                actual_outcome = test._test_function(candidate)
                if actual_outcome != recorded_outcome:
                    is_survivor = False
                    break

        if is_survivor:
            survivors.add(candidate)

    return SurvivorSet(
        survivor_id=f"W_L_{ledger.entry_count()}",
        survivors=frozenset(survivors),
        ledger=ledger,
        domain=domain
    )


# ============================================================
# HELPER FUNCTIONS FOR CREATING SAMPLE DATA
# ============================================================

def create_sample_interventions(
    count: int,
    intervention_type: InterventionType = InterventionType.MOLECULE
) -> FrozenSet[Intervention]:
    """Create sample intervention set for testing."""
    interventions = set()
    for i in range(count):
        properties = frozenset([
            ("solubility", i % 3),
            ("toxicity", i % 4),
            ("efficacy", i % 5)
        ])
        intervention = Intervention(
            intervention_id=f"CAND_{i:04d}",
            intervention_type=intervention_type,
            description=f"Candidate molecule {i}",
            properties=properties
        )
        interventions.add(intervention)
    return frozenset(interventions)


def create_sample_intervention_space(
    count: int = 100
) -> InterventionSpace:
    """Create sample intervention space."""
    candidates = create_sample_interventions(count)
    return InterventionSpace(
        space_id="SAMPLE_D0",
        candidates=candidates,
        intervention_type=InterventionType.MOLECULE
    )


def create_sample_test_outcomes() -> FrozenSet[TestOutcome]:
    """Create standard PASS/FAIL outcome set."""
    return frozenset([
        TestOutcome("PASS", True),
        TestOutcome("FAIL", False)
    ])


def create_sample_test_algebra(
    domain: InterventionSpace
) -> Tuple[TestAlgebra, Dict[str, TestDefinition]]:
    """Create sample test algebra with executable tests."""
    outcomes = create_sample_test_outcomes()
    pass_outcome = TestOutcome("PASS", True)
    fail_outcome = TestOutcome("FAIL", False)

    tests: Dict[TestLevel, List[TestDefinition]] = {}
    all_tests: Dict[str, TestDefinition] = {}

    # T1: In-silico test - check solubility property
    def t1_solubility(candidate: Intervention) -> TestOutcome:
        for k, v in candidate.properties:
            if k == "solubility" and v > 0:
                return pass_outcome
        return fail_outcome

    t1_test = TestDefinition(
        test_id="T1_SOLUBILITY",
        level=TestLevel.T1_IN_SILICO,
        outcome_space=outcomes,
        cost=1,
        description="In-silico solubility prediction",
        _test_function=t1_solubility
    )
    tests[TestLevel.T1_IN_SILICO] = [t1_test]
    all_tests[t1_test.test_id] = t1_test

    # T2: In-vitro test - check toxicity
    def t2_toxicity(candidate: Intervention) -> TestOutcome:
        for k, v in candidate.properties:
            if k == "toxicity" and v < 2:
                return pass_outcome
        return fail_outcome

    t2_test = TestDefinition(
        test_id="T2_TOXICITY",
        level=TestLevel.T2_IN_VITRO,
        outcome_space=outcomes,
        cost=100,
        description="In-vitro toxicity assay",
        _test_function=t2_toxicity
    )
    tests[TestLevel.T2_IN_VITRO] = [t2_test]
    all_tests[t2_test.test_id] = t2_test

    # T3: Cell-based test - check efficacy
    def t3_efficacy(candidate: Intervention) -> TestOutcome:
        for k, v in candidate.properties:
            if k == "efficacy" and v > 2:
                return pass_outcome
        return fail_outcome

    t3_test = TestDefinition(
        test_id="T3_EFFICACY",
        level=TestLevel.T3_CELL_BASED,
        outcome_space=outcomes,
        cost=1000,
        description="Cell-based efficacy assay",
        _test_function=t3_efficacy
    )
    tests[TestLevel.T3_CELL_BASED] = [t3_test]
    all_tests[t3_test.test_id] = t3_test

    # T4-T6: Empty for sample (would be animal/clinical/post-market)
    tests[TestLevel.T4_ANIMAL] = []
    tests[TestLevel.T5_CLINICAL] = []
    tests[TestLevel.T6_POST_MARKET] = []

    algebra = TestAlgebra(
        algebra_id="SAMPLE_ALGEBRA",
        tests=tests
    )

    return algebra, all_tests


def create_empty_ledger() -> DiscoveryLedger:
    """Create empty discovery ledger."""
    return DiscoveryLedger(ledger_id="LEDGER_0")
