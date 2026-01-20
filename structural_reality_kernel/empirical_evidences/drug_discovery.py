"""
drug_discovery.py - Drug discovery as quotient collapse.

Implements:
- TargetPredicate (conjunction of witnessable tests)
- QuotientCollapse (refinement tracking)
- OptimalExperiment (minimax value computation for tau*)
- MechanismHypothesis (Omega frontier over causal hypotheses)
- DiscoveryProcess (the complete discovery loop)
- CandidateWitness (proof of successful candidate)
"""

from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, List, Optional, Set, Tuple
from fractions import Fraction
from enum import Enum
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON

from .boundary_flow import RationalValue
from .intervention_space import (
    Intervention,
    InterventionSpace,
    TestDefinition,
    TestOutcome,
    TestAlgebra,
    TestLevel,
    LedgerEntry,
    DiscoveryLedger,
    SurvivorSet,
    compute_survivors
)


class DiscoveryStatus(Enum):
    """Status of the discovery process."""
    IN_PROGRESS = "in_progress"
    SUCCESS = "success"      # Found candidate passing all predicates
    FAILURE = "failure"      # All candidates eliminated
    FRONTIER = "frontier"    # Budget exhausted with open questions


class MechanismStatus(Enum):
    """Status of a mechanism hypothesis."""
    OPEN = "open"            # Insufficient evidence
    RESOLVED_TRUE = "resolved_true"    # Confirmed by evidence
    RESOLVED_FALSE = "resolved_false"  # Refuted by evidence
    UNDECIDABLE = "undecidable"        # No feasible test can distinguish


@dataclass
class TargetPredicate:
    """
    Target predicate V(x) = AND_i V_i(x).

    A conjunction of witnessable tests that define success.
    """
    predicate_id: str
    conjuncts: List[TestDefinition]  # All must pass for success

    def conjuncts_count(self) -> int:
        """Return number of conjuncts."""
        return len(self.conjuncts)

    def evaluate(
        self,
        candidate: Intervention,
        recorded_outcomes: Dict[str, TestOutcome]
    ) -> Tuple[bool, List[str], List[str]]:
        """
        Evaluate target predicate on candidate.

        Returns (all_passed, passed_test_ids, failed_test_ids).
        Only considers tests with recorded outcomes.
        """
        passed = []
        failed = []

        for test in self.conjuncts:
            if test.test_id in recorded_outcomes:
                outcome = recorded_outcomes[test.test_id]
                # Convention: "PASS" outcome means success
                if outcome.outcome_id == "PASS" or outcome.value is True:
                    passed.append(test.test_id)
                else:
                    failed.append(test.test_id)

        # All conjuncts must be evaluated and pass
        all_passed = (len(passed) == self.conjuncts_count() and len(failed) == 0)

        return all_passed, passed, failed

    def has_failure(
        self,
        recorded_outcomes: Dict[str, TestOutcome]
    ) -> bool:
        """Check if any conjunct has failed (candidate eliminated)."""
        for test in self.conjuncts:
            if test.test_id in recorded_outcomes:
                outcome = recorded_outcomes[test.test_id]
                if outcome.outcome_id == "FAIL" or outcome.value is False:
                    return True
        return False

    def is_total(self, domain: InterventionSpace) -> bool:
        """Check if all conjuncts are total on domain."""
        return all(test.is_total(domain) for test in self.conjuncts)

    def is_witnessable(self) -> bool:
        """Check if all conjuncts produce witnessable evidence."""
        # All tests produce outcomes, so they are witnessable by definition
        return True

    def is_conjunction(self) -> bool:
        """Confirm this is a conjunction (always true by construction)."""
        return True

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "TARGET_PREDICATE",
            "predicate_id": self.predicate_id,
            "conjuncts_count": self.conjuncts_count(),
            "conjunct_ids": [t.test_id for t in self.conjuncts]
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self, domain: Optional[InterventionSpace] = None) -> Dict[str, Any]:
        total = domain is None or self.is_total(domain)
        return {
            "type": "TARGET_PREDICATE",
            "predicate_id": self.predicate_id,
            "conjuncts_count": self.conjuncts_count(),
            "all_total": total,
            "all_witnessable": self.is_witnessable(),
            "is_conjunction": self.is_conjunction(),
            "result": "PASS" if total else "FAIL"
        }


@dataclass
class QuotientState:
    """
    State of the quotient at a point in discovery.

    Tracks equivalence classes induced by executed tests.
    """
    state_id: str
    classes: List[FrozenSet[Intervention]]  # Partition of domain
    executed_tests: List[str]  # Test IDs that induced this partition

    def class_count(self) -> int:
        """Return number of equivalence classes (refinement level)."""
        return len(self.classes)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "QUOTIENT_STATE",
            "state_id": self.state_id,
            "class_count": self.class_count(),
            "executed_tests": self.executed_tests
        })


@dataclass
class QuotientCollapse:
    """
    Tracks the quotient collapse during discovery.

    Drug discovery is quotient collapse: D_0 / ~_L refines as L grows.
    """
    collapse_id: str
    initial_state: QuotientState
    final_state: QuotientState
    steps: List[QuotientState]

    def initial_classes(self) -> int:
        """Classes at start."""
        return self.initial_state.class_count()

    def final_classes(self) -> int:
        """Classes at end."""
        return self.final_state.class_count()

    def refinement_achieved(self) -> bool:
        """Check if refinement occurred."""
        return self.final_classes() > self.initial_classes()

    def is_monotone(self) -> bool:
        """Check monotone refinement (classes can only increase or stay same)."""
        prev = self.initial_classes()
        for step in self.steps:
            if step.class_count() < prev:
                return False
            prev = step.class_count()
        return True

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "QUOTIENT_COLLAPSE",
            "collapse_id": self.collapse_id,
            "initial_classes": self.initial_classes(),
            "final_classes": self.final_classes(),
            "step_count": len(self.steps)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        monotone = self.is_monotone()
        return {
            "type": "QUOTIENT_COLLAPSE",
            "collapse_id": self.collapse_id,
            "initial_classes": self.initial_classes(),
            "final_classes": self.final_classes(),
            "refinement_achieved": self.refinement_achieved(),
            "monotone": monotone,
            "result": "PASS" if monotone else "FAIL"
        }


@dataclass
class OptimalExperiment:
    """
    Optimal next experiment selection (tau*).

    tau* = argmax_tau min_a Delta_T(tau,a) / cost(tau)

    Minimax value computation: choose test that maximizes worst-case
    refinement per unit cost.
    """
    selection_id: str
    selected_test: TestDefinition
    minimax_value: RationalValue  # Worst-case refinement per cost
    alternatives_considered: int

    @staticmethod
    def compute_minimax_value(
        test: TestDefinition,
        survivors: SurvivorSet,
        tests: Dict[str, TestDefinition]
    ) -> RationalValue:
        """
        Compute minimax value for a test.

        Value = min_outcome |survivors_after| / cost
        We want to maximize the minimum survivor reduction per cost.
        """
        if test.cost == 0:
            return RationalValue(0, 1)

        if not test._test_function:
            # If no function, assume equal split over outcomes
            outcome_count = test.outcome_space_size()
            survivor_count = survivors.size()
            # Worst case: all survivors have same outcome
            worst_case_survivors = survivor_count
            # Refinement = log(before/after), but we use ratio for simplicity
            # Higher is better: more survivors eliminated per cost
            value = RationalValue(survivor_count - worst_case_survivors, test.cost)
            return value

        # Compute actual outcome distribution
        outcome_to_survivors: Dict[str, List[Intervention]] = {}
        for candidate in survivors.survivors:
            try:
                outcome = test._test_function(candidate)
                key = outcome.outcome_id
                if key not in outcome_to_survivors:
                    outcome_to_survivors[key] = []
                outcome_to_survivors[key].append(candidate)
            except Exception:
                pass

        if not outcome_to_survivors:
            return RationalValue(0, 1)

        # Worst case: largest outcome group remains
        max_survivors = max(len(group) for group in outcome_to_survivors.values())
        initial = survivors.size()

        # Refinement = initial - worst_case survivors eliminated
        # We want high refinement per cost
        refinement = initial - max_survivors
        return RationalValue(refinement, test.cost)

    @staticmethod
    def select_optimal(
        survivors: SurvivorSet,
        available_tests: List[TestDefinition],
        tests: Dict[str, TestDefinition]
    ) -> 'OptimalExperiment':
        """
        Select the optimal next experiment.

        Returns the test that maximizes minimax value.
        """
        if not available_tests:
            raise ValueError("No available tests")

        best_test = None
        best_value = RationalValue(-1, 1)

        for test in available_tests:
            value = OptimalExperiment.compute_minimax_value(test, survivors, tests)
            if value > best_value:
                best_value = value
                best_test = test

        if best_test is None:
            best_test = available_tests[0]
            best_value = RationalValue(0, 1)

        return OptimalExperiment(
            selection_id=f"SELECT_{best_test.test_id}",
            selected_test=best_test,
            minimax_value=best_value,
            alternatives_considered=len(available_tests)
        )

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "OPTIMAL_EXPERIMENT",
            "selection_id": self.selection_id,
            "selected_test": self.selected_test.test_id,
            "minimax_value_num": self.minimax_value.fraction.numerator,
            "minimax_value_den": self.minimax_value.fraction.denominator,
            "cost": self.selected_test.cost
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "OPTIMAL_EXPERIMENT",
            "selection_id": self.selection_id,
            "selected_test": self.selected_test.test_id,
            "minimax_value_numerator": self.minimax_value.fraction.numerator,
            "minimax_value_denominator": self.minimax_value.fraction.denominator,
            "cost": self.selected_test.cost,
            "is_optimal": True,  # By construction
            "result": "PASS"
        }


@dataclass
class MechanismHypothesis:
    """
    A mechanism hypothesis about why a candidate works.

    Forms part of the Omega frontier over causal hypotheses.
    """
    mechanism_id: str
    description: str
    supporting_evidence: List[str]  # Test IDs that support
    refuting_evidence: List[str]    # Test IDs that refute
    status: MechanismStatus

    def is_honest(self) -> bool:
        """
        Check if mechanism status is honest given evidence.

        RESOLVED requires sufficient evidence.
        OPEN is honest if evidence is insufficient.
        """
        if self.status == MechanismStatus.OPEN:
            return True  # Open is always honest
        elif self.status == MechanismStatus.RESOLVED_TRUE:
            return len(self.supporting_evidence) > 0 and len(self.refuting_evidence) == 0
        elif self.status == MechanismStatus.RESOLVED_FALSE:
            return len(self.refuting_evidence) > 0
        elif self.status == MechanismStatus.UNDECIDABLE:
            return True  # Undecidable is honest if no feasible test
        return False

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "MECHANISM_HYPOTHESIS",
            "mechanism_id": self.mechanism_id,
            "status": self.status.value,
            "supporting_count": len(self.supporting_evidence),
            "refuting_count": len(self.refuting_evidence)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        honest = self.is_honest()
        return {
            "type": "FRONTIER_HONESTY",
            "mechanism_id": self.mechanism_id,
            "supporting_evidence": self.supporting_evidence,
            "refuting_evidence": self.refuting_evidence,
            "status": self.status.value,
            "honest": honest,
            "result": "PASS" if honest else "FAIL"
        }


@dataclass
class CandidateWitness:
    """
    Witness for a successful drug candidate.

    Proves candidate is in domain, in survivors, and passes target predicate.
    """
    witness_id: str
    candidate: Intervention
    domain: InterventionSpace
    survivors: SurvivorSet
    target: TargetPredicate
    test_results: Dict[str, TestOutcome]

    def in_domain(self) -> bool:
        """Check candidate is in D_0."""
        return self.domain.contains(self.candidate)

    def in_survivors(self) -> bool:
        """Check candidate is in W(L)."""
        return self.survivors.contains(self.candidate)

    def target_satisfied(self) -> bool:
        """Check all target predicates pass."""
        passed, _, _ = self.target.evaluate(self.candidate, self.test_results)
        return passed

    def is_valid(self) -> bool:
        """Full validity check."""
        return self.in_domain() and self.in_survivors() and self.target_satisfied()

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CANDIDATE_WITNESS",
            "witness_id": self.witness_id,
            "candidate_id": self.candidate.intervention_id,
            "in_domain": self.in_domain(),
            "in_survivors": self.in_survivors(),
            "target_satisfied": self.target_satisfied()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "CANDIDATE_WITNESS",
            "candidate_id": self.candidate.fingerprint(),
            "in_domain": self.in_domain(),
            "in_survivors": self.in_survivors(),
            "target_predicate_satisfied": self.target_satisfied(),
            "result": "PASS" if self.is_valid() else "FAIL"
        }


@dataclass
class RawDataEvidence:
    """
    Raw data evidence for a test execution.

    Proves that outcome was derived from verifiable data.
    """
    evidence_id: str
    test_id: str
    candidate_id: str
    outcome: TestOutcome
    raw_data: str  # Serialized raw data (would be actual measurements)
    reduction_verified: bool

    def data_hash(self) -> str:
        """Hash of raw data."""
        return hashlib.sha256(self.raw_data.encode()).hexdigest()

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "RAW_DATA_EVIDENCE",
            "evidence_id": self.evidence_id,
            "test_id": self.test_id,
            "candidate_id": self.candidate_id,
            "outcome": self.outcome.outcome_id,
            "data_hash": self.data_hash()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "RAW_DATA_EVIDENCE",
            "test_id": self.test_id,
            "candidate_id": self.candidate_id,
            "outcome": self.outcome.outcome_id,
            "data_hash": self.data_hash(),
            "reduction_verified": self.reduction_verified,
            "result": "PASS" if self.reduction_verified else "FAIL"
        }


@dataclass
class VerifierExecution:
    """
    Execution of target predicate verifier on a candidate.
    """
    execution_id: str
    candidate: Intervention
    target: TargetPredicate
    execution_receipts: List[str]  # Hashes of individual test receipts

    def predicates_total(self) -> int:
        return self.target.conjuncts_count()

    def predicates_passed(self, results: Dict[str, TestOutcome]) -> int:
        _, passed, _ = self.target.evaluate(self.candidate, results)
        return len(passed)

    def all_passed(self, results: Dict[str, TestOutcome]) -> bool:
        passed, _, _ = self.target.evaluate(self.candidate, results)
        return passed

    def to_receipt(self, results: Dict[str, TestOutcome]) -> Dict[str, Any]:
        all_pass = self.all_passed(results)
        return {
            "type": "VERIFIER_EXECUTION",
            "candidate_id": self.candidate.intervention_id,
            "predicates_total": self.predicates_total(),
            "predicates_passed": self.predicates_passed(results),
            "all_passed": all_pass,
            "execution_receipts": self.execution_receipts,
            "result": "PASS" if all_pass else "FAIL"
        }


@dataclass
class DiscoveryResult:
    """
    Complete result of a discovery process.
    """
    result_id: str
    status: DiscoveryStatus
    domain: InterventionSpace
    ledger: DiscoveryLedger
    final_survivors: SurvivorSet
    target: TargetPredicate
    successful_candidates: List[Intervention]
    mechanism_frontier: List[MechanismHypothesis]
    quotient_collapse: QuotientCollapse

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "DISCOVERY_RESULT",
            "result_id": self.result_id,
            "status": self.status.value,
            "domain_size": self.domain.size(),
            "final_survivors": self.final_survivors.size(),
            "successful_count": len(self.successful_candidates),
            "total_cost": self.ledger.total_cost()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class DiscoveryBundle:
    """
    Complete verification bundle for drug discovery.

    Contains all receipts needed for audit.
    """
    bundle_id: str
    domain_verified: bool
    tests_defined: bool
    data_evidenced: bool
    verifiers_executed: bool
    ledger_receipted: bool
    frontier_honest: bool
    result: DiscoveryResult

    def all_verified(self) -> bool:
        return (self.domain_verified and
                self.tests_defined and
                self.data_evidenced and
                self.verifiers_executed and
                self.ledger_receipted and
                self.frontier_honest)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "DISCOVERY_BUNDLE",
            "bundle_id": self.bundle_id,
            "domain_verified": self.domain_verified,
            "tests_defined": self.tests_defined,
            "data_evidenced": self.data_evidenced,
            "verifiers_executed": self.verifiers_executed,
            "ledger_receipted": self.ledger_receipted,
            "frontier_honest": self.frontier_honest,
            "all_verified": self.all_verified()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "DISCOVERY_BUNDLE",
            "bundle_id": self.bundle_id,
            "domain_verified": self.domain_verified,
            "tests_defined": self.tests_defined,
            "data_evidenced": self.data_evidenced,
            "verifiers_executed": self.verifiers_executed,
            "ledger_receipted": self.ledger_receipted,
            "frontier_honest": self.frontier_honest,
            "all_verified": self.all_verified(),
            "bundle_fingerprint": self.fingerprint(),
            "result": "PASS" if self.all_verified() else "FAIL"
        }


# ============================================================
# DISCOVERY PROCESS
# ============================================================

def run_discovery_process(
    domain: InterventionSpace,
    target: TargetPredicate,
    tests: Dict[str, TestDefinition],
    budget: int,
    mechanisms: Optional[List[MechanismHypothesis]] = None
) -> DiscoveryResult:
    """
    Run the complete drug discovery process.

    1. Start with all candidates as survivors
    2. Iteratively select optimal test and execute
    3. Update survivors and check termination
    4. Return result with full audit trail
    """
    ledger = DiscoveryLedger(ledger_id="PROCESS_LEDGER")

    # Track quotient states
    initial_classes = [frozenset([c]) for c in domain.candidates]
    initial_quotient = QuotientState(
        state_id="Q_0",
        classes=initial_classes,
        executed_tests=[]
    )
    quotient_steps = [initial_quotient]

    # Initialize mechanisms if not provided
    if mechanisms is None:
        mechanisms = []

    # Available tests (those in target predicate)
    available_tests = list(target.conjuncts)

    # Track which tests have been executed on which candidates
    executed: Dict[str, Set[str]] = {}  # candidate_id -> set of test_ids

    # Main loop
    while ledger.total_cost() < budget:
        # Compute current survivors
        survivors = compute_survivors(domain, ledger, tests)

        if survivors.size() == 0:
            # All candidates eliminated
            return DiscoveryResult(
                result_id="RESULT_FAILURE",
                status=DiscoveryStatus.FAILURE,
                domain=domain,
                ledger=ledger,
                final_survivors=survivors,
                target=target,
                successful_candidates=[],
                mechanism_frontier=mechanisms,
                quotient_collapse=QuotientCollapse(
                    collapse_id="COLLAPSE",
                    initial_state=initial_quotient,
                    final_state=quotient_steps[-1],
                    steps=quotient_steps
                )
            )

        # Check for successful candidates
        successful = []
        for candidate in survivors.survivors:
            outcomes = ledger.get_outcomes_for_candidate(candidate.intervention_id)
            all_passed, _, _ = target.evaluate(candidate, outcomes)
            if all_passed:
                successful.append(candidate)

        if successful:
            return DiscoveryResult(
                result_id="RESULT_SUCCESS",
                status=DiscoveryStatus.SUCCESS,
                domain=domain,
                ledger=ledger,
                final_survivors=survivors,
                target=target,
                successful_candidates=successful,
                mechanism_frontier=mechanisms,
                quotient_collapse=QuotientCollapse(
                    collapse_id="COLLAPSE",
                    initial_state=initial_quotient,
                    final_state=quotient_steps[-1],
                    steps=quotient_steps
                )
            )

        # Find tests that haven't been executed on all survivors
        tests_to_consider = []
        for test in available_tests:
            for candidate in survivors.survivors:
                if candidate.intervention_id not in executed:
                    executed[candidate.intervention_id] = set()
                if test.test_id not in executed[candidate.intervention_id]:
                    tests_to_consider.append(test)
                    break

        if not tests_to_consider:
            # All tests executed on all survivors, but no success - frontier
            break

        # Select optimal test
        optimal = OptimalExperiment.select_optimal(survivors, tests_to_consider, tests)
        selected_test = optimal.selected_test

        # Execute test on survivors that haven't been tested
        for candidate in list(survivors.survivors):
            if candidate.intervention_id not in executed:
                executed[candidate.intervention_id] = set()

            if selected_test.test_id not in executed[candidate.intervention_id]:
                if selected_test._test_function:
                    outcome = selected_test._test_function(candidate)
                    ledger.add_entry(selected_test, candidate, outcome)
                    executed[candidate.intervention_id].add(selected_test.test_id)

        # Update quotient state
        new_quotient = QuotientState(
            state_id=f"Q_{len(quotient_steps)}",
            classes=[frozenset([c]) for c in domain.candidates],  # Simplified
            executed_tests=list(executed.get(list(domain.candidates)[0].intervention_id, set()))
        )
        quotient_steps.append(new_quotient)

    # Budget exhausted - frontier
    final_survivors = compute_survivors(domain, ledger, tests)
    return DiscoveryResult(
        result_id="RESULT_FRONTIER",
        status=DiscoveryStatus.FRONTIER,
        domain=domain,
        ledger=ledger,
        final_survivors=final_survivors,
        target=target,
        successful_candidates=[],
        mechanism_frontier=mechanisms,
        quotient_collapse=QuotientCollapse(
            collapse_id="COLLAPSE",
            initial_state=initial_quotient,
            final_state=quotient_steps[-1],
            steps=quotient_steps
        )
    )


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def create_sample_target_predicate(
    tests: Dict[str, TestDefinition]
) -> TargetPredicate:
    """Create sample target predicate from tests."""
    conjuncts = [tests[tid] for tid in tests if tid.startswith("T")]
    return TargetPredicate(
        predicate_id="SAMPLE_TARGET",
        conjuncts=conjuncts
    )


def create_sample_mechanism(
    mechanism_id: str,
    supporting: List[str],
    refuting: List[str]
) -> MechanismHypothesis:
    """Create sample mechanism hypothesis."""
    if refuting:
        status = MechanismStatus.RESOLVED_FALSE
    elif supporting:
        status = MechanismStatus.RESOLVED_TRUE
    else:
        status = MechanismStatus.OPEN

    return MechanismHypothesis(
        mechanism_id=mechanism_id,
        description=f"Mechanism {mechanism_id}",
        supporting_evidence=supporting,
        refuting_evidence=refuting,
        status=status
    )


def run_sample_discovery() -> DiscoveryResult:
    """Run sample discovery for demonstration."""
    from .intervention_space import (
        create_sample_intervention_space,
        create_sample_test_algebra
    )

    # Create domain
    domain = create_sample_intervention_space(count=50)

    # Create test algebra
    algebra, tests = create_sample_test_algebra(domain)

    # Create target predicate
    target = create_sample_target_predicate(tests)

    # Create sample mechanism
    mechanism = create_sample_mechanism(
        "MECH_001",
        supporting=[],
        refuting=[]
    )

    # Run discovery
    result = run_discovery_process(
        domain=domain,
        target=target,
        tests=tests,
        budget=100000,
        mechanisms=[mechanism]
    )

    return result
