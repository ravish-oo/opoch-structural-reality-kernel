"""
computation.py - Computational problems as quotient collapse.

A computational problem is the task of collapsing an answer partition
of a finite possibility space using feasible witness tests.

Ans_L(q) := {q(x) : x in W(L)}
Solved <=> |Ans_L(q)| = 1
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import Test
from core.receipts import CanonicalJSON


@dataclass
class Query:
    """
    A query q: D0 -> B mapping possibilities to answers.

    The query partitions the possibility space into answer fibers.
    """
    query_id: str
    evaluator: Callable[[Any], Any]
    answer_space: FrozenSet[Any]

    def evaluate(self, x: Any) -> Any:
        """Evaluate query on element."""
        return self.evaluator(x)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "QUERY",
            "query_id": self.query_id,
            "answer_space_size": len(self.answer_space)
        })


@dataclass
class AnswerSet:
    """
    The answer set Ans_L(q) = {q(x) : x in W(L)}.

    This is the only admissible "remaining answer state".
    """
    query_id: str
    answers: FrozenSet[Any]
    survivors: FrozenSet[Any]
    answer_partition: Dict[Any, FrozenSet[Any]]  # answer -> elements

    @property
    def is_solved(self) -> bool:
        """Solved <=> |Ans| = 1."""
        return len(self.answers) == 1

    @property
    def answer_count(self) -> int:
        return len(self.answers)

    @property
    def unique_answer(self) -> Optional[Any]:
        """Get the unique answer if solved."""
        if self.is_solved:
            return next(iter(self.answers))
        return None

    def get_witness(self) -> Optional[Any]:
        """Get a witness for the unique answer if solved."""
        if self.is_solved:
            answer = self.unique_answer
            witnesses = self.answer_partition.get(answer, frozenset())
            if witnesses:
                return next(iter(witnesses))
        return None

    def fingerprint(self) -> str:
        """Compute frontier fingerprint."""
        sorted_answers = sorted(str(a) for a in self.answers)
        canonical = CanonicalJSON.serialize({
            "query_id": self.query_id,
            "answers": sorted_answers,
            "survivor_count": len(self.survivors)
        })
        return hashlib.sha256(canonical.encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "ANSWER_SET",
            "query_id": self.query_id,
            "survivor_count": len(self.survivors),
            "answer_count": self.answer_count,
            "is_solved": self.is_solved,
            "unique_answer": str(self.unique_answer) if self.is_solved else "NONE",
            "frontier_fingerprint": self.fingerprint()[:32],
            "result": "PASS"
        }


@dataclass
class ComputationContract:
    """
    A complete computation contract.

    Specifies:
    - D0: finite possibility space
    - verifier V: total PASS/FAIL
    - cost model c(tau)
    """
    contract_id: str
    d0: FrozenSet[Any]
    query: Query
    tests: Dict[str, Test]

    def __post_init__(self):
        # Build cost model
        self.cost_model = {tid: test.cost for tid, test in self.tests.items()}

    @property
    def d0_size(self) -> int:
        return len(self.d0)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "COMPUTATION_CONTRACT",
            "contract_id": self.contract_id,
            "d0_size": self.d0_size,
            "query_id": self.query.query_id,
            "test_count": len(self.tests)
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "COMPUTATION_CONTRACT",
            "contract_id": self.contract_id,
            "d0_size": self.d0_size,
            "verifier_id": self.query.query_id,
            "cost_model": "explicit",
            "answer_space_size": len(self.query.answer_space),
            "test_count": len(self.tests),
            "result": "PASS"
        }


@dataclass
class SeparatorWitness:
    """
    A witness that a test separates the answer set.

    A separator tau partitions W into outcome fibers W_a.
    """
    separator_id: str
    test: Test
    is_total: bool
    partition: Dict[Any, FrozenSet[Any]]  # outcome -> elements
    original_answer_count: int
    reduced_answer_counts: Dict[Any, int]  # outcome -> answer count in fiber

    @property
    def partition_count(self) -> int:
        """Number of nonempty partitions."""
        return len([v for v in self.partition.values() if v])

    @property
    def reduces_answer(self) -> bool:
        """Does this separator reduce |Ans| on at least one branch?"""
        for outcome, count in self.reduced_answer_counts.items():
            if count < self.original_answer_count:
                return True
        return False

    @property
    def max_reduced_count(self) -> int:
        """Maximum answer count across all branches (worst case)."""
        if not self.reduced_answer_counts:
            return self.original_answer_count
        return max(self.reduced_answer_counts.values())

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "SEPARATOR_WITNESS",
            "separator_id": self.separator_id,
            "is_total": self.is_total,
            "partition_count": self.partition_count,
            "reduces_answer": self.reduces_answer
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SEPARATOR_WITNESS",
            "separator_id": self.separator_id,
            "is_total": self.is_total,
            "partitions_count": self.partition_count,
            "reduces_answer": self.reduces_answer,
            "cost": self.test.cost,
            "original_answers": self.original_answer_count,
            "max_branch_answers": self.max_reduced_count,
            "result": "PASS" if self.is_total else "FAIL"
        }


class AnswerComputer:
    """
    Computes the answer set Ans_L(q) from survivors.
    """

    def __init__(self, query: Query):
        self.query = query

    def compute_answer_set(self, survivors: FrozenSet[Any]) -> AnswerSet:
        """
        Compute Ans_L(q) = {q(x) : x in W(L)}.
        """
        answers: Set[Any] = set()
        partition: Dict[Any, Set[Any]] = {}

        for x in survivors:
            answer = self.query.evaluate(x)
            answers.add(answer)

            if answer not in partition:
                partition[answer] = set()
            partition[answer].add(x)

        return AnswerSet(
            query_id=self.query.query_id,
            answers=frozenset(answers),
            survivors=survivors,
            answer_partition={a: frozenset(s) for a, s in partition.items()}
        )

    def apply_separator(
        self,
        survivors: FrozenSet[Any],
        test: Test
    ) -> SeparatorWitness:
        """
        Apply a separator test and compute the partition.
        """
        # Compute original answer set
        original_answer_set = self.compute_answer_set(survivors)

        # Partition survivors by test outcome
        partition: Dict[Any, Set[Any]] = {}
        all_have_outcome = True

        for x in survivors:
            try:
                outcome = test.evaluator(x)
                if outcome not in partition:
                    partition[outcome] = set()
                partition[outcome].add(x)
            except Exception:
                all_have_outcome = False

        # Compute answer counts for each fiber
        reduced_counts: Dict[Any, int] = {}
        for outcome, fiber in partition.items():
            fiber_answers = set(self.query.evaluate(x) for x in fiber)
            reduced_counts[outcome] = len(fiber_answers)

        return SeparatorWitness(
            separator_id=test.test_id,
            test=test,
            is_total=all_have_outcome,
            partition={o: frozenset(s) for o, s in partition.items()},
            original_answer_count=original_answer_set.answer_count,
            reduced_answer_counts=reduced_counts
        )


@dataclass
class QuotientCollapseStep:
    """
    A single step in quotient collapse.
    """
    step_index: int
    separator_used: str
    outcome_observed: Any
    survivors_before: int
    survivors_after: int
    answers_before: int
    answers_after: int
    cost_incurred: int

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "QUOTIENT_COLLAPSE_STEP",
            "step": self.step_index,
            "separator": self.separator_used,
            "answers_before": self.answers_before,
            "answers_after": self.answers_after
        })


@dataclass
class QuotientCollapseTrace:
    """
    Complete trace of quotient collapse.
    """
    contract_id: str
    steps: List[QuotientCollapseStep]
    final_answer_set: AnswerSet
    total_cost: int

    @property
    def is_solved(self) -> bool:
        return self.final_answer_set.is_solved

    @property
    def step_count(self) -> int:
        return len(self.steps)

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "QUOTIENT_COLLAPSE_TRACE",
            "contract_id": self.contract_id,
            "step_count": self.step_count,
            "total_cost": self.total_cost,
            "is_solved": self.is_solved,
            "final_answer_count": self.final_answer_set.answer_count,
            "result": "PASS" if self.is_solved else "OMEGA"
        }


class QuotientCollapser:
    """
    Performs quotient collapse using separators.
    """

    def __init__(self, contract: ComputationContract):
        self.contract = contract
        self.answer_computer = AnswerComputer(contract.query)

    def collapse_with_sequence(
        self,
        survivors: FrozenSet[Any],
        separator_sequence: List[Tuple[str, Any]]  # (test_id, outcome)
    ) -> QuotientCollapseTrace:
        """
        Collapse using a specific sequence of separators and outcomes.
        """
        current_survivors = survivors
        steps: List[QuotientCollapseStep] = []
        total_cost = 0

        for i, (test_id, expected_outcome) in enumerate(separator_sequence):
            if test_id not in self.contract.tests:
                continue

            test = self.contract.tests[test_id]
            answer_before = self.answer_computer.compute_answer_set(current_survivors)

            # Apply separator
            witness = self.answer_computer.apply_separator(current_survivors, test)

            # Take the fiber matching expected outcome
            if expected_outcome in witness.partition:
                new_survivors = witness.partition[expected_outcome]
            else:
                # No elements with this outcome
                new_survivors = frozenset()

            answer_after = self.answer_computer.compute_answer_set(new_survivors)

            steps.append(QuotientCollapseStep(
                step_index=i,
                separator_used=test_id,
                outcome_observed=expected_outcome,
                survivors_before=len(current_survivors),
                survivors_after=len(new_survivors),
                answers_before=answer_before.answer_count,
                answers_after=answer_after.answer_count,
                cost_incurred=test.cost
            ))

            total_cost += test.cost
            current_survivors = new_survivors

            # Stop if solved
            if answer_after.is_solved:
                break

        final_answer_set = self.answer_computer.compute_answer_set(current_survivors)

        return QuotientCollapseTrace(
            contract_id=self.contract.contract_id,
            steps=steps,
            final_answer_set=final_answer_set,
            total_cost=total_cost
        )

    def collapse_greedy(
        self,
        survivors: FrozenSet[Any],
        max_steps: int = 100
    ) -> QuotientCollapseTrace:
        """
        Collapse using greedy separator selection.

        At each step, choose the separator that minimizes worst-case
        remaining answers weighted by cost.
        """
        current_survivors = survivors
        steps: List[QuotientCollapseStep] = []
        total_cost = 0

        for i in range(max_steps):
            answer_set = self.answer_computer.compute_answer_set(current_survivors)

            if answer_set.is_solved:
                break

            # Find best separator
            best_test_id = None
            best_score = float('inf')
            best_outcome = None

            for test_id, test in self.contract.tests.items():
                witness = self.answer_computer.apply_separator(current_survivors, test)

                if not witness.is_total:
                    continue

                # Score = cost + max remaining answers
                score = test.cost + witness.max_reduced_count

                if score < best_score:
                    best_score = score
                    best_test_id = test_id
                    # Take the branch with minimum answers
                    min_answers = float('inf')
                    for outcome, count in witness.reduced_answer_counts.items():
                        if count < min_answers:
                            min_answers = count
                            best_outcome = outcome

            if best_test_id is None:
                break

            # Apply best separator
            test = self.contract.tests[best_test_id]
            witness = self.answer_computer.apply_separator(current_survivors, test)
            new_survivors = witness.partition.get(best_outcome, frozenset())
            answer_after = self.answer_computer.compute_answer_set(new_survivors)

            steps.append(QuotientCollapseStep(
                step_index=i,
                separator_used=best_test_id,
                outcome_observed=best_outcome,
                survivors_before=len(current_survivors),
                survivors_after=len(new_survivors),
                answers_before=answer_set.answer_count,
                answers_after=answer_after.answer_count,
                cost_incurred=test.cost
            ))

            total_cost += test.cost
            current_survivors = new_survivors

        final_answer_set = self.answer_computer.compute_answer_set(current_survivors)

        return QuotientCollapseTrace(
            contract_id=self.contract.contract_id,
            steps=steps,
            final_answer_set=final_answer_set,
            total_cost=total_cost
        )
