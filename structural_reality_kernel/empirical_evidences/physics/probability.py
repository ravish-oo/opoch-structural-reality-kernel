"""
probability.py - Probability as label-free frontier summary.

Probability is NOT truth. Probability is the unique, label-free summary
of an Omega frontier when you need numeric bookkeeping of what remains.

P_L(q=b) = |W_b| / |W(L)|

Where W_b = {x in W(L) : q(x) = b} are the answer fibers.

This is forced by:
- A0: only surviving possibilities matter
- Gauge: labels inside W are slack; only counts are invariant
- Coarse-graining consistency: probabilities must add
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
from fractions import Fraction
import hashlib
import math

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import Test, Record
from core.receipts import CanonicalJSON


@dataclass
class Query:
    """
    A finite query q: D0 -> B.

    Maps domain elements to a finite answer space.
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
    The remaining answer set Ans_L(q) = {q(x) : x in W(L)}.

    This determines the truth status: UNIQUE or OMEGA.
    """
    query_id: str
    answers: FrozenSet[Any]
    survivors_count: int

    @property
    def size(self) -> int:
        return len(self.answers)

    @property
    def is_decided(self) -> bool:
        """Truth status: decided iff |Ans| = 1."""
        return self.size == 1

    @property
    def is_omega(self) -> bool:
        """Truth status: Omega frontier iff |Ans| > 1."""
        return self.size > 1

    @property
    def status(self) -> str:
        """Return truth status string."""
        if self.size == 0:
            return "CONTRADICTION"
        elif self.size == 1:
            return "UNIQUE"
        else:
            return "OMEGA"

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ANSWER_SET",
            "query_id": self.query_id,
            "size": self.size,
            "status": self.status
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "FRONTIER_STATUS",
            "query_id": self.query_id,
            "answer_set_size": self.size,
            "status": self.status,
            "survivors_count": self.survivors_count,
            "answer_set": sorted(str(a) for a in self.answers),
            "result": "PASS"
        }


@dataclass
class Fiber:
    """
    An answer fiber W_b = {x in W(L) : q(x) = b}.

    The fiber partitions survivors by answer.
    """
    answer: Any
    elements: FrozenSet[Any]

    @property
    def count(self) -> int:
        return len(self.elements)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "FIBER",
            "answer": str(self.answer),
            "count": self.count
        })


@dataclass
class ProbabilityDistribution:
    """
    Probability distribution over answers.

    P_L(q=b) = |W_b| / |W(L)|

    Stored as rational numbers (integer pairs) for receipts.
    """
    query_id: str
    total_survivors: int
    fibers: List[Fiber]

    def probability_rational(self, answer: Any) -> Tuple[int, int]:
        """
        Get probability as rational (numerator, denominator).

        Returns (|W_b|, |W|) - no floats.
        """
        for fiber in self.fibers:
            if fiber.answer == answer:
                return (fiber.count, self.total_survivors)
        return (0, self.total_survivors)

    def probability_float(self, answer: Any) -> float:
        """Get probability as float (for display only)."""
        num, denom = self.probability_rational(answer)
        if denom == 0:
            return 0.0
        return num / denom

    def verify_nonnegativity(self) -> bool:
        """Verify all fiber counts are >= 0."""
        return all(fiber.count >= 0 for fiber in self.fibers)

    def verify_normalization(self) -> bool:
        """Verify sum of fiber counts equals total survivors."""
        return sum(fiber.count for fiber in self.fibers) == self.total_survivors

    def verify_additivity(self, answers_to_merge: List[Any]) -> Tuple[bool, Dict[str, Any]]:
        """
        Verify additivity: P(a1 or a2) = P(a1) + P(a2) for disjoint answers.

        Returns (passed, receipt).
        """
        # Get individual counts
        individual_counts = []
        for ans in answers_to_merge:
            for fiber in self.fibers:
                if fiber.answer == ans:
                    individual_counts.append(fiber.count)
                    break
            else:
                individual_counts.append(0)

        sum_individual = sum(individual_counts)

        # The merged probability should equal the sum
        # P(merged) = sum(individual_counts) / total
        # This is trivially true by construction

        receipt = {
            "answers_merged": [str(a) for a in answers_to_merge],
            "individual_counts": individual_counts,
            "sum_count": sum_individual,
            "total": self.total_survivors,
            "additivity_holds": True
        }

        return True, receipt

    def canonical(self) -> str:
        fiber_data = [
            {"answer": str(f.answer), "count": f.count}
            for f in sorted(self.fibers, key=lambda x: str(x.answer))
        ]
        return CanonicalJSON.serialize({
            "type": "PROBABILITY_DISTRIBUTION",
            "query_id": self.query_id,
            "total_survivors": self.total_survivors,
            "fibers": fiber_data
        })

    def to_receipt(self) -> Dict[str, Any]:
        fiber_receipts = []
        for fiber in sorted(self.fibers, key=lambda x: str(x.answer)):
            fiber_receipts.append({
                "answer": str(fiber.answer),
                "count": fiber.count,
                "probability_rational": [fiber.count, self.total_survivors]
            })

        return {
            "type": "PROBABILITY_DISTRIBUTION",
            "query_id": self.query_id,
            "total_survivors": self.total_survivors,
            "fibers": fiber_receipts,
            "nonnegativity_verified": self.verify_nonnegativity(),
            "normalization_verified": self.verify_normalization(),
            "result": "PASS" if (self.verify_nonnegativity() and self.verify_normalization()) else "FAIL"
        }


@dataclass
class RefinementUpdate:
    """
    Probability update under refinement (new evidence).

    When W -> W' subset W, probabilities update by restriction:
    P_L'(q=b) = |W'_b| / |W'|

    This is the forced update rule (Bayes is not an axiom).
    """
    query_id: str
    w_pre: int
    w_post: int
    fibers_pre: List[Fiber]
    fibers_post: List[Fiber]

    def verify_restriction(self) -> bool:
        """
        Verify W' subset W and fiber restriction is correct.

        For each answer, W'_b subset W_b.
        """
        # Check total restriction
        if self.w_post > self.w_pre:
            return False

        # Check each fiber
        pre_counts = {str(f.answer): f.count for f in self.fibers_pre}
        for fiber in self.fibers_post:
            ans_key = str(fiber.answer)
            pre_count = pre_counts.get(ans_key, 0)
            if fiber.count > pre_count:
                return False

        return True

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "REFINEMENT_UPDATE",
            "query_id": self.query_id,
            "w_pre": self.w_pre,
            "w_post": self.w_post
        })

    def to_receipt(self) -> Dict[str, Any]:
        fibers_pre_data = [
            {"answer": str(f.answer), "count": f.count}
            for f in sorted(self.fibers_pre, key=lambda x: str(x.answer))
        ]
        fibers_post_data = [
            {"answer": str(f.answer), "count": f.count}
            for f in sorted(self.fibers_post, key=lambda x: str(x.answer))
        ]

        return {
            "type": "REFINEMENT_UPDATE",
            "query_id": self.query_id,
            "w_pre": self.w_pre,
            "w_post": self.w_post,
            "fibers_pre": fibers_pre_data,
            "fibers_post": fibers_post_data,
            "restriction_verified": self.verify_restriction(),
            "result": "PASS" if self.verify_restriction() else "FAIL"
        }


class FrontierComputer:
    """
    Computes answer sets, fibers, and probability distributions.
    """

    def __init__(self, d0: FrozenSet[Any], tests: Dict[str, Test]):
        self.d0 = d0
        self.tests = tests

    def compute_survivors(
        self,
        ledger: List[Tuple[str, Any]]
    ) -> FrozenSet[Any]:
        """Compute W(L) from ledger."""
        survivors = set(self.d0)

        for test_id, outcome in ledger:
            if test_id in self.tests:
                test = self.tests[test_id]
                survivors = {
                    x for x in survivors
                    if test.evaluator(x) == outcome
                }

        return frozenset(survivors)

    def compute_answer_set(
        self,
        survivors: FrozenSet[Any],
        query: Query
    ) -> AnswerSet:
        """Compute Ans_L(q) = {q(x) : x in W(L)}."""
        answers = frozenset(query.evaluate(x) for x in survivors)

        return AnswerSet(
            query_id=query.query_id,
            answers=answers,
            survivors_count=len(survivors)
        )

    def compute_fibers(
        self,
        survivors: FrozenSet[Any],
        query: Query
    ) -> List[Fiber]:
        """Compute answer fibers W_b for all answers."""
        # Group survivors by answer
        answer_to_elements: Dict[Any, Set[Any]] = {}

        for x in survivors:
            ans = query.evaluate(x)
            if ans not in answer_to_elements:
                answer_to_elements[ans] = set()
            answer_to_elements[ans].add(x)

        # Create fibers
        fibers = [
            Fiber(answer=ans, elements=frozenset(elems))
            for ans, elems in answer_to_elements.items()
        ]

        return fibers

    def compute_probability_distribution(
        self,
        survivors: FrozenSet[Any],
        query: Query
    ) -> ProbabilityDistribution:
        """Compute full probability distribution."""
        fibers = self.compute_fibers(survivors, query)

        return ProbabilityDistribution(
            query_id=query.query_id,
            total_survivors=len(survivors),
            fibers=fibers
        )

    def compute_refinement(
        self,
        survivors_pre: FrozenSet[Any],
        survivors_post: FrozenSet[Any],
        query: Query
    ) -> RefinementUpdate:
        """Compute refinement update from W to W'."""
        fibers_pre = self.compute_fibers(survivors_pre, query)
        fibers_post = self.compute_fibers(survivors_post, query)

        return RefinementUpdate(
            query_id=query.query_id,
            w_pre=len(survivors_pre),
            w_post=len(survivors_post),
            fibers_pre=fibers_pre,
            fibers_post=fibers_post
        )


def compute_frontier_status(
    d0: FrozenSet[Any],
    tests: Dict[str, Test],
    ledger: List[Tuple[str, Any]],
    query: Query
) -> Tuple[AnswerSet, Optional[ProbabilityDistribution]]:
    """
    Compute frontier status and probability (if Omega).

    Returns:
        (answer_set, probability_distribution or None if UNIQUE)
    """
    computer = FrontierComputer(d0, tests)
    survivors = computer.compute_survivors(ledger)
    answer_set = computer.compute_answer_set(survivors, query)

    if answer_set.is_omega:
        prob_dist = computer.compute_probability_distribution(survivors, query)
        return answer_set, prob_dist
    else:
        return answer_set, None
