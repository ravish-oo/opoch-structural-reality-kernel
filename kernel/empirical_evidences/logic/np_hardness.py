"""
np_hardness.py - NP and NP-hardness as separator theorems.

NP is a witness contract: polynomial verifier, polynomial witness bound.
NP-hardness is the existence of instance families where any separator
sequence has worst-case cost growing exponentially.
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
from itertools import product

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import Test
from core.receipts import CanonicalJSON

from .computation import Query, ComputationContract, AnswerComputer


@dataclass
class WitnessContract:
    """
    An NP witness contract.

    Specifies:
    - verifier V(x, w) -> PASS/FAIL
    - witness bound |w| <= p(|x|)
    - instance x
    """
    contract_id: str
    instance: Any
    witness_space: FrozenSet[Any]
    verifier: Callable[[Any], bool]  # verifier(w) for fixed instance

    @property
    def witness_count(self) -> int:
        return len(self.witness_space)

    def verify_witness(self, w: Any) -> bool:
        """Check if w is a valid witness."""
        return self.verifier(w)

    def find_witness(self) -> Optional[Any]:
        """Search for a satisfying witness (brute force)."""
        for w in self.witness_space:
            if self.verifier(w):
                return w
        return None

    def count_witnesses(self) -> int:
        """Count satisfying witnesses."""
        return sum(1 for w in self.witness_space if self.verifier(w))

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "WITNESS_CONTRACT",
            "contract_id": self.contract_id,
            "witness_space_size": self.witness_count
        })

    def to_receipt(self) -> Dict[str, Any]:
        satisfying = self.count_witnesses()
        return {
            "type": "WITNESS_CONTRACT",
            "contract_id": self.contract_id,
            "witness_space_size": self.witness_count,
            "satisfying_witnesses": satisfying,
            "is_satisfiable": satisfying > 0,
            "result": "PASS"
        }


@dataclass
class NPDecision:
    """
    The decision for an NP instance.

    Either:
    - UNIQUE + witness (if SAT)
    - UNIQUE + exhaustive proof (if UNSAT)
    - OMEGA + frontier (if stopped early)
    """
    decision_type: str  # "SAT" | "UNSAT" | "OMEGA"
    witness: Optional[Any]
    witnesses_checked: int
    total_witnesses: int

    @property
    def is_complete(self) -> bool:
        """Is the decision complete (not Omega)?"""
        return self.decision_type != "OMEGA"

    @property
    def coverage_ratio(self) -> float:
        """Fraction of witness space checked."""
        if self.total_witnesses == 0:
            return 1.0
        return self.witnesses_checked / self.total_witnesses

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "NP_DECISION",
            "decision_type": self.decision_type,
            "is_complete": self.is_complete
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "NP_DECISION",
            "decision_type": self.decision_type,
            "witness": str(self.witness) if self.witness else "NONE",
            "witnesses_checked": self.witnesses_checked,
            "total_witnesses": self.total_witnesses,
            "coverage_ratio_display": str(round(self.coverage_ratio, 6)),
            "is_complete": self.is_complete,
            "result": "PASS" if self.is_complete else "OMEGA"
        }


@dataclass
class LowerBoundInstance:
    """
    An instance in a lower-bound family.

    For adversarial instances, each test eliminates at most k witnesses.
    """
    instance_id: str
    size_parameter: int  # n
    witness_space_size: int  # 2^n for SAT
    max_elimination_per_test: int  # k
    worst_case_steps: int  # ceil(2^n / k)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "LOWER_BOUND_INSTANCE",
            "instance_id": self.instance_id,
            "size_parameter": self.size_parameter,
            "worst_case_steps": self.worst_case_steps
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "LOWER_BOUND_INSTANCE",
            "instance_id": self.instance_id,
            "size_parameter": self.size_parameter,
            "witness_space_size": self.witness_space_size,
            "max_elimination_per_test": self.max_elimination_per_test,
            "worst_case_steps": self.worst_case_steps,
            "result": "PASS"
        }


@dataclass
class LowerBoundFamily:
    """
    A family of lower-bound instances (NP-hard witness).

    Demonstrates that for some instance families, no cheap separator
    sequence exists under the given test algebra.
    """
    family_name: str
    instances: List[LowerBoundInstance]
    growth_type: str  # "EXPONENTIAL" | "POLYNOMIAL"

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "LOWER_BOUND_FAMILY",
            "family_name": self.family_name,
            "instance_count": len(self.instances),
            "growth_type": self.growth_type
        })

    def to_receipt(self) -> Dict[str, Any]:
        if self.instances:
            sizes = [i.size_parameter for i in self.instances]
            steps = [i.worst_case_steps for i in self.instances]
        else:
            sizes = []
            steps = []

        return {
            "type": "LOWER_BOUND_WITNESS",
            "family_name": self.family_name,
            "instance_count": len(self.instances),
            "size_parameters": sizes[:5],
            "worst_case_steps": steps[:5],
            "growth_type": self.growth_type,
            "elimination_per_test": self.instances[0].max_elimination_per_test if self.instances else 1,
            "result": "PASS"
        }


class SATInstance:
    """
    A SAT instance as a witness contract.

    For n variables:
    - witness space: {0,1}^n
    - verifier: evaluate formula
    """

    def __init__(self, n: int, clauses: List[List[int]]):
        """
        Create SAT instance.

        clauses: list of clauses, each clause is list of literals
                 positive = variable, negative = negated variable
        """
        self.n = n
        self.clauses = clauses
        self.witness_space = self._generate_witness_space()

    def _generate_witness_space(self) -> FrozenSet[Tuple[int, ...]]:
        """Generate all possible assignments."""
        return frozenset(product([0, 1], repeat=self.n))

    def evaluate(self, assignment: Tuple[int, ...]) -> bool:
        """Evaluate formula on assignment."""
        for clause in self.clauses:
            clause_satisfied = False
            for lit in clause:
                var_idx = abs(lit) - 1
                if var_idx >= len(assignment):
                    continue
                val = assignment[var_idx]
                if (lit > 0 and val == 1) or (lit < 0 and val == 0):
                    clause_satisfied = True
                    break
            if not clause_satisfied:
                return False
        return True

    def to_witness_contract(self, contract_id: str) -> WitnessContract:
        """Convert to witness contract."""
        return WitnessContract(
            contract_id=contract_id,
            instance=self.clauses,
            witness_space=self.witness_space,
            verifier=self.evaluate
        )


def create_unique_sat_instance(n: int, target_assignment: Tuple[int, ...]) -> SATInstance:
    """
    Create a SAT instance with exactly one satisfying assignment.

    F_a = AND_{i=1}^n l_i where l_i = x_i if a_i=1 else NOT x_i

    This is the adversarial family for lower bounds.
    """
    clauses = []
    for i, val in enumerate(target_assignment):
        if val == 1:
            clauses.append([i + 1])  # x_i
        else:
            clauses.append([-(i + 1)])  # NOT x_i

    return SATInstance(n, clauses)


def create_lower_bound_family(
    family_name: str,
    sizes: List[int],
    elimination_per_test: int = 1
) -> LowerBoundFamily:
    """
    Create a lower-bound family demonstrating exponential growth.

    For each size n:
    - witness space has 2^n elements
    - each test eliminates at most elimination_per_test witnesses
    - worst case requires 2^n / elimination_per_test steps
    """
    instances = []

    for n in sizes:
        witness_space_size = 2 ** n
        worst_case = (witness_space_size + elimination_per_test - 1) // elimination_per_test

        instances.append(LowerBoundInstance(
            instance_id=f"{family_name}_n{n}",
            size_parameter=n,
            witness_space_size=witness_space_size,
            max_elimination_per_test=elimination_per_test,
            worst_case_steps=worst_case
        ))

    return LowerBoundFamily(
        family_name=family_name,
        instances=instances,
        growth_type="EXPONENTIAL"
    )


class NPSolver:
    """
    Solves NP instances by quotient collapse.
    """

    def __init__(self, contract: WitnessContract):
        self.contract = contract

    def solve_exhaustive(self, budget: Optional[int] = None) -> NPDecision:
        """
        Solve by exhaustive search (brute force).
        """
        checked = 0
        total = self.contract.witness_count

        for w in self.contract.witness_space:
            if budget is not None and checked >= budget:
                return NPDecision(
                    decision_type="OMEGA",
                    witness=None,
                    witnesses_checked=checked,
                    total_witnesses=total
                )

            if self.contract.verifier(w):
                return NPDecision(
                    decision_type="SAT",
                    witness=w,
                    witnesses_checked=checked + 1,
                    total_witnesses=total
                )

            checked += 1

        return NPDecision(
            decision_type="UNSAT",
            witness=None,
            witnesses_checked=checked,
            total_witnesses=total
        )

    def solve_with_separator(
        self,
        separator: Callable[[Any], Any],
        target_outcome: Any
    ) -> Tuple[FrozenSet[Any], int]:
        """
        Apply a separator and return the filtered witness space.
        """
        filtered = set()
        eliminated = 0

        for w in self.contract.witness_space:
            if separator(w) == target_outcome:
                filtered.add(w)
            else:
                eliminated += 1

        return frozenset(filtered), eliminated


@dataclass
class SeparatorEffectiveness:
    """
    Measures how effective a separator is for an NP instance.
    """
    separator_id: str
    outcomes: Dict[Any, int]  # outcome -> count of witnesses
    max_elimination: int  # max witnesses eliminated by any outcome
    min_remaining: int  # min witnesses remaining after any outcome

    @property
    def is_effective(self) -> bool:
        """Is this separator effective (eliminates at least half)?"""
        total = sum(self.outcomes.values())
        return self.max_elimination >= total // 2

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "SEPARATOR_EFFECTIVENESS",
            "separator_id": self.separator_id,
            "max_elimination": self.max_elimination,
            "is_effective": self.is_effective
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SEPARATOR_EFFECTIVENESS",
            "separator_id": self.separator_id,
            "outcome_count": len(self.outcomes),
            "max_elimination": self.max_elimination,
            "min_remaining": self.min_remaining,
            "is_effective": self.is_effective,
            "result": "PASS"
        }


def analyze_separator_effectiveness(
    contract: WitnessContract,
    separator: Callable[[Any], Any],
    separator_id: str
) -> SeparatorEffectiveness:
    """
    Analyze how effective a separator is for the witness space.
    """
    outcomes: Dict[Any, int] = {}

    for w in contract.witness_space:
        outcome = separator(w)
        outcomes[outcome] = outcomes.get(outcome, 0) + 1

    total = sum(outcomes.values())
    max_elimination = max(total - count for count in outcomes.values())
    min_remaining = min(outcomes.values())

    return SeparatorEffectiveness(
        separator_id=separator_id,
        outcomes=outcomes,
        max_elimination=max_elimination,
        min_remaining=min_remaining
    )


def create_computation_contract_from_np(
    contract: WitnessContract,
    tests: Dict[str, Test]
) -> ComputationContract:
    """
    Convert an NP witness contract to a computation contract.
    """
    # Query: is witness satisfying?
    query = Query(
        query_id=f"sat_{contract.contract_id}",
        evaluator=lambda w: "SAT" if contract.verifier(w) else "UNSAT",
        answer_space=frozenset(["SAT", "UNSAT"])
    )

    return ComputationContract(
        contract_id=contract.contract_id,
        d0=contract.witness_space,
        query=query,
        tests=tests
    )
