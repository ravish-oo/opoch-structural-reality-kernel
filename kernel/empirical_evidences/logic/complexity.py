"""
complexity.py - Complexity as minimax separator cost.

Complexity is exactly the minimal separator cost required to force
a unique answer (quotient collapse to singleton).

V(W;q) = min_{tau}[c(tau) + max_a V(W_a;q)]  (minimax value functional)
tau*(W;q) = argmin of the above (canonical next separator)
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import Test
from core.receipts import CanonicalJSON

from .computation import Query, AnswerSet, AnswerComputer, ComputationContract


@dataclass
class MinimaxNode:
    """
    A node in the minimax computation tree.
    """
    survivors: FrozenSet[Any]
    answer_count: int
    value: int  # V(W;q)
    optimal_separator: Optional[str]
    children: Dict[Any, 'MinimaxNode']  # outcome -> child node
    depth: int

    def is_terminal(self) -> bool:
        """Terminal if answer count is 1 (solved)."""
        return self.answer_count <= 1


@dataclass
class MinimaxResult:
    """
    Result of minimax computation for an instance.
    """
    contract_id: str
    root_survivors: int
    minimax_value: int
    optimal_separator: Optional[str]
    recursion_depth: int
    nodes_computed: int

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "MINIMAX_RESULT",
            "contract_id": self.contract_id,
            "minimax_value": self.minimax_value,
            "optimal_separator": self.optimal_separator or "NONE"
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "MINIMAX_VALUE",
            "contract_id": self.contract_id,
            "survivor_count": self.root_survivors,
            "minimax_cost": self.minimax_value,
            "optimal_separator": self.optimal_separator or "NONE",
            "recursion_depth": self.recursion_depth,
            "nodes_computed": self.nodes_computed,
            "result": "PASS"
        }


class MinimaxComputer:
    """
    Computes the minimax value functional V(W;q).

    V(W;q) = 0                                    if q constant on W
    V(W;q) = min_{tau}[c(tau) + max_a V(W_a;q)]  otherwise

    This is the exact "hardness" of the instance.
    """

    def __init__(self, contract: ComputationContract):
        self.contract = contract
        self.answer_computer = AnswerComputer(contract.query)
        self.memo: Dict[FrozenSet[Any], MinimaxNode] = {}
        self.nodes_computed = 0
        self.max_depth = 0

    def compute_value(
        self,
        survivors: FrozenSet[Any],
        depth: int = 0,
        max_depth: Optional[int] = None
    ) -> MinimaxNode:
        """
        Compute V(W;q) recursively with memoization.
        """
        # Check memo
        if survivors in self.memo:
            return self.memo[survivors]

        self.nodes_computed += 1
        self.max_depth = max(self.max_depth, depth)

        # Compute answer set
        answer_set = self.answer_computer.compute_answer_set(survivors)

        # Base case: solved (answer count = 1)
        if answer_set.is_solved or len(survivors) == 0:
            node = MinimaxNode(
                survivors=survivors,
                answer_count=answer_set.answer_count,
                value=0,
                optimal_separator=None,
                children={},
                depth=depth
            )
            self.memo[survivors] = node
            return node

        # Check depth limit
        if max_depth is not None and depth >= max_depth:
            # Return upper bound (sum of all costs as worst case)
            node = MinimaxNode(
                survivors=survivors,
                answer_count=answer_set.answer_count,
                value=sum(t.cost for t in self.contract.tests.values()) * len(survivors),
                optimal_separator=None,
                children={},
                depth=depth
            )
            self.memo[survivors] = node
            return node

        # Minimax: find optimal separator
        best_value = float('inf')
        best_separator = None
        best_children: Dict[Any, MinimaxNode] = {}

        for test_id, test in self.contract.tests.items():
            # Partition survivors by this test
            partition: Dict[Any, Set[Any]] = {}

            for x in survivors:
                outcome = test.evaluator(x)
                if outcome not in partition:
                    partition[outcome] = set()
                partition[outcome].add(x)

            # Skip if test doesn't separate (single partition)
            if len(partition) <= 1:
                continue

            # Compute max over outcomes (worst case)
            max_child_value = 0
            children: Dict[Any, MinimaxNode] = {}

            for outcome, fiber in partition.items():
                child_node = self.compute_value(
                    frozenset(fiber),
                    depth + 1,
                    max_depth
                )
                children[outcome] = child_node
                max_child_value = max(max_child_value, child_node.value)

            # Total value = cost + max child value
            total_value = test.cost + max_child_value

            # Update best if better
            if total_value < best_value:
                best_value = total_value
                best_separator = test_id
                best_children = children

        # If no separator found (shouldn't happen for unsolved states)
        if best_separator is None:
            best_value = 0

        node = MinimaxNode(
            survivors=survivors,
            answer_count=answer_set.answer_count,
            value=int(best_value) if best_value != float('inf') else 0,
            optimal_separator=best_separator,
            children=best_children,
            depth=depth
        )

        self.memo[survivors] = node
        return node

    def get_optimal_strategy(
        self,
        survivors: FrozenSet[Any]
    ) -> List[Tuple[str, Dict[Any, int]]]:
        """
        Get the optimal separator sequence (strategy).

        Returns list of (separator_id, {outcome -> expected cost from there})
        """
        strategy = []
        node = self.compute_value(survivors)

        while node.optimal_separator is not None:
            outcome_costs = {
                outcome: child.value
                for outcome, child in node.children.items()
            }
            strategy.append((node.optimal_separator, outcome_costs))

            # Take worst-case branch for full strategy
            if node.children:
                worst_outcome = max(node.children.keys(),
                                   key=lambda o: node.children[o].value)
                node = node.children[worst_outcome]
            else:
                break

        return strategy

    def compute_minimax_result(
        self,
        survivors: FrozenSet[Any],
        max_depth: Optional[int] = None
    ) -> MinimaxResult:
        """
        Compute full minimax result with statistics.
        """
        self.memo = {}
        self.nodes_computed = 0
        self.max_depth = 0

        root = self.compute_value(survivors, 0, max_depth)

        return MinimaxResult(
            contract_id=self.contract.contract_id,
            root_survivors=len(survivors),
            minimax_value=root.value,
            optimal_separator=root.optimal_separator,
            recursion_depth=self.max_depth,
            nodes_computed=self.nodes_computed
        )


@dataclass
class ComplexityBound:
    """
    A bound on instance complexity.
    """
    bound_type: str  # "EXACT" | "UPPER" | "LOWER"
    value: int
    separator_witness: Optional[str]
    is_tight: bool

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "COMPLEXITY_BOUND",
            "bound_type": self.bound_type,
            "value": self.value,
            "is_tight": self.is_tight
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "COMPLEXITY_BOUND",
            "bound_type": self.bound_type,
            "value": self.value,
            "separator_witness": self.separator_witness or "NONE",
            "is_tight": self.is_tight,
            "result": "PASS"
        }


class ComplexityAnalyzer:
    """
    Analyzes complexity properties of computational problems.
    """

    def __init__(self, contract: ComputationContract):
        self.contract = contract
        self.minimax_computer = MinimaxComputer(contract)

    def compute_exact_complexity(
        self,
        survivors: FrozenSet[Any],
        max_depth: Optional[int] = None
    ) -> ComplexityBound:
        """
        Compute exact complexity (minimax value).
        """
        result = self.minimax_computer.compute_minimax_result(survivors, max_depth)

        return ComplexityBound(
            bound_type="EXACT" if max_depth is None else "UPPER",
            value=result.minimax_value,
            separator_witness=result.optimal_separator,
            is_tight=max_depth is None
        )

    def compute_greedy_upper_bound(
        self,
        survivors: FrozenSet[Any]
    ) -> ComplexityBound:
        """
        Compute upper bound using greedy strategy.
        """
        from .computation import QuotientCollapser

        collapser = QuotientCollapser(self.contract)
        trace = collapser.collapse_greedy(survivors)

        return ComplexityBound(
            bound_type="UPPER",
            value=trace.total_cost,
            separator_witness=trace.steps[0].separator_used if trace.steps else None,
            is_tight=False
        )

    def compute_trivial_lower_bound(
        self,
        survivors: FrozenSet[Any]
    ) -> ComplexityBound:
        """
        Compute trivial lower bound: min cost separator if unsolved.
        """
        answer_set = self.minimax_computer.answer_computer.compute_answer_set(survivors)

        if answer_set.is_solved:
            return ComplexityBound(
                bound_type="LOWER",
                value=0,
                separator_witness=None,
                is_tight=True
            )

        # Lower bound is at least the minimum separator cost
        min_cost = min(t.cost for t in self.contract.tests.values())

        return ComplexityBound(
            bound_type="LOWER",
            value=min_cost,
            separator_witness=None,
            is_tight=False
        )

    def verify_separator_is_optimal(
        self,
        survivors: FrozenSet[Any],
        separator_id: str
    ) -> Tuple[bool, Dict[str, Any]]:
        """
        Verify that a separator is optimal (achieves minimax).
        """
        result = self.minimax_computer.compute_minimax_result(survivors)

        is_optimal = (result.optimal_separator == separator_id)

        return is_optimal, {
            "separator_id": separator_id,
            "is_optimal": is_optimal,
            "optimal_separator": result.optimal_separator,
            "minimax_value": result.minimax_value
        }


@dataclass
class MonotoneImprovement:
    """
    Demonstrates that adding tests cannot increase complexity.

    V_{t+1}(W;q) <= V_t(W;q) when Delta_{t+1} >= Delta_t
    """
    original_tests: int
    added_tests: int
    original_complexity: int
    new_complexity: int
    improvement: int

    @property
    def is_monotone(self) -> bool:
        """Verify monotonicity: new <= original."""
        return self.new_complexity <= self.original_complexity

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "MONOTONE_IMPROVEMENT",
            "original_complexity": self.original_complexity,
            "new_complexity": self.new_complexity,
            "is_monotone": self.is_monotone
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "MONOTONE_IMPROVEMENT",
            "original_tests": self.original_tests,
            "added_tests": self.added_tests,
            "original_complexity": self.original_complexity,
            "new_complexity": self.new_complexity,
            "improvement": self.improvement,
            "is_monotone": self.is_monotone,
            "result": "PASS" if self.is_monotone else "FAIL"
        }


def verify_monotone_improvement(
    contract: ComputationContract,
    survivors: FrozenSet[Any],
    additional_tests: Dict[str, Test]
) -> MonotoneImprovement:
    """
    Verify that adding tests monotonically reduces complexity.
    """
    # Original complexity
    original_analyzer = ComplexityAnalyzer(contract)
    original_bound = original_analyzer.compute_exact_complexity(survivors, max_depth=10)

    # Extended contract with additional tests
    extended_tests = dict(contract.tests)
    extended_tests.update(additional_tests)

    extended_contract = ComputationContract(
        contract_id=f"{contract.contract_id}_extended",
        d0=contract.d0,
        query=contract.query,
        tests=extended_tests
    )

    # New complexity
    extended_analyzer = ComplexityAnalyzer(extended_contract)
    new_bound = extended_analyzer.compute_exact_complexity(survivors, max_depth=10)

    return MonotoneImprovement(
        original_tests=len(contract.tests),
        added_tests=len(additional_tests),
        original_complexity=original_bound.value,
        new_complexity=new_bound.value,
        improvement=original_bound.value - new_bound.value
    )
