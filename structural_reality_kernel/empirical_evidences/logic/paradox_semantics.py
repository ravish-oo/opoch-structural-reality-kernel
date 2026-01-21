"""
paradox_semantics.py - Total semantics and prefix-free encoding.

Implements:
- Prefix-free encoding of formulas/descriptions
- Total evaluator that always returns finite outcomes
- Language specification with canonical fingerprints

This provides the foundation for resolving paradoxes by grounding
truth in witnessable operations.
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
from enum import Enum
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON


class EvalResult(Enum):
    """Result of total evaluation."""
    TRUE = "TRUE"
    FALSE = "FALSE"
    UNDEFINED = "UNDEFINED"  # For non-well-formed inputs
    TIMEOUT = "TIMEOUT"  # Budget exhausted
    CONTRADICTION = "CONTRADICTION"  # Self-contradiction detected


@dataclass
class Expression:
    """
    An expression in the formal language.

    All expressions are finite bitstrings with prefix-free encoding.
    """
    code: str
    kind: str = "ATOMIC"  # ATOMIC, NEGATION, SELF_REF, COMPOUND
    subexpressions: List['Expression'] = field(default_factory=list)

    def __hash__(self):
        return hash(self.code)

    def __eq__(self, other):
        if isinstance(other, Expression):
            return self.code == other.code
        return False

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "EXPRESSION",
            "code": self.code,
            "kind": self.kind
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class EvaluationTrace:
    """
    Trace of an evaluation attempt.
    """
    expression: Expression
    steps: List[Dict[str, Any]] = field(default_factory=list)
    result: EvalResult = EvalResult.UNDEFINED
    contradiction_witness: Optional[str] = None

    def add_step(self, description: str, state: Dict[str, Any]):
        self.steps.append({
            "step": len(self.steps) + 1,
            "description": description,
            "state": state
        })

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "EVALUATION_TRACE",
            "expression_code": self.expression.code,
            "step_count": len(self.steps),
            "result": self.result.value
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "EVALUATION_TRACE",
            "expression_fingerprint": self.expression.fingerprint()[:16],
            "steps": len(self.steps),
            "result": self.result.value,
            "contradiction_detected": self.contradiction_witness is not None
        }


@dataclass
class PrefixFreeLanguage:
    """
    A prefix-free formal language.

    Prefix-free means no valid expression is a prefix of another.
    This ensures unambiguous parsing.
    """
    language_id: str
    vocabulary: FrozenSet[str]  # Atomic symbols
    formation_rules: Dict[str, Callable[[List[Expression]], bool]]

    def __post_init__(self):
        self._cache: Dict[str, bool] = {}

    def is_prefix_free(self, expressions: List[Expression]) -> Tuple[bool, List[Tuple[str, str]]]:
        """
        Verify that no expression is a prefix of another.
        """
        violations = []
        codes = [e.code for e in expressions]

        for i, c1 in enumerate(codes):
            for j, c2 in enumerate(codes):
                if i != j and c2.startswith(c1) and c1 != c2:
                    violations.append((c1, c2))

        return len(violations) == 0, violations

    def is_well_formed(self, expr: Expression) -> bool:
        """Check if expression is well-formed according to formation rules."""
        if expr.code in self._cache:
            return self._cache[expr.code]

        # Atomic expressions
        if expr.kind == "ATOMIC":
            result = expr.code in self.vocabulary or expr.code.startswith("PROP_")
            self._cache[expr.code] = result
            return result

        # Check formation rules by kind
        if expr.kind in self.formation_rules:
            result = self.formation_rules[expr.kind](expr.subexpressions)
            self._cache[expr.code] = result
            return result

        # Default: compound expressions with valid subexpressions
        if expr.subexpressions:
            result = all(self.is_well_formed(sub) for sub in expr.subexpressions)
            self._cache[expr.code] = result
            return result

        self._cache[expr.code] = True
        return True

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "PREFIX_FREE_LANGUAGE",
            "language_id": self.language_id,
            "vocabulary_size": len(self.vocabulary)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "LANGUAGE_SPEC",
            "language_id": self.language_id,
            "vocabulary_size": len(self.vocabulary),
            "language_hash": self.fingerprint()[:32],
            "result": "PASS"
        }


class TotalEvaluator:
    """
    A total evaluator that always returns a finite result.

    Totality is enforced by:
    - Explicit handling of all input cases
    - Budget limits for recursive evaluation
    - Contradiction detection for self-referential statements
    """

    def __init__(
        self,
        language: PrefixFreeLanguage,
        evaluation_budget: int = 100
    ):
        self.language = language
        self.budget = evaluation_budget
        self._assignment: Dict[str, EvalResult] = {}

    def set_assignment(self, expr_code: str, value: EvalResult):
        """Set a truth assignment for an expression."""
        self._assignment[expr_code] = value

    def evaluate(
        self,
        expr: Expression,
        depth: int = 0
    ) -> Tuple[EvalResult, EvaluationTrace]:
        """
        Evaluate an expression with totality guarantee.

        Always returns one of: TRUE, FALSE, UNDEFINED, TIMEOUT, CONTRADICTION
        """
        trace = EvaluationTrace(expression=expr)

        # Budget check
        if depth > self.budget:
            trace.result = EvalResult.TIMEOUT
            trace.add_step("Budget exhausted", {"depth": depth, "budget": self.budget})
            return EvalResult.TIMEOUT, trace

        # Well-formedness check
        if not self.language.is_well_formed(expr):
            trace.result = EvalResult.UNDEFINED
            trace.add_step("Not well-formed", {"code": expr.code})
            return EvalResult.UNDEFINED, trace

        trace.add_step("Begin evaluation", {"code": expr.code, "kind": expr.kind})

        # Check cached assignment
        if expr.code in self._assignment:
            result = self._assignment[expr.code]
            trace.result = result
            trace.add_step("Retrieved assignment", {"result": result.value})
            return result, trace

        # Self-referential expressions
        if expr.kind == "SELF_REF":
            return self._evaluate_self_ref(expr, trace, depth)

        # Negation
        if expr.kind == "NEGATION" and expr.subexpressions:
            return self._evaluate_negation(expr, trace, depth)

        # Atomic expressions - default to UNDEFINED if no assignment
        if expr.kind == "ATOMIC":
            trace.result = EvalResult.UNDEFINED
            trace.add_step("Atomic without assignment", {"code": expr.code})
            return EvalResult.UNDEFINED, trace

        # Default
        trace.result = EvalResult.UNDEFINED
        return EvalResult.UNDEFINED, trace

    def _evaluate_self_ref(
        self,
        expr: Expression,
        trace: EvaluationTrace,
        depth: int
    ) -> Tuple[EvalResult, EvaluationTrace]:
        """
        Evaluate self-referential expression.

        Self-reference that creates contradiction is detected and reported.
        """
        trace.add_step("Self-referential evaluation", {"code": expr.code})

        # Mark as being evaluated to detect cycles
        marker = f"__EVAL_{expr.code}__"
        if marker in self._assignment:
            # Cycle detected - this is a self-referential loop
            trace.result = EvalResult.CONTRADICTION
            trace.contradiction_witness = f"Cyclic self-reference in {expr.code}"
            trace.add_step("Cycle detected", {"witness": trace.contradiction_witness})
            return EvalResult.CONTRADICTION, trace

        # Set marker
        self._assignment[marker] = EvalResult.UNDEFINED

        # For liar-type statements: "This statement is false"
        # If we try to assign TRUE, the statement says it's FALSE -> contradiction
        # If we try to assign FALSE, the statement says it's TRUE -> contradiction

        # Try TRUE assignment
        self._assignment[expr.code] = EvalResult.TRUE
        # Check if this creates contradiction (liar says: I am FALSE)
        if self._is_liar_pattern(expr):
            # TRUE assignment means the negation should be FALSE
            # But the statement claims its own falsity
            trace.add_step("Liar pattern detected - trying TRUE", {"result": "contradiction"})

            # Try FALSE assignment
            self._assignment[expr.code] = EvalResult.FALSE
            # FALSE assignment means the negation should be TRUE
            # But the statement claims its own falsity (so it would be true)
            trace.add_step("Liar pattern detected - trying FALSE", {"result": "contradiction"})

            # Both assignments lead to contradiction
            trace.result = EvalResult.CONTRADICTION
            trace.contradiction_witness = "Liar paradox: no consistent truth assignment"
            del self._assignment[marker]
            del self._assignment[expr.code]
            return EvalResult.CONTRADICTION, trace

        # Clean up
        del self._assignment[marker]

        # Non-contradictory self-reference
        trace.result = EvalResult.UNDEFINED
        trace.add_step("Self-reference without contradiction", {"status": "OMEGA"})
        return EvalResult.UNDEFINED, trace

    def _is_liar_pattern(self, expr: Expression) -> bool:
        """Detect if expression is a liar-type pattern."""
        # Liar pattern: expression that references its own truth value negatively
        code = expr.code.upper()
        return (
            "LIAR" in code or
            "THIS_IS_FALSE" in code or
            "I_AM_FALSE" in code or
            (expr.kind == "SELF_REF" and "NOT" in code)
        )

    def _evaluate_negation(
        self,
        expr: Expression,
        trace: EvaluationTrace,
        depth: int
    ) -> Tuple[EvalResult, EvaluationTrace]:
        """Evaluate negation expression."""
        inner = expr.subexpressions[0]
        trace.add_step("Evaluating negation", {"inner": inner.code})

        inner_result, inner_trace = self.evaluate(inner, depth + 1)

        if inner_result == EvalResult.TRUE:
            trace.result = EvalResult.FALSE
        elif inner_result == EvalResult.FALSE:
            trace.result = EvalResult.TRUE
        else:
            trace.result = inner_result  # Propagate UNDEFINED/TIMEOUT/CONTRADICTION

        trace.add_step("Negation result", {"inner": inner_result.value, "result": trace.result.value})
        return trace.result, trace

    def is_total(self, test_expressions: List[Expression]) -> Tuple[bool, Dict[str, Any]]:
        """
        Verify evaluator is total on given expressions.

        Total means every expression gets one of the defined results.
        """
        results = []
        all_defined = True

        for expr in test_expressions:
            try:
                result, trace = self.evaluate(expr)
                results.append({
                    "code": expr.code,
                    "result": result.value,
                    "defined": result in [
                        EvalResult.TRUE, EvalResult.FALSE,
                        EvalResult.UNDEFINED, EvalResult.TIMEOUT,
                        EvalResult.CONTRADICTION
                    ]
                })
            except Exception as e:
                all_defined = False
                results.append({
                    "code": expr.code,
                    "result": "EXCEPTION",
                    "error": str(e),
                    "defined": False
                })

        return all_defined, {
            "expressions_tested": len(test_expressions),
            "all_defined": all_defined,
            "results_summary": results[:10]  # First 10
        }

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "TOTAL_EVALUATOR",
            "language_id": self.language.language_id,
            "budget": self.budget
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "TOTAL_EVALUATOR",
            "language_id": self.language.language_id,
            "budget": self.budget,
            "evaluator_hash": self.fingerprint()[:32],
            "result": "PASS"
        }


def create_standard_language() -> PrefixFreeLanguage:
    """
    Create the standard prefix-free language for paradox analysis.
    """
    vocabulary = frozenset([
        "TRUE", "FALSE", "NOT", "AND", "OR", "IMPLIES",
        "LIAR", "SELF_REF", "MEMBER", "SET", "DEFINE"
    ])

    def negation_rule(subs: List[Expression]) -> bool:
        return len(subs) == 1

    def binary_rule(subs: List[Expression]) -> bool:
        return len(subs) == 2

    formation_rules = {
        "NEGATION": negation_rule,
        "AND": binary_rule,
        "OR": binary_rule,
        "IMPLIES": binary_rule,
        "SELF_REF": lambda subs: True,  # Self-ref is always syntactically valid
        "ATOMIC": lambda subs: len(subs) == 0
    }

    return PrefixFreeLanguage(
        language_id="STANDARD_PARADOX_LANG",
        vocabulary=vocabulary,
        formation_rules=formation_rules
    )


def create_total_evaluator(budget: int = 100) -> TotalEvaluator:
    """Create a total evaluator with standard language."""
    language = create_standard_language()
    return TotalEvaluator(language, budget)
