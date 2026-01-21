"""
intelligence.py - Intelligence as refinement efficiency.

Implements:
- Intelligence metrics (chi, p)
- Refinement per observation (chi = Delta_K / Delta_T)
- Refinement per energy (p = Delta_K / Delta_E)
- Thought waste analysis
- Viability-constrained optimization
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
from .life_attractor import AttractorRegion, CoarseGraining, Instance


@dataclass
class ModelSpace:
    """
    The subsystem's model space - what it can distinguish.

    K = log|Q_Delta(W_S)| = feasible distinguishability
    """
    model_id: str
    states: FrozenSet[Any]
    quotient_classes: List[FrozenSet[Any]]  # Equivalence classes under feasible tests

    def distinguishability(self) -> int:
        """
        K = |Q_Delta(W_S)| - number of distinguishable classes.

        We use count (not log) for exact arithmetic.
        """
        return len(self.quotient_classes)

    def log_distinguishability(self) -> RationalValue:
        """K as rational (count representation)."""
        return RationalValue(self.distinguishability(), 1)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "MODEL_SPACE",
            "model_id": self.model_id,
            "state_count": len(self.states),
            "class_count": self.distinguishability()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class IntelligenceStep:
    """
    A single step in the intelligence measurement.

    Records Delta_K, Delta_T, Delta_E for one observation.
    """
    step_id: str
    time_step: int

    # Changes during this step
    delta_k: RationalValue  # Change in distinguishability
    delta_t: RationalValue  # Time increment (survivor ratio)
    delta_e: int  # Energy cost

    # State tracking
    viable: bool  # Whether system remained in attractor

    @property
    def chi(self) -> RationalValue:
        """
        Refinement per observation: chi = Delta_K / Delta_T

        Intelligence metric 1.
        """
        if self.delta_t.fraction == 0:
            return RationalValue(0, 1)
        return self.delta_k / self.delta_t

    @property
    def p(self) -> RationalValue:
        """
        Refinement per energy: p = Delta_K / Delta_E

        Intelligence metric 2.
        """
        if self.delta_e == 0:
            return RationalValue(0, 1)
        return RationalValue(
            self.delta_k.fraction.numerator,
            self.delta_k.fraction.denominator * self.delta_e
        )

    def is_efficient(self) -> bool:
        """Check if step was efficient (positive chi and p while viable)."""
        return (self.chi.is_positive() and
                self.p.is_positive() and
                self.viable)

    def is_thought_waste(self) -> bool:
        """
        Check if this step was wasted thought.

        Thought waste: Delta_T > 0 and Delta_K ~ 0
        """
        return (self.delta_t.is_positive() and
                not self.delta_k.is_positive())

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "INTELLIGENCE_STEP",
            "step_id": self.step_id,
            "time_step": self.time_step,
            "delta_k_num": self.delta_k.fraction.numerator,
            "delta_k_den": self.delta_k.fraction.denominator,
            "delta_t_num": self.delta_t.fraction.numerator,
            "delta_t_den": self.delta_t.fraction.denominator,
            "delta_e": self.delta_e,
            "viable": self.viable
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        chi = self.chi
        p = self.p
        return {
            "type": "INTELLIGENCE_METRICS",
            "step_id": self.step_id,
            "delta_k_numerator": self.delta_k.fraction.numerator,
            "delta_k_denominator": self.delta_k.fraction.denominator,
            "delta_t_numerator": self.delta_t.fraction.numerator,
            "delta_t_denominator": self.delta_t.fraction.denominator,
            "delta_e": self.delta_e,
            "chi_numerator": chi.fraction.numerator,
            "chi_denominator": chi.fraction.denominator,
            "p_numerator": p.fraction.numerator,
            "p_denominator": p.fraction.denominator,
            "viable": self.viable,
            "efficient": self.is_efficient(),
            "thought_waste": self.is_thought_waste(),
            "result": "PASS"
        }


@dataclass
class IntelligenceTrace:
    """
    Complete trace of intelligence measurements over time.

    Tracks chi and p across multiple steps.
    """
    trace_id: str
    steps: List[IntelligenceStep]
    attractor: AttractorRegion

    def total_delta_k(self) -> RationalValue:
        """Total refinement gained."""
        total = RationalValue(0, 1)
        for step in self.steps:
            total = total + step.delta_k
        return total

    def total_delta_t(self) -> RationalValue:
        """Total time spent."""
        total = RationalValue(0, 1)
        for step in self.steps:
            total = total + step.delta_t
        return total

    def total_delta_e(self) -> int:
        """Total energy spent."""
        return sum(step.delta_e for step in self.steps)

    def average_chi(self) -> RationalValue:
        """Average refinement per observation."""
        total_t = self.total_delta_t()
        if total_t.fraction == 0:
            return RationalValue(0, 1)
        return self.total_delta_k() / total_t

    def average_p(self) -> RationalValue:
        """Average refinement per energy."""
        total_e = self.total_delta_e()
        if total_e == 0:
            return RationalValue(0, 1)
        total_k = self.total_delta_k()
        return RationalValue(
            total_k.fraction.numerator,
            total_k.fraction.denominator * total_e
        )

    def viability_ratio(self) -> RationalValue:
        """Ratio of viable steps."""
        if not self.steps:
            return RationalValue(0, 1)
        viable_count = sum(1 for s in self.steps if s.viable)
        return RationalValue(viable_count, len(self.steps))

    def thought_waste_ratio(self) -> RationalValue:
        """Ratio of thought-waste steps."""
        if not self.steps:
            return RationalValue(0, 1)
        waste_count = sum(1 for s in self.steps if s.is_thought_waste())
        return RationalValue(waste_count, len(self.steps))

    def is_intelligent(self) -> bool:
        """
        Check if trace demonstrates intelligence.

        Intelligent = high chi, high p, while remaining viable.
        """
        return (self.average_chi().is_positive() and
                self.average_p().is_positive() and
                self.viability_ratio() >= RationalValue(1, 2))

    def canonical(self) -> str:
        avg_chi = self.average_chi()
        avg_p = self.average_p()
        return CanonicalJSON.serialize({
            "type": "INTELLIGENCE_TRACE",
            "trace_id": self.trace_id,
            "step_count": len(self.steps),
            "total_k_num": self.total_delta_k().fraction.numerator,
            "total_k_den": self.total_delta_k().fraction.denominator,
            "avg_chi_num": avg_chi.fraction.numerator,
            "avg_chi_den": avg_chi.fraction.denominator,
            "avg_p_num": avg_p.fraction.numerator,
            "avg_p_den": avg_p.fraction.denominator
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        avg_chi = self.average_chi()
        avg_p = self.average_p()
        viability = self.viability_ratio()
        waste = self.thought_waste_ratio()
        return {
            "type": "INTELLIGENCE_TRACE",
            "trace_id": self.trace_id,
            "step_count": len(self.steps),
            "total_delta_k_num": self.total_delta_k().fraction.numerator,
            "total_delta_k_den": self.total_delta_k().fraction.denominator,
            "total_delta_e": self.total_delta_e(),
            "avg_chi_num": avg_chi.fraction.numerator,
            "avg_chi_den": avg_chi.fraction.denominator,
            "avg_p_num": avg_p.fraction.numerator,
            "avg_p_den": avg_p.fraction.denominator,
            "viability_ratio_num": viability.fraction.numerator,
            "viability_ratio_den": viability.fraction.denominator,
            "thought_waste_ratio_num": waste.fraction.numerator,
            "thought_waste_ratio_den": waste.fraction.denominator,
            "intelligent": self.is_intelligent(),
            "result": "PASS" if self.is_intelligent() else "FAIL"
        }


# ============================================================
# THOUGHT WASTE ANALYSIS
# ============================================================

@dataclass
class ThoughtWasteAnalysis:
    """
    Analysis of thought waste in an intelligence trace.

    Thought waste: Delta_T > 0 and Delta_K ~ 0
    """
    analysis_id: str
    trace: IntelligenceTrace

    def wasted_steps(self) -> List[IntelligenceStep]:
        """Get steps that represent thought waste."""
        return [s for s in self.trace.steps if s.is_thought_waste()]

    def efficient_steps(self) -> List[IntelligenceStep]:
        """Get steps that were efficient."""
        return [s for s in self.trace.steps if s.is_efficient()]

    def wasted_time(self) -> RationalValue:
        """Total time wasted on non-productive thought."""
        total = RationalValue(0, 1)
        for step in self.wasted_steps():
            total = total + step.delta_t
        return total

    def wasted_energy(self) -> int:
        """Total energy wasted on non-productive thought."""
        return sum(step.delta_e for step in self.wasted_steps())

    def efficiency_ratio(self) -> RationalValue:
        """Ratio of efficient to total steps."""
        if not self.trace.steps:
            return RationalValue(0, 1)
        return RationalValue(len(self.efficient_steps()), len(self.trace.steps))

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "THOUGHT_WASTE_ANALYSIS",
            "analysis_id": self.analysis_id,
            "total_steps": len(self.trace.steps),
            "wasted_steps": len(self.wasted_steps()),
            "efficient_steps": len(self.efficient_steps())
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        wasted_t = self.wasted_time()
        eff = self.efficiency_ratio()
        return {
            "type": "THOUGHT_WASTE_ANALYSIS",
            "analysis_id": self.analysis_id,
            "total_steps": len(self.trace.steps),
            "wasted_steps": len(self.wasted_steps()),
            "efficient_steps": len(self.efficient_steps()),
            "wasted_time_num": wasted_t.fraction.numerator,
            "wasted_time_den": wasted_t.fraction.denominator,
            "wasted_energy": self.wasted_energy(),
            "efficiency_ratio_num": eff.fraction.numerator,
            "efficiency_ratio_den": eff.fraction.denominator,
            "result": "PASS"
        }


# ============================================================
# INTELLIGENCE MEASUREMENT
# ============================================================

def measure_intelligence_step(
    model_before: ModelSpace,
    model_after: ModelSpace,
    survivors_before: int,
    survivors_after: int,
    energy_cost: int,
    attractor: AttractorRegion,
    current_macrostate: str,
    step_id: str,
    time_step: int
) -> IntelligenceStep:
    """
    Measure intelligence metrics for a single step.

    Computes Delta_K, Delta_T, Delta_E and checks viability.
    """
    # Delta_K = change in distinguishability
    k_before = model_before.distinguishability()
    k_after = model_after.distinguishability()
    delta_k = RationalValue(k_after - k_before, 1)

    # Delta_T = log(|W|/|W'|) represented as ratio
    if survivors_after == 0:
        delta_t = RationalValue(survivors_before, 1)
    else:
        delta_t = RationalValue(survivors_before, survivors_after)

    # Viability check
    viable = attractor.contains(current_macrostate)

    return IntelligenceStep(
        step_id=step_id,
        time_step=time_step,
        delta_k=delta_k,
        delta_t=delta_t,
        delta_e=energy_cost,
        viable=viable
    )


def create_intelligence_trace(
    steps: List[IntelligenceStep],
    attractor: AttractorRegion,
    trace_id: str = "TRACE"
) -> IntelligenceTrace:
    """Create an intelligence trace from steps."""
    return IntelligenceTrace(
        trace_id=trace_id,
        steps=steps,
        attractor=attractor
    )


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def create_sample_model_space(
    microstates: Set[Any],
    coarse_graining: CoarseGraining
) -> ModelSpace:
    """Create sample model space from microstates and coarse-graining."""
    # Group microstates by macrostate
    macro_to_micros: Dict[str, Set[Any]] = {}
    for micro in microstates:
        macro = coarse_graining.apply(micro)
        if macro not in macro_to_micros:
            macro_to_micros[macro] = set()
        macro_to_micros[macro].add(micro)

    quotient_classes = [frozenset(micros) for micros in macro_to_micros.values()]

    return ModelSpace(
        model_id="SAMPLE_MODEL",
        states=frozenset(microstates),
        quotient_classes=quotient_classes
    )


def create_refined_model_space(
    model: ModelSpace,
    additional_test: Dict[Any, Any]
) -> ModelSpace:
    """
    Create refined model space by adding a new test.

    This increases distinguishability (splits some classes).
    """
    # Split classes based on new test outcomes
    new_classes = []

    for eq_class in model.quotient_classes:
        # Group by test outcome
        outcome_to_states: Dict[Any, Set[Any]] = {}
        for state in eq_class:
            outcome = additional_test.get(state, "DEFAULT")
            if outcome not in outcome_to_states:
                outcome_to_states[outcome] = set()
            outcome_to_states[outcome].add(state)

        # Create new classes from splits
        for states in outcome_to_states.values():
            new_classes.append(frozenset(states))

    return ModelSpace(
        model_id=f"{model.model_id}_REFINED",
        states=model.states,
        quotient_classes=new_classes
    )


def create_sample_intelligence_trace(
    microstates: Set[Any],
    coarse_graining: CoarseGraining,
    attractor: AttractorRegion,
    num_steps: int = 5
) -> IntelligenceTrace:
    """Create sample intelligence trace for demonstration."""
    steps = []
    current_model = create_sample_model_space(microstates, coarse_graining)

    for i in range(num_steps):
        # Simulate refinement: add a test that splits some classes
        test = {m: m % (i + 2) for m in microstates if isinstance(m, int)}
        refined_model = create_refined_model_space(current_model, test)

        # Simulate survivor shrinkage
        survivors_before = len(microstates) - i
        survivors_after = max(1, len(microstates) - i - 1)

        # Energy cost
        energy = 5

        # Current macrostate (arbitrary for demo)
        current_macro = list(attractor.viable_macrostates)[0] if attractor.viable_macrostates else "DEFAULT"

        step = measure_intelligence_step(
            model_before=current_model,
            model_after=refined_model,
            survivors_before=survivors_before,
            survivors_after=survivors_after,
            energy_cost=energy,
            attractor=attractor,
            current_macrostate=current_macro,
            step_id=f"STEP_{i}",
            time_step=i
        )

        steps.append(step)
        current_model = refined_model

    return IntelligenceTrace(
        trace_id="SAMPLE_TRACE",
        steps=steps,
        attractor=attractor
    )
