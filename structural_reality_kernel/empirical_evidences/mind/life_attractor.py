"""
life_attractor.py - Life as boundary-flow fixed points.

Implements:
- Life as macro-attractor with boundary maintenance
- Coarse-graining maps defining macrostates
- Replication morphism
- Variation and selection
- Viability predicate
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

from .boundary_flow import (
    RationalValue, WorldState, SubsystemCut, JoinMultiplicity,
    BoundaryFlow, BoundaryEpisode, compute_join_multiplicity
)


class ViabilityStatus(Enum):
    """Status of viability check."""
    PASS = "PASS"
    FAIL = "FAIL"


@dataclass
class CoarseGraining:
    """
    A coarse-graining map R that defines macrostates.

    R: W_S -> M (macrostate space)

    Defined by tests available to the subsystem.
    """
    map_id: str
    micro_to_macro: Dict[Any, str]  # microstate -> macrostate

    def apply(self, microstate: Any) -> str:
        """Apply coarse-graining to get macrostate."""
        return self.micro_to_macro.get(microstate, "UNKNOWN")

    def apply_to_set(self, microstates: FrozenSet[Any]) -> FrozenSet[str]:
        """Apply coarse-graining to a set of microstates."""
        return frozenset(self.apply(m) for m in microstates)

    def macrostates(self) -> Set[str]:
        """Get all possible macrostates."""
        return set(self.micro_to_macro.values())

    def canonical(self) -> str:
        sorted_map = sorted(
            [(str(k), v) for k, v in self.micro_to_macro.items()],
            key=lambda x: x[0]
        )
        return CanonicalJSON.serialize({
            "type": "COARSE_GRAINING",
            "map_id": self.map_id,
            "micro_to_macro": sorted_map
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class Macrostate:
    """
    A macrostate m_t = R(W_S(t)).

    The coarse-grained state of the subsystem.
    """
    state_id: str
    value: str  # The macrostate label
    time_step: int

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "MACROSTATE",
            "state_id": self.state_id,
            "value": self.value,
            "time_step": self.time_step
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class AttractorRegion:
    """
    An attractor region A - the set of viable macrostates.

    A subsystem is "alive" if m_t remains in A over time.
    """
    attractor_id: str
    viable_macrostates: FrozenSet[str]

    def contains(self, macrostate: str) -> bool:
        """Check if macrostate is in the attractor."""
        return macrostate in self.viable_macrostates

    def size(self) -> int:
        """Number of viable macrostates."""
        return len(self.viable_macrostates)

    def canonical(self) -> str:
        sorted_states = sorted(self.viable_macrostates)
        return CanonicalJSON.serialize({
            "type": "ATTRACTOR_REGION",
            "attractor_id": self.attractor_id,
            "viable_macrostates": sorted_states
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class AttractorStability:
    """
    Record of macrostate trajectory stability.

    Tracks how many steps the system stays in attractor A.
    """
    attractor: AttractorRegion
    trajectory: List[Macrostate]
    horizon: int

    def steps_in_attractor(self) -> int:
        """Count steps where macrostate is in attractor."""
        return sum(1 for m in self.trajectory if self.attractor.contains(m.value))

    def stability_ratio(self) -> RationalValue:
        """Ratio of steps in attractor to total steps."""
        total = len(self.trajectory)
        if total == 0:
            return RationalValue(0, 1)
        in_attractor = self.steps_in_attractor()
        return RationalValue(in_attractor, total)

    def is_stable(self, threshold: RationalValue = RationalValue(1, 2)) -> bool:
        """Check if stability ratio exceeds threshold."""
        return self.stability_ratio() >= threshold

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ATTRACTOR_STABILITY",
            "attractor_id": self.attractor.attractor_id,
            "trajectory_length": len(self.trajectory),
            "steps_in_attractor": self.steps_in_attractor(),
            "horizon": self.horizon
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        ratio = self.stability_ratio()
        return {
            "type": "ATTRACTOR_STABILITY",
            "attractor_id": self.attractor.attractor_id,
            "horizon_steps": self.horizon,
            "steps_in_attractor": self.steps_in_attractor(),
            "stability_ratio_numerator": ratio.fraction.numerator,
            "stability_ratio_denominator": ratio.fraction.denominator,
            "stable": self.is_stable(),
            "result": "PASS" if self.is_stable() else "FAIL"
        }


# ============================================================
# REPLICATION
# ============================================================

@dataclass
class Instance:
    """
    An instance of a living system.

    Has both micro and macro state.
    """
    instance_id: str
    microstate: Any
    macrostate: str
    generation: int = 0

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "INSTANCE",
            "instance_id": self.instance_id,
            "microstate": str(self.microstate),
            "macrostate": self.macrostate,
            "generation": self.generation
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class ReplicationEvent:
    """
    A replication event: Rep: A -> A x A.

    Takes one instance and produces two instances in the attractor.
    """
    event_id: str
    input_instance: Instance
    output_instances: Tuple[Instance, Instance]
    cost: int  # Energy cost of replication
    boundary_flow: RationalValue  # Delta_T^Gamma for this event

    def input_in_attractor(self, attractor: AttractorRegion) -> bool:
        """Check if input instance is in attractor."""
        return attractor.contains(self.input_instance.macrostate)

    def outputs_in_attractor(self, attractor: AttractorRegion) -> bool:
        """Check if both output instances are in attractor."""
        return (attractor.contains(self.output_instances[0].macrostate) and
                attractor.contains(self.output_instances[1].macrostate))

    def is_valid_replication(self, attractor: AttractorRegion) -> bool:
        """Check if replication is valid (preserves attractor membership)."""
        return self.input_in_attractor(attractor) and self.outputs_in_attractor(attractor)

    def boundary_flow_positive(self) -> bool:
        """Check if replication required positive boundary flow."""
        return self.boundary_flow.is_positive()

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "REPLICATION_EVENT",
            "event_id": self.event_id,
            "input_id": self.input_instance.instance_id,
            "output_1_id": self.output_instances[0].instance_id,
            "output_2_id": self.output_instances[1].instance_id,
            "cost": self.cost,
            "boundary_flow_num": self.boundary_flow.fraction.numerator,
            "boundary_flow_den": self.boundary_flow.fraction.denominator
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self, attractor: AttractorRegion) -> Dict[str, Any]:
        return {
            "type": "REPLICATION_WITNESS",
            "input_macrostate": self.input_instance.macrostate,
            "output_macrostates": [
                self.output_instances[0].macrostate,
                self.output_instances[1].macrostate
            ],
            "boundary_flow_positive": self.boundary_flow_positive(),
            "cost": self.cost,
            "valid_replication": self.is_valid_replication(attractor),
            "witness_hash": self.fingerprint()[:32],
            "result": "PASS" if self.is_valid_replication(attractor) else "FAIL"
        }


# ============================================================
# VARIATION
# ============================================================

@dataclass
class VariationWitness:
    """
    Witness that two instances differ at micro level but share macroclass.

    This demonstrates unavoidable variation in replication.
    """
    witness_id: str
    instance_1: Instance
    instance_2: Instance
    separating_test_id: str  # Test that distinguishes at micro level
    coarse_graining: CoarseGraining

    def micro_different(self) -> bool:
        """Check if instances differ at micro level."""
        return self.instance_1.microstate != self.instance_2.microstate

    def macro_same(self) -> bool:
        """Check if instances have same macrostate."""
        m1 = self.coarse_graining.apply(self.instance_1.microstate)
        m2 = self.coarse_graining.apply(self.instance_2.microstate)
        return m1 == m2

    def is_valid_variation(self) -> bool:
        """Check if this is valid variation (micro different, macro same)."""
        return self.micro_different() and self.macro_same()

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "VARIATION_WITNESS",
            "witness_id": self.witness_id,
            "instance_1_id": self.instance_1.instance_id,
            "instance_2_id": self.instance_2.instance_id,
            "separating_test": self.separating_test_id
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "VARIATION_WITNESS",
            "instance_1_hash": self.instance_1.fingerprint()[:32],
            "instance_2_hash": self.instance_2.fingerprint()[:32],
            "micro_different": self.micro_different(),
            "macro_same": self.macro_same(),
            "separating_test_id": self.separating_test_id,
            "valid_variation": self.is_valid_variation(),
            "result": "PASS" if self.is_valid_variation() else "FAIL"
        }


# ============================================================
# SELECTION
# ============================================================

@dataclass
class ViabilityVerifier:
    """
    A total viability verifier.

    Viable_H(instance) in {PASS, FAIL}

    Checks if instance stays in attractor A for H steps.
    """
    verifier_id: str
    attractor: AttractorRegion
    horizon: int
    # Dynamics: how instance evolves (microstate -> microstate)
    dynamics: Dict[Any, Any]
    coarse_graining: CoarseGraining

    def evolve(self, microstate: Any) -> Any:
        """Evolve microstate one step."""
        return self.dynamics.get(microstate, microstate)

    def verify(self, instance: Instance) -> ViabilityStatus:
        """
        Check if instance remains viable for horizon steps.

        Total: always returns PASS or FAIL.
        """
        current_micro = instance.microstate

        for step in range(self.horizon):
            macro = self.coarse_graining.apply(current_micro)
            if not self.attractor.contains(macro):
                return ViabilityStatus.FAIL
            current_micro = self.evolve(current_micro)

        return ViabilityStatus.PASS

    def is_total(self) -> bool:
        """Verifier is total by construction (always returns PASS or FAIL)."""
        return True

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "VIABILITY_VERIFIER",
            "verifier_id": self.verifier_id,
            "attractor_id": self.attractor.attractor_id,
            "horizon": self.horizon
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class SelectionResult:
    """
    Result of selection on a population.

    survivors = {x : Viable_H(x) = PASS}
    """
    result_id: str
    initial_population: List[Instance]
    survivors: List[Instance]
    eliminated: List[Instance]
    verifier: ViabilityVerifier

    def survival_ratio(self) -> RationalValue:
        """Ratio of survivors to initial population."""
        total = len(self.initial_population)
        if total == 0:
            return RationalValue(0, 1)
        return RationalValue(len(self.survivors), total)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "SELECTION_RESULT",
            "result_id": self.result_id,
            "initial_count": len(self.initial_population),
            "survivor_count": len(self.survivors),
            "eliminated_count": len(self.eliminated)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SELECTION_VERIFIER",
            "verifier_id": self.verifier.verifier_id,
            "horizon": self.verifier.horizon,
            "total_instances": len(self.initial_population),
            "passed_instances": len(self.survivors),
            "failed_instances": len(self.eliminated),
            "verifier_total": self.verifier.is_total(),
            "result": "PASS"
        }


def run_selection(
    population: List[Instance],
    verifier: ViabilityVerifier
) -> SelectionResult:
    """
    Run selection on a population.

    Filter by viability verifier.
    """
    survivors = []
    eliminated = []

    for instance in population:
        status = verifier.verify(instance)
        if status == ViabilityStatus.PASS:
            survivors.append(instance)
        else:
            eliminated.append(instance)

    return SelectionResult(
        result_id=f"SELECTION_{verifier.verifier_id}",
        initial_population=population,
        survivors=survivors,
        eliminated=eliminated,
        verifier=verifier
    )


# ============================================================
# LIFE DEFINITION
# ============================================================

@dataclass
class LifeDefinition:
    """
    Complete definition of life as boundary-flow fixed point.

    Life = stable macro-attractor + positive boundary flow.
    """
    definition_id: str
    attractor: AttractorRegion
    coarse_graining: CoarseGraining
    boundary_flow: BoundaryFlow
    stability: AttractorStability

    def is_persistent(self) -> bool:
        """Check if macrostate is persistent (stable in attractor)."""
        return self.stability.is_stable()

    def has_boundary_maintenance(self) -> bool:
        """Check if boundary flow is positive."""
        return self.boundary_flow.is_life_sustaining()

    def is_alive(self) -> bool:
        """
        A subsystem is alive if:
        1. Persistence: stable macrostate in attractor
        2. Boundary maintenance: positive boundary flow
        """
        return self.is_persistent() and self.has_boundary_maintenance()

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "LIFE_DEFINITION",
            "definition_id": self.definition_id,
            "attractor_id": self.attractor.attractor_id,
            "persistent": self.is_persistent(),
            "boundary_maintained": self.has_boundary_maintenance(),
            "alive": self.is_alive()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "LIFE_DEFINITION",
            "definition_id": self.definition_id,
            "persistent": self.is_persistent(),
            "boundary_maintained": self.has_boundary_maintenance(),
            "alive": self.is_alive(),
            "definition_hash": self.fingerprint()[:32],
            "result": "PASS" if self.is_alive() else "FAIL"
        }


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def create_sample_coarse_graining(microstates: Set[Any]) -> CoarseGraining:
    """
    Create sample coarse-graining for demonstration.

    Groups microstates into macro classes based on properties.
    """
    micro_to_macro = {}

    for micro in microstates:
        if isinstance(micro, int):
            # Group by range
            if micro < 3:
                micro_to_macro[micro] = "LOW"
            elif micro < 6:
                micro_to_macro[micro] = "MEDIUM"
            else:
                micro_to_macro[micro] = "HIGH"
        else:
            micro_to_macro[micro] = "DEFAULT"

    return CoarseGraining(
        map_id="SAMPLE_COARSE",
        micro_to_macro=micro_to_macro
    )


def create_sample_attractor(coarse_graining: CoarseGraining) -> AttractorRegion:
    """
    Create sample attractor region.

    Includes subset of macrostates as "viable".
    """
    all_macros = coarse_graining.macrostates()
    # Take first few as viable
    viable = frozenset(sorted(all_macros)[:2])  # e.g., LOW and MEDIUM

    return AttractorRegion(
        attractor_id="SAMPLE_ATTRACTOR",
        viable_macrostates=viable
    )


def create_sample_instances(
    microstates: Set[Any],
    coarse_graining: CoarseGraining,
    count: int
) -> List[Instance]:
    """Create sample instances from microstates."""
    instances = []
    for i, micro in enumerate(sorted(microstates, key=str)[:count]):
        macro = coarse_graining.apply(micro)
        instances.append(Instance(
            instance_id=f"INSTANCE_{i}",
            microstate=micro,
            macrostate=macro,
            generation=0
        ))
    return instances


def create_sample_viability_verifier(
    attractor: AttractorRegion,
    coarse_graining: CoarseGraining,
    microstates: Set[Any],
    horizon: int = 5
) -> ViabilityVerifier:
    """Create sample viability verifier."""
    # Simple dynamics: identity (stable)
    dynamics = {m: m for m in microstates}

    return ViabilityVerifier(
        verifier_id="SAMPLE_VERIFIER",
        attractor=attractor,
        horizon=horizon,
        dynamics=dynamics,
        coarse_graining=coarse_graining
    )


def create_replication_event(
    parent: Instance,
    coarse_graining: CoarseGraining,
    variation_factor: int = 1
) -> ReplicationEvent:
    """
    Create a replication event with variation.

    Produces two offspring with slight micro-level variation.
    """
    parent_micro = parent.microstate

    # Create offspring with variation
    if isinstance(parent_micro, int):
        child_1_micro = parent_micro
        child_2_micro = parent_micro + variation_factor
    else:
        child_1_micro = parent_micro
        child_2_micro = str(parent_micro) + "_VAR"

    child_1_macro = coarse_graining.apply(child_1_micro)
    child_2_macro = coarse_graining.apply(child_2_micro)

    child_1 = Instance(
        instance_id=f"{parent.instance_id}_C1",
        microstate=child_1_micro,
        macrostate=child_1_macro,
        generation=parent.generation + 1
    )

    child_2 = Instance(
        instance_id=f"{parent.instance_id}_C2",
        microstate=child_2_micro,
        macrostate=child_2_macro,
        generation=parent.generation + 1
    )

    return ReplicationEvent(
        event_id=f"REP_{parent.instance_id}",
        input_instance=parent,
        output_instances=(child_1, child_2),
        cost=10,  # Replication has cost
        boundary_flow=RationalValue(2, 1)  # Positive boundary flow
    )
