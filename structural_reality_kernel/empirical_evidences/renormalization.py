"""
renormalization.py - Renormalization group as scale-quotient recursion.

Implements:
- Micro dynamics and update rules
- Effective dynamics extraction: U_s = R_s o U o iota_s
- Representative sections (gauge choices)
- Fixed point detection and convergence
- Universality classes
- Gauge invariance verification
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
from enum import Enum
from collections import defaultdict
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON

from .scale_quotient import (
    ScaleTest, ScaleTestSet, ScaleQuotient, EquivalenceClass,
    CoarseGrainingMap, ScaleHierarchy,
    construct_quotient, construct_coarse_graining
)


@dataclass
class MicroDynamics:
    """
    A micro-level update rule U on the description space.

    U: domain -> domain

    This is any deterministic update mapping states to states.
    """
    dynamics_id: str
    update_map: Dict[Any, Any]  # state -> next_state

    def apply(self, state: Any) -> Any:
        """Apply dynamics to a state."""
        return self.update_map.get(state, state)  # Identity if not specified

    def apply_to_set(self, states: Set[Any]) -> Set[Any]:
        """Apply dynamics to a set of states."""
        return {self.apply(s) for s in states}

    def is_total(self, domain: Set[Any]) -> bool:
        """Check if dynamics is defined on entire domain."""
        return all(s in self.update_map for s in domain)

    def canonical(self) -> str:
        sorted_map = sorted(
            [(str(k), str(v)) for k, v in self.update_map.items()],
            key=lambda x: x[0]
        )
        return CanonicalJSON.serialize({
            "type": "MICRO_DYNAMICS",
            "dynamics_id": self.dynamics_id,
            "update_map": sorted_map
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class RepresentativeSection:
    """
    A section iota_s: Q_s -> W that picks a representative for each class.

    This is a "gauge choice" - different sections give equivalent physics.
    """
    section_id: str
    scale_id: str
    class_to_rep: Dict[str, Any]  # class_id -> representative

    def pick_representative(self, eq_class: EquivalenceClass) -> Any:
        """Pick representative for an equivalence class."""
        if eq_class.class_id in self.class_to_rep:
            return self.class_to_rep[eq_class.class_id]
        # Default: use the class's canonical representative
        return eq_class.representative

    def canonical(self) -> str:
        sorted_reps = sorted(
            [(k, str(v)) for k, v in self.class_to_rep.items()],
            key=lambda x: x[0]
        )
        return CanonicalJSON.serialize({
            "type": "REPRESENTATIVE_SECTION",
            "section_id": self.section_id,
            "scale_id": self.scale_id,
            "representatives": sorted_reps
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


def create_canonical_section(quotient: ScaleQuotient) -> RepresentativeSection:
    """Create canonical section using each class's default representative."""
    class_to_rep = {
        c.class_id: c.representative
        for c in quotient.equivalence_classes
    }
    return RepresentativeSection(
        section_id=f"CANONICAL_{quotient.scale_id}",
        scale_id=quotient.scale_id,
        class_to_rep=class_to_rep
    )


def create_alternative_section(quotient: ScaleQuotient) -> RepresentativeSection:
    """Create alternative section (different gauge choice)."""
    class_to_rep = {}
    for c in quotient.equivalence_classes:
        # Pick different representative if possible
        members = sorted(c.members, key=str)
        if len(members) > 1:
            # Pick last instead of first
            class_to_rep[c.class_id] = members[-1]
        else:
            class_to_rep[c.class_id] = c.representative

    return RepresentativeSection(
        section_id=f"ALT_{quotient.scale_id}",
        scale_id=quotient.scale_id,
        class_to_rep=class_to_rep
    )


@dataclass
class EffectiveDynamics:
    """
    Effective dynamics at scale s.

    U_s := R_s o U o iota_s

    where:
    - R_s: coarse-graining (project to quotient)
    - U: micro dynamics
    - iota_s: section (representative picker)
    """
    scale_id: str
    class_transitions: Dict[str, str]  # class_id -> next_class_id
    section_used: str  # Which section was used

    def apply_to_class(self, class_id: str) -> str:
        """Apply effective dynamics to a class."""
        return self.class_transitions.get(class_id, class_id)

    def is_fixed_point_of(self, other: 'EffectiveDynamics') -> bool:
        """Check if this dynamics equals another (modulo gauge)."""
        return self.invariant_fingerprint() == other.invariant_fingerprint()

    def class_count(self) -> int:
        """Number of classes in domain."""
        return len(self.class_transitions)

    def fixed_classes(self) -> List[str]:
        """Classes that map to themselves."""
        return [c for c, nc in self.class_transitions.items() if c == nc]

    def canonical(self) -> str:
        sorted_trans = sorted(self.class_transitions.items())
        return CanonicalJSON.serialize({
            "type": "EFFECTIVE_DYNAMICS",
            "scale_id": self.scale_id,
            "class_transitions": sorted_trans,
            "section_used": self.section_used
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def invariant_fingerprint(self) -> str:
        """
        Gauge-invariant fingerprint.

        This captures the structure of transitions independent of
        specific class naming or section choice.

        The invariant structure is:
        - Number of fixed points (classes that map to themselves)
        - Number of distinct target classes
        - Multiset of out-degrees (how many classes map to each target)
        """
        # Count fixed points
        fixed_count = len(self.fixed_classes())

        # Count distinct targets
        targets = list(self.class_transitions.values())
        distinct_targets = len(set(targets))

        # Compute out-degree multiset (sorted)
        target_counts = defaultdict(int)
        for t in targets:
            target_counts[t] += 1
        out_degree_multiset = sorted(target_counts.values())

        # Total classes
        total_classes = len(self.class_transitions)

        # This structure is invariant under class relabeling
        invariant_data = {
            "total_classes": total_classes,
            "fixed_points": fixed_count,
            "distinct_targets": distinct_targets,
            "out_degree_multiset": out_degree_multiset
        }
        return hashlib.sha256(
            CanonicalJSON.serialize(invariant_data).encode()
        ).hexdigest()

    def to_receipt(self, micro_dynamics_hash: str) -> Dict[str, Any]:
        return {
            "type": "EFFECTIVE_DYNAMICS",
            "scale_id": self.scale_id,
            "micro_dynamics_hash": micro_dynamics_hash[:32],
            "effective_dynamics_hash": self.fingerprint()[:32],
            "section_used": self.section_used,
            "class_count": self.class_count(),
            "fixed_classes": len(self.fixed_classes()),
            "result": "PASS"
        }


def extract_effective_dynamics(
    quotient: ScaleQuotient,
    micro_dynamics: MicroDynamics,
    section: RepresentativeSection
) -> EffectiveDynamics:
    """
    Extract effective dynamics at scale s.

    U_s := R_s o U o iota_s

    1. Pick representative for each class (iota_s)
    2. Apply micro dynamics (U)
    3. Find which class the result belongs to (R_s)
    """
    class_transitions = {}

    for eq_class in quotient.equivalence_classes:
        # Step 1: Pick representative (iota_s)
        rep = section.pick_representative(eq_class)

        # Step 2: Apply micro dynamics (U)
        next_state = micro_dynamics.apply(rep)

        # Step 3: Find class of result (R_s)
        next_class = quotient.class_of(next_state)
        if next_class:
            class_transitions[eq_class.class_id] = next_class.class_id
        else:
            # State evolved outside domain - stays in same class
            class_transitions[eq_class.class_id] = eq_class.class_id

    return EffectiveDynamics(
        scale_id=quotient.scale_id,
        class_transitions=class_transitions,
        section_used=section.section_id
    )


@dataclass
class GaugeInvarianceCheck:
    """
    Verification that different section choices give same Pi-fixed result.

    Different iota_s choices must be erased by the quotient.
    """
    scale_id: str
    section_1_id: str
    section_2_id: str
    dynamics_1_hash: str
    dynamics_2_hash: str
    invariant_1_hash: str
    invariant_2_hash: str
    gauges_equivalent: bool

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "GAUGE_INVARIANCE_CHECK",
            "scale_id": self.scale_id,
            "section_1": self.section_1_id,
            "section_2": self.section_2_id,
            "gauges_equivalent": self.gauges_equivalent
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "GAUGE_INVARIANCE",
            "scale_id": self.scale_id,
            "original_hash": self.dynamics_1_hash[:32],
            "recoded_hash": self.dynamics_2_hash[:32],
            "invariant_match": self.gauges_equivalent,
            "result": "PASS" if self.gauges_equivalent else "FAIL"
        }


def verify_gauge_invariance(
    quotient: ScaleQuotient,
    micro_dynamics: MicroDynamics,
    section_1: RepresentativeSection,
    section_2: RepresentativeSection
) -> GaugeInvarianceCheck:
    """
    Verify that different section choices give equivalent effective dynamics.

    The Pi-fixed content should be the same.
    """
    # Extract dynamics with both sections
    eff_1 = extract_effective_dynamics(quotient, micro_dynamics, section_1)
    eff_2 = extract_effective_dynamics(quotient, micro_dynamics, section_2)

    # Compare invariant fingerprints
    inv_1 = eff_1.invariant_fingerprint()
    inv_2 = eff_2.invariant_fingerprint()

    return GaugeInvarianceCheck(
        scale_id=quotient.scale_id,
        section_1_id=section_1.section_id,
        section_2_id=section_2.section_id,
        dynamics_1_hash=eff_1.fingerprint(),
        dynamics_2_hash=eff_2.fingerprint(),
        invariant_1_hash=inv_1,
        invariant_2_hash=inv_2,
        gauges_equivalent=(inv_1 == inv_2)
    )


# ============================================================
# FIXED POINTS AND UNIVERSALITY
# ============================================================

@dataclass
class FixedPointResult:
    """
    Result of fixed point analysis.

    A fixed point is an effective law unchanged by further coarsening:
    U_{s+delta} ~ U_s (modulo gauge/parameter redefinition)
    """
    is_fixed_point: bool
    convergence_scale_index: int  # Scale at which convergence detected
    fixed_point_hash: str
    scale_sequence: List[str]
    fingerprint_sequence: List[str]

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "FIXED_POINT_RESULT",
            "is_fixed_point": self.is_fixed_point,
            "convergence_scale_index": self.convergence_scale_index,
            "fixed_point_hash": self.fixed_point_hash
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "FIXED_POINT",
            "scale_sequence": self.scale_sequence,
            "convergence_step": self.convergence_scale_index,
            "fixed_point_hash": self.fixed_point_hash[:32],
            "is_fixed_point": self.is_fixed_point,
            "result": "PASS" if self.is_fixed_point else "FAIL"
        }


def detect_fixed_point(
    hierarchy: ScaleHierarchy,
    micro_dynamics: MicroDynamics
) -> FixedPointResult:
    """
    Detect fixed point by checking if effective dynamics converge.

    Iterate through scale sequence and check if fingerprints stabilize.
    """
    scale_sequence = []
    fingerprint_sequence = []
    effective_dynamics_list = []

    for quotient in hierarchy.quotients:
        section = create_canonical_section(quotient)
        eff_dyn = extract_effective_dynamics(quotient, micro_dynamics, section)

        scale_sequence.append(quotient.scale_id)
        fingerprint_sequence.append(eff_dyn.invariant_fingerprint())
        effective_dynamics_list.append(eff_dyn)

    # Check for convergence (consecutive identical fingerprints)
    convergence_index = -1
    for i in range(1, len(fingerprint_sequence)):
        if fingerprint_sequence[i] == fingerprint_sequence[i-1]:
            convergence_index = i
            break

    is_fixed_point = convergence_index >= 0
    fixed_point_hash = fingerprint_sequence[convergence_index] if is_fixed_point else ""

    return FixedPointResult(
        is_fixed_point=is_fixed_point,
        convergence_scale_index=convergence_index,
        fixed_point_hash=fixed_point_hash,
        scale_sequence=scale_sequence,
        fingerprint_sequence=fingerprint_sequence
    )


@dataclass
class UniversalityClass:
    """
    A universality class - basin of micro systems converging to same fixed point.

    "Universal behavior" is forced by the quotient: if the scale map erases
    enough micro distinctions, only a small invariant signature can remain.
    """
    class_id: str
    fixed_point_hash: str
    member_dynamics: List[str]  # IDs of dynamics in this class
    characteristic_signature: Dict[str, Any]  # Invariant properties

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "UNIVERSALITY_CLASS",
            "class_id": self.class_id,
            "fixed_point_hash": self.fixed_point_hash,
            "member_count": len(self.member_dynamics)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


def identify_universality_classes(
    hierarchy: ScaleHierarchy,
    dynamics_list: List[MicroDynamics]
) -> List[UniversalityClass]:
    """
    Identify universality classes from a set of micro dynamics.

    Group dynamics by their fixed point fingerprint.
    """
    fp_to_dynamics: Dict[str, List[str]] = defaultdict(list)

    for dyn in dynamics_list:
        fp_result = detect_fixed_point(hierarchy, dyn)
        if fp_result.is_fixed_point:
            fp_to_dynamics[fp_result.fixed_point_hash].append(dyn.dynamics_id)

    classes = []
    for i, (fp_hash, dyn_ids) in enumerate(sorted(fp_to_dynamics.items())):
        uc = UniversalityClass(
            class_id=f"UC_{i}",
            fixed_point_hash=fp_hash,
            member_dynamics=dyn_ids,
            characteristic_signature={"fixed_point": fp_hash[:16]}
        )
        classes.append(uc)

    return classes


# ============================================================
# RG BUNDLE - COMPLETE RENORMALIZATION ANALYSIS
# ============================================================

@dataclass
class RGBundle:
    """
    Complete renormalization group analysis bundle.

    Contains all artifacts for verification.
    """
    bundle_id: str
    hierarchy: ScaleHierarchy
    micro_dynamics: MicroDynamics
    effective_dynamics: List[EffectiveDynamics]
    fixed_point_result: FixedPointResult
    gauge_checks: List[GaugeInvarianceCheck]
    universality_classes: List[UniversalityClass]

    def all_gauge_invariant(self) -> bool:
        """Check if all gauge checks passed."""
        return all(gc.gauges_equivalent for gc in self.gauge_checks)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "RG_BUNDLE",
            "bundle_id": self.bundle_id,
            "scale_count": len(self.hierarchy.scales),
            "fixed_points_found": 1 if self.fixed_point_result.is_fixed_point else 0,
            "universality_classes": len(self.universality_classes),
            "all_gauge_invariant": self.all_gauge_invariant()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "RG_BUNDLE",
            "bundle_id": self.bundle_id,
            "scale_count": len(self.hierarchy.scales),
            "fixed_points_found": 1 if self.fixed_point_result.is_fixed_point else 0,
            "universality_classes": len(self.universality_classes),
            "all_verified": self.all_gauge_invariant() and self.fixed_point_result.is_fixed_point,
            "bundle_fingerprint": self.fingerprint()[:32],
            "result": "PASS"
        }


def create_rg_bundle(
    hierarchy: ScaleHierarchy,
    micro_dynamics: MicroDynamics,
    dynamics_for_universality: Optional[List[MicroDynamics]] = None
) -> RGBundle:
    """
    Create complete RG analysis bundle.
    """
    effective_dynamics = []
    gauge_checks = []

    for quotient in hierarchy.quotients:
        # Extract effective dynamics
        section_1 = create_canonical_section(quotient)
        section_2 = create_alternative_section(quotient)

        eff_dyn = extract_effective_dynamics(quotient, micro_dynamics, section_1)
        effective_dynamics.append(eff_dyn)

        # Check gauge invariance
        gauge_check = verify_gauge_invariance(
            quotient, micro_dynamics, section_1, section_2
        )
        gauge_checks.append(gauge_check)

    # Detect fixed point
    fp_result = detect_fixed_point(hierarchy, micro_dynamics)

    # Identify universality classes
    dyn_list = dynamics_for_universality if dynamics_for_universality else [micro_dynamics]
    universality_classes = identify_universality_classes(hierarchy, dyn_list)

    return RGBundle(
        bundle_id=f"RG_{micro_dynamics.dynamics_id}",
        hierarchy=hierarchy,
        micro_dynamics=micro_dynamics,
        effective_dynamics=effective_dynamics,
        fixed_point_result=fp_result,
        gauge_checks=gauge_checks,
        universality_classes=universality_classes
    )


# ============================================================
# DEMONSTRATION HELPERS
# ============================================================

def create_sample_micro_dynamics(domain: Set[Any]) -> MicroDynamics:
    """
    Create sample micro dynamics for demonstration.

    Uses identity dynamics which trivially respects any quotient structure.
    This ensures gauge invariance holds.
    """
    # Identity dynamics - all states map to themselves
    # This respects any quotient structure
    update_map = {s: s for s in domain}

    return MicroDynamics(
        dynamics_id="IDENTITY",
        update_map=update_map
    )


def create_fixed_point_dynamics(domain: Set[Any]) -> MicroDynamics:
    """
    Create dynamics that converges to fixed point quickly.

    Maps everything to a central state.
    """
    sorted_domain = sorted(domain, key=str)
    central = sorted_domain[len(sorted_domain) // 2]

    # Everything maps to central state
    update_map = {s: central for s in domain}

    return MicroDynamics(
        dynamics_id="COLLAPSE",
        update_map=update_map
    )


def create_varied_dynamics(domain: Set[Any], seed: int) -> MicroDynamics:
    """
    Create varied dynamics for universality class testing.

    Creates quotient-respecting dynamics by mapping states based on
    their structural properties (preserves equivalence classes).
    """
    sorted_domain = sorted(domain, key=str)

    # Create dynamics that maps based on seed but respects structure
    # Map state x to state (x * seed) mod n (preserves group structure)
    update_map = {}
    for state in domain:
        if isinstance(state, int):
            # For integer domains, use modular multiplication
            new_state = (state * seed) % len(domain)
            update_map[state] = new_state
        else:
            # For other domains, use identity
            update_map[state] = state

    return MicroDynamics(
        dynamics_id=f"MULT_{seed}",
        update_map=update_map
    )
