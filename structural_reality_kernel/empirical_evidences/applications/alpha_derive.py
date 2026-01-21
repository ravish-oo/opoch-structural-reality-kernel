"""
alpha_derive.py - Main derivation logic for the fine-structure constant α.

Implements:
- Channel coequalization (intersection)
- Frontier handling for inconsistent channels
- Optimal separator selection (τ*)
- Complete α certified object generation
- Proof bundle assembly
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple, Union
from fractions import Fraction
from enum import Enum
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON

from .boundary_flow import RationalValue
from .alpha_contract import (
    CertifiedRationalInterval,
    WitnessBundle,
    VerifierResult,
    VerifierStatus,
    ChannelType,
    ChannelResult,
    AlphaContract,
    FrontierWitness,
    GaugeInvarianceCheck,
    check_gauge_invariance,
    create_interval
)
from .alpha_channels import (
    run_channel_a,
    run_channel_b,
    run_all_channels,
    get_channel_summary
)


class AlphaStatus(Enum):
    """Status of α derivation."""
    UNIQUE = "UNIQUE"          # All channels agree, single interval
    FRONTIER = "FRONTIER"       # Channels disagree, frontier state
    INSUFFICIENT = "INSUFFICIENT"  # Not enough precision


@dataclass
class AlphaIntersection:
    """
    Result of intersecting channel intervals.

    Represents either a unique interval (channels agree) or a frontier (channels disagree).
    """
    intersection_id: str
    channel_results: List[ChannelResult]
    final_interval: Optional[CertifiedRationalInterval]
    status: AlphaStatus
    conflict_witness: Optional[FrontierWitness]

    def is_unique(self) -> bool:
        return self.status == AlphaStatus.UNIQUE and self.final_interval is not None

    def certified_digits(self) -> int:
        if self.final_interval:
            return self.final_interval.certified_digits()
        return 0

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ALPHA_INTERSECTION",
            "intersection_id": self.intersection_id,
            "channels_count": len(self.channel_results),
            "status": self.status.value,
            "is_unique": self.is_unique()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        receipt = {
            "type": "ALPHA_INTERSECTION",
            "intersection_id": self.intersection_id,
            "channels_count": len(self.channel_results),
            "intervals_compatible": self.is_unique(),
            "frontier_status": self.status.value,
            "result": "PASS" if self.is_unique() else self.status.value
        }
        if self.final_interval:
            receipt["final_interval_lower"] = str(self.final_interval.lower.fraction)
            receipt["final_interval_upper"] = str(self.final_interval.upper.fraction)
            receipt["certified_digits"] = self.certified_digits()
        return receipt


def intersect_channels(
    channel_results: List[ChannelResult]
) -> AlphaIntersection:
    """
    Intersect intervals from all channels.

    Returns either:
    - UNIQUE: all channels overlap, intersection is the final interval
    - FRONTIER: channels don't overlap, returns conflict witness
    """
    if not channel_results:
        raise ValueError("No channel results to intersect")

    # Start with first channel's interval
    current_interval = channel_results[0].alpha_interval

    for result in channel_results[1:]:
        new_interval = current_interval.intersect(result.alpha_interval)

        if new_interval is None:
            # Channels disagree - compute conflict witness
            conflict = _compute_conflict(channel_results)
            return AlphaIntersection(
                intersection_id="ALPHA_INTERSECT",
                channel_results=channel_results,
                final_interval=None,
                status=AlphaStatus.FRONTIER,
                conflict_witness=conflict
            )

        current_interval = new_interval

    return AlphaIntersection(
        intersection_id="ALPHA_INTERSECT",
        channel_results=channel_results,
        final_interval=current_interval,
        status=AlphaStatus.UNIQUE,
        conflict_witness=None
    )


def _compute_conflict(
    channel_results: List[ChannelResult]
) -> FrontierWitness:
    """
    Compute conflict witness when channels don't overlap.

    Identifies the gap and suggests the minimal separator.
    """
    intervals = {r.channel_id: r.alpha_interval for r in channel_results}

    # Find the pair with the largest gap
    max_gap = RationalValue(0, 1)
    gap_channels = ("", "")

    channel_ids = list(intervals.keys())
    for i, c1 in enumerate(channel_ids):
        for c2 in channel_ids[i+1:]:
            i1, i2 = intervals[c1], intervals[c2]
            if not i1.overlaps(i2):
                # Compute gap
                if i1.upper < i2.lower:
                    gap = i2.lower - i1.upper
                else:
                    gap = i1.lower - i2.upper

                if gap > max_gap:
                    max_gap = gap
                    gap_channels = (c1, c2)

    # Determine minimal separator (which channel to improve)
    # Choose the channel with wider interval (more room for improvement)
    widths = {r.channel_id: r.alpha_interval.width() for r in channel_results}
    widest = max(widths.keys(), key=lambda k: widths[k].fraction)

    separator_desc = f"Improve {widest} measurement precision"

    return FrontierWitness(
        frontier_id="ALPHA_CONFLICT",
        channel_intervals=intervals,
        conflict_gap=max_gap,
        minimal_separator=separator_desc,
        separator_cost_estimate=RationalValue(1, 1)  # Normalized cost
    )


@dataclass
class OptimalSeparator:
    """
    The optimal next measurement to collapse the frontier.

    Chosen by minimax over shrink-per-cost.
    """
    separator_id: str
    channel_to_improve: ChannelType
    expected_shrink: RationalValue
    estimated_cost: RationalValue
    shrink_per_cost: RationalValue
    description: str

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "OPTIMAL_SEPARATOR",
            "separator_id": self.separator_id,
            "channel": self.channel_to_improve.value,
            "shrink_per_cost": str(self.shrink_per_cost.fraction)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "ALPHA_OPTIMAL_SEPARATOR",
            "separator_id": self.separator_id,
            "channel_to_improve": self.channel_to_improve.value,
            "expected_shrink": str(self.expected_shrink.fraction),
            "estimated_cost": str(self.estimated_cost.fraction),
            "description": self.description,
            "result": "PASS"
        }


def compute_optimal_separator(
    intersection: AlphaIntersection,
    desired_digits: int
) -> Optional[OptimalSeparator]:
    """
    Compute the optimal next separator to improve α precision.

    Uses minimax principle: choose measurement that maximally shrinks
    frontier under worst case, per unit cost.
    """
    if not intersection.channel_results:
        return None

    # Current precision
    if intersection.final_interval:
        current_digits = intersection.certified_digits()
        if current_digits >= desired_digits:
            return None  # Already have enough precision

    # Evaluate each channel for potential improvement
    candidates = []

    for result in intersection.channel_results:
        # Estimate improvement potential
        # Wider intervals have more room for improvement
        width = result.alpha_interval.width()

        # Rough cost model (normalized)
        if result.channel_type == ChannelType.ELECTRON_G2:
            cost = RationalValue(100, 1)  # g-2 experiments are expensive
            potential_shrink = width / RationalValue(10, 1)  # Assume 10x improvement possible
        else:
            cost = RationalValue(50, 1)   # Atom recoil somewhat cheaper
            potential_shrink = width / RationalValue(5, 1)   # Assume 5x improvement

        shrink_per_cost = potential_shrink / cost

        candidates.append({
            "channel": result.channel_type,
            "shrink": potential_shrink,
            "cost": cost,
            "ratio": shrink_per_cost
        })

    if not candidates:
        return None

    # Choose best shrink-per-cost
    best = max(candidates, key=lambda c: c["ratio"].fraction)

    return OptimalSeparator(
        separator_id="TAU_STAR",
        channel_to_improve=best["channel"],
        expected_shrink=best["shrink"],
        estimated_cost=best["cost"],
        shrink_per_cost=best["ratio"],
        description=f"Improve {best['channel'].value} measurement"
    )


@dataclass
class AlphaCertified:
    """
    The certified α object - the final output.

    This is NOT a naked number. It is:
    - A certified rational interval
    - Full proof bundle
    - Hash receipt
    """
    alpha_id: str
    interval: CertifiedRationalInterval
    certified_digits: int
    channel_results: List[ChannelResult]
    intersection: AlphaIntersection
    gauge_checks: List[GaugeInvarianceCheck]

    def inverse_interval(self) -> CertifiedRationalInterval:
        """Return 1/α interval (α^-1 ≈ 137)."""
        one = CertifiedRationalInterval(RationalValue(1, 1), RationalValue(1, 1))
        return one.divide(self.interval)

    def display_alpha(self) -> str:
        """Display α value for humans (not for computation)."""
        return self.interval.to_decimal_string(self.certified_digits + 2)

    def display_inverse(self) -> str:
        """Display 1/α for humans."""
        inv = self.inverse_interval()
        return inv.to_decimal_string(self.certified_digits + 2)

    def common_digit_prefix(self) -> str:
        """
        Extract the common decimal prefix for α.

        This is the maximal string of digits that all values in the interval share.
        """
        lower = float(self.interval.lower.fraction)
        upper = float(self.interval.upper.fraction)

        # Convert to strings
        lower_str = f"{lower:.15e}"
        upper_str = f"{upper:.15e}"

        # Find common prefix
        prefix = []
        for l, u in zip(lower_str, upper_str):
            if l == u:
                prefix.append(l)
            else:
                break

        return "".join(prefix)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ALPHA_CERTIFIED",
            "alpha_id": self.alpha_id,
            "interval_lower_num": self.interval.lower.fraction.numerator,
            "interval_lower_den": self.interval.lower.fraction.denominator,
            "interval_upper_num": self.interval.upper.fraction.numerator,
            "interval_upper_den": self.interval.upper.fraction.denominator,
            "certified_digits": self.certified_digits,
            "channels_count": len(self.channel_results)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        inv = self.inverse_interval()
        return {
            "type": "ALPHA_CERTIFIED",
            "alpha_id": self.alpha_id,
            "interval_lower": str(self.interval.lower.fraction),
            "interval_upper": str(self.interval.upper.fraction),
            "certified_digits": self.certified_digits,
            "inverse_interval": f"[{float(inv.lower.fraction):.10f}, {float(inv.upper.fraction):.10f}]",
            "channel_hashes": [r.fingerprint()[:16] for r in self.channel_results],
            "gauge_invariant": all(g.is_invariant for g in self.gauge_checks),
            "bundle_fingerprint": self.fingerprint(),
            "result": "PASS"
        }


@dataclass
class AlphaProofBundle:
    """
    Complete proof bundle for α derivation.

    Contains everything needed to replay and verify the derivation.
    """
    bundle_id: str
    contract: AlphaContract
    channel_results: List[ChannelResult]
    intersection: AlphaIntersection
    alpha_certified: Optional[AlphaCertified]
    gauge_checks: List[GaugeInvarianceCheck]
    optimal_separator: Optional[OptimalSeparator]

    def all_verified(self) -> bool:
        """Check if all verifications passed."""
        channels_ok = all(r.verifier_result.is_success() for r in self.channel_results)
        intersection_ok = self.intersection.is_unique()
        gauge_ok = all(g.is_invariant for g in self.gauge_checks)
        return channels_ok and intersection_ok and gauge_ok

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ALPHA_PROOF_BUNDLE",
            "bundle_id": self.bundle_id,
            "contract_hash": self.contract.fingerprint()[:16],
            "channels_count": len(self.channel_results),
            "all_verified": self.all_verified()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        receipt = {
            "type": "ALPHA_PROOF_BUNDLE",
            "bundle_id": self.bundle_id,
            "contract_hash": self.contract.fingerprint()[:16],
            "channels_verified": len(self.channel_results),
            "intersection_status": self.intersection.status.value,
            "gauge_checks_passed": sum(1 for g in self.gauge_checks if g.is_invariant),
            "all_verified": self.all_verified(),
            "bundle_fingerprint": self.fingerprint(),
            "result": "PASS" if self.all_verified() else "FAIL"
        }
        if self.alpha_certified:
            receipt["certified_digits"] = self.alpha_certified.certified_digits
            receipt["alpha_interval"] = self.alpha_certified.display_alpha()
        if self.optimal_separator:
            receipt["next_separator"] = self.optimal_separator.description
        return receipt


def derive_alpha(
    desired_digits: int = 10
) -> AlphaProofBundle:
    """
    Complete derivation of α with full verification.

    This is the main entry point. It:
    1. Runs all measurement channels
    2. Intersects the intervals
    3. Verifies gauge invariance
    4. Produces certified α or frontier
    5. Identifies optimal next separator if needed
    """
    # Create contract
    contract = AlphaContract(
        contract_id="ALPHA_CONTRACT",
        channels=[ChannelType.ELECTRON_G2, ChannelType.ATOM_RECOIL],
        verifiers={
            ChannelType.ELECTRON_G2: "V_A_ELECTRON_G2",
            ChannelType.ATOM_RECOIL: "V_B_ATOM_RECOIL"
        }
    )

    # Run channels
    channel_results = run_all_channels()

    # Check gauge invariance for each channel
    gauge_checks = [check_gauge_invariance(r) for r in channel_results]

    # Intersect channels
    intersection = intersect_channels(channel_results)

    # Create certified α if unique
    alpha_certified = None
    if intersection.is_unique() and intersection.final_interval:
        alpha_certified = AlphaCertified(
            alpha_id="ALPHA_2024",
            interval=intersection.final_interval,
            certified_digits=intersection.certified_digits(),
            channel_results=channel_results,
            intersection=intersection,
            gauge_checks=gauge_checks
        )

    # Compute optimal separator if needed
    optimal_sep = None
    if not intersection.is_unique() or intersection.certified_digits() < desired_digits:
        optimal_sep = compute_optimal_separator(intersection, desired_digits)

    return AlphaProofBundle(
        bundle_id="ALPHA_BUNDLE",
        contract=contract,
        channel_results=channel_results,
        intersection=intersection,
        alpha_certified=alpha_certified,
        gauge_checks=gauge_checks,
        optimal_separator=optimal_sep
    )


def print_alpha_derivation(bundle: AlphaProofBundle) -> None:
    """Print human-readable derivation report."""
    print("=" * 70)
    print("FINE-STRUCTURE CONSTANT α DERIVATION")
    print("=" * 70)
    print()

    # Contract info
    print("CONTRACT:")
    print(f"  ID: {bundle.contract.contract_id}")
    print(f"  Channels: {[c.value for c in bundle.contract.channels]}")
    print(f"  Unit gauge: {bundle.contract.unit_gauge}")
    print()

    # Channel results
    print("CHANNEL RESULTS:")
    print("-" * 70)
    for result in bundle.channel_results:
        summary = get_channel_summary(result)
        print(f"  {summary['channel_id']} ({summary['channel_type']}):")
        print(f"    α ∈ [{summary['alpha_lower']:.12e}, {summary['alpha_upper']:.12e}]")
        print(f"    α⁻¹ ∈ [{summary['alpha_inverse_lower']:.6f}, {summary['alpha_inverse_upper']:.6f}]")
        print(f"    Width: {summary['width']:.3e}")
        print(f"    Certified digits: {summary['certified_digits']}")
        print(f"    Source: {summary['provenance']}")
        print()

    # Intersection
    print("INTERSECTION:")
    print("-" * 70)
    print(f"  Status: {bundle.intersection.status.value}")

    if bundle.intersection.is_unique() and bundle.intersection.final_interval:
        interval = bundle.intersection.final_interval
        inv = AlphaCertified(
            alpha_id="temp",
            interval=interval,
            certified_digits=interval.certified_digits(),
            channel_results=[],
            intersection=bundle.intersection,
            gauge_checks=[]
        ).inverse_interval()

        print(f"  Final α interval: {interval.to_decimal_string(12)}")
        print(f"  Final α⁻¹ interval: [{float(inv.lower.fraction):.9f}, {float(inv.upper.fraction):.9f}]")
        print(f"  Certified digits: {bundle.intersection.certified_digits()}")
    elif bundle.intersection.conflict_witness:
        print(f"  CONFLICT detected!")
        print(f"  Gap: {bundle.intersection.conflict_witness.conflict_gap}")
        print(f"  Minimal separator: {bundle.intersection.conflict_witness.minimal_separator}")
    print()

    # Gauge invariance
    print("GAUGE INVARIANCE CHECKS:")
    print("-" * 70)
    for check in bundle.gauge_checks:
        status = "PASS" if check.is_invariant else "FAIL"
        print(f"  [{status}] {check.check_id}")
    print()

    # Certified α
    if bundle.alpha_certified:
        print("CERTIFIED α OBJECT:")
        print("-" * 70)
        alpha = bundle.alpha_certified
        print(f"  α = {alpha.display_alpha()}")
        print(f"  α⁻¹ = {alpha.display_inverse()}")
        print(f"  Certified digits: {alpha.certified_digits}")
        print(f"  Common prefix: {alpha.common_digit_prefix()}")
        print(f"  Bundle fingerprint: {alpha.fingerprint()[:32]}...")
    print()

    # Optimal separator
    if bundle.optimal_separator:
        print("NEXT OPTIMAL MEASUREMENT:")
        print("-" * 70)
        sep = bundle.optimal_separator
        print(f"  Channel: {sep.channel_to_improve.value}")
        print(f"  Expected shrink: {float(sep.expected_shrink.fraction):.3e}")
        print(f"  Estimated cost: {float(sep.estimated_cost.fraction)}")
        print(f"  Description: {sep.description}")
    print()

    # Summary
    print("=" * 70)
    print(f"OVERALL: {'PASS' if bundle.all_verified() else 'FAIL'}")
    print(f"Proof bundle fingerprint: {bundle.fingerprint()[:32]}...")
    print("=" * 70)


# ============================================================
# MAIN ENTRY POINT
# ============================================================

if __name__ == "__main__":
    bundle = derive_alpha(desired_digits=10)
    print_alpha_derivation(bundle)
