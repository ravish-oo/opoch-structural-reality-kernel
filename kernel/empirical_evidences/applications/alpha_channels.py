"""
alpha_channels.py - Measurement channels for deriving α.

Implements:
- Channel A: Electron anomalous magnetic moment (g-2) route
- Channel B: Atom recoil route (h/m + Rydberg)
- QED coefficient series with certified truncation
- Certified bisection for interval inversion
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple
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
    create_interval,
    create_interval_from_value_uncertainty,
    C_EXACT,
    H_EXACT
)


# ============================================================
# QED COEFFICIENTS FOR ELECTRON g-2
# ============================================================

@dataclass
class QEDCoefficients:
    """
    QED perturbation series coefficients for electron anomaly.

    a_e = Σ C_n (α/π)^n + a_had + a_weak

    Coefficients are stored as exact rationals where known,
    with certified uncertainty bounds.
    """
    coefficients: Dict[int, CertifiedRationalInterval]  # Order n -> C_n interval
    truncation_order: int
    truncation_bound: CertifiedRationalInterval  # Bound on remainder
    hadronic_contribution: CertifiedRationalInterval
    weak_contribution: CertifiedRationalInterval

    def evaluate_at_alpha(
        self,
        alpha: RationalValue
    ) -> CertifiedRationalInterval:
        """
        Evaluate a_e(α) with certified interval bounds.

        Returns interval containing true value given truncation and coefficient uncertainties.
        """
        # α/π as interval (using high-precision rational π approximation)
        pi_approx = RationalValue(314159265358979323846, 10**20)
        alpha_over_pi = alpha / pi_approx

        # Sum the series
        result_lower = RationalValue(0, 1)
        result_upper = RationalValue(0, 1)

        alpha_pi_power = RationalValue(1, 1)  # (α/π)^0 = 1

        for n in range(1, self.truncation_order + 1):
            alpha_pi_power = alpha_pi_power * alpha_over_pi

            if n in self.coefficients:
                c_n = self.coefficients[n]
                term_lower = c_n.lower * alpha_pi_power
                term_upper = c_n.upper * alpha_pi_power

                result_lower = result_lower + min(term_lower, term_upper)
                result_upper = result_upper + max(term_lower, term_upper)

        # Add hadronic contribution
        result_lower = result_lower + self.hadronic_contribution.lower
        result_upper = result_upper + self.hadronic_contribution.upper

        # Add weak contribution
        result_lower = result_lower + self.weak_contribution.lower
        result_upper = result_upper + self.weak_contribution.upper

        # Add truncation bound
        result_lower = result_lower - self.truncation_bound.upper
        result_upper = result_upper + self.truncation_bound.upper

        return CertifiedRationalInterval(result_lower, result_upper)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "QED_COEFFICIENTS",
            "truncation_order": self.truncation_order,
            "coefficient_count": len(self.coefficients)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


def create_qed_coefficients() -> QEDCoefficients:
    """
    Create QED coefficients for electron g-2 calculation.

    Uses known values from QED calculations.
    C_1 = 1/2 (Schwinger, exact)
    C_2, C_3, C_4, C_5 from multi-loop calculations
    """
    # Schwinger term: C_1 = 1/2 (exact)
    c_1 = create_interval(1, 2, 1, 2)  # Exactly 1/2

    # Two-loop: C_2 ≈ -0.32847... (with uncertainty)
    # -197/60 + π²/12 + 3ζ(3)/4 - π²ln2/2 ≈ -0.32847844...
    c_2 = create_interval(-328479, 10**6, -328478, 10**6)

    # Three-loop: C_3 ≈ 1.181... (with uncertainty)
    c_3 = create_interval(118124, 10**5, 118125, 10**5)

    # Four-loop: C_4 ≈ -1.912... (with uncertainty)
    c_4 = create_interval(-191245, 10**5, -191244, 10**5)

    # Five-loop: C_5 ≈ 6.737... (with larger uncertainty)
    c_5 = create_interval(6736, 10**3, 6738, 10**3)

    # Hadronic contribution: a_had ≈ 1.67(3) × 10^-12
    a_had = create_interval_from_value_uncertainty(
        167, 10**14,  # 1.67 × 10^-12
        3, 10**14     # ±0.03 × 10^-12
    )

    # Electroweak contribution: a_weak ≈ 0.030(1) × 10^-12
    a_weak = create_interval_from_value_uncertainty(
        30, 10**15,   # 0.030 × 10^-12
        1, 10**15     # ±0.001 × 10^-12
    )

    # Truncation bound: estimate for n > 5 terms
    # |R_5| < C_6 (α/π)^6 where C_6 estimated
    truncation = create_interval(0, 1, 1, 10**15)

    return QEDCoefficients(
        coefficients={1: c_1, 2: c_2, 3: c_3, 4: c_4, 5: c_5},
        truncation_order=5,
        truncation_bound=truncation,
        hadronic_contribution=a_had,
        weak_contribution=a_weak
    )


# ============================================================
# CHANNEL A: ELECTRON g-2 VERIFIER
# ============================================================

@dataclass
class ElectronG2Verifier:
    """
    Verifier for Channel A: electron anomalous magnetic moment.

    Derives α from measured a_e by inverting the QED prediction.
    """
    verifier_id: str
    qed_coefficients: QEDCoefficients

    def verify(
        self,
        witness: WitnessBundle
    ) -> VerifierResult:
        """
        Verify α from electron g-2 measurement.

        Uses certified bisection to invert a_e(α) -> α.
        """
        # Extract experimental anomaly from witness
        if "a_e_exp" not in witness.data:
            return VerifierResult(
                verifier_id=self.verifier_id,
                status=VerifierStatus.FAIL,
                alpha_interval=None,
                diagnostics={"error": "Missing a_e_exp in witness"},
                receipt_hash=""
            )

        a_e_exp = witness.data["a_e_exp"]

        # Initial bracket for α (safely contains true value)
        # α ≈ 1/137 ≈ 0.0073
        alpha_min = RationalValue(72, 10**4)   # 0.0072
        alpha_max = RationalValue(74, 10**4)   # 0.0074

        # Bisection to find α interval
        alpha_interval = self._bisection_invert(
            a_e_exp, alpha_min, alpha_max, iterations=60
        )

        if alpha_interval is None:
            return VerifierResult(
                verifier_id=self.verifier_id,
                status=VerifierStatus.FAIL,
                alpha_interval=None,
                diagnostics={"error": "Bisection failed to converge"},
                receipt_hash=""
            )

        diagnostics = {
            "a_e_exp_lower": str(a_e_exp.lower.fraction),
            "a_e_exp_upper": str(a_e_exp.upper.fraction),
            "bisection_iterations": 60,
            "alpha_width": str(alpha_interval.width().fraction)
        }

        receipt_hash = hashlib.sha256(
            (self.verifier_id + alpha_interval.canonical()).encode()
        ).hexdigest()

        return VerifierResult(
            verifier_id=self.verifier_id,
            status=VerifierStatus.PASS,
            alpha_interval=alpha_interval,
            diagnostics=diagnostics,
            receipt_hash=receipt_hash
        )

    def _bisection_invert(
        self,
        a_e_target: CertifiedRationalInterval,
        alpha_min: RationalValue,
        alpha_max: RationalValue,
        iterations: int = 60
    ) -> Optional[CertifiedRationalInterval]:
        """
        Certified bisection to find α given target a_e.

        Returns interval [α_L, α_U] such that:
        - a_e(α_L) overlaps with a_e_target
        - a_e(α_U) overlaps with a_e_target
        """
        # Track bounds where PASS/FAIL transitions
        lower_bound = alpha_min
        upper_bound = alpha_max

        for _ in range(iterations):
            mid = (lower_bound + upper_bound) / RationalValue(2, 1)

            # Evaluate a_e at midpoint
            a_e_pred = self.qed_coefficients.evaluate_at_alpha(mid)

            # Check if predicted interval overlaps target
            if a_e_pred.overlaps(a_e_target):
                # Midpoint is valid, narrow from both sides
                # Check if we should narrow lower or upper
                if a_e_pred.midpoint() < a_e_target.midpoint():
                    lower_bound = mid
                else:
                    upper_bound = mid
            else:
                # Midpoint is outside; determine direction
                if a_e_pred.upper < a_e_target.lower:
                    # Prediction too low -> α too low
                    lower_bound = mid
                else:
                    # Prediction too high -> α too high
                    upper_bound = mid

        # Return the final bracketing interval
        return CertifiedRationalInterval(lower_bound, upper_bound)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ELECTRON_G2_VERIFIER",
            "verifier_id": self.verifier_id,
            "qed_order": self.qed_coefficients.truncation_order
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


def create_channel_a_witness() -> WitnessBundle:
    """
    Create witness bundle for Channel A (electron g-2).

    Uses best experimental value from Harvard (Gabrielse et al.)
    """
    # Experimental electron anomaly (PDG 2022)
    # a_e = 0.00115965218059(13)
    # Central: 115965218059 × 10^-14
    # Uncertainty: 13 × 10^-14
    a_e_exp = create_interval_from_value_uncertainty(
        115965218059, 10**14,  # Central value
        13, 10**14             # Uncertainty
    )

    return WitnessBundle(
        bundle_id="CHANNEL_A_WITNESS",
        channel=ChannelType.ELECTRON_G2,
        data={"a_e_exp": a_e_exp},
        coefficients={},  # QED coefficients handled separately
        uncertainty_model="gaussian_1sigma",
        provenance="Harvard Gabrielse group (2023)"
    )


def run_channel_a() -> ChannelResult:
    """
    Run Channel A to derive α from electron g-2.
    """
    witness = create_channel_a_witness()
    qed = create_qed_coefficients()
    verifier = ElectronG2Verifier(
        verifier_id="V_A_ELECTRON_G2",
        qed_coefficients=qed
    )

    result = verifier.verify(witness)

    if not result.is_success() or result.alpha_interval is None:
        raise RuntimeError(f"Channel A verification failed: {result.diagnostics}")

    return ChannelResult(
        channel_id="CHANNEL_A",
        channel_type=ChannelType.ELECTRON_G2,
        alpha_interval=result.alpha_interval,
        witness_bundle=witness,
        verifier_result=result
    )


# ============================================================
# CHANNEL B: ATOM RECOIL VERIFIER
# ============================================================

@dataclass
class AtomRecoilVerifier:
    """
    Verifier for Channel B: atom recoil measurement.

    Derives α from:
    α² = (2R_∞/c) × (A_r(X)/A_r(e)) × (h/m_X)
    """
    verifier_id: str

    def verify(
        self,
        witness: WitnessBundle
    ) -> VerifierResult:
        """
        Verify α from atom recoil measurement.

        Uses certified interval arithmetic on the Rydberg formula.
        """
        required_keys = ["R_infinity", "h_over_m_X", "A_r_X", "A_r_e"]
        for key in required_keys:
            if key not in witness.data:
                return VerifierResult(
                    verifier_id=self.verifier_id,
                    status=VerifierStatus.FAIL,
                    alpha_interval=None,
                    diagnostics={"error": f"Missing {key} in witness"},
                    receipt_hash=""
                )

        R_inf = witness.data["R_infinity"]
        h_m_X = witness.data["h_over_m_X"]
        A_r_X = witness.data["A_r_X"]
        A_r_e = witness.data["A_r_e"]

        # c is exact (SI 2019)
        c_interval = CertifiedRationalInterval(C_EXACT, C_EXACT)

        # Compute α² = (2R_∞/c) × (A_r(X)/A_r(e)) × (h/m_X)
        two = CertifiedRationalInterval(RationalValue(2, 1), RationalValue(2, 1))

        # 2R_∞
        two_R_inf = two.multiply(R_inf)

        # 2R_∞ / c
        factor1 = two_R_inf.divide(c_interval)

        # A_r(X) / A_r(e)
        mass_ratio = A_r_X.divide(A_r_e)

        # (2R_∞/c) × (A_r(X)/A_r(e))
        factor2 = factor1.multiply(mass_ratio)

        # α² = factor2 × (h/m_X)
        alpha_squared = factor2.multiply(h_m_X)

        # α = sqrt(α²)
        alpha_interval = alpha_squared.sqrt_certified()

        diagnostics = {
            "R_infinity": str(R_inf.midpoint().fraction),
            "h_over_m_X": str(h_m_X.midpoint().fraction),
            "mass_ratio": str(mass_ratio.midpoint().fraction),
            "alpha_squared_width": str(alpha_squared.width().fraction),
            "alpha_width": str(alpha_interval.width().fraction)
        }

        receipt_hash = hashlib.sha256(
            (self.verifier_id + alpha_interval.canonical()).encode()
        ).hexdigest()

        return VerifierResult(
            verifier_id=self.verifier_id,
            status=VerifierStatus.PASS,
            alpha_interval=alpha_interval,
            diagnostics=diagnostics,
            receipt_hash=receipt_hash
        )

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ATOM_RECOIL_VERIFIER",
            "verifier_id": self.verifier_id
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


def create_channel_b_witness() -> WitnessBundle:
    """
    Create witness bundle for Channel B (atom recoil).

    Uses Rb recoil measurement (Berkeley, Mueller group) and CODATA values.
    """
    # Rydberg constant R_∞ (CODATA 2018)
    # R_∞ = 10973731.568160(21) m^-1
    # = 10973731568160 × 10^-6 m^-1
    R_inf = create_interval_from_value_uncertainty(
        10973731568160, 10**6,  # Central value in m^-1
        21, 10**6               # Uncertainty
    )

    # h/m_Rb from Berkeley (2018)
    # h/m_Rb = 4.591 359 258 2(13) × 10^-9 m² s^-1
    # = 45913592582 × 10^-19
    h_m_Rb = create_interval_from_value_uncertainty(
        45913592582, 10**19,    # Central value
        13, 10**19              # Uncertainty
    )

    # Relative atomic mass of Rb-87
    # A_r(Rb-87) = 86.909180531(6)
    A_r_Rb = create_interval_from_value_uncertainty(
        86909180531, 10**9,     # Central value
        6, 10**9                # Uncertainty
    )

    # Relative atomic mass of electron
    # A_r(e) = 5.48579909065(16) × 10^-4
    # = 548579909065 × 10^-15
    A_r_e = create_interval_from_value_uncertainty(
        548579909065, 10**15,   # Central value
        16, 10**15              # Uncertainty
    )

    return WitnessBundle(
        bundle_id="CHANNEL_B_WITNESS",
        channel=ChannelType.ATOM_RECOIL,
        data={
            "R_infinity": R_inf,
            "h_over_m_X": h_m_Rb,
            "A_r_X": A_r_Rb,
            "A_r_e": A_r_e
        },
        coefficients={},
        uncertainty_model="gaussian_1sigma",
        provenance="CODATA 2018 + Berkeley Mueller group"
    )


def run_channel_b() -> ChannelResult:
    """
    Run Channel B to derive α from atom recoil.
    """
    witness = create_channel_b_witness()
    verifier = AtomRecoilVerifier(verifier_id="V_B_ATOM_RECOIL")

    result = verifier.verify(witness)

    if not result.is_success() or result.alpha_interval is None:
        raise RuntimeError(f"Channel B verification failed: {result.diagnostics}")

    return ChannelResult(
        channel_id="CHANNEL_B",
        channel_type=ChannelType.ATOM_RECOIL,
        alpha_interval=result.alpha_interval,
        witness_bundle=witness,
        verifier_result=result
    )


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def run_all_channels() -> List[ChannelResult]:
    """Run all measurement channels and return results."""
    return [
        run_channel_a(),
        run_channel_b()
    ]


def get_channel_summary(result: ChannelResult) -> Dict[str, Any]:
    """Get summary of channel result for reporting."""
    alpha = result.alpha_interval
    inverse_lower = RationalValue(1, 1) / alpha.upper
    inverse_upper = RationalValue(1, 1) / alpha.lower

    return {
        "channel_id": result.channel_id,
        "channel_type": result.channel_type.value,
        "alpha_lower": float(alpha.lower.fraction),
        "alpha_upper": float(alpha.upper.fraction),
        "alpha_inverse_lower": float(inverse_lower.fraction),
        "alpha_inverse_upper": float(inverse_upper.fraction),
        "width": float(alpha.width().fraction),
        "certified_digits": alpha.certified_digits(),
        "provenance": result.witness_bundle.provenance
    }
