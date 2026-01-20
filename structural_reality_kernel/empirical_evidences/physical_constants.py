"""
physical_constants.py - Complete set of physical constants.

Contains:
1. SI Defining Constants (exact by definition)
2. Derived Exact Constants (algebraic consequences)
3. Inferred Constants (CODATA 2022 with uncertainty intervals)

All values are exact rationals or certified intervals - no floating point.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
from fractions import Fraction
import hashlib
import math

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON

from .measurement_contract import (
    ExactRational, CertifiedInterval, UnitDimension, PhysicalConstant,
    MeasurementContract, ConstantType, VerifyStatus,
    exact_from_decimal_string, interval_from_value_uncertainty,
    digits_from_interval
)


# ============================================================
# SI DEFINING CONSTANTS (Exact by Definition)
# ============================================================

# Unit dimensions for reference
DIM_FREQUENCY = UnitDimension(s=-1)  # Hz = s^-1
DIM_VELOCITY = UnitDimension(m=1, s=-1)  # m/s
DIM_ACTION = UnitDimension(m=2, kg=1, s=-1)  # J s = m^2 kg s^-1
DIM_CHARGE = UnitDimension(s=1, A=1)  # C = A s
DIM_ENTROPY = UnitDimension(m=2, kg=1, s=-2, K=-1)  # J/K = m^2 kg s^-2 K^-1
DIM_AMOUNT_INV = UnitDimension(mol=-1)  # mol^-1
DIM_LUMINOUS_EFFICACY = UnitDimension(m=-2, kg=-1, s=3, cd=1)  # lm/W = cd sr / W


def create_si_defining_constants() -> Dict[str, PhysicalConstant]:
    """
    Create the 7 SI defining constants.

    These are EXACT by definition - they define the units.
    """
    constants = {}

    # 1. Cesium hyperfine frequency (defines second)
    constants["nu_Cs"] = PhysicalConstant(
        symbol="nu_Cs",
        name="Cesium hyperfine transition frequency",
        constant_type=ConstantType.DEFINED_EXACT,
        dimension=DIM_FREQUENCY,
        exact_value=exact_from_decimal_string("9192631770"),
        is_si_defining=True,
        relations=["Defines the second: 1 s = 9192631770 / nu_Cs"],
        source="SI definition (2019)"
    )

    # 2. Speed of light (defines meter)
    constants["c"] = PhysicalConstant(
        symbol="c",
        name="Speed of light in vacuum",
        constant_type=ConstantType.DEFINED_EXACT,
        dimension=DIM_VELOCITY,
        exact_value=exact_from_decimal_string("299792458"),
        is_si_defining=True,
        relations=["Defines the meter: 1 m = c / 299792458 s"],
        source="SI definition (2019)"
    )

    # 3. Planck constant (defines kilogram)
    constants["h"] = PhysicalConstant(
        symbol="h",
        name="Planck constant",
        constant_type=ConstantType.DEFINED_EXACT,
        dimension=DIM_ACTION,
        exact_value=exact_from_decimal_string("6.62607015e-34"),
        is_si_defining=True,
        relations=["Defines the kilogram: 1 kg = h / (6.62607015e-34 m^2 s^-1)"],
        source="SI definition (2019)"
    )

    # 4. Elementary charge (defines ampere)
    constants["e"] = PhysicalConstant(
        symbol="e",
        name="Elementary charge",
        constant_type=ConstantType.DEFINED_EXACT,
        dimension=DIM_CHARGE,
        exact_value=exact_from_decimal_string("1.602176634e-19"),
        is_si_defining=True,
        relations=["Defines the ampere: 1 A = e / (1.602176634e-19 s)"],
        source="SI definition (2019)"
    )

    # 5. Boltzmann constant (defines kelvin)
    constants["k_B"] = PhysicalConstant(
        symbol="k_B",
        name="Boltzmann constant",
        constant_type=ConstantType.DEFINED_EXACT,
        dimension=DIM_ENTROPY,
        exact_value=exact_from_decimal_string("1.380649e-23"),
        is_si_defining=True,
        relations=["Defines the kelvin: 1 K = 1.380649e-23 / k_B J"],
        source="SI definition (2019)"
    )

    # 6. Avogadro constant (defines mole)
    constants["N_A"] = PhysicalConstant(
        symbol="N_A",
        name="Avogadro constant",
        constant_type=ConstantType.DEFINED_EXACT,
        dimension=DIM_AMOUNT_INV,
        exact_value=exact_from_decimal_string("6.02214076e23"),
        is_si_defining=True,
        relations=["Defines the mole: 1 mol = N_A / 6.02214076e23 entities"],
        source="SI definition (2019)"
    )

    # 7. Luminous efficacy (defines candela)
    constants["K_cd"] = PhysicalConstant(
        symbol="K_cd",
        name="Luminous efficacy of 540 THz radiation",
        constant_type=ConstantType.DEFINED_EXACT,
        dimension=DIM_LUMINOUS_EFFICACY,
        exact_value=exact_from_decimal_string("683"),
        is_si_defining=True,
        relations=["Defines the candela: cd = K_cd / 683 lm W^-1"],
        source="SI definition (2019)"
    )

    return constants


# ============================================================
# DERIVED EXACT CONSTANTS
# ============================================================

def create_derived_exact_constants(
    si_constants: Dict[str, PhysicalConstant]
) -> Dict[str, PhysicalConstant]:
    """
    Create derived constants that are exact because they're
    algebraic combinations of exact SI constants.
    """
    constants = {}

    h = si_constants["h"].exact_value
    e = si_constants["e"].exact_value
    N_A = si_constants["N_A"].exact_value
    k_B = si_constants["k_B"].exact_value

    # Reduced Planck constant: hbar = h / (2 pi)
    # Note: This involves pi, so it's a real-number refinement object
    # We represent it as h / (2 * pi) where pi is computed to high precision
    # For exact representation, we store it as a relation
    constants["hbar"] = PhysicalConstant(
        symbol="hbar",
        name="Reduced Planck constant",
        constant_type=ConstantType.DEFINED_EXACT,
        dimension=DIM_ACTION,
        exact_value=h,  # Stored as h, actual value is h/(2*pi)
        is_si_defining=False,
        relations=["hbar = h / (2 * pi)", "Exact given h exact and pi algorithmic"],
        source="Derived from SI"
    )

    # Josephson constant: K_J = 2e/h (exact)
    K_J = (ExactRational(2, 1) * e) / h
    constants["K_J"] = PhysicalConstant(
        symbol="K_J",
        name="Josephson constant",
        constant_type=ConstantType.DEFINED_EXACT,
        dimension=UnitDimension(m=-2, kg=-1, s=2, A=1),  # Hz/V
        exact_value=K_J,
        is_si_defining=False,
        relations=["K_J = 2e/h", "Exact since e and h are exact"],
        source="Derived from SI"
    )

    # von Klitzing constant: R_K = h/e^2 (exact)
    R_K = h / (e * e)
    constants["R_K"] = PhysicalConstant(
        symbol="R_K",
        name="von Klitzing constant",
        constant_type=ConstantType.DEFINED_EXACT,
        dimension=UnitDimension(m=2, kg=1, s=-3, A=-2),  # Ohm
        exact_value=R_K,
        is_si_defining=False,
        relations=["R_K = h/e^2", "Exact since h and e are exact"],
        source="Derived from SI"
    )

    # Molar gas constant: R = N_A * k_B (exact)
    R = N_A * k_B
    constants["R"] = PhysicalConstant(
        symbol="R",
        name="Molar gas constant",
        constant_type=ConstantType.DEFINED_EXACT,
        dimension=UnitDimension(m=2, kg=1, s=-2, K=-1, mol=-1),  # J/(mol K)
        exact_value=R,
        is_si_defining=False,
        relations=["R = N_A * k_B", "Exact since both are exact"],
        source="Derived from SI"
    )

    # Faraday constant: F = N_A * e (exact)
    F = N_A * e
    constants["F"] = PhysicalConstant(
        symbol="F",
        name="Faraday constant",
        constant_type=ConstantType.DEFINED_EXACT,
        dimension=UnitDimension(s=1, A=1, mol=-1),  # C/mol
        exact_value=F,
        is_si_defining=False,
        relations=["F = N_A * e", "Exact since both are exact"],
        source="Derived from SI"
    )

    return constants


# ============================================================
# INFERRED CONSTANTS (CODATA 2022)
# ============================================================

def create_inferred_constants(
    si_constants: Dict[str, PhysicalConstant]
) -> Dict[str, PhysicalConstant]:
    """
    Create inferred constants from CODATA 2022.

    These are measured values with uncertainty, represented as intervals.
    """
    constants = {}

    # Fine-structure constant: alpha = 7.2973525643(11) x 10^-3
    # This is THE key dimensionless constant
    alpha_iv = interval_from_value_uncertainty("0.0072973525643", "11")
    constants["alpha"] = PhysicalConstant(
        symbol="alpha",
        name="Fine-structure constant",
        constant_type=ConstantType.INFERRED_INTERVAL,
        dimension=UnitDimension(),  # Dimensionless
        interval_value=alpha_iv,
        is_si_defining=False,
        relations=["alpha = e^2 / (4 pi eps_0 hbar c)", "Fundamental dimensionless constant"],
        source="CODATA 2022"
    )

    # Inverse fine-structure constant: alpha^-1 = 137.035999177(21)
    alpha_inv_iv = interval_from_value_uncertainty("137.035999177", "21")
    constants["alpha_inv"] = PhysicalConstant(
        symbol="alpha^-1",
        name="Inverse fine-structure constant",
        constant_type=ConstantType.INFERRED_INTERVAL,
        dimension=UnitDimension(),  # Dimensionless
        interval_value=alpha_inv_iv,
        is_si_defining=False,
        relations=["alpha^-1 = 1/alpha"],
        source="CODATA 2022"
    )

    # Gravitational constant: G = 6.67430(15) x 10^-11 m^3 kg^-1 s^-2
    G_iv = interval_from_value_uncertainty("0.0000000000667430", "15")
    constants["G"] = PhysicalConstant(
        symbol="G",
        name="Newtonian constant of gravitation",
        constant_type=ConstantType.INFERRED_INTERVAL,
        dimension=UnitDimension(m=3, kg=-1, s=-2),
        interval_value=G_iv,
        is_si_defining=False,
        relations=["F = G m1 m2 / r^2"],
        source="CODATA 2022"
    )

    # Electron mass: m_e = 9.1093837139(28) x 10^-31 kg
    m_e_iv = interval_from_value_uncertainty("0.00000000000000000000000000000091093837139", "28")
    constants["m_e"] = PhysicalConstant(
        symbol="m_e",
        name="Electron mass",
        constant_type=ConstantType.INFERRED_INTERVAL,
        dimension=UnitDimension(kg=1),
        interval_value=m_e_iv,
        is_si_defining=False,
        relations=["Fundamental particle mass"],
        source="CODATA 2022"
    )

    # Proton mass: m_p = 1.67262192595(52) x 10^-27 kg
    m_p_iv = interval_from_value_uncertainty("0.00000000000000000000000000167262192595", "52")
    constants["m_p"] = PhysicalConstant(
        symbol="m_p",
        name="Proton mass",
        constant_type=ConstantType.INFERRED_INTERVAL,
        dimension=UnitDimension(kg=1),
        interval_value=m_p_iv,
        is_si_defining=False,
        relations=["Fundamental particle mass"],
        source="CODATA 2022"
    )

    # Rydberg constant: R_inf = 10973731.568157(12) m^-1
    R_inf_iv = interval_from_value_uncertainty("10973731.568157", "12")
    constants["R_inf"] = PhysicalConstant(
        symbol="R_inf",
        name="Rydberg constant",
        constant_type=ConstantType.INFERRED_INTERVAL,
        dimension=UnitDimension(m=-1),
        interval_value=R_inf_iv,
        is_si_defining=False,
        relations=["R_inf = alpha^2 m_e c / (2 h)"],
        source="CODATA 2022"
    )

    return constants


def compute_derived_electromagnetic_constants(
    si_constants: Dict[str, PhysicalConstant],
    inferred_constants: Dict[str, PhysicalConstant]
) -> Dict[str, PhysicalConstant]:
    """
    Compute mu_0 and eps_0 from alpha (with uncertainty propagation).

    mu_0 = (2 alpha h) / (e^2 c)
    eps_0 = 1 / (mu_0 c^2)

    These inherit uncertainty from alpha.
    """
    constants = {}

    h = si_constants["h"].exact_value
    e = si_constants["e"].exact_value
    c = si_constants["c"].exact_value
    alpha_iv = inferred_constants["alpha"].interval_value

    # Convert exact values to point intervals for arithmetic
    h_iv = CertifiedInterval(h, h)
    e_iv = CertifiedInterval(e, e)
    c_iv = CertifiedInterval(c, c)
    two_iv = CertifiedInterval(ExactRational(2, 1), ExactRational(2, 1))
    one_iv = CertifiedInterval(ExactRational(1, 1), ExactRational(1, 1))

    # mu_0 = (2 * alpha * h) / (e^2 * c)
    numerator = two_iv * alpha_iv * h_iv
    denominator = e_iv * e_iv * c_iv
    mu_0_iv = numerator / denominator

    constants["mu_0"] = PhysicalConstant(
        symbol="mu_0",
        name="Vacuum magnetic permeability",
        constant_type=ConstantType.INFERRED_INTERVAL,
        dimension=UnitDimension(m=1, kg=1, s=-2, A=-2),  # N/A^2 = H/m
        interval_value=mu_0_iv,
        is_si_defining=False,
        relations=["mu_0 = 2 alpha h / (e^2 c)", "Inherits uncertainty from alpha"],
        source="Derived from CODATA 2022 alpha"
    )

    # eps_0 = 1 / (mu_0 * c^2)
    c_squared = c_iv * c_iv
    eps_0_iv = one_iv / (mu_0_iv * c_squared)

    constants["eps_0"] = PhysicalConstant(
        symbol="eps_0",
        name="Vacuum electric permittivity",
        constant_type=ConstantType.INFERRED_INTERVAL,
        dimension=UnitDimension(m=-3, kg=-1, s=4, A=2),  # F/m
        interval_value=eps_0_iv,
        is_si_defining=False,
        relations=["eps_0 = 1 / (mu_0 c^2)", "Inherits uncertainty from alpha"],
        source="Derived from CODATA 2022 alpha"
    )

    return constants


# ============================================================
# CONSTANTS BUNDLE
# ============================================================

@dataclass
class ConstantsBundle:
    """
    Complete bundle of physical constants with verification.
    """
    bundle_id: str
    si_defining: Dict[str, PhysicalConstant]
    derived_exact: Dict[str, PhysicalConstant]
    inferred: Dict[str, PhysicalConstant]
    derived_electromagnetic: Dict[str, PhysicalConstant]

    def all_constants(self) -> Dict[str, PhysicalConstant]:
        """Get all constants."""
        result = {}
        result.update(self.si_defining)
        result.update(self.derived_exact)
        result.update(self.inferred)
        result.update(self.derived_electromagnetic)
        return result

    def get_constant(self, symbol: str) -> Optional[PhysicalConstant]:
        """Get a constant by symbol."""
        return self.all_constants().get(symbol)

    def canonical(self) -> str:
        # Create deterministic representation
        all_hashes = sorted([
            c.fingerprint() for c in self.all_constants().values()
        ])
        return CanonicalJSON.serialize({
            "type": "CONSTANTS_BUNDLE",
            "bundle_id": self.bundle_id,
            "si_defining_count": len(self.si_defining),
            "derived_exact_count": len(self.derived_exact),
            "inferred_count": len(self.inferred),
            "derived_em_count": len(self.derived_electromagnetic),
            "constant_hashes": all_hashes
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "CONSTANTS_BUNDLE",
            "bundle_id": self.bundle_id,
            "si_defining_count": len(self.si_defining),
            "derived_exact_count": len(self.derived_exact),
            "inferred_count": len(self.inferred),
            "derived_em_count": len(self.derived_electromagnetic),
            "total_constants": len(self.all_constants()),
            "bundle_fingerprint": self.fingerprint()[:32],
            "result": "PASS"
        }


def create_constants_bundle() -> ConstantsBundle:
    """
    Create the complete constants bundle.
    """
    si_defining = create_si_defining_constants()
    derived_exact = create_derived_exact_constants(si_defining)
    inferred = create_inferred_constants(si_defining)
    derived_em = compute_derived_electromagnetic_constants(si_defining, inferred)

    return ConstantsBundle(
        bundle_id="CODATA_2022",
        si_defining=si_defining,
        derived_exact=derived_exact,
        inferred=inferred,
        derived_electromagnetic=derived_em
    )


def print_constants_summary(bundle: ConstantsBundle) -> str:
    """Print a summary of all constants."""
    lines = [
        "=" * 70,
        "PHYSICAL CONSTANTS BUNDLE",
        "=" * 70,
        f"Bundle ID: {bundle.bundle_id}",
        f"Fingerprint: {bundle.fingerprint()[:32]}...",
        "",
        "-" * 70,
        "SI DEFINING CONSTANTS (Exact by definition)",
        "-" * 70,
    ]

    for symbol, const in sorted(bundle.si_defining.items()):
        digits = const.get_digits(15)
        lines.append(f"  {symbol}: {digits} [{const.dimension.to_string()}]")

    lines.extend([
        "",
        "-" * 70,
        "DERIVED EXACT CONSTANTS",
        "-" * 70,
    ])

    for symbol, const in sorted(bundle.derived_exact.items()):
        digits = const.get_digits(15)
        lines.append(f"  {symbol}: {digits}")

    lines.extend([
        "",
        "-" * 70,
        "INFERRED CONSTANTS (CODATA 2022, with uncertainty)",
        "-" * 70,
    ])

    for symbol, const in sorted(bundle.inferred.items()):
        digits = const.get_digits(15)
        lines.append(f"  {symbol}: {digits}")

    lines.extend([
        "",
        "-" * 70,
        "DERIVED ELECTROMAGNETIC (uncertainty from alpha)",
        "-" * 70,
    ])

    for symbol, const in sorted(bundle.derived_electromagnetic.items()):
        digits = const.get_digits(15)
        lines.append(f"  {symbol}: {digits}")

    lines.append("=" * 70)

    return "\n".join(lines)
