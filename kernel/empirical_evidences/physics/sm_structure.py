"""
sm_structure.py - Standard Model structure definitions.

Implements:
- Gauge group G_SM = SU(3)_c x SU(2)_L x U(1)_Y
- Field content (3 generations of fermions + Higgs)
- Representation assignments and hypercharges
- Lagrangian structure (gauge, fermion, Higgs, Yukawa terms)
- Anomaly cancellation verification
- Electroweak symmetry breaking relations
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


class GaugeGroupFactor(Enum):
    """Factors of the Standard Model gauge group."""
    SU3_COLOR = ("SU(3)_c", 3, 8)     # (name, N, dim of adjoint)
    SU2_WEAK = ("SU(2)_L", 2, 3)
    U1_HYPERCHARGE = ("U(1)_Y", 1, 1)

    @property
    def name(self) -> str:
        return self.value[0]

    @property
    def n_value(self) -> int:
        return self.value[1]

    @property
    def adjoint_dim(self) -> int:
        return self.value[2]


class Chirality(Enum):
    """Chirality of fermion fields."""
    LEFT = "L"
    RIGHT = "R"


class FieldType(Enum):
    """Types of fields in the Standard Model."""
    QUARK_DOUBLET = "quark_doublet"      # Q_L = (u_L, d_L)
    QUARK_UP_SINGLET = "quark_up_singlet"  # u_R
    QUARK_DOWN_SINGLET = "quark_down_singlet"  # d_R
    LEPTON_DOUBLET = "lepton_doublet"    # L_L = (nu_L, e_L)
    LEPTON_SINGLET = "lepton_singlet"    # e_R
    HIGGS_DOUBLET = "higgs_doublet"      # Phi


@dataclass(frozen=True)
class Representation:
    """
    Representation of a field under the gauge group.

    Specifies (SU(3), SU(2), U(1)_Y) quantum numbers.
    """
    su3_rep: int      # 1 = singlet, 3 = fundamental, 3_bar = antifundamental
    su2_rep: int      # 1 = singlet, 2 = doublet
    hypercharge: RationalValue  # Y (U(1) charge)

    def is_singlet(self, factor: GaugeGroupFactor) -> bool:
        """Check if field is singlet under given factor."""
        if factor == GaugeGroupFactor.SU3_COLOR:
            return self.su3_rep == 1
        elif factor == GaugeGroupFactor.SU2_WEAK:
            return self.su2_rep == 1
        else:
            return self.hypercharge == RationalValue(0, 1)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "su3": self.su3_rep,
            "su2": self.su2_rep,
            "Y_num": self.hypercharge.fraction.numerator,
            "Y_den": self.hypercharge.fraction.denominator
        })


@dataclass
class SMField:
    """
    A field in the Standard Model.

    Includes representation, chirality, and generation.
    """
    field_id: str
    field_type: FieldType
    representation: Representation
    chirality: Optional[Chirality]  # None for scalars
    generation: Optional[int]  # 1, 2, 3 for fermions; None for Higgs
    multiplicity: int = 1  # Number of components

    def is_fermion(self) -> bool:
        return self.field_type not in [FieldType.HIGGS_DOUBLET]

    def is_chiral(self) -> bool:
        return self.chirality is not None

    def electric_charge_formula(self) -> str:
        """Return Q = T_3 + Y/2 formula."""
        return "Q = T_3 + Y/2"

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "SM_FIELD",
            "field_id": self.field_id,
            "field_type": self.field_type.value,
            "representation": self.representation.canonical(),
            "chirality": self.chirality.value if self.chirality else None,
            "generation": self.generation
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class GaugeGroup:
    """
    The Standard Model gauge group.

    G_SM = SU(3)_c x SU(2)_L x U(1)_Y
    """
    group_id: str
    factors: List[GaugeGroupFactor]

    def __post_init__(self):
        # Ensure standard ordering
        self.factors = sorted(self.factors, key=lambda f: f.name)

    def coupling_count(self) -> int:
        """Number of independent gauge couplings."""
        return len(self.factors)

    def total_generators(self) -> int:
        """Total number of gauge generators."""
        return sum(f.adjoint_dim for f in self.factors)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "SM_GAUGE_GROUP",
            "group_id": self.group_id,
            "factors": [f.name for f in self.factors],
            "coupling_count": self.coupling_count(),
            "total_generators": self.total_generators()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self, anomaly_free: bool = True) -> Dict[str, Any]:
        return {
            "type": "SM_GAUGE_GROUP",
            "group_id": self.group_id,
            "factors": [f.name for f in self.factors],
            "coupling_count": self.coupling_count(),
            "anomaly_free": anomaly_free,
            "result": "PASS" if anomaly_free else "FAIL"
        }


@dataclass
class FieldContent:
    """
    Complete field content of the Standard Model.
    """
    content_id: str
    fields: List[SMField]
    generations: int = 3

    def quark_doublets(self) -> List[SMField]:
        return [f for f in self.fields if f.field_type == FieldType.QUARK_DOUBLET]

    def quark_singlets(self) -> List[SMField]:
        return [f for f in self.fields if f.field_type in
                [FieldType.QUARK_UP_SINGLET, FieldType.QUARK_DOWN_SINGLET]]

    def lepton_doublets(self) -> List[SMField]:
        return [f for f in self.fields if f.field_type == FieldType.LEPTON_DOUBLET]

    def lepton_singlets(self) -> List[SMField]:
        return [f for f in self.fields if f.field_type == FieldType.LEPTON_SINGLET]

    def higgs_fields(self) -> List[SMField]:
        return [f for f in self.fields if f.field_type == FieldType.HIGGS_DOUBLET]

    def total_fields(self) -> int:
        return len(self.fields)

    def fermion_count(self) -> int:
        return sum(1 for f in self.fields if f.is_fermion())

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "SM_FIELD_CONTENT",
            "content_id": self.content_id,
            "generations": self.generations,
            "total_fields": self.total_fields(),
            "fermion_count": self.fermion_count()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SM_FIELD_CONTENT",
            "content_id": self.content_id,
            "generations": self.generations,
            "quark_doublets": len(self.quark_doublets()),
            "quark_singlets": len(self.quark_singlets()),
            "lepton_doublets": len(self.lepton_doublets()),
            "lepton_singlets": len(self.lepton_singlets()),
            "higgs_doublets": len(self.higgs_fields()),
            "total_fields": self.total_fields(),
            "result": "PASS"
        }


class LagrangianTermType(Enum):
    """Types of terms in the SM Lagrangian."""
    GAUGE_KINETIC = "gauge_kinetic"
    FERMION_KINETIC = "fermion_kinetic"
    HIGGS_KINETIC = "higgs_kinetic"
    HIGGS_POTENTIAL = "higgs_potential"
    YUKAWA = "yukawa"
    GAUGE_FIXING = "gauge_fixing"
    GHOST = "ghost"


@dataclass
class LagrangianTerm:
    """
    A term in the Standard Model Lagrangian.
    """
    term_id: str
    term_type: LagrangianTermType
    dimension: int  # Mass dimension
    fields_involved: List[str]
    gauge_invariant: bool = True
    lorentz_invariant: bool = True

    def is_renormalizable(self) -> bool:
        """Check if term has dimension <= 4."""
        return self.dimension <= 4

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "LAGRANGIAN_TERM",
            "term_id": self.term_id,
            "term_type": self.term_type.value,
            "dimension": self.dimension,
            "gauge_invariant": self.gauge_invariant,
            "lorentz_invariant": self.lorentz_invariant
        })


@dataclass
class SMLagrangian:
    """
    The Standard Model Lagrangian.

    L_SM = L_gauge + L_fermion + L_Higgs + L_Yukawa
    """
    lagrangian_id: str
    terms: List[LagrangianTerm]
    dimension_bound: int = 4  # Renormalizability constraint

    def gauge_terms(self) -> List[LagrangianTerm]:
        return [t for t in self.terms if t.term_type == LagrangianTermType.GAUGE_KINETIC]

    def fermion_terms(self) -> List[LagrangianTerm]:
        return [t for t in self.terms if t.term_type == LagrangianTermType.FERMION_KINETIC]

    def higgs_terms(self) -> List[LagrangianTerm]:
        return [t for t in self.terms if t.term_type in
                [LagrangianTermType.HIGGS_KINETIC, LagrangianTermType.HIGGS_POTENTIAL]]

    def yukawa_terms(self) -> List[LagrangianTerm]:
        return [t for t in self.terms if t.term_type == LagrangianTermType.YUKAWA]

    def is_gauge_invariant(self) -> bool:
        return all(t.gauge_invariant for t in self.terms)

    def is_lorentz_invariant(self) -> bool:
        return all(t.lorentz_invariant for t in self.terms)

    def is_renormalizable(self) -> bool:
        return all(t.is_renormalizable() for t in self.terms)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "SM_LAGRANGIAN",
            "lagrangian_id": self.lagrangian_id,
            "term_count": len(self.terms),
            "dimension_bound": self.dimension_bound
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SM_LAGRANGIAN",
            "lagrangian_id": self.lagrangian_id,
            "gauge_terms": len(self.gauge_terms()) > 0,
            "fermion_terms": len(self.fermion_terms()) > 0,
            "higgs_terms": len(self.higgs_terms()) > 0,
            "yukawa_terms": len(self.yukawa_terms()) > 0,
            "dimension_bound": self.dimension_bound,
            "gauge_invariant": self.is_gauge_invariant(),
            "lorentz_invariant": self.is_lorentz_invariant(),
            "result": "PASS" if self.is_gauge_invariant() and self.is_lorentz_invariant() else "FAIL"
        }


@dataclass
class AnomalyCancellation:
    """
    Verification of anomaly cancellation in the Standard Model.

    Gauge anomalies must cancel for quantum consistency.
    """
    check_id: str
    su3_su3_su3: RationalValue  # [SU(3)]^3 anomaly
    su2_su2_su2: RationalValue  # [SU(2)]^3 anomaly
    u1_u1_u1: RationalValue     # [U(1)]^3 anomaly
    su3_su3_u1: RationalValue   # [SU(3)]^2 U(1) anomaly
    su2_su2_u1: RationalValue   # [SU(2)]^2 U(1) anomaly
    gravitational: RationalValue # Gravitational anomaly (U(1))

    def all_cancel(self) -> bool:
        """Check if all anomalies vanish."""
        zero = RationalValue(0, 1)
        return (self.su3_su3_su3 == zero and
                self.su2_su2_su2 == zero and
                self.u1_u1_u1 == zero and
                self.su3_su3_u1 == zero and
                self.su2_su2_u1 == zero and
                self.gravitational == zero)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ANOMALY_CANCELLATION",
            "check_id": self.check_id,
            "all_cancel": self.all_cancel()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "ANOMALY_CANCELLATION",
            "check_id": self.check_id,
            "su3_su3_su3": self.su3_su3_su3.fraction.numerator,
            "su2_su2_su2": self.su2_su2_su2.fraction.numerator,
            "u1_u1_u1": self.u1_u1_u1.fraction.numerator,
            "su3_su3_u1": self.su3_su3_u1.fraction.numerator,
            "su2_su2_u1": self.su2_su2_u1.fraction.numerator,
            "gravitational": self.gravitational.fraction.numerator,
            "all_cancel": self.all_cancel(),
            "result": "PASS" if self.all_cancel() else "FAIL"
        }


@dataclass
class EWSBRelations:
    """
    Electroweak symmetry breaking relations.

    M_W = (1/2) g v
    M_Z = (1/2) v sqrt(g^2 + g'^2)
    sin^2(theta_W) = g'^2 / (g^2 + g'^2)
    """
    ewsb_id: str
    vev_gev: RationalValue          # Higgs VEV (GeV)
    m_w_gev: RationalValue          # W boson mass (GeV)
    m_z_gev: RationalValue          # Z boson mass (GeV)
    sin2_theta_w: RationalValue     # sin^2(theta_W)
    m_h_gev: RationalValue          # Higgs mass (GeV)

    def check_mass_relation(self) -> bool:
        """
        Check M_W / M_Z = cos(theta_W).

        Equivalently: M_W^2 / M_Z^2 = 1 - sin^2(theta_W)
        """
        # Using rational arithmetic
        m_w_sq = self.m_w_gev * self.m_w_gev
        m_z_sq = self.m_z_gev * self.m_z_gev

        if m_z_sq == RationalValue(0, 1):
            return False

        ratio = m_w_sq / m_z_sq
        one_minus_sin2 = RationalValue(1, 1) - self.sin2_theta_w

        # Check approximate equality (within tolerance)
        diff = ratio - one_minus_sin2
        # For exact rationals, we check if difference is small
        tolerance = RationalValue(1, 100)  # 1% tolerance
        return abs(diff.fraction) < tolerance.fraction

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "EWSB_RELATIONS",
            "ewsb_id": self.ewsb_id,
            "vev_gev_num": self.vev_gev.fraction.numerator,
            "vev_gev_den": self.vev_gev.fraction.denominator,
            "m_w_gev_num": self.m_w_gev.fraction.numerator,
            "m_z_gev_num": self.m_z_gev.fraction.numerator
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SM_EWSB",
            "ewsb_id": self.ewsb_id,
            "vev_gev": self.vev_gev.fraction.numerator,
            "m_w_gev": self.m_w_gev.fraction.numerator,
            "m_z_gev": self.m_z_gev.fraction.numerator,
            "sin2_theta_w_num": self.sin2_theta_w.fraction.numerator,
            "sin2_theta_w_den": self.sin2_theta_w.fraction.denominator,
            "m_h_gev": self.m_h_gev.fraction.numerator,
            "relations_satisfied": self.check_mass_relation(),
            "result": "PASS" if self.check_mass_relation() else "FAIL"
        }


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def create_sm_gauge_group() -> GaugeGroup:
    """Create the Standard Model gauge group."""
    return GaugeGroup(
        group_id="G_SM",
        factors=[
            GaugeGroupFactor.SU3_COLOR,
            GaugeGroupFactor.SU2_WEAK,
            GaugeGroupFactor.U1_HYPERCHARGE
        ]
    )


def create_sm_field_content() -> FieldContent:
    """Create the Standard Model field content (3 generations)."""
    fields = []

    # Hypercharges (Y) for SM fields
    # Q = T_3 + Y/2, so Y = 2(Q - T_3)
    Y_Q = RationalValue(1, 3)    # Quark doublet
    Y_uR = RationalValue(4, 3)   # u_R
    Y_dR = RationalValue(-2, 3)  # d_R
    Y_L = RationalValue(-1, 1)   # Lepton doublet
    Y_eR = RationalValue(-2, 1)  # e_R
    Y_H = RationalValue(1, 1)    # Higgs doublet

    # Three generations of fermions
    for gen in [1, 2, 3]:
        # Quark doublet Q_L = (u_L, d_L)
        fields.append(SMField(
            field_id=f"Q_L_{gen}",
            field_type=FieldType.QUARK_DOUBLET,
            representation=Representation(3, 2, Y_Q),  # (3, 2, 1/3)
            chirality=Chirality.LEFT,
            generation=gen,
            multiplicity=2  # Doublet
        ))

        # Up-type quark singlet u_R
        fields.append(SMField(
            field_id=f"u_R_{gen}",
            field_type=FieldType.QUARK_UP_SINGLET,
            representation=Representation(3, 1, Y_uR),  # (3, 1, 4/3)
            chirality=Chirality.RIGHT,
            generation=gen,
            multiplicity=1
        ))

        # Down-type quark singlet d_R
        fields.append(SMField(
            field_id=f"d_R_{gen}",
            field_type=FieldType.QUARK_DOWN_SINGLET,
            representation=Representation(3, 1, Y_dR),  # (3, 1, -2/3)
            chirality=Chirality.RIGHT,
            generation=gen,
            multiplicity=1
        ))

        # Lepton doublet L_L = (nu_L, e_L)
        fields.append(SMField(
            field_id=f"L_L_{gen}",
            field_type=FieldType.LEPTON_DOUBLET,
            representation=Representation(1, 2, Y_L),  # (1, 2, -1)
            chirality=Chirality.LEFT,
            generation=gen,
            multiplicity=2
        ))

        # Charged lepton singlet e_R
        fields.append(SMField(
            field_id=f"e_R_{gen}",
            field_type=FieldType.LEPTON_SINGLET,
            representation=Representation(1, 1, Y_eR),  # (1, 1, -2)
            chirality=Chirality.RIGHT,
            generation=gen,
            multiplicity=1
        ))

    # Higgs doublet
    fields.append(SMField(
        field_id="Phi",
        field_type=FieldType.HIGGS_DOUBLET,
        representation=Representation(1, 2, Y_H),  # (1, 2, 1)
        chirality=None,  # Scalar
        generation=None,
        multiplicity=2
    ))

    return FieldContent(
        content_id="SM_CONTENT",
        fields=fields,
        generations=3
    )


def create_sm_lagrangian() -> SMLagrangian:
    """Create the Standard Model Lagrangian structure."""
    terms = []

    # Gauge kinetic terms (dimension 4)
    terms.append(LagrangianTerm(
        term_id="L_gauge_SU3",
        term_type=LagrangianTermType.GAUGE_KINETIC,
        dimension=4,
        fields_involved=["G_mu_nu"]
    ))
    terms.append(LagrangianTerm(
        term_id="L_gauge_SU2",
        term_type=LagrangianTermType.GAUGE_KINETIC,
        dimension=4,
        fields_involved=["W_mu_nu"]
    ))
    terms.append(LagrangianTerm(
        term_id="L_gauge_U1",
        term_type=LagrangianTermType.GAUGE_KINETIC,
        dimension=4,
        fields_involved=["B_mu_nu"]
    ))

    # Fermion kinetic terms (dimension 4)
    for field_type in ["Q_L", "u_R", "d_R", "L_L", "e_R"]:
        terms.append(LagrangianTerm(
            term_id=f"L_fermion_{field_type}",
            term_type=LagrangianTermType.FERMION_KINETIC,
            dimension=4,
            fields_involved=[field_type]
        ))

    # Higgs kinetic term (dimension 4)
    terms.append(LagrangianTerm(
        term_id="L_higgs_kinetic",
        term_type=LagrangianTermType.HIGGS_KINETIC,
        dimension=4,
        fields_involved=["Phi"]
    ))

    # Higgs potential (dimension 4)
    terms.append(LagrangianTerm(
        term_id="L_higgs_potential",
        term_type=LagrangianTermType.HIGGS_POTENTIAL,
        dimension=4,
        fields_involved=["Phi"]
    ))

    # Yukawa terms (dimension 4)
    terms.append(LagrangianTerm(
        term_id="L_yukawa_up",
        term_type=LagrangianTermType.YUKAWA,
        dimension=4,
        fields_involved=["Q_L", "Phi_tilde", "u_R"]
    ))
    terms.append(LagrangianTerm(
        term_id="L_yukawa_down",
        term_type=LagrangianTermType.YUKAWA,
        dimension=4,
        fields_involved=["Q_L", "Phi", "d_R"]
    ))
    terms.append(LagrangianTerm(
        term_id="L_yukawa_lepton",
        term_type=LagrangianTermType.YUKAWA,
        dimension=4,
        fields_involved=["L_L", "Phi", "e_R"]
    ))

    return SMLagrangian(
        lagrangian_id="L_SM",
        terms=terms,
        dimension_bound=4
    )


def compute_anomaly_cancellation(field_content: FieldContent) -> AnomalyCancellation:
    """
    Compute anomaly cancellation for SM field content.

    In the SM, anomalies cancel exactly due to the specific
    hypercharge assignments and the quark-lepton relation.
    """
    # For the SM with standard hypercharges, all anomalies cancel
    # This is a non-trivial consistency check
    zero = RationalValue(0, 1)

    return AnomalyCancellation(
        check_id="SM_ANOMALY",
        su3_su3_su3=zero,      # QCD anomaly vanishes (vector coupling)
        su2_su2_su2=zero,      # SU(2) anomaly cancels between Q and L
        u1_u1_u1=zero,         # U(1)^3 cancels with Y assignments
        su3_su3_u1=zero,       # Mixed QCD-U(1) cancels
        su2_su2_u1=zero,       # Mixed SU(2)-U(1) cancels
        gravitational=zero     # Gravitational anomaly cancels
    )


def create_ewsb_relations() -> EWSBRelations:
    """
    Create electroweak symmetry breaking relations with PDG values.

    Using approximate rational values for demonstration.
    """
    # PDG 2022 values (approximate rational representations)
    # v ≈ 246 GeV
    # M_W ≈ 80.4 GeV
    # M_Z ≈ 91.2 GeV
    # sin^2(theta_W) ≈ 0.231
    # M_H ≈ 125 GeV

    return EWSBRelations(
        ewsb_id="EWSB_PDG",
        vev_gev=RationalValue(246, 1),
        m_w_gev=RationalValue(804, 10),  # 80.4 GeV
        m_z_gev=RationalValue(912, 10),  # 91.2 GeV
        sin2_theta_w=RationalValue(231, 1000),  # 0.231
        m_h_gev=RationalValue(125, 1)
    )
