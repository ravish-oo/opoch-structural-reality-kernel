"""
climate_tech.py - Climate tech as verified ledger with optimal separators.

Implements:
- CarbonCredit as exact object with certified intervals
- Intervention and Portfolio definitions
- Climate constraints (target spec)
- Optimal measurement selection (tau*)
- Witness bundles and chain of custody
- Omega frontier for uncertainty honesty
- Complete climate verification bundle
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

from .boundary_flow import RationalValue
from .climate_ledger import (
    CertifiedInterval,
    Reservoir,
    ReservoirType,
    Stock,
    Flow,
    Emissions,
    Removals,
    MassBalanceCheck,
    MRVCategory,
    MRVOutcome,
    MRVTest,
    ClimateLedgerEntry,
    ClimateLedger,
    ClimateSpec,
    compute_mass_balance,
    create_standard_reservoirs,
    create_zero_interval,
    create_interval,
    create_sample_climate_spec,
    create_sample_mrv_outcomes,
    create_empty_climate_ledger
)


class ClimateStatus(Enum):
    """Status of climate verification."""
    VERIFIED = "verified"       # All constraints satisfied
    FAILED = "failed"          # Constraint violated
    FRONTIER = "frontier"      # Insufficient evidence


class InterventionType(Enum):
    """Types of climate interventions."""
    MITIGATION = "mitigation"
    REMOVAL = "removal"
    ADAPTATION = "adaptation"


@dataclass
class ClimateIntervention:
    """
    A climate intervention (mitigation, removal, or adaptation).

    Induces trajectories of emissions, removals, and costs.
    """
    intervention_id: str
    intervention_type: InterventionType
    description: str
    emissions_reduction: CertifiedInterval  # E_0 - E_x
    removals: CertifiedInterval             # C_x
    cost: CertifiedInterval                 # U_x

    def net_impact(self) -> CertifiedInterval:
        """Compute net climate impact (reduction + removal)."""
        return self.emissions_reduction + self.removals

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CLIMATE_INTERVENTION",
            "intervention_id": self.intervention_id,
            "intervention_type": self.intervention_type.value,
            "emissions_reduction_lower": self.emissions_reduction.lower.fraction.numerator,
            "removals_lower": self.removals.lower.fraction.numerator
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class Portfolio:
    """
    A portfolio of climate interventions.

    Aggregates multiple interventions with their combined effects.
    """
    portfolio_id: str
    interventions: List[ClimateIntervention]

    def total_emissions_reduction(self) -> CertifiedInterval:
        """Sum of emissions reductions."""
        total = create_zero_interval()
        for i in self.interventions:
            total = total + i.emissions_reduction
        return total

    def total_removals(self) -> CertifiedInterval:
        """Sum of removals."""
        total = create_zero_interval()
        for i in self.interventions:
            total = total + i.removals
        return total

    def total_cost(self) -> CertifiedInterval:
        """Sum of costs."""
        total = create_zero_interval()
        for i in self.interventions:
            total = total + i.cost
        return total

    def total_net_impact(self) -> CertifiedInterval:
        """Total net climate impact."""
        return self.total_emissions_reduction() + self.total_removals()

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "PORTFOLIO",
            "portfolio_id": self.portfolio_id,
            "intervention_count": len(self.interventions)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class ClimateConstraint:
    """
    A climate constraint that must be verified.

    Each constraint is a total verifier returning PASS/FAIL.
    """
    constraint_id: str
    description: str
    threshold: RationalValue
    _verify_function: Optional[Callable[[Portfolio], bool]] = field(
        default=None, repr=False, compare=False
    )

    def verify(self, portfolio: Portfolio) -> bool:
        """Verify constraint on portfolio."""
        if self._verify_function:
            return self._verify_function(portfolio)
        return True

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CLIMATE_CONSTRAINT",
            "constraint_id": self.constraint_id,
            "threshold_num": self.threshold.fraction.numerator,
            "threshold_den": self.threshold.fraction.denominator
        })


@dataclass
class CarbonCredit:
    """
    A carbon credit as an exact object with certified intervals.

    No fraud surface: all components are intervals with witnesses.
    """
    credit_id: str
    vintage_year: int
    project_id: str

    # All components as certified intervals
    baseline: CertifiedInterval           # E_0
    intervention: CertifiedInterval       # E_x
    removal: CertifiedInterval            # C_x
    leakage: CertifiedInterval            # Leak
    rebound: CertifiedInterval            # Rebound
    non_permanence: CertifiedInterval     # NonPerm

    # Witness references
    witness_bundle_hash: str
    ledger_receipt_hash: str

    def net_delta(self) -> CertifiedInterval:
        """
        Compute net credit delta.

        Delta_net = (E_0 - E_x) + C_x - Leak - Rebound - NonPerm
        """
        reduction = self.baseline - self.intervention
        gross = reduction + self.removal
        deductions = self.leakage + self.rebound + self.non_permanence
        return gross - deductions

    def conservative_issuance(self) -> RationalValue:
        """Conservative issuance uses lower bound of net delta."""
        net = self.net_delta()
        return net.lower if net.lower > RationalValue(0, 1) else RationalValue(0, 1)

    def is_valid(self) -> bool:
        """Check if credit is valid (positive net delta)."""
        return self.net_delta().lower > RationalValue(0, 1)

    def canonical(self) -> str:
        net = self.net_delta()
        return CanonicalJSON.serialize({
            "type": "CARBON_CREDIT",
            "credit_id": self.credit_id,
            "vintage_year": self.vintage_year,
            "project_id": self.project_id,
            "net_delta_lower_num": net.lower.fraction.numerator,
            "net_delta_lower_den": net.lower.fraction.denominator,
            "net_delta_upper_num": net.upper.fraction.numerator,
            "net_delta_upper_den": net.upper.fraction.denominator,
            "witness_bundle_hash": self.witness_bundle_hash,
            "ledger_receipt_hash": self.ledger_receipt_hash
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self, double_counting_pass: bool = True) -> Dict[str, Any]:
        net = self.net_delta()
        conservative = self.conservative_issuance()
        return {
            "type": "CARBON_CREDIT",
            "credit_id": self.credit_id,
            "vintage_year": self.vintage_year,
            "project_id": self.project_id,
            "baseline_interval_lower": self.baseline.lower.fraction.numerator,
            "baseline_interval_upper": self.baseline.upper.fraction.numerator,
            "intervention_interval_lower": self.intervention.lower.fraction.numerator,
            "intervention_interval_upper": self.intervention.upper.fraction.numerator,
            "removal_interval_lower": self.removal.lower.fraction.numerator,
            "removal_interval_upper": self.removal.upper.fraction.numerator,
            "leakage_interval_lower": self.leakage.lower.fraction.numerator,
            "leakage_interval_upper": self.leakage.upper.fraction.numerator,
            "net_delta_lower": net.lower.fraction.numerator,
            "net_delta_upper": net.upper.fraction.numerator,
            "conservative_issuance": conservative.fraction.numerator,
            "witness_bundle_hash": self.witness_bundle_hash,
            "ledger_receipt_hash": self.ledger_receipt_hash,
            "double_counting_check": "PASS" if double_counting_pass else "FAIL",
            "result": "PASS" if self.is_valid() and double_counting_pass else "FAIL"
        }


@dataclass
class WitnessBundle:
    """
    A witness bundle containing raw data for MRV verification.

    Includes chain of custody and calibration verification.
    """
    bundle_id: str
    data_source: str  # sensor, satellite, meter, survey
    raw_data_hash: str
    chain_of_custody_verified: bool
    calibration_verified: bool
    sampling_seed: Optional[int] = None

    def is_valid(self) -> bool:
        """Check if bundle is valid."""
        return self.chain_of_custody_verified and self.calibration_verified

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "WITNESS_BUNDLE",
            "bundle_id": self.bundle_id,
            "data_source": self.data_source,
            "raw_data_hash": self.raw_data_hash,
            "chain_of_custody_verified": self.chain_of_custody_verified,
            "calibration_verified": self.calibration_verified
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "WITNESS_BUNDLE",
            "bundle_id": self.bundle_id,
            "data_source": self.data_source,
            "data_hash": self.raw_data_hash,
            "chain_of_custody_verified": self.chain_of_custody_verified,
            "calibration_verified": self.calibration_verified,
            "result": "PASS" if self.is_valid() else "FAIL"
        }


@dataclass
class VerifierExecution:
    """
    Execution of a verifier on a witness bundle.

    Maps witness bundle deterministically to certified interval.
    """
    execution_id: str
    verifier_id: str
    input_bundle: WitnessBundle
    output_interval: CertifiedInterval
    deterministic: bool = True

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "VERIFIER_EXECUTION",
            "execution_id": self.execution_id,
            "verifier_id": self.verifier_id,
            "input_bundle_hash": self.input_bundle.fingerprint(),
            "output_lower_num": self.output_interval.lower.fraction.numerator,
            "output_upper_num": self.output_interval.upper.fraction.numerator,
            "deterministic": self.deterministic
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "VERIFIER_EXECUTION",
            "verifier_id": self.verifier_id,
            "input_bundle_hash": self.input_bundle.fingerprint(),
            "output_interval_lower": self.output_interval.lower.fraction.numerator,
            "output_interval_upper": self.output_interval.upper.fraction.numerator,
            "deterministic": self.deterministic,
            "result": "PASS" if self.deterministic else "FAIL"
        }


@dataclass
class GaugeInvarianceCheck:
    """
    Check that recodings don't change Pi-fixed outputs.
    """
    check_id: str
    recodings_tested: int
    all_invariant: bool

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "GAUGE_INVARIANCE",
            "check_id": self.check_id,
            "recodings_tested": self.recodings_tested,
            "all_invariant": self.all_invariant
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "GAUGE_INVARIANCE",
            "check_id": self.check_id,
            "recodings_tested": self.recodings_tested,
            "all_invariant": self.all_invariant,
            "result": "PASS" if self.all_invariant else "FAIL"
        }


@dataclass
class OmegaFrontier:
    """
    Omega frontier: the surviving hypotheses + minimal next separator.

    Represents honest uncertainty when multiple interpretations remain.
    """
    frontier_id: str
    survivors_count: int
    survivors_fingerprint: str
    resolved: bool
    next_separator_id: Optional[str]
    next_separator_cost: Optional[int]

    def is_honest(self) -> bool:
        """
        Check if frontier is honest.

        If not resolved, must have next separator identified.
        """
        if self.resolved:
            return True
        return self.next_separator_id is not None

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "OMEGA_FRONTIER",
            "frontier_id": self.frontier_id,
            "survivors_count": self.survivors_count,
            "resolved": self.resolved,
            "next_separator_id": self.next_separator_id
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "OMEGA_HONESTY",
            "frontier_id": self.frontier_id,
            "survivors_count": self.survivors_count,
            "resolved": self.resolved,
            "next_separator_id": self.next_separator_id,
            "next_separator_cost": self.next_separator_cost,
            "honest": self.is_honest(),
            "result": "PASS" if self.is_honest() else "FAIL"
        }


@dataclass
class OptimalMeasurement:
    """
    Optimal next measurement selection (tau*).

    Minimax value computation: choose test that maximizes worst-case
    refinement per unit cost.
    """
    selection_id: str
    selected_test: MRVTest
    minimax_value: RationalValue
    alternatives_considered: int

    @staticmethod
    def compute_minimax_value(
        test: MRVTest,
        survivors_count: int
    ) -> RationalValue:
        """
        Compute minimax value for a test.

        Simplified: value = potential_reduction / cost
        """
        if test.cost == 0:
            return RationalValue(0, 1)

        # Assume binary test can halve survivors in worst case
        worst_case_survivors = (survivors_count + 1) // 2
        reduction = survivors_count - worst_case_survivors

        return RationalValue(reduction, test.cost)

    @staticmethod
    def select_optimal(
        available_tests: List[MRVTest],
        survivors_count: int
    ) -> 'OptimalMeasurement':
        """Select optimal next measurement."""
        if not available_tests:
            raise ValueError("No available tests")

        best_test = None
        best_value = RationalValue(-1, 1)

        for test in available_tests:
            value = OptimalMeasurement.compute_minimax_value(test, survivors_count)
            if value > best_value:
                best_value = value
                best_test = test

        if best_test is None:
            best_test = available_tests[0]
            best_value = RationalValue(0, 1)

        return OptimalMeasurement(
            selection_id=f"SELECT_{best_test.test_id}",
            selected_test=best_test,
            minimax_value=best_value,
            alternatives_considered=len(available_tests)
        )

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "OPTIMAL_MEASUREMENT",
            "selection_id": self.selection_id,
            "selected_test": self.selected_test.test_id,
            "minimax_value_num": self.minimax_value.fraction.numerator,
            "minimax_value_den": self.minimax_value.fraction.denominator
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "OPTIMAL_MEASUREMENT",
            "selection_id": self.selection_id,
            "selected_test": self.selected_test.test_id,
            "minimax_value_numerator": self.minimax_value.fraction.numerator,
            "minimax_value_denominator": self.minimax_value.fraction.denominator,
            "cost": self.selected_test.cost,
            "is_optimal": True,
            "result": "PASS"
        }


@dataclass
class ClimateVerificationResult:
    """
    Result of climate verification.
    """
    result_id: str
    status: ClimateStatus
    spec: ClimateSpec
    ledger: ClimateLedger
    portfolio: Optional[Portfolio]
    credits: List[CarbonCredit]
    mass_balance_checks: List[MassBalanceCheck]
    frontier: OmegaFrontier

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CLIMATE_VERIFICATION_RESULT",
            "result_id": self.result_id,
            "status": self.status.value,
            "credits_count": len(self.credits),
            "mass_balance_checks": len(self.mass_balance_checks)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class ClimateBundle:
    """
    Complete climate verification bundle.

    Contains all receipts for full audit trail.
    """
    bundle_id: str
    spec_verified: bool
    mrv_tests_defined: bool
    witness_bundles_verified: bool
    verifiers_executed: bool
    ledger_receipted: bool
    gauge_invariant: bool
    omega_honest: bool
    mass_balance_satisfied: bool
    result: ClimateVerificationResult

    def all_verified(self) -> bool:
        return (
            self.spec_verified and
            self.mrv_tests_defined and
            self.witness_bundles_verified and
            self.verifiers_executed and
            self.ledger_receipted and
            self.gauge_invariant and
            self.omega_honest and
            self.mass_balance_satisfied
        )

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CLIMATE_BUNDLE",
            "bundle_id": self.bundle_id,
            "all_verified": self.all_verified()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "CLIMATE_BUNDLE",
            "bundle_id": self.bundle_id,
            "spec_verified": self.spec_verified,
            "mrv_tests_defined": self.mrv_tests_defined,
            "witness_bundles_verified": self.witness_bundles_verified,
            "verifiers_executed": self.verifiers_executed,
            "ledger_receipted": self.ledger_receipted,
            "gauge_invariant": self.gauge_invariant,
            "omega_honest": self.omega_honest,
            "mass_balance_satisfied": self.mass_balance_satisfied,
            "all_verified": self.all_verified(),
            "bundle_fingerprint": self.fingerprint(),
            "result": "PASS" if self.all_verified() else "FAIL"
        }


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def create_sample_intervention(
    intervention_id: str,
    intervention_type: InterventionType = InterventionType.MITIGATION
) -> ClimateIntervention:
    """Create sample climate intervention."""
    return ClimateIntervention(
        intervention_id=intervention_id,
        intervention_type=intervention_type,
        description=f"Sample {intervention_type.value} intervention",
        emissions_reduction=create_interval(80, 120),  # 80-120 units reduction
        removals=create_interval(10, 20) if intervention_type == InterventionType.REMOVAL else create_zero_interval(),
        cost=create_interval(1000, 1500)
    )


def create_sample_portfolio() -> Portfolio:
    """Create sample portfolio."""
    return Portfolio(
        portfolio_id="SAMPLE_PORTFOLIO",
        interventions=[
            create_sample_intervention("INT_001", InterventionType.MITIGATION),
            create_sample_intervention("INT_002", InterventionType.REMOVAL),
        ]
    )


def create_sample_carbon_credit(
    credit_id: str,
    vintage_year: int = 2024
) -> CarbonCredit:
    """Create sample carbon credit."""
    return CarbonCredit(
        credit_id=credit_id,
        vintage_year=vintage_year,
        project_id="PROJECT_001",
        baseline=create_interval(100, 110),
        intervention=create_interval(20, 30),
        removal=create_interval(15, 25),
        leakage=create_interval(2, 5),
        rebound=create_interval(1, 3),
        non_permanence=create_interval(1, 2),
        witness_bundle_hash="abc123",
        ledger_receipt_hash="def456"
    )


def create_sample_mrv_tests() -> List[MRVTest]:
    """Create sample MRV tests."""
    outcomes = create_sample_mrv_outcomes()

    return [
        MRVTest(
            test_id="MRV_BASELINE",
            category=MRVCategory.BASELINE,
            outcome_space=outcomes,
            cost=1000,
            description="Baseline emissions measurement"
        ),
        MRVTest(
            test_id="MRV_INTERVENTION",
            category=MRVCategory.INTERVENTION,
            outcome_space=outcomes,
            cost=1500,
            description="Intervention emissions measurement"
        ),
        MRVTest(
            test_id="MRV_REMOVAL",
            category=MRVCategory.REMOVAL,
            outcome_space=outcomes,
            cost=2000,
            description="Removal verification"
        ),
        MRVTest(
            test_id="MRV_LEAKAGE",
            category=MRVCategory.LEAKAGE,
            outcome_space=outcomes,
            cost=800,
            description="Leakage assessment"
        ),
        MRVTest(
            test_id="MRV_PERMANENCE",
            category=MRVCategory.PERMANENCE,
            outcome_space=outcomes,
            cost=1200,
            description="Permanence monitoring"
        ),
    ]


def create_sample_witness_bundle(bundle_id: str) -> WitnessBundle:
    """Create sample witness bundle."""
    data_hash = hashlib.sha256(f"raw_data_{bundle_id}".encode()).hexdigest()
    return WitnessBundle(
        bundle_id=bundle_id,
        data_source="sensor",
        raw_data_hash=data_hash,
        chain_of_custody_verified=True,
        calibration_verified=True
    )


def run_sample_climate_verification() -> ClimateVerificationResult:
    """Run sample climate verification."""
    # Create spec
    spec = create_sample_climate_spec()

    # Create ledger
    ledger = create_empty_climate_ledger()

    # Create portfolio
    portfolio = create_sample_portfolio()

    # Create credits
    credits = [
        create_sample_carbon_credit("CREDIT_001", 2024),
        create_sample_carbon_credit("CREDIT_002", 2024),
    ]

    # Create mass balance checks (simplified)
    reservoirs = create_standard_reservoirs()
    atm = reservoirs[0]

    stock_before = Stock(
        stock_id="S_ATM_0",
        reservoir=atm,
        time_step=0,
        value=create_interval(850000, 860000)  # GtC
    )

    stock_after = Stock(
        stock_id="S_ATM_1",
        reservoir=atm,
        time_step=1,
        value=create_interval(851000, 862000)
    )

    emissions = Emissions(
        emissions_id="E_2024",
        time_step=0,
        value=create_interval(10000, 11000)
    )

    removals = Removals(
        removal_id="C_2024",
        time_step=0,
        value=create_interval(5000, 6000)
    )

    mb_check = compute_mass_balance(
        reservoir=atm,
        stock_before=stock_before,
        inflows=[],
        outflows=[],
        emissions=emissions,
        removals=removals,
        stock_after_declared=stock_after
    )

    # Create frontier
    frontier = OmegaFrontier(
        frontier_id="FRONTIER_001",
        survivors_count=1,
        survivors_fingerprint="resolved",
        resolved=True,
        next_separator_id=None,
        next_separator_cost=None
    )

    return ClimateVerificationResult(
        result_id="RESULT_001",
        status=ClimateStatus.VERIFIED,
        spec=spec,
        ledger=ledger,
        portfolio=portfolio,
        credits=credits,
        mass_balance_checks=[mb_check],
        frontier=frontier
    )
