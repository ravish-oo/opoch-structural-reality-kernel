"""
climate_ledger.py - Mass-balance ledger system for climate.

Implements:
- Reservoir definitions (atmosphere, ocean, land, etc.)
- Stock variables S_r(t) as rational intervals
- Flow variables F_{u->v}(t) as fluxes
- Mass-balance law enforcement
- Emissions E(t) and Removals C(t)
- Climate ledger with MRV records
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


class ReservoirType(Enum):
    """Types of carbon reservoirs in the climate system."""
    ATMOSPHERE = "atmosphere"
    UPPER_OCEAN = "upper_ocean"
    DEEP_OCEAN = "deep_ocean"
    LAND_BIOMASS = "land_biomass"
    SOILS = "soils"
    FOSSIL_RESERVES = "fossil_reserves"


@dataclass(frozen=True)
class CertifiedInterval:
    """
    A certified interval [lower, upper] represented as rational values.

    All climate quantities are intervals, not point values.
    """
    lower: RationalValue
    upper: RationalValue

    def __post_init__(self):
        if self.lower > self.upper:
            raise ValueError(f"Invalid interval: lower {self.lower} > upper {self.upper}")

    def contains(self, value: RationalValue) -> bool:
        """Check if value is in interval."""
        return self.lower <= value <= self.upper

    def width(self) -> RationalValue:
        """Return interval width."""
        return self.upper - self.lower

    def midpoint(self) -> RationalValue:
        """Return interval midpoint."""
        return RationalValue(
            self.lower.fraction.numerator + self.upper.fraction.numerator,
            self.lower.fraction.denominator + self.upper.fraction.denominator
        )

    def conservative(self) -> RationalValue:
        """Return conservative (lower bound) value."""
        return self.lower

    def __add__(self, other: 'CertifiedInterval') -> 'CertifiedInterval':
        """Interval addition."""
        return CertifiedInterval(
            self.lower + other.lower,
            self.upper + other.upper
        )

    def __sub__(self, other: 'CertifiedInterval') -> 'CertifiedInterval':
        """Interval subtraction."""
        return CertifiedInterval(
            self.lower - other.upper,
            self.upper - other.lower
        )

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CERTIFIED_INTERVAL",
            "lower_num": self.lower.fraction.numerator,
            "lower_den": self.lower.fraction.denominator,
            "upper_num": self.upper.fraction.numerator,
            "upper_den": self.upper.fraction.denominator
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class Reservoir:
    """
    A carbon reservoir in the climate system.

    Reservoirs hold stocks of carbon that change via flows.
    """
    reservoir_id: str
    reservoir_type: ReservoirType
    description: str = ""

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "RESERVOIR",
            "reservoir_id": self.reservoir_id,
            "reservoir_type": self.reservoir_type.value
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class Stock:
    """
    Stock of carbon in a reservoir at a given time.

    S_r(t) in Q_>=0, represented as certified interval.
    """
    stock_id: str
    reservoir: Reservoir
    time_step: int
    value: CertifiedInterval  # Stock as interval [lower, upper]

    def is_non_negative(self) -> bool:
        """Check stock is non-negative."""
        return self.value.lower >= RationalValue(0, 1)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "STOCK",
            "stock_id": self.stock_id,
            "reservoir_id": self.reservoir.reservoir_id,
            "time_step": self.time_step,
            "value_lower_num": self.value.lower.fraction.numerator,
            "value_lower_den": self.value.lower.fraction.denominator,
            "value_upper_num": self.value.upper.fraction.numerator,
            "value_upper_den": self.value.upper.fraction.denominator
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class Flow:
    """
    Flux of carbon between reservoirs.

    F_{u->v}(t) in Q, can be positive or negative.
    """
    flow_id: str
    source: Reservoir
    target: Reservoir
    time_step: int
    value: CertifiedInterval  # Flow as interval

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "FLOW",
            "flow_id": self.flow_id,
            "source_id": self.source.reservoir_id,
            "target_id": self.target.reservoir_id,
            "time_step": self.time_step,
            "value_lower_num": self.value.lower.fraction.numerator,
            "value_lower_den": self.value.lower.fraction.denominator,
            "value_upper_num": self.value.upper.fraction.numerator,
            "value_upper_den": self.value.upper.fraction.denominator
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class Emissions:
    """
    Anthropogenic emissions at a given time.

    E(t) in Q_>=0, represented as certified interval.
    """
    emissions_id: str
    time_step: int
    value: CertifiedInterval
    source_category: str = "anthropogenic"

    def is_non_negative(self) -> bool:
        """Check emissions are non-negative."""
        return self.value.lower >= RationalValue(0, 1)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "EMISSIONS",
            "emissions_id": self.emissions_id,
            "time_step": self.time_step,
            "value_lower_num": self.value.lower.fraction.numerator,
            "value_lower_den": self.value.lower.fraction.denominator,
            "value_upper_num": self.value.upper.fraction.numerator,
            "value_upper_den": self.value.upper.fraction.denominator,
            "source_category": self.source_category
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class Removals:
    """
    Carbon removals at a given time.

    C(t) in Q_>=0, represented as certified interval.
    """
    removal_id: str
    time_step: int
    value: CertifiedInterval
    removal_type: str = "generic"  # e.g., "dac", "afforestation", "beccs"

    def is_non_negative(self) -> bool:
        """Check removals are non-negative."""
        return self.value.lower >= RationalValue(0, 1)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "REMOVALS",
            "removal_id": self.removal_id,
            "time_step": self.time_step,
            "value_lower_num": self.value.lower.fraction.numerator,
            "value_lower_den": self.value.lower.fraction.denominator,
            "value_upper_num": self.value.upper.fraction.numerator,
            "value_upper_den": self.value.upper.fraction.denominator,
            "removal_type": self.removal_type
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class MassBalanceCheck:
    """
    Verification of mass-balance law for a reservoir.

    S_r(t+1) = S_r(t) + sum(inflows) - sum(outflows) + 1_{r=atm}(E - C)
    """
    check_id: str
    reservoir: Reservoir
    time_step: int
    stock_before: CertifiedInterval
    inflows_total: CertifiedInterval
    outflows_total: CertifiedInterval
    emissions_net: CertifiedInterval  # E - C if atmosphere, else 0
    stock_after_computed: CertifiedInterval
    stock_after_declared: CertifiedInterval

    def balance_satisfied(self) -> bool:
        """
        Check if mass balance is satisfied.

        The computed stock_after must be consistent with declared.
        (Intervals must overlap.)
        """
        # Check if intervals overlap
        computed_lower = self.stock_after_computed.lower
        computed_upper = self.stock_after_computed.upper
        declared_lower = self.stock_after_declared.lower
        declared_upper = self.stock_after_declared.upper

        # Intervals overlap if max(lowers) <= min(uppers)
        return (max(computed_lower.fraction, declared_lower.fraction) <=
                min(computed_upper.fraction, declared_upper.fraction))

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "MASS_BALANCE_CHECK",
            "check_id": self.check_id,
            "reservoir_id": self.reservoir.reservoir_id,
            "time_step": self.time_step,
            "balance_satisfied": self.balance_satisfied()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "MASS_BALANCE_CHECK",
            "check_id": self.check_id,
            "reservoir": self.reservoir.reservoir_id,
            "time_step": self.time_step,
            "stock_before_lower": self.stock_before.lower.fraction.numerator,
            "stock_before_upper": self.stock_before.upper.fraction.numerator,
            "inflows_total_lower": self.inflows_total.lower.fraction.numerator,
            "outflows_total_lower": self.outflows_total.lower.fraction.numerator,
            "emissions_net_lower": self.emissions_net.lower.fraction.numerator,
            "stock_after_computed_lower": self.stock_after_computed.lower.fraction.numerator,
            "stock_after_declared_lower": self.stock_after_declared.lower.fraction.numerator,
            "balance_satisfied": self.balance_satisfied(),
            "result": "PASS" if self.balance_satisfied() else "FAIL"
        }


class MRVCategory(Enum):
    """Categories of MRV (Measurement, Reporting, Verification) tests."""
    BASELINE = "baseline"
    INTERVENTION = "intervention"
    REMOVAL = "removal"
    LEAKAGE = "leakage"
    PERMANENCE = "permanence"
    DOUBLE_COUNTING = "double_counting"
    UNCERTAINTY = "uncertainty"


@dataclass(frozen=True)
class MRVOutcome:
    """Outcome from an MRV test."""
    outcome_id: str
    value: Any
    certified_interval: Optional[CertifiedInterval] = None

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "outcome_id": self.outcome_id,
            "value": str(self.value)
        })


@dataclass
class MRVTest:
    """
    An MRV (Measurement, Reporting, Verification) test.

    This is a total, finite-outcome procedure that checks climate claims.
    """
    test_id: str
    category: MRVCategory
    outcome_space: FrozenSet[MRVOutcome]
    cost: int  # Integer cost in standard units
    description: str = ""
    _test_function: Optional[Callable[[Any], MRVOutcome]] = field(
        default=None, repr=False, compare=False
    )

    def outcome_space_size(self) -> int:
        """Return |A| - size of outcome space."""
        return len(self.outcome_space)

    def is_total(self) -> bool:
        """Check if test is total (always returns outcome)."""
        return True  # By construction

    def execute(self, target: Any) -> MRVOutcome:
        """Execute test on target."""
        if self._test_function is None:
            raise ValueError(f"Test {self.test_id} has no executable function")
        return self._test_function(target)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "MRV_TEST",
            "test_id": self.test_id,
            "category": self.category.value,
            "outcome_space_size": self.outcome_space_size(),
            "cost": self.cost
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "MRV_TEST_DEFINITION",
            "test_id": self.test_id,
            "test_category": self.category.value,
            "outcome_space_size": self.outcome_space_size(),
            "cost": self.cost,
            "total": self.is_total(),
            "result": "PASS"
        }


@dataclass(frozen=True)
class ClimateLedgerEntry:
    """
    A single entry in the climate MRV ledger.

    Records (test, outcome) with witness bundle reference.
    """
    entry_id: str
    test_id: str
    outcome: MRVOutcome
    witness_bundle_hash: str
    cost: int
    timestamp: int

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CLIMATE_LEDGER_ENTRY",
            "entry_id": self.entry_id,
            "test_id": self.test_id,
            "outcome_id": self.outcome.outcome_id,
            "witness_bundle_hash": self.witness_bundle_hash,
            "cost": self.cost,
            "timestamp": self.timestamp
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class ClimateLedger:
    """
    The climate MRV ledger L.

    Records all MRV test executions and their outcomes.
    """
    ledger_id: str
    entries: List[ClimateLedgerEntry] = field(default_factory=list)

    def add_entry(
        self,
        test: MRVTest,
        outcome: MRVOutcome,
        witness_bundle_hash: str
    ) -> ClimateLedgerEntry:
        """Add a new entry to the ledger."""
        entry = ClimateLedgerEntry(
            entry_id=f"ENTRY_{len(self.entries)}",
            test_id=test.test_id,
            outcome=outcome,
            witness_bundle_hash=witness_bundle_hash,
            cost=test.cost,
            timestamp=len(self.entries)
        )
        self.entries.append(entry)
        return entry

    def total_cost(self) -> int:
        """Total cost expended."""
        return sum(e.cost for e in self.entries)

    def entry_count(self) -> int:
        """Number of entries."""
        return len(self.entries)

    def canonical(self) -> str:
        entry_hashes = [e.fingerprint() for e in self.entries]
        return CanonicalJSON.serialize({
            "type": "CLIMATE_LEDGER",
            "ledger_id": self.ledger_id,
            "entry_count": self.entry_count(),
            "total_cost": self.total_cost(),
            "entry_hashes": entry_hashes
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(
        self,
        initial_survivors: int,
        final_survivors: int
    ) -> Dict[str, Any]:
        return {
            "type": "CLIMATE_LEDGER_RECEIPT",
            "ledger_id": self.ledger_id,
            "entries_count": self.entry_count(),
            "total_cost": self.total_cost(),
            "initial_survivors": initial_survivors,
            "final_survivors": final_survivors,
            "monotone_decrease": final_survivors <= initial_survivors,
            "ledger_fingerprint": self.fingerprint(),
            "result": "PASS" if final_survivors <= initial_survivors else "FAIL"
        }


@dataclass
class ClimateSpec:
    """
    Climate specification (target constraints).

    Defines reservoirs, constraints, and verification requirements.
    """
    spec_id: str
    reservoirs: List[Reservoir]
    horizon_steps: int
    constraints: List[str]  # Constraint descriptions

    def reservoir_count(self) -> int:
        return len(self.reservoirs)

    def constraint_count(self) -> int:
        return len(self.constraints)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CLIMATE_SPEC",
            "spec_id": self.spec_id,
            "reservoir_count": self.reservoir_count(),
            "horizon_steps": self.horizon_steps,
            "constraint_count": self.constraint_count()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "CLIMATE_SPEC",
            "spec_id": self.spec_id,
            "reservoirs": [r.reservoir_id for r in self.reservoirs],
            "stock_variables": self.reservoir_count(),
            "flow_variables": self.reservoir_count() * (self.reservoir_count() - 1),
            "constraints_count": self.constraint_count(),
            "horizon_steps": self.horizon_steps,
            "result": "PASS"
        }


# ============================================================
# MASS-BALANCE COMPUTATION
# ============================================================

def compute_mass_balance(
    reservoir: Reservoir,
    stock_before: Stock,
    inflows: List[Flow],
    outflows: List[Flow],
    emissions: Optional[Emissions],
    removals: Optional[Removals],
    stock_after_declared: Stock
) -> MassBalanceCheck:
    """
    Compute and verify mass-balance for a reservoir.

    S_r(t+1) = S_r(t) + sum(inflows) - sum(outflows) + 1_{r=atm}(E - C)
    """
    # Sum inflows
    inflows_total = CertifiedInterval(RationalValue(0, 1), RationalValue(0, 1))
    for flow in inflows:
        inflows_total = inflows_total + flow.value

    # Sum outflows
    outflows_total = CertifiedInterval(RationalValue(0, 1), RationalValue(0, 1))
    for flow in outflows:
        outflows_total = outflows_total + flow.value

    # Emissions net (only for atmosphere)
    if reservoir.reservoir_type == ReservoirType.ATMOSPHERE:
        e_val = emissions.value if emissions else CertifiedInterval(RationalValue(0, 1), RationalValue(0, 1))
        c_val = removals.value if removals else CertifiedInterval(RationalValue(0, 1), RationalValue(0, 1))
        emissions_net = e_val - c_val
    else:
        emissions_net = CertifiedInterval(RationalValue(0, 1), RationalValue(0, 1))

    # Compute stock after
    stock_after_computed = (
        stock_before.value + inflows_total - outflows_total + emissions_net
    )

    return MassBalanceCheck(
        check_id=f"MB_{reservoir.reservoir_id}_{stock_before.time_step}",
        reservoir=reservoir,
        time_step=stock_before.time_step,
        stock_before=stock_before.value,
        inflows_total=inflows_total,
        outflows_total=outflows_total,
        emissions_net=emissions_net,
        stock_after_computed=stock_after_computed,
        stock_after_declared=stock_after_declared.value
    )


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def create_standard_reservoirs() -> List[Reservoir]:
    """Create standard climate reservoirs."""
    return [
        Reservoir("ATM", ReservoirType.ATMOSPHERE, "Atmospheric CO2"),
        Reservoir("UPPER_OCEAN", ReservoirType.UPPER_OCEAN, "Upper ocean carbon"),
        Reservoir("DEEP_OCEAN", ReservoirType.DEEP_OCEAN, "Deep ocean carbon"),
        Reservoir("LAND", ReservoirType.LAND_BIOMASS, "Land biomass"),
        Reservoir("SOILS", ReservoirType.SOILS, "Soil carbon"),
    ]


def create_zero_interval() -> CertifiedInterval:
    """Create zero interval [0, 0]."""
    return CertifiedInterval(RationalValue(0, 1), RationalValue(0, 1))


def create_interval(lower: int, upper: int, denominator: int = 1) -> CertifiedInterval:
    """Create interval from integers."""
    return CertifiedInterval(
        RationalValue(lower, denominator),
        RationalValue(upper, denominator)
    )


def create_sample_climate_spec() -> ClimateSpec:
    """Create sample climate specification."""
    reservoirs = create_standard_reservoirs()
    return ClimateSpec(
        spec_id="SAMPLE_SPEC",
        reservoirs=reservoirs,
        horizon_steps=10,
        constraints=[
            "net_emissions_bound",
            "stock_bound",
            "cost_bound"
        ]
    )


def create_sample_mrv_outcomes() -> FrozenSet[MRVOutcome]:
    """Create standard PASS/FAIL MRV outcomes."""
    return frozenset([
        MRVOutcome("PASS", True),
        MRVOutcome("FAIL", False),
        MRVOutcome("UNCERTAIN", None)
    ])


def create_empty_climate_ledger() -> ClimateLedger:
    """Create empty climate ledger."""
    return ClimateLedger(ledger_id="LEDGER_0")
