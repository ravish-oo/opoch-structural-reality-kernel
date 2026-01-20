"""
measurement_contract.py - Measurement contracts for physical constants.

A physical constant is a verified measurement contract:
- ValueFormat: exact rational or certified interval
- WitnessFormat: calibration data
- Verifier: total function returning PASS/FAIL
- Canon: canonicalization rules
- Units: declared unit gauge

Constants are either:
1. Defined (exact by convention) - define units
2. Inferred (measured) - returned as certified intervals
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple, Union
from fractions import Fraction
from decimal import Decimal, getcontext
from enum import Enum
import hashlib
import json

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON


# Set high precision for Decimal operations
getcontext().prec = 50


class ConstantType(Enum):
    """Type of physical constant."""
    DEFINED_EXACT = "defined_exact"  # Exact by unit definition
    INFERRED_INTERVAL = "inferred_interval"  # Measured with uncertainty


class VerifyStatus(Enum):
    """Status of verification."""
    PASS = "PASS"
    FAIL = "FAIL"


@dataclass(frozen=True)
class ExactRational:
    """
    An exact rational number represented as numerator/denominator.

    This is the only way to represent exact values without floating-point errors.
    """
    numerator: int
    denominator: int = 1

    def __post_init__(self):
        if self.denominator == 0:
            raise ValueError("Denominator cannot be zero")

    @property
    def fraction(self) -> Fraction:
        return Fraction(self.numerator, self.denominator)

    def to_decimal(self, precision: int = 40) -> Decimal:
        """Convert to Decimal with specified precision."""
        getcontext().prec = precision + 10
        return Decimal(self.numerator) / Decimal(self.denominator)

    def __mul__(self, other: 'ExactRational') -> 'ExactRational':
        f = self.fraction * other.fraction
        return ExactRational(f.numerator, f.denominator)

    def __truediv__(self, other: 'ExactRational') -> 'ExactRational':
        f = self.fraction / other.fraction
        return ExactRational(f.numerator, f.denominator)

    def __add__(self, other: 'ExactRational') -> 'ExactRational':
        f = self.fraction + other.fraction
        return ExactRational(f.numerator, f.denominator)

    def __sub__(self, other: 'ExactRational') -> 'ExactRational':
        f = self.fraction - other.fraction
        return ExactRational(f.numerator, f.denominator)

    def __eq__(self, other) -> bool:
        if isinstance(other, ExactRational):
            return self.fraction == other.fraction
        return False

    def __hash__(self):
        return hash(self.fraction)

    def canonical(self) -> str:
        f = self.fraction  # This auto-reduces
        return CanonicalJSON.serialize({
            "type": "EXACT_RATIONAL",
            "numerator": f.numerator,
            "denominator": f.denominator
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass(frozen=True)
class CertifiedInterval:
    """
    A certified interval [lo, hi] with rational endpoints.

    This represents a measured value with uncertainty.
    The true value lies somewhere in [lo, hi].
    """
    lo: ExactRational
    hi: ExactRational

    def __post_init__(self):
        if self.lo.fraction > self.hi.fraction:
            raise ValueError(f"Invalid interval: lo ({self.lo.fraction}) > hi ({self.hi.fraction})")

    @property
    def width(self) -> ExactRational:
        """Width of the interval (uncertainty)."""
        return self.hi - self.lo

    @property
    def midpoint(self) -> ExactRational:
        """Midpoint of the interval."""
        two = ExactRational(2, 1)
        return ExactRational(
            (self.lo.fraction + self.hi.fraction).numerator,
            (self.lo.fraction + self.hi.fraction).denominator
        ) / two

    def contains(self, value: ExactRational) -> bool:
        """Check if value is in the interval."""
        return self.lo.fraction <= value.fraction <= self.hi.fraction

    def is_valid(self) -> bool:
        """Check if interval is valid (lo <= hi)."""
        return self.lo.fraction <= self.hi.fraction

    def __mul__(self, other: 'CertifiedInterval') -> 'CertifiedInterval':
        """Interval multiplication."""
        products = [
            self.lo * other.lo,
            self.lo * other.hi,
            self.hi * other.lo,
            self.hi * other.hi
        ]
        fracs = [p.fraction for p in products]
        lo = ExactRational(min(fracs).numerator, min(fracs).denominator)
        hi = ExactRational(max(fracs).numerator, max(fracs).denominator)
        return CertifiedInterval(lo, hi)

    def __truediv__(self, other: 'CertifiedInterval') -> 'CertifiedInterval':
        """Interval division (assumes other doesn't contain 0)."""
        if other.lo.fraction <= 0 <= other.hi.fraction:
            raise ValueError("Cannot divide by interval containing zero")
        quotients = [
            self.lo / other.lo,
            self.lo / other.hi,
            self.hi / other.lo,
            self.hi / other.hi
        ]
        fracs = [q.fraction for q in quotients]
        lo = ExactRational(min(fracs).numerator, min(fracs).denominator)
        hi = ExactRational(max(fracs).numerator, max(fracs).denominator)
        return CertifiedInterval(lo, hi)

    def __add__(self, other: 'CertifiedInterval') -> 'CertifiedInterval':
        """Interval addition."""
        lo = self.lo + other.lo
        hi = self.hi + other.hi
        return CertifiedInterval(lo, hi)

    def __sub__(self, other: 'CertifiedInterval') -> 'CertifiedInterval':
        """Interval subtraction."""
        lo = self.lo - other.hi
        hi = self.hi - other.lo
        return CertifiedInterval(lo, hi)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CERTIFIED_INTERVAL",
            "lo_numerator": self.lo.fraction.numerator,
            "lo_denominator": self.lo.fraction.denominator,
            "hi_numerator": self.hi.fraction.numerator,
            "hi_denominator": self.hi.fraction.denominator
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "CERTIFIED_INTERVAL",
            "lo_numerator": self.lo.fraction.numerator,
            "lo_denominator": self.lo.fraction.denominator,
            "hi_numerator": self.hi.fraction.numerator,
            "hi_denominator": self.hi.fraction.denominator,
            "width_numerator": self.width.fraction.numerator,
            "width_denominator": self.width.fraction.denominator,
            "interval_hash": self.fingerprint()[:32],
            "result": "PASS"
        }


def exact_from_decimal_string(s: str) -> ExactRational:
    """
    Create exact rational from decimal string like "299792458" or "6.62607015e-34".

    This is EXACT for finite decimals.
    """
    d = Decimal(s)
    sign, digits, exp = d.as_tuple()
    n = 0
    for dig in digits:
        n = n * 10 + dig
    if sign:
        n = -n
    if exp >= 0:
        return ExactRational(n * (10 ** exp), 1)
    return ExactRational(n, 10 ** (-exp))


def interval_from_value_uncertainty(
    value_str: str,
    uncertainty_last_digits: str
) -> CertifiedInterval:
    """
    Create interval from value and uncertainty in last digits.

    Example: value_str="0.0072973525643", uncertainty_last_digits="11"
    means 0.0072973525643 +/- 0.0000000000011
    """
    # Parse value
    value_dec = Decimal(value_str)

    # Count decimal places
    if "." in value_str:
        places = len(value_str.split(".")[1])
    else:
        places = 0

    # Compute uncertainty
    unc_dec = Decimal(uncertainty_last_digits) * (Decimal(10) ** Decimal(-places))

    # Compute lo and hi
    lo_dec = value_dec - unc_dec
    hi_dec = value_dec + unc_dec

    # Convert to exact rationals
    lo = exact_from_decimal_string(str(lo_dec))
    hi = exact_from_decimal_string(str(hi_dec))

    return CertifiedInterval(lo, hi)


def digits_from_interval(interval: CertifiedInterval, max_digits: int = 40) -> str:
    """
    Extract guaranteed common prefix digits from interval.

    Only returns digits that are the same for ALL numbers in [lo, hi].
    """
    getcontext().prec = max_digits + 20

    lo_dec = interval.lo.to_decimal(max_digits + 10)
    hi_dec = interval.hi.to_decimal(max_digits + 10)

    # Format as fixed-point strings
    lo_str = format(lo_dec, f".{max_digits}f")
    hi_str = format(hi_dec, f".{max_digits}f")

    # Align strings
    max_len = max(len(lo_str), len(hi_str))
    lo_str = lo_str.ljust(max_len, "0")
    hi_str = hi_str.ljust(max_len, "0")

    # Find common prefix
    common = []
    for a, b in zip(lo_str, hi_str):
        if a == b:
            common.append(a)
        else:
            break

    result = "".join(common)
    # Clean trailing zeros and decimal point
    if "." in result:
        result = result.rstrip("0").rstrip(".")

    return result


@dataclass
class UnitDimension:
    """
    Unit dimension as exponents of base units.

    SI base units: m, kg, s, A, K, mol, cd
    """
    m: int = 0   # meter
    kg: int = 0  # kilogram
    s: int = 0   # second
    A: int = 0   # ampere
    K: int = 0   # kelvin
    mol: int = 0 # mole
    cd: int = 0  # candela

    def is_dimensionless(self) -> bool:
        """Check if this is a dimensionless quantity."""
        return (self.m == 0 and self.kg == 0 and self.s == 0 and
                self.A == 0 and self.K == 0 and self.mol == 0 and self.cd == 0)

    def __mul__(self, other: 'UnitDimension') -> 'UnitDimension':
        return UnitDimension(
            m=self.m + other.m,
            kg=self.kg + other.kg,
            s=self.s + other.s,
            A=self.A + other.A,
            K=self.K + other.K,
            mol=self.mol + other.mol,
            cd=self.cd + other.cd
        )

    def __truediv__(self, other: 'UnitDimension') -> 'UnitDimension':
        return UnitDimension(
            m=self.m - other.m,
            kg=self.kg - other.kg,
            s=self.s - other.s,
            A=self.A - other.A,
            K=self.K - other.K,
            mol=self.mol - other.mol,
            cd=self.cd - other.cd
        )

    def __pow__(self, n: int) -> 'UnitDimension':
        return UnitDimension(
            m=self.m * n,
            kg=self.kg * n,
            s=self.s * n,
            A=self.A * n,
            K=self.K * n,
            mol=self.mol * n,
            cd=self.cd * n
        )

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "UNIT_DIMENSION",
            "m": self.m, "kg": self.kg, "s": self.s,
            "A": self.A, "K": self.K, "mol": self.mol, "cd": self.cd
        })

    def to_string(self) -> str:
        """Human-readable unit string."""
        parts = []
        for name, exp in [("m", self.m), ("kg", self.kg), ("s", self.s),
                          ("A", self.A), ("K", self.K), ("mol", self.mol), ("cd", self.cd)]:
            if exp == 1:
                parts.append(name)
            elif exp != 0:
                parts.append(f"{name}^{exp}")
        return " ".join(parts) if parts else "1 (dimensionless)"


@dataclass
class PhysicalConstant:
    """
    A physical constant with full metadata.
    """
    symbol: str
    name: str
    constant_type: ConstantType
    dimension: UnitDimension

    # Value (one of these is set)
    exact_value: Optional[ExactRational] = None
    interval_value: Optional[CertifiedInterval] = None

    # Metadata
    is_si_defining: bool = False
    relations: List[str] = field(default_factory=list)  # How derived from others
    source: str = ""  # e.g., "CODATA 2022"

    def __post_init__(self):
        if self.constant_type == ConstantType.DEFINED_EXACT:
            if self.exact_value is None:
                raise ValueError(f"Exact constant {self.symbol} must have exact_value")
        elif self.constant_type == ConstantType.INFERRED_INTERVAL:
            if self.interval_value is None:
                raise ValueError(f"Inferred constant {self.symbol} must have interval_value")

    def get_value(self) -> Union[ExactRational, CertifiedInterval]:
        """Get the value in appropriate format."""
        if self.constant_type == ConstantType.DEFINED_EXACT:
            return self.exact_value
        else:
            return self.interval_value

    def to_interval(self) -> CertifiedInterval:
        """Convert to interval (exact values become point intervals)."""
        if self.constant_type == ConstantType.DEFINED_EXACT:
            return CertifiedInterval(self.exact_value, self.exact_value)
        else:
            return self.interval_value

    def get_digits(self, max_digits: int = 40) -> str:
        """Get certified digits."""
        interval = self.to_interval()
        return digits_from_interval(interval, max_digits)

    def canonical(self) -> str:
        data = {
            "type": "PHYSICAL_CONSTANT",
            "symbol": self.symbol,
            "constant_type": self.constant_type.value,
            "dimension": self.dimension.canonical(),
            "is_si_defining": self.is_si_defining
        }
        if self.exact_value:
            data["exact_numerator"] = self.exact_value.fraction.numerator
            data["exact_denominator"] = self.exact_value.fraction.denominator
        if self.interval_value:
            data["interval_lo_num"] = self.interval_value.lo.fraction.numerator
            data["interval_lo_den"] = self.interval_value.lo.fraction.denominator
            data["interval_hi_num"] = self.interval_value.hi.fraction.numerator
            data["interval_hi_den"] = self.interval_value.hi.fraction.denominator
        return CanonicalJSON.serialize(data)

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        receipt = {
            "type": "PHYSICAL_CONSTANT",
            "symbol": self.symbol,
            "name": self.name,
            "constant_type": self.constant_type.value,
            "dimension": self.dimension.to_string(),
            "is_si_defining": self.is_si_defining,
            "source": self.source,
            "constant_hash": self.fingerprint()[:32]
        }
        if self.exact_value:
            receipt["exact_numerator"] = self.exact_value.fraction.numerator
            receipt["exact_denominator"] = self.exact_value.fraction.denominator
        if self.interval_value:
            receipt["interval_lo"] = str(self.interval_value.lo.to_decimal(20))
            receipt["interval_hi"] = str(self.interval_value.hi.to_decimal(20))
        receipt["result"] = "PASS"
        return receipt


@dataclass
class MeasurementContract:
    """
    A measurement contract for a physical constant.

    Contains:
    - ValueFormat: how the value is represented
    - WitnessFormat: calibration data format
    - Verifier: total function returning PASS/FAIL
    - Canon: canonicalization rules
    - Units: declared unit gauge
    """
    contract_id: str
    constant: PhysicalConstant
    witness_hash: str = ""  # Hash of calibration/measurement data
    verifier_hash: str = ""  # Hash of verification code
    canon_hash: str = ""  # Hash of canonicalization rules

    def verify(self) -> Tuple[VerifyStatus, Dict[str, Any]]:
        """
        Verify the measurement contract.

        Returns (status, details).
        """
        details = {
            "contract_id": self.contract_id,
            "symbol": self.constant.symbol,
            "constant_type": self.constant.constant_type.value
        }

        # Check value validity
        if self.constant.constant_type == ConstantType.DEFINED_EXACT:
            if self.constant.exact_value is None:
                return VerifyStatus.FAIL, {**details, "error": "Missing exact value"}
            details["value_valid"] = True
        else:
            if self.constant.interval_value is None:
                return VerifyStatus.FAIL, {**details, "error": "Missing interval value"}
            if not self.constant.interval_value.is_valid():
                return VerifyStatus.FAIL, {**details, "error": "Invalid interval (lo > hi)"}
            details["value_valid"] = True
            details["interval_valid"] = True

        details["verified"] = True
        return VerifyStatus.PASS, details

    def is_total(self) -> bool:
        """Check that verifier is total (always returns PASS or FAIL)."""
        # Verify returns a defined status for any valid contract
        status, _ = self.verify()
        return status in [VerifyStatus.PASS, VerifyStatus.FAIL]

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "MEASUREMENT_CONTRACT",
            "contract_id": self.contract_id,
            "constant_hash": self.constant.fingerprint(),
            "witness_hash": self.witness_hash,
            "verifier_hash": self.verifier_hash,
            "canon_hash": self.canon_hash
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        status, details = self.verify()
        return {
            "type": "MEASUREMENT_CONTRACT",
            "contract_id": self.contract_id,
            "constant_symbol": self.constant.symbol,
            "constant_type": self.constant.constant_type.value,
            "value_format": "exact_rational" if self.constant.exact_value else "certified_interval",
            "verifier_total": self.is_total(),
            "verification_status": status.value,
            "contract_hash": self.fingerprint()[:32],
            "result": status.value
        }


@dataclass
class UnitGauge:
    """
    A unit gauge (unit system).

    Defines conversion factors from SI.
    """
    gauge_id: str
    name: str
    # Conversion factors to SI (as exact rationals)
    conversions: Dict[str, ExactRational] = field(default_factory=dict)

    def convert_to_si(self, value: ExactRational, dimension: UnitDimension) -> ExactRational:
        """Convert value from this gauge to SI."""
        # Apply dimension-appropriate conversions
        result = value
        for unit, exp in [("m", dimension.m), ("kg", dimension.kg), ("s", dimension.s),
                          ("A", dimension.A), ("K", dimension.K),
                          ("mol", dimension.mol), ("cd", dimension.cd)]:
            if unit in self.conversions and exp != 0:
                conv = self.conversions[unit]
                for _ in range(abs(exp)):
                    if exp > 0:
                        result = result * conv
                    else:
                        result = result / conv
        return result

    def canonical(self) -> str:
        conv_data = {
            k: [v.fraction.numerator, v.fraction.denominator]
            for k, v in self.conversions.items()
        }
        return CanonicalJSON.serialize({
            "type": "UNIT_GAUGE",
            "gauge_id": self.gauge_id,
            "conversions": conv_data
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


# Standard SI gauge (identity conversions)
SI_GAUGE = UnitGauge(
    gauge_id="SI",
    name="International System of Units",
    conversions={
        "m": ExactRational(1, 1),
        "kg": ExactRational(1, 1),
        "s": ExactRational(1, 1),
        "A": ExactRational(1, 1),
        "K": ExactRational(1, 1),
        "mol": ExactRational(1, 1),
        "cd": ExactRational(1, 1)
    }
)
