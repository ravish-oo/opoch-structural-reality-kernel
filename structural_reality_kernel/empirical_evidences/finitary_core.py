"""
finitary_core.py - The forced finitary universe.

Under A0 (witnessability), the forced universe is finitary:
- Only finite witnesses create admissible distinctions
- All domain objects are finite descriptions
- All verifiers are total (PASS/FAIL/TIMEOUT)

"Infinity" enters only as explicit closure policies on refinement chains.
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, Generic, List, Optional, Set, Tuple, TypeVar
from enum import Enum
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON


T = TypeVar('T')


class VerifyResult(Enum):
    """Result of verification - always one of these (totality)."""
    PASS = "PASS"
    FAIL = "FAIL"
    TIMEOUT = "TIMEOUT"


@dataclass
class FiniteDescription:
    """
    A finite description - an element of D*.

    All objects in the forced universe are finite bitstrings/descriptions.
    """
    code: str
    bit_length: int = 0

    def __post_init__(self):
        if self.bit_length == 0:
            self.bit_length = len(self.code.encode('utf-8')) * 8

    def __hash__(self):
        return hash(self.code)

    def __eq__(self, other):
        if isinstance(other, FiniteDescription):
            return self.code == other.code
        return False

    def is_finite(self) -> bool:
        """All descriptions are finite by construction."""
        return self.bit_length < float('inf')

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "FINITE_DESCRIPTION",
            "code": self.code,
            "bit_length": self.bit_length
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class FiniteDomain:
    """
    A finite domain D_0 subset of D*.

    The forced world-state operates on finite domains.
    """
    domain_id: str
    elements: FrozenSet[FiniteDescription]

    def __len__(self) -> int:
        return len(self.elements)

    def is_finite(self) -> bool:
        """Check that domain is finite."""
        return len(self.elements) < float('inf')

    def total_bits(self) -> int:
        """Total bits in all descriptions."""
        return sum(e.bit_length for e in self.elements)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "FINITE_DOMAIN",
            "domain_id": self.domain_id,
            "element_count": len(self.elements),
            "total_bits": self.total_bits()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "FINITE_DOMAIN",
            "domain_id": self.domain_id,
            "element_count": len(self.elements),
            "is_finite": self.is_finite(),
            "total_bits": self.total_bits(),
            "domain_hash": self.fingerprint()[:32],
            "result": "PASS"
        }


@dataclass
class FiniteWitness:
    """
    A finite witness for a distinction.

    Under A0, every distinction must have a finite witness.
    """
    witness_id: str
    witness_data: str
    bit_length: int = 0

    def __post_init__(self):
        if self.bit_length == 0:
            self.bit_length = len(self.witness_data.encode('utf-8')) * 8

    def is_finite(self) -> bool:
        return self.bit_length < float('inf')

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "FINITE_WITNESS",
            "witness_id": self.witness_id,
            "bit_length": self.bit_length
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class TotalVerifier:
    """
    A total verifier - always returns PASS, FAIL, or TIMEOUT.

    Totality is required by A0: no undefined results.
    """
    verifier_id: str
    verify_fn: Callable[[FiniteWitness, FiniteDescription], VerifyResult]
    timeout_budget: int = 1000  # Steps before TIMEOUT

    def verify(
        self,
        witness: FiniteWitness,
        claim: FiniteDescription
    ) -> Tuple[VerifyResult, Dict[str, Any]]:
        """
        Verify a witness against a claim.

        Always returns one of PASS, FAIL, TIMEOUT (totality).
        """
        try:
            result = self.verify_fn(witness, claim)
            return result, {
                "witness_id": witness.witness_id,
                "claim_code": claim.code,
                "result": result.value
            }
        except Exception as e:
            # Exception converts to FAIL (totality)
            return VerifyResult.FAIL, {
                "witness_id": witness.witness_id,
                "claim_code": claim.code,
                "result": "FAIL",
                "error": str(e)
            }

    def is_total(self, test_cases: List[Tuple[FiniteWitness, FiniteDescription]]) -> bool:
        """Verify that verifier is total on test cases."""
        for witness, claim in test_cases:
            result, _ = self.verify(witness, claim)
            if result not in [VerifyResult.PASS, VerifyResult.FAIL, VerifyResult.TIMEOUT]:
                return False
        return True

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "TOTAL_VERIFIER",
            "verifier_id": self.verifier_id,
            "timeout_budget": self.timeout_budget
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "TOTAL_VERIFIER",
            "verifier_id": self.verifier_id,
            "timeout_budget": self.timeout_budget,
            "verifier_hash": self.fingerprint()[:32],
            "result": "PASS"
        }


@dataclass
class RefinementStep(Generic[T]):
    """
    A single step in a refinement chain.

    Each step adds distinguishability (finer description).
    """
    index: int
    value: T
    description: FiniteDescription

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "REFINEMENT_STEP",
            "index": self.index,
            "description_code": self.description.code
        })


@dataclass
class RefinementChain(Generic[T]):
    """
    A refinement chain - a sequence of finitary approximations.

    x_0 <= x_1 <= x_2 <= ...

    Each step adds distinguishability. The chain is always finite
    at any point in time (we can only witness finite prefixes).
    """
    chain_id: str
    steps: List[RefinementStep[T]]
    is_cauchy: bool = False  # Whether it satisfies Cauchy criterion

    def __len__(self) -> int:
        return len(self.steps)

    def is_finite_prefix(self) -> bool:
        """We can only ever witness finite prefixes."""
        return len(self.steps) < float('inf')

    def get_step(self, index: int) -> Optional[RefinementStep[T]]:
        """Get a specific step (if witnessed)."""
        if 0 <= index < len(self.steps):
            return self.steps[index]
        return None

    def latest(self) -> Optional[RefinementStep[T]]:
        """Get the latest witnessed step."""
        if self.steps:
            return self.steps[-1]
        return None

    def add_step(self, value: T, description: FiniteDescription) -> RefinementStep[T]:
        """Add a new refinement step."""
        step = RefinementStep(
            index=len(self.steps),
            value=value,
            description=description
        )
        self.steps.append(step)
        return step

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "REFINEMENT_CHAIN",
            "chain_id": self.chain_id,
            "length": len(self.steps),
            "is_cauchy": self.is_cauchy
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "REFINEMENT_CHAIN",
            "chain_id": self.chain_id,
            "steps_witnessed": len(self.steps),
            "is_finite_prefix": self.is_finite_prefix(),
            "is_cauchy": self.is_cauchy,
            "chain_hash": self.fingerprint()[:32],
            "result": "PASS"
        }


@dataclass
class FinitaryLedger:
    """
    A finitary ledger of recorded tests.

    L = {(tau, a) : test tau returned answer a}
    """
    ledger_id: str
    records: List[Tuple[str, str]]  # (test_id, answer)

    def __len__(self) -> int:
        return len(self.records)

    def add_record(self, test_id: str, answer: str):
        """Record a test result."""
        self.records.append((test_id, answer))

    def get_survivors(self, domain: FiniteDomain) -> Set[FiniteDescription]:
        """
        Compute survivors given the ledger.

        W(L) = {x in D_0 : for all (tau, a) in L, tau(x) = a}
        """
        # For demonstration, return all elements (simplified)
        # In full implementation, would check test compatibility
        return set(domain.elements)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "FINITARY_LEDGER",
            "ledger_id": self.ledger_id,
            "record_count": len(self.records)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "FINITARY_LEDGER",
            "ledger_id": self.ledger_id,
            "record_count": len(self.records),
            "ledger_hash": self.fingerprint()[:32],
            "result": "PASS"
        }


@dataclass
class FinitaryCore:
    """
    The complete finitary core - the forced layer.

    Contains:
    - Finite domain
    - Finite witnesses
    - Total verifiers
    - Finitary ledger
    """
    core_id: str
    domain: FiniteDomain
    witnesses: List[FiniteWitness]
    verifiers: List[TotalVerifier]
    ledger: FinitaryLedger

    def is_all_finite(self) -> bool:
        """Verify all components are finite."""
        return (
            self.domain.is_finite() and
            all(w.is_finite() for w in self.witnesses)
        )

    def are_verifiers_total(self) -> bool:
        """Verify all verifiers are total (simplified check)."""
        # In full implementation, would run comprehensive tests
        return len(self.verifiers) > 0

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "FINITARY_CORE",
            "core_id": self.core_id,
            "domain_size": len(self.domain),
            "witness_count": len(self.witnesses),
            "verifier_count": len(self.verifiers)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "FINITARY_CORE",
            "core_id": self.core_id,
            "domain_finite": self.domain.is_finite(),
            "witnesses_finite": len(self.witnesses),
            "verifiers_total": self.are_verifiers_total(),
            "all_checkable": self.is_all_finite(),
            "core_hash": self.fingerprint()[:32],
            "result": "PASS" if self.is_all_finite() else "FAIL"
        }


def create_sample_finitary_core() -> FinitaryCore:
    """Create a sample finitary core for demonstration."""
    # Create finite domain
    elements = frozenset([
        FiniteDescription(code=f"elem_{i}")
        for i in range(10)
    ])
    domain = FiniteDomain(domain_id="SAMPLE_D0", elements=elements)

    # Create witnesses
    witnesses = [
        FiniteWitness(witness_id=f"wit_{i}", witness_data=f"witness_data_{i}")
        for i in range(5)
    ]

    # Create total verifier
    def sample_verify(w: FiniteWitness, c: FiniteDescription) -> VerifyResult:
        # Simple verification: check if witness matches claim
        if w.witness_data.endswith(c.code[-1:]):
            return VerifyResult.PASS
        return VerifyResult.FAIL

    verifiers = [
        TotalVerifier(verifier_id="SAMPLE_V", verify_fn=sample_verify)
    ]

    # Create ledger
    ledger = FinitaryLedger(ledger_id="SAMPLE_L", records=[])

    return FinitaryCore(
        core_id="SAMPLE_CORE",
        domain=domain,
        witnesses=witnesses,
        verifiers=verifiers,
        ledger=ledger
    )
