"""
formal_system.py - Formal systems as finite witness contracts.

A formal system S provides:
- A grammar of formulas
- A grammar of proofs
- A proof verifier: Proof_S(p, phi) in {PASS, FAIL}

Provability: Prov_S(phi) := exists p, Proof_S(p, phi) = PASS
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
from enum import Enum

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON


class VerifierResult(Enum):
    """Result of proof verification."""
    PASS = "PASS"
    FAIL = "FAIL"


@dataclass
class Formula:
    """
    A formula in a formal system.

    Formulas are finite bitstrings/codes that represent statements.
    """
    code: str
    human_readable: str = ""

    def __hash__(self):
        return hash(self.code)

    def __eq__(self, other):
        if isinstance(other, Formula):
            return self.code == other.code
        return False

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "FORMULA",
            "code": self.code
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class Proof:
    """
    A proof in a formal system.

    Proofs are finite witness objects that (if valid) demonstrate a formula.
    """
    code: str
    steps: List[str] = field(default_factory=list)

    def __hash__(self):
        return hash(self.code)

    def __eq__(self, other):
        if isinstance(other, Proof):
            return self.code == other.code
        return False

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "PROOF",
            "code": self.code,
            "step_count": len(self.steps)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class ProofVerification:
    """
    Result of verifying a proof.
    """
    proof: Proof
    formula: Formula
    result: VerifierResult
    reason: str = ""

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "PROOF_VERIFICATION",
            "proof_code": self.proof.code,
            "formula_code": self.formula.code,
            "result": self.result.value
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "PROOF_VERIFICATION",
            "proof_fingerprint": self.proof.fingerprint()[:16],
            "formula_fingerprint": self.formula.fingerprint()[:16],
            "result": self.result.value,
            "reason": self.reason
        }


class FormalSystem:
    """
    A formal system as a finite witness contract.

    Provides:
    - is_well_formed_formula: check if string is valid formula
    - is_well_formed_proof: check if string is valid proof
    - verify_proof: Proof_S(p, phi) -> PASS/FAIL
    """

    def __init__(
        self,
        system_id: str,
        formula_checker: Callable[[str], bool],
        proof_checker: Callable[[str], bool],
        proof_verifier: Callable[[Proof, Formula], VerifierResult]
    ):
        self.system_id = system_id
        self._formula_checker = formula_checker
        self._proof_checker = proof_checker
        self._proof_verifier = proof_verifier

    def is_well_formed_formula(self, code: str) -> bool:
        """Check if code represents a well-formed formula."""
        try:
            return self._formula_checker(code)
        except Exception:
            return False

    def is_well_formed_proof(self, code: str) -> bool:
        """Check if code represents a well-formed proof."""
        try:
            return self._proof_checker(code)
        except Exception:
            return False

    def verify_proof(self, proof: Proof, formula: Formula) -> ProofVerification:
        """
        Proof_S(p, phi) - verify if proof proves formula.

        This is TOTAL: always returns PASS or FAIL, never undefined.
        """
        # Check well-formedness first
        if not self.is_well_formed_proof(proof.code):
            return ProofVerification(
                proof=proof,
                formula=formula,
                result=VerifierResult.FAIL,
                reason="Proof is not well-formed"
            )

        if not self.is_well_formed_formula(formula.code):
            return ProofVerification(
                proof=proof,
                formula=formula,
                result=VerifierResult.FAIL,
                reason="Formula is not well-formed"
            )

        # Run the verifier
        try:
            result = self._proof_verifier(proof, formula)
            return ProofVerification(
                proof=proof,
                formula=formula,
                result=result,
                reason="Verified" if result == VerifierResult.PASS else "Proof does not establish formula"
            )
        except Exception as e:
            return ProofVerification(
                proof=proof,
                formula=formula,
                result=VerifierResult.FAIL,
                reason=f"Verification error: {str(e)}"
            )

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "FORMAL_SYSTEM",
            "system_id": self.system_id
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "FORMAL_SYSTEM",
            "system_id": self.system_id,
            "result": "PASS"
        }


@dataclass
class ProvabilityQuery:
    """
    Query: does there exist a proof of formula phi?

    Prov_S(phi) := exists p, Proof_S(p, phi) = PASS
    """
    formula: Formula
    system_id: str

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "PROVABILITY_QUERY",
            "formula_code": self.formula.code,
            "system_id": self.system_id
        })


@dataclass
class ProvabilityResult:
    """
    Result of provability search.

    Either:
    - PROVABLE with witness proof
    - OMEGA frontier (undetermined within budget)
    """
    query: ProvabilityQuery
    is_provable: Optional[bool]  # None = OMEGA
    witness_proof: Optional[Proof]
    proofs_searched: int
    search_budget: int

    @property
    def is_omega(self) -> bool:
        """Is this an Omega frontier?"""
        return self.is_provable is None

    @property
    def status(self) -> str:
        if self.is_provable is True:
            return "PROVABLE"
        elif self.is_provable is False:
            return "REFUTED"
        else:
            return "OMEGA"

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "PROVABILITY_RESULT",
            "formula_code": self.query.formula.code,
            "status": self.status,
            "proofs_searched": self.proofs_searched
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "PROVABILITY_RESULT",
            "query": self.query.formula.code,
            "status": self.status,
            "witness_proof": self.witness_proof.code if self.witness_proof else "NONE",
            "proofs_searched": self.proofs_searched,
            "search_budget": self.search_budget,
            "frontier_fingerprint": self.fingerprint()[:32] if self.is_omega else "N/A",
            "result": "PASS" if not self.is_omega else "OMEGA"
        }


class ProofSearcher:
    """
    Searches for proofs within a budget.

    Implements bounded proof search for Prov_S(phi).
    """

    def __init__(
        self,
        system: FormalSystem,
        proof_generator: Callable[[int], List[Proof]]
    ):
        """
        Args:
            system: The formal system
            proof_generator: Function that generates candidate proofs up to given size
        """
        self.system = system
        self._proof_generator = proof_generator

    def search(
        self,
        formula: Formula,
        budget: int
    ) -> ProvabilityResult:
        """
        Search for a proof of formula within budget.

        Returns PROVABLE with witness if found, else OMEGA frontier.
        """
        query = ProvabilityQuery(
            formula=formula,
            system_id=self.system.system_id
        )

        proofs_checked = 0

        # Generate and check proofs
        for size in range(1, budget + 1):
            candidates = self._proof_generator(size)

            for proof in candidates:
                proofs_checked += 1

                verification = self.system.verify_proof(proof, formula)

                if verification.result == VerifierResult.PASS:
                    return ProvabilityResult(
                        query=query,
                        is_provable=True,
                        witness_proof=proof,
                        proofs_searched=proofs_checked,
                        search_budget=budget
                    )

                if proofs_checked >= budget:
                    break

            if proofs_checked >= budget:
                break

        # Budget exhausted without finding proof
        return ProvabilityResult(
            query=query,
            is_provable=None,  # OMEGA
            witness_proof=None,
            proofs_searched=proofs_checked,
            search_budget=budget
        )


@dataclass
class ConsistencyQuery:
    """
    Query: does there exist a proof of contradiction (bottom)?

    Con(S) := NOT exists p, Proof_S(p, bottom) = PASS
    """
    system_id: str
    contradiction_formula: Formula  # The formula representing bottom/false

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CONSISTENCY_QUERY",
            "system_id": self.system_id
        })


@dataclass
class ConsistencyResult:
    """
    Result of consistency check.

    Either:
    - INCONSISTENT with witness proof of contradiction
    - OMEGA frontier (no contradiction found within budget)
    """
    query: ConsistencyQuery
    is_consistent: Optional[bool]  # None = OMEGA (not proven either way)
    contradiction_proof: Optional[Proof]
    proofs_searched: int
    search_budget: int

    @property
    def is_omega(self) -> bool:
        return self.is_consistent is None

    @property
    def status(self) -> str:
        if self.is_consistent is True:
            return "CONSISTENT"
        elif self.is_consistent is False:
            return "INCONSISTENT"
        else:
            return "OMEGA"

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CONSISTENCY_RESULT",
            "system_id": self.query.system_id,
            "status": self.status,
            "proofs_searched": self.proofs_searched
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "CONSISTENCY_FRONTIER",
            "system_id": self.query.system_id,
            "search_budget": self.search_budget,
            "proofs_searched": self.proofs_searched,
            "contradiction_found": self.is_consistent is False,
            "frontier_status": self.status,
            "frontier_fingerprint": self.fingerprint()[:32],
            "result": "PASS"
        }


def check_consistency(
    system: FormalSystem,
    contradiction_formula: Formula,
    proof_generator: Callable[[int], List[Proof]],
    budget: int
) -> ConsistencyResult:
    """
    Check system consistency by searching for proof of contradiction.
    """
    query = ConsistencyQuery(
        system_id=system.system_id,
        contradiction_formula=contradiction_formula
    )

    searcher = ProofSearcher(system, proof_generator)
    result = searcher.search(contradiction_formula, budget)

    if result.is_provable is True:
        # Found proof of contradiction = inconsistent
        return ConsistencyResult(
            query=query,
            is_consistent=False,
            contradiction_proof=result.witness_proof,
            proofs_searched=result.proofs_searched,
            search_budget=budget
        )
    else:
        # No contradiction found (within budget) = OMEGA
        return ConsistencyResult(
            query=query,
            is_consistent=None,  # OMEGA - can't prove consistency
            contradiction_proof=None,
            proofs_searched=result.proofs_searched,
            search_budget=budget
        )
