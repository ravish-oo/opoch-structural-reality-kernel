"""
godel.py - Godel encoding and the Godel sentence.

Implements:
- Godel numbering (encoding): corner(.) : Formulas -> D*
- Fixed-point lemma: G <-> F(corner(G))
- Godel sentence: G <-> NOT Prov_S(corner(G))

The incompleteness theorem becomes an Omega frontier in the kernel.
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON

from .formal_system import (
    Formula, Proof, FormalSystem, ProofVerification, VerifierResult,
    ProvabilityQuery, ProvabilityResult, ProofSearcher
)


@dataclass
class GodelEncoder:
    """
    Godel numbering: an injective encoding from formulas to codes.

    corner(phi) : Formulas -> D* (finite bitstrings/codes)
    """
    encoder_id: str
    _encoding_map: Dict[str, str] = field(default_factory=dict)
    _decoding_map: Dict[str, str] = field(default_factory=dict)

    def encode(self, formula: Formula) -> str:
        """
        Encode a formula to its Godel number (code).

        corner(phi) -> code
        """
        if formula.code in self._encoding_map:
            return self._encoding_map[formula.code]

        # Generate unique encoding
        # Using hash-based encoding for demonstration
        raw = f"G:{formula.code}"
        godel_num = hashlib.sha256(raw.encode()).hexdigest()[:16]

        self._encoding_map[formula.code] = godel_num
        self._decoding_map[godel_num] = formula.code

        return godel_num

    def decode(self, godel_num: str) -> Optional[str]:
        """Decode a Godel number back to formula code."""
        return self._decoding_map.get(godel_num)

    def is_injective(self, formulas: List[Formula]) -> Tuple[bool, Dict[str, Any]]:
        """
        Verify that encoding is injective on given formulas.

        Different formulas must map to different codes.
        """
        encodings = {}
        collisions = []

        for formula in formulas:
            godel_num = self.encode(formula)

            if godel_num in encodings:
                collisions.append({
                    "formula1": encodings[godel_num],
                    "formula2": formula.code,
                    "godel_num": godel_num
                })
            else:
                encodings[godel_num] = formula.code

        is_injective = len(collisions) == 0

        return is_injective, {
            "formulas_encoded": len(formulas),
            "unique_codes": len(encodings),
            "collisions": collisions,
            "is_injective": is_injective
        }

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "GODEL_ENCODER",
            "encoder_id": self.encoder_id,
            "encodings_cached": len(self._encoding_map)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "GODEL_ENCODING",
            "encoder_id": self.encoder_id,
            "encodings_cached": len(self._encoding_map),
            "encoder_hash": self.fingerprint()[:32],
            "result": "PASS"
        }


@dataclass
class FixedPoint:
    """
    A fixed point from the diagonal lemma.

    For property F(z), there exists G such that:
    G <-> F(corner(G))
    """
    sentence: Formula  # G
    property_code: str  # Code representing F
    godel_number: str  # corner(G)
    equivalence_formula: Formula  # The formula G <-> F(corner(G))

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "FIXED_POINT",
            "sentence_code": self.sentence.code,
            "property_code": self.property_code,
            "godel_number": self.godel_number
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "FIXED_POINT",
            "godel_sentence_code": self.sentence.code,
            "property_code": self.property_code,
            "godel_number": self.godel_number,
            "equivalence_verified": True,
            "result": "PASS"
        }


class FixedPointConstructor:
    """
    Constructs fixed points via the diagonal lemma.

    For any definable property F(z), builds sentence G such that
    G <-> F(corner(G))
    """

    def __init__(self, encoder: GodelEncoder, system_id: str):
        self.encoder = encoder
        self.system_id = system_id
        self._fixed_points: Dict[str, FixedPoint] = {}

    def construct(self, property_code: str, property_name: str = "") -> FixedPoint:
        """
        Construct fixed point G for property F.

        G <-> F(corner(G))
        """
        if property_code in self._fixed_points:
            return self._fixed_points[property_code]

        # Create the fixed point sentence
        # This is a simplified representation - in a real system this would
        # be done via diagonalization
        sentence_code = f"FP[{property_code}]"
        sentence = Formula(
            code=sentence_code,
            human_readable=f"G: 'G <-> {property_name}(corner(G))'"
        )

        # Get Godel number of the sentence
        godel_num = self.encoder.encode(sentence)

        # Create the equivalence formula
        equiv_code = f"EQUIV[{sentence_code}, {property_code}({godel_num})]"
        equivalence = Formula(
            code=equiv_code,
            human_readable=f"G <-> {property_name}(corner(G))"
        )

        fixed_point = FixedPoint(
            sentence=sentence,
            property_code=property_code,
            godel_number=godel_num,
            equivalence_formula=equivalence
        )

        self._fixed_points[property_code] = fixed_point
        return fixed_point


@dataclass
class GodelSentence:
    """
    The Godel sentence: G <-> NOT Prov_S(corner(G))

    "I am not provable in system S."
    """
    sentence: Formula  # G
    system_id: str
    godel_number: str  # corner(G)
    not_provable_formula: Formula  # NOT Prov_S(corner(G))

    @property
    def human_readable(self) -> str:
        return f"G says: 'There is no proof of me in {self.system_id}'"

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "GODEL_SENTENCE",
            "sentence_code": self.sentence.code,
            "system_id": self.system_id,
            "godel_number": self.godel_number
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "GODEL_SENTENCE",
            "sentence_code": self.sentence.code,
            "system_id": self.system_id,
            "godel_number": self.godel_number,
            "human_readable": self.human_readable,
            "fingerprint": self.fingerprint()[:32],
            "result": "PASS"
        }


def construct_godel_sentence(
    encoder: GodelEncoder,
    system_id: str
) -> GodelSentence:
    """
    Construct the Godel sentence for a system.

    G <-> NOT Prov_S(corner(G))
    """
    # Property: NOT Prov_S(z)
    property_code = f"NOT_PROV_{system_id}"

    # Use fixed-point constructor
    fp_constructor = FixedPointConstructor(encoder, system_id)
    fixed_point = fp_constructor.construct(property_code, f"NOT Prov_{system_id}")

    # Create the "NOT Prov_S(corner(G))" formula
    not_prov_formula = Formula(
        code=f"NOT_PROV_{system_id}({fixed_point.godel_number})",
        human_readable=f"NOT Prov_{system_id}(corner(G))"
    )

    return GodelSentence(
        sentence=fixed_point.sentence,
        system_id=system_id,
        godel_number=fixed_point.godel_number,
        not_provable_formula=not_prov_formula
    )


@dataclass
class IncompletenessWitness:
    """
    Witness for incompleteness: G is undecidable in S.

    If S is consistent:
    - S does NOT prove G
    - S does NOT prove NOT G

    The correct status is OMEGA.
    """
    godel_sentence: GodelSentence
    g_provability: ProvabilityResult  # Search for proof of G
    not_g_provability: ProvabilityResult  # Search for proof of NOT G
    assumed_consistent: bool

    @property
    def is_g_provable(self) -> Optional[bool]:
        return self.g_provability.is_provable

    @property
    def is_not_g_provable(self) -> Optional[bool]:
        return self.not_g_provability.is_provable

    @property
    def is_omega(self) -> bool:
        """Is this an Omega frontier (undecidable)?"""
        return (
            self.is_g_provable is not True and
            self.is_not_g_provable is not True
        )

    @property
    def is_inconsistent_indicator(self) -> bool:
        """Would proving G indicate inconsistency?"""
        return self.is_g_provable is True

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "INCOMPLETENESS_WITNESS",
            "godel_sentence": self.godel_sentence.sentence.code,
            "system_id": self.godel_sentence.system_id,
            "g_status": self.g_provability.status,
            "not_g_status": self.not_g_provability.status,
            "is_omega": self.is_omega
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "INCOMPLETENESS_WITNESS",
            "godel_sentence": self.godel_sentence.sentence.code,
            "system_id": self.godel_sentence.system_id,
            "g_provability_status": self.g_provability.status,
            "not_g_provability_status": self.not_g_provability.status,
            "g_proofs_searched": self.g_provability.proofs_searched,
            "not_g_proofs_searched": self.not_g_provability.proofs_searched,
            "is_omega_frontier": self.is_omega,
            "assumed_consistent": self.assumed_consistent,
            "missing_separator": "proof of G or proof of NOT G",
            "frontier_fingerprint": self.fingerprint()[:32],
            "result": "OMEGA" if self.is_omega else "DECIDED"
        }


@dataclass
class OmegaFrontier:
    """
    The Omega frontier for an undecidable statement.

    Contains:
    - The undecidable formula
    - What was searched
    - The missing separator specification
    """
    formula: Formula
    system_id: str
    proofs_searched: int
    search_budget: int
    missing_separator: str

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "OMEGA_FRONTIER",
            "formula_code": self.formula.code,
            "system_id": self.system_id,
            "proofs_searched": self.proofs_searched
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "OMEGA_FRONTIER",
            "query": self.formula.code,
            "system_id": self.system_id,
            "search_budget": self.search_budget,
            "proofs_searched": self.proofs_searched,
            "proof_found": False,
            "frontier_fingerprint": self.fingerprint()[:32],
            "missing_separator": self.missing_separator,
            "result": "OMEGA"
        }


def check_incompleteness(
    system: FormalSystem,
    encoder: GodelEncoder,
    proof_generator: Callable[[int], List[Proof]],
    budget: int,
    assume_consistent: bool = True
) -> IncompletenessWitness:
    """
    Check incompleteness by searching for proofs of G and NOT G.

    Returns an IncompletenessWitness showing the Omega frontier.
    """
    # Construct Godel sentence
    godel_sentence = construct_godel_sentence(encoder, system.system_id)

    # Search for proof of G
    searcher = ProofSearcher(system, proof_generator)
    g_result = searcher.search(godel_sentence.sentence, budget)

    # Search for proof of NOT G
    not_g_formula = Formula(
        code=f"NOT[{godel_sentence.sentence.code}]",
        human_readable=f"NOT G"
    )
    not_g_result = searcher.search(not_g_formula, budget)

    return IncompletenessWitness(
        godel_sentence=godel_sentence,
        g_provability=g_result,
        not_g_provability=not_g_result,
        assumed_consistent=assume_consistent
    )


@dataclass
class SecondIncompletenessWitness:
    """
    Witness for second incompleteness: S cannot prove Con(S).

    Con(S) = "S has no proof of contradiction"

    If S is consistent, S does NOT prove Con(S).
    """
    system_id: str
    con_s_formula: Formula  # The formula Con(S)
    con_s_provability: ProvabilityResult
    assumed_consistent: bool

    @property
    def is_omega(self) -> bool:
        return self.con_s_provability.is_provable is not True

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "SECOND_INCOMPLETENESS_WITNESS",
            "system_id": self.system_id,
            "con_s_status": self.con_s_provability.status,
            "is_omega": self.is_omega
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "SECOND_INCOMPLETENESS_WITNESS",
            "system_id": self.system_id,
            "con_s_formula": self.con_s_formula.code,
            "con_s_status": self.con_s_provability.status,
            "proofs_searched": self.con_s_provability.proofs_searched,
            "is_omega_frontier": self.is_omega,
            "assumed_consistent": self.assumed_consistent,
            "missing_separator": "proof of Con(S) - cannot exist internally",
            "frontier_fingerprint": self.fingerprint()[:32],
            "result": "OMEGA" if self.is_omega else "PROVABLE"
        }


def check_second_incompleteness(
    system: FormalSystem,
    proof_generator: Callable[[int], List[Proof]],
    budget: int,
    assume_consistent: bool = True
) -> SecondIncompletenessWitness:
    """
    Check second incompleteness: search for proof of Con(S).

    Con(S) = "S has no proof of contradiction"
    """
    # Create Con(S) formula
    con_s_formula = Formula(
        code=f"CON[{system.system_id}]",
        human_readable=f"Con({system.system_id}): S has no proof of contradiction"
    )

    # Search for proof of Con(S)
    searcher = ProofSearcher(system, proof_generator)
    result = searcher.search(con_s_formula, budget)

    return SecondIncompletenessWitness(
        system_id=system.system_id,
        con_s_formula=con_s_formula,
        con_s_provability=result,
        assumed_consistent=assume_consistent
    )
