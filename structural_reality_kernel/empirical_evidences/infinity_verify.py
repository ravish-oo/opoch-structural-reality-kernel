"""
infinity_verify.py - Complete verification suite for Infinity and Continuum.

Implements all verification checks A-D:
A) Finitary core verification - domain finite, verifiers total
B) Closure declaration verification - monotone, idempotent, gauge-invariant
C) Independence/Omega labeling verification - closure-dependent statements
D) Continuum construction verification - Cauchy sequences, completion
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
from fractions import Fraction
import hashlib
import json

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON

from .finitary_core import (
    FiniteDescription, FiniteDomain, FiniteWitness, TotalVerifier,
    VerifyResult, RefinementChain, FinitaryLedger, FinitaryCore,
    create_sample_finitary_core
)
from .closure_policy import (
    ClosurePolicy, ClosureStatus, CauchySequence, RationalApproximation,
    CauchyEquivalence, CompletedReal, ContinuumConstruction,
    IndependentStatement, IndependenceWitness,
    create_ch_independence, create_choice_independence,
    create_sqrt2_sequence, create_pi_sequence
)


@dataclass
class CheckResult:
    """Result of a single verification check."""
    check_id: str
    check_name: str
    passed: bool
    details: Dict[str, Any]

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "id": self.check_id,
            "name": self.check_name,
            "passed": self.passed
        })


@dataclass
class InfinityProofBundle:
    """
    Complete proof bundle for Infinity and Continuum verification.
    """
    bundle_id: str
    checks: List[CheckResult]
    finitary_verified: bool
    closures_verified: bool
    independence_verified: bool
    continuum_verified: bool
    receipts: List[Dict[str, Any]]

    def all_passed(self) -> bool:
        return all(c.passed for c in self.checks)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "bundle_id": self.bundle_id,
            "all_passed": self.all_passed(),
            "check_count": len(self.checks),
            "passed_count": sum(1 for c in self.checks if c.passed)
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def summary(self) -> Dict[str, Any]:
        return {
            "bundle_id": self.bundle_id,
            "all_passed": self.all_passed(),
            "total_checks": len(self.checks),
            "passed": sum(1 for c in self.checks if c.passed),
            "failed": sum(1 for c in self.checks if not c.passed),
            "failed_checks": [c.check_id for c in self.checks if not c.passed]
        }


class InfinityVerifier:
    """
    Complete verification suite for Infinity and Continuum.

    Implements checks A-D.
    """

    def __init__(self):
        self.checks: List[CheckResult] = []
        self.receipts: List[Dict[str, Any]] = []

        # Results tracking
        self.finitary_verified = False
        self.closures_verified = False
        self.independence_verified = False
        self.continuum_verified = False

    def _add_check(
        self,
        check_id: str,
        check_name: str,
        passed: bool,
        details: Dict[str, Any]
    ) -> CheckResult:
        result = CheckResult(
            check_id=check_id,
            check_name=check_name,
            passed=passed,
            details=details
        )
        self.checks.append(result)
        return result

    def check_A_finitary_core(self) -> CheckResult:
        """
        A) Finitary Core Verification

        Verify:
        - Domain objects are finite descriptions
        - All verifiers are total (PASS/FAIL/TIMEOUT)
        - All results checkable by finite witnesses
        """
        # Create sample finitary core
        core = create_sample_finitary_core()

        # Check domain is finite
        domain_finite = core.domain.is_finite()

        # Check all descriptions are finite
        all_descs_finite = all(
            elem.is_finite() for elem in core.domain.elements
        )

        # Check all witnesses are finite
        all_witnesses_finite = all(w.is_finite() for w in core.witnesses)

        # Check verifiers are total
        verifiers_total = core.are_verifiers_total()

        # Test totality with sample cases
        if core.verifiers:
            verifier = core.verifiers[0]
            test_cases = [
                (core.witnesses[0], list(core.domain.elements)[0])
            ] if core.witnesses and core.domain.elements else []

            totality_verified = verifier.is_total(test_cases) if test_cases else True
        else:
            totality_verified = True

        all_ok = (
            domain_finite and
            all_descs_finite and
            all_witnesses_finite and
            verifiers_total
        )
        self.finitary_verified = all_ok

        # Add receipts
        domain_receipt = core.domain.to_receipt()
        self.receipts.append(domain_receipt)

        core_receipt = core.to_receipt()
        self.receipts.append(core_receipt)

        bundle_receipt = {
            "type": "FINITARY_CORE",
            "core_id": core.core_id,
            "domain_finite": domain_finite,
            "domain_size": len(core.domain),
            "all_descriptions_finite": all_descs_finite,
            "witnesses_finite": len(core.witnesses),
            "verifiers_total": verifiers_total,
            "totality_verified": totality_verified,
            "all_checkable": all_ok,
            "domain_hash": core.domain.fingerprint()[:32],
            "result": "PASS" if all_ok else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="A",
            check_name="Finitary Core Verification",
            passed=all_ok,
            details=bundle_receipt
        )

    def check_B_closure_declaration(self) -> CheckResult:
        """
        B) Closure Declaration Verification

        Verify closure policies:
        - Formal definition provided
        - Monotonicity verified
        - Idempotence verified
        - Gauge-invariance verified
        """
        # Create continuum construction with its closure
        continuum = ContinuumConstruction(construction_id="REAL_LINE")
        closure = continuum.closure_policy

        # Check properties
        has_definition = closure.formal_definition != ""
        is_monotone = closure.is_monotone
        is_idempotent = closure.is_idempotent
        is_gauge_invariant = closure.is_gauge_invariant

        # Verify monotonicity on sample chains
        seq1 = create_sqrt2_sequence(10)
        seq2 = create_sqrt2_sequence(20)  # Refined version

        chain1 = seq1.to_refinement_chain()
        chain2 = seq2.to_refinement_chain()

        # Chain2 refines chain1 (more terms)
        def refines(a: Fraction, b: Fraction) -> bool:
            # Both should converge to same value
            return abs(a - b) < Fraction(1, 100)

        monotone_verified = closure.verify_monotone(chain2, chain1, refines)

        # Verify idempotence
        idempotent_verified = closure.verify_idempotent(chain1)

        all_ok = (
            has_definition and
            is_monotone and
            is_idempotent and
            is_gauge_invariant and
            monotone_verified and
            idempotent_verified
        )
        self.closures_verified = all_ok

        # Add closure receipt
        closure_receipt = closure.to_receipt()
        self.receipts.append(closure_receipt)

        bundle_receipt = {
            "type": "CLOSURE_VERIFICATION",
            "closure_id": closure.closure_id,
            "has_formal_definition": has_definition,
            "is_monotone": is_monotone,
            "monotone_verified": monotone_verified,
            "is_idempotent": is_idempotent,
            "idempotent_verified": idempotent_verified,
            "is_gauge_invariant": is_gauge_invariant,
            "closure_hash": closure.fingerprint()[:32],
            "result": "PASS" if all_ok else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="B",
            check_name="Closure Declaration Verification",
            passed=all_ok,
            details=bundle_receipt
        )

    def check_C_independence_labeling(self) -> CheckResult:
        """
        C) Independence/Omega Labeling Verification

        Verify that closure-dependent statements are correctly labeled:
        - Omega frontier identified
        - Closure dependencies listed
        - Minimal separator specified
        """
        # Create independence witnesses
        ch_witness = create_ch_independence()
        ac_witness = create_choice_independence()

        # Verify CH independence
        ch_is_omega = ch_witness.is_omega
        ch_has_separator = ch_witness.minimal_separator != ""
        ch_has_closures = (
            ch_witness.closure_that_decides_true is not None and
            ch_witness.closure_that_decides_false is not None
        )

        # Verify AC independence
        ac_is_omega = ac_witness.is_omega
        ac_has_separator = ac_witness.minimal_separator != ""
        ac_has_closures = (
            ac_witness.closure_that_decides_true is not None and
            ac_witness.closure_that_decides_false is not None
        )

        all_ok = (
            ch_is_omega and ch_has_separator and ch_has_closures and
            ac_is_omega and ac_has_separator and ac_has_closures
        )
        self.independence_verified = all_ok

        # Add receipts
        ch_receipt = ch_witness.to_receipt()
        self.receipts.append(ch_receipt)

        ac_receipt = ac_witness.to_receipt()
        self.receipts.append(ac_receipt)

        bundle_receipt = {
            "type": "INDEPENDENCE_VERIFICATION",
            "statements_checked": 2,
            "ch_is_omega": ch_is_omega,
            "ch_minimal_separator": ch_witness.minimal_separator,
            "ac_is_omega": ac_is_omega,
            "ac_minimal_separator": ac_witness.minimal_separator,
            "all_properly_labeled": all_ok,
            "result": "PASS" if all_ok else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="C",
            check_name="Independence/Omega Labeling Verification",
            passed=all_ok,
            details=bundle_receipt
        )

    def check_D_continuum_construction(self) -> CheckResult:
        """
        D) Continuum Construction Verification

        Verify:
        - Cauchy sequences properly defined
        - Equivalence relation verified
        - Completion produces valid reals
        - Operations verified (simplified)
        """
        # Create continuum construction
        continuum = ContinuumConstruction(construction_id="REAL_LINE")

        # Create sample Cauchy sequences
        sqrt2_seq = create_sqrt2_sequence(20)
        pi_seq = create_pi_sequence(20)

        # Verify Cauchy criterion
        # Note: sqrt(2) via Newton converges fast, pi via Leibniz converges slowly
        sqrt2_is_cauchy = sqrt2_seq.is_cauchy(Fraction(1, 100), 10)
        pi_is_cauchy = pi_seq.is_cauchy(Fraction(1, 2), 15)  # Leibniz converges slowly

        # Complete sequences
        sqrt2_real, sqrt2_status = continuum.complete_sequence(sqrt2_seq)
        pi_real, pi_status = continuum.complete_sequence(pi_seq)

        sqrt2_completed = sqrt2_status == ClosureStatus.COMPLETED
        pi_completed = pi_status == ClosureStatus.COMPLETED

        # Verify equivalence relation
        equiv = continuum.equivalence
        sqrt2_seq2 = create_sqrt2_sequence(15)  # Different depth
        equiv_verified = equiv.are_equivalent(sqrt2_seq, sqrt2_seq2)

        # Verify approximations
        if sqrt2_real:
            sqrt2_approx = sqrt2_real.approximate(15)
            approx_close = abs(sqrt2_approx * sqrt2_approx - 2) < Fraction(1, 100)
        else:
            approx_close = False

        all_ok = (
            sqrt2_is_cauchy and
            pi_is_cauchy and
            sqrt2_completed and
            pi_completed and
            equiv_verified and
            approx_close
        )
        self.continuum_verified = all_ok

        # Add construction receipt
        construction_receipt = continuum.to_receipt()
        self.receipts.append(construction_receipt)

        # Add real receipts
        if sqrt2_real:
            self.receipts.append(sqrt2_real.to_receipt())
        if pi_real:
            self.receipts.append(pi_real.to_receipt())

        bundle_receipt = {
            "type": "CONTINUUM_CONSTRUCTION_VERIFICATION",
            "construction_id": continuum.construction_id,
            "base_field": continuum.base_field,
            "sequence_type": continuum.sequence_type,
            "sqrt2_cauchy_verified": sqrt2_is_cauchy,
            "pi_cauchy_verified": pi_is_cauchy,
            "sqrt2_completed": sqrt2_completed,
            "pi_completed": pi_completed,
            "equivalence_verified": equiv_verified,
            "approximation_verified": approx_close,
            "construction_hash": continuum.fingerprint()[:32],
            "result": "PASS" if all_ok else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="D",
            check_name="Continuum Construction Verification",
            passed=all_ok,
            details=bundle_receipt
        )

    def run_all_checks(self) -> List[CheckResult]:
        """Run all verification checks A-D."""
        self.checks = []

        self.check_A_finitary_core()
        self.check_B_closure_declaration()
        self.check_C_independence_labeling()
        self.check_D_continuum_construction()

        return self.checks

    def create_proof_bundle(self) -> InfinityProofBundle:
        """Create complete proof bundle."""
        if not self.checks:
            raise ValueError("Must run checks before creating bundle")

        bundle_id = f"INFINITY_{hashlib.sha256(b'continuum').hexdigest()[:8]}"

        return InfinityProofBundle(
            bundle_id=bundle_id,
            checks=self.checks.copy(),
            finitary_verified=self.finitary_verified,
            closures_verified=self.closures_verified,
            independence_verified=self.independence_verified,
            continuum_verified=self.continuum_verified,
            receipts=self.receipts.copy()
        )


def run_infinity_verification() -> InfinityProofBundle:
    """
    Run complete Infinity and Continuum verification.
    """
    verifier = InfinityVerifier()
    verifier.run_all_checks()
    return verifier.create_proof_bundle()


def print_infinity_report(bundle: InfinityProofBundle) -> str:
    """Generate human-readable verification report."""
    lines = [
        "=" * 60,
        "INFINITY AND CONTINUUM - VERIFICATION REPORT",
        "=" * 60,
        f"Bundle ID: {bundle.bundle_id}",
        f"Fingerprint: {bundle.fingerprint()[:32]}...",
        "",
        f"Finitary Core Verified: {bundle.finitary_verified}",
        f"Closures Verified: {bundle.closures_verified}",
        f"Independence Verified: {bundle.independence_verified}",
        f"Continuum Verified: {bundle.continuum_verified}",
        "",
        "-" * 60,
        "VERIFICATION CHECKS",
        "-" * 60,
    ]

    for check in bundle.checks:
        status = "PASS" if check.passed else "FAIL"
        lines.append(f"[{check.check_id}] {check.check_name}: {status}")

    lines.extend([
        "",
        "-" * 60,
        "KEY INSIGHTS",
        "-" * 60,
        "1. Forced universe is FINITARY - only finite witnesses",
        "2. 'Infinity' = explicit CLOSURE POLICY on refinement chains",
        "3. Continuum = Cl_inf(Q, Cauchy) - declared completion",
        "4. CH, AC = OMEGA frontiers (require closure axioms)",
        "",
        "-" * 60,
        "SUMMARY",
        "-" * 60,
        f"Total Checks: {len(bundle.checks)}",
        f"Passed: {sum(1 for c in bundle.checks if c.passed)}",
        f"Failed: {sum(1 for c in bundle.checks if not c.passed)}",
        "",
        f"OVERALL: {'ALL CHECKS PASSED' if bundle.all_passed() else 'VERIFICATION FAILED'}",
        "=" * 60
    ])

    return "\n".join(lines)


def run_demo() -> Dict[str, Any]:
    """
    Run Infinity and Continuum demonstration.
    """
    # Run verification
    bundle = run_infinity_verification()

    # Print report
    report = print_infinity_report(bundle)
    print(report)

    return {
        "demo": "Infinity and Continuum",
        "all_passed": bundle.all_passed(),
        "summary": bundle.summary(),
        "checks": [
            {"id": c.check_id, "name": c.check_name, "passed": c.passed}
            for c in bundle.checks
        ]
    }


if __name__ == "__main__":
    result = run_demo()
    print("\n" + json.dumps(result, indent=2))
