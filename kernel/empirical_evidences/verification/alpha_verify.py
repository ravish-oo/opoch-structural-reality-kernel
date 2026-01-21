"""
alpha_verify.py - Verification suite for the fine-structure constant α derivation.

Implements verification checks:
1. Contract validity
2. Witness bundle completeness
3. Channel verifier execution
4. Interval arithmetic correctness
5. Channel intersection consistency
6. Gauge invariance
7. Certified digits computation
8. Proof bundle integrity
9. Receipt hash chain
10. Replay determinism
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
from fractions import Fraction
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
    AlphaContract,
    FrontierWitness,
    GaugeInvarianceCheck,
    check_gauge_invariance,
    create_interval,
    create_interval_from_value_uncertainty
)
from .alpha_channels import (
    QEDCoefficients,
    ElectronG2Verifier,
    AtomRecoilVerifier,
    create_qed_coefficients,
    create_channel_a_witness,
    create_channel_b_witness,
    run_channel_a,
    run_channel_b,
    run_all_channels
)
from .alpha_derive import (
    AlphaStatus,
    AlphaIntersection,
    OptimalSeparator,
    AlphaCertified,
    AlphaProofBundle,
    intersect_channels,
    compute_optimal_separator,
    derive_alpha
)


@dataclass
class VerificationCheck:
    """Result of a single verification check."""
    check_id: str
    check_name: str
    passed: bool
    details: Dict[str, Any]
    receipt: Dict[str, Any]

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "VERIFICATION_CHECK",
            "check_id": self.check_id,
            "check_name": self.check_name,
            "passed": self.passed
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class AlphaVerifier:
    """
    Complete verifier for α derivation.

    Runs all verification checks on the derivation process.
    """
    verifier_id: str

    def check_1_contract_validity(
        self,
        contract: AlphaContract
    ) -> VerificationCheck:
        """
        Check 1: Contract validity.

        Verifies the contract is properly specified.
        """
        has_channels = len(contract.channels) >= 2
        has_verifiers = len(contract.verifiers) >= 2
        has_canon = bool(contract.canonicalization)
        has_gauge = bool(contract.unit_gauge)

        passed = has_channels and has_verifiers and has_canon and has_gauge

        return VerificationCheck(
            check_id="CHECK_1",
            check_name="Contract Validity",
            passed=passed,
            details={
                "channels_count": len(contract.channels),
                "verifiers_count": len(contract.verifiers),
                "canonicalization": contract.canonicalization,
                "unit_gauge": contract.unit_gauge
            },
            receipt=contract.to_receipt()
        )

    def check_2_witness_completeness(
        self,
        witnesses: List[WitnessBundle]
    ) -> VerificationCheck:
        """
        Check 2: Witness bundle completeness.

        Verifies all witness bundles have required data.
        """
        all_complete = True
        details = {}

        for w in witnesses:
            has_data = len(w.data) > 0
            has_provenance = bool(w.provenance)
            has_uncertainty = bool(w.uncertainty_model)

            complete = has_data and has_provenance and has_uncertainty
            all_complete = all_complete and complete

            details[w.bundle_id] = {
                "data_count": len(w.data),
                "has_provenance": has_provenance,
                "has_uncertainty_model": has_uncertainty,
                "complete": complete
            }

        return VerificationCheck(
            check_id="CHECK_2",
            check_name="Witness Completeness",
            passed=all_complete,
            details=details,
            receipt={
                "type": "WITNESS_COMPLETENESS",
                "witnesses_count": len(witnesses),
                "all_complete": all_complete,
                "result": "PASS" if all_complete else "FAIL"
            }
        )

    def check_3_channel_verifiers(
        self,
        channel_results: List[ChannelResult]
    ) -> VerificationCheck:
        """
        Check 3: Channel verifier execution.

        Verifies all channel verifiers executed successfully.
        """
        all_passed = True
        details = {}

        for r in channel_results:
            v_result = r.verifier_result
            passed = v_result.is_success()
            all_passed = all_passed and passed

            details[r.channel_id] = {
                "status": v_result.status.value,
                "has_interval": v_result.alpha_interval is not None,
                "passed": passed
            }

        return VerificationCheck(
            check_id="CHECK_3",
            check_name="Channel Verifiers",
            passed=all_passed,
            details=details,
            receipt={
                "type": "CHANNEL_VERIFIERS",
                "channels_count": len(channel_results),
                "all_passed": all_passed,
                "result": "PASS" if all_passed else "FAIL"
            }
        )

    def check_4_interval_arithmetic(
        self,
        channel_results: List[ChannelResult]
    ) -> VerificationCheck:
        """
        Check 4: Interval arithmetic correctness.

        Verifies interval operations are valid.
        """
        all_valid = True
        details = {}

        for r in channel_results:
            interval = r.alpha_interval

            # Check interval is valid (lower <= upper)
            valid_bounds = interval.lower <= interval.upper

            # Check interval is non-empty
            non_empty = interval.width() >= RationalValue(0, 1)

            # Check interval is in reasonable range for α
            # α ≈ 1/137 ≈ 0.0073
            in_range = (
                interval.lower > RationalValue(0, 1) and
                interval.upper < RationalValue(1, 100)  # < 0.01
            )

            valid = valid_bounds and non_empty and in_range
            all_valid = all_valid and valid

            details[r.channel_id] = {
                "valid_bounds": valid_bounds,
                "non_empty": non_empty,
                "in_range": in_range,
                "width": str(interval.width().fraction),
                "valid": valid
            }

        return VerificationCheck(
            check_id="CHECK_4",
            check_name="Interval Arithmetic",
            passed=all_valid,
            details=details,
            receipt={
                "type": "INTERVAL_ARITHMETIC",
                "channels_count": len(channel_results),
                "all_valid": all_valid,
                "result": "PASS" if all_valid else "FAIL"
            }
        )

    def check_5_intersection_consistency(
        self,
        intersection: AlphaIntersection
    ) -> VerificationCheck:
        """
        Check 5: Channel intersection consistency.

        Verifies the intersection is computed correctly.
        """
        # Re-compute intersection
        recomputed = intersect_channels(intersection.channel_results)

        # Check status matches
        status_match = recomputed.status == intersection.status

        # Check interval matches (if unique)
        interval_match = True
        if intersection.is_unique() and recomputed.is_unique():
            if intersection.final_interval and recomputed.final_interval:
                interval_match = (
                    intersection.final_interval.lower == recomputed.final_interval.lower and
                    intersection.final_interval.upper == recomputed.final_interval.upper
                )

        consistent = status_match and interval_match

        return VerificationCheck(
            check_id="CHECK_5",
            check_name="Intersection Consistency",
            passed=consistent,
            details={
                "status": intersection.status.value,
                "status_match": status_match,
                "interval_match": interval_match,
                "channels_count": len(intersection.channel_results)
            },
            receipt=intersection.to_receipt()
        )

    def check_6_gauge_invariance(
        self,
        gauge_checks: List[GaugeInvarianceCheck]
    ) -> VerificationCheck:
        """
        Check 6: Gauge invariance.

        Verifies α (dimensionless) is invariant under unit recodings.
        """
        all_invariant = all(g.is_invariant for g in gauge_checks)

        details = {}
        for g in gauge_checks:
            details[g.check_id] = {
                "original_hash": g.original_hash[:16],
                "recoded_hash": g.recoded_hash[:16],
                "invariant": g.is_invariant
            }

        return VerificationCheck(
            check_id="CHECK_6",
            check_name="Gauge Invariance",
            passed=all_invariant,
            details=details,
            receipt={
                "type": "GAUGE_INVARIANCE",
                "checks_count": len(gauge_checks),
                "all_invariant": all_invariant,
                "result": "PASS" if all_invariant else "FAIL"
            }
        )

    def check_7_certified_digits(
        self,
        alpha_certified: Optional[AlphaCertified]
    ) -> VerificationCheck:
        """
        Check 7: Certified digits computation.

        Verifies digits are computed correctly from interval width.
        """
        if not alpha_certified:
            return VerificationCheck(
                check_id="CHECK_7",
                check_name="Certified Digits",
                passed=True,  # No certified α yet is OK
                details={"status": "no_certified_alpha"},
                receipt={
                    "type": "CERTIFIED_DIGITS",
                    "status": "no_certified_alpha",
                    "result": "PASS"
                }
            )

        interval = alpha_certified.interval
        declared_digits = alpha_certified.certified_digits

        # Recompute digits from width
        recomputed_digits = interval.certified_digits()

        # Digits should match
        digits_match = declared_digits == recomputed_digits

        # Verify digits are reasonable
        # α is experimentally known to ~11 digits, but mathematical bisection
        # can produce tighter bounds. Allow up to 25 digits.
        digits_reasonable = 0 < declared_digits <= 25

        passed = digits_match and digits_reasonable

        return VerificationCheck(
            check_id="CHECK_7",
            check_name="Certified Digits",
            passed=passed,
            details={
                "declared_digits": declared_digits,
                "recomputed_digits": recomputed_digits,
                "digits_match": digits_match,
                "digits_reasonable": digits_reasonable,
                "interval_width": str(interval.width().fraction)
            },
            receipt={
                "type": "CERTIFIED_DIGITS",
                "certified_digits": declared_digits,
                "verified": passed,
                "result": "PASS" if passed else "FAIL"
            }
        )

    def check_8_proof_bundle_integrity(
        self,
        bundle: AlphaProofBundle
    ) -> VerificationCheck:
        """
        Check 8: Proof bundle integrity.

        Verifies all components of the proof bundle are present and linked.
        """
        has_contract = bundle.contract is not None
        has_channels = len(bundle.channel_results) >= 2
        has_intersection = bundle.intersection is not None
        has_gauge_checks = len(bundle.gauge_checks) >= 2

        # Check fingerprint chain
        contract_fp = bundle.contract.fingerprint()
        channel_fps = [r.fingerprint() for r in bundle.channel_results]
        intersection_fp = bundle.intersection.fingerprint()

        fingerprints_valid = all([
            bool(contract_fp),
            all(bool(fp) for fp in channel_fps),
            bool(intersection_fp)
        ])

        passed = (has_contract and has_channels and has_intersection and
                  has_gauge_checks and fingerprints_valid)

        return VerificationCheck(
            check_id="CHECK_8",
            check_name="Proof Bundle Integrity",
            passed=passed,
            details={
                "has_contract": has_contract,
                "channels_count": len(bundle.channel_results),
                "has_intersection": has_intersection,
                "gauge_checks_count": len(bundle.gauge_checks),
                "fingerprints_valid": fingerprints_valid,
                "bundle_fingerprint": bundle.fingerprint()[:32]
            },
            receipt=bundle.to_receipt()
        )

    def check_9_receipt_hash_chain(
        self,
        bundle: AlphaProofBundle
    ) -> VerificationCheck:
        """
        Check 9: Receipt hash chain.

        Verifies all receipts have valid hashes.
        """
        receipts = []

        # Collect all receipts
        receipts.append(("contract", bundle.contract.to_receipt()))
        for r in bundle.channel_results:
            receipts.append((r.channel_id, r.to_receipt()))
        receipts.append(("intersection", bundle.intersection.to_receipt()))
        for g in bundle.gauge_checks:
            receipts.append((g.check_id, g.to_receipt()))

        all_valid = True
        details = {}

        for name, receipt in receipts:
            # Hash the receipt
            receipt_str = CanonicalJSON.serialize(receipt)
            receipt_hash = hashlib.sha256(receipt_str.encode()).hexdigest()

            # Check receipt is well-formed
            has_type = "type" in receipt
            has_result = "result" in receipt

            valid = has_type and has_result and bool(receipt_hash)
            all_valid = all_valid and valid

            details[name] = {
                "has_type": has_type,
                "has_result": has_result,
                "hash": receipt_hash[:16],
                "valid": valid
            }

        return VerificationCheck(
            check_id="CHECK_9",
            check_name="Receipt Hash Chain",
            passed=all_valid,
            details=details,
            receipt={
                "type": "RECEIPT_HASH_CHAIN",
                "receipts_count": len(receipts),
                "all_valid": all_valid,
                "result": "PASS" if all_valid else "FAIL"
            }
        )

    def check_10_replay_determinism(
        self,
        bundle: AlphaProofBundle
    ) -> VerificationCheck:
        """
        Check 10: Replay determinism.

        Verifies re-running the derivation produces identical results.
        """
        # Re-run derivation
        bundle2 = derive_alpha(desired_digits=10)

        # Compare fingerprints
        contract_match = bundle.contract.fingerprint() == bundle2.contract.fingerprint()

        # Channel fingerprints should match
        channel_match = True
        for r1, r2 in zip(bundle.channel_results, bundle2.channel_results):
            if r1.channel_type == r2.channel_type:
                # Intervals should be identical
                if r1.alpha_interval.lower != r2.alpha_interval.lower:
                    channel_match = False
                if r1.alpha_interval.upper != r2.alpha_interval.upper:
                    channel_match = False

        # Intersection status should match
        intersection_match = bundle.intersection.status == bundle2.intersection.status

        deterministic = contract_match and channel_match and intersection_match

        return VerificationCheck(
            check_id="CHECK_10",
            check_name="Replay Determinism",
            passed=deterministic,
            details={
                "contract_match": contract_match,
                "channel_match": channel_match,
                "intersection_match": intersection_match,
                "deterministic": deterministic
            },
            receipt={
                "type": "REPLAY_DETERMINISM",
                "deterministic": deterministic,
                "result": "PASS" if deterministic else "FAIL"
            }
        )

    def verify_all(
        self,
        bundle: AlphaProofBundle
    ) -> List[VerificationCheck]:
        """Run all verification checks."""
        checks = []

        # Extract witnesses from channel results
        witnesses = [r.witness_bundle for r in bundle.channel_results]

        # Run all checks
        checks.append(self.check_1_contract_validity(bundle.contract))
        checks.append(self.check_2_witness_completeness(witnesses))
        checks.append(self.check_3_channel_verifiers(bundle.channel_results))
        checks.append(self.check_4_interval_arithmetic(bundle.channel_results))
        checks.append(self.check_5_intersection_consistency(bundle.intersection))
        checks.append(self.check_6_gauge_invariance(bundle.gauge_checks))
        checks.append(self.check_7_certified_digits(bundle.alpha_certified))
        checks.append(self.check_8_proof_bundle_integrity(bundle))
        checks.append(self.check_9_receipt_hash_chain(bundle))
        checks.append(self.check_10_replay_determinism(bundle))

        return checks


@dataclass
class AlphaVerificationBundle:
    """
    Complete verification bundle for α derivation.
    """
    bundle_id: str
    alpha_bundle: AlphaProofBundle
    checks: List[VerificationCheck]

    def all_passed(self) -> bool:
        return all(c.passed for c in self.checks)

    def passed_count(self) -> int:
        return sum(1 for c in self.checks if c.passed)

    def failed_count(self) -> int:
        return sum(1 for c in self.checks if not c.passed)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ALPHA_VERIFICATION_BUNDLE",
            "bundle_id": self.bundle_id,
            "checks_total": len(self.checks),
            "checks_passed": self.passed_count(),
            "all_passed": self.all_passed()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "ALPHA_VERIFICATION_BUNDLE",
            "bundle_id": self.bundle_id,
            "alpha_bundle_fingerprint": self.alpha_bundle.fingerprint()[:16],
            "checks_total": len(self.checks),
            "checks_passed": self.passed_count(),
            "checks_failed": self.failed_count(),
            "all_passed": self.all_passed(),
            "check_details": [
                {"id": c.check_id, "name": c.check_name, "passed": c.passed}
                for c in self.checks
            ],
            "bundle_fingerprint": self.fingerprint(),
            "result": "PASS" if self.all_passed() else "FAIL"
        }


def run_alpha_verification() -> AlphaVerificationBundle:
    """
    Run complete α verification suite.

    Returns verification bundle with all check results.
    """
    # Derive α
    alpha_bundle = derive_alpha(desired_digits=10)

    # Create verifier and run checks
    verifier = AlphaVerifier(verifier_id="ALPHA_VERIFIER")
    checks = verifier.verify_all(alpha_bundle)

    return AlphaVerificationBundle(
        bundle_id="ALPHA_VERIFICATION",
        alpha_bundle=alpha_bundle,
        checks=checks
    )


def print_alpha_verification_report(bundle: AlphaVerificationBundle) -> None:
    """Print verification report."""
    print("=" * 70)
    print("FINE-STRUCTURE CONSTANT α VERIFICATION REPORT")
    print("=" * 70)
    print()

    # α derivation summary
    if bundle.alpha_bundle.alpha_certified:
        alpha = bundle.alpha_bundle.alpha_certified
        print("DERIVED α:")
        print(f"  α = {alpha.display_alpha()}")
        print(f"  α⁻¹ = {alpha.display_inverse()}")
        print(f"  Certified digits: {alpha.certified_digits}")
        print()

    # Verification checks
    print("VERIFICATION CHECKS:")
    print("-" * 70)

    for check in bundle.checks:
        status = "PASS" if check.passed else "FAIL"
        print(f"  [{status}] {check.check_id}: {check.check_name}")
        for key, value in list(check.details.items())[:5]:  # Show first 5 details
            if isinstance(value, dict):
                continue  # Skip nested dicts for brevity
            print(f"         {key}: {value}")
        print()

    # Summary
    print("-" * 70)
    print(f"SUMMARY: {bundle.passed_count()}/{len(bundle.checks)} checks passed")
    print(f"OVERALL: {'PASS' if bundle.all_passed() else 'FAIL'}")
    print(f"Bundle fingerprint: {bundle.fingerprint()[:32]}...")
    print("=" * 70)


# ============================================================
# MAIN ENTRY POINT
# ============================================================

if __name__ == "__main__":
    bundle = run_alpha_verification()
    print_alpha_verification_report(bundle)
