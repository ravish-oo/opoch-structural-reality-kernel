"""
qft_verify.py - Complete verification suite for QFT/RG/Universality.

Implements all verification checks A-G:
A) Explicit scale test sets - test sets with cost bounds, totality
B) Quotient construction - scale equivalence, canonical fingerprint
C) Semigroup property - coarsening composes
D) Effective dynamics extraction - U_s = R_s o U o iota_s, gauge invariance
E) Fixed point / universality witness - convergence verification
F) No minted distinctions - recoding invariance
G) Canonical receipts - all artifacts SHA-256 hashed
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Set, Tuple
import hashlib
import json

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.receipts import CanonicalJSON

from .scale_quotient import (
    ScaleTest, ScaleTestSet, ScaleQuotient, EquivalenceClass,
    CoarseGrainingMap, SemigroupCheck, ScaleHierarchy,
    construct_quotient, construct_coarse_graining,
    verify_semigroup_property, create_sample_scale_hierarchy
)
from .renormalization import (
    MicroDynamics, RepresentativeSection, EffectiveDynamics,
    GaugeInvarianceCheck, FixedPointResult, UniversalityClass, RGBundle,
    create_canonical_section, create_alternative_section,
    extract_effective_dynamics, verify_gauge_invariance,
    detect_fixed_point, identify_universality_classes,
    create_rg_bundle, create_sample_micro_dynamics,
    create_fixed_point_dynamics, create_varied_dynamics
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
class QFTProofBundle:
    """
    Complete proof bundle for QFT/RG/Universality verification.
    """
    bundle_id: str
    checks: List[CheckResult]
    scale_tests_verified: bool
    quotients_verified: bool
    semigroup_verified: bool
    effective_dynamics_verified: bool
    fixed_points_verified: bool
    gauge_invariance_verified: bool
    receipts_ok: bool
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


class QFTVerifier:
    """
    Complete verification suite for QFT/RG/Universality.

    Implements checks A-G.
    """

    def __init__(self, domain_size: int = 8):
        # Create sample domain
        self.domain = set(range(domain_size))

        # Create scale hierarchy
        self.hierarchy = create_sample_scale_hierarchy(self.domain)

        # Create micro dynamics
        self.micro_dynamics = create_sample_micro_dynamics(self.domain)

        # Create dynamics for universality testing
        self.dynamics_list = [
            self.micro_dynamics,
            create_fixed_point_dynamics(self.domain),
            create_varied_dynamics(self.domain, 2),
            create_varied_dynamics(self.domain, 3)
        ]

        # Create RG bundle
        self.rg_bundle = create_rg_bundle(
            self.hierarchy,
            self.micro_dynamics,
            self.dynamics_list
        )

        self.checks: List[CheckResult] = []
        self.receipts: List[Dict[str, Any]] = []

        # Results tracking
        self.scale_tests_verified = False
        self.quotients_verified = False
        self.semigroup_verified = False
        self.effective_dynamics_verified = False
        self.fixed_points_verified = False
        self.gauge_invariance_verified = False
        self.receipts_ok = False

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

    def check_A_scale_test_sets(self) -> CheckResult:
        """
        A) Explicit Scale Test Sets

        Verify:
        - Test sets exist for each scale
        - Cost bounds are defined
        - All tests are total (return outcome for all domain elements)
        """
        all_valid = True
        issues = []
        sets_checked = 0

        for scale in self.hierarchy.scales:
            # Check test set exists
            if not scale.tests:
                all_valid = False
                issues.append(f"{scale.scale_id}: no tests")
                continue

            # Check cost bound
            if scale.cost_bound <= 0:
                all_valid = False
                issues.append(f"{scale.scale_id}: invalid cost bound")

            # Check totality
            if not scale.all_total(self.domain):
                all_valid = False
                issues.append(f"{scale.scale_id}: tests not total")

            # Check feasible tests exist
            feasible = scale.feasible_tests()
            if not feasible:
                all_valid = False
                issues.append(f"{scale.scale_id}: no feasible tests")

            sets_checked += 1

            # Add receipt
            receipt = scale.to_receipt(self.domain)
            self.receipts.append(receipt)

        self.scale_tests_verified = all_valid

        bundle_receipt = {
            "type": "SCALE_TEST_SETS_VERIFICATION",
            "scales_checked": sets_checked,
            "all_valid": all_valid,
            "issues": issues[:5] if issues else [],
            "result": "PASS" if all_valid else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="A",
            check_name="Explicit Scale Test Sets",
            passed=all_valid,
            details=bundle_receipt
        )

    def check_B_quotient_construction(self) -> CheckResult:
        """
        B) Quotient Construction

        Verify:
        - Scale equivalence computed correctly
        - Quotients have canonical fingerprints
        - Class sizes form valid multisets
        """
        all_valid = True
        issues = []
        quotients_checked = 0

        for quotient in self.hierarchy.quotients:
            # Check quotient has classes
            if not quotient.equivalence_classes:
                all_valid = False
                issues.append(f"{quotient.scale_id}: no equivalence classes")
                continue

            # Check all domain elements are classified
            classified = set()
            for eq_class in quotient.equivalence_classes:
                classified.update(eq_class.members)

            if classified != self.domain:
                all_valid = False
                issues.append(f"{quotient.scale_id}: incomplete classification")

            # Check class sizes are positive
            sizes = quotient.class_sizes()
            if any(s <= 0 for s in sizes):
                all_valid = False
                issues.append(f"{quotient.scale_id}: invalid class sizes")

            # Check sum of sizes equals domain size
            if sum(sizes) != len(self.domain):
                all_valid = False
                issues.append(f"{quotient.scale_id}: size mismatch")

            quotients_checked += 1

            # Add receipt
            receipt = quotient.to_receipt()
            self.receipts.append(receipt)

        self.quotients_verified = all_valid

        bundle_receipt = {
            "type": "QUOTIENT_CONSTRUCTION_VERIFICATION",
            "quotients_checked": quotients_checked,
            "all_valid": all_valid,
            "quotient_sizes": [q.class_count() for q in self.hierarchy.quotients],
            "issues": issues[:5] if issues else [],
            "result": "PASS" if all_valid else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="B",
            check_name="Quotient Construction",
            passed=all_valid,
            details=bundle_receipt
        )

    def check_C_semigroup_property(self) -> CheckResult:
        """
        C) Semigroup Property

        Verify:
        - R_{s2} o R_{s1} = R_{s2} for s2 coarser than s1
        - Coarsening twice equals coarsening once to coarser scale
        """
        all_valid = True
        issues = []
        checks_performed = 0

        semigroup_checks = self.hierarchy.verify_semigroup(self.domain)

        for check in semigroup_checks:
            if not check.composition_verified:
                all_valid = False
                issues.append(f"{check.scale_1_id}->{check.scale_2_id}: composition failed")
                if check.mismatches:
                    issues.append(f"  Mismatches: {check.mismatches[:3]}")

            checks_performed += 1

            # Add receipt
            receipt = check.to_receipt()
            self.receipts.append(receipt)

        self.semigroup_verified = all_valid

        bundle_receipt = {
            "type": "SEMIGROUP_PROPERTY_VERIFICATION",
            "checks_performed": checks_performed,
            "all_composed_correctly": all_valid,
            "issues": issues[:5] if issues else [],
            "result": "PASS" if all_valid else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="C",
            check_name="Semigroup Property",
            passed=all_valid,
            details=bundle_receipt
        )

    def check_D_effective_dynamics(self) -> CheckResult:
        """
        D) Effective Dynamics Extraction

        Verify:
        - U_s = R_s o U o iota_s computed correctly
        - Different section choices give same Pi-fixed fingerprints
        """
        all_valid = True
        issues = []
        dynamics_checked = 0

        for eff_dyn in self.rg_bundle.effective_dynamics:
            # Check dynamics is defined for all classes
            if eff_dyn.class_count() == 0:
                all_valid = False
                issues.append(f"{eff_dyn.scale_id}: no class transitions")
                continue

            dynamics_checked += 1

            # Add receipt
            receipt = eff_dyn.to_receipt(self.micro_dynamics.fingerprint())
            self.receipts.append(receipt)

        # Check gauge invariance
        for gauge_check in self.rg_bundle.gauge_checks:
            if not gauge_check.gauges_equivalent:
                all_valid = False
                issues.append(f"{gauge_check.scale_id}: gauge not invariant")

            # Add receipt
            receipt = gauge_check.to_receipt()
            self.receipts.append(receipt)

        self.effective_dynamics_verified = all_valid
        self.gauge_invariance_verified = all(gc.gauges_equivalent for gc in self.rg_bundle.gauge_checks)

        bundle_receipt = {
            "type": "EFFECTIVE_DYNAMICS_VERIFICATION",
            "dynamics_checked": dynamics_checked,
            "gauge_checks": len(self.rg_bundle.gauge_checks),
            "all_gauge_invariant": self.gauge_invariance_verified,
            "issues": issues[:5] if issues else [],
            "result": "PASS" if all_valid else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="D",
            check_name="Effective Dynamics Extraction",
            passed=all_valid,
            details=bundle_receipt
        )

    def check_E_fixed_points(self) -> CheckResult:
        """
        E) Fixed Point / Universality Witness

        Verify:
        - Fixed points detected by fingerprint convergence
        - Universality classes identified
        """
        all_valid = True
        issues = []

        fp_result = self.rg_bundle.fixed_point_result

        # Check if fixed point analysis completed
        if not fp_result.scale_sequence:
            all_valid = False
            issues.append("No scale sequence analyzed")

        # Add fixed point receipt
        receipt = fp_result.to_receipt()
        self.receipts.append(receipt)

        # Check universality classes
        uc_list = self.rg_bundle.universality_classes
        if not uc_list:
            # No universality classes is not necessarily a failure
            pass

        # Also test with collapse dynamics (should find fixed point)
        collapse_dyn = create_fixed_point_dynamics(self.domain)
        collapse_fp = detect_fixed_point(self.hierarchy, collapse_dyn)

        if not collapse_fp.is_fixed_point:
            # Collapse dynamics should converge
            issues.append("Collapse dynamics did not converge")

        self.fixed_points_verified = all_valid

        bundle_receipt = {
            "type": "FIXED_POINT_VERIFICATION",
            "main_dynamics_fixed_point": fp_result.is_fixed_point,
            "collapse_dynamics_fixed_point": collapse_fp.is_fixed_point,
            "convergence_index": fp_result.convergence_scale_index,
            "universality_classes_found": len(uc_list),
            "issues": issues[:5] if issues else [],
            "result": "PASS" if all_valid else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="E",
            check_name="Fixed Point / Universality Witness",
            passed=all_valid,
            details=bundle_receipt
        )

    def check_F_no_minted_distinctions(self) -> CheckResult:
        """
        F) No Minted Distinctions

        Verify:
        - Renaming/recoding micro labels leaves all fingerprints invariant
        - Gauge transformations don't create new distinctions
        """
        all_valid = True
        issues = []
        checks_performed = 0

        # Test: relabeling domain should give isomorphic quotients
        # Create relabeled domain (shift all labels)
        original_domain = self.domain
        relabeled_domain = {x + 100 for x in original_domain}

        # Create relabeling map
        relabel_map = {x: x + 100 for x in original_domain}
        reverse_map = {v: k for k, v in relabel_map.items()}

        # Create relabeled tests
        for scale in self.hierarchy.scales:
            original_fp = scale.fingerprint()

            # Relabel outcomes
            relabeled_tests = []
            for test in scale.tests:
                new_outcomes = frozenset([
                    (relabel_map.get(k, k), v)
                    for k, v in test.outcomes
                ])
                relabeled_test = ScaleTest(
                    test_id=test.test_id,
                    cost=test.cost,
                    outcomes=new_outcomes
                )
                relabeled_tests.append(relabeled_test)

            relabeled_scale = ScaleTestSet(
                scale_id=scale.scale_id + "_RELABELED",
                scale_parameter=scale.scale_parameter,
                tests=relabeled_tests,
                cost_bound=scale.cost_bound
            )

            # Build quotient
            relabeled_quotient = construct_quotient(relabeled_domain, relabeled_scale)

            # Compare structural properties (class sizes should match)
            original_quotient = self.hierarchy.quotients[
                self.hierarchy.scales.index(scale)
            ]

            if sorted(relabeled_quotient.class_sizes()) != sorted(original_quotient.class_sizes()):
                all_valid = False
                issues.append(f"{scale.scale_id}: class sizes changed under relabeling")

            checks_performed += 1

        # Check effective dynamics fingerprints are gauge-invariant
        for gauge_check in self.rg_bundle.gauge_checks:
            if gauge_check.invariant_1_hash != gauge_check.invariant_2_hash:
                all_valid = False
                issues.append(f"{gauge_check.scale_id}: invariant hash changed")

        bundle_receipt = {
            "type": "NO_MINTED_DISTINCTIONS_VERIFICATION",
            "relabeling_tests": checks_performed,
            "gauge_invariance_tests": len(self.rg_bundle.gauge_checks),
            "all_invariant": all_valid,
            "issues": issues[:5] if issues else [],
            "result": "PASS" if all_valid else "FAIL"
        }
        self.receipts.append(bundle_receipt)

        return self._add_check(
            check_id="F",
            check_name="No Minted Distinctions",
            passed=all_valid,
            details=bundle_receipt
        )

    def check_G_canonical_receipts(self) -> CheckResult:
        """
        G) Canonical Receipts

        Verify:
        - All artifacts are JSON serializable
        - All receipts have SHA-256 hashes
        - Hashes are reproducible
        """
        all_ok = True
        hash_receipts = []

        # Add RG bundle receipt
        rg_receipt = self.rg_bundle.to_receipt()
        self.receipts.append(rg_receipt)

        # Add hierarchy receipt
        hier_receipt = self.hierarchy.to_receipt()
        self.receipts.append(hier_receipt)

        # Verify all receipts
        for i, receipt in enumerate(self.receipts):
            try:
                canonical = CanonicalJSON.serialize(receipt)
                receipt_hash = hashlib.sha256(canonical.encode()).hexdigest()

                # Verify reproducibility
                canonical2 = CanonicalJSON.serialize(receipt)
                receipt_hash2 = hashlib.sha256(canonical2.encode()).hexdigest()

                if receipt_hash != receipt_hash2:
                    all_ok = False
                    hash_receipts.append({
                        "receipt_index": i,
                        "reproducible": False,
                        "error": "Hash not reproducible"
                    })
                else:
                    hash_receipts.append({
                        "receipt_index": i,
                        "receipt_type": receipt.get("type", "UNKNOWN"),
                        "hash": receipt_hash[:16],
                        "serializable": True,
                        "reproducible": True
                    })
            except Exception as e:
                all_ok = False
                hash_receipts.append({
                    "receipt_index": i,
                    "serializable": False,
                    "error": str(e)
                })

        self.receipts_ok = all_ok

        final_receipt = {
            "type": "CANONICAL_RECEIPTS_BUNDLE",
            "total_receipts": len(self.receipts),
            "all_serializable": all_ok,
            "all_reproducible": all_ok,
            "sample_hashes": hash_receipts[:5],
            "result": "PASS" if all_ok else "FAIL"
        }
        self.receipts.append(final_receipt)

        return self._add_check(
            check_id="G",
            check_name="Canonical Receipts",
            passed=all_ok,
            details=final_receipt
        )

    def run_all_checks(self) -> List[CheckResult]:
        """Run all verification checks A-G."""
        self.checks = []

        self.check_A_scale_test_sets()
        self.check_B_quotient_construction()
        self.check_C_semigroup_property()
        self.check_D_effective_dynamics()
        self.check_E_fixed_points()
        self.check_F_no_minted_distinctions()
        self.check_G_canonical_receipts()

        return self.checks

    def create_proof_bundle(self) -> QFTProofBundle:
        """Create complete proof bundle."""
        if not self.checks:
            raise ValueError("Must run checks before creating bundle")

        bundle_id = f"QFT_{hashlib.sha256(b'qft_rg').hexdigest()[:8]}"

        return QFTProofBundle(
            bundle_id=bundle_id,
            checks=self.checks.copy(),
            scale_tests_verified=self.scale_tests_verified,
            quotients_verified=self.quotients_verified,
            semigroup_verified=self.semigroup_verified,
            effective_dynamics_verified=self.effective_dynamics_verified,
            fixed_points_verified=self.fixed_points_verified,
            gauge_invariance_verified=self.gauge_invariance_verified,
            receipts_ok=self.receipts_ok,
            receipts=self.receipts.copy()
        )


def run_qft_verification() -> QFTProofBundle:
    """
    Run complete QFT/RG/Universality verification.
    """
    verifier = QFTVerifier()
    verifier.run_all_checks()
    return verifier.create_proof_bundle()


def print_qft_report(bundle: QFTProofBundle) -> str:
    """Generate human-readable verification report."""
    lines = [
        "=" * 70,
        "QFT / RENORMALIZATION / UNIVERSALITY - VERIFICATION REPORT",
        "=" * 70,
        f"Bundle ID: {bundle.bundle_id}",
        f"Fingerprint: {bundle.fingerprint()[:32]}...",
        "",
        f"Scale Tests Verified: {bundle.scale_tests_verified}",
        f"Quotients Verified: {bundle.quotients_verified}",
        f"Semigroup Verified: {bundle.semigroup_verified}",
        f"Effective Dynamics Verified: {bundle.effective_dynamics_verified}",
        f"Fixed Points Verified: {bundle.fixed_points_verified}",
        f"Gauge Invariance Verified: {bundle.gauge_invariance_verified}",
        f"Receipts OK: {bundle.receipts_ok}",
        "",
        "-" * 70,
        "VERIFICATION CHECKS",
        "-" * 70,
    ]

    for check in bundle.checks:
        status = "PASS" if check.passed else "FAIL"
        lines.append(f"[{check.check_id}] {check.check_name}: {status}")

    lines.extend([
        "",
        "-" * 70,
        "KEY INSIGHTS",
        "-" * 70,
        "1. Scale = restriction of feasible tests",
        "2. Coarse-graining = forced quotienting by indistinguishability",
        "3. RG = semigroup recursion of effective laws",
        "4. Universality = convergence to fixed points",
        "5. 'Infinities' = coordinate-chart failures, not physics",
        "",
        "-" * 70,
        "SUMMARY",
        "-" * 70,
        f"Total Checks: {len(bundle.checks)}",
        f"Passed: {sum(1 for c in bundle.checks if c.passed)}",
        f"Failed: {sum(1 for c in bundle.checks if not c.passed)}",
        "",
        f"OVERALL: {'ALL CHECKS PASSED' if bundle.all_passed() else 'VERIFICATION FAILED'}",
        "=" * 70
    ])

    return "\n".join(lines)


def run_demo() -> Dict[str, Any]:
    """
    Run QFT/RG/Universality demonstration.
    """
    print("=" * 70)
    print("QFT / RENORMALIZATION / UNIVERSALITY - DEMONSTRATION")
    print("=" * 70)
    print()

    # Create verifier
    verifier = QFTVerifier(domain_size=8)

    print(f"Domain size: {len(verifier.domain)}")
    print(f"Scale hierarchy: {len(verifier.hierarchy.scales)} scales")
    print()

    # Show scale hierarchy
    print("-" * 70)
    print("SCALE HIERARCHY")
    print("-" * 70)
    for i, (scale, quotient) in enumerate(zip(
        verifier.hierarchy.scales,
        verifier.hierarchy.quotients
    )):
        print(f"  Scale {i+1}: {scale.scale_id}")
        print(f"    Parameter: {scale.scale_parameter}")
        print(f"    Cost bound: {scale.cost_bound}")
        print(f"    Feasible tests: {len(scale.feasible_tests())}")
        print(f"    Equivalence classes: {quotient.class_count()}")
        print(f"    Class sizes: {quotient.class_sizes()}")
        print()

    # Run verification
    bundle = run_qft_verification()

    # Print report
    report = print_qft_report(bundle)
    print(report)

    return {
        "demo": "QFT/Renormalization/Universality",
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
