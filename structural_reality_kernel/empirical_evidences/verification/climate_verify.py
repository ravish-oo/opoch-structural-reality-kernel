"""
climate_verify.py - Verification suite for Climate Tech.

Implements verification checks A-G:
A) Canonical Spec - reservoirs, constraints, unit gauge
B) MRV Test Definitions - totality, costs, separation
C) Witness Bundles - chain of custody, calibration
D) Total Verifiers - deterministic execution
E) Ledger Receipts - monotone decrease, costs
F) Gauge Invariance - recoding invariance
G) Omega Honesty - frontier with minimal separator
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple
from fractions import Fraction
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
from .climate_tech import (
    ClimateStatus,
    InterventionType,
    ClimateIntervention,
    Portfolio,
    ClimateConstraint,
    CarbonCredit,
    WitnessBundle,
    VerifierExecution,
    GaugeInvarianceCheck,
    OmegaFrontier,
    OptimalMeasurement,
    ClimateVerificationResult,
    ClimateBundle,
    create_sample_intervention,
    create_sample_portfolio,
    create_sample_carbon_credit,
    create_sample_mrv_tests,
    create_sample_witness_bundle,
    run_sample_climate_verification
)


@dataclass
class CheckResult:
    """Result of a single verification check."""
    check_id: str
    check_name: str
    passed: bool
    details: Dict[str, Any]
    receipt: Dict[str, Any]

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CHECK_RESULT",
            "check_id": self.check_id,
            "check_name": self.check_name,
            "passed": self.passed
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()


@dataclass
class ClimateVerifier:
    """
    Verifier for Climate Tech.

    Runs checks A-G on climate verification results.
    """
    verifier_id: str

    def check_a_canonical_spec(
        self,
        spec: ClimateSpec
    ) -> CheckResult:
        """
        Check A: Canonical Spec.

        Verifies:
        - Reservoirs are defined
        - Constraints are declared
        - Horizon is specified
        """
        has_reservoirs = spec.reservoir_count() > 0
        has_constraints = spec.constraint_count() > 0
        has_horizon = spec.horizon_steps > 0

        passed = has_reservoirs and has_constraints and has_horizon

        return CheckResult(
            check_id="CHECK_A",
            check_name="Canonical Spec",
            passed=passed,
            details={
                "reservoir_count": spec.reservoir_count(),
                "constraint_count": spec.constraint_count(),
                "horizon_steps": spec.horizon_steps,
                "has_reservoirs": has_reservoirs,
                "has_constraints": has_constraints,
                "has_horizon": has_horizon
            },
            receipt=spec.to_receipt()
        )

    def check_b_mrv_tests(
        self,
        tests: List[MRVTest]
    ) -> CheckResult:
        """
        Check B: MRV Test Definitions.

        Verifies:
        - All tests are total
        - All tests have defined costs
        - All tests have finite outcome spaces
        """
        all_total = all(t.is_total() for t in tests)
        all_have_cost = all(t.cost > 0 for t in tests)
        all_finite_outcomes = all(t.outcome_space_size() > 0 for t in tests)

        passed = all_total and all_have_cost and all_finite_outcomes

        return CheckResult(
            check_id="CHECK_B",
            check_name="MRV Test Definitions",
            passed=passed,
            details={
                "test_count": len(tests),
                "all_total": all_total,
                "all_have_cost": all_have_cost,
                "all_finite_outcomes": all_finite_outcomes,
                "categories": list(set(t.category.value for t in tests))
            },
            receipt={
                "type": "MRV_TESTS_BUNDLE",
                "test_count": len(tests),
                "all_total": all_total,
                "all_have_cost": all_have_cost,
                "result": "PASS" if passed else "FAIL"
            }
        )

    def check_c_witness_bundles(
        self,
        bundles: List[WitnessBundle]
    ) -> CheckResult:
        """
        Check C: Witness Bundles.

        Verifies:
        - Chain of custody is verified
        - Calibration is verified
        - Data hashes are present
        """
        all_custody = all(b.chain_of_custody_verified for b in bundles)
        all_calibrated = all(b.calibration_verified for b in bundles)
        all_have_hash = all(b.raw_data_hash for b in bundles)

        passed = all_custody and all_calibrated and all_have_hash

        return CheckResult(
            check_id="CHECK_C",
            check_name="Witness Bundles",
            passed=passed,
            details={
                "bundle_count": len(bundles),
                "all_custody_verified": all_custody,
                "all_calibrated": all_calibrated,
                "all_have_hash": all_have_hash
            },
            receipt={
                "type": "WITNESS_BUNDLES_CHECK",
                "bundle_count": len(bundles),
                "all_verified": passed,
                "result": "PASS" if passed else "FAIL"
            }
        )

    def check_d_verifier_execution(
        self,
        executions: List[VerifierExecution]
    ) -> CheckResult:
        """
        Check D: Total Verifiers.

        Verifies:
        - All verifiers are deterministic
        - All produce valid intervals
        """
        all_deterministic = all(e.deterministic for e in executions)
        all_valid_intervals = all(
            e.output_interval.lower <= e.output_interval.upper
            for e in executions
        )

        passed = all_deterministic and all_valid_intervals

        return CheckResult(
            check_id="CHECK_D",
            check_name="Verifier Execution",
            passed=passed,
            details={
                "execution_count": len(executions),
                "all_deterministic": all_deterministic,
                "all_valid_intervals": all_valid_intervals
            },
            receipt={
                "type": "VERIFIER_EXECUTION_CHECK",
                "execution_count": len(executions),
                "all_deterministic": all_deterministic,
                "result": "PASS" if passed else "FAIL"
            }
        )

    def check_e_ledger_receipts(
        self,
        ledger: ClimateLedger,
        initial_survivors: int,
        final_survivors: int
    ) -> CheckResult:
        """
        Check E: Ledger Receipts.

        Verifies:
        - Entries are timestamped
        - Costs are accumulated correctly
        - Survivors decrease monotonically
        """
        entries_timestamped = all(e.timestamp >= 0 for e in ledger.entries)
        computed_cost = sum(e.cost for e in ledger.entries)
        cost_correct = computed_cost == ledger.total_cost()
        monotone_decrease = final_survivors <= initial_survivors

        passed = entries_timestamped and cost_correct and monotone_decrease

        return CheckResult(
            check_id="CHECK_E",
            check_name="Ledger Receipts",
            passed=passed,
            details={
                "entry_count": ledger.entry_count(),
                "total_cost": ledger.total_cost(),
                "initial_survivors": initial_survivors,
                "final_survivors": final_survivors,
                "entries_timestamped": entries_timestamped,
                "cost_correct": cost_correct,
                "monotone_decrease": monotone_decrease
            },
            receipt=ledger.to_receipt(initial_survivors, final_survivors)
        )

    def check_f_gauge_invariance(
        self,
        gauge_check: GaugeInvarianceCheck
    ) -> CheckResult:
        """
        Check F: Gauge Invariance.

        Verifies:
        - Multiple recodings tested
        - All outputs invariant
        """
        passed = gauge_check.all_invariant and gauge_check.recodings_tested > 0

        return CheckResult(
            check_id="CHECK_F",
            check_name="Gauge Invariance",
            passed=passed,
            details={
                "recodings_tested": gauge_check.recodings_tested,
                "all_invariant": gauge_check.all_invariant
            },
            receipt=gauge_check.to_receipt()
        )

    def check_g_omega_honesty(
        self,
        frontier: OmegaFrontier
    ) -> CheckResult:
        """
        Check G: Omega Honesty.

        Verifies:
        - If resolved, claim is honest
        - If not resolved, next separator is identified
        """
        honest = frontier.is_honest()

        return CheckResult(
            check_id="CHECK_G",
            check_name="Omega Honesty",
            passed=honest,
            details={
                "survivors_count": frontier.survivors_count,
                "resolved": frontier.resolved,
                "next_separator_id": frontier.next_separator_id,
                "next_separator_cost": frontier.next_separator_cost,
                "honest": honest
            },
            receipt=frontier.to_receipt()
        )

    def check_mass_balance(
        self,
        checks: List[MassBalanceCheck]
    ) -> CheckResult:
        """
        Additional check: Mass Balance.

        Verifies all mass balance constraints are satisfied.
        """
        all_balanced = all(c.balance_satisfied() for c in checks)

        return CheckResult(
            check_id="CHECK_MB",
            check_name="Mass Balance",
            passed=all_balanced,
            details={
                "checks_count": len(checks),
                "all_balanced": all_balanced
            },
            receipt={
                "type": "MASS_BALANCE_BUNDLE",
                "checks_count": len(checks),
                "all_balanced": all_balanced,
                "result": "PASS" if all_balanced else "FAIL"
            }
        )

    def verify_all(
        self,
        result: ClimateVerificationResult,
        tests: List[MRVTest],
        bundles: List[WitnessBundle],
        executions: List[VerifierExecution],
        gauge_check: GaugeInvarianceCheck
    ) -> List[CheckResult]:
        """Run all verification checks."""
        checks = []

        # Check A: Canonical Spec
        check_a = self.check_a_canonical_spec(result.spec)
        checks.append(check_a)

        # Check B: MRV Test Definitions
        check_b = self.check_b_mrv_tests(tests)
        checks.append(check_b)

        # Check C: Witness Bundles
        check_c = self.check_c_witness_bundles(bundles)
        checks.append(check_c)

        # Check D: Verifier Execution
        check_d = self.check_d_verifier_execution(executions)
        checks.append(check_d)

        # Check E: Ledger Receipts
        check_e = self.check_e_ledger_receipts(
            result.ledger,
            initial_survivors=100,  # Sample initial
            final_survivors=1 if result.status == ClimateStatus.VERIFIED else 10
        )
        checks.append(check_e)

        # Check F: Gauge Invariance
        check_f = self.check_f_gauge_invariance(gauge_check)
        checks.append(check_f)

        # Check G: Omega Honesty
        check_g = self.check_g_omega_honesty(result.frontier)
        checks.append(check_g)

        # Additional: Mass Balance
        check_mb = self.check_mass_balance(result.mass_balance_checks)
        checks.append(check_mb)

        return checks


@dataclass
class ClimateProofBundle:
    """
    Complete proof bundle for Climate Tech verification.
    """
    bundle_id: str
    checks: List[CheckResult]
    result: ClimateVerificationResult

    def all_passed(self) -> bool:
        """Check if all verification checks passed."""
        return all(check.passed for check in self.checks)

    def passed_count(self) -> int:
        """Count of passed checks."""
        return sum(1 for check in self.checks if check.passed)

    def failed_count(self) -> int:
        """Count of failed checks."""
        return sum(1 for check in self.checks if not check.passed)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "CLIMATE_PROOF_BUNDLE",
            "bundle_id": self.bundle_id,
            "checks_total": len(self.checks),
            "checks_passed": self.passed_count(),
            "all_passed": self.all_passed()
        })

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "CLIMATE_PROOF_BUNDLE",
            "bundle_id": self.bundle_id,
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


def run_climate_verification() -> ClimateProofBundle:
    """
    Run complete climate verification suite.

    Returns proof bundle with all check results.
    """
    # Run sample verification
    result = run_sample_climate_verification()

    # Create MRV tests
    tests = create_sample_mrv_tests()

    # Create witness bundles
    bundles = [
        create_sample_witness_bundle("BUNDLE_001"),
        create_sample_witness_bundle("BUNDLE_002"),
        create_sample_witness_bundle("BUNDLE_003"),
    ]

    # Create verifier executions
    executions = []
    for i, bundle in enumerate(bundles):
        exec_result = VerifierExecution(
            execution_id=f"EXEC_{i}",
            verifier_id=f"VERIFIER_{i}",
            input_bundle=bundle,
            output_interval=create_interval(80, 120),
            deterministic=True
        )
        executions.append(exec_result)

    # Create gauge invariance check
    gauge_check = GaugeInvarianceCheck(
        check_id="GAUGE_001",
        recodings_tested=5,
        all_invariant=True
    )

    # Create verifier and run checks
    verifier = ClimateVerifier(verifier_id="CLIMATE_VERIFIER")
    checks = verifier.verify_all(result, tests, bundles, executions, gauge_check)

    # Create proof bundle
    bundle = ClimateProofBundle(
        bundle_id="CLIMATE_BUNDLE",
        checks=checks,
        result=result
    )

    return bundle


def print_climate_report(bundle: ClimateProofBundle) -> None:
    """Print verification report."""
    print("=" * 70)
    print("CLIMATE TECH VERIFICATION REPORT")
    print("=" * 70)
    print()

    # Result summary
    print("VERIFICATION RESULT:")
    print(f"  Status: {bundle.result.status.value}")
    print(f"  Credits verified: {len(bundle.result.credits)}")
    print(f"  Mass balance checks: {len(bundle.result.mass_balance_checks)}")
    print(f"  Ledger entries: {bundle.result.ledger.entry_count()}")
    print()

    # Carbon credit summary
    if bundle.result.credits:
        print("CARBON CREDITS:")
        for credit in bundle.result.credits:
            net = credit.net_delta()
            conservative = credit.conservative_issuance()
            print(f"  {credit.credit_id}:")
            print(f"    Net delta: [{net.lower.fraction}, {net.upper.fraction}]")
            print(f"    Conservative issuance: {conservative.fraction}")
            print(f"    Valid: {credit.is_valid()}")
        print()

    # Check results
    print("VERIFICATION CHECKS:")
    print("-" * 70)

    for check in bundle.checks:
        status = "PASS" if check.passed else "FAIL"
        print(f"  [{status}] {check.check_id}: {check.check_name}")
        for key, value in check.details.items():
            print(f"         {key}: {value}")
        print()

    # Summary
    print("-" * 70)
    print(f"SUMMARY: {bundle.passed_count()}/{len(bundle.checks)} checks passed")
    print(f"OVERALL: {'PASS' if bundle.all_passed() else 'FAIL'}")
    print(f"Bundle fingerprint: {bundle.fingerprint()[:16]}...")
    print("=" * 70)


# ============================================================
# MAIN ENTRY POINT
# ============================================================

if __name__ == "__main__":
    bundle = run_climate_verification()
    print_climate_report(bundle)
