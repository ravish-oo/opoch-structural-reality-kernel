"""
Structural Reality Kernel - CLI

Usage:
    python -m structural_reality_kernel verify
    python -m structural_reality_kernel demo <name>
    python -m structural_reality_kernel all
"""

import argparse
import json
import sys
from typing import Dict, Any

from .core import (
    Ledger, Test, Record, Survivors, PiStar, Budget,
    compute_kernel_state, KernelState
)
from .core.verify import (
    VerificationSuite, ProofBundle, verify_kernel_run,
    print_verification_report
)
from .core.receipts import ReceiptChain, CanonicalJSON
from .core.controller import PiController
from .core.nsl import NSLEngine, Distinction
from .core.theorem_generator import Contract, TheoremGenerator


def run_core_verification() -> Dict[str, Any]:
    """
    Run core kernel verification tests.

    Tests fundamental axioms and properties.
    """
    results = {
        "test": "Core Verification",
        "checks": []
    }

    # Create simple test case
    d0 = frozenset(range(10))

    def parity_test(x: int) -> str:
        return "EVEN" if x % 2 == 0 else "ODD"

    def mod3_test(x: int) -> str:
        return f"MOD{x % 3}"

    tests = {
        "parity": Test(
            test_id="parity",
            evaluator=parity_test,
            cost=1,
            outcome_space=frozenset(["EVEN", "ODD"])
        ),
        "mod3": Test(
            test_id="mod3",
            evaluator=mod3_test,
            cost=1,
            outcome_space=frozenset(["MOD0", "MOD1", "MOD2"])
        )
    }

    # Initialize
    ledger = Ledger()
    controller = PiController(seed=42)
    receipt_chain = ReceiptChain()

    # Build distinctions
    distinctions = {}
    for test_id, test in tests.items():
        for outcome in test.outcome_space:
            d_id = f"{test_id}:{outcome}"
            distinctions[d_id] = Distinction(
                distinction_id=d_id,
                description=f"Test {test_id} gives {outcome}",
                test_id=test_id,
                outcome=outcome
            )

    nsl_engine = NSLEngine(distinctions)

    # Initial state
    initial_state = compute_kernel_state(d0, ledger, tests, alpha=1)
    receipt_chain.append(
        operation="INIT",
        pre_state_fp="GENESIS",
        post_state_fp=initial_state.canonical_fingerprint(),
        payload={"d0_size": len(d0)}
    )

    # Apply some tests (only if feasible)
    for x in [0, 1, 2]:  # Simulate "actual" being x
        # Check budget before applying
        survivors = Survivors(d0, ledger, tests)
        budget = Budget(len(survivors), alpha=1)
        test = tests["parity"]

        if not budget.is_feasible(test):
            break  # Budget exhausted

        outcome = test(x)
        record = Record(test_id="parity", outcome=outcome)
        ledger = ledger.append(record)

        state = compute_kernel_state(d0, ledger, tests, alpha=1)
        receipt_chain.append(
            operation="APPLY_TEST",
            pre_state_fp=initial_state.canonical_fingerprint(),
            post_state_fp=state.canonical_fingerprint(),
            payload={
                "test_id": "parity",
                "outcome": outcome,
                "w_pre": len(initial_state.survivors),
                "w_post": len(state.survivors),
                "cost": 1
            }
        )
        initial_state = state

    # Final state
    final_state = compute_kernel_state(d0, ledger, tests, alpha=1)

    # Run verification
    bundle = verify_kernel_run(
        d0=d0,
        tests=tests,
        ledger=ledger,
        receipt_chain=receipt_chain,
        controller=controller,
        nsl_engine=nsl_engine,
        final_state=final_state,
        alpha=1
    )

    results["checks"] = [
        {"id": c.check_id, "name": c.check_name, "passed": c.passed}
        for c in bundle.checks
    ]
    results["all_passed"] = bundle.all_passed()
    results["bundle_fingerprint"] = bundle.fingerprint()[:32]

    return results


def run_demo(demo_name: str) -> Dict[str, Any]:
    """Run a specific demo."""
    if demo_name == "np_sat":
        from .demos.np_sat import run_np_sat_demo
        return run_np_sat_demo()
    elif demo_name == "mapf":
        from .demos.mapf import run_mapf_demo
        return run_mapf_demo()
    elif demo_name == "quantum_gns":
        from .demos.quantum_gns import run_quantum_gns_demo
        return run_quantum_gns_demo()
    elif demo_name == "gravity":
        from .demos.gravity_cost_geometry import run_gravity_demo
        return run_gravity_demo()
    else:
        return {"error": f"Unknown demo: {demo_name}"}


def run_all() -> Dict[str, Any]:
    """Run all verifications and demos."""
    results = {
        "structural_reality_kernel": "1.0.0",
        "verification": {},
        "demos": {}
    }

    print("=" * 60)
    print("STRUCTURAL REALITY KERNEL - FULL VERIFICATION")
    print("=" * 60)
    print()

    # Core verification
    print("Running core verification...")
    core_result = run_core_verification()
    results["verification"]["core"] = core_result
    status = "PASS" if core_result["all_passed"] else "FAIL"
    print(f"  Core Verification: {status}")
    for check in core_result["checks"]:
        check_status = "OK" if check["passed"] else "FAIL"
        print(f"    [{check['id']}] {check['name']}: {check_status}")
    print()

    # Demos
    demos = ["np_sat", "mapf", "quantum_gns", "gravity"]
    for demo_name in demos:
        print(f"Running {demo_name} demo...")
        try:
            demo_result = run_demo(demo_name)
            results["demos"][demo_name] = demo_result
            status = "PASS" if demo_result.get("all_passed", False) else "FAIL"
            print(f"  {demo_name}: {status}")
            if "tests" in demo_result:
                for test in demo_result["tests"]:
                    test_status = "OK" if test.get("passed", False) else "FAIL"
                    print(f"    - {test.get('name', 'unknown')}: {test_status}")
        except Exception as e:
            results["demos"][demo_name] = {"error": str(e)}
            print(f"  {demo_name}: ERROR - {e}")
        print()

    # Summary
    print("=" * 60)
    print("SUMMARY")
    print("=" * 60)

    all_passed = core_result["all_passed"]
    for demo_name, demo_result in results["demos"].items():
        if "error" in demo_result:
            all_passed = False
        elif not demo_result.get("all_passed", False):
            all_passed = False

    results["all_passed"] = all_passed
    print(f"Overall Result: {'ALL PASS' if all_passed else 'SOME FAILURES'}")
    print("=" * 60)

    return results


def main():
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Structural Reality Kernel CLI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python -m structural_reality_kernel verify
    python -m structural_reality_kernel demo np_sat
    python -m structural_reality_kernel demo mapf
    python -m structural_reality_kernel demo quantum_gns
    python -m structural_reality_kernel demo gravity
    python -m structural_reality_kernel all
        """
    )

    subparsers = parser.add_subparsers(dest="command", help="Command to run")

    # verify command
    verify_parser = subparsers.add_parser("verify", help="Run core verification")

    # demo command
    demo_parser = subparsers.add_parser("demo", help="Run a specific demo")
    demo_parser.add_argument(
        "name",
        choices=["np_sat", "mapf", "quantum_gns", "gravity"],
        help="Demo to run"
    )

    # all command
    all_parser = subparsers.add_parser("all", help="Run all verifications and demos")

    # json output option
    parser.add_argument(
        "--json",
        action="store_true",
        help="Output results as JSON"
    )

    args = parser.parse_args()

    if args.command == "verify":
        result = run_core_verification()
    elif args.command == "demo":
        result = run_demo(args.name)
    elif args.command == "all":
        result = run_all()
    else:
        parser.print_help()
        sys.exit(1)

    if args.json or args.command in ["verify", "demo"]:
        print(json.dumps(result, indent=2, default=str))

    # Exit with appropriate code
    if result.get("all_passed", True):
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
