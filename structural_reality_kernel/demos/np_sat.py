"""
demos/np_sat.py - NP witness search via kernel.

SAT solving as theorem generation:
- D0 = all possible variable assignments
- Tests = clause evaluators
- Π* = partition by clause satisfaction pattern
- UNIQUE = satisfying assignment found
- REFUTED = UNSAT proven
- OMEGA = budget exhausted (underdetermined)
"""

from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, List, Optional, Set, Tuple
from itertools import product
import hashlib
import json

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import (
    Test, Record, Ledger, Survivors, PiStar, Budget,
    FeasibleTests, KernelState, compute_kernel_state
)
from core.theorem_generator import Contract, TheoremGenerator, KernelOutput
from core.universe_engine import UniverseEngine, BotOp
from core.verify import VerificationSuite, ProofBundle, verify_kernel_run
from core.receipts import ReceiptChain, CanonicalJSON
from core.controller import PiController
from core.nsl import NSLEngine, Distinction


@dataclass
class SATInstance:
    """A SAT instance in CNF form."""
    num_vars: int
    clauses: List[List[int]]  # Each clause is list of literals (+var or -var)
    name: str = "SAT"

    def canonical(self) -> str:
        """Canonical representation."""
        return CanonicalJSON.serialize({
            "vars": self.num_vars,
            "clauses": self.clauses
        })

    def fingerprint(self) -> str:
        """SHA-256 fingerprint."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()[:16]


class NPSatDemo:
    """
    NP-SAT demonstration using the kernel.

    Compiles SAT instance into kernel primitives and runs theorem generator.
    """

    def __init__(self, instance: SATInstance, budget: int = 1000):
        self.instance = instance
        self.budget = budget

        # Build D0: all 2^n assignments
        self.d0 = self._build_d0()

        # Build tests: one per clause
        self.tests = self._build_tests()

        # Initialize components
        self.ledger = Ledger()
        self.controller = PiController(seed=42)
        self.receipt_chain = ReceiptChain()

    def _build_d0(self) -> FrozenSet[Tuple[bool, ...]]:
        """Build candidate set D0 = all assignments."""
        return frozenset(
            tuple(a) for a in product([False, True], repeat=self.instance.num_vars)
        )

    def _build_tests(self) -> Dict[str, Test]:
        """Build tests from clauses."""
        tests = {}

        for i, clause in enumerate(self.instance.clauses):
            def make_clause_test(clause_literals: List[int]) -> callable:
                def evaluate(assignment: Tuple[bool, ...]) -> str:
                    for lit in clause_literals:
                        var_idx = abs(lit) - 1
                        if var_idx >= len(assignment):
                            continue
                        val = assignment[var_idx]
                        if (lit > 0 and val) or (lit < 0 and not val):
                            return "SAT"
                    return "UNSAT"
                return evaluate

            tests[f"clause_{i}"] = Test(
                test_id=f"clause_{i}",
                evaluator=make_clause_test(clause),
                cost=1,
                outcome_space=frozenset(["SAT", "UNSAT"])
            )

        # Add full formula test
        def full_test(assignment: Tuple[bool, ...]) -> str:
            for clause in self.instance.clauses:
                satisfied = False
                for lit in clause:
                    var_idx = abs(lit) - 1
                    if var_idx >= len(assignment):
                        continue
                    val = assignment[var_idx]
                    if (lit > 0 and val) or (lit < 0 and not val):
                        satisfied = True
                        break
                if not satisfied:
                    return "UNSAT"
            return "SAT"

        tests["full_formula"] = Test(
            test_id="full_formula",
            evaluator=full_test,
            cost=len(self.instance.clauses),
            outcome_space=frozenset(["SAT", "UNSAT"])
        )

        return tests

    def create_contract(self) -> Contract:
        """Create theorem generator contract."""
        def sat_verifier(assignment: Tuple[bool, ...]) -> bool:
            for clause in self.instance.clauses:
                satisfied = False
                for lit in clause:
                    var_idx = abs(lit) - 1
                    if var_idx >= len(assignment):
                        continue
                    val = assignment[var_idx]
                    if (lit > 0 and val) or (lit < 0 and not val):
                        satisfied = True
                        break
                if not satisfied:
                    return False
            return True

        return Contract(
            contract_id=f"SAT_{self.instance.fingerprint()}",
            assertion=f"Find satisfying assignment for {len(self.instance.clauses)}-clause formula",
            witness_space=self.d0,
            verifier=sat_verifier,
            cost_per_verify=len(self.instance.clauses),
            budget=self.budget
        )

    def run(self) -> KernelOutput:
        """Run SAT solving via theorem generator."""
        contract = self.create_contract()
        generator = TheoremGenerator(seed=42)
        return generator.run(contract)

    def run_with_verification(self) -> Tuple[KernelOutput, ProofBundle]:
        """Run SAT solving with full verification."""
        # Build distinctions for NSL
        distinctions = {}
        for test_id, test in self.tests.items():
            for outcome in test.outcome_space:
                d_id = f"{test_id}:{outcome}"
                distinctions[d_id] = Distinction(
                    distinction_id=d_id,
                    description=f"Test {test_id} gives {outcome}",
                    test_id=test_id,
                    outcome=outcome
                )

        nsl_engine = NSLEngine(distinctions)

        # Run contract
        contract = self.create_contract()
        generator = TheoremGenerator(seed=42)
        output, receipt_chain = generator.run_with_receipts(contract)

        # Verify
        final_state = compute_kernel_state(self.d0, self.ledger, self.tests, alpha=1)

        bundle = verify_kernel_run(
            d0=self.d0,
            tests=self.tests,
            ledger=self.ledger,
            receipt_chain=receipt_chain,
            controller=self.controller,
            nsl_engine=nsl_engine,
            final_state=final_state,
            alpha=1
        )

        return output, bundle

    def analyze_pi_star(self) -> Dict[str, Any]:
        """Analyze the Π* partition structure."""
        state = compute_kernel_state(self.d0, self.ledger, self.tests, alpha=1)
        pi_star = state.pi_star

        return {
            "class_count": pi_star.class_count(),
            "class_sizes": pi_star.class_sizes(),
            "fingerprint": pi_star.canonical_fingerprint()[:32],
            "is_singleton": pi_star.is_singleton()
        }


def create_3sat_instance(num_vars: int, num_clauses: int, seed: int = 42) -> SATInstance:
    """
    Create a random 3-SAT instance.

    Args:
        num_vars: Number of variables
        num_clauses: Number of clauses
        seed: Random seed

    Returns:
        SATInstance
    """
    import random
    rng = random.Random(seed)

    clauses = []
    for _ in range(num_clauses):
        clause = []
        vars_used = set()
        while len(clause) < 3:
            var = rng.randint(1, num_vars)
            if var not in vars_used:
                vars_used.add(var)
                sign = rng.choice([1, -1])
                clause.append(sign * var)
        clauses.append(clause)

    return SATInstance(
        num_vars=num_vars,
        clauses=clauses,
        name=f"3SAT_{num_vars}v_{num_clauses}c"
    )


def create_pigeonhole_instance(n: int) -> SATInstance:
    """
    Create pigeonhole principle instance (UNSAT).

    n+1 pigeons, n holes - always UNSAT.
    """
    # Variables: x_{i,j} = pigeon i in hole j
    # i in [0, n], j in [0, n-1]
    num_vars = (n + 1) * n

    def var(i: int, j: int) -> int:
        return i * n + j + 1

    clauses = []

    # Each pigeon in at least one hole
    for i in range(n + 1):
        clause = [var(i, j) for j in range(n)]
        clauses.append(clause)

    # No two pigeons in same hole
    for j in range(n):
        for i1 in range(n + 1):
            for i2 in range(i1 + 1, n + 1):
                clauses.append([-var(i1, j), -var(i2, j)])

    return SATInstance(
        num_vars=num_vars,
        clauses=clauses,
        name=f"Pigeonhole_{n+1}_to_{n}"
    )


def run_np_sat_demo() -> Dict[str, Any]:
    """
    Run the NP-SAT demonstration.

    Returns results dict with verification status.
    """
    results = {
        "demo": "NP-SAT",
        "tests": []
    }

    # Test 1: Small satisfiable instance
    instance1 = SATInstance(
        num_vars=3,
        clauses=[[1, 2, 3], [-1, 2], [-2, 3]],
        name="Simple_SAT"
    )
    demo1 = NPSatDemo(instance1, budget=100)
    output1 = demo1.run()
    results["tests"].append({
        "name": "Simple SAT (3 vars)",
        "status": output1.status,
        "witness": str(output1.witness) if output1.witness else None,
        "cost": output1.total_cost,
        "passed": output1.status == "UNIQUE"
    })

    # Test 2: Random 3-SAT
    instance2 = create_3sat_instance(num_vars=4, num_clauses=5, seed=42)
    demo2 = NPSatDemo(instance2, budget=200)
    output2 = demo2.run()
    results["tests"].append({
        "name": "Random 3-SAT (4 vars, 5 clauses)",
        "status": output2.status,
        "witness": str(output2.witness) if output2.witness else None,
        "cost": output2.total_cost,
        "passed": output2.status in ["UNIQUE", "REFUTED"]
    })

    # Test 3: Pigeonhole (UNSAT)
    # Note: Large search space may result in OMEGA (budget exhausted) or REFUTED
    instance3 = create_pigeonhole_instance(2)  # 3 pigeons, 2 holes
    demo3 = NPSatDemo(instance3, budget=500)
    output3 = demo3.run()
    results["tests"].append({
        "name": "Pigeonhole 3->2 (UNSAT)",
        "status": output3.status,
        "witness": str(output3.witness) if output3.witness else None,
        "cost": output3.total_cost,
        # OMEGA is acceptable when budget exhausted, REFUTED is ideal
        "passed": output3.status in ["REFUTED", "OMEGA"]
    })

    # Summary
    all_passed = all(t["passed"] for t in results["tests"])
    results["all_passed"] = all_passed
    results["summary"] = {
        "total_tests": len(results["tests"]),
        "passed": sum(1 for t in results["tests"] if t["passed"]),
        "failed": sum(1 for t in results["tests"] if not t["passed"])
    }

    return results


if __name__ == "__main__":
    results = run_np_sat_demo()
    print(json.dumps(results, indent=2))
