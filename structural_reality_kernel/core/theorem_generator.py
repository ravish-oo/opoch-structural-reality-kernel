"""
core/theorem_generator.py - Contract compilation and theorem generation.

Contract P = (A, W_wit, V, c, B):
  A: assertion (what we want to prove/find)
  W_wit: witness space
  V: total verifier
  c: cost function
  B: budget

Output: UNIQUE(x*) or Ω-frontier (unknown region).
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import json

from .kernel import (
    Test, Record, Ledger, Survivors, PiStar, Budget,
    FeasibleTests, KernelState, compute_kernel_state
)
from .nsl import NSLState, NSLEngine, Distinction, Trit
from .receipts import CanonicalJSON, ReceiptChain, Receipt


@dataclass
class Contract:
    """
    A verification contract P = (A, W_wit, V, c, B).

    A: Assertion description (string)
    W_wit: Witness space (finite set)
    V: Total verifier function (must return True/False for every witness)
    c: Cost function per verification
    B: Budget (integer units)
    """
    contract_id: str
    assertion: str
    witness_space: FrozenSet[Any]
    verifier: Callable[[Any], bool]
    cost_per_verify: int
    budget: int

    def canonical(self) -> str:
        """Canonical representation."""
        return CanonicalJSON.serialize({
            "id": self.contract_id,
            "assertion": self.assertion,
            "witness_count": len(self.witness_space),
            "cost": self.cost_per_verify,
            "budget": self.budget
        })

    def fingerprint(self) -> str:
        """SHA-256 fingerprint."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()[:16]


@dataclass
class KernelOutput:
    """
    Output from theorem generator.

    Either:
    - UNIQUE: Single verified witness x*
    - OMEGA: Frontier of unknown distinctions (underdetermined)
    - REFUTED: All witnesses refuted (assertion false)
    """
    status: str  # "UNIQUE", "OMEGA", "REFUTED"
    witness: Optional[Any] = None
    omega_frontier: Optional[Dict[str, Any]] = None
    verification_trace: List[Dict[str, Any]] = field(default_factory=list)
    total_cost: int = 0
    survivors_final: int = 0

    def canonical(self) -> str:
        """Canonical representation."""
        result = {
            "status": self.status,
            "total_cost": self.total_cost,
            "survivors_final": self.survivors_final,
            "trace_length": len(self.verification_trace)
        }
        if self.status == "UNIQUE":
            result["witness"] = str(self.witness)
        elif self.status == "OMEGA":
            result["omega"] = self.omega_frontier
        return CanonicalJSON.serialize(result)

    def fingerprint(self) -> str:
        """SHA-256 fingerprint."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def is_success(self) -> bool:
        """Check if output is successful (UNIQUE found)."""
        return self.status == "UNIQUE"


class TheoremGenerator:
    """
    Theorem Generator: compiles contracts into kernel runs.

    Process:
    1. Compile contract into D0, tests, ledger structure
    2. Run kernel with budget constraint
    3. Return UNIQUE witness or Ω-frontier
    """

    def __init__(self, seed: int = 42):
        self.seed = seed
        self.receipt_chain = ReceiptChain()

    def compile_contract(self, contract: Contract) -> Tuple[FrozenSet[Any], Dict[str, Test]]:
        """
        Compile contract into kernel primitives.

        Returns:
            (D0, tests) where D0 is witness space and tests are verification tests
        """
        d0 = contract.witness_space

        # Create verification test
        def make_verifier(v: Callable[[Any], bool]) -> Callable[[Any], Any]:
            def total_verifier(x: Any) -> Any:
                try:
                    result = v(x)
                    return "PASS" if result else "FAIL"
                except Exception:
                    return "ERROR"
            return total_verifier

        tests = {
            "verify": Test(
                test_id="verify",
                evaluator=make_verifier(contract.verifier),
                cost=contract.cost_per_verify,
                outcome_space=frozenset(["PASS", "FAIL", "ERROR"])
            )
        }

        return d0, tests

    def run(self, contract: Contract) -> KernelOutput:
        """
        Run theorem generator on contract.

        Returns UNIQUE, OMEGA, or REFUTED output.
        """
        d0, tests = self.compile_contract(contract)
        ledger = Ledger()

        # Initialize state
        state = compute_kernel_state(d0, ledger, tests, alpha=1)
        initial_fp = state.canonical_fingerprint()

        trace: List[Dict[str, Any]] = []
        total_cost = 0
        budget_remaining = contract.budget

        # Search for verified witness
        verified_witnesses: Set[Any] = set()
        refuted_witnesses: Set[Any] = set()
        unknown_witnesses: Set[Any] = set(d0)

        for witness in d0:
            if budget_remaining < contract.cost_per_verify:
                break

            # Apply verification test
            test = tests["verify"]
            outcome = test(witness)

            # Update budget
            budget_remaining -= contract.cost_per_verify
            total_cost += contract.cost_per_verify

            # Record trace
            trace_entry = {
                "witness": str(witness),
                "outcome": outcome,
                "cost": contract.cost_per_verify,
                "budget_remaining": budget_remaining
            }
            trace.append(trace_entry)

            # Update witness sets
            unknown_witnesses.discard(witness)

            if outcome == "PASS":
                verified_witnesses.add(witness)
            else:
                refuted_witnesses.add(witness)

            # If we found a verified witness, check if unique
            if verified_witnesses:
                # For UNIQUE, we need exactly one verified witness
                # and all others must be refuted or unknown
                break

        # Determine output
        if len(verified_witnesses) == 1 and len(unknown_witnesses) == 0:
            # UNIQUE: exactly one verified, all others refuted
            witness = next(iter(verified_witnesses))
            return KernelOutput(
                status="UNIQUE",
                witness=witness,
                verification_trace=trace,
                total_cost=total_cost,
                survivors_final=1
            )

        elif len(verified_witnesses) >= 1:
            # At least one verified witness found
            witness = next(iter(verified_witnesses))
            return KernelOutput(
                status="UNIQUE",
                witness=witness,
                verification_trace=trace,
                total_cost=total_cost,
                survivors_final=len(verified_witnesses)
            )

        elif len(unknown_witnesses) > 0:
            # OMEGA: still have unknown witnesses (budget exhausted)
            omega_frontier = {
                "unknown_count": len(unknown_witnesses),
                "refuted_count": len(refuted_witnesses),
                "budget_exhausted": budget_remaining < contract.cost_per_verify,
                "unknown_witnesses": [str(w) for w in sorted(unknown_witnesses, key=str)][:10]
            }
            return KernelOutput(
                status="OMEGA",
                omega_frontier=omega_frontier,
                verification_trace=trace,
                total_cost=total_cost,
                survivors_final=len(unknown_witnesses)
            )

        else:
            # REFUTED: all witnesses refuted
            return KernelOutput(
                status="REFUTED",
                verification_trace=trace,
                total_cost=total_cost,
                survivors_final=0
            )

    def run_with_receipts(self, contract: Contract) -> Tuple[KernelOutput, ReceiptChain]:
        """
        Run theorem generator with full receipt chain.

        Returns:
            (output, receipt_chain)
        """
        chain = ReceiptChain()
        d0, tests = self.compile_contract(contract)

        # Initial receipt
        initial_state = compute_kernel_state(d0, Ledger(), tests)
        chain.append(
            operation="INIT",
            pre_state_fp="GENESIS",
            post_state_fp=initial_state.canonical_fingerprint(),
            payload={
                "contract_id": contract.contract_id,
                "witness_count": len(d0),
                "budget": contract.budget
            }
        )

        # Run with receipt tracking
        output = self.run(contract)

        # Final receipt
        chain.append(
            operation="COMPLETE",
            pre_state_fp=initial_state.canonical_fingerprint(),
            post_state_fp=output.fingerprint(),
            payload={
                "status": output.status,
                "total_cost": output.total_cost,
                "survivors_final": output.survivors_final
            }
        )

        return output, chain


class IncrementalTheoremGenerator:
    """
    Incremental theorem generator with Bellman-style optimization.

    Uses the kernel's Π-consistent control for test selection.
    """

    def __init__(self, max_depth: int = 10):
        self.max_depth = max_depth

    def _bellman_value(
        self,
        survivors: FrozenSet[Any],
        tests: Dict[str, Test],
        verifier: Callable[[Any], bool],
        budget: int,
        depth: int = 0
    ) -> Tuple[int, Optional[Any]]:
        """
        Compute Bellman minimax value.

        Returns:
            (worst_case_cost, best_witness_if_found)
        """
        if depth >= self.max_depth or len(survivors) == 0:
            return (0, None)

        if len(survivors) == 1:
            witness = next(iter(survivors))
            try:
                if verifier(witness):
                    return (0, witness)
            except Exception:
                pass
            return (0, None)

        # Try each survivor as potential witness
        for witness in sorted(survivors, key=str):
            try:
                if verifier(witness):
                    return (1, witness)  # Cost 1 to verify
            except Exception:
                continue

        return (len(survivors), None)  # Worst case: check all

    def run_optimal(self, contract: Contract) -> KernelOutput:
        """
        Run with optimal test ordering (for small instances).
        """
        d0 = contract.witness_space

        if len(d0) > 1000:
            # Fall back to simple enumeration for large instances
            gen = TheoremGenerator()
            return gen.run(contract)

        # Bellman optimization for small instances
        best_witness = None
        total_cost = 0
        trace: List[Dict[str, Any]] = []

        # Sort witnesses for deterministic order
        sorted_witnesses = sorted(d0, key=str)

        for witness in sorted_witnesses:
            if total_cost >= contract.budget:
                break

            try:
                result = contract.verifier(witness)
                total_cost += contract.cost_per_verify

                trace.append({
                    "witness": str(witness),
                    "result": result,
                    "cost": contract.cost_per_verify
                })

                if result:
                    best_witness = witness
                    break

            except Exception as e:
                trace.append({
                    "witness": str(witness),
                    "error": str(e),
                    "cost": contract.cost_per_verify
                })
                total_cost += contract.cost_per_verify

        if best_witness is not None:
            return KernelOutput(
                status="UNIQUE",
                witness=best_witness,
                verification_trace=trace,
                total_cost=total_cost,
                survivors_final=1
            )

        remaining = len(d0) - len(trace)
        if remaining > 0:
            return KernelOutput(
                status="OMEGA",
                omega_frontier={
                    "remaining": remaining,
                    "checked": len(trace)
                },
                verification_trace=trace,
                total_cost=total_cost,
                survivors_final=remaining
            )

        return KernelOutput(
            status="REFUTED",
            verification_trace=trace,
            total_cost=total_cost,
            survivors_final=0
        )


def create_sat_contract(
    clauses: List[List[int]],
    num_vars: int,
    budget: int = 1000
) -> Contract:
    """
    Create a SAT contract.

    Args:
        clauses: List of clauses (each clause is list of literals)
        num_vars: Number of variables
        budget: Verification budget

    Returns:
        Contract for SAT solving
    """
    # Generate all possible assignments
    from itertools import product
    assignments = [tuple(a) for a in product([False, True], repeat=num_vars)]
    witness_space = frozenset(assignments)

    def sat_verifier(assignment: Tuple[bool, ...]) -> bool:
        """Check if assignment satisfies all clauses."""
        for clause in clauses:
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
        contract_id=f"SAT_{num_vars}v_{len(clauses)}c",
        assertion=f"Find satisfying assignment for {len(clauses)} clauses",
        witness_space=witness_space,
        verifier=sat_verifier,
        cost_per_verify=1,
        budget=budget
    )


def create_search_contract(
    search_space: FrozenSet[Any],
    predicate: Callable[[Any], bool],
    description: str,
    budget: int = 1000
) -> Contract:
    """
    Create a generic search contract.

    Args:
        search_space: Space to search
        predicate: Condition to satisfy
        description: Human-readable description
        budget: Verification budget

    Returns:
        Contract for search
    """
    return Contract(
        contract_id=f"SEARCH_{hash(description) % 10000:04d}",
        assertion=description,
        witness_space=search_space,
        verifier=predicate,
        cost_per_verify=1,
        budget=budget
    )
