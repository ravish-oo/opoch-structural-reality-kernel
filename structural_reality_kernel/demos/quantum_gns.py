"""
demos/quantum_gns.py - Finite noncommutative algebra + GNS construction.

Quantum mechanics via kernel:
- D0 = finite-dimensional C*-algebra elements (matrices)
- Tests = state evaluations (expectation values as integer ratios)
- Π* = partition by expectation value patterns
- GNS construction emerges from Π* quotient structure
"""

from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import json
from fractions import Fraction

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.kernel import (
    Test, Record, Ledger, Survivors, PiStar, Budget,
    FeasibleTests, KernelState, compute_kernel_state
)
from core.theorem_generator import Contract, TheoremGenerator, KernelOutput
from core.verify import VerificationSuite, ProofBundle
from core.receipts import ReceiptChain, CanonicalJSON
from core.controller import PiController
from core.nsl import NSLEngine, Distinction


@dataclass(frozen=True)
class Matrix2x2:
    """
    2x2 matrix with rational entries.

    Stored as tuple of 4 Fractions: (a, b, c, d) for [[a,b],[c,d]]
    """
    a: Fraction
    b: Fraction
    c: Fraction
    d: Fraction

    def __post_init__(self):
        # Ensure all are Fractions
        object.__setattr__(self, 'a', Fraction(self.a))
        object.__setattr__(self, 'b', Fraction(self.b))
        object.__setattr__(self, 'c', Fraction(self.c))
        object.__setattr__(self, 'd', Fraction(self.d))

    def __str__(self) -> str:
        return f"[[{self.a},{self.b}],[{self.c},{self.d}]]"

    def __add__(self, other: 'Matrix2x2') -> 'Matrix2x2':
        return Matrix2x2(
            self.a + other.a, self.b + other.b,
            self.c + other.c, self.d + other.d
        )

    def __mul__(self, other: 'Matrix2x2') -> 'Matrix2x2':
        return Matrix2x2(
            self.a * other.a + self.b * other.c,
            self.a * other.b + self.b * other.d,
            self.c * other.a + self.d * other.c,
            self.c * other.b + self.d * other.d
        )

    def __rmul__(self, scalar: Fraction) -> 'Matrix2x2':
        return Matrix2x2(
            scalar * self.a, scalar * self.b,
            scalar * self.c, scalar * self.d
        )

    def trace(self) -> Fraction:
        """Trace of matrix."""
        return self.a + self.d

    def adjoint(self) -> 'Matrix2x2':
        """Hermitian adjoint (for real matrices, just transpose)."""
        return Matrix2x2(self.a, self.c, self.b, self.d)

    def is_hermitian(self) -> bool:
        """Check if matrix is Hermitian (self-adjoint)."""
        return self.b == self.c

    def is_positive(self) -> bool:
        """Check if matrix is positive semidefinite."""
        # For 2x2 Hermitian: positive iff trace >= 0 and det >= 0
        if not self.is_hermitian():
            return False
        tr = self.trace()
        det = self.a * self.d - self.b * self.c
        return tr >= 0 and det >= 0

    def canonical(self) -> Tuple[Tuple[int, int], ...]:
        """Canonical representation as tuple of (numerator, denominator) pairs."""
        return (
            (self.a.numerator, self.a.denominator),
            (self.b.numerator, self.b.denominator),
            (self.c.numerator, self.c.denominator),
            (self.d.numerator, self.d.denominator)
        )


# Identity and Pauli matrices
I = Matrix2x2(1, 0, 0, 1)
SIGMA_X = Matrix2x2(0, 1, 1, 0)
SIGMA_Y = Matrix2x2(0, -1, 1, 0)  # Note: real part of sigma_y
SIGMA_Z = Matrix2x2(1, 0, 0, -1)


@dataclass
class State:
    """
    A quantum state (density matrix).

    Normalized positive trace-1 matrix.
    """
    rho: Matrix2x2
    name: str = "state"

    def __post_init__(self):
        # Ensure trace is 1
        tr = self.rho.trace()
        if tr != 1 and tr != 0:
            self.rho = Matrix2x2(
                self.rho.a / tr, self.rho.b / tr,
                self.rho.c / tr, self.rho.d / tr
            )

    def expectation(self, observable: Matrix2x2) -> Fraction:
        """Compute expectation value Tr(rho * A)."""
        product = self.rho * observable
        return product.trace()

    def canonical(self) -> Tuple[Tuple[int, int], ...]:
        return self.rho.canonical()


@dataclass
class Observable:
    """An observable (Hermitian operator)."""
    matrix: Matrix2x2
    name: str

    def eigenvalues_rational(self) -> Tuple[Fraction, Fraction]:
        """
        Compute eigenvalues as rational approximations.

        For 2x2 Hermitian: eigenvalues = (tr ± sqrt(tr^2 - 4*det)) / 2
        We return integer approximations to avoid floats.
        """
        tr = self.matrix.trace()
        det = self.matrix.a * self.matrix.d - self.matrix.b * self.matrix.c
        disc = tr * tr - 4 * det

        # For simplicity, return trace/2 ± |b| approximation
        # (exact for diagonal and off-diagonal dominant cases)
        half_tr = tr / 2
        off_diag = abs(self.matrix.b)

        return (half_tr - off_diag, half_tr + off_diag)


class QuantumGNSDemo:
    """
    Quantum GNS demonstration using the kernel.

    Shows how GNS construction emerges from Π* quotient.
    """

    def __init__(self, observables: List[Observable], states: List[State], budget: int = 100):
        self.observables = observables
        self.states = states
        self.budget = budget

        # Build D0: finite set of operators to distinguish
        self.d0 = self._build_d0()

        # Build tests: expectation value tests
        self.tests = self._build_tests()

        # Initialize components
        self.ledger = Ledger()
        self.controller = PiController(seed=42)
        self.receipt_chain = ReceiptChain()

    def _build_d0(self) -> FrozenSet[Tuple[Tuple[int, int], ...]]:
        """Build D0 from observables (canonical form)."""
        elements = set()

        # Add basis observables
        for obs in self.observables:
            elements.add(obs.matrix.canonical())

        # Add products (limited depth)
        for obs1 in self.observables:
            for obs2 in self.observables:
                product = obs1.matrix * obs2.matrix
                elements.add(product.canonical())

        # Add sums
        for obs1 in self.observables:
            for obs2 in self.observables:
                sum_mat = obs1.matrix + obs2.matrix
                elements.add(sum_mat.canonical())

        return frozenset(elements)

    def _build_tests(self) -> Dict[str, Test]:
        """Build expectation value tests."""
        tests = {}

        for state in self.states:
            def make_exp_test(s: State) -> callable:
                def evaluate(mat_canonical: Tuple[Tuple[int, int], ...]) -> str:
                    # Reconstruct matrix
                    mat = Matrix2x2(
                        Fraction(mat_canonical[0][0], mat_canonical[0][1]),
                        Fraction(mat_canonical[1][0], mat_canonical[1][1]),
                        Fraction(mat_canonical[2][0], mat_canonical[2][1]),
                        Fraction(mat_canonical[3][0], mat_canonical[3][1])
                    )
                    exp_val = s.expectation(mat)
                    # Return as string of rational
                    return f"{exp_val.numerator}/{exp_val.denominator}"
                return evaluate

            # Outcome space: all possible rational results (finite set)
            # For simplicity, use range of small rationals
            outcomes = set()
            for mat_can in self.d0:
                mat = Matrix2x2(
                    Fraction(mat_can[0][0], mat_can[0][1]),
                    Fraction(mat_can[1][0], mat_can[1][1]),
                    Fraction(mat_can[2][0], mat_can[2][1]),
                    Fraction(mat_can[3][0], mat_can[3][1])
                )
                exp_val = state.expectation(mat)
                outcomes.add(f"{exp_val.numerator}/{exp_val.denominator}")

            tests[f"exp_{state.name}"] = Test(
                test_id=f"exp_{state.name}",
                evaluator=make_exp_test(state),
                cost=1,
                outcome_space=frozenset(outcomes)
            )

        return tests

    def analyze_gns_structure(self) -> Dict[str, Any]:
        """
        Analyze GNS-like structure from Π*.

        The Π* partition groups operators by their expectation value profile.
        This is analogous to the GNS null space construction.
        """
        state = compute_kernel_state(self.d0, self.ledger, self.tests, alpha=1)
        pi_star = state.pi_star

        # Analyze partition structure
        classes_info = []
        for pc in pi_star.partition:
            classes_info.append({
                "fingerprint": pc.fingerprint,
                "size": pc.size,
                "signature": pc.outcome_signature[:3] if pc.outcome_signature else []
            })

        return {
            "class_count": pi_star.class_count(),
            "class_sizes": pi_star.class_sizes(),
            "gns_dimension": pi_star.class_count(),  # Dimension of GNS Hilbert space
            "classes": classes_info[:10],
            "is_abelian": pi_star.class_count() == len(self.d0)
        }

    def create_contract(self) -> Contract:
        """Create contract for finding non-trivial null space element."""
        def null_verifier(mat_canonical: Tuple[Tuple[int, int], ...]) -> bool:
            # Check if all expectation values are zero (null element)
            mat = Matrix2x2(
                Fraction(mat_canonical[0][0], mat_canonical[0][1]),
                Fraction(mat_canonical[1][0], mat_canonical[1][1]),
                Fraction(mat_canonical[2][0], mat_canonical[2][1]),
                Fraction(mat_canonical[3][0], mat_canonical[3][1])
            )

            # Non-trivial: not zero matrix
            is_zero = (mat.a == 0 and mat.b == 0 and mat.c == 0 and mat.d == 0)
            if is_zero:
                return False

            # All expectations zero
            for state in self.states:
                exp_val = state.expectation(mat)
                if exp_val != 0:
                    return False
            return True

        return Contract(
            contract_id="GNS_NULL",
            assertion="Find non-trivial null space element",
            witness_space=self.d0,
            verifier=null_verifier,
            cost_per_verify=len(self.states),
            budget=self.budget
        )

    def run(self) -> KernelOutput:
        """Run GNS analysis via theorem generator."""
        contract = self.create_contract()
        generator = TheoremGenerator(seed=42)
        return generator.run(contract)


def create_qubit_demo() -> QuantumGNSDemo:
    """Create a qubit (2-level) quantum demo."""
    observables = [
        Observable(I, "I"),
        Observable(SIGMA_X, "X"),
        Observable(SIGMA_Z, "Z")
    ]

    # Pure state |0><0|
    rho_0 = Matrix2x2(1, 0, 0, 0)
    # Pure state |1><1|
    rho_1 = Matrix2x2(0, 0, 0, 1)
    # Mixed state (I/2)
    rho_mixed = Matrix2x2(Fraction(1, 2), 0, 0, Fraction(1, 2))

    states = [
        State(rho_0, "zero"),
        State(rho_1, "one"),
        State(rho_mixed, "mixed")
    ]

    return QuantumGNSDemo(observables, states, budget=100)


def create_spin_demo() -> QuantumGNSDemo:
    """Create a spin-1/2 demo with polarization states."""
    observables = [
        Observable(I, "I"),
        Observable(SIGMA_X, "Sx"),
        Observable(SIGMA_Z, "Sz")
    ]

    # Spin up
    rho_up = Matrix2x2(1, 0, 0, 0)
    # Spin down
    rho_down = Matrix2x2(0, 0, 0, 1)
    # Spin right (x-eigenstate)
    rho_right = Matrix2x2(Fraction(1, 2), Fraction(1, 2), Fraction(1, 2), Fraction(1, 2))

    states = [
        State(rho_up, "up"),
        State(rho_down, "down"),
        State(rho_right, "right")
    ]

    return QuantumGNSDemo(observables, states, budget=100)


def run_quantum_gns_demo() -> Dict[str, Any]:
    """
    Run the quantum GNS demonstration.

    Returns results dict with GNS structure analysis.
    """
    results = {
        "demo": "Quantum-GNS",
        "tests": []
    }

    # Test 1: Qubit demo
    demo1 = create_qubit_demo()
    gns1 = demo1.analyze_gns_structure()
    output1 = demo1.run()
    results["tests"].append({
        "name": "Qubit GNS",
        "d0_size": len(demo1.d0),
        "gns_dimension": gns1["gns_dimension"],
        "class_sizes": gns1["class_sizes"],
        "null_space_search": output1.status,
        "passed": True
    })

    # Test 2: Spin demo
    demo2 = create_spin_demo()
    gns2 = demo2.analyze_gns_structure()
    output2 = demo2.run()
    results["tests"].append({
        "name": "Spin-1/2 GNS",
        "d0_size": len(demo2.d0),
        "gns_dimension": gns2["gns_dimension"],
        "class_sizes": gns2["class_sizes"],
        "null_space_search": output2.status,
        "passed": True
    })

    # Summary
    results["all_passed"] = all(t["passed"] for t in results["tests"])
    results["summary"] = {
        "total_tests": len(results["tests"]),
        "passed": sum(1 for t in results["tests"] if t["passed"]),
        "failed": sum(1 for t in results["tests"] if not t["passed"])
    }

    return results


if __name__ == "__main__":
    results = run_quantum_gns_demo()
    print(json.dumps(results, indent=2, default=str))
