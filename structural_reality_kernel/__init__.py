"""
Structural Reality Kernel
=========================

The complete ⊥ → ⊥op universe kernel implementation.

A self-contained system that:
- Starts from nothing (⊥)
- Evolves through witnessable tests
- Reaches operational nothingness (⊥op)
- Generates theorems and proofs

All outputs are deterministic, gauge-invariant, and verifiable.
No floats in receipts - integers and ratios only.
"""

from .core import (
    # Kernel primitives
    Ledger, Test, Record, Survivors, PiStar,
    Budget, TimeWitness, EnergyLedger,

    # NSL
    NSLState, NSLClosure, Trit,

    # Control
    PiController, CommutationChecker,

    # Gauge
    Canonicalizer, GaugeChecker,

    # Receipts
    Receipt, CanonicalJSON, PrimeLedger,

    # Theorem generation
    Contract, TheoremGenerator, KernelOutput,

    # Universe engine
    UniverseEngine, BotOp,

    # Verification
    ProofBundle, VerificationSuite
)

__version__ = "1.0.0"
__author__ = "Structural Reality Project"

__all__ = [
    # Kernel primitives
    "Ledger", "Test", "Record", "Survivors", "PiStar",
    "Budget", "TimeWitness", "EnergyLedger",

    # NSL
    "NSLState", "NSLClosure", "Trit",

    # Control
    "PiController", "CommutationChecker",

    # Gauge
    "Canonicalizer", "GaugeChecker",

    # Receipts
    "Receipt", "CanonicalJSON", "PrimeLedger",

    # Theorem generation
    "Contract", "TheoremGenerator", "KernelOutput",

    # Universe engine
    "UniverseEngine", "BotOp",

    # Verification
    "ProofBundle", "VerificationSuite"
]
