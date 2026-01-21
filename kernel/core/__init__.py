"""
Structural Reality Kernel - Core Module
The complete ⊥→⊥op universe kernel implementation.
"""

from .kernel import (
    Ledger, Test, Record, Survivors, PiStar,
    Budget, TimeWitness, EnergyLedger, KernelState,
    FeasibleTests, TotalTime, PartitionClass,
    compute_kernel_state
)
from .nsl import NSLState, NSLClosure, Trit
from .controller import PiController, CommutationChecker
from .gauge import Canonicalizer, GaugeChecker
from .receipts import Receipt, CanonicalJSON, PrimeLedger
from .theorem_generator import Contract, TheoremGenerator, KernelOutput
from .universe_engine import UniverseEngine, BotOp
from .verify import ProofBundle, VerificationSuite

__version__ = "1.0.0"
__all__ = [
    "Ledger", "Test", "Record", "Survivors", "PiStar",
    "Budget", "TimeWitness", "EnergyLedger", "KernelState",
    "FeasibleTests", "TotalTime", "PartitionClass",
    "compute_kernel_state",
    "NSLState", "NSLClosure", "Trit",
    "PiController", "CommutationChecker",
    "Canonicalizer", "GaugeChecker",
    "Receipt", "CanonicalJSON", "PrimeLedger",
    "Contract", "TheoremGenerator", "KernelOutput",
    "UniverseEngine", "BotOp",
    "ProofBundle", "VerificationSuite"
]
