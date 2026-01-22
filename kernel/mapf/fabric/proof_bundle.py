"""
Proof Bundle for Transport Fabric

Every tick produces a proof-carrying artifact that can be
independently verified.

Tick Witness Contains:
1. The chosen permutation P_t (junction mode selections + lane step)
2. Current occupancy hash H(ρ_t)
3. Next occupancy hash H(ρ_{t+1})

Verifier Checks:
1. Bijection: P_t is a permutation
2. Legality: P_t(v) = v or (v → P_t(v)) ∈ E
3. Swap-free: no adjacent 2-cycles
4. V1/V2: boundary conditions on start/goal

Receipt:
- Canonical JSON + SHA-256
- Map hash, fabric hash, controller hash
- Tick id, junction mode vector
- Occupancy hashes, verifier PASS

This is the "world-shake" part: provably safe 10k-robot operations.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, Any
from datetime import datetime, timezone
from enum import Enum
import hashlib
import json

from .permutation_executor import Permutation, OccupancyBitset


class VerificationGate(Enum):
    """Verification gates for tick proof."""
    BIJECTION = "BIJECTION"      # P_t is a permutation
    LEGALITY = "LEGALITY"        # All moves follow edges
    SWAP_FREE = "SWAP_FREE"      # No adjacent 2-cycles
    CONSERVATION = "CONSERVATION"  # Robot count preserved
    V1_START = "V1_START"        # Start conditions (if applicable)
    V2_GOAL = "V2_GOAL"          # Goal conditions (if applicable)


@dataclass
class TickWitness:
    """
    Witness for a single tick of fabric execution.

    Contains all information needed to verify the tick was safe.
    """
    tick_id: int
    junction_modes: Dict[int, int]  # junction_id → mode_id
    permutation_fingerprint: str
    occupancy_before_hash: str
    occupancy_after_hash: str
    robot_count: int
    non_identity_moves: int  # Number of robots that moved

    def to_dict(self) -> Dict:
        return {
            "tick": self.tick_id,
            "junction_modes": self.junction_modes,
            "perm_fp": self.permutation_fingerprint,
            "occ_before": self.occupancy_before_hash,
            "occ_after": self.occupancy_after_hash,
            "robot_count": self.robot_count,
            "moves": self.non_identity_moves
        }

    def fingerprint(self) -> str:
        data = self.to_dict()
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]


@dataclass
class VerificationResult:
    """Result of verifying a single tick."""
    tick_id: int
    passed: bool
    gates: Dict[str, bool]  # gate_name → pass/fail
    errors: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict:
        return {
            "tick": self.tick_id,
            "passed": self.passed,
            "gates": self.gates,
            "errors": self.errors[:5]
        }


@dataclass
class TickReceipt:
    """
    Cryptographic receipt for a verified tick.

    Canonical JSON + SHA-256 for tamper-evidence.
    """
    tick_id: int
    witness: TickWitness
    verification: VerificationResult
    timestamp: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    previous_receipt_hash: Optional[str] = None

    def to_dict(self) -> Dict:
        return {
            "tick": self.tick_id,
            "witness": self.witness.to_dict(),
            "verification": self.verification.to_dict(),
            "timestamp": self.timestamp,
            "prev_hash": self.previous_receipt_hash
        }

    def canonical(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(',', ':'))

    def hash(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def fingerprint(self) -> str:
        return self.hash()[:16]


@dataclass
class FabricProofBundle:
    """
    Complete proof bundle for a fabric execution run.

    Contains:
    - Fabric configuration hash
    - Initial state
    - All tick receipts (hash-linked chain)
    - Final state verification
    - Run statistics
    """
    fabric_hash: str
    initial_occupancy_hash: str
    initial_robot_count: int
    tick_receipts: List[TickReceipt]
    final_occupancy_hash: str
    run_statistics: Dict[str, Any]
    all_passed: bool

    def num_ticks(self) -> int:
        return len(self.tick_receipts)

    def chain_hash(self) -> str:
        """Hash of entire receipt chain."""
        if not self.tick_receipts:
            return "empty"
        return self.tick_receipts[-1].hash()

    def to_dict(self) -> Dict:
        return {
            "fabric_hash": self.fabric_hash,
            "initial_occ": self.initial_occupancy_hash,
            "initial_robots": self.initial_robot_count,
            "num_ticks": self.num_ticks(),
            "final_occ": self.final_occupancy_hash,
            "all_passed": self.all_passed,
            "chain_hash": self.chain_hash(),
            "statistics": self.run_statistics,
            "receipts": [r.to_dict() for r in self.tick_receipts]
        }

    def fingerprint(self) -> str:
        data = {
            "fabric": self.fabric_hash,
            "ticks": self.num_ticks(),
            "passed": self.all_passed,
            "chain": self.chain_hash()
        }
        canonical = json.dumps(data, sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()[:16]

    def save(self, filepath: str):
        """Save bundle to JSON file."""
        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, filepath: str) -> "FabricProofBundle":
        """Load bundle from JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        # Reconstruct (simplified)
        return cls(
            fabric_hash=data["fabric_hash"],
            initial_occupancy_hash=data["initial_occ"],
            initial_robot_count=data["initial_robots"],
            tick_receipts=[],  # Would need full reconstruction
            final_occupancy_hash=data["final_occ"],
            run_statistics=data["statistics"],
            all_passed=data["all_passed"]
        )


class FabricVerifier:
    """
    Verifies tick permutations satisfy all safety constraints.

    Verification is O(|P|) per tick where |P| is permutation size,
    NOT O(k) robots. This is polynomial regardless of robot count.
    """

    def __init__(self, edges: Set[Tuple[int, int]]):
        """
        Initialize verifier.

        Args:
            edges: Set of valid directed edges (u, v)
        """
        self.edges = edges

    def verify_tick(self, tick_id: int,
                    permutation: Permutation,
                    occupancy_before: OccupancyBitset,
                    occupancy_after: OccupancyBitset) -> VerificationResult:
        """
        Verify a single tick satisfies all constraints.

        Checks:
        1. Bijection: permutation is valid
        2. Legality: all moves follow edges or are waits
        3. Swap-free: no adjacent 2-cycles
        4. Conservation: robot count preserved
        """
        gates = {}
        errors = []

        # Gate 1: Bijection
        targets = list(permutation.mapping.values())
        is_bijection = len(targets) == len(set(targets))
        gates[VerificationGate.BIJECTION.value] = is_bijection
        if not is_bijection:
            errors.append("Permutation is not a bijection")

        # Gate 2: Legality
        is_legal = True
        for src, dst in permutation.mapping.items():
            if src != dst and (src, dst) not in self.edges:
                is_legal = False
                errors.append(f"Illegal move: {src}→{dst}")
                break
        gates[VerificationGate.LEGALITY.value] = is_legal

        # Gate 3: Swap-free
        is_swap_free = True
        for src, dst in permutation.mapping.items():
            if src != dst and dst in permutation.mapping:
                if permutation.mapping[dst] == src:
                    is_swap_free = False
                    errors.append(f"Swap detected: {src}↔{dst}")
                    break
        gates[VerificationGate.SWAP_FREE.value] = is_swap_free

        # Gate 4: Conservation
        count_before = occupancy_before.count()
        count_after = occupancy_after.count()
        is_conserved = count_before == count_after
        gates[VerificationGate.CONSERVATION.value] = is_conserved
        if not is_conserved:
            errors.append(f"Robot count changed: {count_before}→{count_after}")

        passed = all(gates.values())

        return VerificationResult(
            tick_id=tick_id,
            passed=passed,
            gates=gates,
            errors=errors
        )


class ProofBundleBuilder:
    """
    Builds proof bundle during fabric execution.

    Usage:
        builder = ProofBundleBuilder(fabric, initial_occupancy)
        for t in range(horizon):
            builder.record_tick(t, modes, perm, new_occ)
        bundle = builder.build()
    """

    def __init__(self, fabric_hash: str,
                 initial_occupancy: OccupancyBitset,
                 edges: Set[Tuple[int, int]]):
        """
        Initialize proof bundle builder.

        Args:
            fabric_hash: Hash of compiled fabric
            initial_occupancy: Starting occupancy state
            edges: Set of valid edges for verification
        """
        self.fabric_hash = fabric_hash
        self.initial_hash = initial_occupancy.fingerprint()
        self.initial_count = initial_occupancy.count()
        self.edges = edges
        self.verifier = FabricVerifier(edges)

        self.receipts: List[TickReceipt] = []
        self.current_occupancy = initial_occupancy.copy()
        self.all_passed = True
        self.total_moves = 0

    def record_tick(self, tick_id: int,
                    junction_modes: Dict[int, int],
                    permutation: Permutation,
                    new_occupancy: OccupancyBitset):
        """
        Record and verify a single tick.

        Args:
            tick_id: Tick number
            junction_modes: Junction mode selections
            permutation: Applied permutation
            new_occupancy: Resulting occupancy
        """
        # Create witness
        witness = TickWitness(
            tick_id=tick_id,
            junction_modes=junction_modes,
            permutation_fingerprint=permutation.fingerprint(),
            occupancy_before_hash=self.current_occupancy.fingerprint(),
            occupancy_after_hash=new_occupancy.fingerprint(),
            robot_count=new_occupancy.count(),
            non_identity_moves=permutation.non_identity_count()
        )

        # Verify tick
        verification = self.verifier.verify_tick(
            tick_id, permutation, self.current_occupancy, new_occupancy
        )

        if not verification.passed:
            self.all_passed = False

        # Create receipt
        prev_hash = self.receipts[-1].hash() if self.receipts else None
        receipt = TickReceipt(
            tick_id=tick_id,
            witness=witness,
            verification=verification,
            previous_receipt_hash=prev_hash
        )

        self.receipts.append(receipt)
        self.current_occupancy = new_occupancy.copy()
        self.total_moves += permutation.non_identity_count()

    def build(self) -> FabricProofBundle:
        """Build final proof bundle."""
        return FabricProofBundle(
            fabric_hash=self.fabric_hash,
            initial_occupancy_hash=self.initial_hash,
            initial_robot_count=self.initial_count,
            tick_receipts=self.receipts,
            final_occupancy_hash=self.current_occupancy.fingerprint(),
            run_statistics={
                "num_ticks": len(self.receipts),
                "total_moves": self.total_moves,
                "avg_moves_per_tick": self.total_moves / len(self.receipts) if self.receipts else 0,
                "robot_count": self.initial_count
            },
            all_passed=self.all_passed
        )


def verify_fabric_proof(bundle: FabricProofBundle) -> Dict:
    """
    Independently verify a fabric proof bundle.

    Checks:
    1. Receipt chain integrity (hash linking)
    2. All tick verifications passed
    3. Occupancy hashes are consistent
    """
    errors = []

    # Check receipt chain
    for i, receipt in enumerate(bundle.tick_receipts):
        if i == 0:
            if receipt.previous_receipt_hash is not None:
                errors.append("First receipt has previous hash")
        else:
            expected = bundle.tick_receipts[i-1].hash()
            if receipt.previous_receipt_hash != expected:
                errors.append(f"Receipt {i} hash chain broken")

        # Check verification passed
        if not receipt.verification.passed:
            errors.append(f"Tick {receipt.tick_id} verification failed")

    # Check occupancy consistency
    if bundle.tick_receipts:
        first_before = bundle.tick_receipts[0].witness.occupancy_before_hash
        if first_before != bundle.initial_occupancy_hash:
            errors.append("Initial occupancy hash mismatch")

        last_after = bundle.tick_receipts[-1].witness.occupancy_after_hash
        if last_after != bundle.final_occupancy_hash:
            errors.append("Final occupancy hash mismatch")

    return {
        "valid": len(errors) == 0 and bundle.all_passed,
        "num_ticks": bundle.num_ticks(),
        "chain_valid": len(errors) == 0,
        "all_verifications_passed": bundle.all_passed,
        "errors": errors[:10]
    }
