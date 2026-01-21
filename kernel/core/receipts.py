"""
core/receipts.py - Canonical receipts and audit trail.

Every artifact hashed as SHA-256 of canonical JSON with sorted keys, no whitespace.
Never hash floats. Store only integers, ratios, and canonical strings.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple, FrozenSet
import hashlib
import json
from datetime import datetime


class CanonicalJSON:
    """
    Canonical JSON serializer.

    Rules:
    - Keys sorted lexicographically
    - No whitespace between separators
    - Numbers as integers only (no floats in receipts)
    - Arrays preserve order
    - Strings minimally escaped
    - None becomes null
    """

    @staticmethod
    def serialize(obj: Any) -> str:
        """
        Serialize object to canonical JSON string.

        Raises ValueError if floats are detected.
        """
        CanonicalJSON._check_no_floats(obj)
        return json.dumps(obj, sort_keys=True, separators=(",", ":"))

    @staticmethod
    def _check_no_floats(obj: Any, path: str = "root") -> None:
        """Recursively check that no floats are present."""
        if isinstance(obj, float):
            raise ValueError(f"Float detected at {path}. Use integer ratios instead.")
        elif isinstance(obj, dict):
            for k, v in obj.items():
                CanonicalJSON._check_no_floats(v, f"{path}.{k}")
        elif isinstance(obj, (list, tuple)):
            for i, v in enumerate(obj):
                CanonicalJSON._check_no_floats(v, f"{path}[{i}]")

    @staticmethod
    def fingerprint(obj: Any) -> str:
        """SHA-256 fingerprint of canonical JSON."""
        canonical = CanonicalJSON.serialize(obj)
        return hashlib.sha256(canonical.encode()).hexdigest()

    @staticmethod
    def fingerprint_short(obj: Any, length: int = 16) -> str:
        """Truncated fingerprint for display."""
        return CanonicalJSON.fingerprint(obj)[:length]


@dataclass
class Receipt:
    """
    An immutable receipt for a kernel operation.

    Contains:
    - Operation type
    - Pre-state fingerprint
    - Post-state fingerprint
    - Integer-only payload
    - Timestamp (integer epoch seconds)
    - Chain link to previous receipt
    """
    receipt_id: str
    operation: str
    pre_state_fp: str
    post_state_fp: str
    payload: Dict[str, Any]
    timestamp_epoch: int
    previous_receipt_fp: Optional[str]

    def __post_init__(self):
        """Validate no floats in payload."""
        CanonicalJSON._check_no_floats(self.payload, "payload")

    def canonical(self) -> str:
        """Canonical representation."""
        return CanonicalJSON.serialize({
            "id": self.receipt_id,
            "op": self.operation,
            "pre": self.pre_state_fp,
            "post": self.post_state_fp,
            "payload": self.payload,
            "ts": self.timestamp_epoch,
            "prev": self.previous_receipt_fp
        })

    def fingerprint(self) -> str:
        """SHA-256 fingerprint of receipt."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def verify_chain(self, previous: Optional['Receipt']) -> bool:
        """Verify chain link to previous receipt."""
        if self.previous_receipt_fp is None:
            return previous is None
        if previous is None:
            return False
        return self.previous_receipt_fp == previous.fingerprint()


class ReceiptChain:
    """
    Append-only chain of receipts.

    Each receipt links to the previous by fingerprint hash.
    """

    def __init__(self):
        self.receipts: List[Receipt] = []
        self._receipt_count = 0

    def append(
        self,
        operation: str,
        pre_state_fp: str,
        post_state_fp: str,
        payload: Dict[str, Any]
    ) -> Receipt:
        """Append a new receipt to the chain."""
        self._receipt_count += 1

        previous_fp = None
        if self.receipts:
            previous_fp = self.receipts[-1].fingerprint()

        receipt = Receipt(
            receipt_id=f"R{self._receipt_count:08d}",
            operation=operation,
            pre_state_fp=pre_state_fp,
            post_state_fp=post_state_fp,
            payload=payload,
            timestamp_epoch=int(datetime.now().timestamp()),
            previous_receipt_fp=previous_fp
        )

        self.receipts.append(receipt)
        return receipt

    def verify_chain_integrity(self) -> Tuple[bool, Optional[int]]:
        """
        Verify entire chain integrity.

        Returns:
            (True, None) if valid
            (False, index) if broken at index
        """
        for i, receipt in enumerate(self.receipts):
            previous = self.receipts[i-1] if i > 0 else None
            if not receipt.verify_chain(previous):
                return (False, i)
        return (True, None)

    def chain_fingerprint(self) -> str:
        """Fingerprint of entire chain."""
        if not self.receipts:
            return hashlib.sha256(b"EMPTY_CHAIN").hexdigest()
        return self.receipts[-1].fingerprint()

    def __len__(self) -> int:
        return len(self.receipts)

    def __iter__(self):
        return iter(self.receipts)


@dataclass
class PrimeLedgerEntry:
    """
    Prime-number based ledger entry for compact verification.

    Each test outcome maps to a unique prime.
    Ledger state = product of primes (big integer).
    """
    test_id: str
    outcome: Any
    prime: int

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "test_id": self.test_id,
            "outcome": str(self.outcome),
            "prime": self.prime
        })


class PrimeLedger:
    """
    Prime-product ledger for compact verification.

    Maps each (test_id, outcome) pair to a unique prime.
    Ledger state = product of all recorded primes.

    This allows:
    - O(1) append (multiply)
    - O(log n) membership check (divisibility)
    - Canonical representation (single big integer)
    """

    # First 100 primes for bootstrapping
    PRIMES = [
        2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47,
        53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113,
        127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197,
        199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271, 277, 281,
        283, 293, 307, 311, 313, 317, 331, 337, 347, 349, 353, 359, 367, 373, 379,
        383, 389, 397, 401, 409, 419, 421, 431, 433, 439, 443, 449, 457, 461, 463,
        467, 479, 487, 491, 499, 503, 509, 521, 523, 541
    ]

    def __init__(self):
        self.product: int = 1  # Empty ledger = 1
        self.prime_map: Dict[str, int] = {}  # (test_id, outcome_str) -> prime
        self._next_prime_idx = 0
        self.entries: List[PrimeLedgerEntry] = []

    def _get_prime(self, test_id: str, outcome: Any) -> int:
        """Get or assign prime for (test_id, outcome) pair."""
        key = f"{test_id}:{outcome}"
        if key not in self.prime_map:
            if self._next_prime_idx >= len(self.PRIMES):
                # Generate more primes if needed
                self._extend_primes()
            self.prime_map[key] = self.PRIMES[self._next_prime_idx]
            self._next_prime_idx += 1
        return self.prime_map[key]

    def _extend_primes(self) -> None:
        """Generate more primes using sieve."""
        # Simple extension: find next prime after last
        candidate = self.PRIMES[-1] + 2
        while True:
            is_prime = True
            for p in self.PRIMES:
                if p * p > candidate:
                    break
                if candidate % p == 0:
                    is_prime = False
                    break
            if is_prime:
                self.PRIMES.append(candidate)
                if len(self.PRIMES) >= self._next_prime_idx + 100:
                    break
            candidate += 2

    def record(self, test_id: str, outcome: Any) -> PrimeLedgerEntry:
        """Record a test outcome."""
        prime = self._get_prime(test_id, outcome)
        self.product *= prime

        entry = PrimeLedgerEntry(
            test_id=test_id,
            outcome=outcome,
            prime=prime
        )
        self.entries.append(entry)
        return entry

    def contains(self, test_id: str, outcome: Any) -> bool:
        """Check if (test_id, outcome) is recorded."""
        key = f"{test_id}:{outcome}"
        if key not in self.prime_map:
            return False
        prime = self.prime_map[key]
        return self.product % prime == 0

    def canonical(self) -> str:
        """Canonical representation."""
        return CanonicalJSON.serialize({
            "product": self.product,
            "entry_count": len(self.entries)
        })

    def fingerprint(self) -> str:
        """SHA-256 fingerprint."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def product_bits(self) -> int:
        """Number of bits in product (measure of ledger size)."""
        if self.product == 0:
            return 0
        return self.product.bit_length()


@dataclass
class AuditBundle:
    """
    Complete audit bundle for a kernel run.

    Contains:
    - Initial state fingerprint
    - Final state fingerprint
    - Receipt chain
    - Verification results
    """
    run_id: str
    initial_state_fp: str
    final_state_fp: str
    receipt_chain: ReceiptChain
    verification_results: Dict[str, Any]

    def canonical(self) -> str:
        """Canonical representation."""
        return CanonicalJSON.serialize({
            "run_id": self.run_id,
            "initial_fp": self.initial_state_fp,
            "final_fp": self.final_state_fp,
            "chain_fp": self.receipt_chain.chain_fingerprint(),
            "receipt_count": len(self.receipt_chain),
            "verification": self.verification_results
        })

    def fingerprint(self) -> str:
        """SHA-256 fingerprint of entire bundle."""
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def is_valid(self) -> bool:
        """Check if bundle is valid."""
        chain_valid, _ = self.receipt_chain.verify_chain_integrity()
        return chain_valid and self.verification_results.get("all_passed", False)


def create_receipt_for_test(
    test_id: str,
    outcome: Any,
    pre_survivors: int,
    post_survivors: int,
    cost: int,
    pre_state_fp: str,
    post_state_fp: str,
    chain: ReceiptChain
) -> Receipt:
    """
    Create a receipt for a test application.

    All values are integers - no floats.
    """
    payload = {
        "test_id": test_id,
        "outcome": str(outcome),
        "w_pre": pre_survivors,
        "w_post": post_survivors,
        "cost": cost,
        "survivors_ratio": [pre_survivors, post_survivors]
    }

    return chain.append(
        operation="APPLY_TEST",
        pre_state_fp=pre_state_fp,
        post_state_fp=post_state_fp,
        payload=payload
    )
