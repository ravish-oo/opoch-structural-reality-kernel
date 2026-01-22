"""
Production Proof Bundles and Cryptographic Receipts

Mathematical Foundation:
All artifacts must be:
1. Deterministically serializable (canonical JSON)
2. Cryptographically hashable (SHA-256)
3. Independently verifiable

This ensures:
- Same input -> same receipt
- Tamper-evident audit trail
- Proof-carrying computation

Output Contract Receipts:
- UNIQUE: solution + verifier PASS + receipt chain
- UNSAT: min-cut witness + receipt
- OMEGA_GAP: frontier witness + resource gap + receipt
"""

from dataclasses import dataclass, field
from typing import Dict, List, Any, Optional
from datetime import datetime, timezone
from enum import Enum
import hashlib
import json


class CanonicalJSON:
    """
    Deterministic JSON serialization.

    Rules:
    1. Keys sorted alphabetically
    2. No whitespace
    3. No floats (convert to string or rational)
    4. Consistent encoding (UTF-8)
    """

    @staticmethod
    def serialize(obj: Any) -> str:
        """
        Serialize object to canonical JSON string.

        Args:
            obj: Object to serialize (must be JSON-compatible)

        Returns:
            Canonical JSON string
        """
        return json.dumps(
            obj,
            sort_keys=True,
            separators=(',', ':'),
            ensure_ascii=True,
            default=CanonicalJSON._default_encoder
        )

    @staticmethod
    def _default_encoder(obj: Any) -> Any:
        """Custom encoder for non-standard types."""
        if isinstance(obj, Enum):
            return obj.value
        if isinstance(obj, (set, frozenset)):
            return sorted(list(obj))
        if isinstance(obj, datetime):
            return obj.isoformat()
        if hasattr(obj, 'to_dict'):
            return obj.to_dict()
        if hasattr(obj, '__dict__'):
            return {k: v for k, v in obj.__dict__.items() if not k.startswith('_')}
        raise TypeError(f"Cannot serialize {type(obj)}")

    @staticmethod
    def fingerprint(obj: Any) -> str:
        """
        Compute SHA-256 fingerprint of canonical JSON.

        Args:
            obj: Object to fingerprint

        Returns:
            Hex string of first 16 bytes of SHA-256
        """
        canonical = CanonicalJSON.serialize(obj)
        return hashlib.sha256(canonical.encode('utf-8')).hexdigest()[:32]

    @staticmethod
    def full_hash(obj: Any) -> str:
        """
        Compute full SHA-256 hash of canonical JSON.

        Args:
            obj: Object to hash

        Returns:
            Full hex string of SHA-256
        """
        canonical = CanonicalJSON.serialize(obj)
        return hashlib.sha256(canonical.encode('utf-8')).hexdigest()


class ProductionReceiptType(Enum):
    """Type of receipt."""
    INSTANCE = "INSTANCE"
    FLOW_SOLUTION = "FLOW_SOLUTION"
    PATH_DECOMPOSITION = "PATH_DECOMPOSITION"
    VERIFICATION = "VERIFICATION"
    MIN_CUT_WITNESS = "MIN_CUT_WITNESS"
    OMEGA_GAP = "OMEGA_GAP"
    PROOF_BUNDLE = "PROOF_BUNDLE"


@dataclass
class ProductionReceipt:
    """
    Cryptographic receipt for a computation step.

    Contains:
    - type: What kind of computation
    - payload: The actual data
    - fingerprint: SHA-256 of payload
    - timestamp: When computed (UTC)
    - previous_hash: Link to previous receipt (chain)
    """
    receipt_type: ProductionReceiptType
    payload: Dict[str, Any]
    fingerprint: str = field(init=False)
    timestamp: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    previous_hash: Optional[str] = None

    def __post_init__(self):
        """Compute fingerprint of payload."""
        self.fingerprint = CanonicalJSON.fingerprint(self.payload)

    def to_dict(self) -> Dict:
        return {
            "type": self.receipt_type.value,
            "payload": self.payload,
            "fingerprint": self.fingerprint,
            "timestamp": self.timestamp,
            "previous_hash": self.previous_hash
        }

    def canonical(self) -> str:
        """Canonical JSON representation."""
        return CanonicalJSON.serialize(self.to_dict())

    def hash(self) -> str:
        """Full hash of receipt."""
        return CanonicalJSON.full_hash(self.to_dict())


@dataclass
class ProductionReceiptChain:
    """
    Chain of receipts with hash linking.

    Each receipt links to the previous, creating
    a tamper-evident audit trail.
    """
    receipts: List[ProductionReceipt] = field(default_factory=list)

    def append(self, receipt_type: ProductionReceiptType, payload: Dict[str, Any]) -> ProductionReceipt:
        """
        Append new receipt to chain.

        Args:
            receipt_type: Type of receipt
            payload: Receipt data

        Returns:
            The created receipt
        """
        previous_hash = self.receipts[-1].hash() if self.receipts else None

        receipt = ProductionReceipt(
            receipt_type=receipt_type,
            payload=payload,
            previous_hash=previous_hash
        )
        self.receipts.append(receipt)
        return receipt

    def verify_chain(self) -> Dict:
        """
        Verify chain integrity.

        Checks:
        1. Each receipt's fingerprint matches payload
        2. Each previous_hash matches actual previous receipt hash
        """
        errors = []

        for i, receipt in enumerate(self.receipts):
            # Verify fingerprint
            expected_fp = CanonicalJSON.fingerprint(receipt.payload)
            if receipt.fingerprint != expected_fp:
                errors.append({
                    "index": i,
                    "error": "FINGERPRINT_MISMATCH",
                    "expected": expected_fp,
                    "actual": receipt.fingerprint
                })

            # Verify previous hash
            if i == 0:
                if receipt.previous_hash is not None:
                    errors.append({
                        "index": i,
                        "error": "FIRST_RECEIPT_HAS_PREVIOUS"
                    })
            else:
                expected_prev = self.receipts[i-1].hash()
                if receipt.previous_hash != expected_prev:
                    errors.append({
                        "index": i,
                        "error": "PREVIOUS_HASH_MISMATCH",
                        "expected": expected_prev,
                        "actual": receipt.previous_hash
                    })

        return {
            "valid": len(errors) == 0,
            "num_receipts": len(self.receipts),
            "errors": errors
        }

    def to_dict(self) -> Dict:
        return {
            "receipts": [r.to_dict() for r in self.receipts],
            "chain_hash": self.chain_hash()
        }

    def chain_hash(self) -> str:
        """Hash of entire chain."""
        if not self.receipts:
            return "empty"
        return self.receipts[-1].hash()


class ProductionOutputStatus(Enum):
    """Final output status (the three output types)."""
    UNIQUE = "UNIQUE"        # Solution found with proof
    UNSAT = "UNSAT"          # Proven infeasible
    OMEGA_GAP = "OMEGA_GAP"  # Resource limit reached


@dataclass
class ProductionProofBundle:
    """
    Complete proof bundle for production-scale MAPF solution.

    Contains all artifacts needed for independent verification:
    - Instance specification
    - Solution (if UNIQUE)
    - Verification results
    - Receipt chain
    - Statistics

    Output Contract:
    - UNIQUE: solution + PASS + receipt
    - UNSAT: min-cut witness + receipt
    - OMEGA_GAP: frontier + gap description + receipt
    """
    status: ProductionOutputStatus
    instance_fingerprint: str
    solution_fingerprint: Optional[str]
    verification_passed: bool
    receipt_chain: ProductionReceiptChain
    statistics: Dict[str, Any] = field(default_factory=dict)

    # Optional components based on status
    solution_data: Optional[Dict] = None
    min_cut_witness: Optional[Dict] = None
    omega_gap_data: Optional[Dict] = None

    def __post_init__(self):
        """Validate bundle structure."""
        if self.status == ProductionOutputStatus.UNIQUE:
            assert self.solution_data is not None, "UNIQUE requires solution"
            assert self.verification_passed, "UNIQUE requires verification PASS"
        elif self.status == ProductionOutputStatus.UNSAT:
            assert self.min_cut_witness is not None, "UNSAT requires min-cut witness"
        elif self.status == ProductionOutputStatus.OMEGA_GAP:
            assert self.omega_gap_data is not None, "OMEGA_GAP requires gap data"

    def to_dict(self) -> Dict:
        result = {
            "status": self.status.value,
            "instance_fingerprint": self.instance_fingerprint,
            "solution_fingerprint": self.solution_fingerprint,
            "verification_passed": self.verification_passed,
            "receipt_chain": self.receipt_chain.to_dict(),
            "statistics": self.statistics
        }

        if self.solution_data:
            result["solution"] = self.solution_data
        if self.min_cut_witness:
            result["min_cut_witness"] = self.min_cut_witness
        if self.omega_gap_data:
            result["omega_gap"] = self.omega_gap_data

        return result

    def fingerprint(self) -> str:
        """Canonical fingerprint of bundle."""
        return CanonicalJSON.fingerprint(self.to_dict())

    def save(self, filepath: str):
        """Save bundle to JSON file."""
        with open(filepath, 'w') as f:
            f.write(CanonicalJSON.serialize(self.to_dict()))

    @classmethod
    def load(cls, filepath: str) -> "ProductionProofBundle":
        """Load bundle from JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: Dict) -> "ProductionProofBundle":
        """Reconstruct bundle from dictionary."""
        # Reconstruct receipt chain
        chain = ProductionReceiptChain()
        for r_data in data.get("receipt_chain", {}).get("receipts", []):
            receipt = ProductionReceipt(
                receipt_type=ProductionReceiptType(r_data["type"]),
                payload=r_data["payload"],
                previous_hash=r_data.get("previous_hash")
            )
            # Override computed timestamp with stored
            receipt.timestamp = r_data.get("timestamp", receipt.timestamp)
            chain.receipts.append(receipt)

        return cls(
            status=ProductionOutputStatus(data["status"]),
            instance_fingerprint=data["instance_fingerprint"],
            solution_fingerprint=data.get("solution_fingerprint"),
            verification_passed=data["verification_passed"],
            receipt_chain=chain,
            statistics=data.get("statistics", {}),
            solution_data=data.get("solution"),
            min_cut_witness=data.get("min_cut_witness"),
            omega_gap_data=data.get("omega_gap")
        )


class ProductionProofBundleBuilder:
    """
    Builder for constructing proof bundles step by step.

    Usage:
        builder = ProductionProofBundleBuilder()
        builder.add_instance(instance)
        builder.add_flow_result(flow_result)
        builder.add_paths(decomposition)
        builder.add_verification(verification_result)
        bundle = builder.build()
    """

    def __init__(self):
        self.receipt_chain = ProductionReceiptChain()
        self.instance_data: Optional[Dict] = None
        self.flow_data: Optional[Dict] = None
        self.path_data: Optional[Dict] = None
        self.verification_data: Optional[Dict] = None
        self.min_cut_data: Optional[Dict] = None
        self.omega_gap_data: Optional[Dict] = None
        self.statistics: Dict = {}

    def add_instance(self, instance_data: Dict) -> "ProductionProofBundleBuilder":
        """Add instance specification."""
        self.instance_data = instance_data
        self.receipt_chain.append(ProductionReceiptType.INSTANCE, instance_data)
        return self

    def add_flow_result(self, flow_data: Dict) -> "ProductionProofBundleBuilder":
        """Add flow computation result."""
        self.flow_data = flow_data
        self.receipt_chain.append(ProductionReceiptType.FLOW_SOLUTION, flow_data)
        return self

    def add_paths(self, path_data: Dict) -> "ProductionProofBundleBuilder":
        """Add path decomposition."""
        self.path_data = path_data
        self.receipt_chain.append(ProductionReceiptType.PATH_DECOMPOSITION, path_data)
        return self

    def add_verification(self, verification_data: Dict) -> "ProductionProofBundleBuilder":
        """Add verification result."""
        self.verification_data = verification_data
        self.receipt_chain.append(ProductionReceiptType.VERIFICATION, verification_data)
        return self

    def add_min_cut_witness(self, min_cut_data: Dict) -> "ProductionProofBundleBuilder":
        """Add min-cut witness for UNSAT."""
        self.min_cut_data = min_cut_data
        self.receipt_chain.append(ProductionReceiptType.MIN_CUT_WITNESS, min_cut_data)
        return self

    def add_omega_gap(self, gap_data: Dict) -> "ProductionProofBundleBuilder":
        """Add omega gap data for timeout/budget exceeded."""
        self.omega_gap_data = gap_data
        self.receipt_chain.append(ProductionReceiptType.OMEGA_GAP, gap_data)
        return self

    def set_statistics(self, stats: Dict) -> "ProductionProofBundleBuilder":
        """Set computation statistics."""
        self.statistics = stats
        return self

    def build(self) -> ProductionProofBundle:
        """Build the final proof bundle."""
        # Determine status
        if self.verification_data and self.verification_data.get("passed", False):
            status = ProductionOutputStatus.UNIQUE
            solution_data = self.path_data
            solution_fp = CanonicalJSON.fingerprint(self.path_data) if self.path_data else None
        elif self.min_cut_data:
            status = ProductionOutputStatus.UNSAT
            solution_data = None
            solution_fp = None
        else:
            status = ProductionOutputStatus.OMEGA_GAP
            solution_data = None
            solution_fp = None

        instance_fp = CanonicalJSON.fingerprint(self.instance_data) if self.instance_data else "unknown"

        return ProductionProofBundle(
            status=status,
            instance_fingerprint=instance_fp,
            solution_fingerprint=solution_fp,
            verification_passed=(status == ProductionOutputStatus.UNIQUE),
            receipt_chain=self.receipt_chain,
            statistics=self.statistics,
            solution_data=solution_data,
            min_cut_witness=self.min_cut_data,
            omega_gap_data=self.omega_gap_data
        )


def verify_production_proof(bundle: ProductionProofBundle) -> Dict:
    """
    Independently verify a proof bundle.

    Checks:
    1. Receipt chain integrity
    2. Fingerprints match
    3. Status is consistent with contents
    """
    errors = []

    # Check receipt chain
    chain_result = bundle.receipt_chain.verify_chain()
    if not chain_result["valid"]:
        errors.extend(chain_result["errors"])

    # Check instance fingerprint matches
    for receipt in bundle.receipt_chain.receipts:
        if receipt.receipt_type == ProductionReceiptType.INSTANCE:
            expected_fp = CanonicalJSON.fingerprint(receipt.payload)
            if expected_fp != bundle.instance_fingerprint:
                errors.append({
                    "error": "INSTANCE_FINGERPRINT_MISMATCH",
                    "expected": expected_fp,
                    "actual": bundle.instance_fingerprint
                })

    # Check status consistency
    if bundle.status == ProductionOutputStatus.UNIQUE:
        if not bundle.verification_passed:
            errors.append({"error": "UNIQUE_WITHOUT_VERIFICATION_PASS"})
        if bundle.solution_data is None:
            errors.append({"error": "UNIQUE_WITHOUT_SOLUTION"})

    elif bundle.status == ProductionOutputStatus.UNSAT:
        if bundle.min_cut_witness is None:
            errors.append({"error": "UNSAT_WITHOUT_MIN_CUT"})

    elif bundle.status == ProductionOutputStatus.OMEGA_GAP:
        if bundle.omega_gap_data is None:
            errors.append({"error": "OMEGA_GAP_WITHOUT_GAP_DATA"})

    return {
        "valid": len(errors) == 0,
        "bundle_fingerprint": bundle.fingerprint(),
        "errors": errors
    }
