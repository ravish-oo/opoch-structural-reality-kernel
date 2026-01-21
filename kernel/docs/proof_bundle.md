# Proof Bundle Specification

## Overview

A Proof Bundle is a complete, self-contained verification artifact for any kernel run. It contains all information needed to independently verify that the computation followed kernel axioms.

## Structure

```
ProofBundle
├── bundle_id: str           # Unique identifier
├── checks: List[CheckResult] # All verification results
├── receipt_chain_fp: str    # Fingerprint of receipt chain
├── initial_state_fp: str    # Initial Π* fingerprint
├── final_state_fp: str      # Final Π* fingerprint
├── gauge_witnesses: Dict    # Gauge invariance proofs
└── metadata: Dict           # Additional information
```

## Verification Checks (A-J)

### A. Ledger Hash-Chain Integrity
Each receipt links to the previous by SHA-256 fingerprint:
```
receipt[i].previous_receipt_fp == SHA256(receipt[i-1].canonical())
```

### B. Survivor Consistency
W(L) recomputed from ledger matches recorded value:
```
recomputed_survivors == recorded_survivors
```

### C. Gauge Invariance (Π* Class-Sizes)
Class-size multiset is unchanged under recoding of D0:
```
class_sizes(Π*(L)) == class_sizes(Π*(recoded_L))
```

### D. Time Ratios Are Integers
All (|W_pre|, |W_post|) values are integers, never floats:
```
isinstance(w_pre, int) and isinstance(w_post, int)
```

### E. Budget Monotonicity
Budget never increases after applying a test:
```
Budget(L ∪ {r}) ≤ Budget(L)
```

### F. Feasibility Constraint
Test cost never exceeds budget at time of application:
```
c(τ) ≤ BudgetUnits(L) for all applied τ
```

### G. NSL Closure Idempotent
The closure operator is idempotent:
```
Cl(Cl(s)) == Cl(s)
```

### H. No Floats in Receipts
All receipt payloads contain only integers, strings, lists, and dicts:
```
no_floats(receipt.payload) == True
```

### I. Commutation Law
Controller satisfies N ∘ Q = Q ∘ N:
```
commutation_witness.passed == True
```

### J. Final Π* Fingerprint
Final state fingerprint matches recomputation:
```
final_state.pi_star.canonical_fingerprint() == recorded_fp
```

## Canonical JSON Format

All receipts use canonical JSON with:
- Keys sorted lexicographically
- No whitespace between separators
- Integers only (no floats)
- Deterministic serialization

```python
json.dumps(obj, sort_keys=True, separators=(",", ":"))
```

## Fingerprinting

All fingerprints are SHA-256 hashes of canonical JSON:
```python
hashlib.sha256(canonical_json.encode()).hexdigest()
```

## Gauge Witnesses

The bundle includes witnesses for gauge invariance:

1. **Recoding Invariance**: Random bijection on D0 labels
2. **Order Invariance**: Ledger records in different orders
3. **Renaming Invariance**: Outcome label bijection

## Usage

```python
from structural_reality_kernel.core.verify import verify_kernel_run, print_verification_report

bundle = verify_kernel_run(
    d0=d0,
    tests=tests,
    ledger=ledger,
    receipt_chain=receipt_chain,
    controller=controller,
    nsl_engine=nsl_engine,
    final_state=final_state
)

# Check if all passed
if bundle.all_passed():
    print("Verification PASSED")
else:
    print("Verification FAILED")
    for check in bundle.checks:
        if not check.passed:
            print(f"  Failed: {check.check_name}")

# Print full report
print(print_verification_report(bundle))
```

## Self-Verification

The proof bundle itself can be verified:

```python
# Bundle fingerprint
bundle_fp = bundle.fingerprint()

# Verify bundle integrity
assert bundle.all_passed()
assert len(bundle.checks) == 10  # A through J
```

## Determinism Guarantee

Given the same:
- D0 (candidate set)
- Tests (with deterministic evaluators)
- Initial ledger
- Random seed

The kernel will produce byte-identical outputs including:
- All receipts
- All fingerprints
- Proof bundle

This enables independent verification by third parties.
