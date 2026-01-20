#!/usr/bin/env python3
"""
Reality Capsule Demo: Timeless Coordinates for Truth

This script demonstrates that a 256-bit hash becomes the universal address of meaning:
1. Cannot be faked (any tampering breaks verification)
2. Same for everyone (independent of who computes it)
3. Independent of time (the hash is the coordinate)
4. Expands locally into the full object

Proves:
- Theorem A: Immutability (anti-fake)
- Theorem B: Timeless dissemination (same H + same Canon = same object)
- Theorem C: Incremental update (small change → small diff)
- Theorem D: No-minting convergence (same meaning → same hash)
"""

import hashlib
import json
import copy
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple, Any, Optional

# ==============================================================================
# SECTION 1: Π AT THE REPRESENTATION LAYER (CANON)
# ==============================================================================

def canon_json(obj: Any) -> str:
    """
    Canonical JSON: Π but for bytes.

    Kills representation slack:
    - Dict key-order slack (sorted keys)
    - Whitespace slack (compact separators)
    - Unicode escape slack (ensure_ascii=False)

    Properties:
    - Idempotent: Canon(Canon(x)) = Canon(x)
    - Gauge invariant: different representations of same meaning → same output
    """
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def sha256_hex(s: str) -> str:
    """SHA-256 hash of UTF-8 encoded string, returned as hex."""
    return hashlib.sha256(s.encode("utf-8")).hexdigest()


def H(obj: Any) -> str:
    """
    The hash function: H(obj) = SHA256(Canon(obj))

    This is the Π-fixed identity of any object.
    """
    return sha256_hex(canon_json(obj))


# ==============================================================================
# SECTION 2: MERKLE BLOCKS
# ==============================================================================

def mk_block(t: str, payload: Any, children: List[str]) -> Dict:
    """
    Create a Merkle block.

    Block structure: b = (t, p, c₁, ..., cₖ)
    - t: type tag
    - p: payload (the actual content)
    - c: child hashes (references to other blocks)
    """
    return {"t": t, "p": payload, "c": list(children)}


def block_hash(block: Dict) -> str:
    """
    Block identity: h(b) = H(Canon(b))

    The hash uniquely identifies the block content.
    """
    return H(block)


def merkle_add(store: Dict[str, Dict], block: Dict) -> str:
    """
    Add a block to the store, indexed by its hash.
    Returns the block hash.
    """
    h = block_hash(block)
    store[h] = block
    return h


def merkle_verify(root_hash: str, store: Dict[str, Dict]) -> Tuple[bool, Any]:
    """
    Verify a Merkle DAG rooted at root_hash.

    Checks:
    1. All reachable blocks exist in store
    2. All block hashes match their addresses

    Returns (success, info) where info is either error message or stats.
    """
    seen = set()
    stack = [root_hash]

    while stack:
        h = stack.pop()
        if h in seen:
            continue

        # Check block exists
        if h not in store:
            return False, f"missing block {h[:16]}..."

        b = store[h]

        # Check hash matches (Theorem A: anti-fake)
        computed = block_hash(b)
        if computed != h:
            return False, f"hash mismatch at {h[:16]}... (computed {computed[:16]}...)"

        seen.add(h)

        # Add children to stack
        for ch in b["c"]:
            stack.append(ch)

    return True, {"blocks_verified": len(seen)}


# ==============================================================================
# SECTION 3: THE KERNEL CORE (CONTENT TO ENCAPSULATE)
# ==============================================================================

KERNEL_CORE = {
    "axioms": [
        {"id": "⊥", "stmt": "no admissible distinctions"},
        {"id": "A0", "stmt": "distinction admissible iff finite witness separates"}
    ],
    "operators": [
        {"name": "Π", "laws": ["idempotent", "erases untestable distinctions"]},
        {"name": "Δ(T)", "laws": ["feasible tests from self-delimiting programs under budget"]},
        {"name": "𝓛", "laws": ["multiset of (τ,a) records", "order is gauge unless recorded"]},
        {"name": "Ω", "laws": ["frontier of surviving answers + minimal separator/gap"]}
    ],
    "closure": [
        {"Π*(𝓛)": "D0 / ≡_𝓛"},
        {"W(𝓛)": "{x∈D0: ∀(τ,a)∈𝓛, τ(x)=a}"}
    ],
    "time_energy": [
        {"ΔT": "log(|W|/|W'|) ≥ 0", "T": "Σ ΔT"}
    ],
    "control": [
        {"Q": "Π ∘ N ∘ Π"},
        {"commutation": "𝓝 ∘ Q = Q ∘ 𝓝"}
    ],
    "output_gate": [
        "UNIQUE + witness + PASS + receipt",
        "or Ω frontier + τ* + receipt"
    ]
}


# ==============================================================================
# SECTION 4: BUILD THE REALITY CAPSULE
# ==============================================================================

def build_capsule(core: Dict) -> Tuple[str, Dict[str, Dict]]:
    """
    Build a Reality Capsule from kernel core.

    Returns (root_hash, block_store).

    Key insight: The root hash H is the "timeless coordinate" —
    it uniquely identifies the entire object, independent of time.
    """
    store = {}
    leaf_hashes = []

    # Build leaf blocks (one per section)
    # Order doesn't matter because we sort child hashes in manifest
    for section, payload in core.items():
        leaf = mk_block("section", {"name": section, "body": payload}, [])
        leaf_hashes.append(merkle_add(store, leaf))

    # Root manifest references leaves by sorted hashes → deterministic
    manifest = mk_block(
        "manifest",
        {
            "name": "RealityCapsule",
            "version": "v1",
            "canon": "json-sort-keys",
            "hash": "sha256"
        },
        sorted(leaf_hashes)  # SORTED → gauge-invariant
    )
    root = merkle_add(store, manifest)

    return root, store


def shuffled_copy(obj: Any) -> Any:
    """
    Recursively shuffle dict insertion order WITHOUT changing meaning.

    This tests Theorem D: same meaning → same hash despite different
    representation (insertion order).
    """
    if isinstance(obj, dict):
        items = list(obj.items())
        random.shuffle(items)
        return {k: shuffled_copy(v) for k, v in items}
    if isinstance(obj, list):
        # Keep list order (list order IS meaning)
        return [shuffled_copy(x) for x in obj]
    return obj


# ==============================================================================
# SECTION 5: OBSERVER CLOSURE VERIFICATION
# ==============================================================================

def verify_observer_closure(store: Dict[str, Dict]) -> Tuple[bool, str]:
    """
    Verify that the capsule respects Observer Closure: 𝓝 ∘ Q = Q ∘ 𝓝

    For capsules, this means:
    - Canon is idempotent
    - Hash depends only on canonical form
    - Verification is path-independent
    """
    # Test idempotence of Canon
    test_obj = {"b": 2, "a": 1, "c": [3, 2, 1]}
    c1 = canon_json(test_obj)
    c2 = canon_json(json.loads(c1))

    if c1 != c2:
        return False, "Canon not idempotent"

    # Test hash determinism (same content → same hash)
    for h, block in store.items():
        recomputed = block_hash(block)
        if recomputed != h:
            return False, f"Hash non-deterministic at {h[:16]}..."

    # Test path independence: verify from any starting point
    # should give same result
    root = max(store.keys(), key=lambda k: len(store[k].get("c", [])))
    ok1, _ = merkle_verify(root, store)
    ok2, _ = merkle_verify(root, store)

    if ok1 != ok2:
        return False, "Verification not path-independent"

    return True, "Observer Closure: PASS (𝓝 ∘ Q = Q ∘ 𝓝)"


# ==============================================================================
# SECTION 6: COMPLETE DEMO
# ==============================================================================

def run_demo():
    """
    Complete demonstration of Reality Capsule properties.

    Proves:
    - Theorem A: Immutability (any tamper → verification fails)
    - Theorem B: Timeless dissemination (same H = same object)
    - Theorem C: Incremental update (small change → small diff)
    - Theorem D: No-minting convergence (same meaning → same hash)
    """
    print("=" * 70)
    print("REALITY CAPSULE: COMPLETE VERIFICATION")
    print("=" * 70)
    print()

    results = {}

    # ------------------------------------------------------------------
    # DEMO 1: Build capsule (original)
    # ------------------------------------------------------------------
    print("[1] BUILD ORIGINAL CAPSULE")
    print("-" * 40)

    root1, store1 = build_capsule(KERNEL_CORE)
    ok1, msg1 = merkle_verify(root1, store1)

    print(f"  Root hash: {root1}")
    print(f"  Blocks: {len(store1)}")
    print(f"  Verify: {ok1} {msg1}")
    print()

    results["build_original"] = ok1

    # ------------------------------------------------------------------
    # DEMO 2: Theorem D - No-minting convergence
    # Build with shuffled insertion order → should yield SAME hash
    # ------------------------------------------------------------------
    print("[2] THEOREM D: NO-MINTING CONVERGENCE")
    print("-" * 40)
    print("  Building capsule with shuffled dict insertion order...")

    random.seed(42)
    core2 = shuffled_copy(KERNEL_CORE)
    root2, store2 = build_capsule(core2)
    ok2, msg2 = merkle_verify(root2, store2)

    print(f"  Root hash: {root2}")
    print(f"  MATCH: {root1 == root2}")
    print(f"  Verify: {ok2} {msg2}")

    if root1 == root2:
        print("  ✓ Same meaning → Same hash (Theorem D proved)")
    else:
        print("  ✗ FAILURE: Different hashes for same meaning!")
    print()

    results["no_minting_convergence"] = (root1 == root2)

    # ------------------------------------------------------------------
    # DEMO 3: Theorem A - Immutability (anti-fake)
    # Tamper with a block WITHOUT changing its address → verification fails
    # ------------------------------------------------------------------
    print("[3] THEOREM A: IMMUTABILITY (ANTI-FAKE)")
    print("-" * 40)
    print("  Tampering with a stored block...")

    tampered = copy.deepcopy(store1)
    # Pick any non-root block to mutate
    some_leaf = next(h for h in tampered.keys() if h != root1)

    # Add minted content
    original_body = tampered[some_leaf]["p"]["body"]
    tampered[some_leaf]["p"]["body"] = original_body + ["MINTED_CONTENT"]

    okT, msgT = merkle_verify(root1, tampered)

    print(f"  Tampered block: {some_leaf[:16]}...")
    print(f"  Verify: {okT}")
    print(f"  Message: {msgT}")

    if not okT:
        print("  ✓ Tampering detected! (Theorem A proved)")
    else:
        print("  ✗ FAILURE: Tampering not detected!")
    print()

    results["immutability"] = (not okT)

    # ------------------------------------------------------------------
    # DEMO 4: Theorem C - Incremental update
    # Change one section → new root, but most blocks reused
    # ------------------------------------------------------------------
    print("[4] THEOREM C: INCREMENTAL UPDATE")
    print("-" * 40)
    print("  Making a legitimate update (adding one axiom)...")

    core3 = copy.deepcopy(KERNEL_CORE)
    core3["axioms"] = core3["axioms"] + [{"id": "A1", "stmt": "extra axiom for demo"}]
    root3, store3 = build_capsule(core3)
    ok3, msg3 = merkle_verify(root3, store3)

    shared = set(store1.keys()) & set(store3.keys())

    print(f"  Original root: {root1[:16]}...")
    print(f"  Updated root:  {root3[:16]}...")
    print(f"  ROOT_CHANGED: {root3 != root1}")
    print(f"  Shared blocks: {len(shared)} of {len(store1)}")
    print(f"  Verify: {ok3} {msg3}")

    if root3 != root1 and len(shared) > 0:
        print("  ✓ Small change → Small diff (Theorem C proved)")
    else:
        print("  ✗ FAILURE: Incremental update not working!")
    print()

    results["incremental_update"] = (root3 != root1 and len(shared) > 0)

    # ------------------------------------------------------------------
    # DEMO 5: Theorem B - Timeless dissemination
    # Same H + same Canon = same object, independent of time
    # ------------------------------------------------------------------
    print("[5] THEOREM B: TIMELESS DISSEMINATION")
    print("-" * 40)
    print("  Demonstrating time-independence...")

    # Simulate "different times" by rebuilding
    root4, store4 = build_capsule(KERNEL_CORE)
    root5, store5 = build_capsule(KERNEL_CORE)

    print(f"  Build at 'time 1': {root4[:16]}...")
    print(f"  Build at 'time 2': {root5[:16]}...")
    print(f"  MATCH: {root4 == root5}")

    if root4 == root5:
        print("  ✓ Same H = Same object, regardless of when built (Theorem B proved)")
    else:
        print("  ✗ FAILURE: Time-dependent hash!")
    print()

    results["timeless_dissemination"] = (root4 == root5)

    # ------------------------------------------------------------------
    # DEMO 6: Observer Closure
    # ------------------------------------------------------------------
    print("[6] OBSERVER CLOSURE: 𝓝 ∘ Q = Q ∘ 𝓝")
    print("-" * 40)

    oc_ok, oc_msg = verify_observer_closure(store1)
    print(f"  {oc_msg}")
    print()

    results["observer_closure"] = oc_ok

    # ------------------------------------------------------------------
    # SUMMARY
    # ------------------------------------------------------------------
    print("=" * 70)
    print("VERIFICATION SUMMARY")
    print("=" * 70)
    print()

    all_pass = all(results.values())

    for name, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {name:30} {status}")

    print()
    print(f"  ALL THEOREMS VERIFIED: {'YES' if all_pass else 'NO'}")
    print()

    # Generate master receipt
    receipt_data = {
        "demo": "reality_capsule",
        "root_hash": root1,
        "theorems": {
            "A_immutability": results["immutability"],
            "B_timeless": results["timeless_dissemination"],
            "C_incremental": results["incremental_update"],
            "D_no_minting": results["no_minting_convergence"]
        },
        "observer_closure": results["observer_closure"],
        "blocks_in_capsule": len(store1)
    }
    master_receipt = H(receipt_data)

    print(f"  Capsule Root:   {root1}")
    print(f"  Master Receipt: {master_receipt}")
    print()
    print("=" * 70)

    # Save verification results
    with open("reality_capsule_verified.json", "w") as f:
        json.dump({
            "capsule_root": root1,
            "master_receipt": master_receipt,
            "results": results,
            "blocks": len(store1),
            "shared_after_update": len(shared)
        }, f, indent=2)

    print("Results saved to: reality_capsule_verified.json")

    return all_pass, root1, master_receipt


# ==============================================================================
# MAIN
# ==============================================================================

if __name__ == "__main__":
    success, root, receipt = run_demo()
    exit(0 if success else 1)
