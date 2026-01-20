#!/usr/bin/env python3
# self_improvement_proper_demo.py
#
# Proper kernel self-improvement demo:
#   Ω → compute canonical τ* → execute test → record → Π-closure → repeat → UNIQUE
# No human injection. τ* is an executable distinguisher.
#
# Outputs:
#   - prints step-by-step collapse
#   - writes self_improvement_proper_demo.json with receipts + verifier replay

import json, hashlib
from dataclasses import dataclass
from typing import Any, Dict, List, Tuple, Callable, Optional, Set

# ---------- canonical receipts ----------
def canon_json(obj: Any) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")

def sha256_hex(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()

def add_receipt(payload: Dict[str, Any]) -> Dict[str, Any]:
    p = dict(payload)
    p.pop("receipt_sha256", None)
    payload["receipt_sha256"] = sha256_hex(canon_json(p))
    return payload

# ---------- Toy "world": hidden byte x* ----------
# This is the "physical reality" surrogate. You cannot see it in the loop.
X_TRUE = 173
DOMAIN = list(range(256))  # hypothesis space H

@dataclass(frozen=True)
class Test:
    name: str
    cost: int
    fn: Callable[[int], int]  # returns 0/1 outcome

def make_tests() -> List[Test]:
    tests = []
    # bit tests are canonical distinguishers on byte space
    for bit in range(8):
        tests.append(Test(name=f"bit_{bit}", cost=1, fn=lambda x, b=bit: (x >> b) & 1))
    # redundant (still valid) tests
    tests.append(Test(name="msb", cost=1, fn=lambda x: (x >> 7) & 1))
    tests.append(Test(name="lsb", cost=1, fn=lambda x: x & 1))
    return tests

TESTS = make_tests()
TEST_BY_NAME = {t.name: t for t in TESTS}

@dataclass
class Record:
    test: str
    outcome: int

def fiber(ledger: List[Record]) -> List[int]:
    """W(𝓛): remaining worlds consistent with recorded test outcomes."""
    W = DOMAIN[:]
    for r in ledger:
        t = TEST_BY_NAME[r.test]
        W = [x for x in W if t.fn(x) == r.outcome]
    return W

def split_sizes(W: List[int], test_name: str) -> Tuple[int,int]:
    """How many worlds go to outcome 0 and outcome 1 under test."""
    t = TEST_BY_NAME[test_name]
    ones = sum(1 for x in W if t.fn(x) == 1)
    zeros = len(W) - ones
    return zeros, ones

def tau_star(W: List[int], remaining_budget: int, used_tests: Set[str]) -> Optional[str]:
    """
    Canonical separator τ* (minimax):
      pick a test that splits W and minimizes the worst-case branch size,
      with deterministic tie-break.
    """
    best_key = None
    best_name = None
    for t in TESTS:
        if t.name in used_tests:
            continue
        if t.cost > remaining_budget:
            continue
        z,o = split_sizes(W, t.name)
        if z == 0 or o == 0:
            continue  # doesn't distinguish
        worst = max(z, o)
        key = (worst, t.cost, t.name)  # name only breaks true ties
        if best_key is None or key < best_key:
            best_key = key
            best_name = t.name
    return best_name

def verify_run(run: Dict[str, Any]) -> Dict[str, Any]:
    """
    Total verifier that replays the run:
      - checks each recorded W_size_before matches recomputed fiber size
      - checks chosen_test equals recomputed canonical τ*
      - checks final UNIQUE is correct if decided
    """
    ledger: List[Record] = []
    used = set()
    ok = True
    reason = None

    for i, step in enumerate(run["steps"]):
        W = fiber(ledger)
        if step["W_size_before"] != len(W):
            ok = False
            reason = f"W_size_before mismatch at step {i}"
            break

        if len(W) == 1:
            # should stop with chosen_test None
            if step["chosen_test"] is not None:
                ok = False
                reason = f"did not stop when decided at step {i}"
                break
            continue

        remB = run["budget_B0"] - step["T_before"]
        expected = tau_star(W, remB, used)

        # Handle Ω boundary: chosen_test is None when budget exhausted or no distinguisher
        if step["chosen_test"] is None:
            # This is valid if expected is also None (Ω boundary)
            if expected is not None:
                ok = False
                reason = f"stopped at step {i} but τ* was available: {expected}"
                break
            # Valid Ω boundary - stop verification here
            continue

        if step["chosen_test"] != expected:
            ok = False
            reason = f"chosen_test not canonical τ* at step {i} (got {step['chosen_test']} expected {expected})"
            break

        # apply the recorded outcome
        out = step["outcome"]
        if out not in (0,1):
            ok = False
            reason = f"bad outcome at step {i}"
            break

        ledger.append(Record(step["chosen_test"], out))
        used.add(step["chosen_test"])

    W_final = fiber(ledger)
    decided = (len(W_final) == 1)
    if run["final"]["decided"] != decided:
        ok = False
        reason = "final decided mismatch"
    if decided and run["final"]["x_identified"] != W_final[0]:
        ok = False
        reason = "final x mismatch"

    return {
        "PASS": ok,
        "reason": reason,
        "final_W_size": len(W_final),
        "final_x": (W_final[0] if decided else None),
        "true_x": X_TRUE
    }

def run_self_improvement_demo(B0: int) -> Dict[str, Any]:
    ledger: List[Record] = []
    used = set()
    T = 0
    steps: List[Dict[str, Any]] = []

    while True:
        W = fiber(ledger)

        # UNIQUE reached
        if len(W) == 1:
            steps.append({
                "step": len(steps),
                "T_before": T,
                "W_size_before": len(W),
                "chosen_test": None,
                "outcome": None,
                "note": "UNIQUE reached"
            })
            break

        # Budget boundary (Ω boundary)
        remB = B0 - T
        if remB <= 0:
            steps.append({
                "step": len(steps),
                "T_before": T,
                "W_size_before": len(W),
                "chosen_test": None,
                "outcome": None,
                "note": "BUDGET EXHAUSTED (Ω boundary)"
            })
            break

        # Compute canonical τ*
        choice = tau_star(W, remB, used)
        if choice is None:
            steps.append({
                "step": len(steps),
                "T_before": T,
                "W_size_before": len(W),
                "chosen_test": None,
                "outcome": None,
                "note": "NO DISTINGUISHER AVAILABLE (Ω boundary)"
            })
            break

        # Execute test against hidden reality (physical "world response")
        outcome = TEST_BY_NAME[choice].fn(X_TRUE)
        z,o = split_sizes(W, choice)

        # Record step
        steps.append({
            "step": len(steps),
            "T_before": T,
            "W_size_before": len(W),
            "chosen_test": choice,
            "outcome": outcome,
            "split_sizes": {"0": z, "1": o},
            "cost": TEST_BY_NAME[choice].cost
        })

        # Append ledger record (irreversible in this demo)
        ledger.append(Record(choice, outcome))
        used.add(choice)
        T += TEST_BY_NAME[choice].cost

    W_final = fiber(ledger)

    run = {
        "demo": "Proper self-improvement: Ω → apply executable τ* → new record → Π-closure → repeat",
        "budget_B0": B0,
        "steps": steps,
        "final": {
            "T_spent": T,
            "decided": len(W_final) == 1,
            "x_identified": (W_final[0] if len(W_final)==1 else None),
            "remaining_family_size": len(W_final),
        }
    }
    run["verifier"] = verify_run(run)
    add_receipt(run)
    return run

def main():
    # Two runs: one that reaches UNIQUE, one that hits Ω boundary (budget)
    run_full = run_self_improvement_demo(B0=8)   # enough to identify exact byte
    run_limited = run_self_improvement_demo(B0=3) # stops early with Ω boundary

    suite = {
        "title": "Kernel self-improvement demo: canonical τ* executed each step",
        "world": {"domain_size": 256, "hidden_true_x": {"hidden": True, "revealed": X_TRUE}},
        "runs": {"full_budget": run_full, "budget_limited": run_limited}
    }
    add_receipt(suite)

    # Print a human-readable step log
    print("\n=== FULL BUDGET RUN (should reach UNIQUE) ===")
    for s in run_full["steps"]:
        print(s)
    print("FINAL:", run_full["final"])
    print("VERIFIER:", run_full["verifier"])
    print("RECEIPT:", run_full["receipt_sha256"])

    print("\n=== BUDGET-LIMITED RUN (should stop at Ω boundary) ===")
    for s in run_limited["steps"]:
        print(s)
    print("FINAL:", run_limited["final"])
    print("VERIFIER:", run_limited["verifier"])
    print("RECEIPT:", run_limited["receipt_sha256"])

    # Write full artifact
    out_path = "self_improvement_proper_demo.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(suite, f, indent=2, sort_keys=True)

    print("\nWrote:", out_path)
    print("MASTER RECEIPT:", suite["receipt_sha256"])

if __name__ == "__main__":
    main()
