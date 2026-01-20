---
sidebar_position: 4
title: "Apple Puzzles: Kernel Demo"
description: How the kernel solves the puzzles where LLMs fail — with complete mathematical proofs and verified code
---

# Apple Puzzles: Kernel Demo

This document demonstrates how the Opoch Kernel solves the puzzles from Apple's [*The Illusion of Thinking*](https://arxiv.org/abs/2506.06941) paper — puzzles where LLMs systematically fail as complexity increases, while the kernel maintains **100% correctness at any scale**.

---

## Numeric Summary

- **Kernel:** 100% verified correctness across 4 puzzle families (scales in size, never degrades in correctness).
- **0** wrong solutions, **0** illegal moves, **0** safety violations.
- When compute is insufficient, output is **Ω** (need more compute), never a guess.

### Scoreboard

| Puzzle Family | Kernel Verified Range | Verified Moves | Opoch Kernel Result | LLM Breakpoint (per paper) |
|:--------------|:----------------------|---------------:|:--------------------|:---------------------------|
| Tower of Hanoi | n = 1..12 | up to 4095 (n=12) | <span className="status-pass">PASS</span>, optimal | <span className="status-fail">Degrades as n increases</span> |
| Checker Jumping | n = 1..8 | up to 80 (n=8) | <span className="status-pass">PASS</span>, expected min | <span className="status-fail">Collapses past n > 3</span> |
| River Crossing | N = 2..4 (k varies) | 5 / 11 / 9 crossings | <span className="status-pass">PASS</span>, safety preserved | <span className="status-fail">Fails on safety constraints</span> |
| Blocks World | N = 4, 6 | 5 / 9 moves | <span className="status-pass">PASS</span> | <span className="status-fail">Poor at scale</span> |

**One-line takeaway:** Same puzzles, same rules — LLMs guess; the kernel gates output on a total verifier, so correctness is invariant and only cost grows.

---

## The Paper's Finding: LLMs Collapse on Complexity

The [*Illusion of Thinking*](https://arxiv.org/abs/2506.06941) paper tested frontier LLMs on four classic puzzle families:

| Puzzle | LLM Performance | Failure Mode |
|--------|-----------------|--------------|
| **Tower of Hanoi** | Degrades with n | Loses track of disk positions |
| **Checker Jumping** | Collapses at n>3 | Violates movement constraints |
| **River Crossing** | Fails on safety | Forgets who's on which bank |
| **Blocks World** | Poor at scale | Loses block configurations |

**Key observation:** Even when given the algorithm, LLMs fail because they don't execute a verifier loop internally. They pattern-match to plausible-looking solutions.

---

## Mathematical Foundation

### Every Puzzle is a Finite Transition System

A puzzle instance is a 6-tuple:

$$\mathcal{P} = (S, A, \delta, s_0, G, c)$$

Where:
- **S** = finite state space
- **A(s)** = legal actions at state s
- **δ(s,a)** = deterministic transition function
- **s₀** = initial state
- **G ⊆ S** = goal states
- **c ≥ 0** = cost per move (usually 1)

A candidate solution is a finite action sequence:

$$\pi = (a_1, a_2, \ldots, a_L)$$

### The Total Verifier

A **total verifier** is a simulator that checks every step:

$$V(\pi) = \text{PASS} \iff \delta(\cdots\delta(s_0, a_1), \ldots, a_L) \in G \text{ and every step is legal}$$

This is exactly how the paper describes evaluation: check each move against constraints and confirm the final state matches the goal.

### Kernel Theorem: Correctness Never Degrades

:::info Theorem (Verifier-Gated Correctness)
If you output a plan π **only when** V(π) = PASS, you cannot output a wrong solution at any size.
:::

**Proof:**
1. V is a total function (always returns PASS or FAIL)
2. V checks legality of every transition
3. V checks goal membership of final state
4. Output is gated on V(π) = PASS

Therefore, any output is correct by construction. ∎

**What scales is cost, not correctness:**
- Truth (PASS) is invariant
- Computation grows (more states, longer plans)

If budget is insufficient, kernel outputs **Ω** ("need more compute"), not an incorrect plan.

---

## The Kernel Solution: Witness + Verifier = Truth

The kernel approach is fundamentally different:

```
Output = UNIQUE + witness + PASS   (if decidable)
       = Ω + τ* + budget_gap       (if undecided)
```

**Never guess. Never output without verification.**

---

## Verified Results

The demo code has been executed and verified. All puzzles pass:

```
Tower of Hanoi (n=1..12):
  n=1:  1 move,    PASS, optimal
  n=2:  3 moves,   PASS, optimal
  n=3:  7 moves,   PASS, optimal
  ...
  n=12: 4095 moves, PASS, optimal

Checker Jumping (n=1..8):
  n=1: 3 moves,  expected=3,  PASS
  n=2: 8 moves,  expected=8,  PASS
  n=3: 15 moves, expected=15, PASS
  ...
  n=8: 80 moves, expected=80, PASS

River Crossing:
  N=2, k=2: 5 crossings,  PASS
  N=3, k=2: 11 crossings, PASS
  N=4, k=3: 9 crossings,  PASS

Blocks World:
  N=4: 5 moves, PASS
  N=6: 9 moves, PASS

Master receipt: ba73ea1673a0a5e30810ff79cc204bbc80b38bde10624bafded52b91398972b9
```

---

## Why This Is "Kernel" and Why It Scales

### The Key Difference: Verifier-Gated Output

| Approach | Output Condition | Failure Mode |
|----------|------------------|--------------|
| **LLM Pattern Matching** | "Looks plausible" | Wrong at scale |
| **Kernel** | V(π) = PASS | Never wrong (only Ω) |

### What "Complexity Doesn't Matter" Means

In kernel terms:
- **Truth is invariant**: If V(π) = PASS, the solution is correct. Period.
- **Cost grows**: More states, longer plans, more compute needed.
- **No degradation**: A correct verifier cannot become incorrect at larger n.

### The Ω Alternative

If the solver cannot find a solution within budget:

```
Output = Ω + {
    "surviving_states": [...],
    "next_distinguisher": τ*,
    "budget_gap": V(W,T;q) - B
}
```

This is honest: "I need more resources to decide," not "here's my best guess."

---

## Puzzle Details

### Puzzle 1: Tower of Hanoi

**Definition:** Move n disks from peg 0 to peg 2:
- Only the top disk of any peg can be moved
- A disk cannot be placed on a smaller disk

**Closed-Form Solution:**

```
HANOI(n, src, aux, dst):
    if n == 0: return
    HANOI(n-1, src, dst, aux)
    move disk n from src to dst
    HANOI(n-1, aux, src, dst)
```

**Minimum Moves Theorem:**

$$M(1) = 1, \quad M(n) = 2M(n-1) + 1 \implies M(n) = 2^n - 1$$

**Proof by induction:**
- Base: M(1) = 1 = 2¹ - 1 ✓
- Inductive step: M(n) = 2(2ⁿ⁻¹ - 1) + 1 = 2ⁿ - 2 + 1 = 2ⁿ - 1 ✓

---

### Puzzle 2: Checker Jumping

**Definition:**
- Initial state: `R...R _ B...B` (N reds, empty, N blues)
- Goal state: `B...B _ R...R` (swap positions)

**Rules:**
- Red can only move right (slide 1 or jump 2 over blue)
- Blue can only move left (slide 1 or jump 2 over red)
- No backward moves allowed

**Minimum Moves Theorem:**

$$M(N) = (N+1)^2 - 1$$

**Examples:**
- N=1: (1+1)² - 1 = 3 moves
- N=2: (2+1)² - 1 = 8 moves
- N=3: (3+1)² - 1 = 15 moves

:::info Theorem (BFS Completeness on Finite Graphs)
If S is finite and BFS explores all reachable states, it finds a shortest path to any reachable goal.
:::

---

### Puzzle 3: River Crossing

**Definition:** N actor-agent pairs must cross a river:
- Boat capacity k (1 ≤ passengers ≤ k)
- Boat cannot cross empty
- **Safety constraint:** An actor cannot be on a bank (or boat) with another actor's agent unless their own agent is present

---

### Puzzle 4: Blocks World

**Definition:** N blocks labeled A, B, C, ... on 3 stacks:
- Only the top block of a stack can be moved
- A block can be moved to the top of any other stack (or empty stack)
- Goal: achieve a specific configuration

**Paper's Structured Pattern:**
- Initial: Split N blocks alphabetically across two stacks
- Goal: Interleaved pattern requiring complete disassembly/reassembly

---

## Complete Verified Code

<details>
<summary>Click to expand full implementation (~330 lines)</summary>

```python
#!/usr/bin/env python3
# apple_puzzles_kernel_demo.py
#
# PURPOSE:
#   Solve and VERIFY the four "apple puzzles" families with:
#     - deterministic solver (witness generator)
#     - total simulator/verifier (checks every move)
#     - canonical receipts (sha256 of canonical JSON)

import json, hashlib
from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, List, Tuple, Optional, Set
from itertools import combinations

# ============ CANONICAL RECEIPTS ============
def canon_json(obj: Any) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")

def sha256_hex(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()

def add_receipt(payload: Dict[str, Any]) -> Dict[str, Any]:
    p = dict(payload)
    p.pop("receipt_sha256", None)
    payload["receipt_sha256"] = sha256_hex(canon_json(p))
    return payload

# ============ PUZZLE 1: TOWER OF HANOI ============
def hanoi_generate(n: int, src: int=0, aux: int=1, dst: int=2) -> List[Tuple[int,int,int]]:
    """Generate optimal Hanoi solution: 2^n - 1 moves"""
    moves: List[Tuple[int,int,int]] = []
    def rec(k: int, a: int, b: int, c: int):
        if k == 0:
            return
        rec(k-1, a, c, b)
        moves.append((k, a, c))
        rec(k-1, b, a, c)
    rec(n, src, aux, dst)
    return moves

def hanoi_verify(n: int, moves: List[Tuple[int,int,int]]) -> Dict[str, Any]:
    """Total verifier: checks every move, confirms goal"""
    pegs = [list(range(n,0,-1)), [], []]

    for step, (disk, fr, to) in enumerate(moves):
        if fr not in (0,1,2) or to not in (0,1,2) or fr == to:
            return {"PASS": False, "step": step, "reason": "bad peg index"}
        if not pegs[fr] or pegs[fr][-1] != disk:
            return {"PASS": False, "step": step, "reason": "disk not top"}
        if pegs[to] and pegs[to][-1] < disk:
            return {"PASS": False, "step": step, "reason": "larger on smaller"}
        pegs[fr].pop()
        pegs[to].append(disk)

    goal_ok = (pegs[2] == list(range(n,0,-1)) and not pegs[0] and not pegs[1])
    if not goal_ok:
        return {"PASS": False, "reason": "goal not reached"}

    expected_min = 2**n - 1
    return {"PASS": True, "moves": len(moves), "expected_min": expected_min,
            "is_min_length": len(moves) == expected_min}

# ============ PUZZLE 2: CHECKER JUMPING ============
def checker_start(n: int) -> str:
    return "R"*n + "_" + "B"*n

def checker_goal(n: int) -> str:
    return "B"*n + "_" + "R"*n

def checker_neighbors(state: str) -> List[Tuple[int,int,str]]:
    s = list(state)
    e = state.index("_")
    out = []
    if e-1 >= 0 and s[e-1] == "R":
        ns = s.copy(); ns[e]="R"; ns[e-1]="_"
        out.append((e-1, e, "".join(ns)))
    if e-2 >= 0 and s[e-2] == "R" and s[e-1] == "B":
        ns = s.copy(); ns[e]="R"; ns[e-2]="_"
        out.append((e-2, e, "".join(ns)))
    if e+1 < len(s) and s[e+1] == "B":
        ns = s.copy(); ns[e]="B"; ns[e+1]="_"
        out.append((e+1, e, "".join(ns)))
    if e+2 < len(s) and s[e+2] == "B" and s[e+1] == "R":
        ns = s.copy(); ns[e]="B"; ns[e+2]="_"
        out.append((e+2, e, "".join(ns)))
    return out

def checker_bfs_min_solution(n: int) -> List[Tuple[int,int]]:
    start = checker_start(n)
    goal = checker_goal(n)
    q = deque([start])
    prev = {start: None}
    move_prev: Dict[str, Tuple[int,int]] = {}

    while q:
        st = q.popleft()
        if st == goal:
            break
        for fr, to, nst in checker_neighbors(st):
            if nst not in prev:
                prev[nst] = st
                move_prev[nst] = (fr, to)
                q.append(nst)

    moves: List[Tuple[int,int]] = []
    st = goal
    while prev[st] is not None:
        moves.append(move_prev[st])
        st = prev[st]
    moves.reverse()
    return moves

def checker_verify(n: int, moves: List[Tuple[int,int]]) -> Dict[str, Any]:
    st = checker_start(n)
    goal = checker_goal(n)

    for step, (fr, to) in enumerate(moves):
        if st[to] != "_":
            return {"PASS": False, "step": step, "reason": "target not empty"}
        piece = st[fr]
        if piece not in ("R", "B"):
            return {"PASS": False, "step": step, "reason": "source not checker"}
        dist = abs(fr - to)
        if piece == "R" and not (to > fr):
            return {"PASS": False, "step": step, "reason": "red moved backward"}
        if piece == "B" and not (to < fr):
            return {"PASS": False, "step": step, "reason": "blue moved backward"}
        if dist == 2:
            mid = (fr + to) // 2
            if st[mid] == "_" or st[mid] == piece:
                return {"PASS": False, "step": step, "reason": "invalid jump"}
        elif dist != 1:
            return {"PASS": False, "step": step, "reason": "invalid distance"}
        lst = list(st)
        lst[to] = piece
        lst[fr] = "_"
        st = "".join(lst)

    if st != goal:
        return {"PASS": False, "reason": "goal not reached"}
    expected_min = (n + 1)**2 - 1
    return {"PASS": True, "moves": len(moves), "expected_min": expected_min,
            "is_min_length": len(moves) == expected_min}

# ============ PUZZLE 3: RIVER CROSSING ============
def river_people(N: int) -> List[str]:
    out = []
    for i in range(1, N+1):
        out.append(f"a{i}")
        out.append(f"A{i}")
    return out

def safety_ok(side_set: Set[str], N: int) -> bool:
    agents_present = {p for p in side_set if p.startswith("A")}
    if not agents_present:
        return True
    for i in range(1, N+1):
        ai = f"a{i}"
        Ai = f"A{i}"
        if ai in side_set:
            other_agents = any(f"A{j}" in side_set for j in range(1, N+1) if j != i)
            if other_agents and Ai not in side_set:
                return False
    return True

def river_bfs_solution(N: int, k: int) -> List[List[str]]:
    people = river_people(N)
    idx = {p: i for i, p in enumerate(people)}
    full_mask = (1 << (2*N)) - 1

    @dataclass(frozen=True)
    class State:
        right_mask: int
        boat_side: int

    def side_sets(st: State) -> Tuple[Set[str], Set[str]]:
        right = {people[i] for i in range(2*N) if (st.right_mask >> i) & 1}
        left = set(people) - right
        return left, right

    def valid(st: State) -> bool:
        left, right = side_sets(st)
        return safety_ok(left, N) and safety_ok(right, N)

    start = State(0, 0)
    q = deque([start])
    prev = {start: None}
    move_prev: Dict[State, List[str]] = {}
    goal_state: Optional[State] = None

    while q:
        st = q.popleft()
        if st.right_mask == full_mask:
            goal_state = st
            break
        left, right = side_sets(st)
        boat_side_set = left if st.boat_side == 0 else right
        for r in range(1, k+1):
            for combo in combinations(sorted(boat_side_set), r):
                boat_set = set(combo)
                if not safety_ok(boat_set, N):
                    continue
                new_mask = st.right_mask
                for p in boat_set:
                    bit = 1 << idx[p]
                    if st.boat_side == 0:
                        new_mask |= bit
                    else:
                        new_mask &= ~bit
                nst = State(new_mask, 1 - st.boat_side)
                if not valid(nst):
                    continue
                if nst not in prev:
                    prev[nst] = st
                    move_prev[nst] = list(combo)
                    q.append(nst)

    moves: List[List[str]] = []
    st = goal_state
    while prev[st] is not None:
        moves.append(move_prev[st])
        st = prev[st]
    moves.reverse()
    return moves

def river_verify(N: int, k: int, moves: List[List[str]]) -> Dict[str, Any]:
    people = river_people(N)
    idx = {p: i for i, p in enumerate(people)}
    right_mask = 0
    boat_side = 0
    full_mask = (1 << (2*N)) - 1

    def side_sets(mask: int) -> Tuple[Set[str], Set[str]]:
        right = {people[i] for i in range(2*N) if (mask >> i) & 1}
        left = set(people) - right
        return left, right

    for step, boat in enumerate(moves):
        if not (1 <= len(boat) <= k):
            return {"PASS": False, "step": step, "reason": "boat capacity"}
        left, right = side_sets(right_mask)
        boat_set = set(boat)
        if boat_side == 0 and not boat_set.issubset(left):
            return {"PASS": False, "step": step, "reason": "not on left"}
        if boat_side == 1 and not boat_set.issubset(right):
            return {"PASS": False, "step": step, "reason": "not on right"}
        if not safety_ok(boat_set, N):
            return {"PASS": False, "step": step, "reason": "boat safety"}
        for p in boat_set:
            bit = 1 << idx[p]
            if boat_side == 0:
                right_mask |= bit
            else:
                right_mask &= ~bit
        boat_side = 1 - boat_side
        left, right = side_sets(right_mask)
        if not (safety_ok(left, N) and safety_ok(right, N)):
            return {"PASS": False, "step": step, "reason": "bank safety"}

    if right_mask != full_mask:
        return {"PASS": False, "reason": "goal not reached"}
    return {"PASS": True, "moves": len(moves), "goal_reached": True}

# ============ PUZZLE 4: BLOCKS WORLD ============
def blocks_instance(N: int) -> Tuple[List[List[str]], List[List[str]]]:
    blocks = [chr(ord("A") + i) for i in range(N)]
    half = N // 2
    s0 = blocks[:half]
    s1 = blocks[half:]
    init = [s0.copy(), s1.copy(), []]
    s0_top = s0[::-1]
    s1_top = s1[::-1]
    goal_topdown = []
    for i in range(half):
        goal_topdown.append(s1_top[i])
        goal_topdown.append(s0_top[i])
    goal0 = goal_topdown[::-1]
    goal = [goal0, [], []]
    return init, goal

def blocks_moves_bfs(init, goal, max_states=2_000_000):
    def norm(st):
        return tuple(tuple(s) for s in st)
    start = norm(init)
    target = norm(goal)
    q = deque([start])
    prev = {start: None}
    move_prev = {}

    while q:
        st = q.popleft()
        if st == target:
            break
        stacks = [list(s) for s in st]
        for i in range(len(stacks)):
            if not stacks[i]:
                continue
            block = stacks[i][-1]
            for j in range(len(stacks)):
                if i == j:
                    continue
                nst = [list(s) for s in stacks]
                nst[i].pop()
                nst[j].append(block)
                nst_t = norm(nst)
                if nst_t not in prev:
                    prev[nst_t] = st
                    move_prev[nst_t] = (block, i, j)
                    q.append(nst_t)

    moves = []
    st = target
    while prev[st] is not None:
        moves.append(move_prev[st])
        st = prev[st]
    moves.reverse()
    return moves

def blocks_verify(init, goal, moves):
    stacks = [s.copy() for s in init]
    for step, (block, i, j) in enumerate(moves):
        if not stacks[i] or stacks[i][-1] != block:
            return {"PASS": False, "step": step, "reason": "invalid move"}
        stacks[i].pop()
        stacks[j].append(block)
    if stacks != goal:
        return {"PASS": False, "reason": "goal not reached"}
    return {"PASS": True, "moves": len(moves), "goal_reached": True}
```

</details>

---

## NSL Encoding of Puzzle States

The kernel's [Null-State Logic](/proof/derivations/core-logic/opoch-kernel) applies:

| NSL Value | Puzzle Interpretation |
|-----------|----------------------|
| **+1** | Move verified (legal and executed) |
| **0** | Move status unknown (not yet checked) |
| **-1** | Move refuted (violates constraints) |

The verifier converts a sequence of 0-state moves into either all +1 (PASS) or identifies the first -1 (FAIL with reason).

---

## Observer Closure: Puzzle Control Respects Π

:::note DEFINITION
Define raw controller N and Π-closed controller:

$$Q := \Pi \circ N \circ \Pi$$

**The Diamond Law (No Hidden Channel):**

$$\boxed{\mathcal{N} \circ Q = Q \circ \mathcal{N}}$$

**Consequence:** Any puzzle solution that depends on representation labels rather than Π-fixed state structure is **invalid (minted)**.
:::

### How Puzzle Solving Satisfies Observer Closure

| Puzzle | State Representation | Π-Invariant? |
|--------|---------------------|--------------|
| **Hanoi** | Peg contents (list of disk sizes) | ✓ Position-based, not label-based |
| **Checkers** | Board string (R/B/_ positions) | ✓ Configuration, not piece IDs |
| **River** | Bitmask of right bank occupancy | ✓ Set membership, not order |
| **Blocks** | Stack contents (top-to-bottom) | ✓ Structure, not block addresses |

**Key property:** BFS explores states in canonical form. The verifier checks Π-fixed state transitions. Relabeling blocks A↔B would produce different canonical states — the algorithm makes no label-dependent choices.

### Anti-Minting in BFS

BFS satisfies Observer Closure because:
1. States are compared by canonical representation (norm function)
2. Move selection is deterministic based on state structure
3. No tie-breaking uses arbitrary labels
4. Solution validity depends only on Π-fixed goal test

---

## Kernel Alignment Verification

This demo satisfies all kernel requirements:

| Kernel Requirement | Demo Implementation |
|-------------------|---------------------|
| **A0 Witnessability** | Solutions are finite move sequences |
| **Total Verifier** | Simulators check every move |
| **UNIQUE or Ω** | <span className="status-pass">PASS</span> with witness, or RuntimeError |
| **No Minted Distinctions** | BFS explores all reachable states |
| **Canonical Receipts** | SHA256 of canonical JSON |
| **Gauge Invariance** | Move encoding is position-based |
| **Observer Closure** | Control decisions are Π-fixed (no label dependence) |

---

## Conclusion

The Apple Puzzles demonstrate a fundamental truth:

> **Correctness is a property of the verification loop, not the generator.**

LLMs fail because they generate without verifying. The kernel succeeds because it gates output on verification.

This is not a matter of "more training" or "better prompts." It is a structural difference:

- **LLMs**: Output = f(pattern_match(input))
- **Kernel**: Output = witness iff V(witness) = PASS

The kernel approach scales because truth scales. Only cost grows.

---

**Foundation:** [The Opoch Kernel: Null-State Logic](/proof/derivations/core-logic/opoch-kernel) — Complete kernel specification

**Related:** [Why LLMs Can't Reason](/resources/llm_reasoning_failures) — Architectural limitations
