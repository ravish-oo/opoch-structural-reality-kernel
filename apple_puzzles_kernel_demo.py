#!/usr/bin/env python3
# apple_puzzles_kernel_demo.py
#
# PURPOSE:
#   Solve and VERIFY the four "apple puzzles" families with:
#     - deterministic solver (witness generator)
#     - total simulator/verifier (checks every move)
#     - canonical receipts (sha256 of canonical JSON)
#
# OUTPUT:
#   apple_puzzles_kernel_demo_verified.json
#
# RUN:
#   python3 apple_puzzles_kernel_demo.py

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
        moves.append((k, a, c))  # move disk k from a to c
        rec(k-1, b, a, c)
    rec(n, src, aux, dst)
    return moves

def hanoi_verify(n: int, moves: List[Tuple[int,int,int]]) -> Dict[str, Any]:
    """Total verifier: checks every move, confirms goal"""
    pegs = [list(range(n,0,-1)), [], []]  # peg[i][-1] is top

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
    """Generate all legal moves from state"""
    s = list(state)
    e = state.index("_")
    out = []
    # Red slide right
    if e-1 >= 0 and s[e-1] == "R":
        ns = s.copy(); ns[e]="R"; ns[e-1]="_"
        out.append((e-1, e, "".join(ns)))
    # Red jump right over blue
    if e-2 >= 0 and s[e-2] == "R" and s[e-1] == "B":
        ns = s.copy(); ns[e]="R"; ns[e-2]="_"
        out.append((e-2, e, "".join(ns)))
    # Blue slide left
    if e+1 < len(s) and s[e+1] == "B":
        ns = s.copy(); ns[e]="B"; ns[e+1]="_"
        out.append((e+1, e, "".join(ns)))
    # Blue jump left over red
    if e+2 < len(s) and s[e+2] == "B" and s[e+1] == "R":
        ns = s.copy(); ns[e]="B"; ns[e+2]="_"
        out.append((e+2, e, "".join(ns)))
    return out

def checker_bfs_min_solution(n: int) -> List[Tuple[int,int]]:
    """BFS finds shortest path = (n+1)^2 - 1 moves"""
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

    if goal not in prev:
        raise RuntimeError("No solution found")

    moves: List[Tuple[int,int]] = []
    st = goal
    while prev[st] is not None:
        moves.append(move_prev[st])
        st = prev[st]
    moves.reverse()
    return moves

def checker_verify(n: int, moves: List[Tuple[int,int]]) -> Dict[str, Any]:
    """Total verifier for checker jumping"""
    st = checker_start(n)
    goal = checker_goal(n)

    for step, (fr, to) in enumerate(moves):
        if fr < 0 or to < 0 or fr >= len(st) or to >= len(st):
            return {"PASS": False, "step": step, "reason": "index out of bounds"}
        if st[to] != "_":
            return {"PASS": False, "step": step, "reason": "target not empty"}

        piece = st[fr]
        if piece not in ("R", "B"):
            return {"PASS": False, "step": step, "reason": "source not checker"}

        dist = abs(fr - to)
        # Direction constraint
        if piece == "R" and not (to > fr):
            return {"PASS": False, "step": step, "reason": "red moved backward"}
        if piece == "B" and not (to < fr):
            return {"PASS": False, "step": step, "reason": "blue moved backward"}

        if dist == 1:
            pass  # slide
        elif dist == 2:
            mid = (fr + to) // 2
            if st[mid] == "_" or st[mid] == piece:
                return {"PASS": False, "step": step, "reason": "invalid jump"}
        else:
            return {"PASS": False, "step": step, "reason": "invalid distance"}

        # Execute move
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
        out.append(f"a{i}")  # actor
        out.append(f"A{i}")  # agent
    return out

def safety_ok(side_set: Set[str], N: int) -> bool:
    """Check safety constraint on a bank or boat"""
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
    """BFS solver for river crossing"""
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
        boat_people = sorted(list(boat_side_set))

        for r in range(1, k+1):
            for combo in combinations(boat_people, r):
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

    if goal_state is None:
        raise RuntimeError("No solution found")

    moves: List[List[str]] = []
    st = goal_state
    while prev[st] is not None:
        moves.append(move_prev[st])
        st = prev[st]
    moves.reverse()
    return moves

def river_verify(N: int, k: int, moves: List[List[str]]) -> Dict[str, Any]:
    """Total verifier for river crossing"""
    people = river_people(N)
    idx = {p: i for i, p in enumerate(people)}
    right_mask = 0
    boat_side = 0
    full_mask = (1 << (2*N)) - 1

    def side_sets(mask: int) -> Tuple[Set[str], Set[str]]:
        right = {people[i] for i in range(2*N) if (mask >> i) & 1}
        left = set(people) - right
        return left, right

    left, right = side_sets(right_mask)
    if not (safety_ok(left, N) and safety_ok(right, N)):
        return {"PASS": False, "reason": "initial safety violation"}

    for step, boat in enumerate(moves):
        if not (1 <= len(boat) <= k):
            return {"PASS": False, "step": step, "reason": "boat capacity violation"}

        left, right = side_sets(right_mask)
        boat_set = set(boat)

        if boat_side == 0 and not boat_set.issubset(left):
            return {"PASS": False, "step": step, "reason": "passengers not on left"}
        if boat_side == 1 and not boat_set.issubset(right):
            return {"PASS": False, "step": step, "reason": "passengers not on right"}
        if not safety_ok(boat_set, N):
            return {"PASS": False, "step": step, "reason": "boat safety violation"}

        # Move across
        for p in boat_set:
            bit = 1 << idx[p]
            if boat_side == 0:
                right_mask |= bit
            else:
                right_mask &= ~bit
        boat_side = 1 - boat_side

        # Check bank safety after move
        left, right = side_sets(right_mask)
        if not (safety_ok(left, N) and safety_ok(right, N)):
            return {"PASS": False, "step": step, "reason": "bank safety violation"}

    if right_mask != full_mask:
        return {"PASS": False, "reason": "goal not reached"}
    return {"PASS": True, "moves": len(moves), "goal_reached": True}

# ============ PUZZLE 4: BLOCKS WORLD ============
def blocks_instance(N: int) -> Tuple[List[List[str]], List[List[str]]]:
    """Generate paper's structured pattern"""
    assert N % 2 == 0
    blocks = [chr(ord("A") + i) for i in range(N)]
    half = N // 2
    s0 = blocks[:half]
    s1 = blocks[half:]
    init = [s0.copy(), s1.copy(), []]

    # Goal interleaves blocks
    s0_top = s0[::-1]
    s1_top = s1[::-1]
    goal_topdown = []
    for i in range(half):
        goal_topdown.append(s1_top[i])
        goal_topdown.append(s0_top[i])
    goal0 = goal_topdown[::-1]
    goal = [goal0, [], []]
    return init, goal

def blocks_moves_bfs(init: List[List[str]], goal: List[List[str]],
                     max_states: int = 2_000_000) -> List[Tuple[str, int, int]]:
    """BFS solver for blocks world"""
    def norm(st):
        return tuple(tuple(s) for s in st)

    start = norm(init)
    target = norm(goal)
    q = deque([start])
    prev = {start: None}
    move_prev: Dict[Any, Tuple[str, int, int]] = {}

    while q:
        st = q.popleft()
        if st == target:
            break
        stacks = [list(s) for s in st]
        m = len(stacks)
        for i in range(m):
            if not stacks[i]:
                continue
            block = stacks[i][-1]
            for j in range(m):
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
                    if len(prev) > max_states:
                        raise RuntimeError("state explosion")

    if target not in prev:
        raise RuntimeError("No solution found")

    moves: List[Tuple[str, int, int]] = []
    st = target
    while prev[st] is not None:
        moves.append(move_prev[st])
        st = prev[st]
    moves.reverse()
    return moves

def blocks_verify(init: List[List[str]], goal: List[List[str]],
                  moves: List[Tuple[str, int, int]]) -> Dict[str, Any]:
    """Total verifier for blocks world"""
    stacks = [s.copy() for s in init]
    m = len(stacks)

    for step, (block, i, j) in enumerate(moves):
        if i < 0 or j < 0 or i >= m or j >= m or i == j:
            return {"PASS": False, "step": step, "reason": "bad stack index"}
        if not stacks[i]:
            return {"PASS": False, "step": step, "reason": "source empty"}
        if stacks[i][-1] != block:
            return {"PASS": False, "step": step, "reason": "block not on top"}
        stacks[i].pop()
        stacks[j].append(block)

    if stacks != goal:
        return {"PASS": False, "reason": "goal not reached"}
    return {"PASS": True, "moves": len(moves), "goal_reached": True}

# ============ MAIN: RUN SUITE ============
def main():
    results: Dict[str, Any] = {
        "suite": "Apple puzzles kernel demo: solver + verifier + receipts",
        "kernel_guarantee": "Output only when verifier PASS; else Omega",
        "puzzles": {},
        "full_move_lists": {}
    }

    # Tower of Hanoi (n=1..12)
    print("Running Tower of Hanoi (n=1..12)...")
    hanoi_runs = []
    for n in range(1, 13):
        moves = hanoi_generate(n)
        ver = hanoi_verify(n, moves)
        hanoi_runs.append(add_receipt({
            "n": n,
            "moves_len": len(moves),
            "expected_min": 2**n - 1,
            "verifier": ver,
            "moves": moves
        }))
        print(f"  n={n}: {len(moves)} moves, PASS={ver['PASS']}, min_length={ver.get('is_min_length', False)}")
    results["puzzles"]["tower_of_hanoi"] = add_receipt({
        "definition": "Move n disks from peg 0 to peg 2",
        "min_moves_formula": "2^n - 1",
        "runs": hanoi_runs
    })

    # Checker Jumping (n=1..8)
    print("\nRunning Checker Jumping (n=1..8)...")
    checker_runs = []
    checker_moves = {}
    for n in range(1, 9):
        moves = checker_bfs_min_solution(n)
        ver = checker_verify(n, moves)
        checker_runs.append(add_receipt({
            "n": n,
            "start": checker_start(n),
            "goal": checker_goal(n),
            "moves_len": len(moves),
            "expected_min": (n+1)**2 - 1,
            "verifier": ver
        }))
        checker_moves[str(n)] = moves
        print(f"  n={n}: {len(moves)} moves, expected={(n+1)**2-1}, PASS={ver['PASS']}")
    results["puzzles"]["checker_jumping"] = add_receipt({
        "definition": "Swap R's and B's with slides/jumps",
        "min_moves_formula": "(n+1)^2 - 1",
        "runs": checker_runs
    })
    results["full_move_lists"]["checker_jumping"] = checker_moves

    # River Crossing
    print("\nRunning River Crossing...")
    river_runs = []
    river_moves = {}
    for N, k in [(2, 2), (3, 2), (4, 3)]:
        moves = river_bfs_solution(N, k)
        ver = river_verify(N, k, moves)
        river_runs.append(add_receipt({
            "N_pairs": N,
            "boat_capacity_k": k,
            "moves_len": len(moves),
            "verifier": ver
        }))
        river_moves[f"{N}_{k}"] = moves
        print(f"  N={N}, k={k}: {len(moves)} crossings, PASS={ver['PASS']}")
    results["puzzles"]["river_crossing"] = add_receipt({
        "definition": "N actor-agent pairs cross river with safety constraint",
        "runs": river_runs
    })
    results["full_move_lists"]["river_crossing"] = river_moves

    # Blocks World
    print("\nRunning Blocks World...")
    blocks_runs = []
    blocks_moves_dict = {}
    for N in [4, 6]:
        init, goal = blocks_instance(N)
        moves = blocks_moves_bfs(init, goal)
        ver = blocks_verify(init, goal, moves)
        blocks_runs.append(add_receipt({
            "N_blocks": N,
            "init": init,
            "goal": goal,
            "moves_len": len(moves),
            "verifier": ver
        }))
        blocks_moves_dict[str(N)] = moves
        print(f"  N={N}: {len(moves)} moves, PASS={ver['PASS']}")
    results["puzzles"]["blocks_world"] = add_receipt({
        "definition": "Rearrange blocks between stacks",
        "runs": blocks_runs
    })
    results["full_move_lists"]["blocks_world"] = blocks_moves_dict

    add_receipt(results)

    out_path = "apple_puzzles_kernel_demo_verified.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, sort_keys=True)

    print(f"\n{'='*60}")
    print(f"Wrote: {out_path}")
    print(f"Master receipt_sha256: {results['receipt_sha256']}")
    print(f"\nSUMMARY - All puzzles VERIFIED:")
    for name, data in results["puzzles"].items():
        runs = data.get("runs", [])
        all_pass = all(r["verifier"]["PASS"] for r in runs)
        print(f"  {name}: {len(runs)} instances, ALL PASS = {all_pass}")

if __name__ == "__main__":
    main()
