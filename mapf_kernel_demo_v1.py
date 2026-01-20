"""
MAPF_KERNEL_DEMO_v1
Multi-Agent Path Finding with:
  - Hard verifier (PASS or minimal conflict witness)
  - Exact CBS solver (sum-of-costs optimal under standard assumptions)
  - Deterministic receipts (SHA-256 of canonical JSON witness)

Model:
  - Discrete time
  - Move to neighbor or wait
  - No vertex conflicts
  - No edge-swap conflicts
  - Agents stay at goal once arrived (standard MAPF convention)
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, FrozenSet, List, Optional, Set, Tuple, Iterable
from heapq import heappush, heappop
import hashlib
import json
import math


# ──────────────────────────────────────────────────────────────────────────────
# Canon / Receipt
# ──────────────────────────────────────────────────────────────────────────────

def canon_json(obj) -> str:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False)

def sha256_hex(s: str) -> str:
    return hashlib.sha256(s.encode("utf-8")).hexdigest()


# ──────────────────────────────────────────────────────────────────────────────
# Grid graph
# ──────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Grid:
    w: int
    h: int
    obstacles: FrozenSet[int]  # vertex ids blocked

    def vid(self, x: int, y: int) -> int:
        return y * self.w + x

    def xy(self, v: int) -> Tuple[int, int]:
        return (v % self.w, v // self.w)

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.w and 0 <= y < self.h

    def passable(self, v: int) -> bool:
        return v not in self.obstacles

    def neighbors(self, v: int) -> List[int]:
        # 4-neighborhood + wait
        x, y = self.xy(v)
        cand = [(x, y), (x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        out: List[int] = []
        for nx, ny in cand:
            if self.in_bounds(nx, ny):
                nv = self.vid(nx, ny)
                if self.passable(nv):
                    out.append(nv)
        return out

    def manhattan(self, a: int, b: int) -> int:
        ax, ay = self.xy(a)
        bx, by = self.xy(b)
        return abs(ax - bx) + abs(ay - by)

    def stable_id(self) -> str:
        # canonical identity of map layout
        obs = sorted(list(self.obstacles))
        return sha256_hex(canon_json({"w": self.w, "h": self.h, "obstacles": obs}))


# ──────────────────────────────────────────────────────────────────────────────
# Constraints
# ──────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class VertexConstraint:
    agent: int
    v: int
    t: int

@dataclass(frozen=True)
class EdgeConstraint:
    agent: int
    u: int
    v: int
    t: int  # forbids u->v from t to t+1


@dataclass
class AgentConstraints:
    vertex_forbid: Set[Tuple[int, int]]  # (t, v)
    edge_forbid: Set[Tuple[int, int, int]]  # (t, u, v)

    def forbids_vertex(self, v: int, t: int) -> bool:
        return (t, v) in self.vertex_forbid

    def forbids_edge(self, u: int, v: int, t: int) -> bool:
        return (t, u, v) in self.edge_forbid


def compile_constraints(constraints: List[object], k: int) -> List[AgentConstraints]:
    out = [AgentConstraints(set(), set()) for _ in range(k)]
    for c in constraints:
        if isinstance(c, VertexConstraint):
            out[c.agent].vertex_forbid.add((c.t, c.v))
        elif isinstance(c, EdgeConstraint):
            out[c.agent].edge_forbid.add((c.t, c.u, c.v))
        else:
            raise ValueError(f"Unknown constraint type: {c}")
    return out


# ──────────────────────────────────────────────────────────────────────────────
# Verifier
# ──────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Conflict:
    kind: str              # "VERTEX" or "EDGE_SWAP" or "DYNAMICS" or "STARTGOAL"
    t: int
    a1: int
    a2: int
    v: Optional[int] = None
    u: Optional[int] = None
    w: Optional[int] = None
    detail: str = ""

class Verifier:
    @staticmethod
    def verify(grid: Grid, starts: List[int], goals: List[int], paths: List[List[int]]) -> Tuple[bool, Optional[Conflict]]:
        k = len(starts)
        if len(paths) != k:
            return False, Conflict("STARTGOAL", 0, -1, -1, detail="wrong number of paths")

        # check each path starts and ends correctly, and legal moves
        makespan = max(len(p) for p in paths) - 1
        for i in range(k):
            p = paths[i]
            if len(p) == 0:
                return False, Conflict("STARTGOAL", 0, i, -1, detail="empty path")
            if p[0] != starts[i]:
                return False, Conflict("STARTGOAL", 0, i, -1, detail="wrong start")
            # Pad convention: agent stays at goal after arrival; so last vertex must be goal
            if p[-1] != goals[i]:
                return False, Conflict("STARTGOAL", len(p)-1, i, -1, detail="wrong goal/end")
            # Dynamics
            for t in range(len(p) - 1):
                u = p[t]
                v = p[t+1]
                if not grid.passable(u) or not grid.passable(v):
                    return False, Conflict("DYNAMICS", t, i, -1, u=u, w=v, detail="hit obstacle")
                if v not in grid.neighbors(u):
                    return False, Conflict("DYNAMICS", t, i, -1, u=u, w=v, detail="non-neighbor move")

        # Pad all paths to makespan by waiting at goal
        pad_paths: List[List[int]] = []
        for i in range(k):
            p = paths[i][:]
            while len(p) < makespan + 1:
                p.append(goals[i])
            pad_paths.append(p)

        # Vertex conflicts
        for t in range(makespan + 1):
            occ: Dict[int, int] = {}
            for i in range(k):
                v = pad_paths[i][t]
                if v in occ:
                    j = occ[v]
                    return False, Conflict("VERTEX", t, j, i, v=v, detail="same vertex")
                occ[v] = i

        # Edge swaps
        for t in range(makespan):
            moves: Dict[Tuple[int, int], int] = {}
            for i in range(k):
                u = pad_paths[i][t]
                v = pad_paths[i][t+1]
                moves[(u, v)] = i
            for (u, v), i in moves.items():
                if (v, u) in moves:
                    j = moves[(v, u)]
                    if i != j and u != v:  # ignore double-wait
                        a1, a2 = (i, j) if i < j else (j, i)
                        return False, Conflict("EDGE_SWAP", t, a1, a2, u=u, w=v, detail="head-on swap")

        return True, None


# ──────────────────────────────────────────────────────────────────────────────
# Low-level A* (single agent with constraints)
# ──────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True, order=True)
class AStarState:
    v: int
    t: int

def reconstruct(came_from: Dict[AStarState, AStarState], end: AStarState) -> List[int]:
    cur = end
    rev: List[int] = [cur.v]
    while cur in came_from:
        cur = came_from[cur]
        rev.append(cur.v)
    rev.reverse()
    return rev

def violates_goal_stay(goal: int, t_arrive: int, horizon: int, ac: AgentConstraints) -> bool:
    # If any vertex constraint forbids being at goal at time >= arrival, goal-stay fails.
    for tt in range(t_arrive, horizon + 1):
        if ac.forbids_vertex(goal, tt):
            return True
    # Also forbid waiting edge goal->goal if any edge constraint says so
    for tt in range(t_arrive, horizon):
        if ac.forbids_edge(goal, goal, tt):
            return True
    return False

def astar_single_agent(
    grid: Grid,
    start: int,
    goal: int,
    ac: AgentConstraints,
    horizon: int,
) -> Optional[List[int]]:
    # A* over (v,t) with t<=horizon, cost=1 per step.
    start_state = AStarState(start, 0)
    gscore: Dict[AStarState, int] = {start_state: 0}
    came_from: Dict[AStarState, AStarState] = {}

    def h(v: int) -> int:
        return grid.manhattan(v, goal)

    # (f, tie1, tie2, state)
    open_heap: List[Tuple[int, int, int, AStarState]] = []
    heappush(open_heap, (h(start), 0, 0, start_state))

    visited_best: Set[AStarState] = set()

    while open_heap:
        f, g, _, cur = heappop(open_heap)
        if cur in visited_best:
            continue
        visited_best.add(cur)

        # vertex constraint at current time
        if ac.forbids_vertex(cur.v, cur.t):
            continue

        if cur.v == goal:
            # accept only if goal-stay is allowed through horizon
            if not violates_goal_stay(goal, cur.t, horizon, ac):
                return reconstruct(came_from, cur)

        if cur.t == horizon:
            continue

        for nv in grid.neighbors(cur.v):
            nt = cur.t + 1
            # enforce edge constraint
            if ac.forbids_edge(cur.v, nv, cur.t):
                continue
            # enforce vertex constraint at next time
            if ac.forbids_vertex(nv, nt):
                continue

            nxt = AStarState(nv, nt)
            tentative = g + 1
            if nxt not in gscore or tentative < gscore[nxt]:
                gscore[nxt] = tentative
                came_from[nxt] = cur
                heappush(open_heap, (tentative + h(nv), tentative, nt, nxt))

    return None


# ──────────────────────────────────────────────────────────────────────────────
# CBS (sum-of-costs optimal)
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class CBSNode:
    constraints: List[object]           # VertexConstraint / EdgeConstraint
    paths: List[List[int]]
    cost: int
    makespan: int
    node_id: str                        # deterministic fingerprint for tie-break

    def __lt__(self, other):
        # For heap tie-breaking: compare by cost then node_id
        if self.cost != other.cost:
            return self.cost < other.cost
        return self.node_id < other.node_id

def soc_cost(paths: List[List[int]]) -> int:
    # sum of path lengths - 1 (moves+waits until arrival)
    return sum(len(p) - 1 for p in paths)

def detect_first_conflict(paths: List[List[int]], goals: List[int]) -> Optional[Conflict]:
    k = len(paths)
    makespan = max(len(p) for p in paths) - 1
    pad = []
    for i in range(k):
        p = paths[i][:]
        while len(p) < makespan + 1:
            p.append(goals[i])
        pad.append(p)

    # deterministic: scan time, then lower agent ids first
    for t in range(makespan + 1):
        occ: Dict[int, int] = {}
        for i in range(k):
            v = pad[i][t]
            if v in occ:
                j = occ[v]
                a1, a2 = (j, i) if j < i else (i, j)
                return Conflict("VERTEX", t, a1, a2, v=v, detail="same vertex")
            occ[v] = i

    for t in range(makespan):
        moves: Dict[Tuple[int, int], int] = {}
        for i in range(k):
            u = pad[i][t]
            v = pad[i][t+1]
            moves[(u, v)] = i
        for (u, v), i in moves.items():
            if (v, u) in moves:
                j = moves[(v, u)]
                if i != j and u != v:
                    a1, a2 = (i, j) if i < j else (j, i)
                    return Conflict("EDGE_SWAP", t, a1, a2, u=u, w=v, detail="head-on swap")
    return None

def canonical_node_id(grid_id: str, constraints: List[object], paths: List[List[int]]) -> str:
    # Deterministic fingerprint for tie-breaking in heap.
    # Canonicalize constraints as sorted tuples.
    c_repr = []
    for c in constraints:
        if isinstance(c, VertexConstraint):
            c_repr.append(("V", c.agent, c.v, c.t))
        else:
            assert isinstance(c, EdgeConstraint)
            c_repr.append(("E", c.agent, c.u, c.v, c.t))
    c_repr.sort()

    obj = {"grid": grid_id, "constraints": c_repr, "paths": paths}
    return sha256_hex(canon_json(obj))

def forbid(agent: int, conf: Conflict) -> object:
    if conf.kind == "VERTEX":
        assert conf.v is not None
        return VertexConstraint(agent=agent, v=conf.v, t=conf.t)
    if conf.kind == "EDGE_SWAP":
        assert conf.u is not None and conf.w is not None
        # forbid the directed move u->w for this agent at time t
        return EdgeConstraint(agent=agent, u=conf.u, v=conf.w, t=conf.t)
    raise ValueError("Unsupported conflict type")

def cbs_soc(
    grid: Grid,
    starts: List[int],
    goals: List[int],
    horizon_slack: int = 30,
    hard_horizon_cap: int = 400,
) -> Tuple[Optional[List[List[int]]], dict]:
    """
    Returns:
      paths or None, and stats dict.
    """
    k = len(starts)
    grid_id = grid.stable_id()

    def pick_horizon_for_agent(i: int, ac: AgentConstraints, base: int) -> int:
        # Robust upper bound for low-level search:
        # base distance + constraint pressure + slack, capped.
        pressure = len(ac.vertex_forbid) + len(ac.edge_forbid)
        H = base + pressure + horizon_slack
        return min(H, hard_horizon_cap)

    # Root node
    root_constraints: List[object] = []
    acs = compile_constraints(root_constraints, k)
    root_paths: List[List[int]] = []
    for i in range(k):
        base = grid.manhattan(starts[i], goals[i])
        H = pick_horizon_for_agent(i, acs[i], base)
        p = astar_single_agent(grid, starts[i], goals[i], acs[i], H)
        if p is None:
            return None, {"status": "Ω", "reason": f"no initial path for agent {i}"}
        root_paths.append(p)

    root = CBSNode(
        constraints=root_constraints,
        paths=root_paths,
        cost=soc_cost(root_paths),
        makespan=max(len(p) for p in root_paths) - 1,
        node_id=canonical_node_id(grid_id, root_constraints, root_paths),
    )

    # OPEN ordered by (cost, tie-break)
    open_heap: List[Tuple[int, str, CBSNode]] = []
    heappush(open_heap, (root.cost, root.node_id, root))

    expansions = 0
    generated = 1
    max_open = 1

    while open_heap:
        _, _, node = heappop(open_heap)
        expansions += 1

        # If already conflict-free, verify and return.
        ok, fail = Verifier.verify(grid, starts, goals, node.paths)
        if ok:
            return node.paths, {
                "status": "UNIQUE",
                "expansions": expansions,
                "generated": generated,
                "max_open": max_open,
                "soc": node.cost,
                "makespan": node.makespan,
            }

        # else get first conflict (deterministic)
        conf = detect_first_conflict(node.paths, goals)
        if conf is None:
            # should not happen if verifier failed, but keep safe
            return None, {"status": "Ω", "reason": "verifier failed but no conflict detected", "verifier": str(fail)}

        # Branch on the two agents involved
        for a in [conf.a1, conf.a2]:
            child_constraints = list(node.constraints) + [forbid(a, conf)]
            child_acs = compile_constraints(child_constraints, k)

            # Copy paths and replan only agent a
            child_paths = [p[:] for p in node.paths]

            base = grid.manhattan(starts[a], goals[a])
            H = pick_horizon_for_agent(a, child_acs[a], base)

            new_path = astar_single_agent(grid, starts[a], goals[a], child_acs[a], H)
            if new_path is None:
                # This branch is infeasible for agent a under constraints; skip it.
                continue

            child_paths[a] = new_path
            child = CBSNode(
                constraints=child_constraints,
                paths=child_paths,
                cost=soc_cost(child_paths),
                makespan=max(len(p) for p in child_paths) - 1,
                node_id=canonical_node_id(grid_id, child_constraints, child_paths),
            )
            heappush(open_heap, (child.cost, child.node_id, child))
            generated += 1

        max_open = max(max_open, len(open_heap))

    return None, {"status": "Ω", "reason": "no solution found (search exhausted)"}


# ──────────────────────────────────────────────────────────────────────────────
# Receipt object
# ──────────────────────────────────────────────────────────────────────────────

def make_receipt(
    grid: Grid,
    starts: List[int],
    goals: List[int],
    paths: List[List[int]],
    stats: dict,
) -> Tuple[str, dict]:
    witness = {
        "map": {"w": grid.w, "h": grid.h, "obstacles": sorted(list(grid.obstacles)), "map_id": grid.stable_id()},
        "starts": starts,
        "goals": goals,
        "objective": "SOC",
        "soc": stats.get("soc"),
        "makespan": stats.get("makespan"),
        "paths": paths,
        "stats": stats,
        "verifier": "PASS",
    }
    receipt = sha256_hex(canon_json(witness))
    return receipt, witness


# ──────────────────────────────────────────────────────────────────────────────
# Demo scenario: warehouse-like grid, 16 agents, forced interactions
# ──────────────────────────────────────────────────────────────────────────────

def build_warehouse_like_grid() -> Grid:
    """
    Warehouse motif:
      - 25x18 grid
      - shelf blocks as obstacles forming aisles
      - clear cross-aisle corridors
    """
    w, h = 25, 18
    obs: Set[int] = set()

    def add_rect(x0, y0, x1, y1):
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                obs.add(y * w + x)

    # Create vertical shelf blocks (aisles)
    # Shelves occupy bands; leave corridors at y=4 and y=13 as cross-aisles.
    for x0 in [3, 8, 13, 18]:
        add_rect(x0, 1, x0 + 2, 3)     # upper shelf
        add_rect(x0, 5, x0 + 2, 12)    # middle shelf
        add_rect(x0, 14, x0 + 2, 16)   # lower shelf

    # Leave outer perimeter passable; also a central open zone
    return Grid(w=w, h=h, obstacles=frozenset(obs))

def demo():
    grid = build_warehouse_like_grid()
    k = 16

    # Starts/goals: mostly independent flows + one forced swap across a narrow corridor
    # Use vertex ids for determinism
    def V(x, y): return grid.vid(x, y)

    starts = [
        V(1, 2), V(1, 6), V(1, 10), V(1, 15),
        V(23, 2), V(23, 6), V(23, 10), V(23, 15),
        V(6, 4), V(11, 4), V(16, 4), V(21, 4),
        V(6, 13), V(11, 13), V(16, 13), V(21, 13),
    ]

    goals = [
        V(23, 2), V(23, 6), V(23, 10), V(23, 15),
        V(1, 2), V(1, 6), V(1, 10), V(1, 15),
        # Cross-aisle agents: shift right by one "bay"
        V(11, 4), V(16, 4), V(21, 4), V(6, 4),
        V(11, 13), V(16, 13), V(21, 13), V(6, 13),
    ]

    paths, stats = cbs_soc(grid, starts, goals, horizon_slack=50, hard_horizon_cap=500)
    if paths is None:
        print("Ω:", stats)
        return

    ok, conf = Verifier.verify(grid, starts, goals, paths)
    print("VERIFIER:", "PASS" if ok else f"FAIL {conf}")
    print("STATS:", stats)

    receipt, witness = make_receipt(grid, starts, goals, paths, stats)
    print("RECEIPT_SHA256:", receipt)

    # Tamper test: introduce an illegal edge-swap by forcing agent 0 to move into agent 4's start at t=1
    # This should FAIL verifier deterministically with a conflict witness.
    tampered = [p[:] for p in paths]
    if len(tampered[0]) >= 2:
        tampered[0][1] = starts[4]  # force collision attempt
    ok2, conf2 = Verifier.verify(grid, starts, goals, tampered)
    print("TAMPER_VERIFIER:", "PASS" if ok2 else f"FAIL {conf2.kind} at t={conf2.t} agents=({conf2.a1},{conf2.a2}) detail={conf2.detail}")

    # Print first 3 paths (short preview)
    for i in range(min(3, k)):
        print(f"PATH[{i}] len={len(paths[i])}: ", [grid.xy(v) for v in paths[i][:12]], "...")


if __name__ == "__main__":
    demo()
