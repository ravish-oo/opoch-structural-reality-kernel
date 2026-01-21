"""
mapf_ilp.py - ILP Feasibility Oracle for MAPF.

This implements the time-expanded network ILP formulation for MAPF.
Used as a cross-check against CBS to verify the equivalence theorem:

    ILP feasible with horizon T ⟺ MAPF feasible with makespan ≤ T

The ILP is NOT used for optimization (CBS does that).
It serves as an independent feasibility oracle.

Formulation:
- Binary variables x[i,v,t] = 1 iff agent i at vertex v at time t
- Binary variables y[i,e,t] = 1 iff agent i traverses edge e at time t
- Constraints:
  * Start: x[i,s_i,0] = 1
  * Goal: x[i,g_i,T] = 1
  * Flow conservation: at each (v,t), sum of in-flows = sum of out-flows
  * Vertex capacity: sum_i x[i,v,t] ≤ 1 for all (v,t)
  * Edge swap: y[i,(u,v),t] + y[j,(v,u),t] ≤ 1 for all edges and pairs

If ILP solver unavailable, falls back to constraint propagation check.
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set, Tuple
from collections import defaultdict

from .model import (
    Graph,
    MAPFInstance,
    Path,
    MAPFResult,
    ResultStatus,
    UNSATCertificate,
    GapInfo,
    FrontierWitness,
    H,
    canon_json
)
from .verifier import verify_paths

# Try to import PuLP for ILP solving
try:
    import pulp
    HAS_PULP = True
except ImportError:
    HAS_PULP = False


# ============================================================
# ILP FORMULATION (TIME-EXPANDED NETWORK)
# ============================================================

@dataclass
class ILPResult:
    """Result of ILP feasibility check."""
    feasible: bool
    paths: Optional[List[Path]] = None
    solver_status: str = ""
    solve_time_ms: Optional[int] = None
    variables_count: int = 0
    constraints_count: int = 0

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "feasible": self.feasible,
            "solver_status": self.solver_status,
            "variables_count": self.variables_count,
            "constraints_count": self.constraints_count
        }
        if self.paths is not None:
            d["paths"] = self.paths
        if self.solve_time_ms is not None:
            d["solve_time_ms"] = self.solve_time_ms
        return d


def build_ilp_model(instance: MAPFInstance, horizon: int):
    """
    Build ILP model for MAPF feasibility.

    Uses time-expanded network formulation.
    """
    if not HAS_PULP:
        raise RuntimeError("PuLP not available for ILP solving")

    k = instance.num_agents
    G = instance.graph
    V = G.vertices
    T = horizon

    # Create ILP problem
    prob = pulp.LpProblem("MAPF_Feasibility", pulp.LpMinimize)

    # Variables: x[i,v,t] = 1 iff agent i at vertex v at time t
    x = {}
    for i in range(k):
        for v in V:
            for t in range(T + 1):
                x[i, v, t] = pulp.LpVariable(f"x_{i}_{v}_{t}", cat=pulp.LpBinary)

    # Variables: y[i,u,v,t] = 1 iff agent i moves from u to v at time t
    y = {}
    for i in range(k):
        for (u, v) in G.edges:
            for t in range(T):
                y[i, u, v, t] = pulp.LpVariable(f"y_{i}_{u}_{v}_{t}", cat=pulp.LpBinary)
        # Wait actions: self-loops
        for v in V:
            for t in range(T):
                y[i, v, v, t] = pulp.LpVariable(f"y_{i}_{v}_{v}_{t}", cat=pulp.LpBinary)

    # Objective: minimize sum of costs (optional, feasibility check)
    prob += 0  # Feasibility only

    # Constraint 1: Start conditions
    # x[i, s_i, 0] = 1 and x[i, v, 0] = 0 for v != s_i
    for i in range(k):
        s_i = instance.starts[i]
        prob += x[i, s_i, 0] == 1, f"start_{i}"
        for v in V:
            if v != s_i:
                prob += x[i, v, 0] == 0, f"not_start_{i}_{v}"

    # Constraint 2: Goal conditions
    # x[i, g_i, T] = 1
    for i in range(k):
        g_i = instance.goals[i]
        prob += x[i, g_i, T] == 1, f"goal_{i}"

    # Constraint 3: Flow conservation
    # At each (i, v, t), agent must come from somewhere and go somewhere
    for i in range(k):
        for v in V:
            for t in range(1, T + 1):
                # Inflows to (v, t): edges ending at v, or wait at v
                inflows = []
                for u in G.neighbors(v):
                    if (u, v) in G.edges:
                        inflows.append(y[i, u, v, t-1])
                # Self-loop (wait)
                inflows.append(y[i, v, v, t-1])
                # Also edges from other vertices
                for (u, vv) in G.edges:
                    if vv == v and (i, u, v, t-1) in y and y[i, u, v, t-1] not in inflows:
                        inflows.append(y[i, u, v, t-1])

                # x[i,v,t] = sum of inflows
                prob += x[i, v, t] == pulp.lpSum(inflows), f"flow_in_{i}_{v}_{t}"

    # Constraint 4: Outflow conservation
    # At each (i, v, t) for t < T, if agent is there, must go somewhere
    for i in range(k):
        for v in V:
            for t in range(T):
                # Outflows from (v, t)
                outflows = []
                for u in G.neighbors(v):
                    if (v, u) in G.edges and (i, v, u, t) in y:
                        outflows.append(y[i, v, u, t])
                # Self-loop (wait)
                outflows.append(y[i, v, v, t])

                # x[i,v,t] = sum of outflows
                prob += x[i, v, t] == pulp.lpSum(outflows), f"flow_out_{i}_{v}_{t}"

    # Constraint 5: Vertex capacity (no vertex conflicts)
    # sum_i x[i,v,t] <= 1 for all (v, t)
    for v in V:
        for t in range(T + 1):
            prob += pulp.lpSum(x[i, v, t] for i in range(k)) <= 1, f"vertex_cap_{v}_{t}"

    # Constraint 6: Edge swap capacity (no edge-swap conflicts)
    # y[i,u,v,t] + y[j,v,u,t] <= 1 for all edges (u,v), pairs i<j, times t
    for (u, v) in G.edges:
        if (v, u) in G.edges:  # Only for undirected edges
            for t in range(T):
                for i in range(k):
                    for j in range(i + 1, k):
                        if (i, u, v, t) in y and (j, v, u, t) in y:
                            prob += y[i, u, v, t] + y[j, v, u, t] <= 1, f"swap_{u}_{v}_{t}_{i}_{j}"

    return prob, x, y


def solve_ilp(instance: MAPFInstance, horizon: int) -> ILPResult:
    """
    Solve ILP for MAPF feasibility.

    Returns feasibility status and extracted paths if feasible.
    """
    if not HAS_PULP:
        return ILPResult(
            feasible=False,
            solver_status="NO_SOLVER",
            variables_count=0,
            constraints_count=0
        )

    try:
        prob, x, y = build_ilp_model(instance, horizon)

        # Count variables and constraints
        vars_count = len(x) + len(y)
        constraints_count = len(prob.constraints)

        # Solve
        import time
        start_time = time.time()
        status = prob.solve(pulp.PULP_CBC_CMD(msg=0))
        solve_time = int((time.time() - start_time) * 1000)

        if pulp.LpStatus[status] == "Optimal":
            # Extract paths from solution
            k = instance.num_agents
            T = horizon
            paths = []

            for i in range(k):
                path = []
                for t in range(T + 1):
                    for v in instance.graph.vertices:
                        if pulp.value(x[i, v, t]) > 0.5:
                            path.append(v)
                            break
                paths.append(path)

            return ILPResult(
                feasible=True,
                paths=paths,
                solver_status="OPTIMAL",
                solve_time_ms=solve_time,
                variables_count=vars_count,
                constraints_count=constraints_count
            )
        else:
            return ILPResult(
                feasible=False,
                solver_status=pulp.LpStatus[status],
                solve_time_ms=solve_time,
                variables_count=vars_count,
                constraints_count=constraints_count
            )

    except Exception as e:
        return ILPResult(
            feasible=False,
            solver_status=f"ERROR: {str(e)}",
            variables_count=0,
            constraints_count=0
        )


# ============================================================
# CONSTRAINT PROPAGATION FALLBACK
# ============================================================

def constraint_propagation_check(instance: MAPFInstance, horizon: int) -> ILPResult:
    """
    Fallback feasibility check using constraint propagation.

    This is less powerful than ILP but doesn't require external solver.
    Uses forward reachability analysis with conflict detection.
    """
    k = instance.num_agents
    G = instance.graph
    T = horizon

    # Check basic reachability first
    for i in range(k):
        dist = bfs_distance(G, instance.starts[i], instance.goals[i])
        if dist is None or dist > T:
            return ILPResult(
                feasible=False,
                solver_status="UNREACHABLE",
                variables_count=0,
                constraints_count=0
            )

    # Check for goal collisions (immediate UNSAT)
    goal_count = defaultdict(list)
    for i, g in enumerate(instance.goals):
        goal_count[g].append(i)

    for g, agents in goal_count.items():
        if len(agents) > 1:
            return ILPResult(
                feasible=False,
                solver_status="GOAL_COLLISION",
                variables_count=0,
                constraints_count=0
            )

    # For small instances, try exhaustive search
    if k <= 3 and len(G.vertices) <= 10 and T <= 10:
        result = exhaustive_search(instance, T)
        if result is not None:
            return ILPResult(
                feasible=True,
                paths=result,
                solver_status="EXHAUSTIVE_SEARCH",
                variables_count=0,
                constraints_count=0
            )
        else:
            return ILPResult(
                feasible=False,
                solver_status="EXHAUSTIVE_INFEASIBLE",
                variables_count=0,
                constraints_count=0
            )

    # Otherwise, return UNKNOWN
    return ILPResult(
        feasible=False,
        solver_status="UNKNOWN_NO_SOLVER",
        variables_count=0,
        constraints_count=0
    )


def bfs_distance(graph: Graph, start: int, goal: int) -> Optional[int]:
    """BFS to find shortest path distance."""
    if start == goal:
        return 0

    visited = {start}
    queue = [(start, 0)]

    while queue:
        v, d = queue.pop(0)
        for u in graph.neighbors(v):
            if u == goal:
                return d + 1
            if u not in visited:
                visited.add(u)
                queue.append((u, d + 1))

    return None


def exhaustive_search(
    instance: MAPFInstance,
    horizon: int
) -> Optional[List[Path]]:
    """
    Exhaustive search for small instances.

    Uses iterative deepening DFS on joint state space.
    """
    k = instance.num_agents
    G = instance.graph

    # Joint state: tuple of (agent_positions)
    start_state = tuple(instance.starts)
    goal_state = tuple(instance.goals)

    def get_successors(state: Tuple[int, ...]) -> List[Tuple[int, ...]]:
        """Generate all valid successor states."""
        # Generate all possible moves for each agent
        moves_per_agent = []
        for i in range(k):
            v = state[i]
            # Can wait or move to any neighbor
            possible = [v] + list(G.neighbors(v))
            moves_per_agent.append(possible)

        # Cartesian product of moves
        from itertools import product
        all_moves = list(product(*moves_per_agent))

        # Filter out conflicts
        valid = []
        for next_state in all_moves:
            # Check vertex conflicts
            if len(set(next_state)) < k:
                continue  # Same vertex

            # Check edge swaps
            swap_found = False
            for i in range(k):
                for j in range(i + 1, k):
                    u_i, v_i = state[i], next_state[i]
                    u_j, v_j = state[j], next_state[j]
                    if u_i == v_j and v_i == u_j and u_i != v_i:
                        swap_found = True
                        break
                if swap_found:
                    break

            if not swap_found:
                valid.append(next_state)

        return valid

    # BFS on joint state space
    visited = {start_state: [start_state]}
    queue = [(start_state, 0)]

    while queue:
        state, t = queue.pop(0)

        if state == goal_state:
            # Reconstruct paths
            path_states = visited[state]
            paths = []
            for i in range(k):
                paths.append([s[i] for s in path_states])
            return paths

        if t >= horizon:
            continue

        for next_state in get_successors(state):
            if next_state not in visited:
                visited[next_state] = visited[state] + [next_state]
                queue.append((next_state, t + 1))

    return None


# ============================================================
# MAIN ILP ORACLE
# ============================================================

def ilp_feasibility_check(
    instance: MAPFInstance,
    horizon: int
) -> ILPResult:
    """
    ILP feasibility oracle.

    Uses PuLP if available, otherwise falls back to constraint propagation.
    """
    if HAS_PULP:
        return solve_ilp(instance, horizon)
    else:
        return constraint_propagation_check(instance, horizon)


def cross_check_cbs_ilp(
    instance: MAPFInstance,
    cbs_result: MAPFResult,
    horizon: Optional[int] = None
) -> Dict[str, Any]:
    """
    Cross-check CBS result against ILP oracle.

    Verifies the equivalence theorem:
    - CBS UNIQUE with makespan M → ILP feasible with horizon M
    - CBS UNSAT → ILP infeasible for all reasonable horizons
    - CBS OMEGA_GAP → cannot conclude (resource limit)

    Returns cross-check result with consistency flag.
    """
    result = {
        "cbs_status": cbs_result.status.value,
        "cross_check_performed": False,
        "consistent": None,
        "ilp_result": None
    }

    if cbs_result.status == ResultStatus.UNIQUE:
        # CBS found solution - ILP should be feasible with same horizon
        if cbs_result.paths is None:
            result["error"] = "CBS UNIQUE but no paths"
            return result

        makespan = max(len(p) - 1 for p in cbs_result.paths)
        H = horizon if horizon is not None else makespan

        ilp_result = ilp_feasibility_check(instance, H)
        result["cross_check_performed"] = True
        result["ilp_result"] = ilp_result.to_dict()
        result["horizon_used"] = H

        if ilp_result.solver_status in ["NO_SOLVER", "UNKNOWN_NO_SOLVER"]:
            result["consistent"] = "UNKNOWN"
            result["note"] = "ILP solver unavailable"
        else:
            result["consistent"] = ilp_result.feasible
            if not ilp_result.feasible:
                result["error"] = "CBS found solution but ILP infeasible - INCONSISTENCY"

    elif cbs_result.status == ResultStatus.UNSAT:
        # CBS proved infeasible - ILP should be infeasible
        # Test with generous horizon
        H = horizon if horizon is not None else len(instance.graph.vertices) * 2

        ilp_result = ilp_feasibility_check(instance, H)
        result["cross_check_performed"] = True
        result["ilp_result"] = ilp_result.to_dict()
        result["horizon_used"] = H

        if ilp_result.solver_status in ["NO_SOLVER", "UNKNOWN_NO_SOLVER"]:
            result["consistent"] = "UNKNOWN"
            result["note"] = "ILP solver unavailable"
        else:
            result["consistent"] = not ilp_result.feasible
            if ilp_result.feasible:
                result["error"] = "CBS returned UNSAT but ILP feasible - INCONSISTENCY"

    else:  # OMEGA_GAP
        result["consistent"] = "N/A"
        result["note"] = "CBS returned OMEGA_GAP - resource limit, cannot cross-check"

    return result


class ILPOracle:
    """
    ILP Feasibility Oracle wrapper.

    Provides independent verification of MAPF feasibility.
    """

    def __init__(self, instance: MAPFInstance):
        self.instance = instance

    def check_feasibility(self, horizon: int) -> ILPResult:
        """Check if instance is feasible with given horizon."""
        return ilp_feasibility_check(self.instance, horizon)

    def cross_check_with_cbs(
        self,
        cbs_result: MAPFResult,
        horizon: Optional[int] = None
    ) -> Dict[str, Any]:
        """Cross-check CBS result for consistency."""
        return cross_check_cbs_ilp(self.instance, cbs_result, horizon)

    def solver_available(self) -> bool:
        """Check if ILP solver is available."""
        return HAS_PULP


# ============================================================
# ILP THEOREMS (EQUIVALENCE)
# ============================================================

"""
THEOREM 6.1: ILP-MAPF Equivalence
For a MAPF instance I and horizon T:
  ILP(I, T) feasible ⟺ ∃ valid MAPF solution with makespan ≤ T

Proof:
(→) If ILP feasible, the binary solution defines paths for each agent.
    By construction, ILP constraints enforce:
    - Start/goal conditions (MAPF requirements)
    - Flow conservation (valid paths)
    - Vertex capacity (no vertex conflicts)
    - Edge swap capacity (no edge-swap conflicts)
    Hence the extracted paths form a valid MAPF solution.

(←) If valid solution S exists with makespan ≤ T, define:
    - x[i,v,t] = 1 iff agent i at vertex v at time t in S
    - y[i,u,v,t] = 1 iff agent i moves from u to v at time t in S
    Since S is valid, all ILP constraints are satisfied.
    Hence ILP is feasible.
QED.

THEOREM 6.2: ILP as Independent Oracle
ILP provides an independent feasibility check:
- Uses different algorithm than CBS (MIP solver vs tree search)
- Same constraints but different encoding
- Cross-check verifies both are correct

If CBS and ILP disagree on feasibility, one has a bug.
This is the validation principle: independent implementations must agree.
"""
