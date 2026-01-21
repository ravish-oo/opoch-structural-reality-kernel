"""
mapf_verifier.py - MAPF Verifier (Truth Gate).

THE VERIFIER IS THE SOURCE OF TRUTH.
CBS, ILP, and all other solvers are proposal mechanisms.
Only the verifier determines validity.

Implements verification checks V1-V5 with padding-before-checking:
- V1 Start: p_i(0) = s_i
- V2 Goal at horizon: p_i(T) = g_i
- V3 Dynamics: edge or wait at each t
- V4 Vertex conflict: no same vertex at same time
- V5 Edge swap: no head-on swaps

On FAIL, returns minimal conflict witness (τ* in kernel terms).
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from .model import (
    Graph,
    MAPFInstance,
    Path,
    Conflict,
    ConflictType,
    VerifierCheck,
    VerifierResult,
    pad_path_to_horizon,
    canon_json,
    sha256_hex,
    H
)


def verify_paths(
    instance: MAPFInstance,
    paths: List[Path],
    horizon: int
) -> VerifierResult:
    """
    VERIFIER: The source of truth.

    Returns PASS or FAIL with minimal conflict witness.
    This function is TOTAL: it always returns a result.

    Checks in order:
    - V1: Start conditions
    - V2: Goal conditions
    - V3: Dynamics (valid moves)
    - V4: Vertex conflicts
    - V5: Edge-swap conflicts

    The first failure found is the minimal separator (τ*).
    """
    k = instance.num_agents
    G = instance.graph
    T = horizon

    # Pad all paths to horizon T (goal-hold convention)
    # THIS MUST HAPPEN BEFORE CHECKING
    padded = []
    for i, p in enumerate(paths):
        if not p:
            return VerifierResult(
                passed=False,
                check=VerifierCheck.V1_START,
                details={"agent": i, "error": "empty_path"}
            )
        padded_path = pad_path_to_horizon(p, T, instance.goals[i])
        padded.append(padded_path)

    # V1: Check start conditions
    # p_i(0) = s_i for all i
    for i in range(k):
        if padded[i][0] != instance.starts[i]:
            return VerifierResult(
                passed=False,
                check=VerifierCheck.V1_START,
                details={
                    "agent": i,
                    "expected": instance.starts[i],
                    "actual": padded[i][0]
                }
            )

    # V2: Check goal conditions
    # p_i(T) = g_i for all i
    for i in range(k):
        if padded[i][T] != instance.goals[i]:
            return VerifierResult(
                passed=False,
                check=VerifierCheck.V2_GOAL,
                details={
                    "agent": i,
                    "expected": instance.goals[i],
                    "actual": padded[i][T]
                }
            )

    # V3: Check dynamics (valid moves)
    # Each step must be a valid edge OR a wait (same vertex)
    for i in range(k):
        for t in range(T):
            u = padded[i][t]
            v = padded[i][t + 1]
            if u != v and not G.is_edge(u, v):
                return VerifierResult(
                    passed=False,
                    check=VerifierCheck.V3_DYNAMICS,
                    details={
                        "agent": i,
                        "time": t,
                        "move": (u, v),
                        "error": "invalid_edge"
                    }
                )

    # V4: Vertex conflicts
    # No two agents at same vertex at same time
    # Deterministic ordering: iterate time steps, then agent pairs
    for t in range(T + 1):
        occupied = {}  # vertex -> agent who first occupied it
        for i in range(k):
            v = padded[i][t]
            if v in occupied:
                j = occupied[v]  # j < i since we iterate in order
                # Return minimal conflict witness (τ*)
                return VerifierResult(
                    passed=False,
                    check=VerifierCheck.V4_VERTEX,
                    conflict=Conflict(
                        conflict_type=ConflictType.VERTEX,
                        time=t,
                        agents=(j, i),
                        vertex=v
                    )
                )
            occupied[v] = i

    # V5: Edge-swap conflicts
    # No two agents swap positions on an edge
    # Deterministic ordering: iterate time steps, then agent pairs (i < j)
    for t in range(T):
        for i in range(k):
            u_i = padded[i][t]
            v_i = padded[i][t + 1]

            # Skip if agent i is waiting (no swap possible)
            if u_i == v_i:
                continue

            for j in range(i + 1, k):
                u_j = padded[j][t]
                v_j = padded[j][t + 1]

                # Skip if agent j is waiting
                if u_j == v_j:
                    continue

                # Check for swap: i goes u_i -> v_i, j goes u_j -> v_j
                # Swap occurs if u_i == v_j AND v_i == u_j
                if u_i == v_j and v_i == u_j:
                    # Store DIRECTED edges per agent
                    return VerifierResult(
                        passed=False,
                        check=VerifierCheck.V5_EDGE_SWAP,
                        conflict=Conflict(
                            conflict_type=ConflictType.EDGE_SWAP,
                            time=t,
                            agents=(i, j),
                            edge_i=(u_i, v_i),  # Agent i's directed edge
                            edge_j=(u_j, v_j)   # Agent j's directed edge
                        )
                    )

    # All checks passed
    return VerifierResult(passed=True)


def get_first_conflict(
    instance: MAPFInstance,
    paths: List[Path],
    horizon: int
) -> Optional[Conflict]:
    """
    Get the first conflict (τ*) under deterministic ordering.

    This is used by CBS to determine which conflict to branch on.
    Returns None if no conflicts (solution is valid).
    """
    result = verify_paths(instance, paths, horizon)
    if result.passed:
        return None
    return result.conflict


class MAPFVerifier:
    """
    Complete MAPF verifier with receipt generation.

    This is the Truth Gate - the single source of truth.
    """

    def __init__(self, instance: MAPFInstance):
        self.instance = instance

    def verify(self, paths: List[Path], horizon: int) -> VerifierResult:
        """Verify paths against the instance."""
        return verify_paths(self.instance, paths, horizon)

    def verify_and_receipt(
        self,
        paths: List[Path],
        horizon: int
    ) -> Tuple[VerifierResult, Optional[str]]:
        """
        Verify paths and generate receipt if valid.

        Returns (result, receipt) where receipt is None if failed.
        """
        result = self.verify(paths, horizon)
        if result.passed:
            cost = sum(len(p) - 1 for p in paths)
            receipt = H({
                "paths": paths,
                "cost": cost,
                "verifier": "PASS"
            })
            return result, receipt
        return result, None

    def is_valid(self, paths: List[Path], horizon: int) -> bool:
        """Quick check if paths are valid."""
        return self.verify(paths, horizon).passed


# ============================================================
# VERIFIER THEOREMS (PROVEN BY CONSTRUCTION)
# ============================================================

"""
THEOREM 3.1: Verifier Soundness
If verify(P) = PASS, then P is a valid MAPF solution.

Proof: The verifier checks exactly the constraints that define validity
(dynamics V1-V3, collisions V4-V5). If all checks pass, all constraints
hold by construction. QED.

THEOREM 3.2: Verifier Completeness
If P is a valid MAPF solution, then verify(P) = PASS.

Proof: A valid solution satisfies all defining constraints. Each verifier
check tests one constraint. Since all constraints hold, no check fails. QED.

THEOREM 3.3: Minimal Separator Property
If verify(P) = FAIL, the returned conflict is a minimal separator witness:
a finite, concrete certificate distinguishing valid from invalid.

It contains type, time, agents, and location. This is τ* in kernel terms.

VERIFICATION COMPLEXITY:
Time:  O(k * T)     for V1-V3 (each agent, each step)
     + O(k * T)     for V4 (hash lookup per agent per step)
     + O(k² * T)    for V5 (agent pairs per step)
     = O(k² * T)    total

Space: O(k * T) for padded paths
"""
