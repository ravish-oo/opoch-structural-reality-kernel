"""
mapf_benchmarks.py - MAPF Benchmark Test Suite.

Implements the 5 mandatory tests from the verification playbook:
1. Grid Swap (2×2) - Tests swap resolution with bypass
2. Corridor Swap (minimal) - Tests edge-swap detection and resolution
3. Bottleneck - Tests coordination through narrow passage
4. Goal Collision - Tests UNSAT detection for shared goals
5. Verifier Soundness - Tests all V1-V5 checks

Each test has:
- Setup: Create instance
- Execute: Run CBS
- Verify: Check result status and correctness
- Cross-check: Verify against ILP (when available)
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import time

from ..model import (
    Graph,
    MAPFInstance,
    Path,
    MAPFResult,
    ResultStatus,
    create_grid_graph,
    create_line_graph,
    create_corridor_with_bypass,
    H,
    canon_json
)
from ..verifier import verify_paths, MAPFVerifier
from ..cbs import cbs_solve, CBSSolver
from .mapf_ilp import ILPOracle, cross_check_cbs_ilp, HAS_PULP


@dataclass
class BenchmarkResult:
    """Result of a benchmark test."""
    name: str
    passed: bool
    expected_status: str
    actual_status: str
    details: Dict[str, Any]
    elapsed_ms: int
    cross_check: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "name": self.name,
            "passed": self.passed,
            "expected_status": self.expected_status,
            "actual_status": self.actual_status,
            "elapsed_ms": self.elapsed_ms,
            "details": self.details
        }
        if self.cross_check is not None:
            d["cross_check"] = self.cross_check
        return d


# ============================================================
# TEST 1: Grid Swap (2×2)
# ============================================================

def test_grid_swap() -> BenchmarkResult:
    """
    Grid Swap test on 2×2 grid.

    Layout:
      0 -- 1
      |    |
      2 -- 3

    Agent 0: 0 → 3 (diagonal)
    Agent 1: 3 → 0 (diagonal opposite)

    Expected: UNIQUE (agents can coordinate using 4-connectivity)
    """
    name = "Grid Swap (2×2)"
    start_time = time.time()

    # Create 2×2 grid
    G = create_grid_graph(2, 2)

    # Agents swap corners
    instance = MAPFInstance(
        graph=G,
        starts=[0, 3],
        goals=[3, 0]
    )

    # Run CBS
    result = cbs_solve(instance, max_time=50, max_nodes=1000)
    elapsed = int((time.time() - start_time) * 1000)

    # Expected: UNIQUE (solution exists)
    passed = result.status == ResultStatus.UNIQUE
    details = {
        "nodes_expanded": result.nodes_expanded,
        "instance_fingerprint": instance.fingerprint()[:16]
    }

    if result.status == ResultStatus.UNIQUE and result.paths:
        details["cost"] = result.cost
        details["makespan"] = max(len(p) - 1 for p in result.paths)
        details["paths"] = result.paths

        # Verify solution
        T = max(len(p) - 1 for p in result.paths)
        verify_result = verify_paths(instance, result.paths, T)
        details["verifier_passed"] = verify_result.passed
        if not verify_result.passed:
            passed = False

    # Cross-check with ILP
    cross_check = cross_check_cbs_ilp(instance, result)
    if cross_check.get("consistent") == False:
        passed = False

    return BenchmarkResult(
        name=name,
        passed=passed,
        expected_status="UNIQUE",
        actual_status=result.status.value,
        details=details,
        elapsed_ms=elapsed,
        cross_check=cross_check
    )


# ============================================================
# TEST 2: Corridor Swap (Minimal)
# ============================================================

def test_corridor_swap() -> BenchmarkResult:
    """
    Corridor Swap test (minimal).

    Layout without bypass:
      0 -- 1 -- 2

    Agent 0: 0 → 2
    Agent 1: 2 → 0

    This is UNSAT without a bypass vertex.

    Layout with bypass at position 1:
      0 -- 1 -- 2
           |
           3 (bypass)

    Agent 0: 0 → 2
    Agent 1: 2 → 0

    Expected: UNIQUE (one agent uses bypass)
    """
    name = "Corridor Swap (with bypass)"
    start_time = time.time()

    # Create corridor with bypass
    G = create_corridor_with_bypass(3, 1)  # Length 3, bypass at position 1

    # Agents swap ends
    instance = MAPFInstance(
        graph=G,
        starts=[0, 2],
        goals=[2, 0]
    )

    # Run CBS
    result = cbs_solve(instance, max_time=50, max_nodes=1000)
    elapsed = int((time.time() - start_time) * 1000)

    # Expected: UNIQUE
    passed = result.status == ResultStatus.UNIQUE
    details = {
        "nodes_expanded": result.nodes_expanded,
        "instance_fingerprint": instance.fingerprint()[:16]
    }

    if result.status == ResultStatus.UNIQUE and result.paths:
        details["cost"] = result.cost
        details["makespan"] = max(len(p) - 1 for p in result.paths)
        details["paths"] = result.paths

        # Verify solution
        T = max(len(p) - 1 for p in result.paths)
        verify_result = verify_paths(instance, result.paths, T)
        details["verifier_passed"] = verify_result.passed
        if not verify_result.passed:
            passed = False

        # Check that bypass is used
        bypass_used = any(3 in path for path in result.paths)
        details["bypass_used"] = bypass_used

    # Cross-check with ILP
    cross_check = cross_check_cbs_ilp(instance, result)
    if cross_check.get("consistent") == False:
        passed = False

    return BenchmarkResult(
        name=name,
        passed=passed,
        expected_status="UNIQUE",
        actual_status=result.status.value,
        details=details,
        elapsed_ms=elapsed,
        cross_check=cross_check
    )


# ============================================================
# TEST 3: Bottleneck
# ============================================================

def test_bottleneck() -> BenchmarkResult:
    """
    Bottleneck test.

    Layout with bypass:
      0 -- 1
      |    |
      2 -- 5 (bypass)
      |
      3 -- 4

    Agent 0: 0 → 4
    Agent 1: 4 → 0

    The bypass at vertex 5 allows agents to pass each other.
    Expected: UNIQUE (agents can coordinate using bypass)
    """
    name = "Bottleneck"
    start_time = time.time()

    # Create bottleneck graph with bypass
    # 0 -- 1
    # |    |
    # 2 -- 5 (bypass)
    # |
    # 3 -- 4
    vertices = [0, 1, 2, 3, 4, 5]
    edges = set()
    # Top part
    edges.add((0, 1))
    edges.add((1, 0))
    # Left column
    edges.add((0, 2))
    edges.add((2, 0))
    edges.add((2, 3))
    edges.add((3, 2))
    edges.add((3, 4))
    edges.add((4, 3))
    # Bypass connections
    edges.add((1, 5))
    edges.add((5, 1))
    edges.add((2, 5))
    edges.add((5, 2))

    G = Graph(vertices=vertices, edges=edges)

    # Agents cross through bottleneck
    instance = MAPFInstance(
        graph=G,
        starts=[0, 4],
        goals=[4, 0]
    )

    # Run CBS
    result = cbs_solve(instance, max_time=50, max_nodes=1000)
    elapsed = int((time.time() - start_time) * 1000)

    # Expected: UNIQUE
    passed = result.status == ResultStatus.UNIQUE
    details = {
        "nodes_expanded": result.nodes_expanded,
        "instance_fingerprint": instance.fingerprint()[:16]
    }

    if result.status == ResultStatus.UNIQUE and result.paths:
        details["cost"] = result.cost
        details["makespan"] = max(len(p) - 1 for p in result.paths)
        details["paths"] = result.paths

        # Verify solution
        T = max(len(p) - 1 for p in result.paths)
        verify_result = verify_paths(instance, result.paths, T)
        details["verifier_passed"] = verify_result.passed
        if not verify_result.passed:
            passed = False

    # Cross-check with ILP
    cross_check = cross_check_cbs_ilp(instance, result)
    if cross_check.get("consistent") == False:
        passed = False

    return BenchmarkResult(
        name=name,
        passed=passed,
        expected_status="UNIQUE",
        actual_status=result.status.value,
        details=details,
        elapsed_ms=elapsed,
        cross_check=cross_check
    )


# ============================================================
# TEST 4: Goal Collision (UNSAT)
# ============================================================

def test_goal_collision() -> BenchmarkResult:
    """
    Goal Collision test (UNSAT).

    Layout:
      0 -- 1 -- 2

    Agent 0: 0 → 1
    Agent 1: 2 → 1

    Both agents have the same goal (vertex 1).
    This is UNSAT due to vertex capacity at the goal.

    Expected: UNSAT with GOAL_COLLISION certificate
    """
    name = "Goal Collision (UNSAT)"
    start_time = time.time()

    # Create line graph
    G = create_line_graph(3)

    # Both agents target vertex 1
    instance = MAPFInstance(
        graph=G,
        starts=[0, 2],
        goals=[1, 1]  # Same goal!
    )

    # Run CBS
    result = cbs_solve(instance, max_time=50, max_nodes=1000)
    elapsed = int((time.time() - start_time) * 1000)

    # Expected: UNSAT
    passed = result.status == ResultStatus.UNSAT
    details = {
        "nodes_expanded": result.nodes_expanded,
        "instance_fingerprint": instance.fingerprint()[:16]
    }

    if result.status == ResultStatus.UNSAT and result.certificate:
        details["certificate_type"] = result.certificate.cert_type
        details["certificate_reason"] = result.certificate.reason
        # Check that it's specifically a GOAL_COLLISION
        if result.certificate.cert_type == "GOAL_COLLISION":
            details["correct_certificate"] = True
        else:
            details["correct_certificate"] = False
            passed = False

    # Cross-check with ILP
    cross_check = cross_check_cbs_ilp(instance, result)
    if cross_check.get("consistent") == False:
        passed = False

    return BenchmarkResult(
        name=name,
        passed=passed,
        expected_status="UNSAT",
        actual_status=result.status.value,
        details=details,
        elapsed_ms=elapsed,
        cross_check=cross_check
    )


# ============================================================
# TEST 5: Verifier Soundness
# ============================================================

def test_verifier_soundness() -> BenchmarkResult:
    """
    Verifier Soundness test.

    Tests all V1-V5 checks by constructing paths that violate each:
    - V1: Wrong start
    - V2: Wrong goal
    - V3: Invalid edge
    - V4: Vertex conflict
    - V5: Edge swap conflict

    Each should be detected and return FAIL with correct check ID.
    """
    name = "Verifier Soundness"
    start_time = time.time()

    # Simple grid for testing
    G = create_grid_graph(3, 3)
    # Vertices: 0 1 2
    #           3 4 5
    #           6 7 8

    instance = MAPFInstance(
        graph=G,
        starts=[0, 8],
        goals=[8, 0]
    )

    verifier = MAPFVerifier(instance)
    all_passed = True
    sub_tests = []

    # Grid layout:
    # 0 -- 1 -- 2
    # |    |    |
    # 3 -- 4 -- 5
    # |    |    |
    # 6 -- 7 -- 8

    # Test V1: Wrong start
    # Agent 0 starts at 1, not 0 (but path uses valid edges)
    paths_v1 = [[1, 4, 7, 8], [8, 5, 2, 0]]
    result_v1 = verifier.verify(paths_v1, 3)
    v1_ok = not result_v1.passed and result_v1.check.value == "V1"
    sub_tests.append({"test": "V1_START", "passed": v1_ok, "check": result_v1.check.value if result_v1.check else None})
    if not v1_ok:
        all_passed = False

    # Test V2: Wrong goal
    # Agent 0 ends at 5, not 8 (valid path, wrong destination)
    paths_v2 = [[0, 1, 4, 5], [8, 5, 2, 1]]
    result_v2 = verifier.verify(paths_v2, 3)
    v2_ok = not result_v2.passed and result_v2.check.value == "V2"
    sub_tests.append({"test": "V2_GOAL", "passed": v2_ok, "check": result_v2.check.value if result_v2.check else None})
    if not v2_ok:
        all_passed = False

    # Test V3: Invalid edge (0 to 8 directly - not connected)
    paths_v3 = [[0, 8, 8, 8], [8, 5, 2, 0]]  # Agent 0 jumps from 0 to 8
    result_v3 = verifier.verify(paths_v3, 3)
    v3_ok = not result_v3.passed and result_v3.check.value == "V3"
    sub_tests.append({"test": "V3_DYNAMICS", "passed": v3_ok, "check": result_v3.check.value if result_v3.check else None})
    if not v3_ok:
        all_passed = False

    # Test V4: Vertex conflict (both at vertex 4 at time 2)
    # Use valid paths that create vertex conflict at 4
    paths_v4 = [[0, 1, 4, 5, 8], [8, 7, 4, 3, 0]]  # Both at 4 at t=2
    result_v4 = verifier.verify(paths_v4, 4)
    v4_ok = not result_v4.passed and result_v4.check.value == "V4"
    sub_tests.append({"test": "V4_VERTEX", "passed": v4_ok, "check": result_v4.check.value if result_v4.check else None})
    if not v4_ok:
        all_passed = False

    # Test V5: Edge swap conflict on edge (3,4) at time 2
    # Agent 0: 0 → 3 → 3(wait) → 4 → 7 → 8  (moves 3→4 at t=2)
    # Agent 1: 8 → 5 → 4 → 3 → 0 → 0        (moves 4→3 at t=2)
    paths_v5 = [[0, 3, 3, 4, 7, 8], [8, 5, 4, 3, 0, 0]]
    result_v5 = verifier.verify(paths_v5, 5)
    v5_ok = not result_v5.passed and result_v5.check.value == "V5"
    sub_tests.append({"test": "V5_EDGE_SWAP", "passed": v5_ok, "check": result_v5.check.value if result_v5.check else None})
    if not v5_ok:
        all_passed = False

    # Test PASS case: valid paths (from CBS)
    cbs_result = cbs_solve(instance, max_time=50, max_nodes=1000)
    if cbs_result.status == ResultStatus.UNIQUE and cbs_result.paths:
        T = max(len(p) - 1 for p in cbs_result.paths)
        result_pass = verifier.verify(cbs_result.paths, T)
        pass_ok = result_pass.passed
        sub_tests.append({"test": "PASS_CASE", "passed": pass_ok})
        if not pass_ok:
            all_passed = False
    else:
        sub_tests.append({"test": "PASS_CASE", "passed": False, "note": "CBS failed"})
        all_passed = False

    elapsed = int((time.time() - start_time) * 1000)

    return BenchmarkResult(
        name=name,
        passed=all_passed,
        expected_status="ALL_CHECKS_CORRECT",
        actual_status="ALL_CHECKS_CORRECT" if all_passed else "SOME_CHECKS_FAILED",
        details={"sub_tests": sub_tests},
        elapsed_ms=elapsed
    )


# ============================================================
# ADDITIONAL BENCHMARKS
# ============================================================

def test_simple_corridor_unsat() -> BenchmarkResult:
    """
    Simple corridor without bypass (UNSAT).

    Layout:
      0 -- 1 -- 2

    Agent 0: 0 → 2
    Agent 1: 2 → 0

    No bypass available, so agents cannot swap.

    Expected: UNSAT (ALL_BRANCHES_PRUNED)
    """
    name = "Corridor No Bypass (UNSAT)"
    start_time = time.time()

    # Create simple line
    G = create_line_graph(3)

    # Agents must swap but can't
    instance = MAPFInstance(
        graph=G,
        starts=[0, 2],
        goals=[2, 0]
    )

    # Run CBS - this is a provably UNSAT instance (agents must pass through
    # vertex 1 but any path causes either vertex conflict or edge swap)
    # CBS may return OMEGA_GAP (resource limit) before proving UNSAT
    result = cbs_solve(instance, max_time=100, max_nodes=2000)
    elapsed = int((time.time() - start_time) * 1000)

    # Expected: UNSAT (or OMEGA_GAP with ILP confirming infeasibility)
    details = {
        "nodes_expanded": result.nodes_expanded,
        "instance_fingerprint": instance.fingerprint()[:16]
    }

    # Directly check with ILP to get ground truth
    from .mapf_ilp import ilp_feasibility_check
    ilp_result = ilp_feasibility_check(instance, horizon=10)
    cross_check = {"ilp_result": ilp_result.to_dict()}

    if result.status == ResultStatus.UNSAT:
        passed = True
        if result.certificate:
            details["certificate_type"] = result.certificate.cert_type
            details["certificate_reason"] = result.certificate.reason
    elif result.status == ResultStatus.OMEGA_GAP:
        # OMEGA_GAP is acceptable if ILP confirms infeasibility
        if ilp_result.feasible == False:
            passed = True
            details["note"] = f"CBS hit resource limit, ILP confirms infeasibility (status: {ilp_result.solver_status})"
        else:
            passed = False
            details["error"] = f"CBS returned OMEGA_GAP, ILP status: {ilp_result.solver_status}"
    else:
        passed = False
        details["error"] = f"Unexpected status: {result.status.value}"

    return BenchmarkResult(
        name=name,
        passed=passed,
        expected_status="UNSAT",
        actual_status=result.status.value,
        details=details,
        elapsed_ms=elapsed,
        cross_check=cross_check
    )


def test_trivial_single_agent() -> BenchmarkResult:
    """
    Trivial single agent test.

    Layout:
      0 -- 1 -- 2

    Agent 0: 0 → 2

    Expected: UNIQUE with optimal path [0, 1, 2]
    """
    name = "Single Agent Trivial"
    start_time = time.time()

    G = create_line_graph(3)
    instance = MAPFInstance(
        graph=G,
        starts=[0],
        goals=[2]
    )

    result = cbs_solve(instance, max_time=50, max_nodes=1000)
    elapsed = int((time.time() - start_time) * 1000)

    passed = result.status == ResultStatus.UNIQUE
    details = {
        "nodes_expanded": result.nodes_expanded
    }

    if result.status == ResultStatus.UNIQUE and result.paths:
        details["paths"] = result.paths
        details["cost"] = result.cost
        # Optimal path should be [0, 1, 2] with cost 2
        optimal = result.paths[0] == [0, 1, 2] and result.cost == 2
        details["optimal"] = optimal
        if not optimal:
            passed = False

    return BenchmarkResult(
        name=name,
        passed=passed,
        expected_status="UNIQUE",
        actual_status=result.status.value,
        details=details,
        elapsed_ms=elapsed
    )


# ============================================================
# BENCHMARK RUNNER
# ============================================================

def run_all_benchmarks() -> Dict[str, Any]:
    """
    Run all benchmark tests.

    Returns summary with all results.
    """
    benchmarks = [
        test_grid_swap,
        test_corridor_swap,
        test_bottleneck,
        test_goal_collision,
        test_verifier_soundness,
        test_simple_corridor_unsat,
        test_trivial_single_agent
    ]

    results = []
    total_passed = 0
    total_elapsed = 0

    for benchmark in benchmarks:
        result = benchmark()
        results.append(result.to_dict())
        if result.passed:
            total_passed += 1
        total_elapsed += result.elapsed_ms

    return {
        "total_tests": len(benchmarks),
        "passed": total_passed,
        "failed": len(benchmarks) - total_passed,
        "total_elapsed_ms": total_elapsed,
        "ilp_available": HAS_PULP,
        "results": results
    }


def run_mandatory_benchmarks() -> Dict[str, Any]:
    """
    Run the 5 mandatory benchmark tests.

    These are the required tests from the verification playbook.
    """
    mandatory = [
        test_grid_swap,
        test_corridor_swap,
        test_bottleneck,
        test_goal_collision,
        test_verifier_soundness
    ]

    results = []
    total_passed = 0

    for benchmark in mandatory:
        result = benchmark()
        results.append(result.to_dict())
        if result.passed:
            total_passed += 1

    return {
        "total_mandatory": len(mandatory),
        "passed": total_passed,
        "all_mandatory_passed": total_passed == len(mandatory),
        "results": results
    }


class MAPFBenchmarkSuite:
    """
    MAPF Benchmark Suite wrapper.

    Provides methods to run benchmarks and generate reports.
    """

    def __init__(self):
        self.mandatory_tests = [
            test_grid_swap,
            test_corridor_swap,
            test_bottleneck,
            test_goal_collision,
            test_verifier_soundness
        ]
        self.additional_tests = [
            test_simple_corridor_unsat,
            test_trivial_single_agent
        ]

    def run_mandatory(self) -> Dict[str, Any]:
        """Run mandatory tests only."""
        return run_mandatory_benchmarks()

    def run_all(self) -> Dict[str, Any]:
        """Run all tests."""
        return run_all_benchmarks()

    def run_single(self, test_name: str) -> Optional[BenchmarkResult]:
        """Run a single test by name."""
        test_map = {
            "grid_swap": test_grid_swap,
            "corridor_swap": test_corridor_swap,
            "bottleneck": test_bottleneck,
            "goal_collision": test_goal_collision,
            "verifier_soundness": test_verifier_soundness,
            "corridor_unsat": test_simple_corridor_unsat,
            "single_agent": test_trivial_single_agent
        }
        if test_name in test_map:
            return test_map[test_name]()
        return None


# ============================================================
# BENCHMARK THEOREMS
# ============================================================

"""
Each benchmark test validates specific MAPF properties:

TEST 1 (Grid Swap):
- Validates CBS can find solutions requiring coordination
- Tests both vertex and edge-swap conflict resolution

TEST 2 (Corridor Swap):
- Validates bypass detection and usage
- Tests edge-swap branching in CBS

TEST 3 (Bottleneck):
- Validates sequencing through narrow passages
- Tests constraint propagation effectiveness

TEST 4 (Goal Collision):
- Validates UNSAT detection for impossible instances
- Tests early termination for goal conflicts

TEST 5 (Verifier Soundness):
- Validates all V1-V5 checks function correctly
- Tests that each constraint violation is caught

ACCEPTANCE CRITERION:
All 5 mandatory tests must PASS for implementation acceptance.
"""
