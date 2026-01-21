"""
mapf_benchmark_runner.py - MAPF Benchmark Runner with Metrics.

Runs MAPF benchmarks on MovingAI instances and publishes all required metrics:
- Makespan
- Sum of Costs (SOC)
- Runtime (ms)
- Nodes expanded (CBS)
- Success rate under fixed budget limits
- Anytime curve (if OMEGA_GAP progressively)

This is the complete benchmarking layer for industry validation.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple, Callable
from pathlib import Path
import json
import time
import statistics

from ..model import (
    Graph,
    MAPFInstance,
    MAPFResult,
    ResultStatus,
    H,
    canon_json
)
from ..cbs import cbs_solve, CBSSolver
from ..verifier import verify_paths, MAPFVerifier
from ..ilp import ilp_feasibility_check, cross_check_cbs_ilp
from .movingai import (
    MovingAIMap,
    MovingAIScenario,
    MovingAIBenchmarkLoader,
    create_test_map,
    create_test_scenario,
    get_standard_test_maps
)
from ..planviz import PlanVizExporter, export_to_planviz


# ============================================================
# BENCHMARK METRICS
# ============================================================

@dataclass
class BenchmarkMetrics:
    """
    Complete metrics for a single benchmark run.

    These are the required metrics from the verification playbook.
    """
    instance_name: str
    num_agents: int
    map_size: Tuple[int, int]

    # Solution metrics
    solved: bool
    status: str
    makespan: int
    sum_of_costs: int

    # Performance metrics
    runtime_ms: int
    nodes_expanded: int

    # Verification
    verifier_passed: bool
    ilp_consistent: Optional[bool]
    receipt: Optional[str]

    # Budget info
    hit_node_limit: bool
    hit_time_limit: bool

    def to_dict(self) -> Dict[str, Any]:
        return {
            "instance": self.instance_name,
            "num_agents": self.num_agents,
            "map_size": list(self.map_size),
            "solved": self.solved,
            "status": self.status,
            "makespan": self.makespan,
            "sum_of_costs": self.sum_of_costs,
            "runtime_ms": self.runtime_ms,
            "nodes_expanded": self.nodes_expanded,
            "verifier_passed": self.verifier_passed,
            "ilp_consistent": self.ilp_consistent,
            "receipt": self.receipt,
            "hit_node_limit": self.hit_node_limit,
            "hit_time_limit": self.hit_time_limit
        }


@dataclass
class BenchmarkSummary:
    """
    Summary statistics for a benchmark set.
    """
    total_instances: int
    solved: int
    unsolved: int
    success_rate: float

    # Aggregate metrics (for solved instances)
    avg_makespan: float
    avg_soc: float
    avg_runtime_ms: float
    avg_nodes: float

    # Distribution
    min_runtime_ms: int
    max_runtime_ms: int
    median_runtime_ms: float

    # Budget hits
    node_limit_hits: int
    time_limit_hits: int

    def to_dict(self) -> Dict[str, Any]:
        return {
            "total_instances": self.total_instances,
            "solved": self.solved,
            "unsolved": self.unsolved,
            "success_rate": self.success_rate,
            "avg_makespan": self.avg_makespan,
            "avg_soc": self.avg_soc,
            "avg_runtime_ms": self.avg_runtime_ms,
            "avg_nodes": self.avg_nodes,
            "min_runtime_ms": self.min_runtime_ms,
            "max_runtime_ms": self.max_runtime_ms,
            "median_runtime_ms": self.median_runtime_ms,
            "node_limit_hits": self.node_limit_hits,
            "time_limit_hits": self.time_limit_hits
        }


def compute_summary(metrics: List[BenchmarkMetrics]) -> BenchmarkSummary:
    """Compute summary statistics from individual metrics."""
    if not metrics:
        return BenchmarkSummary(
            total_instances=0, solved=0, unsolved=0, success_rate=0.0,
            avg_makespan=0.0, avg_soc=0.0, avg_runtime_ms=0.0, avg_nodes=0.0,
            min_runtime_ms=0, max_runtime_ms=0, median_runtime_ms=0.0,
            node_limit_hits=0, time_limit_hits=0
        )

    solved = [m for m in metrics if m.solved]
    unsolved = [m for m in metrics if not m.solved]

    runtimes = [m.runtime_ms for m in metrics]

    return BenchmarkSummary(
        total_instances=len(metrics),
        solved=len(solved),
        unsolved=len(unsolved),
        success_rate=len(solved) / len(metrics) if metrics else 0.0,
        avg_makespan=statistics.mean([m.makespan for m in solved]) if solved else 0.0,
        avg_soc=statistics.mean([m.sum_of_costs for m in solved]) if solved else 0.0,
        avg_runtime_ms=statistics.mean(runtimes) if runtimes else 0.0,
        avg_nodes=statistics.mean([m.nodes_expanded for m in metrics]) if metrics else 0.0,
        min_runtime_ms=min(runtimes) if runtimes else 0,
        max_runtime_ms=max(runtimes) if runtimes else 0,
        median_runtime_ms=statistics.median(runtimes) if runtimes else 0.0,
        node_limit_hits=sum(1 for m in metrics if m.hit_node_limit),
        time_limit_hits=sum(1 for m in metrics if m.hit_time_limit)
    )


# ============================================================
# BENCHMARK RUNNER
# ============================================================

class BenchmarkRunner:
    """
    Complete benchmark runner for MAPF.

    Runs benchmarks on MovingAI instances with all required metrics.
    """

    def __init__(
        self,
        max_time_steps: int = 100,
        max_nodes: int = 10000,
        time_limit_ms: int = 60000,
        do_ilp_crosscheck: bool = True,
        output_dir: Optional[str] = None
    ):
        self.max_time_steps = max_time_steps
        self.max_nodes = max_nodes
        self.time_limit_ms = time_limit_ms
        self.do_ilp_crosscheck = do_ilp_crosscheck
        self.output_dir = output_dir

        if output_dir:
            Path(output_dir).mkdir(parents=True, exist_ok=True)
            self.planviz_exporter = PlanVizExporter(
                str(Path(output_dir) / "planviz")
            )
        else:
            self.planviz_exporter = None

    def run_single(
        self,
        instance: MAPFInstance,
        instance_name: str,
        map_data: MovingAIMap
    ) -> BenchmarkMetrics:
        """
        Run a single benchmark instance.

        Returns complete metrics for the run.
        """
        start_time = time.time()

        # Run CBS
        result = cbs_solve(
            instance,
            max_time=self.max_time_steps,
            max_nodes=self.max_nodes
        )

        runtime_ms = int((time.time() - start_time) * 1000)

        # Check if we hit limits
        hit_node_limit = (
            result.status == ResultStatus.OMEGA_GAP and
            result.nodes_expanded >= self.max_nodes
        )
        hit_time_limit = runtime_ms >= self.time_limit_ms

        # Compute solution metrics
        if result.status == ResultStatus.UNIQUE and result.paths:
            makespan = max(len(p) - 1 for p in result.paths)
            soc = sum(len(p) - 1 for p in result.paths)
            solved = True

            # Verify
            T = makespan
            verify_result = verify_paths(instance, result.paths, T)
            verifier_passed = verify_result.passed
        else:
            makespan = 0
            soc = 0
            solved = False
            verifier_passed = False

        # ILP cross-check
        ilp_consistent = None
        if self.do_ilp_crosscheck and solved:
            cross_check = cross_check_cbs_ilp(instance, result)
            ilp_consistent = cross_check.get("consistent")
            if ilp_consistent == "UNKNOWN":
                ilp_consistent = None

        # Export to PlanViz
        if self.planviz_exporter and solved:
            self.planviz_exporter.export_solution(
                instance_name, instance, result, map_data, runtime_ms
            )

        return BenchmarkMetrics(
            instance_name=instance_name,
            num_agents=instance.num_agents,
            map_size=(map_data.width, map_data.height),
            solved=solved,
            status=result.status.value,
            makespan=makespan,
            sum_of_costs=soc,
            runtime_ms=runtime_ms,
            nodes_expanded=result.nodes_expanded,
            verifier_passed=verifier_passed,
            ilp_consistent=ilp_consistent,
            receipt=result.receipt[:32] if result.receipt else None,
            hit_node_limit=hit_node_limit,
            hit_time_limit=hit_time_limit
        )

    def run_batch(
        self,
        instances: List[Tuple[MAPFInstance, str, MovingAIMap]],
        progress_callback: Optional[Callable[[int, int], None]] = None
    ) -> Tuple[List[BenchmarkMetrics], BenchmarkSummary]:
        """
        Run a batch of benchmark instances.

        Args:
            instances: List of (instance, name, map_data) tuples
            progress_callback: Optional callback(current, total) for progress

        Returns:
            (metrics_list, summary)
        """
        metrics = []
        total = len(instances)

        for i, (instance, name, map_data) in enumerate(instances):
            if progress_callback:
                progress_callback(i + 1, total)

            metric = self.run_single(instance, name, map_data)
            metrics.append(metric)

        summary = compute_summary(metrics)

        return metrics, summary

    def run_scaling_test(
        self,
        map_config: Dict[str, Any],
        agent_counts: List[int],
        seeds_per_count: int = 5
    ) -> Dict[str, Any]:
        """
        Run scaling test with increasing agent counts.

        Returns scaling curve data.
        """
        results = []
        loader = MovingAIBenchmarkLoader()

        for num_agents in agent_counts:
            count_metrics = []

            for seed in range(seeds_per_count):
                try:
                    instance, map_data, scenario = loader.create_test_instance(
                        map_config, num_agents, seed=seed
                    )
                    name = f"{map_config['name']}_{num_agents}agents_seed{seed}"
                    metric = self.run_single(instance, name, map_data)
                    count_metrics.append(metric)
                except ValueError:
                    continue  # Not enough cells

            if count_metrics:
                summary = compute_summary(count_metrics)
                results.append({
                    "num_agents": num_agents,
                    "instances": len(count_metrics),
                    "success_rate": summary.success_rate,
                    "avg_runtime_ms": summary.avg_runtime_ms,
                    "avg_nodes": summary.avg_nodes,
                    "avg_makespan": summary.avg_makespan,
                    "avg_soc": summary.avg_soc
                })

        return {
            "map_config": map_config,
            "agent_counts": agent_counts,
            "seeds_per_count": seeds_per_count,
            "scaling_curve": results
        }


# ============================================================
# STANDARD BENCHMARK SUITE
# ============================================================

def run_standard_benchmarks(
    output_dir: str = "./benchmark_results",
    agent_counts: List[int] = [2, 4, 8, 16],
    do_ilp: bool = True
) -> Dict[str, Any]:
    """
    Run the complete standard benchmark suite.

    This runs all standard test maps with various agent counts.
    """
    runner = BenchmarkRunner(
        max_time_steps=100,
        max_nodes=10000,
        do_ilp_crosscheck=do_ilp,
        output_dir=output_dir
    )

    all_results = {}
    loader = MovingAIBenchmarkLoader()

    print("=" * 60)
    print("MAPF STANDARD BENCHMARK SUITE")
    print("=" * 60)

    for config in get_standard_test_maps():
        map_name = config["name"]
        print(f"\nMap: {map_name}")
        print("-" * 40)

        map_results = []

        for num_agents in agent_counts:
            try:
                instance, map_data, scenario = loader.create_test_instance(
                    config, num_agents, seed=42
                )
                name = f"{map_name}_{num_agents}agents"
                metric = runner.run_single(instance, name, map_data)
                map_results.append(metric)

                status = "SOLVED" if metric.solved else metric.status
                print(f"  {num_agents} agents: {status} "
                      f"(makespan={metric.makespan}, soc={metric.sum_of_costs}, "
                      f"runtime={metric.runtime_ms}ms, nodes={metric.nodes_expanded})")

            except ValueError as e:
                print(f"  {num_agents} agents: SKIPPED ({e})")

        if map_results:
            summary = compute_summary(map_results)
            all_results[map_name] = {
                "config": config,
                "metrics": [m.to_dict() for m in map_results],
                "summary": summary.to_dict()
            }

    # Save results
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    results_file = Path(output_dir) / "benchmark_results.json"
    with open(results_file, 'w') as f:
        json.dump(all_results, f, indent=2)

    print("\n" + "=" * 60)
    print(f"Results saved to: {results_file}")
    print("=" * 60)

    return all_results


# ============================================================
# ANYTIME BENCHMARKING
# ============================================================

@dataclass
class AnytimePoint:
    """Single point in anytime curve."""
    time_ms: int
    nodes_expanded: int
    best_cost: Optional[int]
    status: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "time_ms": self.time_ms,
            "nodes_expanded": self.nodes_expanded,
            "best_cost": self.best_cost,
            "status": self.status
        }


def run_anytime_benchmark(
    instance: MAPFInstance,
    map_data: MovingAIMap,
    checkpoints_ms: List[int] = [100, 500, 1000, 5000, 10000, 30000, 60000]
) -> List[AnytimePoint]:
    """
    Run anytime benchmark with checkpoints.

    Records solution quality at various time points.
    """
    points = []

    for checkpoint_ms in checkpoints_ms:
        # Estimate nodes based on time (rough heuristic)
        estimated_nodes = checkpoint_ms * 10

        start_time = time.time()
        result = cbs_solve(
            instance,
            max_time=100,
            max_nodes=estimated_nodes
        )
        actual_time = int((time.time() - start_time) * 1000)

        if result.status == ResultStatus.UNIQUE and result.paths:
            best_cost = sum(len(p) - 1 for p in result.paths)
        else:
            best_cost = None

        points.append(AnytimePoint(
            time_ms=actual_time,
            nodes_expanded=result.nodes_expanded,
            best_cost=best_cost,
            status=result.status.value
        ))

    return points


# ============================================================
# REPORT GENERATION
# ============================================================

def generate_benchmark_report(
    results: Dict[str, Any],
    output_file: str
) -> str:
    """
    Generate a markdown report from benchmark results.
    """
    lines = [
        "# MAPF Benchmark Report",
        "",
        "## Summary",
        "",
    ]

    total_solved = 0
    total_instances = 0

    for map_name, data in results.items():
        summary = data.get("summary", {})
        total_solved += summary.get("solved", 0)
        total_instances += summary.get("total_instances", 0)

    lines.append(f"- Total instances: {total_instances}")
    lines.append(f"- Total solved: {total_solved}")
    lines.append(f"- Overall success rate: {total_solved/total_instances*100:.1f}%" if total_instances else "- N/A")
    lines.append("")

    lines.append("## Results by Map")
    lines.append("")

    for map_name, data in results.items():
        summary = data.get("summary", {})
        lines.append(f"### {map_name}")
        lines.append("")
        lines.append(f"- Instances: {summary.get('total_instances', 0)}")
        lines.append(f"- Solved: {summary.get('solved', 0)}")
        lines.append(f"- Success rate: {summary.get('success_rate', 0)*100:.1f}%")
        lines.append(f"- Avg makespan: {summary.get('avg_makespan', 0):.1f}")
        lines.append(f"- Avg SOC: {summary.get('avg_soc', 0):.1f}")
        lines.append(f"- Avg runtime: {summary.get('avg_runtime_ms', 0):.1f}ms")
        lines.append(f"- Avg nodes: {summary.get('avg_nodes', 0):.0f}")
        lines.append("")

        # Individual results table
        metrics = data.get("metrics", [])
        if metrics:
            lines.append("| Agents | Status | Makespan | SOC | Runtime (ms) | Nodes |")
            lines.append("|--------|--------|----------|-----|--------------|-------|")
            for m in metrics:
                status = "SOLVED" if m["solved"] else m["status"]
                lines.append(
                    f"| {m['num_agents']} | {status} | {m['makespan']} | "
                    f"{m['sum_of_costs']} | {m['runtime_ms']} | {m['nodes_expanded']} |"
                )
            lines.append("")

    report = '\n'.join(lines)

    with open(output_file, 'w') as f:
        f.write(report)

    return report


# ============================================================
# VERIFICATION INTEGRATION
# ============================================================

def verify_benchmark_results(
    results: Dict[str, Any]
) -> Dict[str, Any]:
    """
    Verify all benchmark results for correctness.

    Checks:
    1. All solved instances have verifier_passed=True
    2. All receipts are present for solved instances
    3. ILP cross-check is consistent
    """
    verification = {
        "all_verified": True,
        "issues": [],
        "statistics": {
            "total_solved": 0,
            "verifier_passed": 0,
            "ilp_consistent": 0,
            "receipts_present": 0
        }
    }

    for map_name, data in results.items():
        for m in data.get("metrics", []):
            if m["solved"]:
                verification["statistics"]["total_solved"] += 1

                if m["verifier_passed"]:
                    verification["statistics"]["verifier_passed"] += 1
                else:
                    verification["all_verified"] = False
                    verification["issues"].append(
                        f"{m['instance']}: Verifier failed on solved instance"
                    )

                if m.get("receipt"):
                    verification["statistics"]["receipts_present"] += 1
                else:
                    verification["issues"].append(
                        f"{m['instance']}: Missing receipt on solved instance"
                    )

                if m.get("ilp_consistent") == True:
                    verification["statistics"]["ilp_consistent"] += 1
                elif m.get("ilp_consistent") == False:
                    verification["all_verified"] = False
                    verification["issues"].append(
                        f"{m['instance']}: ILP cross-check failed"
                    )

    return verification


# ============================================================
# MAIN ENTRY POINT
# ============================================================

def main():
    """Run standard benchmarks from command line."""
    import sys

    output_dir = sys.argv[1] if len(sys.argv) > 1 else "./benchmark_results"

    results = run_standard_benchmarks(output_dir=output_dir)

    # Generate report
    report_file = Path(output_dir) / "benchmark_report.md"
    generate_benchmark_report(results, str(report_file))
    print(f"Report saved to: {report_file}")

    # Verify results
    verification = verify_benchmark_results(results)
    print(f"\nVerification: {'PASSED' if verification['all_verified'] else 'FAILED'}")
    if verification["issues"]:
        for issue in verification["issues"]:
            print(f"  - {issue}")


if __name__ == "__main__":
    main()
