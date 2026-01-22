"""
Metrics for Transport Fabric

Comprehensive metrics for 10k-robot warehouse operations:
- Throughput (tasks/tick, robots moved/tick)
- Latency (task completion time)
- Congestion (queue lengths, blocked robots)
- Utilization (active vs idle robots)
- Proof metrics (verification pass rate)
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional
from collections import defaultdict
import statistics


@dataclass
class ThroughputMetrics:
    """Throughput metrics for a time window."""
    tasks_completed: int
    robots_moved: int
    ticks: int

    @property
    def tasks_per_tick(self) -> float:
        return self.tasks_completed / self.ticks if self.ticks > 0 else 0

    @property
    def moves_per_tick(self) -> float:
        return self.robots_moved / self.ticks if self.ticks > 0 else 0

    def to_dict(self) -> Dict:
        return {
            "tasks_completed": self.tasks_completed,
            "robots_moved": self.robots_moved,
            "ticks": self.ticks,
            "tasks_per_tick": self.tasks_per_tick,
            "moves_per_tick": self.moves_per_tick
        }


@dataclass
class LatencyMetrics:
    """Task latency metrics."""
    latencies: List[int]

    @property
    def mean(self) -> float:
        return statistics.mean(self.latencies) if self.latencies else 0

    @property
    def median(self) -> float:
        return statistics.median(self.latencies) if self.latencies else 0

    @property
    def p95(self) -> float:
        if not self.latencies:
            return 0
        sorted_lat = sorted(self.latencies)
        idx = int(0.95 * len(sorted_lat))
        return sorted_lat[min(idx, len(sorted_lat) - 1)]

    @property
    def p99(self) -> float:
        if not self.latencies:
            return 0
        sorted_lat = sorted(self.latencies)
        idx = int(0.99 * len(sorted_lat))
        return sorted_lat[min(idx, len(sorted_lat) - 1)]

    def to_dict(self) -> Dict:
        return {
            "count": len(self.latencies),
            "mean": self.mean,
            "median": self.median,
            "p95": self.p95,
            "p99": self.p99,
            "min": min(self.latencies) if self.latencies else 0,
            "max": max(self.latencies) if self.latencies else 0
        }


@dataclass
class CongestionMetrics:
    """Congestion metrics at a point in time."""
    queue_lengths: Dict[int, int]  # vertex → queue length
    blocked_robots: int
    total_queue: int

    @property
    def max_queue(self) -> int:
        return max(self.queue_lengths.values()) if self.queue_lengths else 0

    @property
    def congested_vertices(self) -> int:
        return sum(1 for q in self.queue_lengths.values() if q > 3)

    def to_dict(self) -> Dict:
        return {
            "total_queue": self.total_queue,
            "max_queue": self.max_queue,
            "blocked_robots": self.blocked_robots,
            "congested_vertices": self.congested_vertices,
            "num_vertices_with_queue": len([q for q in self.queue_lengths.values() if q > 0])
        }


@dataclass
class CongestionMap:
    """Spatial congestion map."""
    vertex_congestion: Dict[int, float]  # vertex → average queue over time

    def hotspots(self, threshold: float = 2.0) -> List[int]:
        """Get congestion hotspots."""
        return [v for v, c in self.vertex_congestion.items() if c > threshold]

    def to_dict(self) -> Dict:
        return {
            "num_vertices": len(self.vertex_congestion),
            "hotspots": len(self.hotspots()),
            "max_congestion": max(self.vertex_congestion.values()) if self.vertex_congestion else 0,
            "avg_congestion": sum(self.vertex_congestion.values()) / len(self.vertex_congestion) if self.vertex_congestion else 0
        }


@dataclass
class UtilizationMetrics:
    """Robot utilization metrics."""
    active_robots: int      # Currently moving
    idle_robots: int        # Waiting
    total_robots: int
    active_ratio_history: List[float]

    @property
    def utilization(self) -> float:
        return self.active_robots / self.total_robots if self.total_robots > 0 else 0

    @property
    def avg_utilization(self) -> float:
        return statistics.mean(self.active_ratio_history) if self.active_ratio_history else 0

    def to_dict(self) -> Dict:
        return {
            "active": self.active_robots,
            "idle": self.idle_robots,
            "total": self.total_robots,
            "utilization": self.utilization,
            "avg_utilization": self.avg_utilization
        }


@dataclass
class ProofMetrics:
    """Proof verification metrics."""
    ticks_verified: int
    all_passed: bool
    gate_pass_rates: Dict[str, float]  # gate → pass rate

    def to_dict(self) -> Dict:
        return {
            "ticks_verified": self.ticks_verified,
            "all_passed": self.all_passed,
            "gate_pass_rates": self.gate_pass_rates
        }


@dataclass
class ThroughputAnalysis:
    """Complete throughput analysis."""
    instantaneous: ThroughputMetrics
    rolling_avg: ThroughputMetrics
    peak: ThroughputMetrics
    theoretical_max: float

    @property
    def efficiency(self) -> float:
        """Ratio of actual to theoretical throughput."""
        if self.theoretical_max == 0:
            return 0
        return self.rolling_avg.tasks_per_tick / self.theoretical_max

    def to_dict(self) -> Dict:
        return {
            "instantaneous": self.instantaneous.to_dict(),
            "rolling_avg": self.rolling_avg.to_dict(),
            "peak": self.peak.to_dict(),
            "theoretical_max": self.theoretical_max,
            "efficiency": self.efficiency
        }


class FabricMetrics:
    """
    Comprehensive metrics collector for transport fabric.

    Tracks all metrics during simulation and provides analysis.
    """

    def __init__(self, num_robots: int, num_vertices: int):
        """
        Initialize metrics collector.

        Args:
            num_robots: Total number of robots
            num_vertices: Number of vertices in fabric
        """
        self.num_robots = num_robots
        self.num_vertices = num_vertices

        # Time series
        self.moves_per_tick: List[int] = []
        self.tasks_completed_per_tick: List[int] = []
        self.queue_lengths_per_tick: List[int] = []
        self.active_robots_per_tick: List[int] = []

        # Latencies
        self.task_latencies: List[int] = []

        # Congestion tracking
        self.vertex_queue_sums: Dict[int, int] = defaultdict(int)
        self.vertex_queue_counts: Dict[int, int] = defaultdict(int)

        # Proof tracking
        self.verification_results: List[bool] = []
        self.gate_results: Dict[str, List[bool]] = defaultdict(list)

        # Counters
        self.total_moves = 0
        self.total_tasks_completed = 0
        self.current_tick = 0

    def record_tick(self, moves: int, tasks_completed: int,
                    queue_state: Optional[Dict[int, int]] = None,
                    active_robots: int = 0):
        """
        Record metrics for a single tick.

        Args:
            moves: Number of robot moves this tick
            tasks_completed: Tasks completed this tick
            queue_state: {vertex: queue_length}
            active_robots: Number of robots that moved
        """
        self.moves_per_tick.append(moves)
        self.tasks_completed_per_tick.append(tasks_completed)
        self.active_robots_per_tick.append(active_robots)

        self.total_moves += moves
        self.total_tasks_completed += tasks_completed
        self.current_tick += 1

        # Record queue state
        if queue_state:
            total_queue = sum(queue_state.values())
            self.queue_lengths_per_tick.append(total_queue)

            for v, q in queue_state.items():
                self.vertex_queue_sums[v] += q
                self.vertex_queue_counts[v] += 1

    def record_task_completion(self, latency: int):
        """Record a task completion latency."""
        self.task_latencies.append(latency)

    def record_verification(self, passed: bool, gates: Dict[str, bool]):
        """Record tick verification result."""
        self.verification_results.append(passed)
        for gate, result in gates.items():
            self.gate_results[gate].append(result)

    def get_throughput(self, window: int = 100) -> ThroughputMetrics:
        """Get throughput metrics for last N ticks."""
        recent_moves = self.moves_per_tick[-window:] if self.moves_per_tick else []
        recent_tasks = self.tasks_completed_per_tick[-window:] if self.tasks_completed_per_tick else []

        return ThroughputMetrics(
            tasks_completed=sum(recent_tasks),
            robots_moved=sum(recent_moves),
            ticks=len(recent_moves)
        )

    def get_latency(self) -> LatencyMetrics:
        """Get task latency metrics."""
        return LatencyMetrics(latencies=self.task_latencies)

    def get_congestion(self) -> CongestionMetrics:
        """Get current congestion metrics."""
        queue_lengths = {}
        for v in self.vertex_queue_sums:
            if self.vertex_queue_counts[v] > 0:
                queue_lengths[v] = self.vertex_queue_sums[v] // self.vertex_queue_counts[v]

        total_queue = self.queue_lengths_per_tick[-1] if self.queue_lengths_per_tick else 0

        return CongestionMetrics(
            queue_lengths=queue_lengths,
            blocked_robots=0,  # Would need more tracking
            total_queue=total_queue
        )

    def get_congestion_map(self) -> CongestionMap:
        """Get spatial congestion map."""
        avg_congestion = {}
        for v in self.vertex_queue_sums:
            if self.vertex_queue_counts[v] > 0:
                avg_congestion[v] = self.vertex_queue_sums[v] / self.vertex_queue_counts[v]
        return CongestionMap(vertex_congestion=avg_congestion)

    def get_utilization(self) -> UtilizationMetrics:
        """Get utilization metrics."""
        active = self.active_robots_per_tick[-1] if self.active_robots_per_tick else 0
        history = [a / self.num_robots for a in self.active_robots_per_tick] if self.active_robots_per_tick else []

        return UtilizationMetrics(
            active_robots=active,
            idle_robots=self.num_robots - active,
            total_robots=self.num_robots,
            active_ratio_history=history
        )

    def get_proof_metrics(self) -> ProofMetrics:
        """Get proof verification metrics."""
        gate_rates = {}
        for gate, results in self.gate_results.items():
            if results:
                gate_rates[gate] = sum(results) / len(results)

        return ProofMetrics(
            ticks_verified=len(self.verification_results),
            all_passed=all(self.verification_results) if self.verification_results else True,
            gate_pass_rates=gate_rates
        )

    def get_throughput_analysis(self, theoretical_max: float = 0) -> ThroughputAnalysis:
        """Get complete throughput analysis."""
        instantaneous = self.get_throughput(window=1)
        rolling = self.get_throughput(window=100)

        # Find peak
        peak_moves = max(self.moves_per_tick) if self.moves_per_tick else 0
        peak_tasks = max(self.tasks_completed_per_tick) if self.tasks_completed_per_tick else 0
        peak = ThroughputMetrics(peak_tasks, peak_moves, 1)

        return ThroughputAnalysis(
            instantaneous=instantaneous,
            rolling_avg=rolling,
            peak=peak,
            theoretical_max=theoretical_max
        )

    def summary(self) -> Dict:
        """Get complete metrics summary."""
        return {
            "run_info": {
                "num_robots": self.num_robots,
                "num_vertices": self.num_vertices,
                "total_ticks": self.current_tick
            },
            "throughput": self.get_throughput().to_dict(),
            "latency": self.get_latency().to_dict(),
            "congestion": self.get_congestion().to_dict(),
            "utilization": self.get_utilization().to_dict(),
            "proof": self.get_proof_metrics().to_dict(),
            "totals": {
                "total_moves": self.total_moves,
                "total_tasks": self.total_tasks_completed,
                "avg_moves_per_tick": self.total_moves / self.current_tick if self.current_tick > 0 else 0
            }
        }
