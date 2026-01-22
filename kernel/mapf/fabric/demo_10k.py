"""
10,000-Robot Warehouse Demo

The demo that shakes the world.

This demonstrates:
- 10k robot tokens on a 2D warehouse fabric
- Real-time tick loop (target: 10-30 Hz equivalent)
- Proof overlay: "Permutation PASS" + receipt hash per tick
- Publishable run hash for replay

No benchmark needed - the proof is in the receipts.
You see 10k moving with no collisions and immediate recovery.

Usage:
    python -m kernel.mapf.fabric.demo_10k

CORRECTED MATHEMATICAL FOUNDATION:
----------------------------------
The tick IS a permutation P_t ∈ Match(A).
Junction modes are derived coordinates, not control inputs.

Controller outputs edge PREFERENCES (costs).
Matching solver finds optimal feasible permutation.
Hall witness certifies UNSAT if infeasible.

No minted infeasible ticks - the matching IS the separator.
"""

import time
import random
from typing import Dict, List, Set, Tuple, Optional
from dataclasses import dataclass

from .warehouse_graph import FabricWarehouseGraph, VertexType
from .fabric_compiler import FabricCompiler, TransportFabric
from .junction_gadgets import JunctionGadget
from .permutation_executor import (
    OccupancyBitset,
    Permutation,
    PermutationExecutor,
    FabricExecutor,
)
from .controller import BackpressureController, QueueState
from .label_tracker import LabelTracker, extract_robot_paths
from .proof_bundle import ProofBundleBuilder, FabricProofBundle, verify_fabric_proof
from .task_layer import TaskManager, DestinationClass, TaskType, TaskGenerator
from .metrics import FabricMetrics
from .permutation_completion import CompletionStatus


@dataclass
class DemoConfig:
    """Configuration for 10k demo."""
    num_robots: int = 10000
    grid_width: int = 200
    grid_height: int = 200
    horizon: int = 1000
    aisle_width: int = 3
    shelf_depth: int = 4
    num_stations: int = 50
    target_hz: float = 30.0


@dataclass
class DemoResult:
    """Result from 10k demo run."""
    num_robots: int
    num_ticks: int
    total_time_ms: float
    ticks_per_second: float
    all_verifications_passed: bool
    total_moves: int
    proof_fingerprint: str
    metrics: Dict


def create_large_warehouse(config: DemoConfig) -> FabricWarehouseGraph:
    """
    Create a large warehouse graph for 10k robots.

    Uses one-way aisles for higher throughput.
    """
    print(f"Creating {config.grid_width}x{config.grid_height} warehouse...")

    warehouse = FabricWarehouseGraph.create_aisle_warehouse(
        width=config.grid_width,
        height=config.grid_height,
        aisle_width=config.aisle_width,
        shelf_depth=config.shelf_depth,
        num_stations=config.num_stations
    )

    stats = warehouse.statistics()
    print(f"  Vertices: {stats['num_free']} free / {stats['num_vertices']} total")
    print(f"  Edges: {stats['num_edges']}")
    print(f"  Stations: {stats['num_stations']}")
    print(f"  Junction candidates: {stats['junction_candidates']}")

    return warehouse


def compile_fabric(warehouse: FabricWarehouseGraph) -> TransportFabric:
    """Compile warehouse into transport fabric."""
    print("\nCompiling transport fabric...")
    start = time.time()

    compiler = FabricCompiler(warehouse)
    fabric = compiler.compile()

    elapsed = (time.time() - start) * 1000
    stats = fabric.statistics()

    print(f"  Compilation time: {elapsed:.0f}ms")
    print(f"  Lanes: {stats['num_lanes']}")
    print(f"  Junctions: {stats['num_junctions']}")
    print(f"  Total junction modes: {stats['total_junction_modes']}")

    return fabric


def initialize_robots(fabric: TransportFabric,
                      num_robots: int) -> Tuple[OccupancyBitset, Dict[int, int]]:
    """
    Initialize robot positions randomly on free vertices.

    Returns:
        (occupancy_bitset, {robot_id: start_vertex})
    """
    free_vertices = list(fabric.lane_decomposition.vertex_to_lane.keys())
    free_vertices.extend(fabric.lane_decomposition.junction_vertices)

    if len(free_vertices) < num_robots:
        raise ValueError(f"Not enough free vertices ({len(free_vertices)}) for {num_robots} robots")

    # Random placement
    random.shuffle(free_vertices)
    robot_vertices = free_vertices[:num_robots]

    # Create occupancy
    occupancy = OccupancyBitset(fabric.warehouse.num_vertices, set(robot_vertices))

    # Create robot mapping
    robot_positions = {i: v for i, v in enumerate(robot_vertices)}

    return occupancy, robot_positions


def run_tick(fabric: TransportFabric,
             executor: FabricExecutor,
             controller: BackpressureController,
             occupancy: OccupancyBitset,
             queues: QueueState,
             tick_id: int) -> Tuple[OccupancyBitset, Permutation, Dict[int, int], bool]:
    """
    Run a single tick of the transport fabric.

    CORRECTED: The tick IS a permutation found by matching.
    Controller outputs preferences, not modes.

    Returns:
        (new_occupancy, permutation, derived_modes, success)
    """
    # 1. Compute edge preferences based on backpressure
    preferences = controller.compute_edge_preferences(occupancy, queues)

    # 2. Execute tick (solve matching, apply permutation)
    new_occupancy, permutation, result = executor.execute_tick(occupancy, preferences)

    # 3. Check if tick succeeded
    if result.status != CompletionStatus.SUCCESS:
        # UNSAT - return unchanged occupancy with Hall witness info
        print(f"  Tick {tick_id} UNSAT: {result.hall_witness}")
        return occupancy, Permutation.identity(fabric.warehouse.num_vertices), {}, False

    # 4. Derive junction modes from permutation (for logging/UI)
    derived_modes = executor._derive_junction_modes(permutation)

    return new_occupancy, permutation, derived_modes, True


def run_demo(config: DemoConfig) -> DemoResult:
    """
    Run the 10k robot demo.

    This is the main entry point that demonstrates:
    - Scale (10k robots)
    - Speed (real-time tick rate)
    - Safety (proof bundles every tick)
    """
    print("\n" + "="*70)
    print("10,000-ROBOT WAREHOUSE DEMO")
    print("Transport Fabric with Permutation-Based Updates")
    print("="*70)

    # Create warehouse
    warehouse = create_large_warehouse(config)

    # Compile fabric
    fabric = compile_fabric(warehouse)

    # Initialize robots
    print(f"\nInitializing {config.num_robots} robots...")
    occupancy, robot_positions = initialize_robots(fabric, config.num_robots)
    print(f"  Robots placed: {occupancy.count()}")

    # Initialize systems
    executor = FabricExecutor(fabric)
    controller = BackpressureController(fabric)
    queues = QueueState()

    # Initialize metrics
    metrics = FabricMetrics(config.num_robots, fabric.warehouse.num_vertices)

    # Initialize proof builder
    proof_builder = ProofBundleBuilder(
        fabric_hash=fabric.fingerprint(),
        initial_occupancy=occupancy,
        edges=set(fabric.warehouse.edges)
    )

    # Initialize label tracker (for path recovery)
    label_tracker = LabelTracker(robot_positions)

    print(f"\nRunning {config.horizon} ticks...")
    print(f"Target: {config.target_hz} Hz ({1000/config.target_hz:.1f}ms per tick)")
    print("-"*70)

    # Run tick loop
    start_time = time.time()
    tick_times = []

    unsat_count = 0
    for t in range(config.horizon):
        tick_start = time.time()

        # Execute tick (matching-based, guaranteed feasible or UNSAT)
        new_occupancy, permutation, modes, success = run_tick(
            fabric, executor, controller, occupancy, queues, t
        )

        if not success:
            unsat_count += 1
            # Skip this tick - occupancy unchanged
            tick_elapsed = (time.time() - tick_start) * 1000
            tick_times.append(tick_elapsed)
            continue

        # Record proof
        proof_builder.record_tick(t, modes, permutation, new_occupancy)

        # Track labels
        label_tracker.apply_permutation(permutation)

        # Update metrics
        moves = permutation.non_identity_count()
        active = sum(1 for v in occupancy.occupied_vertices()
                    if permutation.apply(v) != v)
        metrics.record_tick(moves, 0, None, active)

        # Progress update
        tick_elapsed = (time.time() - tick_start) * 1000
        tick_times.append(tick_elapsed)

        if (t + 1) % 100 == 0:
            avg_tick = sum(tick_times[-100:]) / min(100, len(tick_times))
            hz = 1000 / avg_tick if avg_tick > 0 else 0
            print(f"  Tick {t+1}: {avg_tick:.2f}ms/tick ({hz:.1f} Hz) | "
                  f"Moves: {moves} | Hash: {new_occupancy.fingerprint()[:8]}")

        occupancy = new_occupancy

    total_time = (time.time() - start_time) * 1000
    avg_tick_time = total_time / config.horizon
    actual_hz = 1000 / avg_tick_time

    print("-"*70)
    print(f"Completed {config.horizon} ticks in {total_time/1000:.2f}s")
    print(f"Average: {avg_tick_time:.2f}ms/tick ({actual_hz:.1f} Hz)")

    # Build proof bundle
    print("\nBuilding proof bundle...")
    proof_bundle = proof_builder.build()
    print(f"  Proof fingerprint: {proof_bundle.fingerprint()}")
    print(f"  All verifications passed: {proof_bundle.all_passed}")

    # Verify proof
    print("\nVerifying proof bundle...")
    verification = verify_fabric_proof(proof_bundle)
    print(f"  Chain valid: {verification['chain_valid']}")
    print(f"  All gates passed: {verification['all_verifications_passed']}")

    # Get metrics summary
    metrics_summary = metrics.summary()

    print("\n" + "="*70)
    print("DEMO RESULTS")
    print("="*70)
    print(f"Robots:           {config.num_robots:,}")
    print(f"Ticks:            {config.horizon:,}")
    print(f"UNSAT ticks:      {unsat_count} (Hall witness provided)")
    print(f"Total time:       {total_time/1000:.2f}s")
    print(f"Tick rate:        {actual_hz:.1f} Hz")
    print(f"Total moves:      {metrics.total_moves:,}")
    print(f"Moves/tick:       {metrics.total_moves/max(1, config.horizon-unsat_count):.0f}")
    print(f"Verification:     {'ALL PASS' if proof_bundle.all_passed else 'FAIL'}")
    print(f"Proof hash:       {proof_bundle.fingerprint()}")
    print("="*70)

    return DemoResult(
        num_robots=config.num_robots,
        num_ticks=config.horizon,
        total_time_ms=total_time,
        ticks_per_second=actual_hz,
        all_verifications_passed=proof_bundle.all_passed,
        total_moves=metrics.total_moves,
        proof_fingerprint=proof_bundle.fingerprint(),
        metrics=metrics_summary
    )


def run_scaling_demo() -> List[DemoResult]:
    """
    Run scaling demo across different robot counts.

    Demonstrates linear scaling with robot count.
    """
    print("\n" + "="*70)
    print("SCALING DEMO: 1K to 10K ROBOTS")
    print("="*70)

    results = []
    robot_counts = [1000, 2500, 5000, 7500, 10000]

    for num_robots in robot_counts:
        # Adjust grid size for robot count
        grid_size = max(100, int((num_robots * 4) ** 0.5))

        config = DemoConfig(
            num_robots=num_robots,
            grid_width=grid_size,
            grid_height=grid_size,
            horizon=200,  # Shorter for scaling test
        )

        try:
            result = run_demo(config)
            results.append(result)
        except Exception as e:
            print(f"Error at {num_robots} robots: {e}")
            break

    # Print summary
    print("\n" + "="*70)
    print("SCALING SUMMARY")
    print("="*70)
    print(f"{'Robots':<10} {'Ticks':<8} {'Time(s)':<10} {'Hz':<10} {'Moves/tick':<12} {'Status'}")
    print("-"*70)

    for r in results:
        status = "PASS" if r.all_verifications_passed else "FAIL"
        moves_per_tick = r.total_moves / r.num_ticks
        print(f"{r.num_robots:<10,} {r.num_ticks:<8} {r.total_time_ms/1000:<10.2f} "
              f"{r.ticks_per_second:<10.1f} {moves_per_tick:<12.0f} {status}")

    return results


def main():
    """Main entry point for 10k demo."""
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "scaling":
        run_scaling_demo()
    else:
        # Default: 10k demo
        config = DemoConfig(
            num_robots=10000,
            grid_width=200,
            grid_height=200,
            horizon=500,
        )
        run_demo(config)


if __name__ == "__main__":
    main()
