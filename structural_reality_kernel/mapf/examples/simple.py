#!/usr/bin/env python3
"""
Simple MAPF Example

Demonstrates the quotient collapse solving a 4-agent MAPF instance.

Usage:
    python -m structural_reality_kernel.mapf.examples.simple
"""

import sys
import os

# Add parent directory to path for standalone execution
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from structural_reality_kernel.mapf.model import MAPFInstance, create_grid_graph
from structural_reality_kernel.mapf.cbs import CBSSolver
from structural_reality_kernel.mapf.verifier import verify_paths


def main():
    print("=" * 60)
    print("MAPF QUOTIENT COLLAPSE DEMO")
    print("=" * 60)

    # Create an 8x8 grid
    grid_size = 8
    graph = create_grid_graph(grid_size, grid_size)

    print(f"\nGrid: {grid_size}x{grid_size} = {grid_size**2} cells")
    print(f"Agents: 4")

    # 4 agents starting at corners, going to opposite corners
    starts = [0, 7, 56, 63]  # Corners: (0,0), (7,0), (0,7), (7,7)
    goals = [63, 56, 7, 0]   # Opposite corners

    print("\nAgent Configuration:")
    for i, (s, g) in enumerate(zip(starts, goals)):
        sx, sy = s % grid_size, s // grid_size
        gx, gy = g % grid_size, g // grid_size
        print(f"  Agent {i}: ({sx},{sy}) -> ({gx},{gy})")

    # Create instance
    instance = MAPFInstance(graph=graph, starts=starts, goals=goals)

    # Calculate naive search space
    naive_space = (grid_size ** 2) ** len(starts)
    print(f"\nNaive Search Space: {grid_size}^2^{len(starts)} = {naive_space:,} states")

    # Solve with CBS
    print("\nSolving with CBS (Quotient Collapse)...")
    solver = CBSSolver(instance, max_time=50, max_nodes=1000)
    result = solver.solve()

    if result.is_unique():
        print(f"\n✓ SOLUTION FOUND!")
        print(f"  Status: UNIQUE")
        print(f"  CBS Nodes Explored: {result.nodes_expanded}")
        print(f"  Makespan: {max(len(p)-1 for p in result.paths)} timesteps")
        print(f"  Sum of Costs: {result.cost}")

        # Calculate compression
        compression = naive_space / max(result.nodes_expanded, 1)
        import math
        log_compression = math.log10(compression) if compression > 0 else 0
        print(f"\n  Compression Ratio: {compression:,.0f}x")
        print(f"  Log10 Compression: 10^{log_compression:.1f}")

        # Verify solution
        print("\n--- V1-V5 Verification ---")
        T = max(len(p) - 1 for p in result.paths)
        verification = verify_paths(instance, result.paths, T)

        if verification.passed:
            print("  V1 (Start):    PASS")
            print("  V2 (Goal):     PASS")
            print("  V3 (Dynamics): PASS")
            print("  V4 (Vertex):   PASS")
            print("  V5 (Edge):     PASS")
            print("\n  ✓ ALL VERIFICATIONS PASSED")
        else:
            print(f"  FAILED: {verification.check}")

        # Print paths
        print("\n--- Paths ---")
        for i, path in enumerate(result.paths):
            path_str = " -> ".join(str(v) for v in path[:min(8, len(path))])
            if len(path) > 8:
                path_str += f" ... ({len(path)} steps)"
            print(f"  Agent {i}: {path_str}")

        # Receipt
        print(f"\n  Receipt: {result.receipt[:32]}...")

    elif result.is_unsat():
        print(f"\n✗ NO SOLUTION EXISTS")
        print(f"  Certificate: {result.certificate}")

    else:
        print(f"\n⚠ BUDGET EXHAUSTED (OMEGA)")
        print(f"  Nodes expanded: {result.nodes_expanded}")

    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
