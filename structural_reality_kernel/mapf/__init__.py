"""
MAPF (Multi-Agent Path Finding) Module

Complete implementation of MAPF solving via quotient collapse.

Submodules:
    model      - Data structures (Graph, Instance, Path, Conflict)
    cbs        - CBS solver (quotient collapse algorithm)
    verifier   - V1-V5 verification
    ilp        - ILP cross-validation
    planviz    - Visualization and video generation
    proof_bundle - Cryptographic proof bundles

    benchmarks/ - Benchmark framework
    adapters/   - Integration adapters (ROS2, Unity, Isaac)
    simulation/ - Gazebo simulation
    examples/   - Runnable demos

Quick Start:
    from structural_reality_kernel.mapf import MAPFInstance, CBSSolver, verify_paths

    instance = MAPFInstance(graph, starts, goals)
    solver = CBSSolver(instance)
    result = solver.solve()
"""

from .model import (
    MAPFInstance,
    Graph,
    Conflict,
    Constraint,
    MAPFResult,
    ResultStatus,
    create_grid_graph,
    create_line_graph,
)

from .cbs import CBSSolver

from .verifier import (
    verify_paths,
    VerifierResult,
    VerifierCheck,
)

from .planviz import (
    PlanVizExporter,
    PlanVizVideoRenderer,
    quick_visualize,
)

__all__ = [
    # Model
    "MAPFInstance",
    "Graph",
    "Conflict",
    "Constraint",
    "MAPFResult",
    "ResultStatus",
    "create_grid_graph",
    "create_line_graph",

    # Solver
    "CBSSolver",

    # Verification
    "verify_paths",
    "VerifierResult",
    "VerifierCheck",

    # Visualization
    "PlanVizExporter",
    "PlanVizVideoRenderer",
    "quick_visualize",
]
