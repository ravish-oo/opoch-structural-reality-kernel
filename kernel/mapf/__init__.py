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

    production/ - Production-scale solver (100-500 robots)
                  Uses unlabeled flow with Π-fixed gauge collapse
                  O(|V|×T) complexity via quotient reduction

Quick Start (CBS):
    from kernel.mapf import MAPFInstance, CBSSolver, verify_paths

    instance = MAPFInstance(graph, starts, goals)
    solver = CBSSolver(instance)
    result = solver.solve()

Quick Start (Production Scale):
    from kernel.mapf.production import (
        solve_production_mapf,
        TimeExpandedGraph,
        WarehouseGraph,
        create_warehouse_graph,
    )

    warehouse = create_warehouse_graph(40, 40)
    te_graph = TimeExpandedGraph(
        base_graph=warehouse,
        horizon=50,
        starts=frozenset([...]),
        goals=frozenset([...])
    )
    result = solve_production_mapf(te_graph)
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

# Optional visualization (may have missing dependencies)
try:
    from .planviz import (
        PlanVizExporter,
        PlanVizVideoRenderer,
        quick_visualize,
    )
    HAS_PLANVIZ = True
except ImportError:
    HAS_PLANVIZ = False
    PlanVizExporter = None
    PlanVizVideoRenderer = None
    quick_visualize = None

# Production-scale solver (100-500 robots via unlabeled flow)
from .production import (
    # Graph construction
    TimeExpandedGraph,
    TimeExpandedNode,
    TimeExpandedEdge,
    EdgeType,
    WarehouseGraph,
    create_warehouse_graph,
    # Solving
    solve_production_mapf,
    ProductionFlowResult,
    ProductionFlowStatus,
    MinCutWitness,
    # Path extraction
    extract_paths,
    PathDecompositionResult,
    ExtractedPath,
    RobotAssignment,
    paths_to_labeled_solution,
    # Constraints
    SwapConstraintGadget,
    verify_flow_constraints,
    # Verification
    verify_production_solution,
    ProductionVerificationResult,
    # Proofs
    ProductionProofBundle,
    ProductionProofBundleBuilder,
    verify_production_proof,
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

    # Production-scale solver
    "TimeExpandedGraph",
    "TimeExpandedNode",
    "TimeExpandedEdge",
    "EdgeType",
    "WarehouseGraph",
    "create_warehouse_graph",
    "solve_production_mapf",
    "ProductionFlowResult",
    "ProductionFlowStatus",
    "MinCutWitness",
    "extract_paths",
    "PathDecompositionResult",
    "ExtractedPath",
    "RobotAssignment",
    "paths_to_labeled_solution",
    "SwapConstraintGadget",
    "verify_flow_constraints",
    "verify_production_solution",
    "ProductionVerificationResult",
    "ProductionProofBundle",
    "ProductionProofBundleBuilder",
    "verify_production_proof",
]
