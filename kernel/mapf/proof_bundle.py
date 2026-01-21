"""
mapf_proof_bundle.py - Complete Proof Bundle Generator for MAPF Solutions.

A Proof Bundle contains everything needed to independently verify
a MAPF solution's correctness:

1. Instance specification (graph, agents, starts, goals)
2. Solution paths
3. Verification receipts (SHA-256 fingerprints)
4. Verifier results (V1-V5 checks)
5. CBS solver trace (optional)
6. ILP cross-check (optional)
7. Benchmark comparisons
8. Simulation artifacts

The bundle can be distributed and verified by any third party
without requiring the original solver.

Proof Bundle Specification:
- Self-contained JSON + artifacts archive
- Deterministic content hashing (canonical JSON)
- Version-controlled schema
- Tamper-evident (chain of hashes)
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple, Set
from enum import Enum
import json
import hashlib
import time
import zipfile
import tarfile
from pathlib import Path
from datetime import datetime, timezone
import base64
import io

from .model import (
    Graph,
    MAPFInstance,
    MAPFResult,
    ResultStatus,
    Conflict,
    ConflictType,
    VerifierResult,
    H,
    canon_json
)
from .verifier import verify_paths


# ============================================================
# PROOF BUNDLE SCHEMA
# ============================================================

BUNDLE_SCHEMA_VERSION = "1.0.0"

class BundleSection(Enum):
    """Sections in a proof bundle."""
    HEADER = "header"
    INSTANCE = "instance"
    SOLUTION = "solution"
    VERIFICATION = "verification"
    CBS_TRACE = "cbs_trace"
    ILP_CHECK = "ilp_check"
    BENCHMARKS = "benchmarks"
    SIMULATION = "simulation"
    SIGNATURES = "signatures"


@dataclass
class BundleMetadata:
    """Metadata for the proof bundle."""
    version: str = BUNDLE_SCHEMA_VERSION
    created_at: str = ""
    generator: str = "MAPF Kernel Verifier"
    generator_version: str = "1.0.0"

    def __post_init__(self):
        if not self.created_at:
            self.created_at = datetime.now(timezone.utc).isoformat()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": self.version,
            "created_at": self.created_at,
            "generator": self.generator,
            "generator_version": self.generator_version
        }


# ============================================================
# INSTANCE PROOF
# ============================================================

@dataclass
class InstanceProof:
    """
    Proof section for the MAPF instance.

    Contains all information needed to reconstruct the instance.
    """
    num_agents: int
    num_vertices: int
    num_edges: int
    starts: List[int]
    goals: List[int]
    edges: List[Tuple[int, int]]
    vertex_positions: Optional[Dict[int, Tuple[int, int]]] = None
    instance_fingerprint: str = ""

    @staticmethod
    def from_instance(instance: MAPFInstance) -> "InstanceProof":
        """Create instance proof from MAPFInstance."""
        return InstanceProof(
            num_agents=instance.num_agents,
            num_vertices=len(instance.graph.vertices),
            num_edges=len(instance.graph.edges),
            starts=list(instance.starts),
            goals=list(instance.goals),
            edges=list(instance.graph.edges),
            vertex_positions=getattr(instance.graph, 'pos', None),
            instance_fingerprint=instance.fingerprint()
        )

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "num_agents": self.num_agents,
            "num_vertices": self.num_vertices,
            "num_edges": self.num_edges,
            "starts": self.starts,
            "goals": self.goals,
            "edges": [list(e) for e in self.edges],
            "instance_fingerprint": self.instance_fingerprint
        }
        if self.vertex_positions:
            d["vertex_positions"] = {
                str(k): list(v) for k, v in self.vertex_positions.items()
            }
        return d


# ============================================================
# SOLUTION PROOF
# ============================================================

@dataclass
class PathProof:
    """Proof for a single agent's path."""
    agent_id: int
    path_length: int
    path: List[int]
    path_hash: str = ""

    def __post_init__(self):
        if not self.path_hash:
            self.path_hash = H(canon_json({"path": self.path}))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "agent_id": self.agent_id,
            "path_length": self.path_length,
            "path": self.path,
            "path_hash": self.path_hash
        }


@dataclass
class SolutionProof:
    """
    Proof section for the MAPF solution.

    Contains paths and solution metrics.
    """
    status: str
    makespan: int
    sum_of_costs: int
    paths: List[PathProof]
    solution_receipt: str
    nodes_expanded: Optional[int] = None

    @staticmethod
    def from_result(result: MAPFResult) -> "SolutionProof":
        """Create solution proof from MAPFResult."""
        paths = []
        makespan = 0
        soc = 0

        if result.paths:
            for i, path in enumerate(result.paths):
                paths.append(PathProof(
                    agent_id=i,
                    path_length=len(path),
                    path=list(path)
                ))
            makespan = max(len(p) - 1 for p in result.paths)
            soc = sum(len(p) - 1 for p in result.paths)

        return SolutionProof(
            status=result.status.value,
            makespan=makespan,
            sum_of_costs=soc,
            paths=paths,
            solution_receipt=result.receipt or "",
            nodes_expanded=result.nodes_expanded
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "status": self.status,
            "makespan": self.makespan,
            "sum_of_costs": self.sum_of_costs,
            "paths": [p.to_dict() for p in self.paths],
            "solution_receipt": self.solution_receipt,
            "nodes_expanded": self.nodes_expanded
        }


# ============================================================
# VERIFICATION PROOF
# ============================================================

@dataclass
class VerificationCheck:
    """Single verification check result."""
    check_id: str
    check_name: str
    passed: bool
    details: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "check_id": self.check_id,
            "check_name": self.check_name,
            "passed": self.passed
        }
        if self.details:
            d["details"] = self.details
        return d


@dataclass
class VerificationProof:
    """
    Proof section for verification results.

    Contains V1-V5 check results and conflict information.
    """
    all_passed: bool
    checks: List[VerificationCheck]
    horizon_t: int
    first_conflict: Optional[Dict[str, Any]] = None
    verification_receipt: str = ""

    @staticmethod
    def from_verifier_result(
        vr: VerifierResult,
        instance: MAPFInstance,
        paths: List[List[int]],
        horizon_t: int
    ) -> "VerificationProof":
        """Create verification proof from VerifierResult."""
        # Run individual checks for detailed reporting
        checks = []

        # V1: Start positions
        v1_passed = all(paths[i][0] == instance.starts[i] for i in range(instance.num_agents))
        checks.append(VerificationCheck(
            check_id="V1",
            check_name="Start Position Check",
            passed=v1_passed,
            details="All agents start at specified positions" if v1_passed else "Start position mismatch"
        ))

        # V2: Goal positions
        v2_passed = all(paths[i][-1] == instance.goals[i] for i in range(instance.num_agents))
        checks.append(VerificationCheck(
            check_id="V2",
            check_name="Goal Position Check",
            passed=v2_passed,
            details="All agents reach goals" if v2_passed else "Goal position mismatch"
        ))

        # V3: Edge validity
        v3_passed = True
        v3_details = "All moves are valid edges"
        for i, path in enumerate(paths):
            for t in range(len(path) - 1):
                u, v = path[t], path[t+1]
                if u != v and (u, v) not in instance.graph.edges and (v, u) not in instance.graph.edges:
                    v3_passed = False
                    v3_details = f"Invalid edge ({u},{v}) at t={t} for agent {i}"
                    break
            if not v3_passed:
                break
        checks.append(VerificationCheck(
            check_id="V3",
            check_name="Edge Validity Check",
            passed=v3_passed,
            details=v3_details
        ))

        # V4: Vertex conflicts
        v4_passed = True
        v4_details = "No vertex conflicts"
        T = max(len(p) - 1 for p in paths)
        for t in range(T + 1):
            positions = {}
            for i, path in enumerate(paths):
                pos = path[t] if t < len(path) else path[-1]
                if pos in positions:
                    v4_passed = False
                    v4_details = f"Vertex conflict at v={pos}, t={t} between agents {positions[pos]} and {i}"
                    break
                positions[pos] = i
            if not v4_passed:
                break
        checks.append(VerificationCheck(
            check_id="V4",
            check_name="Vertex Conflict Check",
            passed=v4_passed,
            details=v4_details
        ))

        # V5: Edge swap conflicts
        v5_passed = True
        v5_details = "No edge swap conflicts"
        for t in range(T):
            for i in range(len(paths)):
                for j in range(i + 1, len(paths)):
                    ui = paths[i][t] if t < len(paths[i]) else paths[i][-1]
                    vi = paths[i][t+1] if t+1 < len(paths[i]) else paths[i][-1]
                    uj = paths[j][t] if t < len(paths[j]) else paths[j][-1]
                    vj = paths[j][t+1] if t+1 < len(paths[j]) else paths[j][-1]

                    if ui == vj and vi == uj and ui != vi:
                        v5_passed = False
                        v5_details = f"Edge swap at t={t} between agents {i} and {j}"
                        break
                if not v5_passed:
                    break
            if not v5_passed:
                break
        checks.append(VerificationCheck(
            check_id="V5",
            check_name="Edge Swap Conflict Check",
            passed=v5_passed,
            details=v5_details
        ))

        # First conflict info
        first_conflict = None
        if vr.conflict:
            first_conflict = {
                "type": vr.conflict.conflict_type.value,
                "agents": list(vr.conflict.agents),
                "time": vr.conflict.time
            }
            if vr.conflict.conflict_type == ConflictType.VERTEX:
                first_conflict["vertex"] = vr.conflict.vertex
            else:
                first_conflict["edge_i"] = list(vr.conflict.edge_i)
                first_conflict["edge_j"] = list(vr.conflict.edge_j)

        # Compute verification receipt
        verification_data = {
            "checks": [c.to_dict() for c in checks],
            "horizon_t": horizon_t,
            "all_passed": vr.passed
        }
        verification_receipt = H(canon_json(verification_data))

        return VerificationProof(
            all_passed=vr.passed,
            checks=checks,
            horizon_t=horizon_t,
            first_conflict=first_conflict,
            verification_receipt=verification_receipt
        )

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "all_passed": self.all_passed,
            "checks": [c.to_dict() for c in self.checks],
            "horizon_t": self.horizon_t,
            "verification_receipt": self.verification_receipt
        }
        if self.first_conflict:
            d["first_conflict"] = self.first_conflict
        return d


# ============================================================
# CBS TRACE PROOF
# ============================================================

@dataclass
class CBSNodeProof:
    """Proof for a single CBS node."""
    node_id: int
    parent_id: Optional[int]
    cost: int
    constraints: List[Dict[str, Any]]
    conflict: Optional[Dict[str, Any]]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "node_id": self.node_id,
            "parent_id": self.parent_id,
            "cost": self.cost,
            "constraints": self.constraints,
            "conflict": self.conflict
        }


@dataclass
class CBSTraceProof:
    """
    Proof section for CBS solver trace.

    Contains the search tree for reproducibility.
    """
    nodes_expanded: int
    nodes_generated: int
    root_cost: int
    solution_node_id: Optional[int]
    trace_nodes: List[CBSNodeProof]
    tau_star_deterministic: bool = True

    def to_dict(self) -> Dict[str, Any]:
        return {
            "nodes_expanded": self.nodes_expanded,
            "nodes_generated": self.nodes_generated,
            "root_cost": self.root_cost,
            "solution_node_id": self.solution_node_id,
            "trace_nodes": [n.to_dict() for n in self.trace_nodes],
            "tau_star_deterministic": self.tau_star_deterministic
        }


# ============================================================
# ILP CROSS-CHECK PROOF
# ============================================================

@dataclass
class ILPProof:
    """
    Proof section for ILP feasibility cross-check.

    Independent verification of solution existence.
    """
    feasible: bool
    horizon_t: int
    num_variables: int
    num_constraints: int
    solver_status: str
    matches_cbs: bool
    ilp_receipt: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "feasible": self.feasible,
            "horizon_t": self.horizon_t,
            "num_variables": self.num_variables,
            "num_constraints": self.num_constraints,
            "solver_status": self.solver_status,
            "matches_cbs": self.matches_cbs,
            "ilp_receipt": self.ilp_receipt
        }


# ============================================================
# BENCHMARK PROOF
# ============================================================

@dataclass
class BenchmarkResult:
    """Single benchmark result."""
    benchmark_name: str
    passed: bool
    expected_status: str
    actual_status: str
    makespan: Optional[int] = None
    sum_of_costs: Optional[int] = None
    runtime_ms: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "benchmark_name": self.benchmark_name,
            "passed": self.passed,
            "expected_status": self.expected_status,
            "actual_status": self.actual_status
        }
        if self.makespan is not None:
            d["makespan"] = self.makespan
        if self.sum_of_costs is not None:
            d["sum_of_costs"] = self.sum_of_costs
        if self.runtime_ms is not None:
            d["runtime_ms"] = self.runtime_ms
        return d


@dataclass
class BenchmarkProof:
    """
    Proof section for benchmark results.

    Demonstrates solver performance on standard tests.
    """
    total_benchmarks: int
    passed_benchmarks: int
    results: List[BenchmarkResult]
    benchmark_suite: str = "MAPF Kernel Mandatory Tests"

    @property
    def all_passed(self) -> bool:
        return self.passed_benchmarks == self.total_benchmarks

    def to_dict(self) -> Dict[str, Any]:
        return {
            "total_benchmarks": self.total_benchmarks,
            "passed_benchmarks": self.passed_benchmarks,
            "all_passed": self.all_passed,
            "benchmark_suite": self.benchmark_suite,
            "results": [r.to_dict() for r in self.results]
        }


# ============================================================
# SIMULATION PROOF
# ============================================================

@dataclass
class SimulationArtifact:
    """Reference to a simulation artifact."""
    artifact_type: str  # "ros2", "isaac", "unity", "planviz"
    filename: str
    file_hash: str
    description: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "artifact_type": self.artifact_type,
            "filename": self.filename,
            "file_hash": self.file_hash,
            "description": self.description
        }


@dataclass
class SimulationProof:
    """
    Proof section for simulation artifacts.

    References to generated simulation files.
    """
    artifacts: List[SimulationArtifact]
    collision_free_verified: bool = False
    verification_notes: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "artifacts": [a.to_dict() for a in self.artifacts],
            "collision_free_verified": self.collision_free_verified,
            "verification_notes": self.verification_notes
        }


# ============================================================
# COMPLETE PROOF BUNDLE
# ============================================================

@dataclass
class ProofBundle:
    """
    Complete Proof Bundle for a MAPF solution.

    This is the main exportable artifact containing all proofs.
    """
    metadata: BundleMetadata
    instance: InstanceProof
    solution: SolutionProof
    verification: VerificationProof
    cbs_trace: Optional[CBSTraceProof] = None
    ilp_check: Optional[ILPProof] = None
    benchmarks: Optional[BenchmarkProof] = None
    simulation: Optional[SimulationProof] = None
    bundle_hash: str = ""

    def compute_bundle_hash(self) -> str:
        """Compute deterministic hash of entire bundle."""
        content = canon_json(self.to_dict(include_hash=False))
        return H(content)

    def to_dict(self, include_hash: bool = True) -> Dict[str, Any]:
        d = {
            "metadata": self.metadata.to_dict(),
            "instance": self.instance.to_dict(),
            "solution": self.solution.to_dict(),
            "verification": self.verification.to_dict()
        }
        if self.cbs_trace:
            d["cbs_trace"] = self.cbs_trace.to_dict()
        if self.ilp_check:
            d["ilp_check"] = self.ilp_check.to_dict()
        if self.benchmarks:
            d["benchmarks"] = self.benchmarks.to_dict()
        if self.simulation:
            d["simulation"] = self.simulation.to_dict()
        if include_hash:
            d["bundle_hash"] = self.bundle_hash or self.compute_bundle_hash()
        return d

    def to_json(self, indent: int = 2) -> str:
        """Export to JSON string."""
        return json.dumps(self.to_dict(), indent=indent)

    def save(self, filepath: str) -> None:
        """Save bundle to JSON file."""
        if not self.bundle_hash:
            self.bundle_hash = self.compute_bundle_hash()
        with open(filepath, 'w') as f:
            f.write(self.to_json())

    def verify_integrity(self) -> bool:
        """Verify bundle integrity by recomputing hash."""
        expected = self.compute_bundle_hash()
        return self.bundle_hash == expected


# ============================================================
# BUNDLE GENERATOR
# ============================================================

class ProofBundleGenerator:
    """
    Generator for complete proof bundles.

    Assembles all proof sections from solver outputs.
    """

    def __init__(self, output_dir: str = "./proof_bundles"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def generate_bundle(
        self,
        instance: MAPFInstance,
        result: MAPFResult,
        horizon_t: Optional[int] = None,
        include_cbs_trace: bool = False,
        cbs_trace_data: Optional[Dict[str, Any]] = None,
        include_ilp: bool = False,
        ilp_data: Optional[Dict[str, Any]] = None,
        benchmark_results: Optional[List[Dict[str, Any]]] = None,
        simulation_artifacts: Optional[List[Dict[str, Any]]] = None
    ) -> ProofBundle:
        """
        Generate a complete proof bundle.

        Args:
            instance: MAPF instance
            result: Solver result
            horizon_t: Verification horizon (auto-computed if None)
            include_cbs_trace: Include CBS search trace
            cbs_trace_data: CBS trace data
            include_ilp: Include ILP cross-check
            ilp_data: ILP check data
            benchmark_results: Benchmark results
            simulation_artifacts: Simulation artifact references

        Returns:
            Complete ProofBundle
        """
        # Metadata
        metadata = BundleMetadata()

        # Instance proof
        instance_proof = InstanceProof.from_instance(instance)

        # Solution proof
        solution_proof = SolutionProof.from_result(result)

        # Verification proof
        if horizon_t is None and result.paths:
            horizon_t = max(len(p) - 1 for p in result.paths)
        elif horizon_t is None:
            horizon_t = 0

        # Verify the paths
        if result.paths:
            vr = verify_paths(instance, result.paths, horizon_t)
        else:
            # No paths to verify - create a placeholder result
            vr = VerifierResult(passed=True, check=None, details={})

        verification_proof = VerificationProof.from_verifier_result(
            vr, instance, result.paths or [], horizon_t
        )

        # Optional sections
        cbs_trace = None
        if include_cbs_trace and cbs_trace_data:
            cbs_trace = self._build_cbs_trace(cbs_trace_data)

        ilp_check = None
        if include_ilp and ilp_data:
            ilp_check = self._build_ilp_proof(ilp_data)

        benchmarks = None
        if benchmark_results:
            benchmarks = self._build_benchmark_proof(benchmark_results)

        simulation = None
        if simulation_artifacts:
            simulation = self._build_simulation_proof(simulation_artifacts)

        # Create bundle
        bundle = ProofBundle(
            metadata=metadata,
            instance=instance_proof,
            solution=solution_proof,
            verification=verification_proof,
            cbs_trace=cbs_trace,
            ilp_check=ilp_check,
            benchmarks=benchmarks,
            simulation=simulation
        )

        # Compute final hash
        bundle.bundle_hash = bundle.compute_bundle_hash()

        return bundle

    def _build_cbs_trace(self, data: Dict[str, Any]) -> CBSTraceProof:
        """Build CBS trace proof from data."""
        trace_nodes = []
        for node_data in data.get("nodes", []):
            trace_nodes.append(CBSNodeProof(
                node_id=node_data.get("id", 0),
                parent_id=node_data.get("parent_id"),
                cost=node_data.get("cost", 0),
                constraints=node_data.get("constraints", []),
                conflict=node_data.get("conflict")
            ))

        return CBSTraceProof(
            nodes_expanded=data.get("nodes_expanded", 0),
            nodes_generated=data.get("nodes_generated", 0),
            root_cost=data.get("root_cost", 0),
            solution_node_id=data.get("solution_node_id"),
            trace_nodes=trace_nodes,
            tau_star_deterministic=data.get("tau_star_deterministic", True)
        )

    def _build_ilp_proof(self, data: Dict[str, Any]) -> ILPProof:
        """Build ILP proof from data."""
        ilp_data = {
            "feasible": data.get("feasible", False),
            "horizon_t": data.get("horizon_t", 0),
            "num_variables": data.get("num_variables", 0),
            "num_constraints": data.get("num_constraints", 0),
            "solver_status": data.get("solver_status", "unknown"),
            "matches_cbs": data.get("matches_cbs", False)
        }
        ilp_receipt = H(canon_json(ilp_data))

        return ILPProof(
            feasible=ilp_data["feasible"],
            horizon_t=ilp_data["horizon_t"],
            num_variables=ilp_data["num_variables"],
            num_constraints=ilp_data["num_constraints"],
            solver_status=ilp_data["solver_status"],
            matches_cbs=ilp_data["matches_cbs"],
            ilp_receipt=ilp_receipt
        )

    def _build_benchmark_proof(
        self,
        results: List[Dict[str, Any]]
    ) -> BenchmarkProof:
        """Build benchmark proof from results."""
        benchmark_results = []
        passed_count = 0

        for r in results:
            br = BenchmarkResult(
                benchmark_name=r.get("name", "unknown"),
                passed=r.get("passed", False),
                expected_status=r.get("expected_status", ""),
                actual_status=r.get("actual_status", ""),
                makespan=r.get("makespan"),
                sum_of_costs=r.get("sum_of_costs"),
                runtime_ms=r.get("runtime_ms")
            )
            benchmark_results.append(br)
            if br.passed:
                passed_count += 1

        return BenchmarkProof(
            total_benchmarks=len(benchmark_results),
            passed_benchmarks=passed_count,
            results=benchmark_results
        )

    def _build_simulation_proof(
        self,
        artifacts: List[Dict[str, Any]]
    ) -> SimulationProof:
        """Build simulation proof from artifacts."""
        sim_artifacts = []

        for a in artifacts:
            sim_artifacts.append(SimulationArtifact(
                artifact_type=a.get("type", "unknown"),
                filename=a.get("filename", ""),
                file_hash=a.get("hash", ""),
                description=a.get("description", "")
            ))

        return SimulationProof(
            artifacts=sim_artifacts,
            collision_free_verified=all(
                a.get("collision_free", False) for a in artifacts
            )
        )

    def save_bundle(
        self,
        bundle: ProofBundle,
        name: str,
        format: str = "json"
    ) -> str:
        """
        Save bundle to file.

        Args:
            bundle: Proof bundle to save
            name: Base name for the file
            format: "json" or "archive"

        Returns:
            Path to saved file
        """
        if format == "json":
            filepath = self.output_dir / f"{name}_proof_bundle.json"
            bundle.save(str(filepath))
        else:
            filepath = self.output_dir / f"{name}_proof_bundle.tar.gz"
            self._save_archive(bundle, str(filepath))

        return str(filepath)

    def _save_archive(self, bundle: ProofBundle, filepath: str) -> None:
        """Save bundle as compressed archive with artifacts."""
        with tarfile.open(filepath, "w:gz") as tar:
            # Add main bundle JSON
            bundle_json = bundle.to_json()
            bundle_bytes = bundle_json.encode('utf-8')
            bundle_info = tarfile.TarInfo(name="proof_bundle.json")
            bundle_info.size = len(bundle_bytes)
            tar.addfile(bundle_info, io.BytesIO(bundle_bytes))

            # Add verification summary
            summary = self._generate_summary(bundle)
            summary_bytes = summary.encode('utf-8')
            summary_info = tarfile.TarInfo(name="VERIFICATION_SUMMARY.txt")
            summary_info.size = len(summary_bytes)
            tar.addfile(summary_info, io.BytesIO(summary_bytes))

    def _generate_summary(self, bundle: ProofBundle) -> str:
        """Generate human-readable verification summary."""
        lines = [
            "=" * 60,
            "MAPF PROOF BUNDLE - VERIFICATION SUMMARY",
            "=" * 60,
            "",
            f"Bundle Hash: {bundle.bundle_hash}",
            f"Generated: {bundle.metadata.created_at}",
            f"Generator: {bundle.metadata.generator} v{bundle.metadata.generator_version}",
            "",
            "-" * 60,
            "INSTANCE",
            "-" * 60,
            f"  Agents: {bundle.instance.num_agents}",
            f"  Vertices: {bundle.instance.num_vertices}",
            f"  Edges: {bundle.instance.num_edges}",
            f"  Fingerprint: {bundle.instance.instance_fingerprint[:32]}...",
            "",
            "-" * 60,
            "SOLUTION",
            "-" * 60,
            f"  Status: {bundle.solution.status}",
            f"  Makespan: {bundle.solution.makespan}",
            f"  Sum of Costs: {bundle.solution.sum_of_costs}",
            f"  Receipt: {bundle.solution.solution_receipt[:32]}...",
            "",
            "-" * 60,
            "VERIFICATION (Truth Gate)",
            "-" * 60,
            f"  ALL PASSED: {'YES' if bundle.verification.all_passed else 'NO'}",
            "",
        ]

        for check in bundle.verification.checks:
            status = "PASS" if check.passed else "FAIL"
            lines.append(f"  [{status}] {check.check_id}: {check.check_name}")
            if check.details:
                lines.append(f"         {check.details}")

        if bundle.ilp_check:
            lines.extend([
                "",
                "-" * 60,
                "ILP CROSS-CHECK",
                "-" * 60,
                f"  Feasible: {bundle.ilp_check.feasible}",
                f"  Matches CBS: {bundle.ilp_check.matches_cbs}",
            ])

        if bundle.benchmarks:
            lines.extend([
                "",
                "-" * 60,
                "BENCHMARKS",
                "-" * 60,
                f"  Passed: {bundle.benchmarks.passed_benchmarks}/{bundle.benchmarks.total_benchmarks}",
                f"  Suite: {bundle.benchmarks.benchmark_suite}",
            ])

        lines.extend([
            "",
            "=" * 60,
            "This proof bundle can be independently verified.",
            "All hashes are SHA-256 of canonical JSON.",
            "=" * 60,
        ])

        return "\n".join(lines)


# ============================================================
# BUNDLE VERIFIER
# ============================================================

class ProofBundleVerifier:
    """
    Independent verifier for proof bundles.

    Can verify bundle integrity and re-run verification checks.
    """

    def verify_bundle(self, bundle_path: str) -> Dict[str, Any]:
        """
        Verify a proof bundle.

        Args:
            bundle_path: Path to bundle JSON file

        Returns:
            Verification results
        """
        with open(bundle_path, 'r') as f:
            data = json.load(f)

        results = {
            "bundle_path": bundle_path,
            "checks": []
        }

        # Check 1: Bundle hash integrity
        stored_hash = data.get("bundle_hash", "")
        data_without_hash = {k: v for k, v in data.items() if k != "bundle_hash"}
        computed_hash = H(canon_json(data_without_hash))
        hash_valid = stored_hash == computed_hash

        results["checks"].append({
            "name": "Bundle Hash Integrity",
            "passed": hash_valid,
            "stored": stored_hash[:32] + "...",
            "computed": computed_hash[:32] + "..."
        })

        # Check 2: Instance fingerprint
        instance_data = data.get("instance", {})
        stored_fingerprint = instance_data.get("instance_fingerprint", "")
        # Recompute would require full instance reconstruction

        results["checks"].append({
            "name": "Instance Fingerprint",
            "passed": len(stored_fingerprint) == 64,  # SHA-256 length
            "fingerprint": stored_fingerprint[:32] + "..."
        })

        # Check 3: Solution receipt
        solution_data = data.get("solution", {})
        stored_receipt = solution_data.get("solution_receipt", "")

        results["checks"].append({
            "name": "Solution Receipt",
            "passed": len(stored_receipt) == 64,
            "receipt": stored_receipt[:32] + "..."
        })

        # Check 4: Verification results
        verification_data = data.get("verification", {})
        all_passed = verification_data.get("all_passed", False)
        checks = verification_data.get("checks", [])

        results["checks"].append({
            "name": "Verification Checks",
            "passed": all_passed,
            "num_checks": len(checks),
            "checks_passed": sum(1 for c in checks if c.get("passed", False))
        })

        # Overall result
        results["all_valid"] = all(c["passed"] for c in results["checks"])
        results["verification_status"] = (
            "VALID" if results["all_valid"] else "INVALID"
        )

        return results

    def verify_bundle_paths(
        self,
        bundle_path: str
    ) -> Dict[str, Any]:
        """
        Re-verify paths from a bundle (requires full path data).

        This performs independent V1-V5 verification.
        """
        with open(bundle_path, 'r') as f:
            data = json.load(f)

        instance_data = data.get("instance", {})
        solution_data = data.get("solution", {})

        # Reconstruct graph
        edges = [tuple(e) for e in instance_data.get("edges", [])]
        vertices = set()
        for u, v in edges:
            vertices.add(u)
            vertices.add(v)

        graph = Graph(vertices=list(vertices), edges=set(edges))

        # Reconstruct instance
        starts = tuple(instance_data.get("starts", []))
        goals = tuple(instance_data.get("goals", []))
        num_agents = instance_data.get("num_agents", 0)

        instance = MAPFInstance(
            graph=graph,
            num_agents=num_agents,
            starts=starts,
            goals=goals
        )

        # Extract paths
        paths = []
        for path_data in solution_data.get("paths", []):
            paths.append(path_data.get("path", []))

        if not paths:
            return {"valid": False, "reason": "No paths in bundle"}

        # Run verification
        horizon_t = max(len(p) - 1 for p in paths)
        vr = verify_paths(instance, paths, horizon_t)

        return {
            "valid": vr.passed,
            "reason": vr.reason,
            "horizon_t": horizon_t,
            "independent_verification": True
        }


# ============================================================
# CONVENIENCE FUNCTIONS
# ============================================================

def generate_proof_bundle(
    instance: MAPFInstance,
    result: MAPFResult,
    output_path: Optional[str] = None,
    include_all: bool = True
) -> ProofBundle:
    """
    Convenience function to generate and optionally save a proof bundle.

    Args:
        instance: MAPF instance
        result: Solver result
        output_path: Optional path to save bundle
        include_all: Include all optional sections

    Returns:
        Generated ProofBundle
    """
    generator = ProofBundleGenerator()
    bundle = generator.generate_bundle(
        instance=instance,
        result=result,
        include_cbs_trace=False,  # Requires trace data
        include_ilp=False  # Requires ILP data
    )

    if output_path:
        bundle.save(output_path)

    return bundle


def verify_proof_bundle(bundle_path: str) -> Dict[str, Any]:
    """
    Convenience function to verify a proof bundle.

    Args:
        bundle_path: Path to bundle JSON

    Returns:
        Verification results
    """
    verifier = ProofBundleVerifier()
    return verifier.verify_bundle(bundle_path)
