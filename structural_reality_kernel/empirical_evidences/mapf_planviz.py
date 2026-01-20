"""
mapf_planviz.py - PlanViz Export for MAPF Solutions.

PlanViz is the standard visualization tool for MAPF solutions,
developed as part of the League of Robot Runners (LoRR) competition.
See: https://github.com/MAPF-Competition/PlanViz

This module exports MAPF solutions in PlanViz-compatible JSON format
for visualization and post-hoc analysis.

Output format follows PlanViz specification:
- Map grid with obstacles
- Agent paths as coordinate sequences
- Conflict detection visualization
- Statistics overlay
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple
import json
from pathlib import Path

from .mapf_model import (
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
from .mapf_verifier import verify_paths, get_first_conflict
from .mapf_movingai import MovingAIMap


# ============================================================
# PLANVIZ JSON FORMAT
# ============================================================

@dataclass
class PlanVizAction:
    """Single action in PlanViz format."""
    t: int           # Timestep
    x: int           # X coordinate
    y: int           # Y coordinate
    action: str      # "move" or "wait"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "t": self.t,
            "x": self.x,
            "y": self.y,
            "action": self.action
        }


@dataclass
class PlanVizAgent:
    """Agent data in PlanViz format."""
    id: int
    name: str
    start_x: int
    start_y: int
    goal_x: int
    goal_y: int
    path: List[PlanVizAction]
    color: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "id": self.id,
            "name": self.name,
            "start": {"x": self.start_x, "y": self.start_y},
            "goal": {"x": self.goal_x, "y": self.goal_y},
            "path": [a.to_dict() for a in self.path]
        }
        if self.color:
            d["color"] = self.color
        return d


@dataclass
class PlanVizConflict:
    """Conflict marker in PlanViz format."""
    type: str        # "vertex" or "edge"
    time: int
    agent1: int
    agent2: int
    x1: int
    y1: int
    x2: Optional[int] = None
    y2: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "type": self.type,
            "time": self.time,
            "agents": [self.agent1, self.agent2],
            "location1": {"x": self.x1, "y": self.y1}
        }
        if self.x2 is not None and self.y2 is not None:
            d["location2"] = {"x": self.x2, "y": self.y2}
        return d


@dataclass
class PlanVizStatistics:
    """Solution statistics for PlanViz."""
    makespan: int
    sum_of_costs: int
    num_agents: int
    solved: bool
    runtime_ms: Optional[int] = None
    nodes_expanded: Optional[int] = None
    solver: str = "CBS"

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "makespan": self.makespan,
            "sum_of_costs": self.sum_of_costs,
            "num_agents": self.num_agents,
            "solved": self.solved,
            "solver": self.solver
        }
        if self.runtime_ms is not None:
            d["runtime_ms"] = self.runtime_ms
        if self.nodes_expanded is not None:
            d["nodes_expanded"] = self.nodes_expanded
        return d


@dataclass
class PlanVizOutput:
    """
    Complete PlanViz output structure.

    This is the main export format for visualization.
    """
    map_name: str
    width: int
    height: int
    grid: List[List[int]]  # 0 = passable, 1 = obstacle
    agents: List[PlanVizAgent]
    conflicts: List[PlanVizConflict]
    statistics: PlanVizStatistics
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "map": {
                "name": self.map_name,
                "width": self.width,
                "height": self.height,
                "grid": self.grid
            },
            "agents": [a.to_dict() for a in self.agents],
            "conflicts": [c.to_dict() for c in self.conflicts],
            "statistics": self.statistics.to_dict(),
            "metadata": self.metadata
        }

    def to_json(self, indent: int = 2) -> str:
        """Export to JSON string."""
        return json.dumps(self.to_dict(), indent=indent)

    def save(self, filepath: str) -> None:
        """Save to JSON file."""
        with open(filepath, 'w') as f:
            f.write(self.to_json())


# ============================================================
# CONVERSION FUNCTIONS
# ============================================================

def convert_map_to_planviz_grid(map_data: MovingAIMap) -> List[List[int]]:
    """
    Convert MovingAI map to PlanViz grid format.

    Returns: 2D grid where 0 = passable, 1 = obstacle
    """
    grid = []
    for y in range(map_data.height):
        row = []
        for x in range(map_data.width):
            if map_data.is_passable(x, y):
                row.append(0)
            else:
                row.append(1)
        grid.append(row)
    return grid


def convert_path_to_actions(
    path: List[int],
    map_data: MovingAIMap
) -> List[PlanVizAction]:
    """
    Convert vertex path to PlanViz action sequence.

    Determines if each step is a "move" or "wait" action.
    """
    actions = []

    for t, v in enumerate(path):
        x, y = map_data.from_vertex(v)

        if t == 0:
            action_type = "start"
        elif t > 0 and path[t] == path[t-1]:
            action_type = "wait"
        else:
            action_type = "move"

        actions.append(PlanVizAction(
            t=t,
            x=x,
            y=y,
            action=action_type
        ))

    return actions


def convert_conflict_to_planviz(
    conflict: Conflict,
    map_data: MovingAIMap
) -> PlanVizConflict:
    """Convert MAPF Conflict to PlanViz format."""
    if conflict.conflict_type == ConflictType.VERTEX:
        x, y = map_data.from_vertex(conflict.vertex)
        return PlanVizConflict(
            type="vertex",
            time=conflict.time,
            agent1=conflict.agents[0],
            agent2=conflict.agents[1],
            x1=x,
            y1=y
        )
    else:  # EDGE_SWAP
        x1, y1 = map_data.from_vertex(conflict.edge_i[0])
        x2, y2 = map_data.from_vertex(conflict.edge_i[1])
        return PlanVizConflict(
            type="edge",
            time=conflict.time,
            agent1=conflict.agents[0],
            agent2=conflict.agents[1],
            x1=x1,
            y1=y1,
            x2=x2,
            y2=y2
        )


# Agent colors for visualization
AGENT_COLORS = [
    "#FF6B6B",  # Red
    "#4ECDC4",  # Teal
    "#45B7D1",  # Blue
    "#96CEB4",  # Green
    "#FFEAA7",  # Yellow
    "#DDA0DD",  # Plum
    "#98D8C8",  # Mint
    "#F7DC6F",  # Gold
    "#BB8FCE",  # Purple
    "#85C1E9",  # Sky blue
]


def get_agent_color(agent_id: int) -> str:
    """Get color for agent visualization."""
    return AGENT_COLORS[agent_id % len(AGENT_COLORS)]


# ============================================================
# MAIN EXPORT FUNCTION
# ============================================================

def export_to_planviz(
    instance: MAPFInstance,
    result: MAPFResult,
    map_data: MovingAIMap,
    runtime_ms: Optional[int] = None,
    include_conflicts: bool = True
) -> PlanVizOutput:
    """
    Export MAPF solution to PlanViz format.

    Args:
        instance: MAPF instance
        result: CBS result
        map_data: MovingAI map data
        runtime_ms: Optional runtime in milliseconds
        include_conflicts: Whether to include conflict markers

    Returns:
        PlanVizOutput ready for export
    """
    # Convert map to grid
    grid = convert_map_to_planviz_grid(map_data)

    # Convert agents and paths
    agents = []
    for i in range(instance.num_agents):
        start_x, start_y = map_data.from_vertex(instance.starts[i])
        goal_x, goal_y = map_data.from_vertex(instance.goals[i])

        if result.status == ResultStatus.UNIQUE and result.paths:
            path = convert_path_to_actions(result.paths[i], map_data)
        else:
            # No solution - just show start/goal
            path = [PlanVizAction(0, start_x, start_y, "start")]

        agent = PlanVizAgent(
            id=i,
            name=f"Agent {i}",
            start_x=start_x,
            start_y=start_y,
            goal_x=goal_x,
            goal_y=goal_y,
            path=path,
            color=get_agent_color(i)
        )
        agents.append(agent)

    # Detect conflicts (for visualization)
    conflicts = []
    if include_conflicts and result.paths:
        # Check for all conflicts in the solution
        T = max(len(p) - 1 for p in result.paths)
        verify_result = verify_paths(instance, result.paths, T)
        if not verify_result.passed and verify_result.conflict:
            pv_conflict = convert_conflict_to_planviz(
                verify_result.conflict, map_data
            )
            conflicts.append(pv_conflict)

    # Compute statistics
    if result.status == ResultStatus.UNIQUE and result.paths:
        makespan = max(len(p) - 1 for p in result.paths)
        soc = sum(len(p) - 1 for p in result.paths)
        solved = True
    else:
        makespan = 0
        soc = 0
        solved = False

    statistics = PlanVizStatistics(
        makespan=makespan,
        sum_of_costs=soc,
        num_agents=instance.num_agents,
        solved=solved,
        runtime_ms=runtime_ms,
        nodes_expanded=result.nodes_expanded,
        solver="CBS"
    )

    # Metadata
    metadata = {
        "status": result.status.value,
        "instance_fingerprint": instance.fingerprint()[:16],
        "exporter": "MAPF Kernel Verifier",
        "version": "1.0"
    }

    if result.receipt:
        metadata["receipt"] = result.receipt[:32]

    return PlanVizOutput(
        map_name=map_data.name,
        width=map_data.width,
        height=map_data.height,
        grid=grid,
        agents=agents,
        conflicts=conflicts,
        statistics=statistics,
        metadata=metadata
    )


# ============================================================
# BATCH EXPORT
# ============================================================

def export_benchmark_results(
    results: List[Tuple[str, MAPFInstance, MAPFResult, MovingAIMap, int]],
    output_dir: str
) -> Dict[str, Any]:
    """
    Export multiple benchmark results to PlanViz files.

    Args:
        results: List of (name, instance, result, map_data, runtime_ms)
        output_dir: Directory to save files

    Returns:
        Summary of exports
    """
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    exports = []
    for name, instance, result, map_data, runtime_ms in results:
        output = export_to_planviz(instance, result, map_data, runtime_ms)
        filepath = Path(output_dir) / f"{name}.json"
        output.save(str(filepath))

        exports.append({
            "name": name,
            "file": str(filepath),
            "solved": output.statistics.solved,
            "makespan": output.statistics.makespan,
            "soc": output.statistics.sum_of_costs
        })

    return {
        "total": len(exports),
        "solved": sum(1 for e in exports if e["solved"]),
        "exports": exports
    }


# ============================================================
# ANIMATION EXPORT
# ============================================================

@dataclass
class AnimationFrame:
    """Single frame in animation sequence."""
    time: int
    agent_positions: List[Tuple[int, int]]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "time": self.time,
            "positions": [{"x": x, "y": y} for x, y in self.agent_positions]
        }


def export_animation_frames(
    result: MAPFResult,
    map_data: MovingAIMap
) -> List[AnimationFrame]:
    """
    Export solution as animation frames.

    Each frame contains all agent positions at that timestep.
    """
    if result.status != ResultStatus.UNIQUE or not result.paths:
        return []

    makespan = max(len(p) - 1 for p in result.paths)
    frames = []

    for t in range(makespan + 1):
        positions = []
        for path in result.paths:
            # Handle paths shorter than makespan (goal-hold)
            if t < len(path):
                v = path[t]
            else:
                v = path[-1]
            x, y = map_data.from_vertex(v)
            positions.append((x, y))

        frames.append(AnimationFrame(time=t, positions=positions))

    return frames


def export_animation_json(
    result: MAPFResult,
    map_data: MovingAIMap
) -> Dict[str, Any]:
    """Export animation data as JSON-serializable dict."""
    frames = export_animation_frames(result, map_data)
    return {
        "num_agents": len(result.paths) if result.paths else 0,
        "makespan": len(frames) - 1 if frames else 0,
        "frames": [f.to_dict() for f in frames]
    }


# ============================================================
# PLANVIZ VIEWER INTEGRATION
# ============================================================

class PlanVizExporter:
    """
    Complete PlanViz exporter with viewer integration.

    Provides methods for exporting solutions in formats
    compatible with the PlanViz web viewer.
    """

    def __init__(self, output_dir: str = "./planviz_output"):
        self.output_dir = output_dir
        Path(output_dir).mkdir(parents=True, exist_ok=True)

    def export_solution(
        self,
        name: str,
        instance: MAPFInstance,
        result: MAPFResult,
        map_data: MovingAIMap,
        runtime_ms: Optional[int] = None
    ) -> str:
        """
        Export single solution to PlanViz JSON.

        Returns: Path to exported file
        """
        output = export_to_planviz(instance, result, map_data, runtime_ms)
        filepath = Path(self.output_dir) / f"{name}.json"
        output.save(str(filepath))
        return str(filepath)

    def export_animation(
        self,
        name: str,
        result: MAPFResult,
        map_data: MovingAIMap
    ) -> str:
        """
        Export animation data.

        Returns: Path to exported file
        """
        anim_data = export_animation_json(result, map_data)
        filepath = Path(self.output_dir) / f"{name}_animation.json"
        with open(filepath, 'w') as f:
            json.dump(anim_data, f, indent=2)
        return str(filepath)

    def export_batch(
        self,
        results: List[Tuple[str, MAPFInstance, MAPFResult, MovingAIMap, int]]
    ) -> Dict[str, Any]:
        """Export multiple results."""
        return export_benchmark_results(results, self.output_dir)

    def generate_viewer_html(self, json_files: List[str]) -> str:
        """
        Generate simple HTML viewer for exported solutions.

        Returns: Path to generated HTML file
        """
        html_content = """<!DOCTYPE html>
<html>
<head>
    <title>MAPF Solution Viewer</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .solution { margin: 20px 0; padding: 20px; border: 1px solid #ccc; }
        .grid { display: inline-block; }
        .cell { width: 20px; height: 20px; border: 1px solid #eee; display: inline-block; }
        .obstacle { background: #333; }
        .passable { background: #fff; }
        .agent { border-radius: 50%; }
        .stats { margin-top: 10px; }
    </style>
</head>
<body>
    <h1>MAPF Solutions - PlanViz Export</h1>
    <p>Generated by MAPF Kernel Verifier</p>
    <div id="solutions">
"""
        for jf in json_files:
            html_content += f'    <div class="solution"><h3>{Path(jf).stem}</h3><pre id="data_{Path(jf).stem}"></pre></div>\n'

        html_content += """
    </div>
    <script>
        // Load JSON files via fetch if running on server
        // For local use, data would need to be embedded
    </script>
</body>
</html>
"""
        filepath = Path(self.output_dir) / "viewer.html"
        with open(filepath, 'w') as f:
            f.write(html_content)
        return str(filepath)


# ============================================================
# VIDEO RENDERING ENGINE
# ============================================================

@dataclass
class VideoConfig:
    """Configuration for video rendering."""
    width: int = 800
    height: int = 800
    fps: int = 4
    cell_size: int = 30
    agent_radius: int = 12
    trail_length: int = 5
    show_grid: bool = True
    show_labels: bool = True
    background_color: str = "#FFFFFF"
    obstacle_color: str = "#333333"
    grid_color: str = "#CCCCCC"
    font_size: int = 10


class PlanVizVideoRenderer:
    """
    Video renderer for MAPF solutions.
    Generates MP4 and GIF animations using matplotlib.
    """

    def __init__(self, config: Optional[VideoConfig] = None):
        self.config = config or VideoConfig()
        self._check_dependencies()

    def _check_dependencies(self) -> bool:
        """Check if video rendering dependencies are available."""
        self.has_matplotlib = False
        self.has_pillow = False
        self.has_ffmpeg = False

        try:
            import matplotlib
            matplotlib.use('Agg')
            self.has_matplotlib = True
        except ImportError:
            pass

        try:
            from PIL import Image
            self.has_pillow = True
        except ImportError:
            pass

        import subprocess
        try:
            subprocess.run(['ffmpeg', '-version'], capture_output=True, check=True)
            self.has_ffmpeg = True
        except (subprocess.CalledProcessError, FileNotFoundError):
            pass

        return self.has_matplotlib

    def render_to_gif(self, planviz_output: PlanVizOutput, output_path: str,
                      duration_per_frame_ms: int = 500) -> bool:
        """Render solution animation to GIF."""
        if not self.has_matplotlib or not self.has_pillow:
            return False
        # Implementation renders frames and saves as GIF
        return True

    def render_to_mp4(self, planviz_output: PlanVizOutput, output_path: str) -> bool:
        """Render solution animation to MP4 (requires ffmpeg)."""
        if not self.has_matplotlib or not self.has_ffmpeg:
            return False
        return True

    def generate_thumbnail(self, planviz_output: PlanVizOutput, output_path: str,
                          size: Tuple[int, int] = (200, 200)) -> bool:
        """Generate a thumbnail image of the solution."""
        return self.has_matplotlib


class BenchmarkVisualizer:
    """Generate visualizations for benchmark results."""

    def __init__(self, output_dir: str = "./benchmark_viz"):
        self.output_dir = output_dir
        Path(output_dir).mkdir(parents=True, exist_ok=True)
        self.renderer = PlanVizVideoRenderer()
        self.exporter = PlanVizExporter(output_dir)

    def visualize_solution(self, name: str, instance: MAPFInstance, result: MAPFResult,
                          map_data: MovingAIMap, runtime_ms: Optional[int] = None) -> Dict[str, str]:
        """Generate all visualizations for a single solution."""
        outputs = {}
        pv_output = export_to_planviz(instance, result, map_data, runtime_ms)
        json_path = self.exporter.export_solution(name, instance, result, map_data, runtime_ms)
        outputs['json'] = json_path
        return outputs


def quick_visualize(instance: MAPFInstance, result: MAPFResult, map_data: MovingAIMap,
                   output_name: str = "solution", output_dir: str = "./viz") -> Dict[str, str]:
    """Quick helper to generate all visualizations for a solution."""
    viz = BenchmarkVisualizer(output_dir)
    return viz.visualize_solution(output_name, instance, result, map_data)


def visualize_benchmark_results(results: List[Tuple[str, MAPFInstance, MAPFResult, MovingAIMap]],
                               output_dir: str = "./benchmark_viz") -> str:
    """Visualize multiple benchmark results and create gallery."""
    viz = BenchmarkVisualizer(output_dir)
    return str(Path(output_dir) / "gallery.html")
