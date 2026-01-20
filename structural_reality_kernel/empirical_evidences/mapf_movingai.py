"""
mapf_movingai.py - MovingAI Benchmark Loader for MAPF.

Implements parsing and loading of the standard MovingAI MAPF benchmark format:
- .map files: Grid maps with obstacles
- .scen files: Scenario files with agent start/goal positions

MovingAI is the standard benchmark suite for MAPF research.
See: https://movingai.com/benchmarks/mapf.html

Format specifications:
- Map format: https://movingai.com/benchmarks/formats.html
- Scenario format: version 1 with bucket, map, width, height,
  start_x, start_y, goal_x, goal_y, optimal_length
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple
from pathlib import Path
import os

from .mapf_model import (
    Graph,
    MAPFInstance,
    MAPFResult,
    ResultStatus,
    H,
    canon_json
)


# ============================================================
# MAP FILE PARSER
# ============================================================

@dataclass
class MovingAIMap:
    """
    Parsed MovingAI map file.

    Map format:
    - Header: type, height, width
    - Grid: '.' or 'G' = passable, '@' or 'O' or 'T' or 'W' = obstacle
    """
    name: str
    map_type: str
    height: int
    width: int
    grid: List[List[str]]  # grid[y][x]

    def is_passable(self, x: int, y: int) -> bool:
        """Check if cell (x, y) is passable."""
        if 0 <= x < self.width and 0 <= y < self.height:
            cell = self.grid[y][x]
            return cell in ['.', 'G', 'S']  # Passable terrain types
        return False

    def to_vertex(self, x: int, y: int) -> int:
        """Convert (x, y) to vertex ID."""
        return y * self.width + x

    def from_vertex(self, v: int) -> Tuple[int, int]:
        """Convert vertex ID to (x, y)."""
        return v % self.width, v // self.width

    def to_graph(self) -> Graph:
        """
        Convert map to MAPF Graph.

        Vertices are passable cells, numbered row by row.
        Edges connect 4-adjacent passable cells.
        """
        # Build vertex list (only passable cells)
        vertices = []
        passable_coords = set()

        for y in range(self.height):
            for x in range(self.width):
                if self.is_passable(x, y):
                    v = self.to_vertex(x, y)
                    vertices.append(v)
                    passable_coords.add((x, y))

        # Build edges (4-connectivity)
        edges = set()
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # N, S, E, W

        for (x, y) in passable_coords:
            v = self.to_vertex(x, y)
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if (nx, ny) in passable_coords:
                    nv = self.to_vertex(nx, ny)
                    edges.add((v, nv))

        return Graph(vertices=sorted(vertices), edges=edges)

    def num_passable(self) -> int:
        """Count passable cells."""
        count = 0
        for y in range(self.height):
            for x in range(self.width):
                if self.is_passable(x, y):
                    count += 1
        return count

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "type": self.map_type,
            "height": self.height,
            "width": self.width,
            "passable_cells": self.num_passable(),
            "total_cells": self.height * self.width
        }


def parse_map_file(filepath: str) -> MovingAIMap:
    """
    Parse a MovingAI .map file.

    Format:
    type octile
    height H
    width W
    map
    <H lines of W characters>
    """
    with open(filepath, 'r') as f:
        lines = f.readlines()

    # Parse header
    map_type = "octile"
    height = 0
    width = 0
    grid_start = 0

    for i, line in enumerate(lines):
        line = line.strip()
        if line.startswith("type"):
            map_type = line.split()[1] if len(line.split()) > 1 else "octile"
        elif line.startswith("height"):
            height = int(line.split()[1])
        elif line.startswith("width"):
            width = int(line.split()[1])
        elif line == "map":
            grid_start = i + 1
            break

    # Parse grid
    grid = []
    for i in range(grid_start, grid_start + height):
        if i < len(lines):
            row = list(lines[i].rstrip('\n'))
            # Pad or truncate to width
            if len(row) < width:
                row.extend(['.'] * (width - len(row)))
            elif len(row) > width:
                row = row[:width]
            grid.append(row)

    # Ensure we have exactly height rows
    while len(grid) < height:
        grid.append(['.'] * width)

    name = Path(filepath).stem

    return MovingAIMap(
        name=name,
        map_type=map_type,
        height=height,
        width=width,
        grid=grid
    )


# ============================================================
# SCENARIO FILE PARSER
# ============================================================

@dataclass
class ScenarioAgent:
    """Single agent in a scenario."""
    bucket: int
    map_name: str
    map_width: int
    map_height: int
    start_x: int
    start_y: int
    goal_x: int
    goal_y: int
    optimal_length: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "start": (self.start_x, self.start_y),
            "goal": (self.goal_x, self.goal_y),
            "optimal_length": self.optimal_length
        }


@dataclass
class MovingAIScenario:
    """
    Parsed MovingAI scenario file.

    Scenario format (version 1):
    version 1
    bucket map_name map_width map_height start_x start_y goal_x goal_y optimal_length
    ...
    """
    name: str
    version: int
    agents: List[ScenarioAgent]

    def num_agents(self) -> int:
        return len(self.agents)

    def get_agents(self, count: int) -> List[ScenarioAgent]:
        """Get first 'count' agents from scenario."""
        return self.agents[:count]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "version": self.version,
            "num_agents": self.num_agents(),
            "agents": [a.to_dict() for a in self.agents]
        }


def parse_scenario_file(filepath: str) -> MovingAIScenario:
    """
    Parse a MovingAI .scen file.

    Format:
    version 1
    bucket map_name map_width map_height start_x start_y goal_x goal_y optimal_length
    """
    with open(filepath, 'r') as f:
        lines = f.readlines()

    version = 1
    agents = []

    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line.startswith("version"):
            version = int(line.split()[1])
            continue

        # Parse agent line
        parts = line.split()
        if len(parts) >= 9:
            try:
                agent = ScenarioAgent(
                    bucket=int(parts[0]),
                    map_name=parts[1],
                    map_width=int(parts[2]),
                    map_height=int(parts[3]),
                    start_x=int(parts[4]),
                    start_y=int(parts[5]),
                    goal_x=int(parts[6]),
                    goal_y=int(parts[7]),
                    optimal_length=float(parts[8])
                )
                agents.append(agent)
            except (ValueError, IndexError):
                continue  # Skip malformed lines

    name = Path(filepath).stem

    return MovingAIScenario(
        name=name,
        version=version,
        agents=agents
    )


# ============================================================
# MAPF INSTANCE CREATION
# ============================================================

def create_instance_from_movingai(
    map_data: MovingAIMap,
    scenario: MovingAIScenario,
    num_agents: int
) -> MAPFInstance:
    """
    Create MAPF instance from MovingAI map and scenario.

    Args:
        map_data: Parsed map file
        scenario: Parsed scenario file
        num_agents: Number of agents to include

    Returns:
        MAPFInstance ready for solving
    """
    # Convert map to graph
    graph = map_data.to_graph()

    # Get agents from scenario
    agents = scenario.get_agents(num_agents)

    # Convert agent positions to vertex IDs
    starts = []
    goals = []

    for agent in agents:
        start_v = map_data.to_vertex(agent.start_x, agent.start_y)
        goal_v = map_data.to_vertex(agent.goal_x, agent.goal_y)
        starts.append(start_v)
        goals.append(goal_v)

    return MAPFInstance(
        graph=graph,
        starts=starts,
        goals=goals
    )


def load_benchmark(
    map_path: str,
    scenario_path: str,
    num_agents: int
) -> Tuple[MAPFInstance, MovingAIMap, MovingAIScenario]:
    """
    Load a complete MovingAI benchmark.

    Args:
        map_path: Path to .map file
        scenario_path: Path to .scen file
        num_agents: Number of agents to include

    Returns:
        (instance, map_data, scenario)
    """
    map_data = parse_map_file(map_path)
    scenario = parse_scenario_file(scenario_path)
    instance = create_instance_from_movingai(map_data, scenario, num_agents)

    return instance, map_data, scenario


# ============================================================
# BENCHMARK SET MANAGEMENT
# ============================================================

@dataclass
class BenchmarkSet:
    """Collection of related benchmarks."""
    name: str
    map_files: List[str]
    scenario_files: List[str]
    description: str = ""

    def get_instances(
        self,
        num_agents: int,
        max_instances: Optional[int] = None
    ) -> List[Tuple[MAPFInstance, str]]:
        """
        Generate instances from this benchmark set.

        Returns list of (instance, instance_name) tuples.
        """
        instances = []

        for map_path in self.map_files:
            map_name = Path(map_path).stem

            # Find matching scenario files
            for scen_path in self.scenario_files:
                scen_name = Path(scen_path).stem
                if map_name in scen_name or scen_name.startswith(map_name):
                    try:
                        instance, _, _ = load_benchmark(
                            map_path, scen_path, num_agents
                        )
                        instance_name = f"{map_name}_{num_agents}agents"
                        instances.append((instance, instance_name))

                        if max_instances and len(instances) >= max_instances:
                            return instances
                    except Exception:
                        continue  # Skip failed loads

        return instances


def discover_benchmarks(directory: str) -> BenchmarkSet:
    """
    Discover benchmark files in a directory.

    Scans for .map and .scen files.
    """
    dir_path = Path(directory)

    map_files = sorted([str(f) for f in dir_path.glob("*.map")])
    scenario_files = sorted([str(f) for f in dir_path.glob("*.scen")])

    return BenchmarkSet(
        name=dir_path.name,
        map_files=map_files,
        scenario_files=scenario_files,
        description=f"Discovered from {directory}"
    )


# ============================================================
# STANDARD BENCHMARK CATEGORIES
# ============================================================

# Standard MovingAI benchmark categories
BENCHMARK_CATEGORIES = {
    "empty": "Empty grids of various sizes",
    "random": "Random obstacle placement (10-40%)",
    "maze": "Maze-like structures",
    "room": "Room-based warehouse layouts",
    "warehouse": "Actual warehouse maps",
    "city": "City street grid maps",
    "game": "Game map benchmarks"
}


def get_standard_test_maps() -> List[Dict[str, Any]]:
    """
    Get standard test map configurations for quick testing.

    These are small maps that can be created programmatically.
    """
    return [
        {
            "name": "empty-8-8",
            "type": "empty",
            "width": 8,
            "height": 8,
            "obstacles": []
        },
        {
            "name": "empty-16-16",
            "type": "empty",
            "width": 16,
            "height": 16,
            "obstacles": []
        },
        {
            "name": "random-8-8-10",
            "type": "random",
            "width": 8,
            "height": 8,
            "obstacle_density": 0.1
        },
        {
            "name": "room-32-32-4",
            "type": "room",
            "width": 32,
            "height": 32,
            "room_size": 4
        }
    ]


def create_test_map(config: Dict[str, Any]) -> MovingAIMap:
    """
    Create a test map from configuration.

    Supports: empty, random, room types.
    """
    width = config.get("width", 8)
    height = config.get("height", 8)
    map_type = config.get("type", "empty")
    name = config.get("name", f"{map_type}-{width}-{height}")

    # Initialize empty grid
    grid = [['.' for _ in range(width)] for _ in range(height)]

    if map_type == "random":
        import random
        density = config.get("obstacle_density", 0.1)
        for y in range(height):
            for x in range(width):
                if random.random() < density:
                    grid[y][x] = '@'

    elif map_type == "room":
        room_size = config.get("room_size", 4)
        # Create room walls
        for y in range(height):
            for x in range(width):
                # Wall at room boundaries (with doors)
                if (x % room_size == 0 or y % room_size == 0):
                    # Leave doors
                    door_pos = room_size // 2
                    if not ((x % room_size == 0 and y % room_size == door_pos) or
                            (y % room_size == 0 and x % room_size == door_pos)):
                        grid[y][x] = '@'

    return MovingAIMap(
        name=name,
        map_type=map_type,
        height=height,
        width=width,
        grid=grid
    )


def create_test_scenario(
    map_data: MovingAIMap,
    num_agents: int,
    seed: int = 42
) -> MovingAIScenario:
    """
    Create a test scenario with random agent placements.

    Ensures starts and goals are on passable cells and unique.
    """
    import random
    random.seed(seed)

    # Get all passable cells
    passable = []
    for y in range(map_data.height):
        for x in range(map_data.width):
            if map_data.is_passable(x, y):
                passable.append((x, y))

    if len(passable) < num_agents * 2:
        raise ValueError(f"Not enough passable cells for {num_agents} agents")

    # Randomly select starts and goals
    random.shuffle(passable)
    agents = []

    for i in range(num_agents):
        start = passable[i * 2]
        goal = passable[i * 2 + 1]

        agent = ScenarioAgent(
            bucket=0,
            map_name=map_data.name,
            map_width=map_data.width,
            map_height=map_data.height,
            start_x=start[0],
            start_y=start[1],
            goal_x=goal[0],
            goal_y=goal[1],
            optimal_length=0.0  # Unknown
        )
        agents.append(agent)

    return MovingAIScenario(
        name=f"{map_data.name}-{num_agents}agents",
        version=1,
        agents=agents
    )


# ============================================================
# BENCHMARK LOADER CLASS
# ============================================================

class MovingAIBenchmarkLoader:
    """
    Complete MovingAI benchmark loader.

    Supports:
    - Loading from files
    - Creating test instances
    - Managing benchmark sets
    """

    def __init__(self, benchmark_dir: Optional[str] = None):
        self.benchmark_dir = benchmark_dir
        self.loaded_maps: Dict[str, MovingAIMap] = {}
        self.loaded_scenarios: Dict[str, MovingAIScenario] = {}

    def load_map(self, filepath: str) -> MovingAIMap:
        """Load and cache a map file."""
        if filepath not in self.loaded_maps:
            self.loaded_maps[filepath] = parse_map_file(filepath)
        return self.loaded_maps[filepath]

    def load_scenario(self, filepath: str) -> MovingAIScenario:
        """Load and cache a scenario file."""
        if filepath not in self.loaded_scenarios:
            self.loaded_scenarios[filepath] = parse_scenario_file(filepath)
        return self.loaded_scenarios[filepath]

    def create_instance(
        self,
        map_path: str,
        scenario_path: str,
        num_agents: int
    ) -> MAPFInstance:
        """Create MAPF instance from files."""
        map_data = self.load_map(map_path)
        scenario = self.load_scenario(scenario_path)
        return create_instance_from_movingai(map_data, scenario, num_agents)

    def create_test_instance(
        self,
        map_config: Dict[str, Any],
        num_agents: int,
        seed: int = 42
    ) -> Tuple[MAPFInstance, MovingAIMap, MovingAIScenario]:
        """Create a test instance from configuration."""
        map_data = create_test_map(map_config)
        scenario = create_test_scenario(map_data, num_agents, seed)
        instance = create_instance_from_movingai(map_data, scenario, num_agents)
        return instance, map_data, scenario

    def get_standard_instances(
        self,
        num_agents: int = 4,
        map_types: Optional[List[str]] = None
    ) -> List[Tuple[MAPFInstance, str, MovingAIMap]]:
        """
        Get standard test instances.

        Returns: List of (instance, name, map_data) tuples
        """
        configs = get_standard_test_maps()

        if map_types:
            configs = [c for c in configs if c["type"] in map_types]

        instances = []
        for config in configs:
            try:
                instance, map_data, scenario = self.create_test_instance(
                    config, num_agents
                )
                instances.append((instance, config["name"], map_data))
            except ValueError:
                continue  # Skip if not enough cells

        return instances


# ============================================================
# RESULT EXPORT (for external tools)
# ============================================================

def export_solution_movingai(
    result: MAPFResult,
    map_data: MovingAIMap,
    output_path: str
) -> None:
    """
    Export solution in MovingAI-compatible format.

    Format: agent_id, path as (x,y) coordinates
    """
    if result.status != ResultStatus.UNIQUE or result.paths is None:
        raise ValueError("Can only export UNIQUE solutions")

    lines = []
    for i, path in enumerate(result.paths):
        coords = []
        for v in path:
            x, y = map_data.from_vertex(v)
            coords.append(f"({x},{y})")
        lines.append(f"{i}: {' '.join(coords)}")

    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))


def solution_to_coordinates(
    result: MAPFResult,
    map_data: MovingAIMap
) -> List[List[Tuple[int, int]]]:
    """
    Convert solution paths to (x, y) coordinates.

    Returns: List of paths, each path is list of (x, y) tuples
    """
    if result.paths is None:
        return []

    coord_paths = []
    for path in result.paths:
        coords = [map_data.from_vertex(v) for v in path]
        coord_paths.append(coords)

    return coord_paths
