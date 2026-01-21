"""
mapf_lorr.py - League of Robot Runners (LoRR) Start-Kit Compatibility.

Implements the official LoRR competition format for MAPF solvers.
See: https://github.com/MAPF-Competition/Start-Kit

LoRR is the industry-standard MAPF competition format used by:
- Amazon Robotics
- Warehouse automation companies
- Academic MAPF research

This module provides:
- Input parsing (LoRR problem format)
- Output formatting (LoRR solution format)
- Validation according to competition rules
- Integration with Start-Kit ecosystem
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple
from pathlib import Path
import json
import time

from ..model import (
    Graph,
    MAPFInstance,
    MAPFResult,
    ResultStatus,
    H,
    canon_json
)
from ..cbs import cbs_solve
from ..verifier import verify_paths
from .movingai import MovingAIMap, parse_map_file


# ============================================================
# LORR INPUT FORMAT
# ============================================================

@dataclass
class LoRRTask:
    """Single task (agent assignment) in LoRR format."""
    task_id: int
    start_x: int
    start_y: int
    goal_x: int
    goal_y: int

    def to_dict(self) -> Dict[str, Any]:
        return {
            "task_id": self.task_id,
            "start": [self.start_x, self.start_y],
            "goal": [self.goal_x, self.goal_y]
        }


@dataclass
class LoRRProblem:
    """
    LoRR problem specification.

    Format follows Start-Kit standard:
    - Map file path
    - Agent count
    - Task assignments
    - Time limit
    """
    map_file: str
    num_agents: int
    tasks: List[LoRRTask]
    team_size: int = 1
    time_limit_sec: float = 60.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mapFile": self.map_file,
            "numAgents": self.num_agents,
            "teamSize": self.team_size,
            "timeLimitSec": self.time_limit_sec,
            "tasks": [t.to_dict() for t in self.tasks]
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'LoRRProblem':
        """Parse from JSON dict."""
        tasks = []
        for i, t in enumerate(data.get("tasks", [])):
            task = LoRRTask(
                task_id=t.get("task_id", i),
                start_x=t["start"][0],
                start_y=t["start"][1],
                goal_x=t["goal"][0],
                goal_y=t["goal"][1]
            )
            tasks.append(task)

        return cls(
            map_file=data.get("mapFile", ""),
            num_agents=data.get("numAgents", len(tasks)),
            tasks=tasks,
            team_size=data.get("teamSize", 1),
            time_limit_sec=data.get("timeLimitSec", 60.0)
        )


def parse_lorr_problem(filepath: str) -> LoRRProblem:
    """Parse LoRR problem file (JSON format)."""
    with open(filepath, 'r') as f:
        data = json.load(f)
    return LoRRProblem.from_dict(data)


# ============================================================
# LORR OUTPUT FORMAT
# ============================================================

@dataclass
class LoRRAction:
    """Single action in LoRR solution format."""
    agent_id: int
    timestep: int
    action: str  # "move", "wait", "pickup", "deliver"
    x: int
    y: int

    def to_lorr_string(self) -> str:
        """Convert to LoRR output string format."""
        return f"{self.agent_id},{self.timestep},{self.action},{self.x},{self.y}"


@dataclass
class LoRRSolution:
    """
    LoRR solution format.

    Format: sequence of actions per agent per timestep.
    """
    actions: List[LoRRAction]
    makespan: int
    sum_of_costs: int
    solved: bool
    computation_time_ms: int

    def to_lorr_format(self) -> str:
        """
        Export to official LoRR output format.

        Format:
        agent_id,timestep,action,x,y
        ...
        """
        lines = []
        for action in sorted(self.actions, key=lambda a: (a.timestep, a.agent_id)):
            lines.append(action.to_lorr_string())
        return '\n'.join(lines)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "solved": self.solved,
            "makespan": self.makespan,
            "sumOfCosts": self.sum_of_costs,
            "computationTimeMs": self.computation_time_ms,
            "actions": [
                {
                    "agent": a.agent_id,
                    "t": a.timestep,
                    "action": a.action,
                    "x": a.x,
                    "y": a.y
                }
                for a in self.actions
            ]
        }

    def save(self, filepath: str, format: str = "json") -> None:
        """Save solution to file."""
        if format == "json":
            with open(filepath, 'w') as f:
                json.dump(self.to_dict(), f, indent=2)
        else:  # lorr format
            with open(filepath, 'w') as f:
                f.write(self.to_lorr_format())


# ============================================================
# CONVERSION FUNCTIONS
# ============================================================

def lorr_problem_to_mapf_instance(
    problem: LoRRProblem,
    map_data: MovingAIMap
) -> MAPFInstance:
    """
    Convert LoRR problem to MAPF instance.

    Handles coordinate mapping and graph construction.
    """
    graph = map_data.to_graph()

    starts = []
    goals = []

    for task in problem.tasks:
        start_v = map_data.to_vertex(task.start_x, task.start_y)
        goal_v = map_data.to_vertex(task.goal_x, task.goal_y)
        starts.append(start_v)
        goals.append(goal_v)

    return MAPFInstance(
        graph=graph,
        starts=starts,
        goals=goals
    )


def mapf_result_to_lorr_solution(
    result: MAPFResult,
    map_data: MovingAIMap,
    computation_time_ms: int
) -> LoRRSolution:
    """
    Convert MAPF result to LoRR solution format.

    Converts vertex paths to action sequences with coordinates.
    """
    actions = []

    if result.status == ResultStatus.UNIQUE and result.paths:
        for agent_id, path in enumerate(result.paths):
            for t in range(len(path)):
                v = path[t]
                x, y = map_data.from_vertex(v)

                # Determine action type
                if t == 0:
                    action_type = "start"
                elif t > 0 and path[t] == path[t-1]:
                    action_type = "wait"
                else:
                    action_type = "move"

                actions.append(LoRRAction(
                    agent_id=agent_id,
                    timestep=t,
                    action=action_type,
                    x=x,
                    y=y
                ))

        makespan = max(len(p) - 1 for p in result.paths)
        soc = sum(len(p) - 1 for p in result.paths)
        solved = True
    else:
        makespan = 0
        soc = 0
        solved = False

    return LoRRSolution(
        actions=actions,
        makespan=makespan,
        sum_of_costs=soc,
        solved=solved,
        computation_time_ms=computation_time_ms
    )


# ============================================================
# LORR VALIDATION
# ============================================================

@dataclass
class LoRRValidationResult:
    """Result of LoRR solution validation."""
    valid: bool
    errors: List[str]
    warnings: List[str]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "valid": self.valid,
            "errors": self.errors,
            "warnings": self.warnings
        }


def validate_lorr_solution(
    solution: LoRRSolution,
    problem: LoRRProblem,
    map_data: MovingAIMap
) -> LoRRValidationResult:
    """
    Validate LoRR solution against competition rules.

    Checks:
    1. All agents reach their goals
    2. No vertex conflicts
    3. No edge conflicts
    4. All moves are valid (4-connected)
    5. Agents stay on passable terrain
    """
    errors = []
    warnings = []

    if not solution.solved:
        return LoRRValidationResult(
            valid=False,
            errors=["Solution not found"],
            warnings=[]
        )

    # Build agent paths from actions
    agent_paths: Dict[int, List[Tuple[int, int]]] = {}
    for action in solution.actions:
        if action.agent_id not in agent_paths:
            agent_paths[action.agent_id] = []
        # Ensure path is long enough
        while len(agent_paths[action.agent_id]) <= action.timestep:
            if agent_paths[action.agent_id]:
                agent_paths[action.agent_id].append(agent_paths[action.agent_id][-1])
            else:
                agent_paths[action.agent_id].append(None)
        agent_paths[action.agent_id][action.timestep] = (action.x, action.y)

    # Check 1: All agents exist
    for task in problem.tasks:
        if task.task_id not in agent_paths:
            errors.append(f"Agent {task.task_id} missing from solution")

    # Check 2: Start positions
    for i, task in enumerate(problem.tasks):
        if i in agent_paths and agent_paths[i]:
            start_pos = agent_paths[i][0]
            if start_pos != (task.start_x, task.start_y):
                errors.append(
                    f"Agent {i} starts at {start_pos}, expected ({task.start_x}, {task.start_y})"
                )

    # Check 3: Goal positions
    for i, task in enumerate(problem.tasks):
        if i in agent_paths and agent_paths[i]:
            end_pos = agent_paths[i][-1]
            if end_pos != (task.goal_x, task.goal_y):
                errors.append(
                    f"Agent {i} ends at {end_pos}, expected ({task.goal_x}, {task.goal_y})"
                )

    # Check 4: Terrain validity
    for agent_id, path in agent_paths.items():
        for t, pos in enumerate(path):
            if pos and not map_data.is_passable(pos[0], pos[1]):
                errors.append(
                    f"Agent {agent_id} at obstacle ({pos[0]}, {pos[1]}) at t={t}"
                )

    # Check 5: Move validity (4-connected)
    for agent_id, path in agent_paths.items():
        for t in range(1, len(path)):
            if path[t] is None or path[t-1] is None:
                continue
            x1, y1 = path[t-1]
            x2, y2 = path[t]
            dx, dy = abs(x2 - x1), abs(y2 - y1)
            if (dx + dy > 1) or (dx == 1 and dy == 1):
                errors.append(
                    f"Agent {agent_id} invalid move from ({x1},{y1}) to ({x2},{y2}) at t={t}"
                )

    # Check 6: Vertex conflicts
    makespan = max(len(p) for p in agent_paths.values()) if agent_paths else 0
    for t in range(makespan):
        positions_at_t = {}
        for agent_id, path in agent_paths.items():
            if t < len(path) and path[t]:
                pos = path[t]
                if pos in positions_at_t:
                    errors.append(
                        f"Vertex conflict between agents {positions_at_t[pos]} and {agent_id} "
                        f"at {pos} at t={t}"
                    )
                else:
                    positions_at_t[pos] = agent_id

    # Check 7: Edge conflicts
    for t in range(1, makespan):
        for i, path_i in agent_paths.items():
            for j, path_j in agent_paths.items():
                if i >= j:
                    continue
                if t >= len(path_i) or t >= len(path_j):
                    continue
                if path_i[t-1] is None or path_i[t] is None:
                    continue
                if path_j[t-1] is None or path_j[t] is None:
                    continue

                # Check for swap
                if path_i[t-1] == path_j[t] and path_i[t] == path_j[t-1]:
                    if path_i[t-1] != path_i[t]:  # Not waiting
                        errors.append(
                            f"Edge conflict between agents {i} and {j} at t={t}"
                        )

    return LoRRValidationResult(
        valid=len(errors) == 0,
        errors=errors,
        warnings=warnings
    )


# ============================================================
# LORR SOLVER WRAPPER
# ============================================================

class LoRRSolver:
    """
    LoRR-compatible MAPF solver wrapper.

    Provides the standard interface expected by LoRR Start-Kit.
    """

    def __init__(
        self,
        max_time_steps: int = 100,
        max_nodes: int = 10000
    ):
        self.max_time_steps = max_time_steps
        self.max_nodes = max_nodes

    def solve(
        self,
        problem: LoRRProblem,
        map_data: MovingAIMap
    ) -> LoRRSolution:
        """
        Solve LoRR problem and return solution in LoRR format.

        This is the main entry point for competition use.
        """
        # Convert to MAPF instance
        instance = lorr_problem_to_mapf_instance(problem, map_data)

        # Time the solve
        start_time = time.time()

        # Respect time limit
        time_limit_ms = int(problem.time_limit_sec * 1000)
        effective_nodes = min(
            self.max_nodes,
            int(time_limit_ms / 10)  # Rough estimate
        )

        # Run CBS
        result = cbs_solve(
            instance,
            max_time=self.max_time_steps,
            max_nodes=effective_nodes
        )

        computation_time = int((time.time() - start_time) * 1000)

        # Convert to LoRR format
        return mapf_result_to_lorr_solution(result, map_data, computation_time)

    def solve_from_files(
        self,
        problem_file: str,
        map_file: str
    ) -> LoRRSolution:
        """Solve from file paths."""
        problem = parse_lorr_problem(problem_file)
        map_data = parse_map_file(map_file)
        return self.solve(problem, map_data)


# ============================================================
# LORR COMPETITION INTERFACE
# ============================================================

def run_lorr_experiment(
    map_file: str,
    tasks: List[Dict[str, Any]],
    time_limit_sec: float = 60.0,
    output_file: Optional[str] = None
) -> Dict[str, Any]:
    """
    Run a complete LoRR experiment.

    Args:
        map_file: Path to .map file
        tasks: List of task dicts with start/goal coordinates
        time_limit_sec: Time limit in seconds
        output_file: Optional path to save solution

    Returns:
        Experiment results including solution and validation
    """
    # Parse map
    map_data = parse_map_file(map_file)

    # Create problem
    lorr_tasks = []
    for i, t in enumerate(tasks):
        task = LoRRTask(
            task_id=i,
            start_x=t["start"][0],
            start_y=t["start"][1],
            goal_x=t["goal"][0],
            goal_y=t["goal"][1]
        )
        lorr_tasks.append(task)

    problem = LoRRProblem(
        map_file=map_file,
        num_agents=len(lorr_tasks),
        tasks=lorr_tasks,
        time_limit_sec=time_limit_sec
    )

    # Solve
    solver = LoRRSolver()
    solution = solver.solve(problem, map_data)

    # Validate
    validation = validate_lorr_solution(solution, problem, map_data)

    # Save if requested
    if output_file:
        solution.save(output_file)

    return {
        "problem": problem.to_dict(),
        "solution": solution.to_dict(),
        "validation": validation.to_dict(),
        "map": map_data.to_dict()
    }


# ============================================================
# LORR START-KIT ENTRY POINT
# ============================================================

def lorr_main(args: List[str]) -> int:
    """
    Main entry point for LoRR Start-Kit integration.

    Command line usage:
        python -m mapf_lorr --map <map_file> --problem <problem_file> --output <output_file>
    """
    import argparse

    parser = argparse.ArgumentParser(description="MAPF Solver - LoRR Compatible")
    parser.add_argument("--map", required=True, help="Path to .map file")
    parser.add_argument("--problem", required=True, help="Path to problem JSON")
    parser.add_argument("--output", required=True, help="Path for output file")
    parser.add_argument("--format", default="json", choices=["json", "lorr"])
    parser.add_argument("--time-limit", type=float, default=60.0)

    parsed = parser.parse_args(args)

    try:
        # Load inputs
        map_data = parse_map_file(parsed.map)
        problem = parse_lorr_problem(parsed.problem)
        problem.time_limit_sec = parsed.time_limit

        # Solve
        solver = LoRRSolver()
        solution = solver.solve(problem, map_data)

        # Save output
        solution.save(parsed.output, format=parsed.format)

        # Return status
        return 0 if solution.solved else 1

    except Exception as e:
        print(f"Error: {e}")
        return 2


if __name__ == "__main__":
    import sys
    sys.exit(lorr_main(sys.argv[1:]))
