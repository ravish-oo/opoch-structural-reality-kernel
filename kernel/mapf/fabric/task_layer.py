"""
Task Layer for Transport Fabric

Manages task generation, assignment, and completion tracking.

At 10k scale, "robots" are tokens and "tasks" are destination-class
routing problems. The task layer:
1. Generates tasks (picks, putaways, charges)
2. Assigns destinations to tokens
3. Tracks completion and throughput
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple, Optional, FrozenSet
from enum import Enum
from collections import defaultdict
import random
import hashlib
import json


class TaskType(Enum):
    """Type of task."""
    PICK = "PICK"           # Go to pick station
    PUTAWAY = "PUTAWAY"     # Go to storage location
    CHARGE = "CHARGE"       # Go to charger
    DOCK = "DOCK"           # Go to dock/exit


class TaskStatus(Enum):
    """Status of a task."""
    PENDING = "PENDING"       # Not yet assigned
    ASSIGNED = "ASSIGNED"     # Assigned to robot
    IN_PROGRESS = "IN_PROGRESS"  # Robot moving to destination
    COMPLETED = "COMPLETED"   # Robot reached destination
    CANCELLED = "CANCELLED"   # Task cancelled


@dataclass
class Task:
    """
    A single task (destination assignment).

    In the fabric model, a task is simply:
    - A destination vertex (or destination class)
    - A priority
    - Status tracking
    """
    task_id: int
    task_type: TaskType
    destination: int          # Target vertex
    destination_class: int    # Class ID for routing
    priority: int = 0         # Higher = more urgent
    status: TaskStatus = TaskStatus.PENDING
    assigned_robot: Optional[int] = None
    created_tick: int = 0
    completed_tick: Optional[int] = None

    def to_dict(self) -> Dict:
        return {
            "id": self.task_id,
            "type": self.task_type.value,
            "destination": self.destination,
            "priority": self.priority,
            "status": self.status.value,
            "robot": self.assigned_robot,
            "created": self.created_tick,
            "completed": self.completed_tick
        }


@dataclass
class DestinationClass:
    """
    A class of destinations (e.g., all pick stations, all chargers).

    Routing decisions are made per-class, not per-destination.
    This keeps the routing problem tractable at scale.
    """
    class_id: int
    name: str
    vertices: FrozenSet[int]
    task_type: TaskType

    def contains(self, v: int) -> bool:
        return v in self.vertices

    def size(self) -> int:
        return len(self.vertices)


class TaskManager:
    """
    Manages tasks for the transport fabric.

    Responsibilities:
    1. Task generation (from external system or simulation)
    2. Assignment to robots
    3. Completion detection
    4. Throughput tracking
    """

    def __init__(self, destination_classes: List[DestinationClass]):
        """
        Initialize task manager.

        Args:
            destination_classes: List of destination classes
        """
        self.destination_classes = {dc.class_id: dc for dc in destination_classes}
        self.tasks: Dict[int, Task] = {}
        self.next_task_id = 0

        # Robot assignments
        self.robot_tasks: Dict[int, int] = {}  # robot_id → task_id
        self.robot_destinations: Dict[int, int] = {}  # robot_id → dest_class

        # Statistics
        self.completed_count = 0
        self.total_latency = 0

    def create_task(self, task_type: TaskType,
                    destination_class: int,
                    destination: Optional[int] = None,
                    priority: int = 0,
                    current_tick: int = 0) -> Task:
        """
        Create a new task.

        Args:
            task_type: Type of task
            destination_class: Class ID for routing
            destination: Specific destination (or random from class)
            priority: Task priority
            current_tick: Current simulation tick

        Returns:
            Created task
        """
        dc = self.destination_classes.get(destination_class)
        if dc is None:
            raise ValueError(f"Unknown destination class: {destination_class}")

        if destination is None:
            # Pick random destination from class
            destination = random.choice(list(dc.vertices))

        task = Task(
            task_id=self.next_task_id,
            task_type=task_type,
            destination=destination,
            destination_class=destination_class,
            priority=priority,
            created_tick=current_tick
        )
        self.tasks[task.task_id] = task
        self.next_task_id += 1

        return task

    def assign_task(self, task_id: int, robot_id: int):
        """
        Assign a task to a robot.

        Args:
            task_id: Task to assign
            robot_id: Robot to assign to
        """
        if task_id not in self.tasks:
            raise ValueError(f"Unknown task: {task_id}")

        task = self.tasks[task_id]
        task.status = TaskStatus.ASSIGNED
        task.assigned_robot = robot_id

        self.robot_tasks[robot_id] = task_id
        self.robot_destinations[robot_id] = task.destination_class

    def complete_task(self, task_id: int, current_tick: int):
        """
        Mark task as completed.

        Args:
            task_id: Task to complete
            current_tick: Current simulation tick
        """
        if task_id not in self.tasks:
            return

        task = self.tasks[task_id]
        task.status = TaskStatus.COMPLETED
        task.completed_tick = current_tick

        # Update statistics
        self.completed_count += 1
        latency = current_tick - task.created_tick
        self.total_latency += latency

        # Clear robot assignment
        if task.assigned_robot is not None:
            self.robot_tasks.pop(task.assigned_robot, None)
            self.robot_destinations.pop(task.assigned_robot, None)

    def check_completions(self, robot_positions: Dict[int, int],
                          current_tick: int) -> List[int]:
        """
        Check which tasks have been completed (robot reached destination).

        Args:
            robot_positions: {robot_id: current_vertex}
            current_tick: Current tick

        Returns:
            List of completed task IDs
        """
        completed = []

        for robot_id, task_id in list(self.robot_tasks.items()):
            task = self.tasks.get(task_id)
            if task is None:
                continue

            current_pos = robot_positions.get(robot_id)
            if current_pos == task.destination:
                self.complete_task(task_id, current_tick)
                completed.append(task_id)

        return completed

    def get_pending_tasks(self, task_type: Optional[TaskType] = None) -> List[Task]:
        """Get all pending (unassigned) tasks."""
        pending = [t for t in self.tasks.values() if t.status == TaskStatus.PENDING]
        if task_type:
            pending = [t for t in pending if t.task_type == task_type]
        return sorted(pending, key=lambda t: -t.priority)

    def get_robot_destination(self, robot_id: int) -> Optional[int]:
        """Get destination class for a robot."""
        return self.robot_destinations.get(robot_id)

    def throughput(self, time_window: int = 100) -> float:
        """
        Calculate throughput (tasks completed per tick).

        Args:
            time_window: Number of ticks to average over

        Returns:
            Tasks per tick
        """
        if time_window == 0:
            return 0.0
        return self.completed_count / time_window

    def average_latency(self) -> float:
        """Average task completion latency."""
        if self.completed_count == 0:
            return 0.0
        return self.total_latency / self.completed_count

    def statistics(self) -> Dict:
        """Get task statistics."""
        by_status = defaultdict(int)
        by_type = defaultdict(int)

        for task in self.tasks.values():
            by_status[task.status.value] += 1
            by_type[task.task_type.value] += 1

        return {
            "total_tasks": len(self.tasks),
            "completed": self.completed_count,
            "pending": by_status.get("PENDING", 0),
            "in_progress": by_status.get("IN_PROGRESS", 0) + by_status.get("ASSIGNED", 0),
            "average_latency": self.average_latency(),
            "by_type": dict(by_type)
        }


class TaskGenerator:
    """
    Generates tasks for simulation/testing.

    Models realistic warehouse task patterns:
    - Pick waves (batched)
    - Putaway streams (continuous)
    - Charging needs (periodic)
    """

    def __init__(self, task_manager: TaskManager,
                 pick_rate: float = 0.1,
                 putaway_rate: float = 0.05,
                 charge_rate: float = 0.01):
        """
        Initialize task generator.

        Args:
            task_manager: Task manager to create tasks in
            pick_rate: Picks per robot per tick
            putaway_rate: Putaways per robot per tick
            charge_rate: Charge tasks per robot per tick
        """
        self.task_manager = task_manager
        self.pick_rate = pick_rate
        self.putaway_rate = putaway_rate
        self.charge_rate = charge_rate

    def generate_tick_tasks(self, num_robots: int,
                            current_tick: int,
                            pick_class: int = 0,
                            storage_class: int = 1,
                            charge_class: int = 2) -> List[Task]:
        """
        Generate tasks for a single tick.

        Args:
            num_robots: Number of robots (scales task generation)
            current_tick: Current tick number
            pick_class: Destination class for picks
            storage_class: Destination class for putaways
            charge_class: Destination class for charging

        Returns:
            List of generated tasks
        """
        tasks = []

        # Generate picks (Poisson-ish)
        num_picks = int(num_robots * self.pick_rate)
        for _ in range(num_picks):
            if pick_class in self.task_manager.destination_classes:
                task = self.task_manager.create_task(
                    TaskType.PICK, pick_class,
                    priority=1, current_tick=current_tick
                )
                tasks.append(task)

        # Generate putaways
        num_putaways = int(num_robots * self.putaway_rate)
        for _ in range(num_putaways):
            if storage_class in self.task_manager.destination_classes:
                task = self.task_manager.create_task(
                    TaskType.PUTAWAY, storage_class,
                    priority=0, current_tick=current_tick
                )
                tasks.append(task)

        # Generate charge tasks (less frequent, higher priority when needed)
        if random.random() < self.charge_rate * num_robots:
            if charge_class in self.task_manager.destination_classes:
                task = self.task_manager.create_task(
                    TaskType.CHARGE, charge_class,
                    priority=2, current_tick=current_tick
                )
                tasks.append(task)

        return tasks


class SimpleTaskAssigner:
    """
    Simple task assignment: assign pending tasks to idle robots.

    In production, would use more sophisticated assignment
    (Hungarian algorithm, etc.).
    """

    def __init__(self, task_manager: TaskManager):
        self.task_manager = task_manager

    def assign_tasks(self, idle_robots: Set[int]) -> Dict[int, int]:
        """
        Assign pending tasks to idle robots.

        Args:
            idle_robots: Set of robot IDs without tasks

        Returns:
            {robot_id: task_id} new assignments
        """
        assignments = {}
        pending = self.task_manager.get_pending_tasks()

        for robot_id in sorted(idle_robots):
            if not pending:
                break

            task = pending.pop(0)
            self.task_manager.assign_task(task.task_id, robot_id)
            assignments[robot_id] = task.task_id

        return assignments
